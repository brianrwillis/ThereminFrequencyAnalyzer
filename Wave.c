/********************************************************************

 * Wave.c - A module that generates a sine wave or a triangle wave,
 *      with a frequency range of 10 Hz to 10 kHz, and and amplitude
 *      range of 0 - 1.0 Vpp, based on the user input.
 *
 *  02/31/2018 Maria Watters
 *********************************************************************
* Header Files - Dependencies
********************************************************************/
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "K65TWR_GPIO.h"
#include "DMA.h"
#include "Wave.h"
#include "DMA.h"

#define SINE 1          /* WaveStruct.type value for sine wave */
#define TRIANGLE 2      /* WaveStruct.type value for triangle wave */
#define DC_OFFSET 680  /* DC offset of 0.6 V - FOR A 1.6 VREF*/
#define AC_MAX 570     /* AC offset of 0.5 V - FOR A 1.6 VREF*/
#define MAX_STEP 20     /* Maximum step size for amplitude */
#define MIN_STEP 0      /* Minimum step size for amplitude */
#define FS 48000        /* Sample rate of PIT and sinewave */
#define SINE_MAX_BIT_SHIFT 10      /* The largest right shift needed to prevent rollover */
#define SINE_CONVERT_BIT_SHIFT 21  /* Bit shift required to convert from q31_t to INT16U */
#define HALF_WAVE 1073527076          /* Q31 value for the first half of a wave period */
#define FULL_WAVE 2147268899          /* Q31 value for the second half of a wave period */
#define TOTAL_BUFFER_LAYERS 2      /* The amount of layers in the ping-pong buffer */
#define BUFFER_SIZE 60             /* The size of the ping-pong buffer */
/*****************************************************************************************
* Allocate task control blocks
*****************************************************************************************/
static OS_TCB WaveTaskTCB;

/*****************************************************************************************
* Allocate task stack space
*****************************************************************************************/
static CPU_STK WaveTaskStk[APP_CFG_WAVE_TASK_STK_SIZE];

/*********************************************************************
* MicroC/OS Resources
********************************************************************/
static OS_MUTEX WaveStructMutexKey;
static OS_SEM WaveChgFlag;
/*********************************************************************
* Private Resources
********************************************************************/
static WAVE_T WaveStruct;
static INT16U WaveOut[TOTAL_BUFFER_LAYERS][BUFFER_SIZE];
typedef enum{POS_WAVE, NEG_WAVE} TRI_T;

/******************************************************************************
 * Private Task Function Prototypes
 ******************************************************************************/
static void WaveTask(void *p_arg);

/******************************************************************************
 * WaveInit() - Creates WaveTask, the Wave Struck Mutex, and the Wave Change Flag.
 *
 *  02/13/2018 Maria Watters
 ******************************************************************************/
void WaveInit(void){
    OS_ERR os_err;

    WaveStruct.amp = 20u;

    /* Task creation for the Wave Task */
    OSTaskCreate(&WaveTaskTCB,
        "Wave Task ",
        WaveTask,
        (void *)0,
        APP_CFG_WAVE_TASK_PRIO,
        &WaveTaskStk[0],
        (APP_CFG_WAVE_TASK_STK_SIZE / 10u),
        APP_CFG_WAVE_TASK_STK_SIZE,
        0,
        0,
        (void *)0,
        (OS_OPT_TASK_NONE),
        &os_err);

    /* Error Trap */
    while(os_err != OS_ERR_NONE){}

    OSMutexCreate(&WaveStructMutexKey, "Wave Struct Mutex ", &os_err);
    while(os_err != OS_ERR_NONE){}

    OSSemCreate(&WaveChgFlag, "Wave Change Flag Semaphore", 0, &os_err);
    while(os_err != OS_ERR_NONE){  }

    (void)DMAInit(&WaveOut[0][0]);
}

/******************************************************************************
 * WaveTask() - Generates the waveform to be output by the DMA/DAC. The waveform
 *  generated depends on the inputed frequency (10 Hz - 10 kHz), amplitude
 *  (0 - 1 Vpp), and waveform type (triangle or sine).
 *
 *  2/15/2018 Maria Watters, Daniel Dodge, Daniel Wilson
 ******************************************************************************/
static void WaveTask(void *p_arg){
    OS_ERR os_err;
    (void)p_arg;
    WAVE_T CurrentStruct;
    TRI_T Wave_State;
    INT16U volume;
    INT8U changeState = 1;
    INT8U wave_index;
    INT32S ac_component;
    INT8U buffer_layer;
    q31_t radians = 0;
    INT32S sin_val;
    INT32S tri_val;


    while(1){

        DB4_TURN_OFF();
        /* Update which 'ping-pong' layer to write to for the DMA */
        DMAPend(&buffer_layer);

        /* Grab the Wave Struct Mutex Key */
        OSMutexPend(&WaveStructMutexKey, 0, OS_OPT_PEND_BLOCKING, (CPU_TS *)0, &os_err);
        while(os_err != OS_ERR_NONE){ }

        CurrentStruct = WaveStruct;

        /* Release the Mutex Key */
        OSMutexPost(&WaveStructMutexKey, OS_OPT_POST_NONE, &os_err);
        while(os_err != OS_ERR_NONE){ }

        DB3_TURN_ON();

        /* Waveform generator */
        if(CurrentStruct.amp == MIN_STEP){
            for(wave_index = 0; wave_index < BUFFER_SIZE; wave_index++){
                WaveOut[buffer_layer][wave_index] = DC_OFFSET;
            }
        }else{
            volume = ((CurrentStruct.amp) *AC_MAX) /MAX_STEP;
            switch(CurrentStruct.type){
            case SINE:
                for(wave_index = 0; wave_index < BUFFER_SIZE; wave_index++){

                    radians = radians + ( (((INT32S)(CurrentStruct.freq)<<17)/FS )<<14);  //radian index

                    if((radians & 0x80000000) == 0x80000000){
                        radians = ~radians;
                        radians = (2147268900) - radians;
                    }else{}

                    sin_val = arm_sin_q31(radians);
//                    ac_component =  (((volume/2)*(sin_val>>11))>>20);
                    ac_component =  (((volume)*(sin_val>>11))>>20);
                    /* bit shift right to convert to INT16U from q31_t*/
                    WaveOut[buffer_layer][wave_index] = (INT16U)(DC_OFFSET + ac_component);

                    }
                break;

            case TRIANGLE:
                for(wave_index = 0; wave_index < BUFFER_SIZE; wave_index++){

                	/* Calculates radian points for triangle wave */
                    radians = radians + ( (((INT32S)(2 * CurrentStruct.freq)<<17)/FS )<<14);

                    if((radians & 0x80000000) == 0x80000000){
                        radians = ~radians;
                        radians = (2147268900) - radians;
                    }else{}

                    if(radians <= HALF_WAVE){
                        tri_val = radians;

                        if(changeState == 1){
                            changeState = 0;
                            if(Wave_State == POS_WAVE){
                                Wave_State = NEG_WAVE;
                            }else{
                                Wave_State = POS_WAVE;
                            }
                        }else{}
                    }else{
                        changeState = 1;
                        tri_val = FULL_WAVE - radians;
                    }

                    //state machine to set positive and negative sides of wave
                    switch(Wave_State){
                    case POS_WAVE:
                        break;
                    case NEG_WAVE:
                        tri_val = ~tri_val;
                        break;
                    default:

                        break;
                    }

                    //sets wave amplitude
                    ac_component =  ((volume*(tri_val>>11))>>20);
                    WaveOut[buffer_layer][wave_index] = (INT16U)(DC_OFFSET + ac_component);
                }
                break;
            default:
                    break;
            }
        }


    }
}

/********************************************************************
* AmpSet() - Copies the adjusted amplitude to WaveStruct.amp via *lamp
*   (local amplitude) by grabbing access to the Wave Struct via the
*   Mutex.
* 2/13/2018 Maria Watters
********************************************************************/
void AmpSet(INT8U *lamp){
    OS_ERR os_err;

    /* Grab the Wave Struct Mutex Key */
    OSMutexPend(&WaveStructMutexKey, 0, OS_OPT_PEND_BLOCKING, (CPU_TS *)0, &os_err);
    while(os_err != OS_ERR_NONE){ }

    /* Override the former amplitude with the new amplitude */
    WaveStruct.amp = *lamp;

    /* Release the Mutex Key */
    OSMutexPost(&WaveStructMutexKey, OS_OPT_POST_NONE, &os_err);
    OSSemPost(&WaveChgFlag, OS_OPT_POST_1, &os_err);
    while(os_err != OS_ERR_NONE){ }
}

/********************************************************************
* FreqSet() - Copies the adjusted frequency to WaveStruct.amp via *lfreq
*   (local frequency) by grabbing access to the Wave Struct via the
*   Mutex.
* 2/15/2018 Maria Watters
********************************************************************/
void FreqSet(INT16U *lfreq){
    OS_ERR os_err;

    /* Grab the Wave Struct Mutex Key */
    OSMutexPend(&WaveStructMutexKey, 0, OS_OPT_PEND_BLOCKING, (CPU_TS *)0, &os_err);
    while(os_err != OS_ERR_NONE){ }

    /* Override the former amplitude with the new amplitude */
    WaveStruct.freq = *lfreq;

    /* Release the Mutex Key */
    OSMutexPost(&WaveStructMutexKey, OS_OPT_POST_NONE, &os_err);
    OSSemPost(&WaveChgFlag, OS_OPT_POST_1, &os_err);
    while(os_err != OS_ERR_NONE){ }
}

/********************************************************************
* TypeSet() - Copies the selected waveform type to WaveStruct.amp via
*   *ltype (local frequency) by grabbing access to the Wave Struct via the
*   Mutex.
* 2/15/2018 Maria Watters
********************************************************************/
void TypeSet(INT8U *ltype){
    OS_ERR os_err;

    /* Grab the Wave Struct Mutex Key */
    OSMutexPend(&WaveStructMutexKey, 0, OS_OPT_PEND_BLOCKING, (CPU_TS *)0, &os_err);
    while(os_err != OS_ERR_NONE){ }

    /* Override the former amplitude with the new amplitude */
    WaveStruct.type = *ltype;

    /* Release the Mutex Key */
    OSMutexPost(&WaveStructMutexKey, OS_OPT_POST_NONE, &os_err);
    while(os_err != OS_ERR_NONE){ }
}

/********************************************************************
* WaveGet() - Copies WaveStruct.freq to *lfreq and WaveStruct.amp
* to *lamp if the values have changed in the wave struct
* 2/14/2018 Maria Watters
********************************************************************/
void WaveGet(INT8U *lamp, INT16U *lfreq){
    OS_ERR os_err;

    /* Checks if change to Wave Struct has been signaled */
    OSSemPend(&WaveChgFlag,0,OS_OPT_PEND_BLOCKING,(CPU_TS *)0,&os_err);
    while(os_err != OS_ERR_NONE){ }

    /* Updates the element in the task that calls this function with the current value */
    *lamp = WaveStruct.amp;
    *lfreq = WaveStruct.freq;

}
