/********************************************************************
 * Tsi.c - A real time clock module that runs under MicroC/OS for
 *  a user adjustable clock.
 *
 *  02/31/2018 Maria Watters
 *********************************************************************
* Header Files - Dependencies
********************************************************************/
#include "MCUType.h"
#include "os.h"
#include "app_cfg.h"
#include "K65TWR_GPIO.h"
#include "Wave.h"
#include "Tsi.h"

/*********************************************************************
* Private Resources
********************************************************************/
#define E1_TOUCH_OFFSET 0x1250U    /* Increases sensitivity of TSI */
#define E2_TOUCH_OFFSET 0x1500U
#define STEP_SIZE 1               /* Amplitude is incremented/decremented linearly */
#define MAX_STEP 20               /* Amplitude is divided into 21 steps (0-20) */
#define MIN_STEP 0
static INT16U tsiBaselineLevels[2];
static INT16U tsiTouchLevels[2];
typedef enum{ELECTRODE2, ELECTRODE1} SENSOR_T;
static WAVE_T NewVoltage;

/************************************* ****************************************************
* Allocate task control blocks
*****************************************************************************************/
static OS_TCB TsiTaskTCB;

/*****************************************************************************************
* Allocate task stack space
*****************************************************************************************/
static CPU_STK TsiTaskStk[APP_CFG_TSI_TASK_STK_SIZE];

/******************************************************************************
 * Task Function Prototypes
 ******************************************************************************/
static void TsiTask(void *p_arg);

/******************************************************************************
 * TsiInit() - Initializes the clocks for PORTB and touch sensors,
 *  disables the PORTB pins for the electrodes, adjusts the parameters of the
 *  touch sensor to be:
 *  REFCHARG: Iref = 16 uA
 *  DVOLT: dV = 592 mV
 *  EXTCHRG: Ielec = 16 uA
 *  PS: sets prescaler to /32
 *  NSCN: electrode measured 16 times a scan
 *  STM: Software trigger scan
 *
 *  02/13/2018 Maria Watters
 ******************************************************************************/
void TsiInit(void){

    OS_ERR os_err;

    /* Initialization of electrodes */
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;                             /* Enables the clock for PORTB */
    SIM_SCGC5 |= SIM_SCGC5_TSI_MASK;                               /* Enables the clock for TSI */
    PORTB_PCR18 &= ~PORT_PCR_MUX_MASK;                             /* Disables the PORTB pin for electrode 2 */
    PORTB_PCR19 &= ~PORT_PCR_MUX_MASK;                             /* Disables the PORTB pin for electrode 1 */
    TSI0_GENCS |= TSI_GENCS_REFCHRG(5);
    TSI0_GENCS |= TSI_GENCS_DVOLT(1);
    TSI0_GENCS |= TSI_GENCS_EXTCHRG(5);
    TSI0_GENCS |= TSI_GENCS_PS(5);
    TSI0_GENCS |= TSI_GENCS_NSCN(15);
    TSI0_GENCS |= TSI_GENCS_TSIEN(1);
    TSI0_GENCS &= ~TSI_GENCS_STM(1);
    NewVoltage.amp = 20u;                                                /* Initializes step to 20 */

    /* Task creation for TSI Task */
    OSTaskCreate(&TsiTaskTCB,
        "TSI Task",
        TsiTask,
        (void *)0,
        APP_CFG_TSI_TASK_PRIO,
        &TsiTaskStk[0],
        (APP_CFG_TSI_TASK_STK_SIZE / 10u),
        APP_CFG_TSI_TASK_STK_SIZE,
        0,
        0,
        (void *)0,
        (OS_OPT_TASK_NONE),
        &os_err);

    /* Error Trap */
    while(os_err != OS_ERR_NONE){}

    /* Morton's code to set touch level offsets: Electrode 1 */
    TSI0_DATA = TSI_DATA_TSICH(12);                                 //TSI0_CH12 is ELECTRODE1
    TSI0_DATA |= TSI_DATA_SWTS(1);                                  //start a scan sequence
    while(!(TSI0_GENCS & TSI_GENCS_EOSF_MASK)){}                    //wait for scan to finish
    TSI0_GENCS |= TSI_GENCS_EOSF(1);                                //Clear scan flag
    tsiBaselineLevels[ELECTRODE1] = (INT16U)(TSI0_DATA & TSI_DATA_TSICNT_MASK);
    tsiTouchLevels[ELECTRODE1] = tsiBaselineLevels[ELECTRODE1] + E1_TOUCH_OFFSET;
    /* Electrode 2 */
    TSI0_DATA = TSI_DATA_TSICH(11);                                 //TSI0_CH11 is ELECTRODE2
    TSI0_DATA |= TSI_DATA_SWTS(1);                                  //start a scan sequence
    while(!(TSI0_GENCS & TSI_GENCS_EOSF_MASK)){}                    //wait for scan to finish
    TSI0_GENCS |= TSI_GENCS_EOSF(1);                                //Clear scan flag
    tsiBaselineLevels[ELECTRODE2] = (INT16U)(TSI0_DATA & TSI_DATA_TSICNT_MASK);
    tsiTouchLevels[ELECTRODE2] = tsiBaselineLevels[ELECTRODE2] + E2_TOUCH_OFFSET;

}

/******************************************************************************
 * TsiTask() - Controls the TSI sensor scanning and increments/decrements the
 *  amplitude of the waveform based if a sensor is pressed. In order for the
 *  amplitude to increment/decrement, the amplitude must also be within the
 *  0-1 Vpp range, and not go out of the range.
 *  OSTimeDly() puts the TsiTask in to the WAITING state for lower priority
 *  tasks to run. The delay is set to 20 ms, which will keep a scan from being
 *  missed.
 ******************************************************************************/
static void TsiTask(void *p_arg){
    OS_ERR os_err;
    INT8U leftBounce = 0;
    INT8U rightBounce = 0;
    (void)p_arg;

    while(1){

        DB3_TURN_OFF();
        OSTimeDly(10,OS_OPT_TIME_PERIODIC,&os_err);
        while(os_err != OS_ERR_NONE){ }
        DB3_TURN_ON();


        /*Start sequence for Electrode 1 */
        TSI0_DATA = TSI_DATA_TSICH(12);
        TSI0_DATA |= TSI_DATA_SWTS(1);


        /* If left electrode has been pressed, increment amplitude */
        if( ((INT16U)(TSI0_DATA & TSI_DATA_TSICNT_MASK) > tsiTouchLevels[ELECTRODE1]) && (NewVoltage.amp < MAX_STEP)){
            if(leftBounce == 0){
            	NewVoltage.amp = NewVoltage.amp + STEP_SIZE;
            }
            else{}
            AmpSet(&NewVoltage.amp);
            leftBounce = 1;
        }else{
        	leftBounce = 0;
        }

        DB3_TURN_OFF();
        OSTimeDly(10,OS_OPT_TIME_PERIODIC,&os_err);
        while(os_err != OS_ERR_NONE){ }
        DB3_TURN_ON();



        /*Start sequence for Electrode 2 */
        TSI0_DATA = TSI_DATA_TSICH(11);
        TSI0_DATA |= TSI_DATA_SWTS(1);

        /* If right electrode has been pressed, decrement amplitude */
        if(((INT16U)(TSI0_DATA & TSI_DATA_TSICNT_MASK) > tsiTouchLevels[ELECTRODE2]) && (NewVoltage.amp > MIN_STEP)){
            if(rightBounce == 0){
            	NewVoltage.amp = NewVoltage.amp - STEP_SIZE;
            }
            else{}
            AmpSet(&NewVoltage.amp);
            rightBounce = 1;
        }else{
        	rightBounce = 0;
        }

    }
}

