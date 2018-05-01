/********************************************************************
* ADC.c - Module for controlling ADC peripheral
* Uses DMA for sample output and PIT0 for sample trigger, based off
* Todd Morton's ADC demo for the K65 Tower Board.
*
* 03/19/2018 Brian Willis
********************************************************************/

#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "K65TWR_GPIO.h"
#include "ADC.h"

#define SAMPLE_RATE 44100       //Rate in Hz that ADC samples at
#define AVERAGING_PER 100       //Time in ms between frequency calculations

#define SAMPLES 2048            //1024 real parts and 1024 imaginary parts. Creates frequency resolution of 44100/1024 = 43Hz
#define FFT_SIZE SAMPLES/2      //FFT size is number of real samples

#define FREQ_AVG_SIZE 20        //Number of frequency calculations to take an average of for note output (1 = no averaging)

//Offset and gain errors from frequency calculations (found experimentally)
#define OFFSET_ERR 0                    //Measured frequency at ~0Hz accurate
#define GAIN_ERR (30 + OFFSET_ERR)      //Measured frequency at 20kHz is 30Hz too high

FP32 Input[SAMPLES];
FP32 Output[FFT_SIZE];

static void ADCTask(void *p_arg);

//Private resources
static OS_TCB adcTaskTCB;                               //Allocate ADC Task control block
static CPU_STK adcTaskStk[APP_CFG_ADC_TASK_STK_SIZE];   //Allocate ADC Task stack space
static OS_SEM NoteChgFlag;

static INT32U adcFreq[FREQ_AVG_SIZE] = {0};             //Frequencies calculated from ADC's readings
static NOTE noteOut;

/*****************************************************************************************
* ADCInit() - Initializes the ADC peripheral
* Uses ADC0 triggered by PIT1 to take samples of incoming waveform.
*****************************************************************************************/
void ADCInit(){
    OS_ERR os_err;

    //Enable PIT1
    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;        //Start SCGC6 clock for PIT
    PIT_MCR &= ~PIT_MCR_MDIS_MASK;          //Enable PIT via MCR
    PIT_LDVAL1 = 60000000/SAMPLE_RATE;      //Set PIT1 sample rate (Bus Clock = 60MHz)
    PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK;       //Enable timer
    PIT_TCTRL1 |= PIT_TCTRL_TIE_MASK;       //Enable timer interrupt

    //Enable ADC0
    SIM_SCGC6 |= SIM_SCGC6_ADC0(1);         //Start SCGC6 clock for ADC0
    ADC0_CFG1 |= ADC_CFG1_ADIV(3);          //Divide bus clock by 8
    ADC0_CFG1 |= ADC_CFG1_MODE(3);          //Set to 16 bit samples
    ADC0_CFG1 |= ADC_CFG1_ADLSMP(1);        //Set to long samples
    ADC0_SC2 |= ADC_SC2_ADTRG(1);           //Set ADC0 for hardware trigger
    ADC0_SC3 |= ADC_SC3_AVGE(1);            //Enable hardware averager
    ADC0_SC3 |= ADC_SC3_AVGS(3);            //Set hardware averager to 32 samples
    SIM_SOPT7 |= SIM_SOPT7_ADC0TRGSEL(5);   //Set PIT1 as hardware trigger
    SIM_SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN(1); //Set ADC0 to have alternate trigger

    PIT_TFLG1 |= PIT_TFLG_TIF_MASK;         //Reset PIT1 interrupt flag
    NVIC_ClearPendingIRQ(PIT1_IRQn);        //Clear PIT1 pending interrupts

    ADC0_SC1A = ADC_SC1_ADCH(3);            //Set input to DADP3

    do{
        ADC0_SC3 = ADC_SC3_CAL(1);                  //Begin calibration
        while((ADC0_SC3 & ADC_SC3_CAL(1)) == 1){}   //Wait for calibration
    } while((ADC0_SC3 & ADC_SC3_CALF(1)) == 1);     //Repeat if failed

    noteOut.note = "X";
    noteOut.oct = 255;
    noteOut.freq = 0;

    OSTaskCreate(&adcTaskTCB,                       //Create ADC Task
                 "ADC Task",
                 ADCTask,
                 (void *) 0,
                 APP_CFG_ADC_TASK_PRIO,
                 &adcTaskStk[0],
                 (APP_CFG_ADC_TASK_STK_SIZE / 10u),
                 APP_CFG_ADC_TASK_STK_SIZE,
                 0,
                 0,
                 (void *) 0,
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 &os_err);
    while(os_err != OS_ERR_NONE){}                  //Error Trap

    OSSemCreate(&NoteChgFlag, "Note Change Flag Semaphore", 0, &os_err);
    while(os_err != OS_ERR_NONE){}                  //Error Trap
}


/*****************************************************************************************
 * ADCTask() - Controls the ADC peripheral
 * Uses ARM DSP functions to analyze incoming waveform and takes a running average of
 * results to achieve more accurate results.
 *****************************************************************************************/
static void ADCTask(void *p_arg){
    OS_ERR os_err;
    (void)p_arg;
    NOTE note_prev = noteOut;
    INT8U conv_cnt = 0;
    INT8U n = 0;
    INT16U dummy_freq = 0;

    arm_cfft_radix4_instance_f32 S;     //ARM CFFT module
    FP32 maxValue;                      //Max FFT value is stored here
    INT32U maxIndex;                    //Index in Output array where max value is

    while(1){
        for (INT16U i = 0; i < SAMPLES; i = i + 2) {
            while((ADC0_SC1A & ADC_SC1_COCO_MASK) == 0){}
            Input[(INT16U)i] = (FP32)ADC0_RA;   //Real part
            Input[(INT16U)(i + 1)] = 0;         //Imaginary part
        }

        //Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1
        arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1);
        //Process the data through the CFFT/CIFFT module
        arm_cfft_radix4_f32(&S, Input);
        //Process the data through the Complex Magniture Module for calculating the magnitude at each bin
        arm_cmplx_mag_f32(Input, Output, FFT_SIZE);

        //Zero out results that contain useless information
        Output[0] = 0;
        for(int j = FFT_SIZE/2; j < FFT_SIZE; j++){
            Output[j] = 0;
        }

        //Finds max magnitude in output spectrum with corresponding index
        arm_max_f32(Output, FFT_SIZE, &maxValue, &maxIndex);

        //Calculate frequency from location of max magnitude
        adcFreq[conv_cnt] = (SAMPLE_RATE*10000/(FFT_SIZE*10000/maxIndex));
        conv_cnt++;

        //Take average of frequency samples
        if(conv_cnt == FREQ_AVG_SIZE){
            noteOut.freq = 0;
            for(INT8U j = 0; j < FREQ_AVG_SIZE; j++){
                noteOut.freq = noteOut.freq + adcFreq[j];
            }
            noteOut.freq = noteOut.freq/FREQ_AVG_SIZE;
            conv_cnt = 0;

            //Adjust measured frequency for offset and gain errors
            noteOut.freq = noteOut.freq - OFFSET_ERR;
            noteOut.freq = (noteOut.freq*20000)/(20000 + GAIN_ERR - OFFSET_ERR);

            //Find note and octave of measured frequency
            //Find n in freq/2^n <= 31.87, where largest n = octave
            //0th octave ends at 31.87Hz
            //Multiply by 100Hz for 2 decimal points of precision
            while(((noteOut.freq*100)/(1<<n)) > 3187){
                n++;
            }
            noteOut.oct = n;
            n = 0;

            //Find note from frequency downshifted to 0th octave
            //Multiply by 100Hz for 2 decimal points of precision
            dummy_freq = (noteOut.freq*100)/(1<<noteOut.oct);
            if(dummy_freq < 1683){
                noteOut.note = "C";
            } else if((dummy_freq >= 1638) && (dummy_freq < 1783)){
                noteOut.note = "C#";
            } else if((dummy_freq >= 1783) && (dummy_freq < 1890)){
                noteOut.note = "D";
            } else if((dummy_freq >= 1890) && (dummy_freq < 2002)){
                noteOut.note = "D#";
            } else if((dummy_freq >= 2002) && (dummy_freq < 2121)){
                noteOut.note = "E";
            } else if((dummy_freq >= 2121) && (dummy_freq < 2247)){
                noteOut.note = "F";
            } else if((dummy_freq >= 2247) && (dummy_freq < 2381)){
                noteOut.note = "F#";
            } else if((dummy_freq >= 2381) && (dummy_freq < 2523)){
                noteOut.note = "G";
            } else if((dummy_freq >= 2523) && (dummy_freq < 2673)){
                noteOut.note = "G#";
            } else if((dummy_freq >= 2673) && (dummy_freq < 2832)){
                noteOut.note = "A";
            } else if((dummy_freq >= 2673) && (dummy_freq < 3000)){
                noteOut.note = "A#";
            } else if((dummy_freq >= 3000) && (dummy_freq <= 3178)){
                noteOut.note = "B";
            } else{}
        } else{}

        //If note structure updated, set flag
        if((*noteOut.note != *note_prev.note)
            || (noteOut.oct != note_prev.oct)
            || (noteOut.freq != note_prev.freq)){
            OSSemPost(&NoteChgFlag, OS_OPT_POST_1, &os_err);
            while(os_err != OS_ERR_NONE){}          //Error Trap
        } else{}
        note_prev = noteOut;
    }
}

/*****************************************************************************************
 * NotePend() - Sets main module's note to ADC module's note when note updates
 *****************************************************************************************/
void NotePend(NOTE *new_note){
    OS_ERR os_err;

    //Wait for note to be updated
    OSSemPend(&NoteChgFlag, 0, OS_OPT_PEND_BLOCKING, (CPU_TS *)0, &os_err);
    while(os_err != OS_ERR_NONE){}      //Error Trap

    *new_note = noteOut;                   //Update note for LCD screen
}
