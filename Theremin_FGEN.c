/*****************************************************************************************
* Theremin_FGEN Project - uC/OS-III project for the K65 Tower Board that includes a
* user-controlled function generator (A32) as well as a frequency analyzer. The function
* generator can alternate between a sinewave and triangle wave by pressing 'A' and 'B' on
* the keypad, respectively. The amplitude ranges from 165mVpp to 3.3Vpp and is changed by
* the board's touch sensing electrodes. The frequency ranges from 10Hz to 10kHz and is
* changed by pressing entering a frequency and pressing '#' to enter.
* The frequency analyzer (A28) detects the frequency of the inputted waveform and
* displays it on the bottom-right of the board's LCD. Accurately measures and displays
* frequencies from 10Hz to 20kHz.
*
* Frequency generator: Dan Dodge, Maria Watters, Daniel Wilson - 02/13/2018
*                      Brian Willis, Modified for Theremin project - 03/19/2018
* Frequency analyzer: Brian Willis - 03/19/2018
*****************************************************************************************/
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "K65TWR_GPIO.h"
#include "LcdLayered.h"
#include "uCOSKey.h"
#include "Time.h"
#include "Tsi.h"
#include "Wave.h"
#include "ADC.h"

#define A 0x11
#define B 0x12
#define C 0x13

#define NOTE_REFRESH_PER 500    //Period in ms that LCD updates note display

typedef enum {RESET, TIME_SET, TIME} STATE;

/*Private Resources*/
static INT16U Freq = 10;
static NOTE note;

/*****************************************************************************************
* Allocate task control blocks
*****************************************************************************************/
static OS_TCB AppTaskStartTCB;
static OS_TCB UITaskTCB;
static OS_TCB DispTaskTCB;
static OS_TCB NoteDispTaskTCB;

/*****************************************************************************************
* Allocate task stack space.
*****************************************************************************************/
static CPU_STK AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];
static CPU_STK UITaskStk[APP_CFG_UI_TASK_STK_SIZE];
static CPU_STK DispTaskStk[APP_CFG_DISP_TASK_STK_SIZE];
static CPU_STK NoteDispTaskStk[APP_CFG_NOTE_DISP_TASK_STK_SIZE];

/*****************************************************************************************
* Task Function Prototypes.
*****************************************************************************************/
static void AppStartTask(void *p_arg);
static void UITask(void *p_arg);
static void DispUserEntry(INT16U newFreq);
static void DispTask(void *p_arg);
static void NoteDispTask(void *p_arg);

/*****************************************************************************************
* main()
*****************************************************************************************/
void main(void){
    OS_ERR  os_err;

    CPU_IntDis();       //Disable all interrupts, OS will enable them

    OSInit(&os_err);                    //Initialize uC/OS-III
    while(os_err != OS_ERR_NONE){}      //Error Trap
    OSTaskCreate(&AppTaskStartTCB,                  //Address of TCB assigned to task
                 "Start Task",                      //Name you want to give the task
                 AppStartTask,                      //Address of the task itself
                 (void *) 0,                        //p_arg is not used so null ptr
                 APP_CFG_TASK_START_PRIO,           //Priority you assign to the task
                 &AppTaskStartStk[0],               //Base address of task’s stack
                 (APP_CFG_TASK_START_STK_SIZE/10u), //Watermark limit for stack growth
                 APP_CFG_TASK_START_STK_SIZE,       //Stack size
                 0,                                 //Size of task message queue
                 0,                                 //Time quanta for round robin
                 (void *) 0,                        //Extension pointer is not used
                 (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),   //Options
                 &os_err);                          //Ptr to error code destination

    while(os_err != OS_ERR_NONE){}      //Error Trap

    OSStart(&os_err);                   //Start multitasking(i.e. give control to uC/OS)
    while(os_err != OS_ERR_NONE){}      //Error Trap
}

/*****************************************************************************************
* STARTUP TASK
* This should run once and be suspended. Could restart everything by resuming.
* (Resuming not tested)
* Todd Morton, 01/06/2016
*****************************************************************************************/
static void AppStartTask(void *p_arg){
    OS_ERR os_err;
    (void)p_arg;        //void compiler warning for unused variable

    OS_CPU_SysTickInitFreq(DEFAULT_SYSTEM_CLOCK);

    GpioDBugBitsInit();
    LcdInit();
    KeyInit();
    TsiInit();
    WaveInit();
    ADCInit();

    OSTaskCreate(&UITaskTCB,                //Create UITask
                "UITask ",
                UITask,
                (void *) 0,
                APP_CFG_UI_TASK_PRIO,
                &UITaskStk[0],
                (APP_CFG_UI_TASK_STK_SIZE / 10u),
                APP_CFG_UI_TASK_STK_SIZE,
                0,
                0,
                (void *) 0,
                (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                &os_err);
    while(os_err != OS_ERR_NONE){}          //Error Trap

    OSTaskCreate(&DispTaskTCB,              //Create DispTask
                "DispTask",
                DispTask,
                (void *) 0,
                APP_CFG_DISP_TASK_PRIO,
                &DispTaskStk[0],
                (APP_CFG_DISP_TASK_STK_SIZE / 10u),
                APP_CFG_DISP_TASK_STK_SIZE,
                0,
                0,
                (void *) 0,
                (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                &os_err);
    while(os_err != OS_ERR_NONE){}          //Error Trap

    OSTaskCreate(&NoteDispTaskTCB,          //Create NoteDispTask
                "NoteDispTask",
                NoteDispTask,
                (void *) 0,
                APP_CFG_NOTE_DISP_TASK_PRIO,
                &NoteDispTaskStk[0],
                (APP_CFG_NOTE_DISP_TASK_STK_SIZE / 10u),
                APP_CFG_NOTE_DISP_TASK_STK_SIZE,
                0,
                0,
                (void *) 0,
                (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                &os_err);
    while(os_err != OS_ERR_NONE){}          //Error Trap

    OSTaskSuspend((OS_TCB *)0, &os_err);
    while(os_err != OS_ERR_NONE){}          //Error Trap
}

/*****************************************************************************************
* UI TASK - Allows the User to set a new frequency
* Allows for values 10-10000
* # to enter new value
* D to delete previous number
* A to switch waveform to sine wave
* B to switch waveform to triangle wave
*****************************************************************************************/
static void UITask(void *p_arg){
    OS_ERR os_err;
    (void)p_arg;

    INT8C keyPress;
    INT8U keyValue;
    INT8U notDone;
    INT8U waveform = 1;
    INT16U newFreq;

    TypeSet(&waveform);
    FreqSet(&Freq);
    while(1){
    	notDone = 1;
    	newFreq = 0;

    	//reads user input character
    	while(notDone){
			keyPress = KeyPend(0, &os_err);

			switch(keyPress){
			case '#': if(newFreq >=10){
							notDone = 0;
						}
						else{
						}
						break;
			//if a D is entered, moves back one space
			case 0x14: newFreq = newFreq/10;
					  break;
			//A and B set the waveform type
			case 0x11: waveform = 1;
				TypeSet(&waveform);
				break;
			case 0x12: waveform = 2;
				TypeSet(&waveform);
				break;
			//any number updates the new frequency
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
			case '0': keyValue = keyPress - 48;
					if(!(((newFreq*10)+keyValue)>10000)){
						newFreq = newFreq*10;
						newFreq += keyValue;
					}
					else{
					}
					break;
			default: break;
			}

			//updates display
			if(notDone){
			    LcdDispClear(FREQ_SET_LAYER);
				DispUserEntry(newFreq);
			}
			else{
				Freq = newFreq;
				FreqSet(&Freq);
				LcdDispClear(FREQ_SET_LAYER);
			}
		}
    }
}


/*****************************************************************************************
* DispUserEntry - helper function for UITask to
* - Displays user frequency entry on bottom left corner on LCD layer 1
*****************************************************************************************/
static void DispUserEntry(INT16U newFreq){
	INT8U numDigits = 0;
	INT8U currentDigit;
	INT16U freqCopy = newFreq;

	//frequency size found
	while(freqCopy > 0){
		numDigits++;
		freqCopy /= 10;
	}

	//frequency displayed
	while(newFreq > 0){
		currentDigit = newFreq%10;
		LcdDispChar(2, numDigits, FREQ_SET_LAYER, currentDigit+48);
		numDigits--;
		newFreq /= 10;
	}
}


/*****************************************************************************************
* Display TASK - Pends on WaveChgFlag
* - Display current volume in top right corner of LCD on top layer
* - Display current frequency in top left corner of LCD on top layer
*****************************************************************************************/
static void DispTask(void *p_arg){

	INT8U numDigits;
	INT8U currentDigit;
	INT16U freqCurrent;
	INT16U freqCopy;
	INT8U ampCurrent;

    (void)p_arg;

    while(1) {
        DB2_TURN_OFF();
        WaveGet(&ampCurrent, &freqCurrent);
        DB2_TURN_ON();
        LcdDispClear(TERM_LAYER);
        numDigits = 0;
        freqCopy = freqCurrent;

        //frequency size found
		while(freqCopy > 0){
			numDigits++;
			freqCopy /= 10;
		}

		//frequency displayed
		while(freqCurrent > 0){
			currentDigit = freqCurrent%10;
			LcdDispChar(1, numDigits, TERM_LAYER, currentDigit+48);
			numDigits--;
			freqCurrent /= 10;
		}

		//amplitude displayed
		if(ampCurrent >= 10){
			currentDigit = ampCurrent%10;
			LcdDispChar(1, 16, TERM_LAYER, currentDigit+48);
			currentDigit = (ampCurrent/10)%10;
			LcdDispChar(1, 15, TERM_LAYER, currentDigit+48);
		}
		else{
			LcdDispChar(1, 16, TERM_LAYER, ampCurrent+48);
		}
    }
}

/*****************************************************************************************
* NoteDisplayTask - Pends on NoteChgFlag
* - Displays current note from inputted waveform (A28) onto LCD screen via data from ADC
*   up to 999999Hz.
*****************************************************************************************/
static void NoteDispTask(void *p_arg){
    OS_ERR os_err;
    (void)p_arg;
    INT8U freq_low = 0;
    INT8U freq_mid = 0;
    INT8U freq_hi = 0;

    while(1){
        OSTimeDly(NOTE_REFRESH_PER, OS_OPT_TIME_PERIODIC, &os_err); //Update LCD with new note periodically
        while(os_err != OS_ERR_NONE){}                              //Error Trap
        NotePend(&note);                                            //Wait for note to change

        LcdDispClear(NOTE_DISP_LAYER);
        //Note
        LcdDispString(1, 7, NOTE_DISP_LAYER, note.note);

        //Octave
        LcdDispByte(1, 12, NOTE_DISP_LAYER, note.oct);
        LcdDispString(1, 9, NOTE_DISP_LAYER, "Oct:");

        //Frequency
        freq_low =(INT8U)(note.freq-((note.freq/100)*100));
        freq_mid = (INT8U)((note.freq-((note.freq/10000)*10000))/100);
        freq_hi = (INT8U)(note.freq/10000);

        if((freq_hi == 0) && (freq_mid == 0)){
            LcdDispDecByte(2, 12, NOTE_DISP_LAYER, freq_low, 0);
        } else{
            LcdDispDecByte(2, 12, NOTE_DISP_LAYER, freq_low, 1);
        }

        if((freq_hi == 0) && (freq_mid != 0)){
            LcdDispDecByte(2, 10, NOTE_DISP_LAYER, freq_mid, 0);
        } else if((freq_hi != 0)){
            LcdDispDecByte(2, 10, NOTE_DISP_LAYER, freq_mid, 1);
        } else{}

        if(freq_hi != 0){
            LcdDispDecByte(2, 8, NOTE_DISP_LAYER, freq_hi, 0);
        } else{}
        LcdDispString(2, 15, NOTE_DISP_LAYER, "Hz");
    }
}

