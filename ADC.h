/********************************************************************
* ADC.h - Header file for ADC module
*
* 03/19/2018 Brian Willis
********************************************************************/
#ifndef ADC_H_
#define ADC_H_

//Note structure
typedef struct{
    INT8C *note;
    INT8U oct;
    INT32U freq;
} NOTE;

void ADCInit(void);
void NotePend(NOTE *new_note);

#endif
