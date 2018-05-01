/********************************************************************
 * Wave.h - The header file for the wave module, Wave.c
 * 02/13/2018 Maria Watters
*********************************************************************
* Public Resources
********************************************************************/

#ifndef SOURCES_WAVE_H_
#define SOURCES_WAVE_H_

/* Global resources */
typedef struct{
    INT8U type;
    INT8U amp;
    INT16U freq;
} WAVE_T;

/******************************************************************************
 * WaveInit() - Initialization routine for the waveform generator
 ******************************************************************************/
void WaveInit(void);
/********************************************************************
* AmpSet() - Copies *lamp to WaveStruct.amp
********************************************************************/
void AmpSet(INT8U *lamp);
/********************************************************************
*FreqSet() - Copies *lfreq to WaveStruct.freq
********************************************************************/
void FreqSet(INT16U *lfreq);
/********************************************************************
* TypeSet() - Copies *ltype to WaveStruct.type
********************************************************************/
void TypeSet(INT8U *ltype);
/********************************************************************
* WaveGet() - Copies WaveStruct.freq to *lfreq and WaveStruct.amp
*   to *lamp if the values have changed in the wave struct
********************************************************************/
void WaveGet(INT8U *lamp, INT16U *lfreq);

#endif /* SOURCES_WAVE_H_ */
