/*
 * DMA.c
 *
 *  Created on: Feb 13, 2018
 *      Author: dodged2
 */
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "K65TWR_GPIO.h"
#include "DMA.h"

#define WAVE_DMA_OUT_CH 0
#define SIZE_CODE_16BIT 1
#define WAVE_BYTES_PER_SAMPLE 2
#define WAVE_NUM_BLOCKS 2
#define WAVE_SAMPLES_PER_BLOCK 60  //number of items in each block
#define WAVE_BYTES_PER_BUFFER 240

typedef struct{
    OS_SEM flag;
    INT8U index;
}DMA_RDY;

DMA_RDY dmaBlockRdy;


/*****************************************************************************************
* DMA Initializations
* - Argument passed through is an address to the Wave ping-pong buffer.
* - DMA will transfer data to the DAC from the Wave block currently not being written to.
*****************************************************************************************/
void DMAInit(INT16U *WaveOut){

    OS_ERR os_err;
    OSSemCreate(&dmaBlockRdy.flag, "Block Ready", 0, &os_err);
    // dmaInBlockRdy.index indicates the buffer currently not being used by the DMA
    // in the Ping-Pong scheme.
    // This is a bit more open loop than I like but there doesn't seem to be a status
    // bit that distinguishes between a half-full interrupt,INTHALF, and a full.
    // interrupt, INTMAJOR. Bottom line, this has to start at one. The DMA fills the
    // [0] block first so, by the time     the HALFINT happens, it is working on the [1]
    // block. The ISR toggles the initial value to zero so the [0] block is processed
    //first.
    dmaBlockRdy.index = 1;
    // Turn on clocks for the DMA, DMAMUX, DAC, and PIT
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX(1) | SIM_SCGC6_PIT(1);
    SIM_SCGC7 |= SIM_SCGC7_DMA(1);
    SIM_SCGC2 |= SIM_SCGC2_DAC0(1);

    VREF_SC = VREF_SC_VREFEN(1) | VREF_SC_REGEN(1);
    DAC0_C0 = DAC_C0_DACEN(1) | DAC_C0_DACRFS(1) | DAC_C0_DACTRGSEL(0);
    DAC0_C1 |= DAC_C1_DMAEN(1); //enable DMA trigger

    //Make sure DMAMUX is disabled
    DMAMUX_CHCFG(WAVE_DMA_OUT_CH) |= DMAMUX_CHCFG_ENBL(0)|DMAMUX_CHCFG_TRIG(0);
    //Configure DMA Channel
    //set source address to read from WaveOut
    DMA_SADDR(WAVE_DMA_OUT_CH) = DMA_SADDR_SADDR(WaveOut);
    //Source size is 2 bytes, destination size is 2 bytes. No modulo feature.
    DMA_ATTR(WAVE_DMA_OUT_CH) = DMA_ATTR_SMOD(0) | DMA_ATTR_SSIZE(SIZE_CODE_16BIT) | DMA_ATTR_DMOD(0) | DMA_ATTR_DSIZE(SIZE_CODE_16BIT);
    DMA_SOFF(WAVE_DMA_OUT_CH) = DMA_SOFF_SOFF(WAVE_BYTES_PER_SAMPLE);
    //Minor loop size is the sample size
    DMA_NBYTES_MLNO(WAVE_DMA_OUT_CH) = DMA_NBYTES_MLNO_NBYTES(WAVE_BYTES_PER_SAMPLE);
    //Set minor loop iteration counters to number of minor loops in the major loop
    DMA_CITER_ELINKNO(WAVE_DMA_OUT_CH) = DMA_CITER_ELINKNO_ELINK(0)|DMA_CITER_ELINKNO_CITER(WAVE_NUM_BLOCKS*WAVE_SAMPLES_PER_BLOCK);
    DMA_BITER_ELINKNO(WAVE_DMA_OUT_CH) = DMA_BITER_ELINKNO_ELINK(0)|DMA_BITER_ELINKNO_BITER(WAVE_NUM_BLOCKS*WAVE_SAMPLES_PER_BLOCK);
    //After major loop is done set the address back to the first byte of the buffer
    DMA_SLAST(WAVE_DMA_OUT_CH) = DMA_SLAST_SLAST(-(WAVE_BYTES_PER_BUFFER));
    //Set transmit destination address to the DAC data register
    DMA_DADDR(WAVE_DMA_OUT_CH) = DMA_DADDR_DADDR(&DAC0_DAT0L);
    //No change in destination address
    DMA_DOFF(WAVE_DMA_OUT_CH) = DMA_DOFF_DOFF(0);
    //No adjustment to destination address.
    DMA_DLAST_SGA(WAVE_DMA_OUT_CH) = DMA_DLAST_SGA_DLASTSGA(0);

    //PIT setup for 48k samples per second
    PIT_MCR &= PIT_MCR_MDIS(0);
    PIT_TCTRL0 |= PIT_TCTRL_TIE(1);
    PIT_TCTRL0 |= PIT_TCTRL_TEN(1);
    //PIT_LDVAL0 |= 120;//testing lower values for faster Dac sampling
    PIT_LDVAL0 |= 1249;                 /* sets the timer start value */
    PIT_TFLG0 |= PIT_TFLG_TIF(1);       /* clear ISF Flag and enable IRQ */

    //Enable interrupt at half filled buffer and end of major loop.
    //This allows "ping-pong" buffer processing.
    DMA_CSR(WAVE_DMA_OUT_CH) = DMA_CSR_ESG(0) | DMA_CSR_MAJORELINK(0) | DMA_CSR_BWC(3) | DMA_CSR_INTHALF(1) |  DMA_CSR_INTMAJOR(1) | DMA_CSR_DREQ(0) | DMA_CSR_START(0);
    //Set the DMAMUX to source 60, enable triggering and enable DMAMUX
    DMAMUX_CHCFG(WAVE_DMA_OUT_CH) = DMAMUX_CHCFG_ENBL(1)|DMAMUX_CHCFG_TRIG(1)|DMAMUX_CHCFG_SOURCE(60);

    //enable DMA interrupt
    NVIC_EnableIRQ(WAVE_DMA_OUT_CH);
    //All set to go, enable DMA channel(s)!
    DMA_SERQ = DMA_SERQ_SERQ(WAVE_DMA_OUT_CH);

}

/*****************************************************************************************
* DMA IRQ
* - Posting dmaBlockRdy.flag tells WaveTask that the DMA is moving on to next block
* - dmaBlockRdy.index tells WaveTask which block to currently write to.
*****************************************************************************************/
void DMA0_DMA16_IRQHandler(void){
    OS_ERR os_err;
    NVIC_ClearPendingIRQ(DMA0_DMA16_IRQn);
    DMA_CINT = DMA_CINT_CINT(0);
    if(dmaBlockRdy.index == 1){
        dmaBlockRdy.index = 0u;
    }else{
        dmaBlockRdy.index = 1u;
    }
    //dmaBlockRdy.index ^= 1;
    //dmaBlockRdy.flag is pended for in WaveTask()
    (void)OSSemPost(&(dmaBlockRdy.flag), OS_OPT_POST_1, &os_err);
    while(os_err != OS_ERR_NONE){
    }
}

/*************************************************************************
 * Allows pending of dmaBlockRdy.flag in Wave.c -> in WaveTask()
 * This is keep WaveTask in waiting until flag has been posted
 * Copies current dmaBlockRdy.index to *buffer_block when dmaBlockRdy.flag is posted
 *************************************************************************/
void DMAPend(INT8U *buffer_layer){
    OS_ERR os_err;

    (void)OSSemPend(&(dmaBlockRdy.flag), 0, OS_OPT_PEND_BLOCKING, (CPU_TS *)0, &os_err);
    while(os_err != OS_ERR_NONE){           /* Error Trap                  */
    }
    *buffer_layer = dmaBlockRdy.index;
}
