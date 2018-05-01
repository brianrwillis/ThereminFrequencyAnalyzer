/*
 * DMA.h
 *
 *  Created on: Feb 13, 2018
 *      Author: dodged2
 */

#ifndef SOURCES_DMA_H_
#define SOURCES_DMA_H_

/*****************************************************************************************
* DMA Initializations
* - Argument passed through is an address to the Wave ping-pong buffer.
* - DMA will transfer data to the DAC from the Wave block currently not being written to.
*****************************************************************************************/
void DMAInit(INT16U *wave_info_block);

/*****************************************************************************************
* DMA IRQ
* - Posting dmaBlockRdy.flag tells WaveTask that the DMA is moving on to next block
* - dmaBlockRdy.index tells WaveTask which block to currently write to.
*****************************************************************************************/
void DMA0_DMA16_IRQHandler(void);

/*************************************************************************
 * Allows pending of dmaBlockRdy.flag in Wave.c -> in WaveTask()
 * This is keep WaveTask in waiting until flag has been posted
 * Copies current dmaBlockRdy.index to *buffer_block when dmaBlockRdy.flag is posted
 *************************************************************************/
void DMAPend(INT8U *buffer_block);

#endif /* SOURCES_DMA_H_ */
