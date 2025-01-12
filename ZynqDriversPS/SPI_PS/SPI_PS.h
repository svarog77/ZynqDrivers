/*
 * SPI_PS.h
 *
 *  Created on: 16 рту. 2024 у.
 *      Author: Viacheslav
 */

#ifndef ZYNQDRIVERSPS_SPI_PS_SPI_PS_H_
#define ZYNQDRIVERSPS_SPI_PS_SPI_PS_H_

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "xparameters.h"

#include "xil_printf.h"
#include "xil_assert.h"
#include "xil_types.h"
#include "xil_io.h"

#include "xspips.h"


#include "../GIC/GIC.h"
/*****************************************************************************/
/************************** Constant Definitions *****************************/
/*****************************************************************************/
#define SPI0_DEV_ID XPAR_PS7_SPI_0_DEVICE_ID
#define SPI1_DEV_ID 0//XPAR_PS7_SPI_1_DEVICE_ID
#define PS7_SPI_CLK	XPAR_PS7_SPI_0_SPI_CLK_FREQ_HZ
#define SPI_MASTER	XSPIPS_MASTER_OPTION
#define SPI0_INTR_ID	XPAR_PS7_SPI_0_INTR
#define SPI1_INTR_ID	XPAR_PS7_SPI_1_INTR
/* Receive buffer size */
#define RX_NBYTES					16U
#define SPI_DEV_MANAGING            0

/*****************************************************************************/
/************************** Variable Declarations ****************************/
/*****************************************************************************/


/*****************************************************************************/
/************************ Macros (Inline Functions) **************************/
/*****************************************************************************/
#define SpiPs_RecvByte(BaseAddress) (u8)	XSpiPs_In32((BaseAddress)  + XSPIPS_RXD_OFFSET)
#define SpiPs_SendByte(BaseAddress, Data) 	XSpiPs_Out32((BaseAddress) + XSPIPS_TXD_OFFSET, (Data))
/*****************************************************************************/
/************************ Enum Typedef Data******** **************************/
/*****************************************************************************/
typedef enum SPI_PS_PROTOCOL{
		MODE0 = 0 ,
		MODE3 = XSPIPS_CLK_ACTIVE_LOW_OPTION |XSPIPS_CLK_PHASE_1_OPTION,
		MODE2 = XSPIPS_CLK_ACTIVE_LOW_OPTION,
		MODE1 = XSPIPS_CLK_PHASE_1_OPTION
}SPI_Protocol_t;
/*The Mode parameter defines the operating mode of the SPI module*/
typedef enum SPI_PS_MODE{
		SLAVEps,
		MASTERps
}SPI_Mode_t;

/*The Manual Slave Select option.*/
typedef enum SPI_PS_SSELECT{
		EN_SEL	= 0x01,
		DIS_SEL = 0x02
}SPI_Manual_SS_t;

/*The Manual Start Enable option.*/
typedef enum SPI_Start_EN{
		ON_Start	= 0x01,
		OFF_Start   = 0x02
}SPI_Manual_Start_t;

/*****************************************************************************/
/************************** Function Prototypes ******************************/
/*****************************************************************************/
static int SPI_PS_Init			(XSpiPs_Config *SpiConfig, u16 SpiDeviceId, XSpiPs *xspiInst, SPI_Protocol_t Protocol,
								 SPI_Mode_t Mode,u8 ClkPrescale, SPI_Manual_SS_t Manual_CS, SPI_Manual_Start_t ManStart);

int 		SPIM0_Init			(SPI_Protocol_t Protocol, SPI_Manual_SS_t Manual_CS, u8 ClkPrescale, SPI_Manual_Start_t ManStart);
int 		SPIS0_Init			(SPI_Protocol_t Protocol, u8 ClkPrescale);
int 		SPIM0_TX			(u8 *SendBuffer, int ByteCount);
void 		SPIM0_Send			(u8 *SendBuffer, int ByteCount);

int 		SPIM1_Init			(SPI_Protocol_t Protocol, SPI_Manual_SS_t Manual_CS, u8 ClkPrescale, SPI_Manual_Start_t ManStart);
int 		SPIS1_Init			(SPI_Protocol_t Protocol, u8 ClkPrescale);
int 		SPIM1_TX			(u8 *SendBuffer, int ByteCount);
void 		SPIM1_Send			(u8 *SendBuffer, int ByteCount);

void 		SpiPsHandler				(void *CallBackRef, u32 StatusEvent, unsigned int ByteCount);
int  		addSpi0ToInterruptSystem	(uint32_t p_SpiInst);
int  		addSpi1ToInterruptSystem	(uint32_t p_SpiInst);

static void 		SpiPs_RX					(XSpiPs SpiInstance, u8 *ReadBuffer,int ByteCount);
static void 		SpiPs_TX					(XSpiPs SpiInstance, u8 *SendBuffer, int ByteCount);
static void 		SpiPsDisableIntrSystem		(XScuGic *IntcInstancePtr, u16 SpiIntrId);
static bool 		SPI_RxFifoIsNotEmpty		(XSpiPs SpiInstance);
static bool 		SPI_TxFifoIsNotEmpty		(XSpiPs SpiInstance);

#endif /* ZYNQDRIVERSPS_SPI_PS_SPI_PS_H_ */
