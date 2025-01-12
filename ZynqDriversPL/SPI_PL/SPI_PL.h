/*
 * SPI_PL.h
 *
 *  Created on: 3 èþë. 2024 ã.
 *      Author: Viacheslav
 */

#ifndef ZYNQDRIVERSPL_SPI_PL_SPI_PL_H_
#define ZYNQDRIVERSPL_SPI_PL_SPI_PL_H_

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "xparameters.h"
#include "xspi.h"

#include "xil_printf.h"

/*****************************************************************************/
/************************** Constant Definitions *****************************/
/*****************************************************************************/
#define CNTRL_SPI_PL  "SPI_PL"
// Receive buffer size
#define RX_NBYTES        16U
/*****************************************************************************/
/************************ Macros (Inline Functions) **************************/
/*****************************************************************************/
/* The Protocol parameter defines the data frame format.
 * Polarity Phase Mode
	   1 	   0 		MODE_0
	   0 	   1 		MODE_1
	   0 	   0 		MODE_2
	   1 	   1 		MODE_3 */
#define MODE0	~(XSP_CLK_ACTIVE_LOW_OPTION |XSP_CLK_PHASE_1_OPTION)
#define MODE1   XSP_CLK_ACTIVE_LOW_OPTION|XSP_CLK_PHASE_1_OPTION
#define MODE2   ~XSP_CLK_ACTIVE_LOW_OPTION
#define MODE3   XSP_CLK_PHASE_1_OPTION
/*****************************************************************************/
/************************** Variable Declarations ****************************/
/*****************************************************************************/
static enum SPI_PL_PROTOCOL{
		MODE_0 = 0 ,
		MODE_1 = XSP_CLK_ACTIVE_LOW_OPTION |XSP_CLK_PHASE_1_OPTION,
		MODE_2 = XSP_CLK_ACTIVE_LOW_OPTION,
		MODE_3 = XSP_CLK_PHASE_1_OPTION
};

/*The Mode parameter defines the operating mode of the SPI module*/
static enum SPI_PL_MODE{
		SLAVE,
		MASTER = XSP_MASTER_OPTION
};

/*The Manual Slave Select option.*/
static enum SPI_SSELECT{
		SEL_EN	= true,
		SEL_OFF = false
};


/*****************************************************************************/
/************************** Function Prototypes ******************************/
/*****************************************************************************/
int 		SPI_PL_Init		(XSpi_Config *SpiConfig, u16 SpiDeviceId, XSpi* spi, u32 Protocol, u32 Mode, bool Ssel);
uint8_t* 	SPI_PL_Read		(XSpi*p_XSpiInst, uint8_t* tx_data, uint8_t nbytes);
void 		SPI_PL_Write	(XSpi*p_XSpiInst, uint8_t* tx_data, uint8_t nbytes);
#endif /* ZYNQDRIVERSPL_SPI_PL_SPI_PL_H_ */
