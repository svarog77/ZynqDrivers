/*
 * SPI_PL.c
 *
 *  Created on: 3 èþë. 2024 ã.
 *      Author: Viacheslav
 */

/***************************** Include Files ********************************/
#include "SPI_PL.h"

/************************** Variable Definitions ****************************/
/* Declare instance and associated pointer for ... */



/******************************************/
// Declare local SPI receive buffer
static uint8_t spi_rx_buf [RX_NBYTES]= {0};
/*---------------------------------------------------------------------------*/
/*------------------------------- FUNCTIONS ---------------------------------*/
/*---------------------------------------------------------------------------*/
/*****************************************************************************
 * Function: SPI_PL_Init()
 *//**
 *
 * @brief		Configures the SPI_PL for use.
 *
 * @details This function configures the axi spi. It sets the SPI protocol, mode of
 *	operation, bit rate, and data width.
 * @param   The pointer to Configuration data
 * @param	SpiDeviceId is the Device ID of the Spi Device and is the
 *			XPAR_<SPI_instance>_DEVICE_ID value from xparameters.h.
 * @param 	SpiInstancePtr is a pointer to the instance of Spi component.
 * @param	The ui32Protocol parameter defines the data frame format.
 * @param	The ui32Mode parameter defines the operating mode of the SPI module.
 * @param   The Manual Slave Select option.
 * @return	Integer indicating result of configuration attempt.
 * 				0 = SUCCESS, 1 = FAILURE
 * */

int SPI_PL_Init(XSpi_Config *SpiConfig, u16 SpiDeviceId, XSpi* spi, u32 Protocol, u32 Mode, bool Ssel){
	int Status;
	u32 Options;
	/* === START CONFIGURATION SEQUENCE ===  */
	/* ---------------------------------------------------------------------
	 * ------------ DEVICE LOOK-UP ------------
	 * -------------------------------------------------------------------- */
    SpiConfig = XSpi_LookupConfig(SpiDeviceId);
    if (SpiConfig == NULL) {
    	xil_printf("\033[43[SPI_PL%d] Return config - NULL \r\n\033[0m", SpiDeviceId);
    	return XST_FAILURE;
    }
	 /* ---------------------------------------------------------------------
	  * ------------ DRIVER INITIALISATION ------------
	  * -------------------------------------------------------------------- */
    Status = XSpi_CfgInitialize(spi, SpiConfig, SpiConfig->BaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("\033[3;31;47m[SPI_PL%d] Return initCFG SPI_PL - FAILURE\n status - %2d\n\033[0m",SpiDeviceId, Status);
		return XST_FAILURE;
	}
	/* ---------------------------------------------------------------------
	* ------------  SELF TEST ------------
	* -------------------------------------------------------------------- */
//	Status = XSpi_SelfTest(spi);
//	if (Status != XST_SUCCESS) {
//		xil_printf("\033[3;31;47m[SPI_PL%d] Return Set Options SPI_PL - FAILURE\n status - %2d\n\033[0m",SpiDeviceId, Status);
//		return XST_FAILURE;
//	}
	/* ---------------------------------------------------------------------
	* ------------ PROJECT-SPECIFIC CONFIGURATION ------------
	* -------------------------------------------------------------------- */
	Options = Protocol;
	Options |= Mode;
	if(Ssel){
		Options |= XSP_MANUAL_SSELECT_OPTION;
	}
    Status = XSpi_SetOptions(spi, Options);
    if (Status != XST_SUCCESS) {
        return Status;
    }
    /* Selects or deselect the slave with which the master communicates*/
    Status = XSpi_SetSlaveSelect(spi, 1);
    if (Status != XST_SUCCESS) {
		xil_printf("\033[3;31;47m[SPI_PL%d] Return Set Slave SPI_PL - FAILURE\n status - %2d\n\033[0m",SpiDeviceId, Status);
        return Status;
    }
    Status = XSpi_Start(spi);
    if (Status != XST_SUCCESS) {
		xil_printf("\033[3;31;47m[SPI_PL%d] Return Start SPI_PL - FAILURE\n status - %2d\n\033[0m",SpiDeviceId, Status);
        return Status;
    }
    /* Disable Global interrupt to use polled mode operation */
    XSpi_IntrGlobalDisable(spi);

	return XST_SUCCESS;
}

/** Function: SPI_PL_Read()
 * The spiReadBytes() function is used by higher level blocks to read data back on the SPI bus*/
uint8_t* SPI_PL_Read(XSpi*p_XSpiInst, uint8_t* tx_data, uint8_t nbytes)
{
  XSpi_Transfer(p_XSpiInst, tx_data, spi_rx_buf, nbytes);
  return spi_rx_buf;
}

/** Function:SPI_PL_Write()
 *//*
 * This function is used to write*/
void SPI_PL_Write(XSpi*p_XSpiInst, uint8_t* tx_data, uint8_t nbytes)
{
  XSpi_Transfer(p_XSpiInst, tx_data, NULL, nbytes);
}
/****** End functions *****/

/****** End of File **********************************************************/
