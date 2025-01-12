/*
 * SPI_PS.c
 *
 *  Created on: 15 ‡‚„. 2024 „.
 *      Author: Viacheslav
 */

/*Example of use
 * int SPI0_Init(void){
	int Status;

	Status = SPIM0_Init(MODE_0, DIS_SEL, XSPIPS_CLK_PRESCALE_16, OFF_Start);

	return Status;
}

	SPIM0_Send(buf_tx, 2);
	SPIM0_TX(buf_tx, 2);
*/
/***************************** Include Files ********************************/
#include "SPI_PS.h"
#include "assert.h"

/************************** Variable Definitions ****************************/
/* Declare instance and associated pointer for ... */
static XSpiPs spi0Inst;
static XSpiPs spi1Inst;

static XSpiPs *p_spi0Inst = &spi0Inst;
static XSpiPs *p_spi1Inst = &spi1Inst;
/*Extern*/
extern XScuGic		XScuGicInst;
static XScuGic 		*p_XScuGicInst = &XScuGicInst;
/* SPI Receive Buffer */
static uint8_t spi_rx_buffer [RX_NBYTES];
/*---------------------------------------------------------------------------*/
/*------------------------------- FUNCTIONS ---------------------------------*/
/*---------------------------------------------------------------------------*/
/*****************************************************************************
 * Function: SPI_PS_Init()
 *//**
 *
 * @brief		Configures the SPI for use.
 *
 * @details This function configures the spi PS. It sets the SPI protocol, mode of
 *	operation, bit rate, and data width.
 * @param   The pointer to Configuration data
 * @param	SpiDeviceId is the Device ID of the Spi Device and is the
 *			XPAR_<SPI_instance>_DEVICE_ID value from xparameters.h.
 * @param 	SpiInstancePtr is a pointer to the instance of Spi component.
 * @param	The ui32Protocol parameter defines the data frame format.
 * @param	The ui32Mode parameter defines the operating mode of the SPI module.
 * @param   The Manual Slave Select option  - XSPIPS_FORCE_SSELECT_OPTION
 * @param	The Manual Statrt Enable option -  XSPIPS_MANUAL_START_OPTION.
 * @return	Integer indicating result of configuration attempt.
 * 				0 = SUCCESS, 1 = FAILURE
 * */

static int SPI_PS_Init(XSpiPs_Config *SpiConfig, u16 SpiDeviceId, XSpiPs *xspiInst, SPI_Protocol_t Protocol, SPI_Mode_t Mode,u8 ClkPrescale, SPI_Manual_SS_t Manual_CS, SPI_Manual_Start_t ManStart)
{
	int Status;
	u32 Options;
	/* === START CONFIGURATION SEQUENCE ===  */
	/* ---------------------------------------------------------------------
	 * ------------ DEVICE LOOK-UP ------------
	 * -------------------------------------------------------------------- */
	SpiConfig = XSpiPs_LookupConfig(SpiDeviceId);
	if (NULL == SpiConfig) {
		xil_printf("\033[43[SPI%d] Return config - NULL \r\n\033[0m", SpiDeviceId);
		return XST_FAILURE;
	}
	 /* ---------------------------------------------------------------------
	  * ------------ DRIVER INITIALISATION ------------
	  * -------------------------------------------------------------------- */
	Status = XSpiPs_CfgInitialize(xspiInst, SpiConfig,
					SpiConfig->BaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("\033[3;31;47m[SPI%d] Return initCFG SPI - FAILURE\n status - %2d\n\033[0m",SpiDeviceId, Status);
		return XST_FAILURE;
	}
	/* ---------------------------------------------------------------------
	* ------------  SELF TEST ------------
	* -------------------------------------------------------------------- */
	//Status = XSpiPs_SelfTest(xspiInst);
	if (Status != XST_SUCCESS) {
		xil_printf("\033[3;31;47m[SPI%d] Return Set Options SPI - FAILURE\n status - %2d\n\033[0m",SpiDeviceId, Status);
		return XST_FAILURE;
	}
	/* ---------------------------------------------------------------------
	* ------------ PROJECT-SPECIFIC CONFIGURATION ------------
	* -------------------------------------------------------------------- */
	Options = Protocol;
	Options |= Mode;
	if(Manual_CS!=DIS_SEL){
		Options |= XSPIPS_FORCE_SSELECT_OPTION;
	}
	if(ManStart!=OFF_Start){
		Options |= XSPIPS_MANUAL_START_OPTION;
	}
	/* Selects or deselect the slave with which the master communicates*/
    Status = XSpiPs_SetOptions(xspiInst, Options);
    if (Status != XST_SUCCESS) {
    	xil_printf("\033[3;31;47m[SPI%d] Return Set Options SPI - FAILURE\n status - %2d\n\033[0m",SpiDeviceId, Status);
        return Status;
    }

    /*Clock Config - Zynq-7000 IO Peripheral Clock - CLK SPI*/
    /* @param 	PS7_SPI_CLK this parameter is set in the Vivado block design. */
    int freq = PS7_SPI_CLK/ClkPrescale;
    xil_printf("\033[3;31;47m[SPI%d]PS7 SPI CLK - %dMHz\n\033[0m",SpiDeviceId, PS7_SPI_CLK/1000000U);
	Status = XSpiPs_SetClkPrescaler(xspiInst, ClkPrescale);
	xil_printf("\033[3;31;47m[SPI%d]SPI CLK - %dMHz\n\033[0m",SpiDeviceId, freq/1000000U);
	/*
	 * Set the Rx FIFO Threshold to the Max Data
	 */
	XSpiPs_SetRXWatermark(xspiInst,100);
	/*
	 * Enable the device.
	 */
	XSpiPs_Enable(xspiInst);
	return XST_SUCCESS;
}
/*****************************************************************************
 * Function: SPIM0_Init()
 *//**
 *
 * @brief		Configures the SPI_0 Master for use. *
 * @param   The Manual Slave Select option  - XSPIPS_FORCE_SSELECT_OPTION
 * @param	The Manual Statrt Enable option -  XSPIPS_MANUAL_START_OPTION.
 * @details 	This function configures the spi PS. It sets the SPI protocol, mode of
 *				operation, bit rate, and data width. *
 * @return	Integer indicating result of configuration attempt.
 * 				0 = SUCCESS, 1 = FAILURE
 * */
int SPIM0_Init(SPI_Protocol_t Protocol, SPI_Manual_SS_t Manual_CS, u8 ClkPrescale, SPI_Manual_Start_t ManStart){
	int Status;
	XSpiPs_Config *Spi_0_Config= NULL;

	Xil_AssertNonvoid(Protocol  == MODE0||MODE1||MODE2||MODE3);
	Xil_AssertNonvoid(Manual_CS == EN_SEL||DIS_SEL);
	Xil_AssertNonvoid(ManStart  == ON_Start||OFF_Start);
	Xil_AssertNonvoid(ClkPrescale	== XSPIPS_CLK_PRESCALE_4||XSPIPS_CLK_PRESCALE_8||XSPIPS_CLK_PRESCALE_16
									 ||XSPIPS_CLK_PRESCALE_32||XSPIPS_CLK_PRESCALE_64||XSPIPS_CLK_PRESCALE_128
									 ||XSPIPS_CLK_PRESCALE_256);

	Status = SPI_PS_Init(Spi_0_Config, SPI0_DEV_ID, p_spi0Inst, Protocol, ClkPrescale, MASTERps, Manual_CS, ManStart);
	if(Status!= XST_SUCCESS){
		xil_printf("\033[3;31;47m Error SPI Init! return %d2\n\r\033[0m", Status);
		return XST_FAILURE;
	}
	xil_printf("\033[3;35mSPI0-Master Init successful\n\r\033[0m");
	return Status;
}
/*****************************************************************************
 * Function: SPIS0_Init()
 *//**
 *
 * @brief		Configures the SPI_0 Slave for use. *
 * @details 	This function configures the spi PS. It sets the SPI protocol, mode of
 *				operation, bit rate, and data width.
 * @return		0 = SUCCESS, 1 = FAILURE
 * */
int SPIS0_Init(SPI_Protocol_t Protocol, u8 ClkPrescale){
	int Status;
	XSpiPs_Config *Spi_0_Config = NULL;

	Xil_AssertNonvoid(Protocol  == MODE0||MODE1||MODE2||MODE3);
	Xil_AssertNonvoid(ClkPrescale	== XSPIPS_CLK_PRESCALE_4||XSPIPS_CLK_PRESCALE_8||XSPIPS_CLK_PRESCALE_16
									 ||XSPIPS_CLK_PRESCALE_32||XSPIPS_CLK_PRESCALE_64||XSPIPS_CLK_PRESCALE_128
									 ||XSPIPS_CLK_PRESCALE_256);

	Status = SPI_PS_Init(Spi_0_Config, SPI0_DEV_ID, p_spi0Inst, Protocol, ClkPrescale, SLAVEps, DIS_SEL, OFF_Start);
		if(Status!= XST_SUCCESS){
			xil_printf("\033[3;31;47m Error SPI Master Init! return %d2\n\r\033[0m", Status);
			return XST_FAILURE;
		}
	xil_printf("\033[3;35mSPI0-Slave Init successful\n\r\033[0m");
	return Status;
}

/*****************************************************************************
 * Function: SPIM0_Send()
 *//**
 *
 * @brief		See description SpiPs_TX
 * @details
 *
 * @return	    None.
 * */
void SPIM0_Send(u8 *SendBuffer, int ByteCount){

	SpiPs_TX(spi0Inst, SendBuffer, ByteCount);
}
/*****************************************************************************
 * Function: SPIM0_TX()
 *//**
 *
 * @brief		Transfers specified data on the SPI bus in polled mode.
 * @details     The function controls CS.
 * @return	    0 = SUCCESS, 1 = FAILURE.
 *
 * */
int SPIM0_TX(u8 *SendBuffer, int ByteCount){
	int Status;
	XSpiPs_SetSlaveSelect(p_spi0Inst, 0U);

	Status = XSpiPs_PolledTransfer(p_spi0Inst, SendBuffer, NULL, ByteCount);
   if (Status != XST_SUCCESS) {
		xil_printf("\033[3;31;47m[SPIM0_TX%d] Return - FAILURE, Status -%d \n\033[0m",Status);
		return XST_FAILURE;
	}
   return XST_SUCCESS;
}
/*****************************************************************************
 * Function: SPIM1_Init()
 *//**
 *
 * @brief		Configures the SPI_1 Master for use. *
 * @details 	This function configures the spi PS. It sets the SPI protocol, mode of
 *				operation, bit rate, and data width.
 * 				0 = SUCCESS, 1 = FAILURE
 * */
int SPIM1_Init(SPI_Protocol_t Protocol, SPI_Manual_SS_t Manual_CS, u8 ClkPrescale, SPI_Manual_Start_t ManStart){
	int Status;
	XSpiPs_Config *Spi_1_Config = NULL;

	Xil_AssertNonvoid(Protocol  == MODE0||MODE1||MODE2||MODE3);
	Xil_AssertNonvoid(Manual_CS == EN_SEL||DIS_SEL);
	Xil_AssertNonvoid(ManStart  == ON_Start||OFF_Start);
	Xil_AssertNonvoid(ClkPrescale	== XSPIPS_CLK_PRESCALE_4||XSPIPS_CLK_PRESCALE_8||XSPIPS_CLK_PRESCALE_16
									 ||XSPIPS_CLK_PRESCALE_32||XSPIPS_CLK_PRESCALE_64||XSPIPS_CLK_PRESCALE_128
									 ||XSPIPS_CLK_PRESCALE_256);
	Status = SPI_PS_Init(Spi_1_Config, SPI1_DEV_ID, p_spi1Inst,Protocol, ClkPrescale, MASTERps, Manual_CS, ManStart);
	if(Status!= XST_SUCCESS){
		xil_printf("\033[3;31;47m Error SPI1 Master Init! return %d2\n\r\033[0m", Status);
		return XST_FAILURE;
	}
	xil_printf("\033[3;35mSPI1-Master Init successful\n\r\033[0m");
	return Status;
}
/*****************************************************************************
 * Function: SPIS1_Init()
 *//**
 *
 * @brief		Configures the SPI_1 Slave for use. *
 * @details 	This function configures the spi PS. It sets the SPI protocol, mode of
 *				operation, bit rate, and data width.
 * 				0 = SUCCESS, 1 = FAILURE
 * */
int SPIS1_Init(SPI_Protocol_t Protocol, u8 ClkPrescale){
	int Status;
	XSpiPs_Config *Spi_1_Config = NULL;

	Xil_AssertNonvoid(Protocol  	== MODE0||MODE1||MODE2||MODE3);
	Xil_AssertNonvoid(ClkPrescale	== XSPIPS_CLK_PRESCALE_4||XSPIPS_CLK_PRESCALE_8||XSPIPS_CLK_PRESCALE_16
									 ||XSPIPS_CLK_PRESCALE_32||XSPIPS_CLK_PRESCALE_64||XSPIPS_CLK_PRESCALE_128
									 ||XSPIPS_CLK_PRESCALE_256);

	Status = SPI_PS_Init(Spi_1_Config, SPI1_DEV_ID, p_spi1Inst,Protocol, ClkPrescale, SLAVEps, DIS_SEL, OFF_Start);
	if(Status!= XST_SUCCESS){
		xil_printf("\033[3;31;47m Error SPI1 Slave Init! return %d2\n\r\033[0m", Status);
		return XST_FAILURE;
	}
	xil_printf("\033[3;35mSPI1-Slave Init successful\n\r\033[0m");
	return Status;
}

/*****************************************************************************
 * Function: SpiPs_RX()
 *//**
 *
 * @brief		Used to read the specified byte of data into the buff. *
 * @details 	Read firstXSPIPS_SR_OFFSET Register to determine whether the receiving RX FIFO is empty.
 * 				Read the data in the RX FIFO when it is not empty.
 * 				Note the following XSPIPS_RXD_OFFSETOnly one Byte of the register is valid.
 *
 * @return	    None.
 * */
static void SpiPs_RX(XSpiPs SpiInstance, u8 *ReadBuffer,int ByteCount)
{
	int Count;
	u32 StatusReg;

	do{
		StatusReg = XSpiPs_ReadReg(SpiInstance.Config.BaseAddress,
					XSPIPS_SR_OFFSET);
	}while(!(StatusReg & XSPIPS_IXR_RXNEMPTY_MASK));

	/*
	 * Reading the Rx Buffer
	 */
	for(Count = 0; Count < ByteCount; Count++){
		ReadBuffer[Count] = SpiPs_RecvByte(SpiInstance.Config.BaseAddress);
	}
}

/*****************************************************************************
 * Function: SpiPs_TX()
 *//**
 *
 * @brief		Used to send the data specified in the buff.
 * @details 	When the TX FIFO is not full, fill the FIFO as much as possible.
 * 				The following Milian logic feels that there should be a problem, and should not be judged by the size of the FIFO,
 * 					the register should be read XSPIPS_SR_OFFSET When the TX FIFO is not full, write as much as possible.
 * 				But here is a loop test with a small piece of data, so I wonít go into details here.
 *
 * @return	    None.
 * */
static void SpiPs_TX(XSpiPs SpiInstance, u8 *SendBuffer, int ByteCount)
{
	uint8_t * dummi= 0;
	u32 StatusReg;
	int TransCount = 0;

	while ((ByteCount > 0) &&
		(TransCount < XSPIPS_FIFO_DEPTH)) {
		SpiPs_SendByte(SpiInstance.Config.BaseAddress,
				*SendBuffer);
		SendBuffer++;
		++TransCount;
		ByteCount--;
	}
/*
 * Wait for the transfer to finish by polling Tx fifo status.
 */
	do {
		StatusReg = XSpiPs_ReadReg(
				SpiInstance.Config.BaseAddress,
					XSPIPS_SR_OFFSET);
	} while ((StatusReg & XSPIPS_IXR_TXOW_MASK) == 0);
}

/*****************************************************************************
 * Function: SPI_RxFifoIsNotEmpty()
 *//**
 *
 * @brief		The function checks the fifo buffer
 * @return	    None.
 * */
static bool SPI_RxFifoIsNotEmpty(XSpiPs SpiInstance) {

    uint32_t status = XSpiPs_ReadReg(SpiInstance.Config.BaseAddress, XSPIPS_SR_OFFSET);

    return ( !( (status & XSPIPS_IXR_RXNEMPTY_MASK) == 0 ) );
}

/*****************************************************************************
 * Function: SPI_TxFifoIsNotEmpty()
 *//**
 *
 * @brief		The function checks the fifo buffer
 * @return	    None.
 * */
static bool SPI_TxFifoIsNotEmpty(XSpiPs SpiInstance) {

    uint32_t status = XSpiPs_ReadReg(SpiInstance.Config.BaseAddress, XSPIPS_SR_OFFSET);

    return ( !( (status & XSPIPS_IXR_TXFULL_MASK) == 0 ) );
}

/*****************************************************************************
 * Function: addSpi0ToInterruptSystem()
 *//**
 *
 * @brief
 *
 * @details		Connects the SPI‡Û44534‚Ûˆ5ÂÍ34Í54 to the interrupt system.
 * 				Carries out the following steps:
 *
 * 				XScuGic_Connect(): Connect the device driver handler for TTC0
 * 				XScuGic_SetPriorityTriggerType():
 *
 * 				XScuGic_Enable(): Enables the interrupt for the TTC0.
 *
 * 				If XScuGic_Connect() is not successful, the routine ends
 * 				immediately	and returns XST_FAILURE.
 *
 *
 * @param[in]	Pointer to the TTC0 Instance
 *
 * @return		Returns result of configuration attempt.
 * 				0L = SUCCESS, 1L = FAILURE
 *
 * @note		The SCUGIC and TTC0 must be initialised before calling
 * 				this function.
 *
****************************************************************************/

int addSpi0ToInterruptSystem(uint32_t p_SpiInst)
{	int status;
/*
 * Setup the handler for the SPI that will be called from the
 * interrupt context when an SPI status occurs, specify a pointer to
 * the SPI driver instance as the callback reference so the handler is
 * able to access the instance data
 */
XSpiPs_SetStatusHandler(p_spi0Inst, p_spi0Inst,
			 (XSpiPs_StatusHandler) SpiPsHandler);
	// Connect a device driver handler for the SPI0 Timer
	status = XScuGic_Connect(p_XScuGicInst, SPI0_INTR_ID,
				  (Xil_ExceptionHandler) SpiPsHandler,
				  (void *) p_spi0Inst);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	/* Set priority and trigger */
//	XScuGic_SetPriorityTriggerType(p_XScuGicInst, .., ..., ...);
/* Enable the interrupt for SPI0 */
	XScuGic_Enable(p_XScuGicInst, SPI0_INTR_ID);
/* Return initialisation result to calling code */
	return status;
}

int addSpi1ToInterruptSystem(uint32_t p_SpiInst)
{	int status;
/*
 * Setup the handler for the SPI that will be called from the
 * interrupt context when an SPI status occurs, specify a pointer to
 * the SPI driver instance as the callback reference so the handler is
 * able to access the instance data
 */
XSpiPs_SetStatusHandler(p_spi1Inst, p_spi1Inst,
			 (XSpiPs_StatusHandler) SpiPsHandler);
	// Connect a device driver handler for the SPI0 Timer
	status = XScuGic_Connect(p_XScuGicInst, SPI1_INTR_ID,
				  (Xil_ExceptionHandler) SpiPsHandler,
				  (void *) p_spi1Inst);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	/* Set priority and trigger */
//	XScuGic_SetPriorityTriggerType(p_XScuGicInst, .., ..., ...);
/* Enable the interrupt for SPI1 */
	XScuGic_Enable(p_XScuGicInst, SPI1_INTR_ID);
/* Return initialisation result to calling code */
	return status;
}

/*****************************************************************************/
/**
*
* This function disables the interrupts that occur for the Spi device.
*
* @param	IntcInstancePtr is the pointer to an INTC instance.
* @param	SpiIntrId is the interrupt Id for an SPI device.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void SpiPsDisableIntrSystem(XScuGic *IntcInstancePtr, u16 SpiIntrId)
{
	/*
	 * Disable the interrupt for the SPI device.
	 */
	XScuGic_Disable(IntcInstancePtr, SpiIntrId);

	/*
	 * Disconnect and disable the interrupt for the Spi device.
	 */
	XScuGic_Disconnect(IntcInstancePtr, SpiIntrId);
}


void SpiPsHandler(void *CallBackRef, u32 StatusEvent, unsigned int ByteCount)
{
	/*
	 * Indicate the transfer on the SPI bus is no longer in progress
	 * regardless of the status event
	 */


	/*
	 * If the event was not transfer done, then track it as an error
	 */
	if (StatusEvent != XST_SPI_TRANSFER_DONE) {

	}
}

void SPI_Write(uint16_t dev, uint16_t addr, uint8_t data) {

/*	uint8_t * dummi;
	uint8_t send_data[3];

    if ( (dev == Device1) || (dev == Device2) ) {


        send_data[0] = SPI_FLAG_WRITE | ( (addr >> 8) & 0x3f );
        send_data[1] = addr & 0xff;
        send_data[2] = data;

        if (dev == Device1) {
            XSpiPs_SetSlaveSelect(&spi1, Device1_CS);
        } else {
            XSpiPs_SetSlaveSelect(&spi1, Device2_CS);
        }
        XSpiPs_PolledTransfer(&spi1, send_data, &dummi, sizeof(send_data));
    }
 */
}
/****** End functions *****/

/****** End of File **********************************************************/
