/*
 * GIC.c
 *
 *  Created on: 4 èþë. 2024 ã.
 *      Author: Viacheslav
 */
/*****************************************************************************/
/************************** Include Files ****************************/
/*****************************************************************************/
#include "GIC.h"

/* Declare instance and associated pointer for XScuGic */
		XScuGic		XScuGicInst;
static  XScuGic 	*p_XScuGicInst = &XScuGicInst;

/*---------------------------------------------------------------------------*/
/*------------------------------- FUNCTIONS ---------------------------------*/
/*---------------------------------------------------------------------------*/
/* Function: xScuGic_Init()
 *//**
 *
 * @brief		Configures the Global Interface Controller (GIC) for use.
 *
 *
 * @details		Starts by doing device look-up, configuration and self-test.
 * 				Then configures the	SCUGIC.
 *
 * 				The initialisation steps are:
 * 				(1) DEVICE LOOK-UP => Calls function "XScuGic_LookupConfig"
 * 				(2) DRIVER INIT => Calls function "XScuGic_CfgInitialize"
 * 				(3) SELF TEST => Calls function "XScuGic_SelfTest"
 * 				(4) SPECIFIC CONFIG => Configures SCUGIC for this project
 *
 * 				If any of the first three states results in XST_FAILURE, the
 * 				initialisation will stop and the XST_FAILURE code will be
 * 				returned to the calling code. If initialisation completes with
 * 				no failures, then XST_SUCCESS is returned.
 *
 * @return		Integer indicating result of configuration attempt.
 * 				0 = SUCCESS, 1 = FAILURE
 *
 * @note		None
 *
******************************************************************************/
int SCU_GicInit(void){

	int status;
	/* Pointer to XScuGic_Config is required for later functions. */
	XScuGic_Config *p_XScuGicCfg = NULL;
	/* === START CONFIGURATION  ===  */
	/* ---------------------------------------------------------------------
	 * ------------ DEVICE LOOK-UP ------------
	 * -------------------------------------------------------------------- */
	p_XScuGicCfg = XScuGic_LookupConfig(PS7_SCUGIC_DEVICE_ID);
	if (p_XScuGicCfg == NULL)
	{
		xil_printf("\033[3;31;47m Error Gic Return config - NULL!  return status - %2d\n\r\033[0m", status);
		return status;
	}
	/* ---------------------------------------------------------------------
	 * ------------ DRIVER INITIALISATION ------------
	 * -------------------------------------------------------------------- */
	status = XScuGic_CfgInitialize(p_XScuGicInst, p_XScuGicCfg, p_XScuGicCfg->CpuBaseAddress);
	if (status != XST_SUCCESS)
	{		xil_printf("\033[3;31;47m[GIC] Return initCFG GIC - FAILURE\n status - %2d\n\033[0m", status);
		return status;	}
	/* ---------------------------------------------------------------------
	* ------------ SELF TEST ------------
	* -------------------------------------------------------------------- */
	status = XScuGic_SelfTest(p_XScuGicInst);
 	//Xil_AssertNonvoid(status == XST_SUCCESS);
	if (status != XST_SUCCESS) {
		xil_printf("\033[3;31;47m[GIC] Return Self Test GIC - FAILURE\n status - %2d\n\033[0m", status);
		return XST_FAILURE;
	}
 	/* If the assertion test fails, we won't get here, but
 	 * leave the code in anyway, for possible future changes. */
 	if (status != XST_SUCCESS)
 	{		xil_printf("\033[3;31;47m[GIC] Return Set Options GIC - FAILURE\n status - %2d\n\033[0m", status);
		 return status;
 	}
	/* ---------------------------------------------------------------------
	* ------------  PROJECT-SPECIFIC CONFIGURATION ------------
	* -------------------------------------------------------------------- */
 	/* Binary point register modified to allow nested interrupts to work.
 	 * This needs to be done before calling Xil_ExceptionInit(). */
 	XScuGic_CPUWriteReg(p_XScuGicInst, XSCUGIC_BIN_PT_OFFSET, 0x03);
 	/* Initialise exception logic */
 	Xil_ExceptionInit();
	/*
	* Connect the interrupt controller interrupt handler to the
	* hardware interrupt handling logic in the processor.
	*/
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
								(Xil_ExceptionHandler) XScuGic_InterruptHandler,
								p_XScuGicInst);
	/* === END CONFIGURATION ===  */
	/* Return initialisation result to calling code */




	return status;

}
/*
 **/

/*****************************************************************************
 * Function:	disableInterrupts()
 *//**
*
* @brief		Calls the Xilinx function "Xil_ExceptionDisable" to disable
* 				interrupts.
*
* @return		None.
*
* @notes:		None.
*
****************************************************************************/
void disableInterrupts(void){
	Xil_ExceptionDisable();
}

/*****************************************************************************
 * Function:	enableInterrupts()
 *//**
*
* @brief		Calls the Xilinx function "Xil_ExceptionEnable" to enable
* 				interrupts.
*
* @return		None.
*
* @notes:		None.
*
****************************************************************************/
void enableInterrupts(void){
	Xil_ExceptionEnable();
}

/****** End functions *****/

/****** End of File **********************************************************/
