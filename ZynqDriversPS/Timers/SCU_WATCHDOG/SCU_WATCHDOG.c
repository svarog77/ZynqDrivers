/*
 * SCU_WATCHDOG.c
 *
 *  Created on: 4 èþë. 2024 ã.
 *      Author: Viacheslav
 */
/***************************** Include Files ********************************/
#include "SCU_WATCHDOG.h"

/*******************************************
 * Variable Declarations
 ******************************************/
/* Declare instance and associated pointer for ... */
static XScuWdt XScWdtInst;
static XScuWdt *p_XScuWdtInst = &XScWdtInst;

/*---------------------------------------------------------------------------*/
/*------------------------------- FUNCTIONS ---------------------------------*/
/*---------------------------------------------------------------------------*/
/*****************************************************************************
 * Function: SCU_WdtInit()
 *//**
 *
 * @brief		Configures the SCUWDT for use.
 *
 *
 * @details		Starts by doing device look-up, configuration and self-test.
 * 				Then configures the	SCU WDT.
 *
 * 				The initialisation steps are:
 * 				(1) DEVICE LOOK-UP => Calls function "XScuWdt_LookupConfig"
 * 				(2) DRIVER INIT => Calls function "XScuWdt_CfgInitialize"
 * 				(3) SELF TEST => Calls function "XScuWdt_SelfTest"
 * 				(4) SPECIFIC CONFIG => Configures SCU Timer for this project
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
int SCU_WdtInit(void){
	int Status;
	XScuWdt_Config *WdtConfig;
#if SCUWDT_DEBUG
checkRebootStatus();
#endif
	WdtConfig = XScuWdt_LookupConfig(SCUWDT_DEVICE_ID);
	XScuWdt_CfgInitialize(p_XScuWdtInst, WdtConfig, WdtConfig->BaseAddr);
	Status = XScuWdt_SelfTest(p_XScuWdtInst);

    if (Status != XST_SUCCESS) {
          return XST_FAILURE;
    }

    // Load the watchdog counter register, and start it.
    XScuWdt_LoadWdt(p_XScuWdtInst, SCUWDT_LOAD_VALUE);
    XScuWdt_Start(p_XScuWdtInst);

    return Status;
}

/*****************************************************************************
 * Function: restartScuWdt()
 *//**
 *
 * @brief		Calls the XScuWdt function "restartScuWdt" to 'kick' the
* 				SCU watchdog timer.
 *
 * @return		None.
 *
 * @note		None.
 *
******************************************************************************/
void restartScuWdt(void){
	XScuWdt_RestartWdt(p_XScuWdtInst);
}
/*****************************************************************************
 * Function: checkRebootStatus()
 *//**
 *
 * @brief		Reads the SLCR Reboot status register and prints out the status
 * 				of possible reboot causes.
 *
 * @return		None.
 *
 * @note		This function is only really meant for debug/informative
 * 				reasons. A terminal must be connected.
 *
******************************************************************************/
void checkRebootStatus(void)
{
	uint32_t slcr_reboot_sts = Xil_In32(SLCR_REBOOT_STATUS_REG);
}
/****** End functions *****/

/****** End of File **********************************************************/
