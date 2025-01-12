/*
 * SCU_TIMER.c
 *
 *  Created on: 4 èþë. 2024 ã.
 *      Author: Viacheslav
 */

/***************************** Include Files ********************************/
#include "SCU_TIMER.h"
/************************** Variable Definitions ****************************/
/* Declare instance and associated pointer for ... */
static XScuTimer ScuTimerInst;
static XScuTimer *p_ScuTimerInst = &ScuTimerInst;

extern XScuGic		XScuGicInst;
static XScuGic 		*p_XScuGicInst = &XScuGicInst;
/*---------------------------------------------------------------------------*/
/*------------------------------- FUNCTIONS ---------------------------------*/
/*---------------------------------------------------------------------------*/

/*Function:SCU_TimerInit
 * *//**
 *
 * @brief		Configures the SCU Timer for use.
 * @details		Starts by doing device look-up, configuration and self-test.
 * 				Then configures the	SCU Timer.
 * @return		Integer indicating result of configuration attempt.
 * 				0 = SUCCESS, 1 = FAILURE
 * */
int SCU_TimerInit(void){
	int Status;
	XScuTimer_Config *TimerConfig = NULL;
	/* === START CONFIGURATION  ===  */
	/* ---------------------------------------------------------------------
	 * ------------  DEVICE LOOK-UP ------------
	 * -------------------------------------------------------------------- */
	TimerConfig = XScuTimer_LookupConfig(SCU_TIMER_DEVICE_ID);
	 /* ---------------------------------------------------------------------
	  * ------------  DRIVER INITIALISATION ------------
	  * -------------------------------------------------------------------- */
	Status = XScuTimer_CfgInitialize(p_ScuTimerInst, TimerConfig, TimerConfig->BaseAddr);
	if (Status != XST_SUCCESS)
	{		xil_printf("\033[3;31;47m[SCU]SCU Timer FAILURE\n status - %2d\n\033[0m", Status);
			return XST_FAILURE;
	}
	/* ---------------------------------------------------------------------
	* ------------  SELF TEST ------------
	* -------------------------------------------------------------------- */
    Status = XScuTimer_SelfTest(p_ScuTimerInst);
    if (Status != XST_SUCCESS) {
    	xil_printf("\033[3;31;47m[SCU]SCU Timer Self Test FAILURE\n status - %2d\n\033[0m", Status);
          return XST_FAILURE;
    }
//InterruptSetup
    Status = addScuTimerToInterruptSystem();
    if (Status != XST_SUCCESS) {
    	xil_printf("\033[3;31;47m[SCU]SCU Timer Connect to IntSys FAILURE\n status - %2d\n\033[0m", Status);
          return XST_FAILURE;
    }
	/* ---------------------------------------------------------------------
	* ------------  PROJECT-SPECIFIC CONFIGURATION ------------
	* -------------------------------------------------------------------- */
    XScuTimer_EnableInterrupt(p_ScuTimerInst);
    XScuTimer_EnableAutoReload(p_ScuTimerInst);
    XScuTimer_LoadTimer(p_ScuTimerInst, SCU_TIMER_LOAD_VALUE);

	xil_printf("\033[3;35m[SYNCH :SCU]Scu Timer-Init successful\n\r\ \033[0m");
	return Status;
}
/*This function is used to check whether or not the SCU Timer has expired
 */
void waitScuTimerExpired(void)
{
 uint32_t scu_timer_expired;
 // Extract the base address from the instance pointer
 uint32_t BASEADDR = p_ScuTimerInst->Config.BaseAddr;
 // 'Spin' in this loop while waiting for the count reach 0
	   do{
		scu_timer_expired = XScuTimer_GetIntrReg(BASEADDR) &
									   XSCUTIMER_ISR_EVENT_FLAG_MASK;
	  } while (scu_timer_expired == 0);
	   // Clear the 'count = 0' bit in the Private Timer Interrupt Register
	   XScuTimer_SetIntrReg(BASEADDR, XSCUTIMER_ISR_EVENT_FLAG_MASK);// (setting the bit clears it), and call the Xilinx restart function.
	   XScuTimer_RestartTimer(p_ScuTimerInst);
}
/*****************************************************************************
 * Function: startScuTimer()
 *//**
 *
 * @brief		Calls the XScuTimer function "XScuTimer_Start".
 *
 * @param		None.
 *
 * @return		None.
 *
 * @note		None
 *
******************************************************************************/

static void startScuTimer(void)
{
	XScuTimer_Start(p_ScuTimerInst);
}

static void stopScuTimer(void){
	XScuTimer_Stop(p_ScuTimerInst);
}

void SCU_Timer_Start(void){
	startScuTimer();
}

void SCU_Timer_Stop(void){
	XScuTimer_Stop(p_ScuTimerInst);
}

int addScuTimerToInterruptSystem(void){
	int Status;
	/* Connect a device driver handler for the XScuTimer */
	Status = XScuGic_Connect(p_XScuGicInst,
							SCU_TIMER_INTR_ID,
							(Xil_ExceptionHandler) ScuTimer_IntrHandler,
							(void *) p_ScuTimerInst);
	if (Status != XST_SUCCESS)
	{	return XST_FAILURE;}
	/* Set priority and trigger type */
	XScuGic_SetPriorityTriggerType(p_XScuGicInst, SCU_TIMER_INTR_ID,
									SCU_TIMER_INTR_PRI, SCU_TIMER_INTR_TRIG);
	/* Enable the interrupt for SCU Timer */
	XScuGic_Enable(p_XScuGicInst, SCU_TIMER_INTR_ID);
	/* Return initialisation result to calling code */
	return Status;
}

void SCU_TimerInterruptEn(void){
	   startScuTimer();
}

void ScuTimer_IntrHandler(void *callback_ref) {
	XScuTimer *tmr = (XScuTimer *)callback_ref;
	/*Action*/

	/*...*/
    XScuTimer_ClearInterruptStatus(tmr);
}
/****** End functions *****/

/****** End of File **********************************************************/

