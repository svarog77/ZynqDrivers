/*
 * SCU_TIMER.h
 *
 *  Created on: 4 èþë. 2024 ã.
 *      Author: Viacheslav
 */

#ifndef ZYNQDIVERSPS_TIMERS_SCU_TIMER_SCU_TIMER_H_
#define ZYNQDIVERSPS_TIMERS_SCU_TIMER_SCU_TIMER_H_
/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "xparameters.h"
#include "xscutimer.h"

#include "xil_exception.h"
#include "xil_printf.h"
#include "xil_assert.h"


#include "../../GIC/GIC.h"
/*****************************************************************************/
/************************** Constant Definitions *****************************/
/*****************************************************************************/
#define SCU_TIMER_DEVICE_ID         XPAR_PS7_SCUTIMER_0_DEVICE_ID
/* Interrupt IDs (from "xparameters_ps.h") */
#define SCU_TIMER_INTR_ID			XPS_SCU_TMR_INT_ID // SCU Private Timer interrupt, 29U
#define SCU_TIMER_LOAD_VALUE        (3330) /* 10 us */
/* SCU PRIVATE TIMER */
#define SCU_TIMER_INTR_PRI			(0xA0)
#define SCU_TIMER_INTR_TRIG			(0x03)
/*****************************************************************************/
/************************** Variable Declarations ****************************/
/*****************************************************************************/




/*****************************************************************************/
/************************ Macros (Inline Functions) **************************/
/*****************************************************************************/



/*****************************************************************************/
/************************** Function Prototypes ******************************/
/*****************************************************************************/
int SCU_TimerInit						(void);
// Interface functions
static void startScuTimer				(void);
static void stopScuTimer				(void);
void   		waitScuTimerExpired			(void);
int 		addScuTimerToInterruptSystem(void);
void 		SCU_Timer_Start				(void);
void 		SCU_Timer_Stop				(void);
void 		SCU_TimerInterruptEn		(void);

void 		ScuTimer_IntrHandler		(void *callback_ref);
#endif /* ZYNQDIVERSPS_TIMERS_SCU_TIMER_SCU_TIMER_H_ */
