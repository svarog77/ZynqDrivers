#ifndef ZYNQDIVERSPS_TIMERS_SCU_WATCHDOG_SCU_TIMER_H_
#define ZYNQDIVERSPS_TIMERS_SCU_WATCHDOG_SCU_TIMER_H_
/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "xparameters.h"
#include "xscuwdt.h"

#include "xil_printf.h"
#include "xil_assert.h"
/*****************************************************************************/
/************************** Constant Definitions *****************************/
/*****************************************************************************/
#define SCUWDT_DEVICE_ID     XPAR_PS7_SCUWDT_0_DEVICE_ID
#define SCUWDT_LOAD_VALUE    (0x7FFFFFF) // 6.44s @ 111MHz

#define SCUWDT_DEBUG  1
#define SLCR_REBOOT_STATUS_REG  0xF8000258U
#define SLCR_RS_SWDT_RST_MASK   0x00010000U
#define SLCR_RS_AWDT0_RST_MASK  0x00020000U
/*****************************************************************************/
/************************** Variable Declarations ****************************/
/*****************************************************************************/




/*****************************************************************************/
/************************ Macros (Inline Functions) **************************/
/*****************************************************************************/



/*****************************************************************************/
/************************** Function Prototypes ******************************/
/*****************************************************************************/
int  SCU_WdtInit		(void);
void restartScuWdt		(void);
void checkRebootStatus	(void);
#endif
