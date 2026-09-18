/*

* app_main.c
*
* Zitto_MB_V1 / S32K144
*
* Application integration layer.
*
* Architecture:
*
* UART Packet
*
    |

*
    v

* App_UartPacketRx()
*
    |

*
    +--> CMD_GPIO_SET     -> Gpio_ControlProcessCommand()

*
    +--> CMD_STATUS_REQ   -> App_SendStatus()

*
    +--> other commands   -> Cmd_RxPacket()

*
* App_Task()
*
    |

*
    +--> UART Task

*
    +--> CAN1 Task

*
    +--> CAN2 Task

*
    +--> GPIO Task

*
    +--> IMU Task

*
    +--> OTA Task

*
* IMPORTANT:
* * GPIO hardware control exists only in gpio_control.c.
* * No gpio_apply() implementation exists here.
* * uart_pkt.h is the owner of UART protocol command IDs.
    */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "app_main.h"
#include "app_config.h"

#include "../UART/uart_pkt.h"

#include "../CMD/cmd.h"

#include "../CAN/can1.h"
#include "../CAN/can2.h"

#include "../GPIO/gpio_control.h"

#include "../IMU/imu.h"

#include "../OTA/ota.h"

#include "../FLM/flm.h"
#include "app_status.h"

#include "device_registers.h"
#include "APP/app_modules.h"
#include "DEBUG/debug_rtt.h"

/* =========================================================================

* LOCAL DEFINITIONS
* ========================================================================= */

#ifndef APP_STATUS_PERIOD_MS
#define APP_STATUS_PERIOD_MS        1000U
#endif

#ifndef APP_HEARTBEAT_PERIOD_MS
#define APP_HEARTBEAT_PERIOD_MS     1000U
#endif

/* =========================================================================

* LOCAL STATE
* ========================================================================= */

static uint32_t s_ms = 0U;

static uint32_t s_last_status_ms = 0U;
static uint32_t s_last_heartbeat_ms = 0U;

static bool s_initialized = false;

/* =========================================================================

* LOCAL FORWARD DECLARATIONS
* ========================================================================= */

static void App_UartPacketRx(const UartPkt_t *pkt);

static void App_SendStatus(void);

static void App_SendHeartbeat(void);
void App_GetStatus(StatusPkt_t *status)
{
    if(status == NULL)
    {
        return;
    }

    status->imu_en = 0U;
    status->csa_en = 0U;
    status->can1_en = 0U;
    status->can2_en = 0U;
    status->flm_en = 0U;

    status->ota_pending = 0U;

    status->reserved0 = 0U;
    status->reserved1 = 0U;

    status->can1_baud_kbps = 0U;
    status->can2_baud_kbps = 0U;

    status->flash_free_pages = 0U;

    status->uptime_ms = 0U;
    status->reset_cause = 0U;

    status->heartbeat_count = 0U;
}
static void App_ProcessPacket(const UartPkt_t *pkt);


/* =========================================================================

* UART RECEIVE CALLBACK
* ========================================================================= */

/*

* All validated UART packets enter here.
*
* uart_pkt.c performs:
*
* * frame synchronization
* * packet length validation
* * checksum / CRC validation
*
* app_main.c performs application-level routing.
  */

static void App_UartPacketRx(const UartPkt_t *pkt)
{
if(pkt == NULL)
{
return;
}


App_ProcessPacket(pkt);


}

/* =========================================================================

* PACKET DISPATCH
* ========================================================================= */

static void App_ProcessPacket(const UartPkt_t *pkt)
{
if(pkt == NULL)
{
return;
}
switch(pkt->type)
{
    /* -------------------------------------------------------------
     * GPIO CONTROL
     *
     * Payload ownership:
     *
     * uart_pkt.h      -> wire format
     * gpio_control.c  -> GPIO validation + hardware access
     * app_main.c      -> routing only
     * ------------------------------------------------------------- */

    case CMD_GPIO_SET:

        /*
         * GPIO module receives the complete packet payload.
         *
         * gpio_control.c is responsible for validating:
         *
         *   - payload length
         *   - GPIO index
         *   - direction
         *   - requested state
         *
         * No GPIO register access is performed here.
         */

        (void)Gpio_ControlProcessCommand(
            pkt->data,
            pkt->len
        );

        break;


    /* -------------------------------------------------------------
     * STATUS REQUEST
     * ------------------------------------------------------------- */

    case CMD_STATUS_REQ:

        App_SendStatus();

        break;


    /* -------------------------------------------------------------
     * ALL OTHER COMMANDS
     *
     * CMD module owns:
     *
     *   Module Enable
     *   MCU Reset
     *   LED Control
     *   Flash Read
     *   Flash Write
     *   Flash Delete
     *   OTA commands
     *
     * GPIO is intentionally handled above.
     * ------------------------------------------------------------- */

    default:

        Cmd_RxPacket(pkt);

        break;
}


}

/* =========================================================================

* STATUS PACKET
* ========================================================================= */

static void App_SendStatus(void)
{
StatusPkt_t status;


memset(&status, 0, sizeof(status));

/*
 * StatusPkt_t is defined in uart_pkt.h.
 *
 * Keep this function independent from GPIO implementation details.
 *
 * GPIO status changes are sent by:
 *
 *     Gpio_ControlSendStatus()
 *
 * when appropriate.
 *
 * The general system status packet is sent here.
 */

(void)Uart_Pkt_SendStatus(&status);


}

/* =========================================================================

* HEARTBEAT
* ========================================================================= */

static void App_SendHeartbeat(void)
{
/*
* uart_pkt.h currently exposes Uart_Pkt_SendHb().
*
* If your uart_pkt.h prototype takes an argument, adjust ONLY this
* call to match the exact prototype.
*/


(void)Uart_Pkt_SendHb();


}

/* =========================================================================

* APPLICATION INITIALIZATION
* ========================================================================= */

void App_Init(void)
{
if(s_initialized == true)
{
return;
}


/* -------------------------------------------------------------
 * Reset local application timing
 * ------------------------------------------------------------- */

s_ms = 0U;

s_last_status_ms = 0U;
s_last_heartbeat_ms = 0U;


/* -------------------------------------------------------------
 * UART PACKET LAYER
 * ------------------------------------------------------------- */

Uart_Pkt_Init();

Uart_Pkt_SetRxCallback(App_UartPacketRx);


/* -------------------------------------------------------------
 * FLASH LOG MANAGER
 * ------------------------------------------------------------- */


#if defined(FLM_MODULE_EN)

#if (FLM_MODULE_EN != 0U)
Flm_Init();
#endif

#endif


/* -------------------------------------------------------------
 * GPIO
 *
 * Must be initialized before:
 *
 *   - first status transmission
 *   - first GPIO command
 * ------------------------------------------------------------- */

Gpio_ControlInit();


/* -------------------------------------------------------------
 * CAN
 * ------------------------------------------------------------- */


#if defined(CAN1_MODULE_EN)

#if (CAN1_MODULE_EN != 0U)
Can1_Init();
#endif

#endif

#if defined(CAN2_MODULE_EN)

#if (CAN2_MODULE_EN != 0U)
Can2_Init();
#endif

#endif


/* -------------------------------------------------------------
 * IMU
 * ------------------------------------------------------------- */


#if defined(IMU_MODULE_EN)

#if (IMU_MODULE_EN != 0U)
Imu_Init();
#endif

#endif


/* -------------------------------------------------------------
 * OTA
 * ------------------------------------------------------------- */


#if defined(OTA_MODULE_EN)

#if (OTA_MODULE_EN != 0U)
OTA_Init();
#endif

#endif


/* -------------------------------------------------------------
 * COMMAND MODULE
 *
 * NOTE:
 *
 * If Cmd_Init() internally registers its own UART callback,
 * it must NOT override App_UartPacketRx().
 *
 * UART callback ownership should remain with App.
 * ------------------------------------------------------------- */

Cmd_Init();


s_initialized = true;


/* Initial system status */

App_SendStatus();


}

/* =========================================================================

* APPLICATION PERIODIC TASK
* ========================================================================= */

void App_Task(void)
{
if(s_initialized == false)
{
return;
}


/* -------------------------------------------------------------
 * UART PACKET PROCESSING
 * ------------------------------------------------------------- */

Uart_Pkt_Task();


/* -------------------------------------------------------------
 * CAN TASKS
 * ------------------------------------------------------------- */


#if defined(CAN1_MODULE_EN)

#if (CAN1_MODULE_EN != 0U)
Can1_Task();
#endif

#endif

#if defined(CAN2_MODULE_EN)

#if (CAN2_MODULE_EN != 0U)
Can2_Task();
#endif

#endif


/* -------------------------------------------------------------
 * GPIO TASK
 *
 * gpio_control.h declares:
 *
 *     void Gpio_ControlTask(void);
 *
 * Therefore no time argument is passed.
 * ------------------------------------------------------------- */

Gpio_ControlTask();


/* -------------------------------------------------------------
 * IMU TASK
 * ------------------------------------------------------------- */


#if defined(IMU_MODULE_EN)

#if (IMU_MODULE_EN != 0U)
Imu_Task();
#endif

#endif


/* -------------------------------------------------------------
 * OTA TASK
 * ------------------------------------------------------------- */


#if defined(OTA_MODULE_EN)

#if (OTA_MODULE_EN != 0U)
OTA_Task();
#endif

#endif


/* -------------------------------------------------------------
 * Periodic timing
 *
 * This assumes App_Task() is called from the main loop and s_ms
 * is updated by App_Tick1ms().
 * ------------------------------------------------------------- */


/* Periodic heartbeat */

if((uint32_t)(s_ms - s_last_heartbeat_ms)
    >= APP_HEARTBEAT_PERIOD_MS)
{
    s_last_heartbeat_ms = s_ms;

    App_SendHeartbeat();
}


/* Periodic system status */

if((uint32_t)(s_ms - s_last_status_ms)
    >= APP_STATUS_PERIOD_MS)
{
    s_last_status_ms = s_ms;

    App_SendStatus();
}


}

/* =========================================================================

* 1 ms APPLICATION TICK
* ========================================================================= */

/*

* Call this function from:
*
* * SysTick ISR
* * LPIT timer ISR
* * another 1 ms system tick source
*
* Do not perform communication or hardware processing here.
  */

void App_Tick1ms(void)
{
s_ms++;
}

/* =========================================================================

* APPLICATION TIME
* ========================================================================= */
uint32_t App_GetTimeMs(void)
{
return s_ms;
}
