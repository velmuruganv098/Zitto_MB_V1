/*
 * main.c  -  Zitto_MB_V1 / S32K144
 *
 * Firmware Revision : V0.0055
 * Change Note       : CAN1 full auto-baud analysis mode
 *
 * V0.0049 PROJECT BASELINE
 *   - CAN1 detection uses NORMAL/RX-evidence sequence so the MCU ACKs PCAN and never transmits a baud probe.
 *   - The frame that proves the baud is preserved into the application queue and
 *     is therefore visible through [CAN1_APP] and MSG_CAN.
 *   - After a valid RX candidate is verified, CAN1 enters NORMAL mode for PCAN ACK; live PCAN baud changes can recover only from bounded error evidence plus
 *     no-valid-RX confirmation; quiet/no-data operation does not trigger scanning.
 *   - FlexCAN mailbox move-in is protected by a bounded BUSY check.
 *   - Future revisions must keep every CAN/UART/module wait bounded and must not
 *     introduce an indefinite wait on CAN, IMU, CSA, APP, Server or UART activity.
 *
 * ==========================================================================
 * CRITICAL BOOT ORDER (do not change):
 *   1. wdog_disable()       - default WDOG ~256ms, fires during CAN init
 *   2. clock_init_80mhz()   - starts SOSC (crystal) + SPLL → 80MHz
 *                             bus clock = 40MHz (CAN uses this)
 *                             SPLLDIV2  = 20MHz (UART uses this)
 *   3. SEGGER_RTT_Init()    - stable clock now available
 *   4. everything else
 * ==========================================================================
 */

/* --------------------------------------------------------------------------
 * MODULE ENABLE FLAGS
 * -------------------------------------------------------------------------- */

#define APP_IMU_ENABLE     0
#define APP_CSA_ENABLE     0
#define APP_CAN1_ENABLE    1
#define APP_CAN2_ENABLE    0
#define APP_FLM_ENABLE     1
#define APP_GPIO_ENABLE    1

#define TASK_DT_MS          5U

/* --------------------------------------------------------------------------
 * INCLUDES
 * -------------------------------------------------------------------------- */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

#include "S32K144.h"
#include "SEGGER_RTT.h"
#include "system_init.h"
#include "UART/uart_pkt.h"
#include "IMU/imu.h"
#include "CSA/csa.h"
#include "CAN/can1.h"
#include "CAN/can2.h"
#include "FLM/flm.h"
#include "OTA/ota.h"
#include "GPIO/gpio_control.h"
#include "DEBUG/debug_rtt.h"

/* --------------------------------------------------------------------------
 * MODULE IDs for CMD_MODULE_EN
 * -------------------------------------------------------------------------- */

#ifndef MOD_IMU
#define MOD_IMU   0U
#endif
#ifndef MOD_CSA
#define MOD_CSA   1U
#endif
#ifndef MOD_CAN1
#define MOD_CAN1  2U
#endif
#ifndef MOD_CAN2
#define MOD_CAN2  3U
#endif
#ifndef MOD_FLM
#define MOD_FLM   4U
#endif

#ifndef FLM_MAX_RECORD_DATA
#ifdef  UART_PKT_MAX_PAYLOAD
#define FLM_MAX_RECORD_DATA   UART_PKT_MAX_PAYLOAD
#else
#define FLM_MAX_RECORD_DATA   256U
#endif
#endif

/* --------------------------------------------------------------------------
 * RUNTIME FLAGS
 * -------------------------------------------------------------------------- */

static uint8_t g_imu_en  = APP_IMU_ENABLE;
static uint8_t g_csa_en  = APP_CSA_ENABLE;
static uint8_t g_can1_en = APP_CAN1_ENABLE;
static uint8_t g_can2_en = APP_CAN2_ENABLE;
static uint8_t g_flm_en  = APP_FLM_ENABLE;

static uint16_t g_led_period = 500U;
static uint8_t  g_led_duty   = 50U;
static uint8_t  g_led_state  = 0U;

static uint8_t  g_reset_arm    = 0U;
static uint32_t g_reset_arm_ms = 0U;
static uint32_t g_tick = 0U;

/* g_last_exception_ipsr: DEFINED in boot_main.c (startup diagnostic).
 * RULE: exactly ONE C file may define each global - all others use extern. */
extern volatile uint32_t g_last_exception_ipsr;
volatile uint32_t g_can1_debug_step = 0U;

/* ==========================================================================
 * HARD FAULT HANDLER
 * Overrides the weak DefaultISR alias.  Keep simple - stack may be corrupt.
 * ========================================================================== */
void HardFault_Handler(void)
{
    uint32_t i;
    volatile uint32_t n;
    for(i = 0U; i < 20U; i++)
    {
        PTA->PTOR = (1UL << 0U);
        PTE->PTOR = (1UL << 5U);
        for(n = 200000U; n != 0U; n--) { __asm volatile("nop"); }
    }
    *((volatile uint32_t *)0xE000ED0CUL) = 0x05FA0004UL;
    while(1) {}
}

/* --------------------------------------------------------------------------
 * STARTUP DELAY
 * -------------------------------------------------------------------------- */
static void delay_ms(volatile uint32_t ms)
{
    while(ms-- != 0U)
    {
        volatile uint32_t n = 80000U;
        while(n-- != 0U) { __asm volatile("nop"); }
    }
}

/* --------------------------------------------------------------------------
 * LED
 * -------------------------------------------------------------------------- */
static void led_init(void)
{
    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTA->PCR[0U] = PORT_PCR_MUX(1U);
    PORTE->PCR[5U] = PORT_PCR_MUX(1U);
    PTA->PDDR |= (1UL << 0U);
    PTE->PDDR |= (1UL << 5U);
    PTA->PCOR  = (1UL << 0U);
    PTE->PCOR  = (1UL << 5U);
}

static void led_task(void)
{
    uint32_t now, on_time;
    uint8_t  want;
    if(g_led_period == 0U) { return; }
    now     = Uart_GetMs();
    on_time = ((uint32_t)g_led_period * (uint32_t)g_led_duty) / 100U;
    want    = ((now % g_led_period) < on_time) ? 1U : 0U;
    if(want != g_led_state)
    {
        g_led_state = want;
        if(want != 0U) { PTA->PSOR=(1UL<<0U); PTE->PSOR=(1UL<<5U); }
        else           { PTA->PCOR=(1UL<<0U); PTE->PCOR=(1UL<<5U); }
    }
}

/* --------------------------------------------------------------------------
 * STATUS PACKET
 * -------------------------------------------------------------------------- */
static void send_status(void)
{
    StatusPkt_t s;
    memset(&s, 0, sizeof(s));

    s.imu_en  = g_imu_en;
    s.csa_en  = g_csa_en;
    s.can1_en = g_can1_en;
    s.can2_en = g_can2_en;
    s.flm_en  = g_flm_en;
    s.ota_pending = OTA_IsPending() ? 1U : 0U;

#if APP_CAN1_ENABLE
    s.can1_baud_kbps = Can1_GetBaudrate();
#endif
#if APP_CAN2_ENABLE
    s.can2_baud_kbps = Can2_GetBaudrate();
#endif
#if APP_FLM_ENABLE
    s.flash_free_pages = Flm_GetFreePages();
#endif

    s.uptime_ms   = Uart_GetMs();
    s.reset_cause = (uint8_t)(RCM->SRS & 0xFFU);

    (void)Uart_Pkt_SendStatus(&s);

#if APP_GPIO_ENABLE
    (void)Gpio_ControlSendStatus();
#endif
}

/* --------------------------------------------------------------------------
 * CAN1 RX CALLBACK
 * -------------------------------------------------------------------------- */
#if APP_CAN1_ENABLE
static void can1_rx(uint32_t id, uint8_t ide, uint8_t rtr,
                    uint8_t dlc, const uint8_t *data, uint32_t baud)
{
    CanFramePkt_t f;
    uint8_t i;
    memset(&f, 0, sizeof(f));
    f.bus=1U; f.ide=ide; f.rtr=rtr; f.dlc=dlc; f.can_id=id; f.ts_ms=Uart_GetMs();
    if(data) { for(i=0U;i<8U;i++) f.data[i]=(i<dlc)?data[i]:0U; }

    RTT_LOG("[CAN1_APP] baud=%lu ID=0x%08lX DLC=%u DATA=%02X %02X %02X %02X %02X %02X %02X %02X\\r\\n",
            (unsigned long)baud,
            (unsigned long)id,
            (unsigned)dlc,
            (unsigned)f.data[0], (unsigned)f.data[1],
            (unsigned)f.data[2], (unsigned)f.data[3],
            (unsigned)f.data[4], (unsigned)f.data[5],
            (unsigned)f.data[6], (unsigned)f.data[7]);

    (void)Uart_Pkt_SendCan(&f);
}
#endif

/* --------------------------------------------------------------------------
 * CAN2 RX CALLBACK
 * -------------------------------------------------------------------------- */
#if APP_CAN2_ENABLE
static void can2_rx(uint32_t id, uint8_t ide, uint8_t rtr,
                    uint8_t dlc, const uint8_t *data, uint32_t baud)
{
    CanFramePkt_t f;
    uint8_t i;
    (void)baud;
    memset(&f, 0, sizeof(f));
    f.bus=2U; f.ide=ide; f.rtr=rtr; f.dlc=dlc; f.can_id=id; f.ts_ms=Uart_GetMs();
    if(data) { for(i=0U;i<8U;i++) f.data[i]=(i<dlc)?data[i]:0U; }
    (void)Uart_Pkt_SendCan(&f);
}
#endif

/* --------------------------------------------------------------------------
 * COMMAND HANDLER
 * -------------------------------------------------------------------------- */
static void cmd_handler(uint8_t type, const uint8_t *pl, uint16_t len)
{
    /* OTA commands */
    if((type >= CMD_OTA_START) && (type <= CMD_OTA_ABORT))
    {
        switch(type)
        {
            case CMD_OTA_START:
                if((pl != NULL) && (len >= 8U))
                {
                    uint32_t sz  = ((uint32_t)pl[0]<<24U)|((uint32_t)pl[1]<<16U)|((uint32_t)pl[2]<<8U)|(uint32_t)pl[3];
                    uint32_t crc = ((uint32_t)pl[4]<<24U)|((uint32_t)pl[5]<<16U)|((uint32_t)pl[6]<<8U)|(uint32_t)pl[7];
                    if(OTA_Start(sz,crc)) { Uart_Pkt_SendLog("OTA:start_ok"); }
                    else { Uart_Pkt_SendLog("OTA:start_fail"); }
                } else { Uart_Pkt_SendLog("OTA:bad_len"); }
                break;
            case CMD_OTA_DATA:
                if((pl != NULL) && (len > 0U) && !OTA_WriteChunk(pl,len))
                { Uart_Pkt_SendLog("OTA:write_fail"); }
                break;
            case CMD_OTA_FINISH:
                if(OTA_Finish()) { Uart_Pkt_SendLog("OTA:ok"); }
                else             { Uart_Pkt_SendLog("OTA:verify_fail"); }
                break;
            case CMD_OTA_ABORT:
                OTA_Abort(); Uart_Pkt_SendLog("OTA:aborted");
                break;
            default: break;
        }
        return;
    }

    switch(type)
    {
        case CMD_MODULE_EN:
        {
            uint8_t mod, state;
            if((pl == NULL)||(len < 2U)) { Uart_Pkt_SendLog("CMD_MODULE_EN:bad_len"); break; }
            mod=pl[0]; state=(pl[1]!=0U)?1U:0U;
            switch(mod)
            {
                case MOD_IMU:  g_imu_en  = state; break;
                case MOD_CSA:  g_csa_en  = state; break;
                case MOD_CAN1: g_can1_en = state; break;
                case MOD_CAN2: g_can2_en = state; break;
                case MOD_FLM:  g_flm_en  = state; break;
                default: Uart_Pkt_SendLog("CMD_MODULE_EN:bad_module"); break;
            }
            send_status();
        }
        break;

        case CMD_GPIO_SET:
#if APP_GPIO_ENABLE
            if((pl == NULL)||(len == 0U)) { Uart_Pkt_SendLog("GPIO:bad_cmd"); break; }
            (void)Gpio_ControlProcessCommand(pl, len);
#else
            Uart_Pkt_SendLog("GPIO:disabled");
#endif
            break;

        case CMD_MCU_RESET:
            if(g_reset_arm == 0U)
            { g_reset_arm=1U; g_reset_arm_ms=Uart_GetMs(); Uart_Pkt_SendLog("RESET:armed"); }
            break;

        case CMD_LED_CTRL:
            if((pl != NULL)&&(len >= (uint16_t)sizeof(LedCtrlCmd_t)))
            {
                LedCtrlCmd_t lc;
                memcpy(&lc, pl, sizeof(lc));
                g_led_period = (lc.period_ms != 0U) ? lc.period_ms : 500U;
                g_led_duty   = (lc.duty_pct  <= 100U)? lc.duty_pct  : 50U;
            }
            else { Uart_Pkt_SendLog("LED:bad_cmd"); }
            break;

        case CMD_STATUS_REQ:
            send_status();
            break;

        case CMD_FLASH_RD:
#if APP_FLM_ENABLE
        {
            static uint8_t fb[FLM_MAX_RECORD_DATA];
            uint16_t fl = 0U;
            if(Flm_Read(fb,(uint16_t)sizeof(fb),&fl)==0)
            { (void)Uart_Pkt_Send(MSG_LOG,fb,fl); }
            else { Uart_Pkt_SendLog("FLASH:empty"); }
        }
#else
            Uart_Pkt_SendLog("FLASH:off");
#endif
            break;
