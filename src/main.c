/*
 * main.c  -  Zitto_MB_V1 / S32K144
 *
 * Firmware Revision : V0.0062
 * Change Note       : CAN1 harmonic-alias rejection + gated application RX
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

#define APP_IMU_ENABLE     1
#define APP_CSA_ENABLE     1
#define APP_CAN1_ENABLE    1
#define APP_CAN2_ENABLE    1
#define APP_FLM_ENABLE     1
#define APP_GPIO_ENABLE    1

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
static uint32_t g_hb_count = 0U;

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

    /*
     * FIX: capture the ACTUAL faulting stack frame FIRST, as the very
     * first statement in this function - "mrs %0,msp" is a single
     * instruction with no stack use of its own, executed before this
     * function's own (small, fixed) prologue has a chance to push
     * anything that would offset the value away from where the CPU's
     * automatic exception entry left it. HardFault_Handler is not
     * naked (an earlier, simpler diagnostic pass just read CFSR/HFSR/
     * BFAR/MMFAR, which proved the fault is BFSR=0x04 IMPRECISERR - an
     * imprecise bus fault, meaning the CPU's write buffer only
     * detected the bad write several instructions AFTER it actually
     * happened, so CFSR/HFSR alone cannot say WHICH instruction/write
     * was responsible). PSP is confirmed 0 in this project (no RTOS,
     * everything runs on the main/exception stack), so MSP is always
     * the frame that matters. The hardware exception frame is
     * {R0,R1,R2,R3,R12,LR,PC,xPSR} - offsets 0..7 - PC (offset 6) is
     * the return address into the code that was running when the
     * exception was TAKEN, which for an imprecise fault is only a
     * rough neighborhood of the real culprit, not the exact faulting
     * write - but it is still far more specific than nothing, and
     * combined with ACTLR.DISDEFWBUF being set in main() below
     * (disables write buffering from this boot onward), the NEXT
     * occurrence of this fault will be PRECISE and this same PC
     * capture will then point at the exact faulting instruction.
     */
    uint32_t fault_msp;
    uint32_t fault_pc;
    uint32_t fault_lr;
    __asm volatile ("mrs %0, msp" : "=r" (fault_msp));
    fault_pc = ((volatile uint32_t *)fault_msp)[6];
    fault_lr = ((volatile uint32_t *)fault_msp)[5];

    /*
     * FIX: capture the hardware fault-status registers BEFORE doing
     * anything else, while they are still fresh - CFSR/HFSR/BFAR are
     * plain memory-mapped registers (System Control Block), reading
     * them is just a load, no function call beyond one printf, no
     * extra stack use beyond this function's own locals, so this is
     * safe even if the stack that triggered the fault is suspect.
     * Logged (not just left for a debugger to inspect) specifically
     * because this handler's whole purpose is to recover via reset
     * when nothing is attached - without this, every occurrence was
     * diagnosed blind from indirect RTT evidence (which CAN2 log
     * line printed last), never from the CPU's own account of what
     * actually happened.
     */
    {
        uint32_t cfsr  = *((volatile uint32_t *)0xE000ED28UL);
        uint32_t hfsr  = *((volatile uint32_t *)0xE000ED2CUL);
        uint32_t bfar  = *((volatile uint32_t *)0xE000ED38UL);
        uint32_t mmfar = *((volatile uint32_t *)0xE000ED34UL);

        SEGGER_RTT_printf(0,
            "\r\n[FAULT] HardFault  CFSR=0x%08lX (MMFSR=0x%02lX BFSR=0x%02lX"
            " UFSR=0x%04lX)  HFSR=0x%08lX  BFAR=0x%08lX  MMFAR=0x%08lX\r\n",
            (unsigned long)cfsr,
            (unsigned long)(cfsr & 0xFFUL),
            (unsigned long)((cfsr >> 8U) & 0xFFUL),
            (unsigned long)((cfsr >> 16U) & 0xFFFFUL),
            (unsigned long)hfsr,
            (unsigned long)bfar,
            (unsigned long)mmfar);

        SEGGER_RTT_printf(0,
            "[FAULT]   stacked PC=0x%08lX  LR=0x%08lX  MSP=0x%08lX\r\n",
            (unsigned long)fault_pc,
            (unsigned long)fault_lr,
            (unsigned long)fault_msp);
    }

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
    s.heartbeat_count = g_hb_count;

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
static void can2_rx(const Can2_Frame_t *frame)
{
    CanFramePkt_t f;
    uint8_t i;
    if(frame == NULL) { return; }
    memset(&f, 0, sizeof(f));
    f.bus=2U; f.ide=frame->extended; f.rtr=frame->rtr; f.dlc=frame->dlc;
    f.can_id=frame->id; f.ts_ms=Uart_GetMs();
    for(i=0U;i<8U;i++) { f.data[i]=(i<frame->dlc)?frame->data[i]:0U; }

    RTT_LOG("[CAN2_APP] baud=%lu ID=0x%08lX DLC=%u DATA=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            (unsigned long)Can2_GetBaudrate(),
            (unsigned long)frame->id,
            (unsigned)frame->dlc,
            (unsigned)f.data[0], (unsigned)f.data[1],
            (unsigned)f.data[2], (unsigned)f.data[3],
            (unsigned)f.data[4], (unsigned)f.data[5],
            (unsigned)f.data[6], (unsigned)f.data[7]);

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

        case CMD_FLASH_WR:
#if APP_FLM_ENABLE
            if((pl!=NULL)&&(len>0U)&&(len<=FLM_MAX_RECORD_DATA))
            {
                if(Flm_Write(pl,len)==0) { Uart_Pkt_SendLog("FLASH:OK"); }
                else                     { Uart_Pkt_SendLog("FLASH:FAIL"); }
            }
            else { Uart_Pkt_SendLog("FLASH:bad_len"); }
#else
            Uart_Pkt_SendLog("FLASH:off");
#endif
            break;

        case CMD_FLASH_DEL:
#if APP_FLM_ENABLE
            if(Flm_Delete()==0) { Uart_Pkt_SendLog("FLASH:deleted"); }
            else                { Uart_Pkt_SendLog("FLASH:FAIL"); }
#else
            Uart_Pkt_SendLog("FLASH:off");
#endif
            break;

        default:
            Uart_Pkt_SendLog("CMD:unknown");
            break;
    }
}

/* ==========================================================================
 * MAIN
 * ========================================================================== */
int main(void)
{
    uint8_t i;
    uint8_t clock_ok;
    uint32_t now_ms;
    uint32_t last_alive_ms = 0U;
    uint32_t last_hb_ms = 0U;
    uint32_t last_imu_task_ms = 0U;
    uint32_t last_imu_tx_ms = 0U;
    uint32_t last_csa_tx_ms = 0U;
    uint32_t last_flm_tx_ms = 0U;


    /*
     * FIX: ACTLR.DISDEFWBUF=1 (bit1) disables the Cortex-M4's default
     * write buffering, forcing every bus fault to be PRECISE instead
     * of imprecise - the CPU stalls until the faulting write's
     * response comes back, so the exception is taken with PC pointing
     * AT the actual faulting instruction rather than several
     * instructions later. Done as the very first thing in main() (a
     * single core-register write, no peripheral/clock dependency, so
     * it is safe before even WDOG/clock init) specifically because a
     * confirmed, reproducible IMPRECISERR HardFault (BFSR=0x04) during
     * CAN2's bus-heavy/bus-off restart path has resisted every static
     * hypothesis tried so far (freeze-cycle merge, mailbox RAM bounds,
     * mask-register mirroring) - CFSR/HFSR alone cannot say WHICH
     * register/RAM write in that sequence is responsible when the
     * fault is imprecise. Costs a little write throughput (writes can
     * no longer post to the buffer and return immediately); acceptable
     * for a debug build chasing this specific fault. See
     * HardFault_Handler()'s stacked-PC capture, which becomes exact
     * once this is set.
     */
    *((volatile uint32_t *)0xE000E008UL) |= (1UL << 1U);
    /* ======================================================================
     * STEP 1: WDOG DISABLE  - absolute first call
     * STEP 2: CLOCK INIT    - starts bus clock (40MHz for CAN) and
     *                         SPLLDIV2 (20MHz for UART)
     *
     * These two were MISSING in the CAN_3 version - root cause of:
     *   UART TX failed byte=1/49   (no SPLLDIV2 clock)
     *   CAN hung at STEP 7 LPMACK  (no bus clock = CAN can't enable)
     *   DefaultISR trap             (WDOG fired during init)
     * ====================================================================== */
    wdog_disable();
    clock_ok = clock_init_80mhz();

    /* THEN RTT can be initialized */
    SEGGER_RTT_Init();
    Debug_RTT_Init();

    if(clock_ok == 0U)
    {
        /* Clock failure is a fatal hardware-init condition, but never spin.
         * Keep RTT alive so the failure is visible and prevent CAN timing
         * code from running against an unknown clock. */
        SEGGER_RTT_printf(0,
            "[BOOT_ERR] 80MHz clock tree init timeout - CAN1 disabled safely\\r\\n");
        g_can1_en = 0U;
        g_can2_en = 0U;
    }

    SEGGER_RTT_printf(0,
        "\r\n================================================\r\n"
        " Zitto MB V1 - VCU Firmware Boot\r\n"
        " Firmware Revision : V0.00631\r\n"
        " Change            : CAN1 pre-RX error immunity + symmetric 2:1 baud corroboration; fixed test OFF\r\n"
        " MCU: S32K144  Clock: 80MHz SPLL  WDOG: OFF\r\n"
        " Modules: IMU=%d CSA=%d CAN1=%d CAN2=%d FLM=%d GPIO=%d\r\n"
        "================================================\r\n\r\n",
        APP_IMU_ENABLE, APP_CSA_ENABLE, APP_CAN1_ENABLE,
        APP_CAN2_ENABLE, APP_FLM_ENABLE, APP_GPIO_ENABLE);

    /* LED */
    RTT_LOG("[BOOT] LED init\r\n");
    led_init();
    for(i=0U; i<6U; i++) { PTA->PTOR=(1UL<<0U); PTE->PTOR=(1UL<<5U); delay_ms(40U); }
    RTT_LOG("[BOOT] LED ok\r\n");

    /* UART - LPUART0  PTC3=TX  PTC2=RX  115200 8N1
     * Clock: SPLLDIV2 = 20MHz (running now because clock_init_80mhz was called) */
    RTT_LOG("[BOOT] UART init\r\n");
    Uart_Init(cmd_handler);
    RTT_LOG("[BOOT] UART ok  uptime=%lums\r\n", (unsigned long)Uart_GetMs());

    /* GPIO */
#if APP_GPIO_ENABLE
    RTT_LOG("[BOOT] GPIO init\r\n");
    Gpio_ControlInit();
    RTT_LOG("[BOOT] GPIO ok\r\n");
#endif

    /* OTA - always init, no #ifdef guards */
    RTT_LOG("[BOOT] OTA init\r\n");
    OTA_Init();
    RTT_LOG("[BOOT] OTA ok  state=%u\r\n", (unsigned)OTA_GetState());

    /* CAN1 - FlexCAN1 PTA12/PTA13 TCAN334 SHDN=PTB2
     * BUS_CLK=40MHz, known-working 500/250/125/1000kbps candidates.
     * Detection uses NORMAL/ACK and RX evidence; firmware generates no TX probe.
     * RX uses MB4..MB15 and application forwarding is decoupled. */
#if APP_CAN1_ENABLE
    RTT_LOG("[BOOT] CAN1 init  FlexCAN1  PTA12/PTA13  SHDN=PTB2\r\n");
    Can1_Init();
    Can1_SetRxCallback(can1_rx);
    RTT_LOG("[BOOT] CAN1 ok  state=%u\r\n", (unsigned)Can1_GetState());
#endif

    /* CAN2 - FlexCAN2 (physical instance 2, NOT FlexCAN0)  PTC16=RX  PTB13=TX  (no SHDN pin) */
#if APP_CAN2_ENABLE
    RTT_LOG("[BOOT] CAN2 init  FlexCAN2  PTC16/PTB13\r\n");
    Can2_Init();
    Can2_SetRxCallback(can2_rx);
    RTT_LOG("[BOOT] CAN2 ok  state=%u\r\n", (unsigned)Can2_GetState());
#endif

    /* FLM - W25N01GV  64 pages/block */
#if APP_FLM_ENABLE
    RTT_LOG("[BOOT] FLM init\r\n");
    Flm_Init();
    RTT_LOG("[BOOT] FLM ok  free=%lu pages\r\n", (unsigned long)Flm_GetFreePages());
#endif

    /* IMU - initialized after CAN1/CAN2 so Imu_Calibrate()'s ~2s blocking
     * calibration cannot delay CAN2's time-sensitive boot-time baud
     * detection. */
#if APP_IMU_ENABLE
    RTT_LOG("[BOOT] IMU init\r\n");
    Imu_Init();
    Imu_Calibrate();
    RTT_LOG("[BOOT] IMU ok\r\n");
#endif

    /* CSA */
#if APP_CSA_ENABLE
    RTT_LOG("[BOOT] CSA init\r\n");
    Csa_Init();
    RTT_LOG("[BOOT] CSA ok\r\n");
#endif

    send_status();
    RTT_LOG("[BOOT] BOOT COMPLETE  uptime=%lums\r\n\r\n", (unsigned long)Uart_GetMs());

    /* ======================================================================
     * MAIN LOOP
     *
     * V0.0048 CAN priority, V0.0065 no fixed delay:
     *   - CAN1 is serviced first every iteration
     *   - UART/OTA remain frequent
     *   - slower sensor/status work keeps explicit Uart_GetMs() time
     *     gates instead of relying on a shared per-iteration delay
     * ====================================================================== */
    while(1)
    {
        now_ms = Uart_GetMs();
        g_tick++;

        /* CAN1 FIRST: minimize RX mailbox service latency. */
#if APP_CAN1_ENABLE
        if(g_can1_en != 0U)
        {
            /* V0.0063: Can1_Task() dispatches RX synchronously via the
             * RX callback now, no separate queue to drain. */
            Can1_Task();
        }
#endif

#if CAN1_FULL_ANALYSIS_MODE
        /* V0.0056: CAN-only bench mode. Do not let UART/RTT/OTA/application
         * work distort the FlexCAN RX-service measurement. */
#else
        Uart_Poll();
        Uart_Pkt_ForwardRTT();
        OTA_Task();
#endif

#if !CAN1_FULL_ANALYSIS_MODE
        /* Alive log every 1s, independent of loop frequency. */
        if((now_ms - last_alive_ms) >= 1000U)
        {
            last_alive_ms = now_ms;
            RTT_LOG("[MAIN] tick=%lu uptime=%lums  CAN1=%lukbps  state=%u  IRQs: or=%lu err=%lu mb=%lu\r\n",
                    (unsigned long)g_tick,
                    (unsigned long)now_ms,
                    (unsigned long)Can1_GetBaudrate(),
                    (unsigned)Can1_GetState(),
                    (unsigned long)Can1_GetIrqCount(),
                    (unsigned long)Can1_GetErrorIrqCount(),
                    (unsigned long)Can1_GetMbIrqCount());

#if APP_CAN1_ENABLE
            {
                Can1_Status_t cs1;
                CanStatusPkt_t p1;
                memset(&cs1, 0, sizeof(cs1));
                Can1_GetStatus(&cs1);
                memset(&p1, 0, sizeof(p1));
                p1.bus = 1U;
                p1.state = (uint8_t)Can1_GetState();
                p1.ready = cs1.ready;
                p1.bus_off = cs1.bus_off;
                p1.detected_baud_kbps = cs1.detected_baud_kbps;
                p1.rx_count = cs1.rx_count;
                p1.error_count = cs1.error_count;
                p1.tx_err_cnt = cs1.tx_err_cnt;
                p1.rx_err_cnt = cs1.rx_err_cnt;
                p1.irq_count = Can1_GetIrqCount();
                p1.error_irq_count = Can1_GetErrorIrqCount();
                p1.mb_irq_count = Can1_GetMbIrqCount();
                p1.ts_ms = now_ms;
                (void)Uart_Pkt_SendCanStatus(&p1);
            }
#endif

#if APP_CAN2_ENABLE
            RTT_LOG("[MAIN] tick=%lu uptime=%lums  CAN2=%lukbps  state=%u  IRQs: or=%lu err=%lu mb=%lu\r\n",
                    (unsigned long)g_tick,
                    (unsigned long)now_ms,
                    (unsigned long)Can2_GetBaudrate(),
                    (unsigned)Can2_GetState(),
                    (unsigned long)Can2_GetIrqCount(),
                    (unsigned long)Can2_GetErrorIrqCount(),
                    (unsigned long)Can2_GetMbIrqCount());

            {
                Can2_Status_t cs2;
                CanStatusPkt_t p2;
                memset(&cs2, 0, sizeof(cs2));
                Can2_GetStatus(&cs2);
                memset(&p2, 0, sizeof(p2));
                p2.bus = 2U;
                p2.state = (uint8_t)cs2.state;
                p2.ready = cs2.ready;
                p2.bus_off = (cs2.bus_off_count != 0U) ? 1U : 0U;
                p2.detected_baud_kbps = cs2.detected_baud_kbps;
                p2.rx_count = cs2.rx_count;
                p2.error_count = cs2.error_count;
                p2.irq_count = Can2_GetIrqCount();
                p2.error_irq_count = Can2_GetErrorIrqCount();
                p2.mb_irq_count = Can2_GetMbIrqCount();
                p2.ts_ms = now_ms;
                (void)Uart_Pkt_SendCanStatus(&p2);
            }
#endif
        }

        /* Software reset */
        if(g_reset_arm != 0U)
        {
            if((now_ms - g_reset_arm_ms) >= 100U)
            {
                RTT_LOG("[RESET] Executing software reset\r\n");
                *((volatile uint32_t *)0xE000ED0CUL) = 0x05FA0004UL;
                while(1) {}
            }
        }
#endif

        /* IMU - Task() does a bit-banged I2C transaction and its
         * velocity/position integration assumes a fixed IMU_DT_MS
         * sample period, so it's gated to that interval instead of
         * running every loop iteration (which previously stretched the
         * ~5ms cooperative loop to ~100ms+ once IMU was enabled - see
         * V0.0065-next_8). UART send stays independently gated at
         * 500ms; ESP doesn't need every sample. */
#if APP_IMU_ENABLE
        if(g_imu_en != 0U)
        {
            if((now_ms - last_imu_task_ms) >= IMU_DT_MS)
            {
                last_imu_task_ms = now_ms;
                Imu_Task();
            }
            if((now_ms - last_imu_tx_ms) >= 500U)
            {
                last_imu_tx_ms = now_ms;
                ImuPkt_t p; memset(&p,0,sizeof(p));
                Imu_GetLastPkt(&p);
                (void)Uart_Pkt_SendImu(&p);
            }
        }
#endif

        /* CSA - Task() does a handful of bit-banged I2C register reads
         * per call; only needs to run as often as we actually send, so
         * gate both together instead of reading every loop iteration. */
#if APP_CSA_ENABLE
        if(g_csa_en != 0U)
        {
            if((now_ms - last_csa_tx_ms) >= 200U)
            {
                last_csa_tx_ms = now_ms;
                Csa_Task();
                CsaPkt_t p; memset(&p,0,sizeof(p));
                Csa_GetLastPkt(&p);
                (void)Uart_Pkt_SendCsa(&p);
            }
        }
#endif

        /* CAN2 remains independent; when enabled it gets the same fast
         * cooperative service cadence and does not wait for CAN1. */
#if APP_CAN2_ENABLE
        if(g_can2_en != 0U) { Can2_Task(); }
#endif

        /* FLM */
#if APP_FLM_ENABLE
        if(g_flm_en != 0U)
        {
            Flm_Task();
            if((now_ms - last_flm_tx_ms) >= 2000U)
            {
                last_flm_tx_ms = now_ms;
                FlmInfo_t info;
                FlmStatusPkt_t p;
                memset(&info, 0, sizeof(info));
                Flm_GetInfo(&info);
                memset(&p, 0, sizeof(p));
                p.total_pages = info.total_pages;
                p.used_pages  = info.used_pages;
                p.free_pages  = info.free_pages;
                p.next_page   = info.next_page;
                p.last_page   = info.last_page;
                p.records     = info.records;
                p.ts_ms       = now_ms;
                (void)Uart_Pkt_SendFlm(&p);
            }
        }
#endif

#if !CAN1_FULL_ANALYSIS_MODE
        /* Heartbeat + status every 5s */
        if((now_ms - last_hb_ms) >= 5000U)
        {
            last_hb_ms = now_ms;
            g_hb_count++;
            (void)Uart_Pkt_SendHb();
            send_status();
        }

        led_task();
#endif

        /* No fixed per-iteration delay here (V0.0065): every section
         * above gates itself off the Uart_GetMs() timebase rather than
         * off loop-iteration counts, so nothing needs a shared sleep to
         * pace it - and measurement showed this delay costing ~45ms
         * against a ~5ms budget in this debug build, which was blocking
         * every other section (including UART TX queue servicing) once
         * per iteration regardless of whether it had any work to do.
         * CAN1/CAN2 detection is explicitly time-based (see their own
         * comments), so it stays correct with the loop running as fast
         * as the enabled sections' real work allows. Bench-analysis mode
         * keeps its own explicit delay unchanged. */
#if CAN1_FULL_ANALYSIS_MODE
        delay_ms(CAN1_ANALYSIS_LOOP_DELAY_MS);
#endif
    }

    return 0;
}
