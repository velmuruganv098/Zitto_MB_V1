/*
 * can1_irq.c  -  Zitto_MB_V1 / S32K144
 *
 * CAN1 (FlexCAN1) interrupt handlers.
 *
 * WHY THESE EXIST:
 *   startup_S32K144.S defines ALL unregistered handlers as weak aliases
 *   to DefaultISR (b DefaultISR  - infinite loop).
 *   Without these: ANY CAN1 interrupt → DefaultISR → hang forever.
 *
 * WHEN THEY FIRE:
 *   After a WDOG/software reset, CAN1->ESR1 flags may remain set from
 *   a previous run.  When the PCC clock is re-enabled, those asserted
 *   flags immediately trigger the IRQ.  These handlers clear the source
 *   and return, preventing the DefaultISR hang.
 *
 * IPSR VALUES (confirm in S32DS → Expressions → g_last_exception_ipsr):
 *   0x65 (101) = CAN1_ORed_IRQHandler   (IRQ 85)
 *   0x66 (102) = CAN1_Error_IRQHandler  (IRQ 86)
 *   0x68 (104) = CAN1_ORed_0_15_MB      (IRQ 88)
 *
 * CAN1 uses POLLING (not interrupts) so these handlers only need to:
 *   1. Clear the interrupt source flags
 *   2. Disable the interrupt at source (IMASK1=0)
 *   3. Log the event for diagnostics
 *   4. Return
 *
 * These handlers include "debug_rtt.h" for RTT_LOG.  If RTT is not yet
 * initialized when an early boot IRQ fires, SEGGER_RTT_printf is safe
 * to call (it initializes internally on first use).
 */

#include "can1.h"
#include "debug_rtt.h"
#include "S32K144.h"
#include <stdint.h>

/* --------------------------------------------------------------------------
 * DIAGNOSTIC COUNTERS
 *
 * Inspect in S32DS:  Expressions window → g_can1_or_irq_count etc.
 * Also visible in main() alive log via Can1_GetIrqCount() etc.
 * -------------------------------------------------------------------------- */
volatile uint32_t g_can1_or_irq_count    = 0U;
volatile uint32_t g_can1_error_irq_count = 0U;
volatile uint32_t g_can1_mb_irq_count    = 0U;

/* ==========================================================================
 * CAN1_ORed_IRQHandler
 *
 * IRQ 85  |  IPSR = 101 = 0x65
 * Bus-Off, Tx Warning, Rx Warning
 * ========================================================================== */
void CAN1_ORed_IRQHandler(void)
{
    uint32_t esr1  = CAN1->ESR1;
    uint32_t ifl   = CAN1->IFLAG1;

    g_can1_or_irq_count++;

    RTT_LOG(
        "[CAN1_IRQ] ORed #%lu  ESR1=0x%08lX  IFLAG1=0x%08lX\r\n",
        (unsigned long)g_can1_or_irq_count,
        (unsigned long)esr1,
        (unsigned long)ifl
    );

    /* Disable interrupts at source and clear flags */
    CAN1->IMASK1 = 0U;
    CAN1->IFLAG1 = ifl;          /* W1C */
    CAN1->ESR1   = 0xFFFFFFFFUL; /* W1C */
}

/* ==========================================================================
 * CAN1_Error_IRQHandler
 *
 * IRQ 86  |  IPSR = 102 = 0x66
 * CAN error detection (ERRINT, BOFFINT)
 *
 * MOST LIKELY HANDLER for the DefaultISR trap during init.
 * ESR1.ERRINT or ESR1.BOFFINT from previous run keeps the IRQ line HIGH.
 * ========================================================================== */
void CAN1_Error_IRQHandler(void)
{
    uint32_t esr1  = CAN1->ESR1;
    uint32_t ctrl1 = CAN1->CTRL1;

    g_can1_error_irq_count++;

    RTT_LOG(
        "[CAN1_IRQ] ERROR #%lu  ESR1=0x%08lX  CTRL1=0x%08lX"
        "  ERRINT=%u  BOFFINT=%u\r\n",
        (unsigned long)g_can1_error_irq_count,
        (unsigned long)esr1,
        (unsigned long)ctrl1,
        (unsigned)((esr1 >> 1U) & 1U),   /* ERRINT  bit1 */
        (unsigned)((esr1 >> 2U) & 1U)    /* BOFFINT bit2 */
    );

    /* Disable error interrupts at source */
    CAN1->CTRL1 &= ~(CAN_CTRL1_ERRMSK_MASK | CAN_CTRL1_BOFFMSK_MASK);

    /* Clear error flags (W1C) */
    CAN1->ESR1 = 0xFFFFFFFFUL;
}

/* ==========================================================================
 * CAN1_ORed_0_15_MB_IRQHandler
 *
 * IRQ 88  |  IPSR = 104 = 0x68
 * Mailbox 0-15 interrupt
 * ========================================================================== */
void CAN1_ORed_0_15_MB_IRQHandler(void)
{
    uint32_t ifl = CAN1->IFLAG1;

    g_can1_mb_irq_count++;

    RTT_LOG(
        "[CAN1_IRQ] MB #%lu  IFLAG1=0x%08lX\r\n",
        (unsigned long)g_can1_mb_irq_count,
        (unsigned long)ifl
    );

    /* CAN1 uses polling - disable MB interrupts and clear flags */
    CAN1->IMASK1 = 0U;
    CAN1->IFLAG1 = ifl;   /* W1C */
}

/* --------------------------------------------------------------------------
 * GETTERS (called from main.c alive log)
 * -------------------------------------------------------------------------- */
uint32_t Can1_GetIrqCount(void)      { return g_can1_or_irq_count;    }
uint32_t Can1_GetErrorIrqCount(void) { return g_can1_error_irq_count; }
uint32_t Can1_GetMbIrqCount(void)    { return g_can1_mb_irq_count;    }
