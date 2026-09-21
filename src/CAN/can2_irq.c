/*
 * can2_irq.c  -  Zitto_MB_V1 / S32K144
 *
 * CAN2 (FlexCAN2, register base CAN2_BASE 0x4002B000 - the THIRD
 * physical FlexCAN instance, NOT FlexCAN0) interrupt handlers.
 *
 * Mirrors can1_irq.c exactly - see that file for the full rationale.
 * Summary:
 *
 * WHY THESE EXIST:
 *   startup_S32K144.S defines ALL unregistered handlers as weak aliases
 *   to DefaultISR (b DefaultISR  - infinite loop).
 *   Without these: ANY CAN2 interrupt -> DefaultISR -> hang forever.
 *
 * WHEN THEY FIRE:
 *   After a WDOG/software reset, CAN2->ESR1 flags may remain set from
 *   a previous run.  When the PCC clock is re-enabled, those asserted
 *   flags immediately trigger the IRQ.  These handlers clear the source
 *   and return, preventing the DefaultISR hang.
 *
 * IRQ NUMBERS (S32K144.h confirmed, see can2.h):
 *   CAN2_ORed_IRQn          = 92  ->  CAN2_ORed_IRQHandler
 *   CAN2_Error_IRQn         = 93  ->  CAN2_Error_IRQHandler
 *   CAN2_ORed_0_15_MB_IRQn  = 95  ->  CAN2_ORed_0_15_MB_IRQHandler
 *
 * CAN2 uses POLLING (not interrupts) so these handlers only need to:
 *   1. Clear the interrupt source flags
 *   2. Disable the interrupt at source (IMASK1=0)
 *   3. Log the event for diagnostics
 *   4. Return
 */

#include "can2.h"
#include "debug_rtt.h"
#include "S32K144.h"
#include <stdint.h>

/* --------------------------------------------------------------------------
 * DIAGNOSTIC COUNTERS
 * -------------------------------------------------------------------------- */
volatile uint32_t g_can2_or_irq_count    = 0U;
volatile uint32_t g_can2_error_irq_count = 0U;
volatile uint32_t g_can2_mb_irq_count    = 0U;

/* ==========================================================================
 * CAN2_ORed_IRQHandler
 *
 * IRQ 92  |  Bus-Off, Tx Warning, Rx Warning
 * ========================================================================== */
void CAN2_ORed_IRQHandler(void)
{
    uint32_t esr1 = CAN2->ESR1;
    uint32_t ifl  = CAN2->IFLAG1;

    g_can2_or_irq_count++;

    RTT_LOG(
        "[CAN2_IRQ] ORed #%lu  ESR1=0x%08lX  IFLAG1=0x%08lX\r\n",
        (unsigned long)g_can2_or_irq_count,
        (unsigned long)esr1,
        (unsigned long)ifl
    );

    /* Disable interrupts at source and clear flags */
    CAN2->IMASK1 = 0U;
    CAN2->IFLAG1 = ifl;          /* W1C */
    CAN2->ESR1   = 0xFFFFFFFFUL; /* W1C */
}

/* ==========================================================================
 * CAN2_Error_IRQHandler
 *
 * IRQ 93  |  CAN error detection (ERRINT, BOFFINT)
 * ========================================================================== */
void CAN2_Error_IRQHandler(void)
{
    uint32_t esr1  = CAN2->ESR1;
    uint32_t ctrl1 = CAN2->CTRL1;

    g_can2_error_irq_count++;

    RTT_LOG(
        "[CAN2_IRQ] ERROR #%lu  ESR1=0x%08lX  CTRL1=0x%08lX"
        "  ERRINT=%u  BOFFINT=%u\r\n",
        (unsigned long)g_can2_error_irq_count,
        (unsigned long)esr1,
        (unsigned long)ctrl1,
        (unsigned)((esr1 >> 1U) & 1U),   /* ERRINT  bit1 */
        (unsigned)((esr1 >> 2U) & 1U)    /* BOFFINT bit2 */
    );

    /* Disable error interrupts at source */
    CAN2->CTRL1 &= ~(CAN_CTRL1_ERRMSK_MASK | CAN_CTRL1_BOFFMSK_MASK);

    /* Clear error flags (W1C) */
    CAN2->ESR1 = 0xFFFFFFFFUL;
}

/* ==========================================================================
 * CAN2_ORed_0_15_MB_IRQHandler
 *
 * IRQ 95  |  Mailbox 0-15 interrupt
 * ========================================================================== */
void CAN2_ORed_0_15_MB_IRQHandler(void)
{
    uint32_t ifl = CAN2->IFLAG1;

    g_can2_mb_irq_count++;

    RTT_LOG(
        "[CAN2_IRQ] MB #%lu  IFLAG1=0x%08lX\r\n",
        (unsigned long)g_can2_mb_irq_count,
        (unsigned long)ifl
    );

    /* CAN2 uses polling - disable MB interrupts and clear flags */
    CAN2->IMASK1 = 0U;
    CAN2->IFLAG1 = ifl;   /* W1C */
}

/* --------------------------------------------------------------------------
 * GETTERS (called from main.c alive log)
 * -------------------------------------------------------------------------- */
uint32_t Can2_GetIrqCount(void)      { return g_can2_or_irq_count;    }
uint32_t Can2_GetErrorIrqCount(void) { return g_can2_error_irq_count; }
uint32_t Can2_GetMbIrqCount(void)    { return g_can2_mb_irq_count;    }
