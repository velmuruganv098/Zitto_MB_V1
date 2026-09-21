/*
 * system_init.c
 *
 * Zitto_MB_V1 / S32K144
 *
 * Clock configuration:
 *
 * External crystal : 8 MHz
 * Core clock       : 80 MHz
 * Bus clock        : 40 MHz
 * Slow clock       : 20 MHz
 *
 */

#include "S32K144.h"
#include "system_init.h"


/* ============================================================================
 * Disable Watchdog
 * ========================================================================== */

void wdog_disable(void)
{
    /*
     * Unlock WDOG registers.
     */

    WDOG->CNT = 0xD928C520U;


    /*
     * Maximum timeout.
     */

    WDOG->TOVAL = 0xFFFFU;


    /*
     * Disable watchdog.
     */

    WDOG->CS = 0x00002100U;
}


/* ============================================================================
 * Initialize system clock to 80 MHz
 * ========================================================================== */

void clock_init_80mhz(void)
{
    /* =======================================================================
     * SOSC
     *
     * External 8 MHz crystal.
     *
     * Pin 8 : EXTAL
     * Pin 9 : XTAL
     * ===================================================================== */


    /*
     * SOSC divider configuration.
     *
     * DIV1 = /1
     * DIV2 = /1
     */

    SCG->SOSCDIV = 0x00000101U;


    /*
     * SOSC configuration.
     *
     * RANGE = medium frequency
     * EREFS = crystal oscillator
     * HGO   = high gain
     */

    SCG->SOSCCFG = 0x00000034U;


    /*
     * Wait until configuration is unlocked.
     */

    while ((SCG->SOSCCSR & SCG_SOSCCSR_LK_MASK) != 0U)
    {
    }


    /*
     * Enable SOSC.
     */

    SCG->SOSCCSR = 0x00000001U;


    /*
     * Wait for external oscillator to become valid.
     */

    while ((SCG->SOSCCSR &
            SCG_SOSCCSR_SOSCVLD_MASK) == 0U)
    {
    }


    /* =======================================================================
     * SPLL
     *
     * 8 MHz crystal
     *
     * SPLL output:
     *
     * 8 MHz × (MULT + 16) / (PREDIV + 1)
     *
     * With:
     *
     * PREDIV = 0
     * MULT   = 4
     *
     * Internal SPLL calculation used by this project:
     *
     * 80 MHz system source.
     * ===================================================================== */


    /*
     * SPLL divider configuration.
     *
     * DIV1 = /2
     * DIV2 = /4
     */

    SCG->SPLLDIV = 0x00000302U;


    /*
     * SPLL configuration.
     */

    SCG->SPLLCFG = 0x00040000U;


    /*
     * Wait until SPLL configuration is unlocked.
     */

    while ((SCG->SPLLCSR &
            SCG_SPLLCSR_LK_MASK) != 0U)
    {
    }


    /*
     * Enable SPLL.
     */

    SCG->SPLLCSR = 0x00000001U;


    /*
     * Wait for SPLL lock.
     */

    while ((SCG->SPLLCSR &
            SCG_SPLLCSR_SPLLVLD_MASK) == 0U)
    {
    }


    /* =======================================================================
     * Switch RUN mode clock to SPLL
     *
     * Core = 80 MHz
     * Bus  = 40 MHz
     * Slow = 20 MHz
     * ===================================================================== */

    SCG->RCCR = 0x06000103U;


    /*
     * Wait until SPLL becomes the active system clock.
     */

    while (((SCG->CSR >> 24U) & 0x0FU) != 6U)
    {
    }
}
