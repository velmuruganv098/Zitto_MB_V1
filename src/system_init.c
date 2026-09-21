/*
 * system_init.c - Zitto_MB_V1 / S32K144
 *
 * Clock configuration:
 *   External crystal : 8 MHz SOSC
 *   SPLL_CLK         : 160 MHz
 *   CORE/SYS_CLK     : 80 MHz
 *   BUS_CLK          : 40 MHz
 *   SLOW/FLASH_CLK   : 26.67 MHz
 *
 * IMPORTANT:
 *   The previous V0.0044 clock values did not match the S32K1xx RCCR
 *   field layout. The corrected SPLL/RCCR values below are the standard
 *   S32K1xx 80MHz RUN configuration and are also the clock basis used by
 *   the CAN1 40MHz bit-timing table.
 *
 * All clock waits are bounded. A missing crystal/PLL can therefore never
 * trap the MCU in an infinite initialization loop.
 */

#include "S32K144.h"
#include "system_init.h"

#define SYSCLK_TIMEOUT 2000000U

static uint8_t prv_WaitSet(volatile uint32_t *reg,
                           uint32_t mask,
                           uint32_t expected)
{
    uint32_t timeout = SYSCLK_TIMEOUT;

    while(((*reg & mask) != expected) && (timeout != 0U))
    {
        timeout--;
    }

    return (timeout != 0U) ? 1U : 0U;
}

void wdog_disable(void)
{
    WDOG->CNT = 0xD928C520U;
    WDOG->TOVAL = 0xFFFFU;
    WDOG->CS = 0x00002100U;
}

/*
 * Return:
 *   1 = 80MHz/40MHz clock tree established
 *   0 = a clock lock/valid/switch timeout occurred
 */
uint8_t clock_init_80mhz(void)
{
    /* SOSC: 8MHz crystal, DIV1=/1, DIV2=/1. */
    SCG->SOSCDIV = 0x00000101U;
    SCG->SOSCCFG = 0x00000034U;

    if(prv_WaitSet(&SCG->SOSCCSR,
                   SCG_SOSCCSR_LK_MASK,
                   0U) == 0U)
    {
        return 0U;
    }

    SCG->SOSCCSR = 0x00000001U;

    if(prv_WaitSet(&SCG->SOSCCSR,
                   SCG_SOSCCSR_SOSCVLD_MASK,
                   SCG_SOSCCSR_SOSCVLD_MASK) == 0U)
    {
        return 0U;
    }

    /*
     * SPLL:
     *   PREDIV=0  -> /1
     *   MULT=24   -> x40
     *   VCO=8*40=320MHz
     *   SPLL=320/2=160MHz
     *   DIV1=/2 -> 80MHz
     *   DIV2=/4 -> 40MHz
     */
    SCG->SPLLDIV = 0x00000302U;

    if(prv_WaitSet(&SCG->SPLLCSR,
                   SCG_SPLLCSR_LK_MASK,
                   0U) == 0U)
    {
        return 0U;
    }

    /* SPLL disabled while configuration is changed. */
    SCG->SPLLCSR = 0x00000000U;
    SCG->SPLLCFG = 0x00180000U;

    if(prv_WaitSet(&SCG->SPLLCSR,
                   SCG_SPLLCSR_LK_MASK,
                   0U) == 0U)
    {
        return 0U;
    }

    SCG->SPLLCSR = 0x00000001U;

    if(prv_WaitSet(&SCG->SPLLCSR,
                   SCG_SPLLCSR_SPLLVLD_MASK,
                   SCG_SPLLCSR_SPLLVLD_MASK) == 0U)
    {
        return 0U;
    }

    /*
     * RCCR field layout:
     *   SCS      = 6  (SPLL)
     *   DIVCORE  = 1  (/2) -> 80MHz
     *   DIVBUS   = 1  (/2) -> 40MHz
     *   DIVSLOW  = 2  (/3) -> 26.67MHz
     *
     * 0x06010012 is the encoded value.
     */
    SCG->RCCR = 0x06010012U;

    if(prv_WaitSet(&SCG->CSR,
                   SCG_CSR_SCS_MASK,
                   (6UL << SCG_CSR_SCS_SHIFT)) == 0U)
    {
        return 0U;
    }

    return 1U;
}
