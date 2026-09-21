/*
 * boot_main.c
 *
 * Zitto_MB_V1 / S32K144
 *
 * Boot helper module.
 *
 * IMPORTANT:
 * This file must NOT contain main().
 *
 * The project entry point is:
 *
 *     src/main.c
 *
 */

#include "boot_main.h"
#include "DEBUG/debug_rtt.h"
#include <stdint.h>

volatile uint32_t g_last_exception_ipsr = 0U;

volatile uint32_t g_fault_cfsr = 0U;
volatile uint32_t g_fault_hfsr = 0U;
volatile uint32_t g_fault_mmfar = 0U;
volatile uint32_t g_fault_bfar = 0U;

volatile uint32_t g_fault_pc = 0U;
void Fault_Capture(void)
{
    volatile uint32_t *CFSR =
        (volatile uint32_t *)0xE000ED28UL;

    volatile uint32_t *HFSR =
        (volatile uint32_t *)0xE000ED2CUL;

    volatile uint32_t *MMFAR =
        (volatile uint32_t *)0xE000ED34UL;

    volatile uint32_t *BFAR =
        (volatile uint32_t *)0xE000ED38UL;


    g_fault_cfsr =
        *CFSR;

    g_fault_hfsr =
        *HFSR;

    g_fault_mmfar =
        *MMFAR;

    g_fault_bfar =
        *BFAR;
}
/* ============================================================================
 * Boot_MainInit
 *
 * Reserved for future bootloader / application handover logic.
 *
 * Current application directly starts from main.c.
 * ========================================================================== */
void Boot_MainInit(void)
{
    /*
     * Reserved.
     *
     * Future possibilities:
     *
     *  - Bootloader/application selection
     *  - OTA image validation
     *  - Application image CRC verification
     *  - Boot reason handling
     *  - Safe recovery mode
     *
     * Do not initialize application peripherals here yet.
     */
}
