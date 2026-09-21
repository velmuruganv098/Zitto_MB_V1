#ifndef FLM_H
#define FLM_H

#include <stdint.h>
#include <stdbool.h>

/*
 * ============================================================
 * Flash Logging Manager
 *
 * Device:
 * S32K144 + W25N01GV NAND Flash
 *
 * ============================================================
 */


/* ------------------------------------------------------------
 * Flash Geometry
 *
 * W25N01GV:
 *
 * 2048 bytes per page
 * 2 pages per block
 *
 * IMPORTANT:
 * Update FLM_FLASH_BLOCK_COUNT according to the physical
 * flash geometry used by your driver.
 * ------------------------------------------------------------ */

#define FLM_FLASH_PAGE_SIZE             2048U

#define FLM_PAGES_PER_BLOCK             2U

/*
 * Total physical flash blocks.
 *
 * Your current allocation uses blocks 0 to 63.
 * Therefore minimum required value is 64.
 *
 * If your existing W25N01GV driver uses the complete device,
 * replace this with the actual total block count.
 */
#define FLM_FLASH_BLOCK_COUNT           1024U


#define FLM_FLASH_TOTAL_PAGES           \
    (FLM_FLASH_BLOCK_COUNT * FLM_PAGES_PER_BLOCK)


/* ------------------------------------------------------------
 * User Flash Region
 *
 * Block 0 is reserved for OTA metadata/system use.
 *
 * User logging uses blocks 1 to 63.
 * ------------------------------------------------------------ */

#define FLM_USER_BLOCK_FIRST            1U

#define FLM_USER_BLOCK_COUNT            63U

#define FLM_USER_BLOCK_LAST             \
    (FLM_USER_BLOCK_FIRST + FLM_USER_BLOCK_COUNT - 1U)


/* ------------------------------------------------------------
 * Derived Page Values
 * ------------------------------------------------------------ */

#define FLM_USER_FIRST_PAGE             \
    (FLM_USER_BLOCK_FIRST * FLM_PAGES_PER_BLOCK)

#define FLM_USER_LAST_PAGE              \
    (((FLM_USER_BLOCK_LAST + 1U) * FLM_PAGES_PER_BLOCK) - 1U)

#define FLM_USER_TOTAL_PAGES            \
    (FLM_USER_BLOCK_COUNT * FLM_PAGES_PER_BLOCK)


/* ------------------------------------------------------------
 * FLM Result
 * ------------------------------------------------------------ */

typedef enum
{
    FLM_OK = 0,

    FLM_ERR_PARAM,

    FLM_ERR_EMPTY,

    FLM_ERR_FULL,

    FLM_ERR_FLASH,

    FLM_ERR_PROTECTED

} FlmResult_t;


/* ------------------------------------------------------------
 * FLM Information
 * ------------------------------------------------------------ */

typedef struct
{
    uint32_t total_pages;

    uint32_t used_pages;

    uint32_t free_pages;

    uint32_t next_page;

    uint32_t last_page;

    uint32_t records;

} FlmInfo_t;


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

void Flm_Init(void);


/* ============================================================
 * USER RECORD API
 * ============================================================ */

FlmResult_t Flm_Write(
    const uint8_t *data,
    uint16_t len
);


FlmResult_t Flm_Read(
    uint8_t *data,
    uint16_t max_len,
    uint16_t *out_len
);


FlmResult_t Flm_Delete(void);


/* ============================================================
 * USER RANGE CHECK
 * ============================================================ */

bool Flm_IsUserBlock(
    uint32_t block
);


bool Flm_IsUserPage(
    uint32_t page
);


/* ============================================================
 * RAW FLASH API
 *
 * Used by OTA.
 *
 * These APIs can access the complete physical flash.
 * Protection of OTA/user regions must be handled by the caller
 * or by higher-level OTA logic.
 * ============================================================ */

uint8_t Flm_PageWrite(
    uint32_t page,
    const uint8_t *data,
    uint16_t len
);


uint8_t Flm_PageRead(
    uint32_t page,
    uint8_t *data,
    uint16_t len
);


uint8_t Flm_BlockErase(
    uint32_t block
);


/* ============================================================
 * STATUS
 * ============================================================ */

void Flm_GetInfo(
    FlmInfo_t *out
);


uint32_t Flm_GetFreePages(void);


uint32_t Flm_GetUsedPages(void);


uint32_t Flm_GetRecordCount(void);


uint8_t Flm_IsReady(void);


/* ============================================================
 * BACKGROUND TASK
 * ============================================================ */

void Flm_Task(void);


#endif /* FLM_H */
