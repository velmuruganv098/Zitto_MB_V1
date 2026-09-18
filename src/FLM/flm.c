#include "flm.h"

#include <string.h>
#include "FLM/flm.h"
#include "DEBUG/debug_rtt.h"

#if APP_FLM_ENABLE

Debug_RTT_Print(
    "[MAIN] FLM INIT\r\n");

Flm_Init();

Debug_RTT_Print(
    "[MAIN] FLM READY\r\n");

#endif


/* ============================================================
 *
 * RECORD FORMAT
 *
 * Each record occupies one NAND page.
 *
 * BYTE 0..3    MAGIC
 * BYTE 4..5    LEN
 * BYTE 6..9    CRC32
 * BYTE 10..    DATA
 *
 * ============================================================ */

#define FLM_REC_MAGIC       0x464C4D31UL

#define FLM_REC_HDR_SIZE    10U

#define FLM_REC_MAX_DATA    \
    (FLM_FLASH_PAGE_SIZE - FLM_REC_HDR_SIZE)


/* ============================================================
 *
 * INTERNAL STATUS
 *
 * ============================================================ */

static uint32_t s_next_page;

static uint32_t s_last_page;

static uint32_t s_records;

static uint8_t s_ready;
static uint8_t s_scanning;
static uint32_t s_scan_page;


/* ============================================================
 *
 * PAGE BUFFER
 *
 * Single reusable page buffer.
 *
 * ============================================================ */

static uint8_t s_page_buf[FLM_FLASH_PAGE_SIZE];


/* ============================================================
 *
 * RECORD HEADER
 *
 * ============================================================ */

typedef struct
{
    uint32_t magic;

    uint16_t len;

    uint32_t crc;

} FlmRecHdr_t;


/* ============================================================
 *
 * LOW LEVEL FLASH DRIVER HOOKS
 *
 * IMPORTANT:
 *
 * Connect these functions to your existing working W25N01GV
 * driver implementation.
 *
 * ============================================================ */

static uint8_t prv_HwPageWrite(
    uint32_t page,
    const uint8_t *data,
    uint16_t len
)
{
    /*
     * Replace this body with your existing NAND page program
     * function.
     */

    (void)page;
    (void)data;
    (void)len;

    return 0U;
}


static uint8_t prv_HwPageRead(
    uint32_t page,
    uint8_t *data,
    uint16_t len
)
{
    /*
     * Replace this body with your existing NAND page read
     * function.
     */

    (void)page;
    (void)data;
    (void)len;

    return 0U;
}


static uint8_t prv_HwBlockErase(
    uint32_t block
)
{
    /*
     * Replace this body with your existing NAND block erase
     * function.
     */

    (void)block;

    return 0U;
}


/* ============================================================
 *
 * CRC32
 *
 * ============================================================ */

static uint32_t prv_Crc32(
    const uint8_t *data,
    uint32_t len
)
{
    uint32_t crc;

    uint8_t i;


    crc = 0xFFFFFFFFUL;


    while (len > 0U)
    {
        crc ^= (uint32_t)(*data);

        data++;

        for (i = 0U; i < 8U; i++)
        {
            if ((crc & 1UL) != 0UL)
            {
                crc =
                    (crc >> 1U)
                    ^
                    0xEDB88320UL;
            }
            else
            {
                crc >>= 1U;
            }
        }

        len--;
    }


    return crc ^ 0xFFFFFFFFUL;
}


/* ============================================================
 *
 * USER BLOCK CHECK
 *
 * ============================================================ */

bool Flm_IsUserBlock(
    uint32_t block
)
{
    if (block < FLM_USER_BLOCK_FIRST)
    {
        return false;
    }


    if (block > FLM_USER_BLOCK_LAST)
    {
        return false;
    }


    return true;
}


/* ============================================================
 *
 * USER PAGE CHECK
 *
 * ============================================================ */

bool Flm_IsUserPage(
    uint32_t page
)
{
    if (page < FLM_USER_FIRST_PAGE)
    {
        return false;
    }


    if (page > FLM_USER_LAST_PAGE)
    {
        return false;
    }


    return true;
}


/* ============================================================
 *
 * RAW PAGE WRITE
 *
 * Used by OTA.
 *
 * ============================================================ */

uint8_t Flm_PageWrite(
    uint32_t page,
    const uint8_t *data,
    uint16_t len
)
{
    if (data == NULL)
    {
        return 1U;
    }


    if (page >= FLM_FLASH_TOTAL_PAGES)
    {
        return 1U;
    }


    if (len == 0U)
    {
        return 1U;
    }


    if (len > FLM_FLASH_PAGE_SIZE)
    {
        return 1U;
    }


    return prv_HwPageWrite(
        page,
        data,
        len
    );
}


/* ============================================================
 *
 * RAW PAGE READ
 *
 * ============================================================ */

uint8_t Flm_PageRead(
    uint32_t page,
    uint8_t *data,
    uint16_t len
)
{
    if (data == NULL)
    {
        return 1U;
    }


    if (page >= FLM_FLASH_TOTAL_PAGES)
    {
        return 1U;
    }


    if (len == 0U)
    {
        return 1U;
    }


    if (len > FLM_FLASH_PAGE_SIZE)
    {
        return 1U;
    }


    return prv_HwPageRead(
        page,
        data,
        len
    );
}


/* ============================================================
 *
 * RAW BLOCK ERASE
 *
 * ============================================================ */

uint8_t Flm_BlockErase(
    uint32_t block
)
{
    if (block >= FLM_FLASH_BLOCK_COUNT)
    {
        return 1U;
    }


    return prv_HwBlockErase(
        block
    );
}


/* ============================================================
 *
 * CHECK PAGE EMPTY
 *
 * NAND erased state = 0xFF.
 *
 * ============================================================ */

static uint8_t prv_PageEmpty(
    uint32_t page
)
{
    uint16_t i;


    if (Flm_PageRead(
            page,
            s_page_buf,
            16U
        ) != 0U)
    {
        return 0U;
    }


    for (i = 0U; i < 16U; i++)
    {
        if (s_page_buf[i] != 0xFFU)
        {
            return 0U;
        }
    }


    return 1U;
}


/* ============================================================
 *
 * VALIDATE RECORD
 *
 * ============================================================ */

static uint8_t prv_RecordValid(
    uint32_t page,
    FlmRecHdr_t *hdr
)
{
    uint32_t crc;


    if (hdr == NULL)
    {
        return 0U;
    }


    if (Flm_PageRead(
            page,
            (uint8_t *)hdr,
            FLM_REC_HDR_SIZE
        ) != 0U)
    {
        return 0U;
    }


    if (hdr->magic != FLM_REC_MAGIC)
    {
        return 0U;
    }


    if (hdr->len == 0U)
    {
        return 0U;
    }


    if (hdr->len > FLM_REC_MAX_DATA)
    {
        return 0U;
    }


    if (Flm_PageRead(
            page,
            s_page_buf,
            (uint16_t)
            (
                FLM_REC_HDR_SIZE
                +
                hdr->len
            )
        ) != 0U)
    {
        return 0U;
    }


    crc = prv_Crc32(
        &s_page_buf[FLM_REC_HDR_SIZE],
        hdr->len
    );


    if (crc != hdr->crc)
    {
        return 0U;
    }


    return 1U;
}


/* ============================================================
 *
 * FIND LAST VALID RECORD
 *
 * ============================================================ */

static uint8_t prv_FindPreviousRecord(
    uint32_t from_page,
    uint32_t *found_page
)
{
    uint32_t page;

    FlmRecHdr_t hdr;


    if (found_page == NULL)
    {
        return 0U;
    }


    page = from_page;


    while (page > FLM_USER_FIRST_PAGE)
    {
        page--;


        if (prv_RecordValid(
                page,
                &hdr
            ) != 0U)
        {
            *found_page = page;

            return 1U;
        }
    }


    return 0U;
}


/* ============================================================
 *
 * INITIALIZE
 *
 * Scans only user storage area.
 *
 * OTA region is never scanned.
 *
 * ============================================================ */

void Flm_Init(void)
{
    s_next_page = FLM_USER_FIRST_PAGE;
    s_last_page = 0U;
    s_records = 0U;
    s_ready = 0U;
    s_scanning = 1U;
    s_scan_page = FLM_USER_FIRST_PAGE;

    memset(s_page_buf, 0xFF, sizeof(s_page_buf));

    RTT_LOG("[FLM] Init started; background scan active\\r\\n");
}



/* ============================================================
 *
 * WRITE USER RECORD
 *
 * ============================================================ */

FlmResult_t Flm_Write(
    const uint8_t *data,
    uint16_t len
)
{
    FlmRecHdr_t hdr;

    uint32_t block;


    if (data == NULL)
    {
        return FLM_ERR_PARAM;
    }


    if (len == 0U)
    {
        return FLM_ERR_PARAM;
    }


    if (len > FLM_REC_MAX_DATA)
    {
        return FLM_ERR_PARAM;
    }


    if (s_ready == 0U)
    {
        return FLM_ERR_FLASH;
    }


    if (s_next_page > FLM_USER_LAST_PAGE)
    {
        return FLM_ERR_FULL;
    }


    block =
        s_next_page
        /
        FLM_PAGES_PER_BLOCK;


    if (!Flm_IsUserBlock(block))
    {
        return FLM_ERR_PROTECTED;
    }


    /*
     * Important:
     *
     * NAND page must be empty before programming.
     */

    if (prv_PageEmpty(s_next_page) == 0U)
    {
        return FLM_ERR_FLASH;
    }


    memset(
        s_page_buf,
        0xFF,
        sizeof(s_page_buf)
    );


    hdr.magic =
        FLM_REC_MAGIC;

    hdr.len =
        len;

    hdr.crc =
        prv_Crc32(
            data,
            len
        );


    memcpy(
        &s_page_buf[0],
        &hdr,
        FLM_REC_HDR_SIZE
    );


    memcpy(
        &s_page_buf[FLM_REC_HDR_SIZE],
        data,
        len
    );


    if (Flm_PageWrite(
            s_next_page,
            s_page_buf,
            (uint16_t)
            (
                FLM_REC_HDR_SIZE
                +
                len
            )
        ) != 0U)
    {
        return FLM_ERR_FLASH;
    }


    /*
     * Verify programmed record.
     */

    if (prv_RecordValid(
            s_next_page,
            &hdr
        ) == 0U)
    {
        return FLM_ERR_FLASH;
    }


    s_last_page =
        s_next_page;

    s_next_page++;

    s_records++;


    return FLM_OK;
}


/* ============================================================
 *
 * READ LAST USER RECORD
 *
 * ============================================================ */

FlmResult_t Flm_Read(
    uint8_t *data,
    uint16_t max_len,
    uint16_t *out_len
)
{
    FlmRecHdr_t hdr;


    if (data == NULL)
    {
        return FLM_ERR_PARAM;
    }


    if (out_len == NULL)
    {
        return FLM_ERR_PARAM;
    }


    if (s_ready == 0U)
    {
        return FLM_ERR_FLASH;
    }


    if (s_records == 0U)
    {
        return FLM_ERR_EMPTY;
    }


    if (prv_RecordValid(
            s_last_page,
            &hdr
        ) == 0U)
    {
        return FLM_ERR_FLASH;
    }


    if (max_len < hdr.len)
    {
        return FLM_ERR_PARAM;
    }


    if (Flm_PageRead(
            s_last_page,
            s_page_buf,
            (uint16_t)
            (
                FLM_REC_HDR_SIZE
                +
                hdr.len
            )
        ) != 0U)
    {
        return FLM_ERR_FLASH;
    }


    memcpy(
        data,
        &s_page_buf[FLM_REC_HDR_SIZE],
        hdr.len
    );


    *out_len =
        hdr.len;


    return FLM_OK;
}


/* ============================================================
 *
 * DELETE LAST RECORD
 *
 * NOTE:
 *
 * NAND cannot erase individual pages.
 *
 * This implementation provides logical delete.
 *
 * Physical block erase occurs only when the entire block
 * becomes unused.
 *
 * ============================================================ */

FlmResult_t Flm_Delete(void)
{
    uint32_t block;

    uint32_t block_first;

    uint32_t previous_page;



    if (s_ready == 0U)
    {
        return FLM_ERR_FLASH;
    }


    if (s_records == 0U)
    {
        return FLM_ERR_EMPTY;
    }


    block =
        s_last_page
        /
        FLM_PAGES_PER_BLOCK;


    if (!Flm_IsUserBlock(block))
    {
        return FLM_ERR_PROTECTED;
    }


    block_first =
        block
        *
        FLM_PAGES_PER_BLOCK;


    /*
     * If deleting the first record in the block,
     * the whole block can be erased.
     */

    if (s_last_page == block_first)
    {
        if (Flm_BlockErase(block) != 0U)
        {
            return FLM_ERR_FLASH;
        }
    }


    /*
     * Find previous valid record.
     */

    if (prv_FindPreviousRecord(
            s_last_page,
            &previous_page
        ) != 0U)
    {
        s_last_page =
            previous_page;

        s_next_page =
            previous_page + 1U;

        s_records--;


        return FLM_OK;
    }


    /*
     * No previous record exists.
     */

    s_last_page =
        0U;

    s_next_page =
        FLM_USER_FIRST_PAGE;

    s_records =
        0U;


    return FLM_OK;
}


/* ============================================================
 *
 * GET FLASH INFORMATION
 *
 * ============================================================ */

void Flm_GetInfo(
    FlmInfo_t *out
)
{
    uint32_t used;


    if (out == NULL)
    {
        return;
    }


    used =
        s_records;


    out->total_pages =
        FLM_USER_TOTAL_PAGES;

    out->used_pages =
        used;


    if (used >= FLM_USER_TOTAL_PAGES)
    {
        out->free_pages =
            0U;
    }
    else
    {
        out->free_pages =
            FLM_USER_TOTAL_PAGES
            -
            used;
    }


    out->next_page =
        s_next_page;

    out->last_page =
        s_last_page;

    out->records =
        s_records;
}


/* ============================================================
 *
 * GET FREE PAGES
 *
 * ============================================================ */

uint32_t Flm_GetFreePages(void)
{
    if (s_records >= FLM_USER_TOTAL_PAGES)
    {
        return 0U;
    }


    return
        FLM_USER_TOTAL_PAGES
        -
        s_records;
}


/* ============================================================
 *
 * GET USED PAGES
 *
 * ============================================================ */

uint32_t Flm_GetUsedPages(void)
{
    return s_records;
}


/* ============================================================
 *
 * GET RECORD COUNT
 *
 * ============================================================ */

uint32_t Flm_GetRecordCount(void)
{
    return s_records;
}


/* ============================================================
 *
 * READY STATUS
 *
 * ============================================================ */

uint8_t Flm_IsReady(void)
{
    return s_ready;
}


/* ============================================================
 *
 * BACKGROUND TASK
 *
 * ============================================================ */

void Flm_Task(void)
{
    uint8_t budget = 2U;
    FlmRecHdr_t hdr;

    if(!s_scanning || s_ready) return;

    while((budget-- != 0U) && (s_scan_page <= FLM_USER_LAST_PAGE))
    {
        if(prv_PageEmpty(s_scan_page) != 0U)
        {
            s_next_page = s_scan_page;
            s_ready = 1U;
            s_scanning = 0U;
            RTT_LOG("[FLM] Scan complete records=%lu next=%lu\\r\\n",
                    (unsigned long)s_records,
                    (unsigned long)s_next_page);
            return;
        }

        if(prv_RecordValid(s_scan_page, &hdr) != 0U)
        {
            s_last_page = s_scan_page;
            s_records++;
            s_next_page = s_scan_page + 1U;
        }
        else
        {
            s_next_page = s_scan_page;
            s_ready = 1U;
            s_scanning = 0U;
            RTT_LOG("[FLM_ERR] Invalid page %lu; scan stopped safely\\r\\n",
                    (unsigned long)s_scan_page);
            return;
        }

        s_scan_page++;
    }

    if(s_scan_page > FLM_USER_LAST_PAGE)
    {
        s_next_page = FLM_USER_LAST_PAGE + 1U;
        s_ready = 1U;
        s_scanning = 0U;
        RTT_LOG("[FLM] Scan complete: storage full records=%lu\\r\\n",
                (unsigned long)s_records);
    }
}
