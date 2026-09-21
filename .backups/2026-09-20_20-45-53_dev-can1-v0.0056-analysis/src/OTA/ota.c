#include "OTA/ota.h"
#include "../APP/app_config.h"
#include "../APP/app_status.h"
#include "../FLM/flm.h"
#include <string.h>
#include "DEBUG/debug_rtt.h"

/* ============================================================

* W25N01GV
* ============================================================ */
  #define OTA_PAGE_SIZE        2048U
  #define OTA_PAGES_PER_BLOCK    64U

/* OTA firmware area */
#define OTA_BLOCK_START       960U

/* Metadata uses final block */
#define OTA_META_BLOCK       1023U
#define OTA_META_PAGE        ((uint32_t)OTA_META_BLOCK*OTA_PAGES_PER_BLOCK)

/* Firmware first page */
#define OTA_FIRST_PAGE       ((uint32_t)OTA_BLOCK_START*OTA_PAGES_PER_BLOCK)

/* Total available firmware pages */
#define OTA_MAX_PAGES        ((uint32_t)OTA_BLOCK_COUNT*OTA_PAGES_PER_BLOCK)

/* Maximum firmware size */
#define OTA_MAX_SIZE         (OTA_MAX_PAGES*OTA_PAGE_SIZE)

/* ============================================================

* OTA METADATA
* ============================================================ */
  #define OTA_META_MAGIC       0x4F544131UL

#define OTA_META_EMPTY       0U
#define OTA_META_DOWNLOADING 1U
#define OTA_META_READY       2U

typedef struct
{
uint32_t magic;
uint32_t state;
uint32_t image_size;
uint32_t image_crc;

uint32_t received_size;

uint32_t meta_crc;
} OtaMeta_t;

/* ============================================================

* INTERNAL
* ============================================================ */
  static OtaState_t s_state;
  static OtaError_t s_error;

static uint32_t s_image_size;
static uint32_t s_image_crc;
static uint32_t s_received;

static uint32_t s_page;
static uint16_t s_page_pos;

static uint8_t s_page_buf[OTA_PAGE_SIZE];

/* ============================================================

* CRC32
*
* Standard CRC32
* Poly 0xEDB88320
* ============================================================ */
  static uint32_t prv_Crc32Update(uint32_t crc,const uint8_t *data,uint32_t len)
  {
  uint8_t b;

  while(len--)
  {
  crc^=*data++;
   for(b=0U;b<8U;b++)
   {
       if(crc&1U)
           crc=(crc>>1)^0xEDB88320UL;
       else
           crc>>=1;
   }
  }

  return crc;
  }

uint32_t OTA_Crc32(const uint8_t *data,uint32_t len)
{
if((data==0)&&(len>0U))
return 0U;
return prv_Crc32Update(0xFFFFFFFFUL,data,len)^0xFFFFFFFFUL;
}

/* ============================================================

* METADATA CRC
* ============================================================ */
  static uint32_t prv_MetaCrc(const OtaMeta_t *m)
  {
  return OTA_Crc32((const uint8_t *)m,
  (uint32_t)(sizeof(OtaMeta_t)-sizeof(uint32_t)));
  }

/* ============================================================

* WRITE METADATA
* ============================================================ */
  static uint8_t prv_WriteMeta(uint32_t state)
  {
  OtaMeta_t m;

  memset(&m,0xFF,sizeof(m));

  m.magic=OTA_META_MAGIC;
  m.state=state;

  m.image_size=s_image_size;
  m.image_crc=s_image_crc;

  m.received_size=s_received;

  m.meta_crc=prv_MetaCrc(&m);

  if(Flm_BlockErase(OTA_META_BLOCK)!=0)
  return 0U;

  if(Flm_PageWrite(OTA_META_PAGE,
  (const uint8_t *)&m,
  (uint16_t)sizeof(m))!=0)
  return 0U;

  return 1U;
  }

/* ============================================================

* READ METADATA
* ============================================================ */
  static uint8_t prv_ReadMeta(OtaMeta_t *m)
  {
  if(!m)
  return 0U;

  if(Flm_PageRead(OTA_META_PAGE,
  (uint8_t *)m,
  (uint16_t)sizeof(OtaMeta_t))!=0)
  return 0U;

  if(m->magic!=OTA_META_MAGIC)
  return 0U;

  if(m->meta_crc!=prv_MetaCrc(m))
  return 0U;

  return 1U;
  }

/* ============================================================

* ERASE OTA FIRMWARE AREA
* ============================================================ */
  static uint8_t prv_EraseImageArea(void)
  {
  uint32_t block;

  for(block=OTA_BLOCK_START;
  block<(OTA_BLOCK_START+OTA_BLOCK_COUNT);
  block++)
  {
  if(Flm_BlockErase(block)!=0)
  return 0U;
  }

  return 1U;
  }

/* ============================================================

* FLUSH CURRENT PAGE
* ============================================================ */
  static uint8_t prv_FlushPage(void)
  {
  uint16_t len;

  if(s_page_pos==0U)
  return 1U;

  len=s_page_pos;

  if(Flm_PageWrite(s_page,
  s_page_buf,
  len)!=0)
  return 0U;

  s_page++;
  s_page_pos=0U;

  memset(s_page_buf,0xFF,sizeof(s_page_buf));

  return 1U;
  }

/* ============================================================

* INIT
* ============================================================ */
  void OTA_Init(void)
  {
  OtaMeta_t m;

  s_state=OTA_IDLE;
  s_error=OTA_ERR_NONE;

  s_image_size=0U;
  s_image_crc=0U;
  s_received=0U;

  s_page=OTA_FIRST_PAGE;
  s_page_pos=0U;

  memset(s_page_buf,0xFF,sizeof(s_page_buf));

  if(prv_ReadMeta(&m))
  {
  if(m.state==OTA_META_READY)
  {
  s_image_size=m.image_size;
  s_image_crc=m.image_crc;
  s_received=m.received_size;
       s_state=OTA_READY;
   }
  }

  App_StatusSetOtaActive(0U);
  }

/* ============================================================

* START OTA
* ============================================================ */
  uint8_t OTA_Start(uint32_t image_size,uint32_t image_crc)
  {
  if(image_size==0U)
  {
  s_error=OTA_ERR_PARAM;
  return 0U;
  }

  if(image_size>OTA_MAX_SIZE)
  {
  s_error=OTA_ERR_SIZE;
  return 0U;
  }

  if((s_state==OTA_RECEIVING)||
  (s_state==OTA_VERIFYING))
  {
  s_error=OTA_ERR_STATE;
  return 0U;
  }

  s_state=OTA_RECEIVING;
  s_error=OTA_ERR_NONE;

  s_image_size=image_size;
  s_image_crc=image_crc;
  s_received=0U;

  s_page=OTA_FIRST_PAGE;
  s_page_pos=0U;

  memset(s_page_buf,0xFF,sizeof(s_page_buf));

  App_StatusSetOtaActive(1U);

  if(!prv_EraseImageArea())
  {
  s_state=OTA_ERROR;
  s_error=OTA_ERR_FLASH;
  App_StatusSetOtaActive(0U);
  return 0U;
  }

  if(!prv_WriteMeta(OTA_META_DOWNLOADING))
  {
  s_state=OTA_ERROR;
  s_error=OTA_ERR_FLASH;
  App_StatusSetOtaActive(0U);
  return 0U;
  }

  return 1U;
  }

/* ============================================================

* WRITE OTA CHUNK
* ============================================================ */
  uint8_t OTA_WriteChunk(const uint8_t *data,uint16_t len)
  {
  uint16_t copy;

  if(!data)
  {
  s_error=OTA_ERR_PARAM;
  return 0U;
  }

  if(s_state!=OTA_RECEIVING)
  {
  s_error=OTA_ERR_STATE;
  return 0U;
  }

  if((s_received+len)>s_image_size)
  {
  s_error=OTA_ERR_SIZE;
  return 0U;
  }

  while(len>0U)
  {
  copy=(uint16_t)(OTA_PAGE_SIZE-s_page_pos);
   if(copy>len)
       copy=len;

   memcpy(&s_page_buf[s_page_pos],data,copy);

   s_page_pos=(uint16_t)(s_page_pos+copy);

   data+=copy;
   len=(uint16_t)(len-copy);

   s_received+=copy;

   if(s_page_pos>=OTA_PAGE_SIZE)
   {
       if(!prv_FlushPage())
       {
           s_state=OTA_ERROR;
           s_error=OTA_ERR_FLASH;
           App_StatusSetOtaActive(0U);
           return 0U;
       }
   }
  }

  return 1U;
  }

/* ============================================================

* VERIFY COMPLETE IMAGE
* ============================================================ */
  static uint8_t prv_VerifyImage(void)
  {
  uint8_t buf[OTA_PAGE_SIZE];

  uint32_t remaining;
  uint32_t page;
  uint16_t len;

  uint32_t crc=0xFFFFFFFFUL;

  remaining=s_image_size;
  page=OTA_FIRST_PAGE;

  while(remaining>0U)
  {
  len=(remaining>OTA_PAGE_SIZE)?
  OTA_PAGE_SIZE:(uint16_t)remaining;
   if(Flm_PageRead(page,buf,len)!=0)
       return 0U;

   crc=prv_Crc32Update(crc,buf,len);

   remaining-=len;
   page++;
  }

  crc^=0xFFFFFFFFUL;

  return (crc==s_image_crc)?1U:0U;
  }

/* ============================================================

* FINISH OTA
* ============================================================ */
  uint8_t OTA_Finish(void)
  {
  if(s_state!=OTA_RECEIVING)
  {
  s_error=OTA_ERR_STATE;
  return 0U;
  }

  if(s_received!=s_image_size)
  {
  s_error=OTA_ERR_INCOMPLETE;
  return 0U;
  }

  s_state=OTA_VERIFYING;

  if(!prv_FlushPage())
  {
  s_state=OTA_ERROR;
  s_error=OTA_ERR_FLASH;
  App_StatusSetOtaActive(0U);
  return 0U;
  }

  if(!prv_VerifyImage())
  {
  s_state=OTA_ERROR;
  s_error=OTA_ERR_CRC;
  App_StatusSetOtaActive(0U);
  return 0U;
  }

  s_state=OTA_READY;

  if(!prv_WriteMeta(OTA_META_READY))
  {
  s_state=OTA_ERROR;
  s_error=OTA_ERR_FLASH;
  App_StatusSetOtaActive(0U);
  return 0U;
  }

  App_StatusSetOtaActive(0U);

  return 1U;
  }

/* ============================================================

* ABORT OTA
* ============================================================ */
  void OTA_Abort(void)
  {
  s_state=OTA_ABORTED;

  s_error=OTA_ERR_NONE;

  s_image_size=0U;
  s_image_crc=0U;
  s_received=0U;

  s_page=OTA_FIRST_PAGE;
  s_page_pos=0U;

  memset(s_page_buf,0xFF,sizeof(s_page_buf));

  prv_WriteMeta(OTA_META_EMPTY);

  App_StatusSetOtaActive(0U);
  }

/* ============================================================

* GET STATE
* ============================================================ */
  OtaState_t OTA_GetState(void)
  {
  return s_state;
  }

OtaError_t OTA_GetError(void)
{
return s_error;
}

uint8_t OTA_IsActive(void)
{
return ((s_state==OTA_RECEIVING)||
(s_state==OTA_VERIFYING))?1U:0U;
}

uint8_t OTA_IsPending(void)
{
return (s_state==OTA_READY)?1U:0U;
}

/* ============================================================

* GET INFORMATION
* ============================================================ */
  void OTA_GetInfo(OtaInfo_t *out)
  {
  if(!out)
  return;

  out->image_size=s_image_size;
  out->image_crc=s_image_crc;
  out->bytes_received=s_received;
  out->bytes_written=s_received;

  out->current_page=(uint16_t)s_page;

  out->state=(uint8_t)s_state;
  out->error=(uint8_t)s_error;

  out->active=OTA_IsActive();
  out->pending=OTA_IsPending();
  }

/* ============================================================

* CLEAR OTA PENDING
*
* Called by boot code after successful firmware installation.
* ============================================================ */
  uint8_t OTA_ClearPending(void)
  {
  if(!prv_WriteMeta(OTA_META_EMPTY))
  return 0U;

  if(s_state==OTA_READY)
  s_state=OTA_IDLE;

  return 1U;
  }

/* ============================================================

* OTA BACKGROUND TASK
* ============================================================ */
  void OTA_Task(void)
  {
  }
