#include "cmd.h"

#include "../APP/app_modules.h"
#include "../APP/app_status.h"

#include "../OTA/ota.h"
#include "../FLM/flm.h"
#include "../APP/app_main.h"
#include "uart_pkt_types.h"

#include <string.h>

/* ============================================================

* INTERNAL
* ============================================================ */

static void prv_SendAck(uint8_t cmd,uint8_t seq,uint8_t result)
{
uint8_t d[3];
d[0]=cmd;
d[1]=seq;
d[2]=result;

Uart_Pkt_Send(CMD_TYPE_ACK,d,sizeof(d));
}

static void prv_SendNack(uint8_t cmd,uint8_t seq,uint8_t result)
{
uint8_t d[3];
d[0]=cmd;
d[1]=seq;
d[2]=result;

Uart_Pkt_Send(CMD_TYPE_NACK,d,sizeof(d));
}

static uint32_t prv_GetU32(const uint8_t *d)
{
return ((uint32_t)d[0]<<24)|
((uint32_t)d[1]<<16)|
((uint32_t)d[2]<<8)|
((uint32_t)d[3]);
}

static void prv_PutU32(uint8_t *d,uint32_t v)
{
d[0]=(uint8_t)(v>>24);
d[1]=(uint8_t)(v>>16);
d[2]=(uint8_t)(v>>8);
d[3]=(uint8_t)v;
}

/* ============================================================

* MODULE ENABLE / DISABLE
*
* PAYLOAD:
*
* BYTE 0 = MODULE ID
* BYTE 1 = ENABLE
*
* 0 = DISABLE
* 1 = ENABLE
* ============================================================ */

static void prv_ModuleEn(const UartPkt_t *pkt)
{
AppModule_t module;
uint8_t enable;
if(pkt->len!=2U)
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_PARAM);
    return;
}

module=(AppModule_t)pkt->data[0];
enable=pkt->data[1];

if(enable>1U)
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_PARAM);
    return;
}

if(module>=APP_MODULE_MAX)
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_MODULE);
    return;
}

if(!App_ModuleSet(module,enable))
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_MODULE);
    return;
}

prv_SendAck(pkt->type,pkt->seq,CMD_OK);
}

/* ============================================================

* STATUS REQUEST
*
* PAYLOAD:
* NONE
* ============================================================ */

static void prv_StatusReq(const UartPkt_t *pkt)
{
if(pkt->len!=0U)
{
prv_SendNack(pkt->type,pkt->seq,CMD_ERR_PARAM);
return;
}
StatusPkt_t status;

App_GetStatus(&status);

Uart_Pkt_SendStatus(&status);

prv_SendAck(pkt->type,pkt->seq,CMD_OK);
}

/* ============================================================

* FLASH WRITE
*
* PAYLOAD:
*
* RAW USER DATA
*
* The complete payload becomes one FLM record.
*
* User commands can never directly access:
*
* Block 0
* OTA Blocks 960-1022
* Metadata Block 1023
* ============================================================ */

static void prv_FlashWrite(const UartPkt_t *pkt)
{
FlmResult_t r;
if(pkt->len==0U)
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_PARAM);
    return;
}

if(!Flm_IsReady())
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_FLASH);
    return;
}

r=Flm_Write(pkt->data,pkt->len);

if(r!=FLM_OK)
{
    if(r==FLM_ERR_FULL)
        prv_SendNack(pkt->type,pkt->seq,CMD_ERR_STATE);
    else
        prv_SendNack(pkt->type,pkt->seq,CMD_ERR_FLASH);

    return;
}

prv_SendAck(pkt->type,pkt->seq,CMD_OK);
}

/* ============================================================

* FLASH READ
*
* PAYLOAD:
* NONE
*
* RESPONSE:
*
* RESULT[1]
* LEN_H[1]
* LEN_L[1]
* DATA[N]
*
* ============================================================ */

static void prv_FlashRead(const UartPkt_t *pkt)
{
static uint8_t buf[UART_MAX_PAYLOAD];
uint16_t len=0U;
FlmResult_t r;

uint8_t tx[UART_MAX_PAYLOAD + 3U];

if(pkt->len!=0U)
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_PARAM);
    return;
}

if(!Flm_IsReady())
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_FLASH);
    return;
}

r=Flm_Read(buf,sizeof(buf),&len);

if(r!=FLM_OK)
{
    if(r==FLM_ERR_EMPTY)
        prv_SendNack(pkt->type,pkt->seq,CMD_ERR_EMPTY);
    else
        prv_SendNack(pkt->type,pkt->seq,CMD_ERR_FLASH);

    return;
}

tx[0]=CMD_OK;
tx[1]=(uint8_t)(len>>8);
tx[2]=(uint8_t)len;

memcpy(&tx[3],buf,len);

Uart_Pkt_Send(CMD_TYPE_FLASH_DATA,
              tx,
              (uint16_t)(len+3U));

prv_SendAck(pkt->type,pkt->seq,CMD_OK);
}

/* ============================================================

* FLASH DELETE
*
* PAYLOAD:
* NONE
*
* Deletes the latest FLM record.
*
* FLM controls the NAND-specific physical erase behaviour.
* ============================================================ */

static void prv_FlashDelete(const UartPkt_t *pkt)
{
FlmResult_t r;
if(pkt->len!=0U)
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_PARAM);
    return;
}

if(!Flm_IsReady())
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_FLASH);
    return;
}

r=Flm_Delete();

if(r!=FLM_OK)
{
    if(r==FLM_ERR_EMPTY)
        prv_SendNack(pkt->type,pkt->seq,CMD_ERR_EMPTY);
    else
        prv_SendNack(pkt->type,pkt->seq,CMD_ERR_FLASH);

    return;
}

prv_SendAck(pkt->type,pkt->seq,CMD_OK);
}

/* ============================================================

* OTA START
*
* PAYLOAD:
*
* IMAGE_SIZE[4]
* IMAGE_CRC32[4]
* ============================================================ */

static void prv_OtaStart(const UartPkt_t *pkt)
{
uint32_t size;
uint32_t crc;
if(pkt->len!=8U)
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_PARAM);
    return;
}

size=prv_GetU32(&pkt->data[0]);
crc=prv_GetU32(&pkt->data[4]);

if(!OTA_Start(size,crc))
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_OTA);
    return;
}

prv_SendAck(pkt->type,pkt->seq,CMD_OK);
}

/* ============================================================

* OTA DATA
* ============================================================ */

static void prv_OtaData(const UartPkt_t *pkt)
{
if(pkt->len==0U)
{
prv_SendNack(pkt->type,pkt->seq,CMD_ERR_PARAM);
return;
}
if(!OTA_WriteChunk(pkt->data,pkt->len))
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_OTA);
    return;
}

prv_SendAck(pkt->type,pkt->seq,CMD_OK);
}

/* ============================================================

* OTA FINISH
* ============================================================ */

static void prv_OtaFinish(const UartPkt_t *pkt)
{
if(pkt->len!=0U)
{
prv_SendNack(pkt->type,pkt->seq,CMD_ERR_PARAM);
return;
}
if(!OTA_Finish())
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_OTA);
    return;
}

prv_SendAck(pkt->type,pkt->seq,CMD_OK);
}

/* ============================================================

* OTA ABORT
* ============================================================ */

static void prv_OtaAbort(const UartPkt_t *pkt)
{
if(pkt->len!=0U)
{
prv_SendNack(pkt->type,pkt->seq,CMD_ERR_PARAM);
return;
}
OTA_Abort();

prv_SendAck(pkt->type,pkt->seq,CMD_OK);
}

/* ============================================================

* OTA INFO
*
* RESPONSE:
*
* STATE[1]
* ERROR[1]
* ACTIVE[1]
* PENDING[1]
* IMAGE_SIZE[4]
* IMAGE_CRC[4]
* RECEIVED[4]
* ============================================================ */

static void prv_OtaInfo(const UartPkt_t *pkt)
{
OtaInfo_t info;
uint8_t d[16];
if(pkt->len!=0U)
{
    prv_SendNack(pkt->type,pkt->seq,CMD_ERR_PARAM);
    return;
}

OTA_GetInfo(&info);

d[0]=info.state;
d[1]=info.error;
d[2]=info.active;
d[3]=info.pending;

prv_PutU32(&d[4],info.image_size);
prv_PutU32(&d[8],info.image_crc);
prv_PutU32(&d[12],info.bytes_received);

Uart_Pkt_Send(CMD_TYPE_OTA_INFO,d,sizeof(d));

prv_SendAck(pkt->type,pkt->seq,CMD_OK);
}

/* ============================================================

* INIT
* ============================================================ */

void Cmd_Init(void)
{
Uart_Pkt_SetRxCallback(Cmd_RxPacket);
}

/* ============================================================

* COMMAND DISPATCHER
* ============================================================ */

void Cmd_RxPacket(const UartPkt_t *pkt)
{
if(!pkt)
return;
switch(pkt->type)
{
    case CMD_MODULE_EN:
        prv_ModuleEn(pkt);
        break;

    case CMD_STATUS_REQ:
        prv_StatusReq(pkt);
        break;

    case CMD_FLASH_RD:
        prv_FlashRead(pkt);
        break;

    case CMD_FLASH_WR:
        prv_FlashWrite(pkt);
        break;

    case CMD_FLASH_DEL:
        prv_FlashDelete(pkt);
        break;

    case CMD_OTA_START:
        prv_OtaStart(pkt);
        break;

    case CMD_OTA_DATA:
        prv_OtaData(pkt);
        break;

    case CMD_OTA_FINISH:
        prv_OtaFinish(pkt);
        break;

    case CMD_OTA_ABORT:
        prv_OtaAbort(pkt);
        break;

    case CMD_OTA_INFO:
        prv_OtaInfo(pkt);
        break;

    default:
        prv_SendNack(pkt->type,
                     pkt->seq,
                     CMD_ERR_UNSUPPORTED);
        break;
}
}
