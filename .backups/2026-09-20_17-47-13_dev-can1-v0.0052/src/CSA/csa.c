#include "csa.h"
#include "S32K144.h"
#include "SEGGER_RTT.h"
#include "DEBUG/debug_rtt.h"

#define CSA_CURRENT_LSB_NA ((uint32_t)((uint32_t)CSA_MAX_CURRENT_MA*1000000UL/32768UL))
#define CSA_CAL_VALUE ((uint16_t)(5120000000000ULL/((uint64_t)CSA_CURRENT_LSB_NA*(uint64_t)CSA_SHUNT_UOHM)))

#define CSA_SDA 2U
#define CSA_SCL 3U
#define CSA_DLY 5U

static Csa_Data_t s_data={0};
static uint32_t s_taskCnt=0U;

#if APP_CSA_ENABLE

Debug_RTT_Print(
    "[MAIN] CSA INIT\r\n");

Csa_Init();

Debug_RTT_Print(
    "[MAIN] CSA READY\r\n");

#endif


static void csa_Us(volatile uint32_t us)
{
while(us--){volatile uint32_t n=80U;while(n--){;}}
}

static void csa_Ms(volatile uint32_t ms)
{
while(ms--){volatile uint32_t n=80000U;while(n--){;}}
}

static int32_t csa_Abs(int32_t x)
{
return x<0?-x:x;
}

static void csa_SdaL(void)
{
PTA->PDDR|=(1UL<<CSA_SDA);
PTA->PCOR=(1UL<<CSA_SDA);
}

static void csa_SdaH(void)
{
PTA->PDDR&=~(1UL<<CSA_SDA);
}

static void csa_SclL(void)
{
PTA->PDDR|=(1UL<<CSA_SCL);
PTA->PCOR=(1UL<<CSA_SCL);
}

static void csa_SclH(void)
{
volatile uint32_t t=5000U;
PTA->PDDR&=~(1UL<<CSA_SCL);
while(!(PTA->PDIR&(1UL<<CSA_SCL))&&--t){;}
csa_Us(CSA_DLY);
}

static uint8_t csa_SdaR(void)
{
return(uint8_t)((PTA->PDIR>>CSA_SDA)&1U);
}

static void csa_Start(void)
{
csa_SdaH();
csa_Us(CSA_DLY);
csa_SclH();
csa_SdaL();
csa_Us(CSA_DLY);
csa_SclL();
csa_Us(CSA_DLY);
}

static void csa_Stop(void)
{
csa_SdaL();
csa_Us(CSA_DLY);
csa_SclH();
csa_SdaH();
csa_Us(CSA_DLY);
}

static void csa_WBit(uint8_t b)
{
if(b)csa_SdaH();
else csa_SdaL();
csa_Us(CSA_DLY);
csa_SclH();
csa_SclL();
csa_Us(CSA_DLY);
}

static uint8_t csa_RBit(void)
{
uint8_t b;
csa_SdaH();
csa_Us(CSA_DLY);
csa_SclH();
b=csa_SdaR();
csa_SclL();
csa_Us(CSA_DLY);
return b;
}

static uint8_t csa_WByte(uint8_t d)
{
int8_t i;
for(i=7;i>=0;i--)csa_WBit((uint8_t)((d>>(uint8_t)i)&1U));
return(uint8_t)!csa_RBit();
}

static uint8_t csa_RByte(uint8_t ack)
{
uint8_t d=0U;
int8_t i;
for(i=7;i>=0;i--)d=(uint8_t)((d<<1U)|csa_RBit());
csa_WBit((uint8_t)!ack);
return d;
}

static int csa_WrReg(uint8_t reg,uint16_t val)
{
uint8_t a1,a2,a3,a4;


csa_Start();
a1=csa_WByte((uint8_t)((CSA_I2C_ADDR<<1U)|0U));
a2=csa_WByte(reg);
a3=csa_WByte((uint8_t)(val>>8U));
a4=csa_WByte((uint8_t)(val&0xFFU));
csa_Stop();

return(a1&&a2&&a3&&a4)?0:-1;


}

static int csa_RdReg(uint8_t reg,uint16_t *val)
{
uint8_t a1,a2,msb,lsb;


if(!val)return-1;

csa_Start();
a1=csa_WByte((uint8_t)((CSA_I2C_ADDR<<1U)|0U));
a2=csa_WByte(reg);
csa_Stop();

if(!a1||!a2)return-1;

csa_Us(10U);
csa_Start();

if(!csa_WByte((uint8_t)((CSA_I2C_ADDR<<1U)|1U))){
    csa_Stop();
    return-2;
}

msb=csa_RByte(1U);
lsb=csa_RByte(0U);

csa_Stop();

*val=(uint16_t)(((uint16_t)msb<<8U)|lsb);

return 0;


}

static uint8_t csa_Probe(void)
{
uint8_t ack;


csa_Start();
ack=csa_WByte((uint8_t)((CSA_I2C_ADDR<<1U)|0U));
csa_Stop();

return ack;


}

static void csa_ReadMeasurement(void)
{
uint16_t r_vbus=0U;
uint16_t r_power=0U;
uint16_t r_masken=0U;
uint16_t tmp=0U;
int16_t r_vshunt=0;
int16_t r_current=0;


if(csa_RdReg(INA226_REG_VSHUNT,&tmp)==0)r_vshunt=(int16_t)tmp;
else return;

if(csa_RdReg(INA226_REG_VBUS,&r_vbus)!=0)return;
if(csa_RdReg(INA226_REG_POWER,&r_power)!=0)return;

if(csa_RdReg(INA226_REG_CURRENT,&tmp)!=0)return;
r_current=(int16_t)tmp;

csa_RdReg(INA226_REG_MASKEN,&r_masken);

s_data.vbus_mv=((int32_t)r_vbus*5L)/4L;
s_data.vshunt_uv=((int32_t)r_vshunt*5L)/2L;
s_data.current_ma=((int32_t)r_current*(int32_t)CSA_CURRENT_LSB_NA)/1000000L;
s_data.power_mw=((int32_t)r_power*25L*(int32_t)CSA_CURRENT_LSB_NA)/1000000L;


}

void Csa_Init(void)
{
uint16_t mfr=0U;
uint16_t die=0U;
uint16_t cfg=0U;
uint16_t cfg_rb=0U;
uint16_t calib_rb=0U;


s_data.connected=0U;
s_data.vbus_mv=0;
s_data.vshunt_uv=0;
s_data.current_ma=0;
s_data.power_mw=0;
s_data.mfr_id=0U;
s_data.die_id=0U;
s_taskCnt=0U;

PCC->PCCn[PCC_PORTA_INDEX]|=PCC_PCCn_CGC_MASK;

PORTA->PCR[CSA_SDA]=PORT_PCR_MUX(1U);
PORTA->PCR[CSA_SCL]=PORT_PCR_MUX(1U);

csa_SdaH();
PTA->PDDR&=~(1UL<<CSA_SCL);
csa_Us(CSA_DLY);

SEGGER_RTT_printf(0,"\r\n[CSA] INA226-Q1 Init\r\n");

if(!csa_Probe()){
    SEGGER_RTT_printf(0,"[CSA] I2C NACK Addr=0x%02X\r\n",CSA_I2C_ADDR);
    return;
}

SEGGER_RTT_printf(0,"[CSA] I2C ACK Addr=0x%02X\r\n",CSA_I2C_ADDR);

if(csa_RdReg(INA226_REG_MFR_ID,&mfr)!=0||mfr!=INA226_MFR_ID_VAL){
    SEGGER_RTT_printf(0,"[CSA] MFR FAIL 0x%04X\r\n",mfr);
    return;
}

s_data.mfr_id=mfr;

if(csa_RdReg(INA226_REG_DIE_ID,&die)!=0||
   (die&INA226_DIE_ID_MASK)!=INA226_DIE_ID_VAL){
    SEGGER_RTT_printf(0,"[CSA] DIE FAIL 0x%04X\r\n",die);
    return;
}

s_data.die_id=die;

if(csa_WrReg(INA226_REG_CONFIG,0x8000U)!=0)return;
csa_Ms(2U);

csa_RdReg(INA226_REG_CONFIG,&cfg);

if(csa_WrReg(INA226_REG_CONFIG,INA226_CFG_VAL)!=0)return;
csa_Ms(2U);

if(csa_RdReg(INA226_REG_CONFIG,&cfg_rb)!=0||
   cfg_rb!=INA226_CFG_VAL){
    SEGGER_RTT_printf(0,"[CSA] CFG FAIL 0x%04X\r\n",cfg_rb);
    return;
}

if(csa_WrReg(INA226_REG_CALIB,CSA_CAL_VALUE)!=0)return;
csa_Ms(2U);

if(csa_RdReg(INA226_REG_CALIB,&calib_rb)!=0||
   calib_rb!=CSA_CAL_VALUE){
    SEGGER_RTT_printf(0,"[CSA] CAL FAIL 0x%04X\r\n",calib_rb);
    return;
}

csa_Ms(300U);
csa_ReadMeasurement();

s_data.connected=1U;

SEGGER_RTT_printf(0,
    "[CSA] READY MFR=0x%04X DIE=0x%04X CFG=0x%04X CAL=0x%04X\r\n",
    s_data.mfr_id,s_data.die_id,cfg_rb,calib_rb);

SEGGER_RTT_printf(0,
    "[CSA] Shunt=%luuOhm Max=%lumA\r\n",
    (unsigned long)CSA_SHUNT_UOHM,
    (unsigned long)CSA_MAX_CURRENT_MA);


}

void Csa_Task(void)
{
uint16_t masken=0U;
uint8_t cvrf,ovf;


if(!s_data.connected)return;

s_taskCnt++;

csa_ReadMeasurement();

csa_RdReg(INA226_REG_MASKEN,&masken);

cvrf=(uint8_t)((masken>>3U)&1U);
ovf=(uint8_t)((masken>>2U)&1U);

if((s_taskCnt%CSA_PRINT_EVERY)==0U){
    int32_t vb_i=s_data.vbus_mv/1000L;
    int32_t vb_f=csa_Abs(s_data.vbus_mv%1000L);

    int32_t vs_i=s_data.vshunt_uv/1000L;
    int32_t vs_f=csa_Abs(s_data.vshunt_uv%1000L);

    int32_t cu_i=s_data.current_ma/1000L;
    int32_t cu_f=csa_Abs(s_data.current_ma%1000L);

    int32_t pw_i=s_data.power_mw/1000L;
    int32_t pw_f=csa_Abs(s_data.power_mw%1000L);

    SEGGER_RTT_printf(0,
        "[CSA] %ld.%03ldV %ld.%03ldmV %ld.%03ldA %ld.%03ldW CVRF=%u OVF=%u\r\n",
        (long)vb_i,(long)vb_f,
        (long)vs_i,(long)vs_f,
        (long)cu_i,(long)cu_f,
        (long)pw_i,(long)pw_f,
        (unsigned)cvrf,(unsigned)ovf);
}


}

void Csa_GetData(Csa_Data_t *out)
{
if(out)*out=s_data;
}

void Csa_GetLastPkt(CsaPkt_t *out)
{
if(!out)return;


out->voltage_mv=s_data.vbus_mv;
out->current_ma=s_data.current_ma;
out->power_mw=s_data.power_mw;
out->ts_ms=Uart_GetMs();


}

uint8_t Csa_IsReady(void)
{
return s_data.connected;
}
