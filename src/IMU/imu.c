#include "imu.h"
#include "S32K144.h"
#include "SEGGER_RTT.h"
#include "../UART/uart_pkt.h"
#include "DEBUG/debug_rtt.h"

#define BB_SDA 2U
#define BB_SCL 3U
#define BB_DLY 5U

static int32_t s_ax0,s_ay0,s_az0;
static int32_t s_gx0,s_gy0,s_gz0;
static int32_t s_vx,s_vy,s_vz;
static int32_t s_px,s_py,s_pz;
static int32_t s_ax_mg,s_ay_mg,s_az_mg;
static int32_t s_gx_mdps,s_gy_mdps,s_gz_mdps;
static int16_t s_temp_c10;
static uint8_t s_still,s_ready;
static uint32_t s_n;

/*
 * Calibration timeout: bounds the whole non-blocking calibration
 * sequence in case the sensor stops ACKing partway through (e.g.
 * a flaky I2C bus) - without this, s_cal_state would stay RUNNING
 * forever and IMU data would never start flowing.
 */
#define IMU_CAL_TIMEOUT_MS   (IMU_CAL_SAMPLES * IMU_DT_MS * 3U)

typedef enum
{
    IMU_CAL_IDLE = 0,
    IMU_CAL_RUNNING,
    IMU_CAL_DONE,
    IMU_CAL_TIMED_OUT
} ImuCalState_t;

static ImuCalState_t s_cal_state = IMU_CAL_IDLE;
static uint8_t  s_cal_index;
static int32_t  s_cal_sa,s_cal_sb,s_cal_sc,s_cal_sd,s_cal_se,s_cal_sf;
static int32_t  s_cal_ca,s_cal_cd;
static uint32_t s_cal_last_ms;
static uint32_t s_cal_start_ms;


#if APP_IMU_ENABLE

Debug_RTT_Print(
    "[MAIN] IMU INIT\r\n");

Imu_Init();

Debug_RTT_Print(
    "[MAIN] IMU CALIBRATE\r\n");

Imu_Calibrate();

Debug_RTT_Print(
    "[MAIN] IMU READY\r\n");

#endif

static void prv_Ms(volatile uint32_t ms)
{
while(ms--){volatile uint32_t n=80000U;while(n--){;}}
}

static void prv_Us(volatile uint32_t us)
{
while(us--){volatile uint32_t n=80U;while(n--){;}}
}

static void prv_SdaL(void)
{
PTA->PDDR|=(1UL<<BB_SDA);
PTA->PCOR=(1UL<<BB_SDA);
}

static void prv_SdaH(void)
{
PTA->PDDR&=~(1UL<<BB_SDA);
}

static void prv_SclL(void)
{
PTA->PDDR|=(1UL<<BB_SCL);
PTA->PCOR=(1UL<<BB_SCL);
}

static void prv_SclH(void)
{
volatile uint32_t t=5000U;
PTA->PDDR&=~(1UL<<BB_SCL);
while(!(PTA->PDIR&(1UL<<BB_SCL))&&--t){;}
prv_Us(BB_DLY);
}

static uint8_t prv_SdaR(void)
{
return(uint8_t)((PTA->PDIR>>BB_SDA)&1U);
}

static void prv_I2cInit(void)
{
PCC->PCCn[PCC_PORTA_INDEX]|=PCC_PCCn_CGC_MASK;
PORTA->PCR[BB_SDA]=PORT_PCR_MUX(1U);
PORTA->PCR[BB_SCL]=PORT_PCR_MUX(1U);
prv_SdaH();
PTA->PDDR&=~(1UL<<BB_SCL);
prv_Us(BB_DLY);
}

static void prv_Start(void)
{
prv_SdaH();
prv_Us(BB_DLY);
prv_SclH();
prv_SdaL();
prv_Us(BB_DLY);
prv_SclL();
prv_Us(BB_DLY);
}

static void prv_Stop(void)
{
prv_SdaL();
prv_Us(BB_DLY);
prv_SclH();
prv_SdaH();
prv_Us(BB_DLY);
}

static void prv_WBit(uint8_t b)
{
if(b)prv_SdaH();
else prv_SdaL();
prv_Us(BB_DLY);
prv_SclH();
prv_SclL();
prv_Us(BB_DLY);
}

static uint8_t prv_RBit(void)
{
uint8_t b;
prv_SdaH();
prv_Us(BB_DLY);
prv_SclH();
b=prv_SdaR();
prv_SclL();
prv_Us(BB_DLY);
return b;
}

static uint8_t prv_WByte(uint8_t v)
{
uint8_t i,ack;
for(i=0U;i<8U;i++){
prv_WBit((uint8_t)(v&0x80U));
v<<=1U;
}
ack=prv_RBit();
return ack?0U:1U;
}

static uint8_t prv_RByte(uint8_t ack)
{
uint8_t i,v=0U;
for(i=0U;i<8U;i++)v=(uint8_t)((v<<1U)|prv_RBit());
prv_WBit(ack?0U:1U);
return v;
}

static int prv_Wr(uint8_t reg,uint8_t val)
{
uint8_t a1,a2,a3;
prv_Start();
a1=prv_WByte((uint8_t)((IMU_I2C_ADDR<<1U)|0U));
a2=prv_WByte(reg);
a3=prv_WByte(val);
prv_Stop();
return(a1&&a2&&a3)?0:-1;
}

static int prv_Rd(uint8_t reg,uint8_t *buf,uint8_t len)
{
uint8_t i;
if(!buf||!len)return-1;
prv_Start();
if(!prv_WByte((uint8_t)((IMU_I2C_ADDR<<1U)|0U))){prv_Stop();return-1;}
if(!prv_WByte(reg)){prv_Stop();return-1;}
prv_Start();
if(!prv_WByte((uint8_t)((IMU_I2C_ADDR<<1U)|1U))){prv_Stop();return-1;}
for(i=0U;i<len;i++)buf[i]=prv_RByte((uint8_t)(i<(uint8_t)(len-1U)));
prv_Stop();
return 0;
}

static int16_t prv_S16(uint8_t hi,uint8_t lo)
{
return(int16_t)(((uint16_t)hi<<8U)|lo);
}

static int32_t prv_Abs(int32_t v)
{
return v<0?-v:v;
}

void Imu_Init(void)
{
uint8_t v=0U,r=0U;
s_ready=0U;
prv_I2cInit();
prv_Wr(ICM_REG_SIG_PATH_RST,0x10U);
prv_Ms(10U);


do{
    prv_Ms(1U);
    prv_Rd(ICM_REG_MCLK_RDY,&v,1U);
    r++;
}while(v==0U&&r<50U);

SEGGER_RTT_printf(0,"[IMU] OTP=0x%02X (%dms)\r\n",v,(int)r);

prv_Wr(ICM_REG_INTF_CFG1,0x41U);
prv_Ms(1U);
prv_Wr(ICM_REG_PWR_MGMT0,0x10U);
prv_Ms(2U);

r=0U;
do{
    prv_Ms(1U);
    prv_Rd(ICM_REG_MCLK_RDY,&v,1U);
    r++;
}while(!(v&0x08U)&&r<30U);

SEGGER_RTT_printf(0,"[IMU] MCLK=0x%02X (%dms)\r\n",v,(int)r);

prv_Wr(ICM_REG_PWR_MGMT0,0x14U);
prv_Ms(5U);
prv_Wr(ICM_REG_PWR_MGMT0,0x1CU);
prv_Ms(5U);
prv_Wr(ICM_REG_PWR_MGMT0,0x1FU);
prv_Wr(ICM_REG_GYRO_CFG0,IMU_GYRO_CFG);
prv_Wr(ICM_REG_ACCEL_CFG0,IMU_ACCEL_CFG);
prv_Ms(50U);

prv_Rd(ICM_REG_ACCEL_X1,&v,1U);
s_ready=(v!=0x80U)?1U:0U;

SEGGER_RTT_printf(0,"[IMU] ACCEL_X1=0x%02X %s\r\n",v,s_ready?"SENSORS ON":"FAIL");


}

/*
 * Applies whatever samples were accumulated (full set, or a partial
 * set on timeout) and prints the same summary the old blocking
 * version printed.
 */
static void imu_cal_finish(void)
{
if(s_cal_ca>0){
    s_ax0=s_cal_sa/s_cal_ca;
    s_ay0=s_cal_sb/s_cal_ca;
    s_az0=s_cal_sc/s_cal_ca;
}

if(s_cal_cd>0){
    s_gx0=s_cal_sd/s_cal_cd;
    s_gy0=s_cal_se/s_cal_cd;
    s_gz0=s_cal_sf/s_cal_cd;
}

SEGGER_RTT_printf(0,"Accel: AX=%-6d AY=%-6d AZ=%-6d (%d)\r\n",
    (int)s_ax0,(int)s_ay0,(int)s_az0,(int)s_cal_ca);

SEGGER_RTT_printf(0,"Gyro: GX=%-6d GY=%-6d GZ=%-6d (%d)\r\n\r\n",
    (int)s_gx0,(int)s_gy0,(int)s_gz0,(int)s_cal_cd);

SEGGER_RTT_printf(0," N X mm Y mm Z mm AX cm/s AY cm/s AZ cm/s GX d/s GY d/s GZ d/s T C\r\n");
SEGGER_RTT_printf(0,"-------------------------------------------------\r\n");
}

/*
 * Starts calibration as a non-blocking background state machine
 * instead of busy-waiting ~2s (IMU_CAL_SAMPLES * IMU_DT_MS) here.
 * One sample is taken per imu_cal_step() call, driven from Imu_Task()
 * at its normal IMU_DT_MS cadence from the main loop, so CAN1/CAN2,
 * UART, CSA and FLM keep running the whole time - nothing blocks on
 * this. Imu_IsReady() still reflects hardware init, not calibration
 * progress; IMU packets sent while calibration is running just carry
 * whatever s_ax_mg/etc last held (zero, until the first post-
 * calibration Imu_Task() sample), same as CSA reporting zeros before
 * Csa_Init() finishes.
 */
void Imu_Calibrate(void)
{
if(!s_ready)return;

SEGGER_RTT_printf(0,"Calibrating - keep STILL for 2s...\r\n");

s_cal_index=0U;
s_cal_sa=0;s_cal_sb=0;s_cal_sc=0;s_cal_sd=0;s_cal_se=0;s_cal_sf=0;
s_cal_ca=0;s_cal_cd=0;
s_cal_start_ms=Uart_GetMs();
s_cal_last_ms=s_cal_start_ms;
s_cal_state=IMU_CAL_RUNNING;
}

static void imu_cal_step(void)
{
uint8_t raw[14];
uint32_t now;

now=Uart_GetMs();

if((now-s_cal_start_ms)>=IMU_CAL_TIMEOUT_MS){
    SEGGER_RTT_printf(0,
        "[IMU_ERR] Calibration timeout after %u/%u samples\r\n",
        (unsigned)s_cal_index,(unsigned)IMU_CAL_SAMPLES);
    imu_cal_finish();
    s_cal_state=IMU_CAL_TIMED_OUT;
    return;
}

if((now-s_cal_last_ms)<IMU_DT_MS)return;
s_cal_last_ms=now;

if(prv_Rd(ICM_REG_TEMP_DATA1,raw,14U)==0){
    int16_t ax=prv_S16(raw[2],raw[3]);
    int16_t ay=prv_S16(raw[4],raw[5]);
    int16_t az=prv_S16(raw[6],raw[7]);
    int16_t gx=prv_S16(raw[8],raw[9]);
    int16_t gy=prv_S16(raw[10],raw[11]);
    int16_t gz=prv_S16(raw[12],raw[13]);

    if(ax!=(int16_t)0x8000){
        s_cal_sa+=ax;s_cal_sb+=ay;s_cal_sc+=az;s_cal_ca++;
    }

    if(gx>-IMU_GYRO_ZRO_THR&&gx<IMU_GYRO_ZRO_THR&&
       gy>-IMU_GYRO_ZRO_THR&&gy<IMU_GYRO_ZRO_THR&&
       gz>-IMU_GYRO_ZRO_THR&&gz<IMU_GYRO_ZRO_THR){
        s_cal_sd+=gx;s_cal_se+=gy;s_cal_sf+=gz;s_cal_cd++;
    }
}

s_cal_index++;

if(s_cal_index>=IMU_CAL_SAMPLES){
    imu_cal_finish();
    s_cal_state=IMU_CAL_DONE;
}
}

void Imu_Task(void)
{
uint8_t raw[14],still;
int16_t tp,ax,ay,az,gx,gy,gz;
int32_t dax,day,daz,dgx,dgy,dgz;
int32_t acx,acy,acz,gdx,gdy,gdz,tc;
if(!s_ready)return;
if(s_cal_state==IMU_CAL_RUNNING){imu_cal_step();return;}
if(prv_Rd(ICM_REG_TEMP_DATA1,raw,14U)!=0)return;

tp=prv_S16(raw[0],raw[1]);
ax=prv_S16(raw[2],raw[3]);
ay=prv_S16(raw[4],raw[5]);
az=prv_S16(raw[6],raw[7]);
gx=prv_S16(raw[8],raw[9]);
gy=prv_S16(raw[10],raw[11]);
gz=prv_S16(raw[12],raw[13]);

dax=(int32_t)ax-s_ax0;
day=(int32_t)ay-s_ay0;
daz=(int32_t)az-s_az0;
dgx=(int32_t)gx-s_gx0;
dgy=(int32_t)gy-s_gy0;
dgz=(int32_t)gz-s_gz0;

s_ax_mg=(dax*1000L)/16384L;
s_ay_mg=(day*1000L)/16384L;
s_az_mg=(daz*1000L)/16384L;
s_gx_mdps=(dgx*1000L)/131L;
s_gy_mdps=(dgy*1000L)/131L;
s_gz_mdps=(dgz*1000L)/131L;
s_temp_c10=(int16_t)(((int32_t)tp*10L)/128L+250L);

still=(uint8_t)(
    dax>-IMU_STILL_THR&&dax<IMU_STILL_THR&&
    day>-IMU_STILL_THR&&day<IMU_STILL_THR&&
    daz>-IMU_STILL_THR&&daz<IMU_STILL_THR);

if(still)s_still++;
else s_still=0U;

if(s_still>=IMU_ZVU_CNT)s_vx=s_vy=s_vz=0;
if(s_still>=IMU_ZPU_CNT)s_px=s_py=s_pz=0;
#if IMU_FEAT_DISPLACEMENT
if(!still){
s_vx+=(dax*490500L)/16384L;
s_vy+=(day*490500L)/16384L;
s_vz+=(daz*490500L)/16384L;
s_px+=s_vx/200L;
s_py+=s_vy/200L;
s_pz+=s_vz/200L;
}
#endif
s_n++;
acx=(dax*9810L)/16384L;
acy=(day*9810L)/16384L;
acz=(daz*9810L)/16384L;
gdx=(dgx*10L)/131L;
gdy=(dgy*10L)/131L;
gdz=(dgz*10L)/131L;
tc=((int32_t)tp*100L)/128L+2500L;

/*
 * This table used to print unconditionally every Imu_Task() call
 * (~20 Hz once running) - pure debug console spam with no consumer,
 * that both stole cooperative-loop time and could overflow the RTT
 * no-block-skip up-buffer under load. Bounded to ~1 Hz like the
 * other periodic RTT status lines.
 */
{
static uint32_t s_last_print_ms=0U;
uint32_t now_print=Uart_GetMs();
if((now_print-s_last_print_ms)>=1000U)
{
s_last_print_ms=now_print;
SEGGER_RTT_printf(0,"%4lu",(unsigned long)s_n);
#if IMU_FEAT_DISPLACEMENT
SEGGER_RTT_printf(0," %ld.%02ld %ld.%02ld %ld.%02ld",
(long)(s_px/100L),(long)(prv_Abs(s_px)%100L),
(long)(s_py/100L),(long)(prv_Abs(s_py)%100L),
(long)(s_pz/100L),(long)(prv_Abs(s_pz)%100L));
#endif

#if IMU_FEAT_ACCEL
SEGGER_RTT_printf(0," %ld.%01ld %ld.%01ld %ld.%01ld",
(long)(acx/10L),(long)(prv_Abs(acx)%10L),
(long)(acy/10L),(long)(prv_Abs(acy)%10L),
(long)(acz/10L),(long)(prv_Abs(acz)%10L));
#endif

#if IMU_FEAT_GYRO
SEGGER_RTT_printf(0," %ld.%01ld %ld.%01ld %ld.%01ld",
(long)(gdx/10L),(long)(prv_Abs(gdx)%10L),
(long)(gdy/10L),(long)(prv_Abs(gdy)%10L),
(long)(gdz/10L),(long)(prv_Abs(gdz)%10L));
#endif

#if IMU_FEAT_TEMP
SEGGER_RTT_printf(0," %ld.%02ld",
(long)(tc/100L),(long)(prv_Abs(tc)%100L));
#endif
SEGGER_RTT_printf(0,"\r\n");
}
}
}

void Imu_GetLastPkt(ImuPkt_t *out)
{
if(!out)return;
out->ax_mg=s_ax_mg;
out->ay_mg=s_ay_mg;
out->az_mg=s_az_mg;
out->gx_mdps=s_gx_mdps;
out->gy_mdps=s_gy_mdps;
out->gz_mdps=s_gz_mdps;
out->temp_c10=s_temp_c10;
out->ts_ms=Uart_GetMs();
}

uint8_t Imu_IsReady(void)
{
return s_ready;
}
