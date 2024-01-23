/*** Altair Embed x64 2023 Build 131 Automatic C Code Generator ***/
/*  Output for C:\Users\guilhermes\Documents\marcopolo\STM32F429ZIT6\CAN1-RxTx-F429ZIT6-V2.vsm at Tue Jan 23 18:31:55 2024 */
/*  Target: STM32[F429ZI] */

#include "math.h"
#include "cgen.h"
#include "cgendll.h"
#define STM32F429xx 1
#define STM32F4 1
#undef FAR
#include "stm32F4xx_hal.h"
#include "stm32defs.h"
#include "canBus.h"
extern SIM_STATE *vsmMainTask;
int MHZ=16;
CAN_TRANSMIT canTrans7 = { 0x100, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0};
CAN_RECEIVE canRec5 = { 0x10,0x0,0,0,0,0,0,0,0,0,0x1FFFFFFFL, 0, 0, 0, 0};
static int16_t newMsg;


/* vsmIdleLoop for background threads */
void idleLoop(void)
{
  CanProcessRecFifo(0);
  CanProcessTransFifo(0);
}
static SIGNAL vsm__vsOutSig4[5]={{T_INT,0}, {T_UCHAR,0}, {T_ULONG,0}, {T_INT,0}, {T_ULONG,0}};
static ARG_DESCR outArgInfo4[]={
  { T_INT,0,0,0},
  { T_UCHAR,0,0,0},
  { T_ULONG,0,0,0},
  { T_INT,0,0,0},
  { T_ULONG,0,0,0},
};
static ARG_DESCR inArgInfo4[]={
0};
static SIM_STATE tSim={0,0,0
,outArgInfo4, inArgInfo4,0,5,0,0,0,0,1,1,1,0,0,0,0,0,vsm__vsOutSig4};
SIM_STATE *vsmMainTask=&tSim;

/*CAN Tx/Rx*/
void SysTick_Handler(void){
static int16_t vsm__pulseCnt20=99;
  int16_t vsm_20;
static CGDOUBLE vsm__delayOutBuf23=0;
static CGDOUBLE vsm__delayInBuf23=0;
  CGDOUBLE vsm_21;
  uint_least8_t vsm_28;
static int16_t vsm__squareCnt39=0;
  int16_t vsm_39;
  uint32_t vsm_38;
  SIM_STATE *sim = &tSim;
  GET_MAX_STACK_USED();
  vsm_20 = (++vsm__pulseCnt20 > 99?vsm__pulseCnt20=0,1:0);
  if ( vsm_20) vsm__delayOutBuf23=vsm__delayInBuf23;
  vsm_21 = ( vsm__delayOutBuf23+1.f);
  vsm_28 = ((unsigned char) vsm_21);
  vsm_39 = (++vsm__squareCnt39 > 10?(vsm__squareCnt39>=20?vsm__squareCnt39=0,1:1):0);
  vsm_38 = !((uint32_t)((unsigned long) vsm_39));
  CHECK_CAN_REC_ADDR(0,canRec5,257);
	
;  newMsg =  CanCheckForMessageArrival(&canRec5);
  if( vsm_20)
  {
  CHECK_CAN_TRANS_ADDR(0,canTrans7,257)
canTrans7.in1 = (canTrans7.in1&0xFF00) | (((uint16_t)1)&0xFF);
canTrans7.in1 = (canTrans7.in1&0xFF) | (((uint16_t)2)<<8);
canTrans7.in2 = (canTrans7.in2&0xFF00) | (((uint16_t)3)&0xFF);
canTrans7.in2 = (canTrans7.in2&0xFF) | (((uint16_t)4)<<8);
canTrans7.in3 = (canTrans7.in3&0xFF00) | (((uint16_t)5)&0xFF);
canTrans7.in3 = (canTrans7.in3&0xFF) | (((uint16_t)6)<<8);
canTrans7.in4 = (canTrans7.in4&0xFF00) | (((uint16_t)7)&0xFF);
canTrans7.in4 = (canTrans7.in4&0xFF) | (((uint16_t) vsm_28)<<8);
  ecan_write_mbox(0, &canTrans7);
  }
  if ( vsm_39)
    {GPIOG->ODR |= 0x4000u;}
  else
    {GPIOG->ODR &= ~(0x4000u);}
  if (((int) vsm_38))
    {GPIOG->ODR |= 0x2000u;}
  else
    {GPIOG->ODR &= ~(0x2000u);}
  vsmMainTask->outSigS[0].u.Int =  newMsg ;
  vsmMainTask->outSigS[1].u.Int =  vsm_28;
  vsmMainTask->outSigS[2].u.ULong = ((unsigned long) vsm_21);
  vsmMainTask->outSigS[3].u.Int =  vsm_39;
  vsmMainTask->outSigS[4].u.ULong =  vsm_38;

  if ( vsm_20)
    vsm__delayInBuf23 =  vsm_21;
  vsmMainTask->tickCount++;
  endOfSampleCount = SysTick->VAL;
}

void main(void)
{
  noIntegrationUsed = 1;
  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  simInit( &tSim );
  disable_interrupts();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  __HAL_RCC_HSI_ENABLE();
  __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);
  __HAL_RCC_PLL_DISABLE();
  __HAL_RCC_PLL_CONFIG(RCC_PLLSOURCE_HSI, 16,192,2,4);
  __HAL_RCC_PLL_ENABLE();
  { int a; for (a = 0; a < 32000;a++) if (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET) break; }
  __HAL_RCC_PLLSAI_CONFIG(192,2,2);
  __HAL_RCC_PLLSAI_ENABLE();
  __HAL_RCC_PLLI2S_CONFIG(50,2);
  __HAL_RCC_PLLI2S_ENABLE();
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1,RCC_CFGR_PPRE1_DIV1);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2,RCC_CFGR_PPRE2_DIV1);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE,RCC_CFGR_HPRE_DIV1);
  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
  __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);
  __HAL_RCC_LSE_CONFIG(RCC_LSE_OFF);
  { int a; for (a = 0; a < 32000;a++) { if (__HAL_RCC_GET_SYSCLK_SOURCE() == (RCC_SYSCLKSOURCE_HSI<< RCC_CFGR_SWS_Pos)) break;}}
  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock * 0.010000);	/*Timer call*/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpioInit(GPIOA, GPIO_PIN_11, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 9);	/*GPIO11 init*/
  gpioInit(GPIOA, GPIO_PIN_12, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 9);	/*GPIO12 init*/
  __HAL_RCC_GPIOG_CLK_ENABLE();
  gpioInit(GPIOG, GPIO_PIN_13, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);	/*GPIO109 init*/
  gpioInit(GPIOG, GPIO_PIN_14, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0);	/*GPIO110 init*/
  CanBusInit(0,4u, 3u,1u,4u,0x10001u);
  ecan_bus_trans_init(0, &canTrans7);
  ecan_bus_rec_init(0, &canRec5);
  startSimDsp(&tSim);
  CanBusStart(0);
  dspWait(vsmMainTask);
}
