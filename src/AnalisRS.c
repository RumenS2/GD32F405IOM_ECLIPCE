#include "AnalisRS.h"
#include "board.h"
#include "variables.h"
#include "hw_config.h"
#include "stm32f4xx_it.h"
#include <string.h>

void RSC_GetStatus(void );
void RSC_Get7ADCvolt(void );
void RSC_Get7ADCspc(void );
void RSC_GetSVpBV(void );
void RSC_GetConfig(void );
void RSC_GetCalib(void );
void RSC_GetOIST(void );
void RSC_SetStatus(void );
void RSC_SetOlCo(void );
void RSC_SetPWM2(void );
void RSC_SetPWM1(void);
void RSC_SetUSERI(void );
void RSC_SetConfig(void );
void RSC_SetCalib(void );
void RSC_SetOutsA32(void );
void ExchgExtDevSomething (uint8_t ExtDevNum, uint8_t GetSubCmdCode);


#define ERRLIN_BUF_LEN (RSBUFLEN/8)  //2*4
uint8_t RS0Buffer[RSBUFLEN],RS2Buffer[RSBUFLEN];
uint32_t ErrLineBuff[ERRLIN_BUF_LEN][2];
uint16_t ELB_Idx=0;
volatile struct RS_State RSS0,RSS2;
volatile uint8_t CommandToIOM=0;
volatile uint8_t ConfigNowReceived=0;
volatile uint8_t AnalisRS0IsBusy=0;
volatile uint8_t AnalisRS2IsBusy=0;
volatile uint32_t globalrserrcnt=0;
//USART_TypeDef* UsartInUse;

void USARTinUSE_RX_START(uint32_t usart_periph)
{
	if (usart_periph==USART0)
	{
		RSS0.State=RSstWait1Rcv;
		LoRCV_Enbl_485_0;
	}
	if (usart_periph==USART2)
	{
		RSS2.State=RSstWait1Rcv;
		LoRCV_Enbl_485_2;
	}
	USART_REG_VAL(usart_periph, USART_INT_RBNE) |= BIT(USART_BIT_POS(USART_INT_RBNE));
	USART_CTL0(usart_periph)|=USART_CTL0_REN;
}
__STATIC_INLINE void USART0_RX_START(void)
{
	RSS0.State=RSstWait1Rcv;
	LoRCV_Enbl_485_0;
	USART_REG_VAL(USART0, USART_INT_RBNE) |= BIT(USART_BIT_POS(USART_INT_RBNE));
	USART_CTL0(USART0)|=USART_CTL0_REN;
}
__STATIC_INLINE void USART2_RX_START(void)
{
	RSS2.State=RSstWait1Rcv;
	LoRCV_Enbl_485_2;
	USART_REG_VAL(USART2, USART_INT_RBNE) |= BIT(USART_BIT_POS(USART_INT_RBNE));
	USART_CTL0(USART2)|=USART_CTL0_REN;
}
__STATIC_INLINE void USART0_TX_START(void)
{
	RSS0.CntTimeout=SysTickCntr+10; //1ms
	RSS0.State=RSstTrn;
	HiTRN_Enbl_485_0;
	USART_CTL0(USART0)|=USART_CTL0_TEN;
	USART_REG_VAL(USART0, USART_INT_TBE) |= BIT(USART_BIT_POS(USART_INT_TBE));
}
__STATIC_INLINE void USART2_TX_START(void)
{
	RSS2.CntTimeout=SysTickCntr+10; //1ms
	RSS2.State=RSstTrn;
	HiTRN_Enbl_485_2;
	USART_CTL0(USART2)|=USART_CTL0_TEN;
	USART_REG_VAL(USART2, USART_INT_TBE) |= BIT(USART_BIT_POS(USART_INT_TBE));
}
__STATIC_INLINE void USARTinUSE_RX_DISABLE(uint32_t usart_periph)
{
	USART_CTL0(usart_periph)&=~USART_CTL0_REN;
	USART_REG_VAL(usart_periph, USART_INT_RBNE) &= ~BIT(USART_BIT_POS(USART_INT_RBNE));
	USART_REG_VAL(usart_periph, USART_INT_IDLE) &= ~BIT(USART_BIT_POS(USART_INT_IDLE));
}

void AnalisMRS3(void);
void AnalisRSmain(void);


void SaveErrInList(uint32_t EFlags,uint32_t Cline)
{
  ErrLineBuff[ELB_Idx][0]=EFlags;ErrLineBuff[ELB_Idx][1]=Cline;
  ELB_Idx++;if (ELB_Idx==ERRLIN_BUF_LEN) ELB_Idx=0;
}

__STATIC_INLINE int SetSemB(volatile uint8_t* addr,uint8_t val)
{
	uint8_t t8;
	pak:;
	  t8=__LDREXB(addr);
	  if (t8) {__CLREX();goto isbusy;} //is already set (not free)
	  if (__STREXB(val,addr)) goto pak;
	  t8=0; //ok
isbusy:;
	  return t8;
}

//!==========================================================================================================
void AnalisRS(void)
{
//	if (SetSemB(&AnalisRS2IsBusy, 1)) goto rs2isbusy;
//	  AnalisMRS3();
//    AnalisRS2IsBusy=0;
//rs2isbusy:
	if (SetSemB(&AnalisRS0IsBusy, 1)) goto rs0isbusy;
    AnalisRSmain();
    AnalisRS0IsBusy=0;
rs0isbusy:;
}

void AnalisMRS3(void) //Master mode
{
	static uint32_t OldSyCt=0;
		if ((SysTickCntr<OldSyCt)) //this moment SysTick roll-over from 0xf..f to 0x0..0
		{
			if (RSS2.CntAnalisRS>SysTickCntr) RSS2.CntAnalisRS=SysTickCntr; //if state is Idle: new question is forced
			if (RSS2.CntTimeout>SysTickCntr) RSS2.CntTimeout=SysTickCntr+30; //if state is no idle: 3000us bonus time for next char
		}
		OldSyCt=SysTickCntr;
//------------------------------------------------------------------------------
		if (RSS2.State==RSstIdle)
		{
			if (RSS2.CntAnalisRS<=SysTickCntr)
			{
			  RSS2.CntAnalisRS=SysTickCntr+537; //53.7ms
			  RSS2.RetrCnt=0;
			  RSS2.CommCntr++; if (RSS2.CommCntr>2) RSS2.CommCntr=1;
			  if (RSS2.CommCntr==1) ExchgExtDevSomething(RsExtSlaveDeviceN,SubCmd_StructSTATUS);
			  if (RSS2.CommCntr==2) ExchgExtDevSomething(RsExtSlaveDeviceN,SubCmd_StructSTATUS);
			  USART2_TX_START(); //because TX was enabled, IDLE frame will be sent
			}
			goto exit_ok;
		}
//IF No idle state, only timeout is criteria for end of Trn/Rcv sequence
	  if (RSS2.CntTimeout>SysTickCntr) goto exit_ok;

	  USARTinUSE_RX_DISABLE(USART2);
	  if (RSS2.State==RSstWait1Rcv){RSS2.ErrCod=5;RSS2.RetrCnt++;RSS2.State=RSstIdle;goto exi_err;}
	  if (RSS2.wCRC!=0)
	  {
		if (RSS2.BegPnt==&RS2Buffer[0]) {RSS2.ErrCod=2;}else{RSS2.ErrCod=3;}
		RSS2.RetrCnt++;RSS2.State=RSstIdle;goto exi_err;
	  }
	  if (RS2Buffer[RsPosDN]!=RsExtSlaveDeviceN) { RSS2.RetrCnt++;RSS2.ErrCod=1;RSS2.State=RSstIdle; goto exi_err;}

	  if (RS2Buffer[RsPosCMD]==COMM_EXCHG) //check slave answer on exchange
	  {
		  RSS2.LastAnsTimeStamp=SysTickCntr;
		  RSS2.RetrCnt=0;RSS2.ErrCod=0;
		  memcpy((void*)&OISt,(const void*)&RS2Buffer[RsPosSubCMD+1],sizeof(OISt));
		  RSS2.State=RSstIdle;goto exit_ok;
	  }
	  RSS2.RetrCnt++;RSS2.ErrCod|=0x20;RSS2.State=RSstIdle; RSS2.CntAnalisRS=SysTickCntr+100;//start new question!!!
exi_err:;
	globalrserrcnt+=RSS2.RetrCnt;
exit_ok:;
}
//!==========================================================================================================
void AnalisRSmain(void) //!main channel (in fact works as slave)
{
	static uint32_t OldSyCt=0;
		if ((SysTickCntr<OldSyCt)) //this moment SysTick roll-over from 0xf..f to 0x0..0
		{
			if (RSS0.CntAnalisRS>SysTickCntr) RSS0.CntAnalisRS=SysTickCntr; //if state is Idle: new question is forced
			if (RSS0.CntTimeout>SysTickCntr) RSS0.CntTimeout=SysTickCntr+30; //if state is no idle: 3000us bonus time for next char
		}
		OldSyCt=SysTickCntr;

  if (RSS0.State==RSstWait1Rcv) goto exii;
  if (RSS0.CntTimeout>SysTickCntr) goto exii;
  if (RSS0.wCRC!=0)
  {
    if (RSS0.BegPnt==&RS0Buffer[0]) {RSS0.ErrCod=2;}else{RSS0.ErrCod=3;}
    RSS0.RetrCnt++;goto exi_err;
  }
  if (RS0Buffer[RsPosDN]!=RsSlaveDeviceN) { RSS0.RetrCnt++;RSS0.ErrCod=1;goto exi_err;}

//forcedtest:;
			if (RS0Buffer[RsPosCMD]==COMM_GET)
			{
              RSS0.RetrCnt=0;RSS0.ErrCod=0;
              if (RS0Buffer[RsPosSubCMD]==SubCmd_StructSTATUS) {RSC_GetStatus();goto cgans;}
              if (RS0Buffer[RsPosSubCMD]==SubCmd_AINvolt) {RSC_Get7ADCvolt();goto cgans;}
              if (RS0Buffer[RsPosSubCMD]==SubCmd_AINspc) {RSC_Get7ADCspc();goto cgans;}
              if (RS0Buffer[RsPosSubCMD]==SubCmd_SVpBV)  {RSC_GetSVpBV();goto cgans;}
              if (RS0Buffer[RsPosSubCMD]==SubCmd_Config) {RSC_GetConfig();goto cgans;}
              if (RS0Buffer[RsPosSubCMD]==SubCmd_Calib)  {RSC_GetCalib();goto cgans;}
              if (RS0Buffer[RsPosSubCMD]==SubCmd_OIST)    {RSC_GetOIST();goto cgans;}
          //unkn cmd
              RSS0.RetrCnt++;RSS0.ErrCod=4;
              goto exi_err;
cgans:;
			  RSS0.LastAnsTimeStamp=SysTickCntr;
              USART0_TX_START(); //idle frame will be generated because TE is set here
              goto exii;
			}
			if (RS0Buffer[RsPosCMD]==COMM_SET)
			{
              RSS0.RetrCnt=0;RSS0.ErrCod=0;
              if (RS0Buffer[RsPosSubCMD]==SubCmd_StructSTATUS) {RSC_SetStatus();goto cgans;}
              if (RS0Buffer[RsPosSubCMD]==SubCmd_OldComands) {RSC_SetOlCo();goto cgans;}
              if (RS0Buffer[RsPosSubCMD]==SubCmd_PWM1) {RSC_SetPWM1();goto cgans;}
              if (RS0Buffer[RsPosSubCMD]==SubCmd_PWM2) {RSC_SetPWM2();goto cgans;}
              if (RS0Buffer[RsPosSubCMD]==SubCmd_USERI) {RSC_SetUSERI();goto cgans;}
              if (RS0Buffer[RsPosSubCMD]==SubCmd_Config) {RSC_SetConfig();goto cgans;}
              if (RS0Buffer[RsPosSubCMD]==SubCmd_Calib) {RSC_SetCalib();goto cgans;}
              if (RS0Buffer[RsPosSubCMD]==SubCmd_OutsA32) {RSC_SetOutsA32();goto cgans;}
           //unkn cmd
              RSS0.RetrCnt++;RSS0.ErrCod=5;
              goto exi_err;
            }
        //unkn cmd
			RSS0.RetrCnt++;RSS0.ErrCod=7;
exi_err:
globalrserrcnt+=RSS0.RetrCnt;
			USART0_RX_START();
exii:;
}

void USART0_IRQHandler (void) //Slave VERSION
{
unsigned int IIR;
  IIR = USART_STAT0(USART0);

   if ((RSS0.State==RSstWait1Rcv))
	   if (IIR&USART_STAT0_RBNE)
		   {
			   RSS0.BegPnt=&RS0Buffer[0];RSS0.wCRC=0xffff;
			   RSS0.CntTimeout=SysTickCntr+3; //~3chars
			   *RSS0.BegPnt=(uint8_t)USART_DATA(USART0);Crc16ModbusFast(&RSS0.wCRC,RSS0.BegPnt);
			   RSS0.BegPnt++;
			   USART_REG_VAL(USART0, USART_INT_IDLE) |= BIT(USART_BIT_POS(USART_INT_IDLE)); //IDLE may be used as END if line have 2x680ohm!!
			   RSS0.State=RSstRcv;
//already cleared with reading form SR and DR if (IIR&(USART_SR_FE|USART_SR_IDLE|USART_SR_LBD)){USART1->SR=(USART_SR_FE|USART_SR_IDLE|USART_SR_LBD);}
			   goto exii;
		   }
	if ((RSS0.State==RSstTrn)&&(IIR&USART_STAT0_TBE))
	{
		if (RSS0.BegPnt==RSS0.EndPnt)
		{
			USART_REG_VAL(USART0, USART_INT_TBE) &= ~BIT(USART_BIT_POS(USART_INT_TBE)); //disable tbe int
			USART_REG_VAL(USART0, USART_INT_TC) |= BIT(USART_BIT_POS(USART_INT_TC)); //enable tc int
	        RSS0.State=RSstWaitTC_data;
		}
		USART_DATA(USART0)=*RSS0.BegPnt;*RSS0.BegPnt=0xcc; //sc_add_30-09-2015
		RSS0.CntTimeout=SysTickCntr+3; //~3chars
		RSS0.BegPnt++;//RSS0.TraceCod++;
		goto exii;
	}
	if ((RSS0.State==RSstWaitTC_data)&&(IIR&USART_STAT0_TC)) //!!!!THIS IS A SLAVE CASE!!!!!!!!!
	{ //  V--------SLAVE returns in Receiv, wait new quest from mast
		USART_REG_VAL(USART0, USART_INT_TC) &= ~BIT(USART_BIT_POS(USART_INT_TC));
		USART_CTL0(USART0)&=~USART_CTL0_TEN;
        USART0_RX_START();
		goto exii;
	}
	if (RSS0.State==RSstRcv)
	{
		*RSS0.BegPnt=(uint8_t)USART_DATA(USART0);

		if (IIR&(USART_STAT0_FERR|USART_STAT0_IDLEF)) // |USART_STAT0_LBDF))
		{
//already cleared with reading form SR and DR USART1->SR=(USART_SR_FE|USART_SR_IDLE|USART_SR_LBD);
			if (RSS0.wCRC==0){USARTinUSE_RX_DISABLE(USART0);RSS0.CntTimeout=SysTickCntr;goto exii;}
		}
		Crc16ModbusFast(&RSS0.wCRC,RSS0.BegPnt);RSS0.CntTimeout=SysTickCntr+3; //~3chars
		RSS0.BegPnt++;if (RSS0.BegPnt>=&RS0Buffer[RSBUFLEN-1]){RSS0.BegPnt--;USARTinUSE_RX_DISABLE(USART0);RSS0.CntTimeout=SysTickCntr;} //no waiting
		goto exii;
	}
	RSS0.ErrCod=(uint8_t)USART_DATA(USART0);
	RSS0.ErrCod=128; //unknown state, TX&RX must be disabled, go in idle
	USARTinUSE_RX_DISABLE(USART0);
	USART_REG_VAL(USART0, USART_INT_TBE) &= ~BIT(USART_BIT_POS(USART_INT_TBE));
	USART_REG_VAL(USART0, USART_INT_TC) &= ~BIT(USART_BIT_POS(USART_INT_TC));
	USART_CTL0(USART0)&=~USART_CTL0_TEN;
	USART0_RX_START();// SLAVE VERSION!!!!!
exii:;
}

//........................................................................................
void USART2_IRQHandler (void) //MASTER VERSION
{
unsigned int IIR;
  IIR = USART_STAT0(USART2);

   if ((RSS2.State==RSstWait1Rcv))
	   if (IIR&USART_STAT0_RBNE)
		   {
			   RSS2.BegPnt=&RS2Buffer[0];RSS2.wCRC=0xffff;
			   RSS2.CntTimeout=SysTickCntr+3; //~3chars
			   *RSS2.BegPnt=(uint8_t)USART_DATA(USART2);Crc16ModbusFast(&RSS2.wCRC,RSS2.BegPnt);
			   RSS2.BegPnt++;
			   USART_REG_VAL(USART2, USART_INT_IDLE) |= BIT(USART_BIT_POS(USART_INT_IDLE)); //IDLE may be used as END if line have 2x680ohm!!
			   RSS2.State=RSstRcv;
//already cleared with reading form SR and DR if (IIR&(USART_SR_FE|USART_SR_IDLE|USART_SR_LBD)){USART1->SR=(USART_SR_FE|USART_SR_IDLE|USART_SR_LBD);}
			   goto exii;
		   }
	if ((RSS2.State==RSstTrn)) //&&(IIR&USART_STAT0_TBE))
	{
		RSS2.CntTimeout=SysTickCntr+3; //~3 chars
		if (RSS2.BegPnt==RSS2.EndPnt)
		{
			USART_REG_VAL(USART2, USART_INT_TBE) &= ~BIT(USART_BIT_POS(USART_INT_TBE));
			USART_DATA(USART2)=*RSS2.BegPnt;*RSS2.BegPnt=0xcc; //sc_add_30-09-2015
			USART_REG_VAL(USART2, USART_INT_TC) |= BIT(USART_BIT_POS(USART_INT_TC));
	        RSS2.State=RSstWaitTC_data;
			goto exii;
		}
		USART_DATA(USART2)=*RSS2.BegPnt;*RSS2.BegPnt=0xcc; //sc_add_30-09-2015
		RSS2.BegPnt++;RSS2.TraceCod++;
		goto exii;
	}
	if ((RSS2.State==RSstWaitTC_data)) //&&(IIR&USART_STAT0_TC)) //!!!!THIS IS A MASTER CASE!!!!!!!!!
	{ //  V--------MASTER go in Receiv, wait answer
		USART_REG_VAL(USART2, USART_INT_TC) &= ~BIT(USART_BIT_POS(USART_INT_TC));
		USART_CTL0(USART2)&=~USART_CTL0_TEN;
		//LL_USART_DisableIT_TC(USART1);LL_USART_DisableDirectionTx(USART1);
        RSS2.State=RSstWait1Rcv;
        USART2_RX_START();
	     RSS2.CntTimeout=SysTickCntr+100+100+RSS2.ExtraCntTimeout; //10+10ms bonus time to slave begin answer
		goto exii;
	}
	if (RSS2.State==RSstRcv)
	{
		if (IIR&(USART_STAT0_FERR|USART_STAT0_IDLEF)) // |USART_STAT0_LBDF))
		{
//already cleared with reading form SR and DR USART1->SR=(USART_SR_FE|USART_SR_IDLE|USART_SR_LBD);
			if (RSS2.wCRC==0){USARTinUSE_RX_DISABLE(USART2);RSS2.CntTimeout=SysTickCntr;goto exii;}
		}
		*RSS2.BegPnt=(uint8_t)USART_DATA(USART2);
		Crc16ModbusFast(&RSS2.wCRC,RSS2.BegPnt);RSS2.CntTimeout=SysTickCntr+3; //~3chars

		RSS2.BegPnt++;if (RSS2.BegPnt>=&RS2Buffer[RSBUFLEN-1]){RSS2.BegPnt--;USARTinUSE_RX_DISABLE(USART2);RSS2.CntTimeout=SysTickCntr;} //no waiting
		goto exii;
	}
	RSS2.ErrCod=(uint8_t)USART_DATA(USART2);
	RSS2.ErrCod=128; //unknown state, TX&RX must be disabled, go in idle
	USARTinUSE_RX_DISABLE(USART2);
	USART_REG_VAL(USART2, USART_INT_TBE) &= ~BIT(USART_BIT_POS(USART_INT_TBE));
	USART_REG_VAL(USART2, USART_INT_TC) &= ~BIT(USART_BIT_POS(USART_INT_TC));
	USART_CTL0(USART2)&=~USART_CTL0_TEN;
//	LL_USART_DisableIT_TXE(USART1);LL_USART_DisableIT_TC(USART1);LL_USART_DisableDirectionTx(USART1);
	RSS2.State=RSstIdle; //MASTER version
exii:;
}
//........................................................................................
//........................................................................................
//........................................................................................
void ExchgExtDevSomething (uint8_t ExtDevNum, uint8_t GetSubCmdCode)
{
                        RS2Buffer[RsPosDN]=ExtDevNum;RS2Buffer[RsPosCMD]=COMM_EXCHG;RS2Buffer[RsPosSubCMD]=GetSubCmdCode;
                        //  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&OlCo,sizeof(OlCo));
                        RSS2.BegPnt=&RS2Buffer[0];RSS2.EndPnt=&RS2Buffer[RsPosSubCMD+sizeof(OlCo)+2];
                        Calc_CS_WithCopy((uint8_t*)&RS2Buffer[0],(uint8_t*)&RS2Buffer[0],RsPosSubCMD+1+sizeof(OlCo));
}

//........................................................................................
void RSC_GetStatus(void)
{
struct Main_State mMState;
mMState=MState;
  memcpy((void*)&RS0Buffer[RsPosSubCMD+1],(const void*)&mMState,sizeof(mMState));
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(mMState)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(mMState));
}
//........................................................................................
void RSC_SetStatus(void)
{
uint32_t errtmp;
  errtmp=MState.ErrorFlags;
  memcpy((void*)&MState,(const void*)&RS0Buffer[RsPosSubCMD+1],sizeof(MState));
  if (MState.USERI!=EnaChgEFUSERI) {MState.ErrorFlags=errtmp;} //no change of error flags allowed
  memcpy((void*)&RS0Buffer[RsPosSubCMD+1],(const void*)&MState,sizeof(MState));
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(MState)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(MState));
}
//........................................................................................
//........................................................................................
void RSC_Get7ADCvolt(void)
{
  memcpy((void*)&RS0Buffer[RsPosSubCMD+1],(const void*)&fADC_ArrVolt[0],sizeof(fADC_ArrVolt));
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(fADC_ArrVolt)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(fADC_ArrVolt));
}
//........................................................................................
void RSC_Get7ADCspc(void)
{
  memcpy((void*)&RS0Buffer[RsPosSubCMD+1],(const void*)&fADC_ArrSpec[0],sizeof(fADC_ArrSpec));
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(fADC_ArrSpec)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(fADC_ArrSpec));
}
//........................................................................................
void RSC_SetOlCo(void)
{
  memcpy((void*)&OlCo,(const void*)&RS0Buffer[RsPosSubCMD+1],sizeof(OlCo));
  iOlCoLastTimeSet=SysTickCntr;
//  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&OlCo,sizeof(OlCo));
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(OlCo)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(OlCo));
}
//........................................................................................
void RSC_SetPWM2(void)
{
  memcpy((void*)&PWM2_Val,(const void*)&RS0Buffer[RsPosSubCMD+1],sizeof(PWM2_Val));
//  TIM3->CCR2 = (uint16_t)(PWM2_Val*0x0a);
//  memcpy((void*)&RS0Buffer[RsPosSubCMD+1],(const void*)&PWM2_Val,sizeof(PWM2_Val));
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(PWM2_Val)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(PWM2_Val));
}
//........................................................................................
void RSC_SetPWM1(void)
{
  memcpy((void*)&PWM1_Val,(const void*)&RS0Buffer[RsPosSubCMD+1],sizeof(PWM1_Val));
//  TIM11->CCR1 = (uint16_t)(PWM1_Val*0x0a);
//  memcpy((void*)&RS0Buffer[RsPosSubCMD+1],(const void*)&PWM2_Val,sizeof(PWM2_Val));
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(PWM1_Val)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(PWM1_Val));
}
//........................................................................................
void RSC_SetUSERI(void)
{
  memcpy((void*)&MState.USERI,(const void*)&RS0Buffer[RsPosSubCMD+1],sizeof(MState.USERI));
//  memcpy((void*)&RS0Buffer[RsPosSubCMD+1],(const void*)&MState.USERI,sizeof(MState.USERI));
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(MState.USERI)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(MState.USERI));
}
//........................................................................................
void RSC_GetSVpBV(void)
{
uint16_t tvart=CurrentSVpBV;
  memcpy((void*)&RS0Buffer[RsPosSubCMD+1],(const void*)&tvart,sizeof(tvart));
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(tvart)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(tvart));
}
//........................................................................................
void RSC_SetCalib(void)
{
struct ALL_Calib T_CAL;
//  T_CAL=R_CAL;
  memcpy((void*)&T_CAL,(const void*)&RS0Buffer[RsPosSubCMD+1],sizeof(T_CAL));
//  memcpy((void*)&RS0Buffer[RsPosSubCMD+1],(const void*)&R_CAL,sizeof(R_CAL));
  if ((T_CAL.RSCmdCod&0xff00)==RSCmdCod_SetWhat)
  {
    if (T_CAL.RSCmdCod&RSCmdCodMask_SetSPLenght) R_CAL.SPLenght=T_CAL.SPLenght;
    if (T_CAL.RSCmdCod&RSCmdCodMask_SetSPPieces) R_CAL.SPPieces=T_CAL.SPPieces;
    if (T_CAL.RSCmdCod&RSCmdCodMask_SetCProfileN) {R_CAL.CProfileN=T_CAL.CProfileN;SetCProfileInRCFG(T_CAL.CProfileN);}
    if (T_CAL.RSCmdCod&RSCmdCodMask_SetMachineType) R_CAL.MachineType=T_CAL.MachineType;
    if (T_CAL.RSCmdCod&RSCmdCodMask_GetR2_CFGMType) GetMTDefaultR2CFG(T_CAL.MachineType);
    if (T_CAL.RSCmdCod&RSCmdCodMask_GetR2_CFGProfileN) GetCProfileNR2CFG(T_CAL.CProfileN);
  }
  R_CAL.RSCmdCod=T_CAL.RSCmdCod; //may be used anywhere as command????
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(R_CAL)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(R_CAL));
}
//........................................................................................
void RSC_GetCalib(void)
{
//  memcpy((void*)&R_CAL,(const void*)&RS0Buffer[RsPosSubCMD+1],sizeof(R_CAL));
  memcpy((void*)&RS0Buffer[RsPosSubCMD+1],(const void*)&R_CAL,sizeof(R_CAL));
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(R_CAL)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(R_CAL));
}
//........................................................................................
void RSC_SetConfig(void)
{
  memcpy((void*)&R2_CFG,(const void*)&RS0Buffer[RsPosSubCMD+1],sizeof(R2_CFG));
//  memcpy((void*)&RS0Buffer[RsPosSubCMD+1],(const void*)&R_CFG,sizeof(R_CFG));
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(R_CFG)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(R_CFG));
}
//........................................................................................
void RSC_GetConfig(void)
{
//  memcpy((void*)&R2_CFG,(const void*)&RSBuffer[RsPosSubCMD+1],sizeof(R2_CFG));
  memcpy((void*)&RS0Buffer[RsPosSubCMD+1],(const void*)&R2_CFG,sizeof(R2_CFG));
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(R2_CFG)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(R2_CFG));
}
//........................................................................................
void RSC_GetOIST(void)
{
//  memcpy((void*)&OISt,(const void*)&RS0Buffer[RsPosSubCMD+1],sizeof(OISt));
  memcpy((void*)&RS0Buffer[RsPosSubCMD+1],(const void*)&OISt,sizeof(OISt));
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(OISt)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(OISt));
}
//........................................................................................
void RSC_SetOutsA32(void)
{
  memcpy((void*)&OutsA32,(const void*)&RS0Buffer[RsPosSubCMD+1],sizeof(OutsA32));
  RSS0.BegPnt=&RS0Buffer[0];RSS0.EndPnt=&RS0Buffer[RsPosSubCMD+sizeof(OutsA32)+2];
  Calc_CS_WithCopy((uint8_t*)&RS0Buffer[0],(uint8_t*)&RS0Buffer[0],RsPosSubCMD+1+sizeof(OutsA32));
}

//........................................................................................
//===========================================================================================//

void Crc16ModbusFast(volatile uint16_t* lCrc, volatile uint8_t* lData) // sourcer32@gmail.com
{
    static const uint16_t CrcTable[16] = { // Nibble lookup for 0xA001 polynomial
        0x0000,0xCC01,0xD801,0x1400,0xF001,0x3C00,0x2800,0xE401,
        0xA001,0x6C00,0x7800,0xB401,0x5000,0x9C01,0x8801,0x4400 };
    uint16_t Crc;
    uint8_t Data;
    Crc=*lCrc; Data=*lData;
    Crc = Crc ^ Data;

    // Process 8-bits, 4 at a time, or 2 rounds

    Crc = (Crc >> 4) ^ CrcTable[Crc & 0xF];
    Crc = (Crc >> 4) ^ CrcTable[Crc & 0xF];

    *lCrc=Crc;
}

//===========================================================================================//
uint16_t Calc_CS(uint8_t* BegAd, uint16_t Len)
{
  uint16_t  wCRC=0xffff;
  for (;Len !=0 ; Len-- )
    {
     Crc16ModbusFast(&wCRC,BegAd);BegAd++;
    }// end for
   return wCRC;
}
//============================================================================================//
void Calc_CS_WithCopy(uint8_t* BegSourceAddr,uint8_t* BegDestAddr, uint16_t Len)
{
  uint16_t  wCRC=0xffff;
  for (;Len !=0 ; Len-- )
  {
    Crc16ModbusFast(&wCRC,BegSourceAddr);
	*BegDestAddr=*BegSourceAddr;
	BegSourceAddr++;BegDestAddr++;
   }// end for

  *BegDestAddr=(uint8_t)wCRC;
  BegDestAddr++;
  *BegDestAddr=(uint8_t)(wCRC>>8);
}
//==============================================================================================//
