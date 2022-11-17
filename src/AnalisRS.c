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


#define ERRLIN_BUF_LEN (RSBUFLEN/8)  //2*4
uint8_t RSBuffer[RSBUFLEN],RS3Buffer[RSBUFLEN];
uint32_t ErrLineBuff[ERRLIN_BUF_LEN][2];
uint16_t ELB_Idx=0;
volatile struct RS_State RSS,RSS2;
volatile uint8_t CommandToIOM=0;
volatile uint8_t ConfigNowReceived=0;
volatile uint8_t AnalisRSIsBusy=0;   //!=0 if AnalisRS are executed
//USART_TypeDef* UsartInUse;

__STATIC_INLINE void USART0_RX_START(void)
{
	LoRCV_Enbl_485_0;
	USART_REG_VAL(USART0, USART_INT_RBNE) |= BIT(USART_BIT_POS(USART_INT_RBNE));
	USART_CTL0(USART0)|=USART_CTL0_REN;
}
__STATIC_INLINE void USART2_RX_START(void)
{
	LoRCV_Enbl_485_2;
	USART_REG_VAL(USART2, USART_INT_RBNE) |= BIT(USART_BIT_POS(USART_INT_RBNE));
	USART_CTL0(USART2)|=USART_CTL0_REN;
}
__STATIC_INLINE void USART0_TX_START(void)
{
	USART_CTL0(USART0)|=USART_CTL0_TEN;
	USART_REG_VAL(USART0, USART_INT_TBE) |= BIT(USART_BIT_POS(USART_INT_TBE));
}
__STATIC_INLINE void USART2_TX_START(void)
{
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

//!==========================================================================================================
void AnalisRS(void)
{
//	uint8_t t8;
//pak:;
//	  t8=__LDREXB(&AnalisRSIsBusy);  //check only from irq if AnalisRS simultaneously is irq AND backgorund used
//	  t8++;
//	  if (__STREXB(t8,&AnalisRSIsBusy)) goto pak;
//	  AnalisMRS3();
	  AnalisRSmain();

//pakex:;
//      t8=__LDREXB(&AnalisRSIsBusy); //check only from irq if AnalisRS simultaneously is irq AND backgorund used
//      t8--;
//      if (__STREXB(t8,&AnalisRSIsBusy)) goto pakex;

}

void AnalisMRS3(void) //Master mode
{
//int16_t ti16;
//int32_t ti32;
//------------------------------------------------
  if (RSS2.CntAnalisRS<SysTickCntr)
  {
    if (RSS2.State==RSstIdle)
    {
      RSS2.CntAnalisRS=SysTickCntr+127; //12.7ms
      RSS2.RetrCnt=0;
      RSS2.CommCntr++; if (RSS2.CommCntr>1) RSS2.CommCntr=1;

      if (RSS2.CommCntr==1)
      {
        RSS2.CntAnalisRS=SysTickCntr+137; //13.7ms
        RS3Buffer[RsPosDN]=RsExtSlaveDeviceN;RS3Buffer[RsPosCMD]=COMM_SET;RS3Buffer[RsPosSubCMD]=SubCmd_StructSTATUS;
//OstSnd        memcpy((void*)&RS3Buffer[RsPosSubCMD+1],(const void*)&OStSnd,sizeof(OStSnd));
//	    RSS3.BegPnt=&RS3Buffer[0];RSS3.EndPnt=&RS3Buffer[RsPosSubCMD+sizeof(OStSnd)+2];
//        Calc_CS_WithCopy((uint8_t*)&RS3Buffer[0],(uint8_t*)&RS3Buffer[0],RsPosSubCMD+1+sizeof(OStSnd));
        HiTRN_Enbl_485_2;RSS2.State=RSstTrn;RSS2.TraceCod=0;
      } //!RsExtSlaveDeviceN 1 SET SubCmd_StructSTATUS
/*
      if (RSS3.CommCntr==2)
      {
        RSS3.CntAnalisRS=SysTickCntr+137; //13.7ms
        RS3Buffer[RsPosDN]=RsExtSlaveDeviceN;RS3Buffer[RsPosCMD]=COMM_GET;RS3Buffer[RsPosSubCMD]=SubCmd_AINspc;
		RSS3.BegPnt=&RS3Buffer[0];RSS3.EndPnt=&RS3Buffer[RsPosSubCMD+2];
		Calc_CS_WithCopy((uint8_t*)&RS3Buffer[0],(uint8_t*)&RS3Buffer[0],RsPosSubCMD+1);
		HiTRN_Enbl_485_3;RSS3.State=RSstTrn;RSS3.TraceCod=0;
      }//!RsExtSlaveDeviceN 1 GET SubCmd_AINspc
*/
/*
      if (RSS3.CommCntr==3)
      {
        RSS3.CntAnalisRS=SysTickCntr+137; //13.7ms
        RS3Buffer[RsPosDN]=Rs2SlaveDeviceN;RS3Buffer[RsPosCMD]=COMM_SET;RS3Buffer[RsPosSubCMD]=CommandToIOM; //SubCmd_ModeAuto SubCmd_ModeMan SubCmd_Reload
		RSS3.BegPnt=&RS3Buffer[0];RSS3.EndPnt=&RS3Buffer[RsPosSubCMD+2];
		Calc_CS_WithCopy((uint8_t*)&RS3Buffer[0],(uint8_t*)&RS3Buffer[0],RsPosSubCMD+1);
        HiTRN_Enbl_485_3;RSS3.State=RSstTrn;RSS3.TraceCod=0;
      }//!Rs2SlaveDeviceN 3 SET SubCmd_ModeAuto SubCmd_ModeMan SubCmd_Reload
*/
      USART2_TX_START();
      goto exiir;
    }
  }
//maybeSomeUnknDelay:;
  if ((RSS2.State!=RSstRcv)&&(RSS2.State!=RSstWait1Rcv)) goto exiir;
  if (RSS2.CntTimeout>SysTickCntr) goto exiir;
  USARTinUSE_RX_DISABLE(USART2);
  if (RSS2.State==RSstWait1Rcv){RSS2.ErrCod=5;RSS2.RetrCnt++;RSS2.State=RSstIdle;goto exii;}
  RSS2.State=0xff;
  if (RSS2.wCRC!=0)
  {
    if (RSS2.BegPnt==&RS3Buffer[0]) {RSS2.ErrCod=2;}else{RSS2.ErrCod=3;}
    RSS2.RetrCnt++;RSS2.State=RSstIdle;goto exii;
  }
  if (RS3Buffer[RsPosDN]!=RsSlaveDeviceN) { RSS2.RetrCnt++;RSS2.ErrCod=1;RSS2.State=RSstIdle; goto exii;}
//!check in exact command  if (RS3Buffer[RsPosDN]!=RsSlaveDeviceN) { RSS3.RetrCnt++;RSS3.ErrCod=1;RSS3.State=RSstIdle; goto exii;}
  if (RS3Buffer[RsPosCMD]==COMM_GET) //check slave answer on COMM_GET
  {
/*
    if (RS3Buffer[RsPosSubCMD]==SubCmd_AINspc)
    {
      if (RS3Buffer[RsPosDN]!=RsExtSlaveDeviceN) {MState.ErrorFlags|=WF_SlaveExtCommErr;RSS3.RetrCnt++;RSS3.ErrCod=1;RSS3.State=RSstIdle; goto exii;}
      RSS3.RetrCnt=0;RSS3.ErrCod=0;MState.ErrorFlags&=~WF_SlaveExtCommErr;
      memcpy((void*)&fExtADC_ArrSpec[0],(const void*)&RS3Buffer[RsPosSubCMD+1],sizeof(fExtADC_ArrSpec));
      RSS3.State=RSstIdle;RSS3.CntAnalisRS=SysTickCntr; //force next question immediately
      goto exii;
    }
*/
    RSS2.RetrCnt++;RSS2.ErrCod=12;RSS2.State=RSstIdle; //start new question!!!
  }
  if (RS3Buffer[RsPosCMD]==COMM_SET) //check slave answer on COMM_SET (in fact this is some as Exchange)
  {   //no another checks?
    if (RS3Buffer[RsPosSubCMD]==SubCmd_StructSTATUS)
    {

      if (RS3Buffer[RsPosDN]!=RsExtSlaveDeviceN) {RSS2.RetrCnt++;RSS2.ErrCod=1;RSS2.State=RSstIdle; goto exii;} //MState.ErrorFlags|=WF_SlaveExtCommErr;
      RSS2.RetrCnt=0;RSS2.ErrCod=0;//MState.ErrorFlags&=~WF_SlaveExtCommErr;
//OStRet      memcpy((void*)&OStRet,(const void*)&RS3Buffer[RsPosSubCMD+1],sizeof(OStRet));
      RSS2.State=RSstIdle;RSS2.CntAnalisRS=SysTickCntr; //force next question immediately
      goto exii;
    }
/*
    if (RS3Buffer[RsPosSubCMD]==CommandToIOM)
    {
      if (RS3Buffer[RsPosDN]!=Rs2SlaveDeviceN) {MState.ErrorFlags|=WF_SlaveExtCommErr;RSS3.RetrCnt++;RSS3.ErrCod=1;RSS3.State=RSstIdle; goto exii;}
        LoadSnsCopy=*((uint16_t*)&RS3Buffer[RsPosSubCMD+3]);
        MState.USERO=*((uint16_t*)&RS3Buffer[RsPosSubCMD+3]);
        LoadPhaseCopy=*((uint16_t*)&RS3Buffer[RsPosSubCMD+1]);
      RSS3.RetrCnt=0;RSS3.ErrCod=0;MState.ErrorFlags&=~WF_SlaveExtCommErr;
      RSS3.State=RSstIdle;goto exii;
    }
*/
     RSS2.RetrCnt=0;RSS2.ErrCod=0;RSS2.State=RSstIdle;//if (RSS3.CommCntr==3) RSS3.CommCntr=2;
     goto exii;
  }
  RSS2.RetrCnt++;RSS2.ErrCod=13;RSS2.State=RSstIdle;//if (RSS3.CommCntr==3)RSS3.CommCntr=2; //start new question!!!
exii:;
//rserrcnt+=RSS3.RetrCnt;
exiir:;
}
//!==========================================================================================================
void AnalisRSmain(void) //!main channel (in fact works as slave)
{
  if (RSS.State!=RSstRcv) goto exii;
  if (RSS.CntTimeout>SysTickCntr) goto exii;
  USARTinUSE_RX_DISABLE(USART1);RSS.State=0xff;
  if (RSBuffer[RsPosDN]!=RsSlaveDeviceN)
  {
     //RSS.State=RSstNoValidAns;
     RSS.RetrCnt++; RSS.ErrCod=2;
     if (RSS.BegPnt==&RSBuffer[0]) {RSS.ErrCod=0x03;} //no message in buff Err 3???
     RSS.State=RSstWait1Rcv;
     USART0_RX_START();
     goto exii;
  }
  if (RSS.wCRC!=0)
  {
	if (RSS.BegPnt==&RSBuffer[0]) {RSS.ErrCod=0x03;}  //no message in buff Err 3???
	else
    {
		RSS.ErrCod=4;
		if (RSS.BegPnt==&RSBuffer[1]) {RSS.ErrCod=0x00;}  //ONLY ONE char in buff
    }
	RSS.State=RSstWait1Rcv;
	USART0_RX_START();
	goto exii;
  }
//forcedtest:;
			if (RSBuffer[RsPosCMD]==COMM_GET)
			{
              RSS.RetrCnt=0;RSS.ErrCod=0;
              if (RSBuffer[RsPosSubCMD]==SubCmd_StructSTATUS) {RSC_GetStatus();goto cgans;}
              if (RSBuffer[RsPosSubCMD]==SubCmd_AINvolt) {RSC_Get7ADCvolt();goto cgans;}
              if (RSBuffer[RsPosSubCMD]==SubCmd_AINspc) {RSC_Get7ADCspc();goto cgans;}
              if (RSBuffer[RsPosSubCMD]==SubCmd_SVpBV)  {RSC_GetSVpBV();goto cgans;}
              if (RSBuffer[RsPosSubCMD]==SubCmd_Config) {RSC_GetConfig();goto cgans;}
              if (RSBuffer[RsPosSubCMD]==SubCmd_Calib)  {RSC_GetCalib();goto cgans;}
              if (RSBuffer[RsPosSubCMD]==SubCmd_OIST)  {RSC_GetOIST();goto cgans;}

          //unkn cmd
              RSS.RetrCnt++;RSS.ErrCod=1;
              RSS.State=RSstWait1Rcv; //wait new question!!!
              USART0_RX_START();
              goto exii;
cgans:;
              HiTRN_Enbl_485_0;RSS.State=RSstTrn;RSS.TraceCod=0;RSS.LastAnsTimeStamp=SysTickCntr;
              USART0_TX_START(); //idle frame will be generated because TE is set here
              goto exii;
			}
			if (RSBuffer[RsPosCMD]==COMM_SET)
			{
              RSS.RetrCnt=0;RSS.ErrCod=0;
              if (RSBuffer[RsPosSubCMD]==SubCmd_StructSTATUS) {RSC_SetStatus();goto cgans;}
              if (RSBuffer[RsPosSubCMD]==SubCmd_OldComands) {RSC_SetOlCo();goto cgans;}
              if (RSBuffer[RsPosSubCMD]==SubCmd_PWM1) {RSC_SetPWM1();goto cgans;}
              if (RSBuffer[RsPosSubCMD]==SubCmd_PWM2) {RSC_SetPWM2();goto cgans;}
              if (RSBuffer[RsPosSubCMD]==SubCmd_USERI) {RSC_SetUSERI();goto cgans;}
              if (RSBuffer[RsPosSubCMD]==SubCmd_Config) {RSC_SetConfig();goto cgans;}
              if (RSBuffer[RsPosSubCMD]==SubCmd_Calib) {RSC_SetCalib();goto cgans;}
              if (RSBuffer[RsPosSubCMD]==SubCmd_OutsA32) {RSC_SetOutsA32();goto cgans;}
               //unkn cmd
              RSS.RetrCnt++;RSS.ErrCod=5;
              RSS.State=RSstWait1Rcv; //wait new question!!!
              USART0_RX_START();
              goto exii;
            }

        //RSS.State=RSstNoValidAns; //unkn cmd
            RSS.RetrCnt++;RSS.ErrCod=6;
            RSS.State=RSstWait1Rcv; //wait new question!!!
            USART0_RX_START();
            goto exii;
exii:;

}

void USART0_IRQHandler (void) //Slave VERSION
{
unsigned int IIR;
  IIR = USART_STAT0(USART0);

   if ((RSS.State==RSstWait1Rcv))
	   if (IIR&USART_STAT0_RBNE)
		   {
			   RSS.BegPnt=&RSBuffer[0];RSS.wCRC=0xffff;
			   RSS.CntTimeout=SysTickCntr+3; //~3chars
			   *RSS.BegPnt=(uint8_t)USART_DATA(USART0);Crc16ModbusFast(&RSS.wCRC,RSS.BegPnt);
			   RSS.BegPnt++;
			   USART_REG_VAL(USART0, USART_INT_IDLE) |= BIT(USART_BIT_POS(USART_INT_IDLE)); //IDLE may be used as END if line have 2x680ohm!!
			   RSS.State=RSstRcv;
//already cleared with reading form SR and DR if (IIR&(USART_SR_FE|USART_SR_IDLE|USART_SR_LBD)){USART1->SR=(USART_SR_FE|USART_SR_IDLE|USART_SR_LBD);}
			   goto exii;
		   }
	if ((RSS.State==RSstTrn)&&(IIR&USART_STAT0_TBE))
	{
		USART_DATA(USART0)=*RSS.BegPnt;*RSS.BegPnt=0xcc; //sc_add_30-09-2015
		RSS.BegPnt++;RSS.TraceCod++;
		if (RSS.BegPnt>RSS.EndPnt)
		{
			USART_REG_VAL(USART0, USART_INT_TBE) &= ~BIT(USART_BIT_POS(USART_INT_TBE));
			USART_REG_VAL(USART0, USART_INT_TC) |= BIT(USART_BIT_POS(USART_INT_TC));
		  //LL_USART_DisableIT_TXE(USART1); LL_USART_EnableIT_TC(USART1);
	        RSS.State=RSstWaitTC_data;
		}
		goto exii;
	}
	if ((RSS.State==RSstWaitTC_data)&&(IIR&USART_STAT0_TC)) //!!!!THIS IS A SLAVE CASE!!!!!!!!!
	{ //  V--------SLAVE returns in Receiv, wait new quest from mast
		USART_REG_VAL(USART0, USART_INT_TC) &= ~BIT(USART_BIT_POS(USART_INT_TC));
		USART_CTL0(USART0)&=~USART_CTL0_TEN;
		//LL_USART_DisableIT_TC(USART1);LL_USART_DisableDirectionTx(USART1);
        RSS.State=RSstWait1Rcv;
        USART0_RX_START();
// not needed in slave mode		RSS.CntTimeout=SysTickCntr+100; //+10ms bonus time to slave begin answer
		goto exii;
	}
	if (RSS.State==RSstRcv)
	{
		*RSS.BegPnt=(uint8_t)USART_DATA(USART0);

		if (IIR&(USART_STAT0_FERR|USART_STAT0_IDLEF)) // |USART_STAT0_LBDF))
		{
//already cleared with reading form SR and DR USART1->SR=(USART_SR_FE|USART_SR_IDLE|USART_SR_LBD);
			if (RSS.wCRC==0){USARTinUSE_RX_DISABLE(USART0);RSS.CntTimeout=SysTickCntr;goto exii;}
		}
		Crc16ModbusFast(&RSS.wCRC,RSS.BegPnt);RSS.CntTimeout=SysTickCntr+3; //~3chars
		RSS.BegPnt++;if (RSS.BegPnt>=&RSBuffer[RSBUFLEN-1]){RSS.BegPnt--;USARTinUSE_RX_DISABLE(USART0);RSS.CntTimeout=SysTickCntr;} //no waiting
		goto exii;
	}
	RSS.ErrCod=(uint8_t)USART_DATA(USART0);
	RSS.ErrCod=128; //unknown state, TX&RX must be disabled, go in idle
	USARTinUSE_RX_DISABLE(USART0);
	USART_REG_VAL(USART0, USART_INT_TBE) &= ~BIT(USART_BIT_POS(USART_INT_TBE));
	USART_REG_VAL(USART0, USART_INT_TC) &= ~BIT(USART_BIT_POS(USART_INT_TC));
	USART_CTL0(USART0)&=~USART_CTL0_TEN;
//	LL_USART_DisableIT_TXE(USART1);LL_USART_DisableIT_TC(USART1);LL_USART_DisableDirectionTx(USART1);
	RSS.State=RSstWait1Rcv; USART0_RX_START();// SLAVE VERSION!!!!!
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
			   RSS2.BegPnt=&RSBuffer[0];RSS2.wCRC=0xffff;
			   RSS2.CntTimeout=SysTickCntr+3; //~3chars
			   *RSS2.BegPnt=(uint8_t)USART_DATA(USART2);Crc16ModbusFast(&RSS2.wCRC,RSS2.BegPnt);
			   RSS2.BegPnt++;
			   USART_REG_VAL(USART2, USART_INT_IDLE) |= BIT(USART_BIT_POS(USART_INT_IDLE)); //IDLE may be used as END if line have 2x680ohm!!
			   RSS2.State=RSstRcv;
//already cleared with reading form SR and DR if (IIR&(USART_SR_FE|USART_SR_IDLE|USART_SR_LBD)){USART1->SR=(USART_SR_FE|USART_SR_IDLE|USART_SR_LBD);}
			   goto exii;
		   }
	if ((RSS2.State==RSstTrn)&&(IIR&USART_STAT0_TBE))
	{
		USART_DATA(USART2)=*RSS2.BegPnt;*RSS2.BegPnt=0xcc; //sc_add_30-09-2015
		RSS2.BegPnt++;RSS2.TraceCod++;
		if (RSS2.BegPnt>RSS2.EndPnt)
		{
			USART_REG_VAL(USART2, USART_INT_TBE) &= ~BIT(USART_BIT_POS(USART_INT_TBE));
			USART_REG_VAL(USART2, USART_INT_TC) |= BIT(USART_BIT_POS(USART_INT_TC));
		  //LL_USART_DisableIT_TXE(USART1); LL_USART_EnableIT_TC(USART1);
	        RSS2.State=RSstWaitTC_data;
		}
		goto exii;
	}
	if ((RSS2.State==RSstWaitTC_data)&&(IIR&USART_STAT0_TC)) //!!!!THIS IS A MASTER CASE!!!!!!!!!
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
		*RSS2.BegPnt=(uint8_t)USART_DATA(USART2);

		if (IIR&(USART_STAT0_FERR|USART_STAT0_IDLEF)) // |USART_STAT0_LBDF))
		{
//already cleared with reading form SR and DR USART1->SR=(USART_SR_FE|USART_SR_IDLE|USART_SR_LBD);
			if (RSS2.wCRC==0){USARTinUSE_RX_DISABLE(USART2);RSS2.CntTimeout=SysTickCntr;goto exii;}
		}
		Crc16ModbusFast(&RSS2.wCRC,RSS2.BegPnt);RSS2.CntTimeout=SysTickCntr+3; //~3chars
		RSS2.BegPnt++;if (RSS2.BegPnt>=&RSBuffer[RSBUFLEN-1]){RSS2.BegPnt--;USARTinUSE_RX_DISABLE(USART2);RSS2.CntTimeout=SysTickCntr;} //no waiting
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
//........................................................................................
void RSC_GetStatus(void)
{
struct Main_State mMState;
mMState=MState;
  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&mMState,sizeof(mMState));
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(mMState)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(mMState));
}
//........................................................................................
void RSC_SetStatus(void)
{
uint32_t errtmp;
  errtmp=MState.ErrorFlags;
  memcpy((void*)&MState,(const void*)&RSBuffer[RsPosSubCMD+1],sizeof(MState));
  if (MState.USERI!=EnaChgEFUSERI) {MState.ErrorFlags=errtmp;} //no change of error flags allowed
  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&MState,sizeof(MState));
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(MState)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(MState));
}
//........................................................................................
//........................................................................................
void RSC_Get7ADCvolt(void)
{
  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&fADC_ArrVolt[0],sizeof(fADC_ArrVolt));
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(fADC_ArrVolt)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(fADC_ArrVolt));
}
//........................................................................................
void RSC_Get7ADCspc(void)
{
  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&fADC_ArrSpec[0],sizeof(fADC_ArrSpec));
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(fADC_ArrSpec)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(fADC_ArrSpec));
}
//........................................................................................
void RSC_SetOlCo(void)
{
  memcpy((void*)&OlCo,(const void*)&RSBuffer[RsPosSubCMD+1],sizeof(OlCo));
  iOlCoLastTimeSet=SysTickCntr;
//  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&OlCo,sizeof(OlCo));
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(OlCo)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(OlCo));
}
//........................................................................................
void RSC_SetPWM2(void)
{
  memcpy((void*)&PWM2_Val,(const void*)&RSBuffer[RsPosSubCMD+1],sizeof(PWM2_Val));
//  TIM3->CCR2 = (uint16_t)(PWM2_Val*0x0a);
//  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&PWM2_Val,sizeof(PWM2_Val));
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(PWM2_Val)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(PWM2_Val));
}
//........................................................................................
void RSC_SetPWM1(void)
{
  memcpy((void*)&PWM1_Val,(const void*)&RSBuffer[RsPosSubCMD+1],sizeof(PWM1_Val));
//  TIM11->CCR1 = (uint16_t)(PWM1_Val*0x0a);
//  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&PWM2_Val,sizeof(PWM2_Val));
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(PWM1_Val)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(PWM1_Val));
}
//........................................................................................
void RSC_SetUSERI(void)
{
  memcpy((void*)&MState.USERI,(const void*)&RSBuffer[RsPosSubCMD+1],sizeof(MState.USERI));
//  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&MState.USERI,sizeof(MState.USERI));
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(MState.USERI)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(MState.USERI));
}
//........................................................................................
void RSC_GetSVpBV(void)
{
uint16_t tvart=CurrentSVpBV;
  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&tvart,sizeof(tvart));
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(tvart)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(tvart));
}
//........................................................................................
void RSC_SetCalib(void)
{
struct ALL_Calib T_CAL;
//  T_CAL=R_CAL;
  memcpy((void*)&T_CAL,(const void*)&RSBuffer[RsPosSubCMD+1],sizeof(T_CAL));
//  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&R_CAL,sizeof(R_CAL));
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
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(R_CAL)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(R_CAL));
}
//........................................................................................
void RSC_GetCalib(void)
{
//  memcpy((void*)&R_CAL,(const void*)&RSBuffer[RsPosSubCMD+1],sizeof(R_CAL));
  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&R_CAL,sizeof(R_CAL));
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(R_CAL)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(R_CAL));
}
//........................................................................................
void RSC_SetConfig(void)
{
  memcpy((void*)&R2_CFG,(const void*)&RSBuffer[RsPosSubCMD+1],sizeof(R2_CFG));
//  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&R_CFG,sizeof(R_CFG));
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(R_CFG)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(R_CFG));
}
//........................................................................................
void RSC_GetConfig(void)
{
//  memcpy((void*)&R2_CFG,(const void*)&RSBuffer[RsPosSubCMD+1],sizeof(R2_CFG));
  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&R2_CFG,sizeof(R2_CFG));
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(R2_CFG)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(R2_CFG));
}
//........................................................................................
void RSC_GetOIST(void)
{
//  memcpy((void*)&OISt,(const void*)&RSBuffer[RsPosSubCMD+1],sizeof(OISt));
  memcpy((void*)&RSBuffer[RsPosSubCMD+1],(const void*)&OISt,sizeof(OISt));
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(OISt)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(OISt));
}
//........................................................................................
void RSC_SetOutsA32(void)
{
  memcpy((void*)&OutsA32,(const void*)&RSBuffer[RsPosSubCMD+1],sizeof(OutsA32));
  RSS.BegPnt=&RSBuffer[0];RSS.EndPnt=&RSBuffer[RsPosSubCMD+sizeof(OutsA32)+2];
  Calc_CS_WithCopy((uint8_t*)&RSBuffer[0],(uint8_t*)&RSBuffer[0],RsPosSubCMD+1+sizeof(OutsA32));
}

//........................................................................................
//===========================================================================================//
void AddCRC(volatile uint16_t *pbCRC,volatile uint8_t *pbBuffer)
{
uint16_t wBit,wTmp;
  wBit=(uint16_t)*pbBuffer;
 *pbCRC^=wBit;
  wBit=8;
  do    {
          wTmp=*pbCRC&0x1;
          *pbCRC=*pbCRC>>1;
          if ( wTmp!=0 ) *pbCRC^=0xa001;
          wBit--;
        } while (wBit>0);
}

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
