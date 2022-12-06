
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "gd32f4xx.h"
#include "main.h"
#include "stm32f4xx_it.h"
#include "ANSTAT.h"
#include "hw_config.h"
#include "variables.h"
#include "board.h"
#include "AnalisRS.h"
#include "rtot.h"

int* __errno(void);
int myerrno;
int* __errno(void){return &myerrno;}

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
int32_t tmp32t=0;
uint32_t tmp32u=0;
uint16_t tmp16u=0;
float Tempfloat;
volatile uint32_t CntPrnt=0,Cntpwdown=0;
	uint16_t StageFlg=0,it=0;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void RecalSomeVars(void);
uint32_t CalcCRC32a32(uint32_t *pBegin, uint32_t *pEnd);
uint32_t CalcCRC32a32(uint32_t *pBegin, uint32_t *pEnd)
{
 uint32_t dum;
    /* Reset CRC generator */
    CRC_CTL |= (uint32_t)CRC_CTL_RST; //CRC->CR = CRC_CR_RESET;
    dum=CRC_DATA;dum=CRC_DATA;dum=CRC_DATA; //CRC->DR;
pak:;
	CRC_DATA = *pBegin; //CRC->DR;
    dum=CRC_DATA;//CRC->DR;
    if (pBegin!=pEnd) {pBegin++;goto pak;}
  dum=CRC_DATA; //CRC->DR;
 return (dum);
}

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_Init();

  /* USER CODE BEGIN Init */
//no in GD32F4xx  LL_FLASH_EnableInstCache();
//no in GD32F4xx  LL_FLASH_EnableDataCache();
//no in GD32F4xx  LL_FLASH_EnablePrefetch();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  EnableBkpSram();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
#if ENCODER_USED==0
  MX_USART2_UART_Init(); //no in encoder mode
  MX_USART1_UART_Init(); //no in encoder mode
#endif
  MX_USART0_UART_Init();USARTinUSE_RX_START(USART0);
  ADC012_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  InitEncoderInterfaceTim2PA0PA1();
  MX_CRC_Init();

  MX_IWDG_Init(); //and run it

  /* USER CODE BEGIN 2 */
  Old32ADC_Arr[TotalCurrent]=1024*(uint32_t)ADC_IDATA1(ADC0);Old32ADC_Arr[SecFlowRegCurr]=1024*(uint32_t)ADC_IDATA2(ADC0);Old32ADC_Arr[SpindelTemp]=1024*(uint32_t)ADC_IDATA0(ADC0);
  Old32ADC_Arr[OilNozhTemp]=1024*(uint32_t)ADC_IDATA1(ADC1);Old32ADC_Arr[FlowRegCurrent]=1024*(uint32_t)ADC_IDATA2(ADC1);Old32ADC_Arr[PumpRolikiTemp]=1024*(uint32_t)ADC_IDATA0(ADC1);
  Old32ADC_Arr[MPWR_24In]=1024*(uint32_t)ADC_IDATA1(ADC2);//ADC_IndSnzNozh=32*ADC3->JDR1;
EnableTasking=0x10;

//------------------------------------------------------------------------------
   OlCo.ButtonsState=0x0001+0x004;  //i.e. neutral
//#define ModeManualSig   (OlCo.ButtonsState&0x0001)
//#define ModeAutoSig     (OlCo.ButtonsState&0x0004)
//#define KeyForwardSig   (OlCo.ButtonsState&0x0310)
//#define KeyBackwardSig  (OlCo.ButtonsState&0x0320)
//#define KeyCutSig       (OlCo.ButtonsState&0x0340)
//#define ResetKeyCutSig  (OlCo.ButtonsState&=~(0x0340))
//#define KeyAutoCorOn    (OlCo.ButtonsState&0x1000)

//------------------GET STAATISTICS AND CURRENT PROFILE NUMBER------------------
 if (CalcCRC32a32((uint32_t*)&E_CAL.wID_CalibValid,(uint32_t*)&E_CAL.CS32))
 {
   MState.ErrorFlags|=EF_CALisLost; R_CAL=C_CAL;
   R_CAL.CS32=CalcCRC32a32((uint32_t*)&R_CAL.wID_CalibValid,((uint32_t*)&R_CAL.CS32)-1);
   E_CAL=R_CAL;//   WriteEEprom32a32((uint32_t*)&E_CAL,(uint32_t*)&R_CAL,sizeof(R_CAL)>>2);
 }
 else
 {
   R_CAL=E_CAL;
 }
 R_CAL.MachineType=0; //in all cases MachineType is not used as parametr. only as comand

 R_CAL.CProfileN=0;
 if ((*E_LastProfNumC0==*E_LastProfNumC1)&&(*E_LastProfNumC1<10))
 {R_CAL.CProfileN=*E_LastProfNumC1;}
 else
 {tmp32u=0;*E_LastProfNumC0=tmp32u;*E_LastProfNumC1=tmp32u;} // WriteEEprom32a32(E_LastProfNumC0,&tmp32u,1);WriteEEprom32a32(E_LastProfNumC1,&tmp32u,1);}

 //------------------GET WORKING CONFIG ACORDING TO PROFILE NUMBER---------------
 if (CalcCRC32a32((uint32_t*)&E_CFG[R_CAL.CProfileN].wID_ConfigValid,(uint32_t*)&E_CFG[R_CAL.CProfileN].CS32))
 {
   MState.ErrorFlags|=EF_CFGisLost;
   R_CFG=C_CFGiom12INV1;R2_CFG=C_CFGiom12INV1; //in case if config is lost but R_CAL.MachineType=0
   GetMTDefaultR2CFG(R_CAL.MachineType); //DO NOTHING if config is lost and R_CAL.MachineType is invalid
   R_CFG=R2_CFG;
   R_CFG.CS32=CalcCRC32a32((uint32_t*)&R_CFG.wID_ConfigValid,((uint32_t*)&R_CFG.CS32)-1);
   E_CFG[R_CAL.CProfileN]=R_CFG;//   WriteEEprom32a32((uint32_t*)&E_CFG[R_CAL.CProfileN],(uint32_t*)&R_CFG,sizeof(R_CFG)>>2);
 }
 else
 {
   R_CFG=E_CFG[R_CAL.CProfileN];
 }
//------------------------------------------------------------------------------
MState.USERI=ReadyRstUSERI; //1=>Ready after reset
StageFlg=0;
OlCo.Analog2In=100;
OlCo.Analog1In=50;


R2_CFG=R_CFG;
OlCo.CurrentMenu=MenuMain;
MState.RunFlag=0;//ClrCLEN;
RecalSomeVars();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
pak:;
	fwdgt_counter_reload(); //	LL_IWDG_ReloadCounter(IWDG);
		if (CntPrnt<=SysTickCntr)
		{
			StageFlg=1-StageFlg;
			if (StageFlg){OffCPUSecLED;}else{OnCPUSecLED;}

				if (( OlCo.CurrentMenu==MenuTestIOs )) 	 CntPrnt=SysTickCntr+1000; else CntPrnt=SysTickCntr+5000;
				ManualModTimer=ManualModTimer+StageFlg;
		}
		if (RSS0.ErrCod)
		{
			if (StageFlg) {if ((CntPrnt-4000)<=SysTickCntr) {OnCPUSecLED;}}
		}
		AnalisRS();

	//..............................................................................
	//.....................RECALCULATE TEMPS AND VOLTAGES...........................
	// standart float real values
	//  yy1=((float)ADC_Arr[InternalTemp])*1.1-(float)RAW_TSENSE_CAL30; //1.1 bbecauce cal is for 3.0v avdd
	//  y21=(float)(RAW_TSENSE_CAL110-RAW_TSENSE_CAL30);
	//  xInternalTemp=30.0+(yy1/y21)*(110.0-30.0))
	//................
	   for (it=0;it<AllADCCh;it++)   //<2uS
	   {
	     fADC_ArrVolt[it]=Uvfrom12bit(ADC_Arr[it]); //*cal1+cal2
	   }
	     AnalisRS();

	   fADC_ArrSpec[OilNozhTemp]=rtot(OhmFromV(fADC_ArrVolt[OilNozhTemp]));  //temp in C
	     AnalisRS();
	   fADC_ArrSpec[SpindelTemp]=rtot(OhmFromV(fADC_ArrVolt[SpindelTemp]));  //temp in C
	     AnalisRS();
	   fADC_ArrSpec[PumpRolikiTemp]=rtot(OhmFromV(fADC_ArrVolt[PumpRolikiTemp]));  //temp in C
	     AnalisRS();
	   fADC_ArrSpec[FlowRegCurrent]=fADC_ArrVolt[FlowRegCurrent]; //shunt is 1ohm
	     AnalisRS();

	//  fADC_ArrSpec[TotalCurrent]=ADC_ArrRaw[TotalCurrent]]*);

	   fADC_ArrSpec[TotalCurrent]=fADC_ArrVolt[TotalCurrent]/(19.8f*0.01f); //Rshunt=0.01ohm gain 19.8
	   fADC_ArrSpec[SecFlowRegCurr]=fADC_ArrVolt[SecFlowRegCurr];
	   fADC_ArrSpec[MPWR_24In]=fADC_ArrVolt[MPWR_24In]*9.85f; //~10 is R divider ratio (22+7.5)/3.3

	//******************************************************************************
	 if (( OlCo.CurrentMenu==MenuTestIOs )) goto background_test_bypass;
	//******************************************************************************

	//....................................................................................
	//------------------CHECK FOR UNDER/OVERHEAT AND STOP---------------------------------

	   if(((int16_t)fADC_ArrSpec[OilNozhTemp]>R_CFG.MaxOilTemp)&&((R_CFG.ReservSystBits&EF_OvrHeatOil)==0))
	    {DiN();MState.ErrorFlags|=EF_OvrHeatOil;EiN();} //MState.RunFlag=RM_Error;
	    else {MState.ErrorFlags&=~EF_OvrHeatOil;}

	   if(((int16_t)fADC_ArrSpec[SpindelTemp]>R_CFG.MaxMotorTemp)&&((R_CFG.ReservSystBits&EF_OvrHeatSpindel)==0))
	    {DiN();MState.ErrorFlags|=EF_OvrHeatSpindel;EiN();} //MState.RunFlag=RM_Error;
	    else {MState.ErrorFlags&=~EF_OvrHeatSpindel;}

	   if(((int16_t)fADC_ArrSpec[PumpRolikiTemp]>R_CFG.MaxMotorTemp)&&((R_CFG.ReservSystBits&EF_OvrHeatPump)==0))
	    {DiN();MState.ErrorFlags|=EF_OvrHeatPump;EiN();} //MState.RunFlag=RM_Error;
	    else {MState.ErrorFlags&=~EF_OvrHeatPump;}

	   AnalisRS();
	 if ((R_CFG.SysB3&0x04))
	 {
	   if((fADC_ArrSpec[OilNozhTemp]<-30.0f)&&((R_CFG.ReservSystBits&EF_UnderThrOil)==0))
	     {DiN();MState.ErrorFlags|=EF_UnderThrOil;MState.RunFlag=RM_Error;EiN();}
	     else {MState.ErrorFlags&=~EF_UnderThrOil;}

	   if((fADC_ArrSpec[SpindelTemp]<-30.0f)&&((R_CFG.ReservSystBits&EF_UnderThrSpindel)==0))
	     {DiN();MState.ErrorFlags|=EF_UnderThrSpindel;MState.RunFlag=RM_Error;EiN();}
	     else {MState.ErrorFlags&=~EF_UnderThrSpindel;}

	   if((fADC_ArrSpec[PumpRolikiTemp]<-30.0f)&&((R_CFG.ReservSystBits&EF_UnderThrPump)==0))
	     {DiN();MState.ErrorFlags|=EF_UnderThrPump;MState.RunFlag=RM_Error;EiN();}
	     else {MState.ErrorFlags&=~EF_UnderThrPump;}
	 }

	     AnalisRS();

	//!debug MState.USERO=((int)RSS.RetrCnt)*256+(int)RSS.ErrCod;
	MState.USERO=NewIntgCurrentSpeed;
	if (NewIntgCurrentSpeed) ManualModTimer=0;
	if (R_CFG.SystB2&0x02) ManualModTimer=0;  //invertor models

	  // MState.ErrorFlags=R_CFG.ReservSystBits;
	//------------------------------------------------------------------------------
	 if (LowPwrCnt>(R_CFG.DelayToSleep))
	 {
	    //off something? change MState.RunFlag???? done in 100us irq
	    //now store cal & go in sleep to reset??? /for reset r62p instead r62 on borad v4/

	    //!!! WRITE_CAL
	   if (R_CAL.wID_CalibValid==0xabcd55aa)
	   {
			 R_CAL.TotalPowerCicles++;
	     R_CAL.CS32=CalcCRC32a32((uint32_t*)&R_CAL.wID_CalibValid,((uint32_t*)&R_CAL.CS32)-1);
	     E_CAL=R_CAL; //WriteEEprom32a32((uint32_t*)&E_CAL,(uint32_t*)&R_CAL,sizeof(R_CAL)>>2);
	     R_CAL.wID_CalibValid=0x355113;Cntpwdown=SysTickCntr;
	   }
	 }
	 else
	 {
	   if (R_CAL.wID_CalibValid==0x355113) {if ((Cntpwdown+20000)<SysTickCntr) R_CAL.wID_CalibValid=0xabcd55aa;} //pwr is up again->continue working
	 }

	//------------------------------------------------------------------------------
	//------------------------------------------------------------------------------
	//------------------CHECK Invertor Faults FOR DISPLAY-------------------------------------
	if (Get_F1err) {Inv1FaultCnt>>=1;}else{Inv1FaultCnt++;}
	if (Inv1FaultCnt>=20) {DiN();MState.ErrorFlags|=EF_Inv1Fault;MState.RunFlag=RM_Error;EiN();}
	if (Inv1FaultCnt==1) {DiN();MState.ErrorFlags&=~EF_Inv1Fault;EiN();}
	if (Get_F2err) {Inv2FaultCnt>>=1;}else{Inv2FaultCnt++;}
	if (Inv2FaultCnt>=20) {DiN();MState.ErrorFlags|=EF_Inv2Fault;MState.RunFlag=RM_Error;EiN();}
	if (Inv2FaultCnt==1) {DiN();MState.ErrorFlags&=~EF_Inv2Fault;EiN();}
	if (Get_F3err) {Inv3FaultCnt>>=1;}else{Inv3FaultCnt++;}
	if (Inv3FaultCnt>=20) {DiN();MState.ErrorFlags|=EF_Inv3Fault;MState.RunFlag=RM_Error;EiN();}
	if (Inv3FaultCnt==1) {DiN();MState.ErrorFlags&=~EF_Inv3Fault;EiN();}

	//------------------------------------------------------------------------------
	//------------------------------------------------------------------------------
	//  if (GetFanThy) MState.ErrorFlags|=EF_HSE_FailedToStart;else MState.ErrorFlags&=~EF_HSE_FailedToStart;

	//-----------------CHECK FOR FAN ON-OFF-----------------------------------------
	if (MState.RunFlag==RM_Error) {FanTHY3OFF;} else
	{
	 if  (R_CFG.SystB2&0x02) //invertor model maximal temp from all motors
	 {
	   if ((R_CFG.SystB2&0x80))  goto exiifan;////is 1 invertor roll+knife? then FanThy is used to switch knife contactor
	// ??  if  (R_CFG.SystB2&0x80) goto exiifan; // model without fan, Thyristor is used to control knife contactor
	   if ( (((int16_t)fADC_ArrSpec[OilNozhTemp])>R_CFG.MinFanTemp)||
	       (((int16_t)fADC_ArrSpec[PumpRolikiTemp])>R_CFG.MinFanTemp)||
	       (((int16_t)fADC_ArrSpec[SpindelTemp])>R_CFG.MinFanTemp) ) {FanTHY3ON; goto exiifan;}

	     AnalisRS();

	   if ( (((int16_t)fADC_ArrSpec[OilNozhTemp])<(R_CFG.MinFanTemp-3))&&
	       (((int16_t)fADC_ArrSpec[PumpRolikiTemp])<(R_CFG.MinFanTemp-3))&&
	       (((int16_t)fADC_ArrSpec[SpindelTemp])<(R_CFG.MinFanTemp-3)) ) FanTHY3OFF;

	 }
	 else
	 {  //only hydr oil temp is criteria
	   if (((int16_t)fADC_ArrSpec[OilNozhTemp])>R_CFG.MinFanTemp) { FanTHY3ON; goto exiifan;}
	   if (((int16_t)fADC_ArrSpec[OilNozhTemp])<(R_CFG.MinFanTemp-3)) FanTHY3OFF;
	 }
	}
	exiifan:
	     AnalisRS();
	//------------------------------------------------------------------------------
	//----------------CHECK for HEATER ON/OFF
	if (HiMStopCnt>MStopTimeCut) {OffNPNHeater;} else  //BRAKE AND HEATER ARE ON SAME OUT  on v3 boards
	{
	  if (MState.RunFlag==RM_Neutral)
	  {
	    OffNPNHeater;OffNPNBrakeMotor;MState.ErrorFlags&=~WF_IntTempHeaterON;
	 		if (KeyCutSig) R_CAL.SPPieces=9999;
	  }
	  if (MState.RunFlag==RM_Manual)
	  {
	    if (((int16_t)fADC_ArrSpec[OilNozhTemp])>R_CFG.MaxIntTemp+1) {OffNPNHeater;MState.ErrorFlags&=~WF_IntTempHeaterON;}
	    if (((int16_t)fADC_ArrSpec[OilNozhTemp])<(R_CFG.MaxIntTemp)) {OnNPNHeater;MState.ErrorFlags|=WF_IntTempHeaterON;}
	  }
	}
	//------------------------------------------------------------------------------

	//------------------CHECK FOR AUTO OR MANUAL MODE-------------------------------
	//.................run in auto, set statistics...and etc

	 if (MState.RunFlag==RM_Neutral)
	 {
	   if (LowCntSpindelEnable>(SPINDEL_CONTACTOR_DELAY+2000)){OffThySpindelENBL;MState.ErrorFlags|=EF_SpindelEnbldInNEUT;}else{MState.ErrorFlags&=~EF_SpindelEnbldInNEUT;}
	 }

	 if ((LowMAutoCnt>MAutoTimeCut)) //auto wait contactor to close
	 {
	  if (KeyCutSig) //------------manual cut in auto mode-----------------------
	  {DiN();OffNPNCutOff;RM_Delay=R_CFG.TimeForCutting;MState.RunFlag=RM_Cutting;EiN();ResetKeyCutSig;}

	     if (MState.RunFlag==RM_Wait)
	     {
	       if (MState.ErrorFlags&(EF_OvrHeatOil|EF_OvrHeatSpindel|EF_OvrHeatPump)) {MState.RunFlag=RM_Error;}
	 			 else
	             {
	       if (R_CAL.SPPieces>0)
	       {
	         if (((R_CFG.SystemBits&0x30)!=0)&&(LowEndSwitchCnt>ESwitchTimeCut))   //check for end switch block in two ES modes
	         {
	            if ((R_CFG.ReservSystBits&EF_EndSwitchBlock)==0x0000) {DiN();MState.ErrorFlags|=EF_EndSwitchBlock;MState.RunFlag=RM_Error; OlCo.CurrentMenu=MenuMain;EiN();}
	         }
	         else
	         {
	           R_CAL.SPPieces--;
	           Tempfloat=(float)(int32_t)R_CAL.SPLenght;
	           if (R_CFG.SystemBits&0x08)
	             {R_CAL.TotalLenght+=R_CAL.SPLenght*10;}   //12addon  //mm->cm
	             else
	             {R_CAL.TotalLenght+=R_CAL.SPLenght;}
	           if (R_CFG.SystemBits&0x08) {Tempfloat=Tempfloat*10.0f;} //12addon  //mm->cm
	           Scmoving(Tempfloat,pRgM); //???? this board have -moving
	         }
	       }
			     }
	     }
	  }
	//--------------------------------------------------------------------------------------
	 AnalisRS(); //from main 100us interrupt ?????
	//--------------------------------------------------------------------------------------
	// if ((RSS.LastAnsTimeStamp+30000)<SysTickCntr) //last 3s no any communication
	// {
	//  MState.RunFlag==RM_Error;
	//  GPIOA->ODR=0;GPIOB->ODR=0;GPIOC->ODR=0;GPIOD->ODR=0;
	// }

	background_test_bypass:;
	//..............................................................................
	 OISt.AODR=GPIO_OCTL(GPIOA);OISt.BODR=GPIO_OCTL(GPIOB);OISt.CODR=GPIO_OCTL(GPIOC);OISt.DODR=GPIO_OCTL(GPIOD);
	 //OISt.AODR=0xffff;OISt.BODR=0xffff;OISt.CODR=GPIOC->ODR;OISt.DODR=GPIOD->ODR;
	 OISt.AIDR=GPIO_ISTAT(GPIOA);OISt.BIDR=GPIO_ISTAT(GPIOB);OISt.CIDR=GPIO_ISTAT(GPIOC);OISt.DIDR=GPIO_ISTAT(GPIOD);
	 extern uint16_t IndSnzNozhFiltered;
	 extern volatile uint16_t Get_InACDC2;
	 if (IndSnzNozhFiltered>2048) OISt.CIDR|=0x0008; else OISt.CIDR&=~0x0008;
	 if (Get_InACDC2==0) OISt.CIDR&=~0x0020; else OISt.CIDR|=0x0020;
	//..............................................................................

	     AnalisRS();

	if (MState.USERI==WriteR2ConfigUSERI)
	{
	   R2_CFG.wID_ConfigValid=0xabcd55aa;
	   R2_CFG.CS32=CalcCRC32a32((uint32_t*)&R2_CFG.wID_ConfigValid,((uint32_t*)&R2_CFG.CS32)-1);
	   if (R_CAL.CProfileN>9) {MState.USERI=BadCprNLastCmdUSERI;goto nowrtr2;}
	   E_CFG[R_CAL.CProfileN]=R2_CFG; // WriteEEprom32a32((uint32_t*)&E_CFG[R_CAL.CProfileN],(uint32_t*)&R2_CFG,sizeof(R_CFG)>>2);
		 tmp32u=R_CAL.CProfileN;
		 *E_LastProfNumC0=tmp32u;*E_LastProfNumC1=tmp32u; //WriteEEprom32a32(E_LastProfNumC0,&tmp32u,1);WriteEEprom32a32(E_LastProfNumC1,&tmp32u,1);
	   R_CFG=R2_CFG;
	     AnalisRS();
	   RecalSomeVars();
	   MState.USERI=ReadyLastCmdUSERI;
	nowrtr2:;
	}
 	if (OlCo.CurrentMenu==MenuTestIOs)
 	{
 	    tmp16u=(uint16_t)OutsA32;
 	    it=(uint16_t)(OutsA32>>16);
 	    tmp16u=tmp16u&it;
 	//     OISt.AODR=tmp16u;OISt.BODR=it;OISt.CODR=GPIOC->ODR;OISt.DODR=GPIOD->ODR;
 	if (MState.USERI==SetOutsA32USERI)
 	{
 		     MState.USERI=ReadyLastCmdUSERI;
 	    if (tmp16u)GPIO_BOP(GPIOA)=it; else GPIO_BC(GPIOA)=it;
 	}
 	if (MState.USERI==SetOutsB32USERI)
 	{
 		     MState.USERI=ReadyLastCmdUSERI;
 	    if (tmp16u) GPIO_BOP(GPIOB)=it; else GPIO_BC(GPIOB)=it;
 	}
 	if (MState.USERI==SetOutsC32USERI)
 	{
 		     MState.USERI=ReadyLastCmdUSERI;
 	    if (tmp16u) GPIO_BOP(GPIOC)=it; else GPIO_BC(GPIOC)=it;
 	}
 	if (MState.USERI==SetOutsD32USERI)
 	{
 		    MState.USERI=ReadyLastCmdUSERI;
 	    if (tmp16u) GPIO_BOP(GPIOD)=it; else GPIO_BC(GPIOD)=it;
 	}

 	}
	goto pak;

	}

	void   GetMTDefaultR2CFG(uint8_t  MachineType)
	{
	 // R2 hold current config if (MachineType==MT_CurrnetR_CFG) R2_CFG=R_CFG;

	 if (MachineType==MT_iom12S) R2_CFG=C_CFGiom12S;
	 if (MachineType==MT_iom6) R2_CFG=C_CFGiom6;
	 if (MachineType==MT_iom6S) R2_CFG=C_CFGiom6S;
	 if (MachineType==MT_iom12D) R2_CFG=C_CFGiom12D;
	 if (MachineType==MT_iom16D) R2_CFG=C_CFGiom16D;
	 if (MachineType==MT_iom16F) R2_CFG=C_CFGiom16F;
	 if (MachineType==MT_iom10S) R2_CFG=C_CFGiom10S;
	 if (MachineType==MT_iom12INV1) R2_CFG=C_CFGiom12INV1;
	 if (MachineType==MT_iom8INV) R2_CFG=C_CFGiom8INV;
	 if (MachineType==MT_iom16INV1) R2_CFG=C_CFGiom16INV1;
	  if (MachineType==MT_iom8INV2) R2_CFG=C_CFGiom8INV2;
	}

	void GetCProfileNR2CFG (uint8_t PN)
	{
	if (PN>9) PN=9;
	 if (CalcCRC32a32((uint32_t*)&E_CFG[PN].wID_ConfigValid,(uint32_t*)&E_CFG[PN].CS32))
	 {
	   R2_CFG=R_CFG;
	 }
	 else
	 {
	   R2_CFG=E_CFG[PN];
	 }

	}

	void SetCProfileInRCFG (uint8_t PN)
	{
	if (PN>9) PN=9;
	 if (CalcCRC32a32((uint32_t*)&E_CFG[PN].wID_ConfigValid,(uint32_t*)&E_CFG[PN].CS32))
	 {
	   R2_CFG=R_CFG;
	   R2_CFG.wID_ConfigValid=0xabcd55aa;
	   R2_CFG.CS32=CalcCRC32a32((uint32_t*)&R2_CFG.wID_ConfigValid,((uint32_t*)&R2_CFG.CS32)-1);
	   E_CFG[PN]=R2_CFG; //WriteEEprom32a32((uint32_t*)&E_CFG[R_CAL.CProfileN],(uint32_t*)&R2_CFG,sizeof(R_CFG)>>2);
		 tmp32u=PN;
		 *E_LastProfNumC0=tmp32u;*E_LastProfNumC1=tmp32u; //WriteEEprom32a32(E_LastProfNumC0,&tmp32u,1);WriteEEprom32a32(E_LastProfNumC1,&tmp32u,1);
	   R_CFG=R2_CFG;
	 }
	 else
	 {
	   tmp32u=PN;
	   *E_LastProfNumC0=tmp32u;*E_LastProfNumC1=tmp32u; //WriteEEprom32a32(E_LastProfNumC0,&tmp32u,1);WriteEEprom32a32(E_LastProfNumC1,&tmp32u,1);
	   R_CFG=E_CFG[PN];
	 }
	   RecalSomeVars();
	}


	void RecalSomeVars(void)
	{
	   if ( (LowMAutoCnt>MAutoTimeCut) ) {MState.RunFlag=RM_Error; OlCo.CurrentMenu=MenuMain;}
	//   GetCTime(&CTime);
	//   EncTstRots=10;FltStartRampLowSpeed=((float)R_CFG.StartRampLowSpeed)/50.0f; //if encoder Test will be used
	   fScaleCurInp=((float)R_CFG.CcMaxSpdRegout)/100.0f;
	   if (R_CFG.SystemBits&0x01) SetKeyAutoCorOn; else ClrKeyAutoCorOn;
	   if (R_CFG.SystB2&0x04) {HiSpinRev_ON;} else {LoSpinRev_OFF;}

	   timer_autoreload_value_config(TIMER2,PWM_AUTORELOAD_DEF_1200-(R_CFG.DebitDeadZeroCorection>>2));
	   timer_autoreload_value_config(TIMER3,PWM_AUTORELOAD_DEF_1200-(R_CFG.DebitDeadZeroCorection>>2));
//	   LL_TIM_SetAutoReload(TIM3,PWM_AUTORELOAD_DEF_1200-(R_CFG.DebitDeadZeroCorection>>2)); //PWM correction
//	   LL_TIM_SetAutoReload(TIM4,PWM_AUTORELOAD_DEF_1200-(R_CFG.DebitDeadZeroCorection>>2)); //PWM correction

	   Ain1Bias=(uint16_t)(R_CFG.AIN1hAIN2lOffset>>16);Ain2Bias=(uint16_t)R_CFG.AIN1hAIN2lOffset;
	}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
