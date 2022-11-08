#include "gd32f4xx.h"

#include "stm32f4xx_it.h"  //for SysTickCntr
#include "hw_config.h"
#include "ANSTAT.h"
#include "board.h"
#include "variables.h"
#include "AnalisRS.h"
#include <stdio.h>
#include <math.h>

//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//****************************************************************************************************
uint32_t IndSnzNozhOldFilter=0;
uint16_t ADC_IndSnzNozh=0,IndSnzNozhFiltered=0,ARTErrCnt=0;
uint16_t CntRackDelayToOpen=0,CntRackHoldTime=0;
volatile uint16_t Get_InACDC2=0; //digital input simulated with ADC Channel

RegStructure  dRgM;      //!<-Main reg structure
RegStructure *pRgM=&dRgM;
//----------------------------------------------------------------------------------------------------
void AdcsInt50uS(void)
{
// OnSecLED;
int16_t itmp16u;

//.....................Get.&.FLTERING ALL ADCs.......................................
//!CHANNELS PB0in8=ALL_total7transistorsIOpAmp PC5in15=IOM_SecFlowReg PA4in4=PIR_IN - IOM_SpindelTemp(TransI)    '''''''''''
 if (ADC_STAT(ADC0) & ADC_FLAG_EOIC)
 { ADC_STAT(ADC0)&=~ADC_FLAG_EOIC;
   ADC_ArrRaw[TotalCurrent]=ADC_IDATA1(ADC0);ADC_ArrRaw[SecFlowRegCurr]=ADC_IDATA2(ADC0);ADC_ArrRaw[SpindelTemp]=ADC_IDATA0(ADC0);
// ADC_ArrRaw[TotalCurrent]=ADC1->JDR2;ADC_ArrRaw[SecFlowRegCurr]=ADC1->JDR3;ADC_ArrRaw[SpindelTemp]=ADC1->JDR1;
 } else {ARTErrCnt++;}

 //!CHANNELS PA6in6=tOIL - IOM_OilNozhTemp(ThyristorT) PA7in7=IOM_FlowRegCurrent PA5in5=PIR_OUT - IOM_PumpRolikiTemp(TransU)'''''''''''''''''
 if (ADC_STAT(ADC1) & ADC_FLAG_EOIC)
 { ADC_STAT(ADC1)&=~ADC_FLAG_EOIC;
   ADC_ArrRaw[OilNozhTemp]=ADC_IDATA1(ADC1);ADC_ArrRaw[FlowRegCurrent]=ADC_IDATA2(ADC1);ADC_ArrRaw[PumpRolikiTemp]=ADC_IDATA0(ADC1);
// ADC_ArrRaw[OilNozhTemp]=ADC2->JDR2;ADC_ArrRaw[FlowRegCurrent]=ADC2->JDR3;ADC_ArrRaw[PumpRolikiTemp]=ADC2->JDR1;
 } else {ARTErrCnt++;}

 //!ADC CHANNELS  PC2in12=PWR24 (Oil or ForwMotorTemp on Svarka) PC3in13=tWATER - IOM_FbNozh (Water on Svarka)     ''''
 if (ADC_STAT(ADC2) & ADC_FLAG_EOIC)
 { ADC_STAT(ADC2)&=~ADC_FLAG_EOIC;
   ADC_ArrRaw[MPWR_24In]=ADC_IDATA1(ADC2);ADC_IndSnzNozh=ADC_IDATA0(ADC2);
// ADC_ArrRaw[MPWR_24In]=ADC3->JDR2;ADC_IndSnzNozh=ADC3->JDR1;
 }  else {ARTErrCnt++;}

  if (ARTErrCnt>12) MState.ErrorFlags|=EF_ADCNotReady;

   for (itmp16u=0;itmp16u<AllADCCh;itmp16u++)   //<2uS
   {
       ADCValue=ADC_ArrRaw[itmp16u];
       Old32ADC_Arr[itmp16u]=ADCValue+((Old32ADC_Arr[itmp16u]*1023)>>10);
       ADC_Arr[itmp16u]=(uint16_t)(Old32ADC_Arr[itmp16u]>>10);
   }
   if (ADC_ArrRaw[SecFlowRegCurr]>2048){if (Get_InACDC2<16) Get_InACDC2++;} // simulate di with adc
   if (ADC_ArrRaw[SecFlowRegCurr]<1024){if (Get_InACDC2) Get_InACDC2--;} // simulate di with adc

   IndSnzNozhOldFilter=ADC_IndSnzNozh+((IndSnzNozhOldFilter*15)>>4);
   IndSnzNozhFiltered=(uint16_t)(IndSnzNozhOldFilter>>4);
   ADCValue = ADC_ArrRaw[FlowRegCurrent];  //for main flow regulator

   ADC_CTL1(ADC0) |= (uint32_t)ADC_CTL1_SWICST;
   ADC_CTL1(ADC1) |= (uint32_t)ADC_CTL1_SWICST;
   ADC_CTL1(ADC2) |= (uint32_t)ADC_CTL1_SWICST;
//   ADC1->CR2 |= ADC_CR2_JSWSTART;  ADC2->CR2 |= ADC_CR2_JSWSTART;  ADC3->CR2 |= ADC_CR2_JSWSTART;
}
//----------------------------------------------------------------------------------------------------
//****************************************************************************************************
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//****************************************************************************************************
//----------------------------------------------------------------------------------------------------

int32_t i32sabs(int32_t ab)
{
    if ( ab<0)
    {
        ab=0-ab;
    }
    return(ab);
}
int16_t i16sabs(int16_t ab)
{
    if ( ab<0)
    {
        ab=0-ab;
    }
    return(ab);
}

#define MaxStopElemnts 100

const float ArrStopPow05[MaxStopElemnts]={ //sqrt(x) x e (0,100)
0.03,     0.05,     0.07,     0.1,     0.2,     0.3,      0.4,      0.5,    0.6,      0.7,     0.8, 0.9,
0.994987,1.410673,1.729161,1.997498,2.233835,2.447456,2.643873,2.826674,2.998351,3.160717,
3.315140,3.462684,3.604192,3.740351,3.871724,3.998784,4.121928,4.241499,4.357790,4.471058,
4.581526,4.689393,4.794834,4.898005,4.999048,5.098088,5.195240,5.290609,5.384289,5.476367,
5.566921,5.656026,5.743733,5.830121,5.915246,5.999164,6.081924,6.163573,6.244154,6.323709,
6.402275,6.479889,6.556584,6.632392,6.707344,6.781466,6.854788,6.927334,6.999127,7.070192,
7.140550,7.210221,7.279225,7.347581,7.415307,7.482420,7.548936,7.614872,7.680241,7.745059,
7.809339,7.873094,7.936337,7.999079,8.061357,8.123158,8.184491,8.245369,8.305800,8.365796,
8.425364,8.484513,8.543254,8.601593,8.659538,8.717099,8.774282,8.831096,8.887546,8.943639,
8.999383,9.054784,9.109847,9.164580,9.218988,9.273077,9.326852,9.380320 //,9.433483,9.486349,
//9.538921//,9.591207,9.643208,9.694930//,9.746378,9.797556,9.848468//,9.899117,9.949510,9.999647
};
const float ArrStopLinear[MaxStopElemnts]={
//0.030000,0.040000,0.060000,0.080000,0.100002,0.250004,0.4006,  0.50009, 0.65,    0.8,     0.95,
0.03,     0.05,     0.1,      0.2,     0.3,     0.4,      0.5,      0.6,    0.7,      0.8,     0.9, 1.0,
1.109016,1.209018,1.309020,1.409022,1.509025,1.609027,1.709029,1.809032,1.909034,2.009036,
2.109039,2.209041,2.309043,2.409045,2.509048,2.609050,2.709052,2.809055,2.909057,3.009059,
3.109061,3.209064,3.309047,3.409030,3.509013,3.608997,3.708980,3.808963,3.908946,4.008930,
4.108913,4.208896,4.308879,4.408863,4.508846,4.608829,4.708812,4.808795,4.908779,5.008762,
5.108745,5.208728,5.308712,5.408695,5.508678,5.608661,5.708644,5.808628,5.908611,6.008594,
6.108577,6.208560,6.308544,6.408527,6.508547,6.608569,6.708590,6.808611,6.908633,7.008654,
7.108675,7.208697,7.308718,7.408740,7.508761,7.608782,7.708804,7.808825,7.908846,8.008868,
8.108890,8.208911,8.308932,8.408954,8.508975,8.608996,8.709018,8.809039,8.909060,9.009082,
9.109103,9.209125,9.309146,9.409167,9.509189,9.609210,9.709231,9.809253 //,9.909274
};

const float ArrStopUltraLowSpeeds[12]=
{0.001,0.004,0.006,0.01,0.03,0.05,0.08,0.1,0.3,0.5,0.7,0.9};
//1     2     3     4     5    6   7   8    9  10  11  12

float StopKula(RegStructure *Rs, float CurSp,uint16_t CurrentAngleRest)
{
float  fcsp,fcs;
uint16_t StopTableSelector;
  StopTableSelector=R_CFG.SystemBits&0x02; //bit 1
  fcs=CurSp;
  if (CurrentAngleRest<MaxStopElemnts)
  {
    if (R_CFG.StopRampLowSpeed!=0)
    {
      if ( StopTableSelector==0)
      {
        fcsp=ArrStopPow05[CurrentAngleRest];
      }
      else //if ( StopTableSelector>0) //1 in our case
      {
        fcsp=ArrStopLinear[CurrentAngleRest];
      }
    }
    else
    {
      if ( StopTableSelector==0)
      {
        if (CurrentAngleRest<12) {fcsp=ArrStopUltraLowSpeeds[CurrentAngleRest];}else{fcsp=ArrStopPow05[CurrentAngleRest];}
      }
      else //if ( StopTableSelector>0) //1 in our case
      {
        if (CurrentAngleRest<12) {fcsp=ArrStopUltraLowSpeeds[CurrentAngleRest];}else{fcsp=ArrStopLinear[CurrentAngleRest];}
      }
    }
    fcsp=fcsp*10.0f*Rs->f1divSpeedIntgTime; //precalculated 1/SpeedIntgTime
    if (fcsp<fabs(CurSp))
    {
      fcs=fcsp;
      if (CurSp<0) fcs=0-fcs; //restore sign
    }
  }

  return(fcs);
}


//**********************************************************************************************************
void CalcRegulator(RegStructure *Rs)
{
  Rs->iCurrentPos=MState.CurrentPosition;
  if (Rs->rState&rSta_UncondREINIT) goto recalc;
  if (Rs->rState&rSta_NOCheckForRECALC) goto recisok;
//....check for reinit
//!!  if (Rs->fMaxMovingSpeed!=Rs->fMaxMovingSpeedCopy) goto recalc;
//!!  if (Rs->iTargetPosCopy!=Rs->iTargetPos) goto recalc;
//! we suppose ramps, and other coeffs cannot be changed during work
  goto recisok;
recalc:; //=============RECALC==============================
   Rs->fMaxMovingSpeedCopy=Rs->fMaxMovingSpeed;
   Rs->iTargetPosCopy=Rs->iTargetPos;Rs->iCurrentPosCopy=Rs->iCurrentPos;
//!fbq feq not used at this moment
//   Rs->fbq=(Rs->fMaxMovingSpeed-Rs->fBegSpd)/Rs->fBegRamp; //->CSpeed=(fCurrentPositionInCurrentRamp*Rs->fbq) + fBeg(End)Spd;
//   Rs->feq=(Rs->fMaxMovingSpeed-Rs->fEndSpd)/Rs->fEndRamp;
//   Rs->fbqMSE=Rs->fBegRamp/((float)MaxStopElemnts); //->CSpeed=fMaxMovSpeed* ArrStopPow05[(int)(fCurrentPositionInCurrentRamp/fbqMSE)] + fBeg(End)Spd;
//   Rs->feqMSE=Rs->fEndRamp/((float)MaxStopElemnts);

   if (Rs->rState&rSta_UncondREINIT)
   {
     if (Rs->rState&rSta_NoZIntegrREINIT) {goto fcmode;}
     Rs->fIntPart=0.0f;
fcmode:;
     Rs->fIntgTargetPosition=(double)Rs->iCurrentPos;
   }

  Rs->rState&=~(rSta_UncondREINIT|rSta_NoZIntegrREINIT);
  Rs->rState|=rSta_READY;
  Rs->fRegTmp=0.0f;
  goto exii;
recisok:;
//===========================================REGFLOW=================================================
  Rs->fRegTmp=0.0f;
  if (Rs->rState&rSta_FROZEN) goto exii;
//  Rs->rState|=rSta_POSSPDWORK;

//------------------------pos/speed regulator--------------------------------------------------
  Rs->i32cpM=i32sabs(Rs->iCurrentPos-Rs->iCurrentPosCopy);  Rs->i32tpM=i32sabs(Rs->iTargetPos);
// OldDifPos=DifPos;
  if ((Rs->rMode&rMod_ESasCorector))
  {
    Rs->DifPosM=i32sabs(Rs->iCurrentPos-Rs->iTargetPos+Rs->ESLimitPlusMinusM);    //in other case speed will be reduced to 0 too early
  }
  else
  {
    Rs->DifPosM=i32sabs(Rs->iCurrentPos-Rs->iTargetPos);
  }

 if (Rs->rState&rSta_POSWORK)
 {  //POS regulator with self calculated start(=100%) stop ramps and fMaxSpeed from LPC
//!! not needed???   if ((MState.RunFlag&0xfff0)!=0x6610)  goto exiiprw; //moving
       Rs->tmpu16m=0;Rs->ftmpm=Rs->fMaxMovingSpeed;//-355;

       Rs->fCurrentSpeed=Rs->fMaxMovingSpeed; //!TODO NEW

       //=============================================

    if (Rs->rMode&rMod_AutoStopRamp) //iEndRamp==0   //ramp stop is forced to 250 if (R_CFG.FlyingCutSpeed!=0)
    {
       if (Rs->rMode&rMod_Inverter) //invertor model
       {
         if (Rs->DifPosM<Rs->i32RampAutoStopLen) //~60cm =~12000
         {
           Rs->ftmpm=(((float)Rs->DifPosM)*Rs->flt1RampAutoStopLen);
// bezmisleno           if (Rs->iEndSpd!=0)
//           {
             if (Rs->ftmpm<Rs->fEndSpd) {Rs->ftmpm=Rs->fEndSpd;}
//           }
           Rs->tmpu16m=1;
         }
       }
        else
       {
         if (Rs->DifPosM<6400) //~30cm
         {
           Rs->tmpu16m=(Rs->DifPosM>>6);
           if (i16sabs(NewIntgCurrentSpeed)>=Rs->tmpu16m)
           {
             Rs->tmpu16m++;
             if (i16sabs(NewIntgCurrentSpeed)>Rs->iEndSpd) {Rs->ftmpm=(float)Rs->tmpu16m*Rs->f1divSpeedIntgTime;}else{Rs->ftmpm=Rs->fEndSpd+0.5f;}
           }

           if ((i16sabs(NewIntgCurrentSpeed)<13)&&(HackIntPartFlag<100))  //hack for mor smooth end
           {
             HackIntPartFlag++;
             Rs->fCurrentSpeed=0.115f; //5.0f;
             Rs->ftmpm=0.115f; //5.0f;
           }

           Rs->tmpu16m=1;
         }
       }
         goto CheckStartRamp;
    }
 //=======================================================================
       if (Rs->DifPosM < Rs->iEndRamp)        //R_CFG.RampStop1mm >0
       {
         if (Rs->iEndRamp>1)
         {
           Rs->tmpu16m=1; //-MovPreStopRamp;
           Rs->ftmpm=StopKula(Rs,Rs->fCurrentSpeed,((float)Rs->DifPosM)*(Rs->FltBegStopLen100));
           if (Rs->iEndSpd!=0)
           {
             if (Rs->ftmpm<Rs->fEndSpd) {Rs->ftmpm=Rs->fEndSpd;}
           }
           //goto exiiramps;
         }
       }
CheckStartRamp:
//       if (Rs->iBegRamp>0)
//       {
         if (Rs->i32cpM<=Rs->iBegRamp)  //we SUPPOSE Rs->iCurrentPos is ZEROED
         {
           Rs->fCurrentSpeed=Rs->fBegStartLenXfMaxMovSpd*(float)(Rs->i32cpM); // /(float)BegStartLen)->division process done in utils.c;
 //          if (Rs->iBegSpd!=0)
 //          {
             if (Rs->fCurrentSpeed<Rs->fBegSpd) {Rs->fCurrentSpeed=Rs->fBegSpd;} //fCurrentSpeed=FltStartRampLowSpeed;}
 //          }
         }
//       }
//!test debug      MState.USERO=(uint16_t)Rs->iBegRamp; //Rs->fCurrentSpeed*10.0f;
//exiiramps:;
     if (Rs->tmpu16m) { if (Rs->ftmpm<Rs->fCurrentSpeed) Rs->fCurrentSpeed=Rs->ftmpm;}
     if (Rs->fCurrentSpeed>Rs->fMaxMovingSpeed) Rs->fCurrentSpeed=Rs->fMaxMovingSpeed;

     if (Rs->iTargetPos>Rs->iCurrentPos)  Rs->fIntgTargetPosition+=(double)(Rs->fCurrentSpeed);
     if (Rs->iTargetPos<Rs->iCurrentPos)  Rs->fIntgTargetPosition-=(double)(Rs->fCurrentSpeed);
//     fTargetPosition+=fCurrentSpeed;  //very long vs very small adding ??????????????   >10m=1000sm=10000mm=250 000imp?????????????????????
//Rs->fIntgTargetPosition+=Rs->fMaxMovingSpeed;
//exiiprw:;
 }
//---------------------------------------------------------------------------------------------
if (Rs->rState&rSta_SPDWORK)
{

     Rs->fIntPart=((float)Rs->fIntgTargetPosition-(float)Rs->iCurrentPos)*Rs->fIntPartMult; //0.04f; //Rs->intruptfIntPartMult;

     if ( Rs->fIntPart>100.0f ) { Rs->fIntPart=100.0f;} //??????????????fTargetPosition=Rs->OldfTargetPosition;}    //3000 FROM ABG!!!!!!
     if ( Rs->fIntPart<-100.0f ) { Rs->fIntPart=-100.0f;} //????????????fTargetPosition=Rs->OldfTargetPosition;}  //FROM ABG!!!!!!
//     Rs->OldfTargetPosition=(float)Rs->fIntgTargetPosition;   //FROM ABG!!!!!!

// PPart!! fPPart=(fCurrentSpeed-OldCSPD);
     Rs->fDPart=0.0f;  //in fact this is a regulator Force section , not a Dpart :)
     if (Rs->fCurrentSpeed!=Rs->fCurrentSpeedOld)
     {
       Rs->fDPart=(Rs->fCurrentSpeed-Rs->fCurrentSpeedOld);
     }
     Rs->fCurrentSpeedOld=Rs->fCurrentSpeed;

     if (FlagRemovDPart!=0) Rs->fDPart=0.0f;

     Rs->fRegTmp=Rs->fIntPart+Rs->fDPart;
     Rs->tmpi16m=(int16_t)Rs->fRegTmp;

        if (Rs->iCurrentPos>Rs->iTargetPos)
        {
          if ( Rs->tmpi16m<(-100) ) {Rs->tmpi16m=100;goto end_format_out;}
          if ( Rs->tmpi16m<0 ) {Rs->tmpi16m=0-Rs->tmpi16m;goto end_format_out;}
          if ( Rs->tmpi16m>0 )
          {
            if (Rs->tmpi16m>(0-SPC_DebitMinimum)) {Rs->tmpi16m=SPC_DebitMinimum;}else{Rs->tmpi16m=0-Rs->tmpi16m;}
            goto end_format_out;
          } //  FRFBZ=0=1;
          Rs->tmpi16m=0;goto end_format_out;
        }
        else
        {
          if ( Rs->tmpi16m>100) Rs->tmpi16m=100;       //100 in origin!!!!!
          if ( Rs->tmpi16m<SPC_DebitMinimum) Rs->tmpi16m=SPC_DebitMinimum;
        }
end_format_out:;
   Rs->fRegTmp=(float)Rs->tmpi16m;


exii:;
}
}




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define MAXTGR 4
void Scmoving(float fx,RegStructure *Rs)
{
    //fMaxSpeed=fCurrentSpeed;
uint16_t smtmp16u,tmp16u;
int32_t i32tmp;
//static int32_t myOldStartPos=0;
float fsmtmp;
R_CFG=R2_CFG;
    ZeroSpdTimeCnt=0;
        HackIntPartFlag=0; Rs->rMode=0;
        if (R_CFG.RampStop1mm==0) Rs->rMode|=rMod_AutoStopRamp;
        if (R_CFG.FlyingCutSpeed!=0)
        {
            Rs->rMode|=rMod_FlyngCut;
            Rs->rState&=~rSta_POSWORK; //PosRegStatus=PRS_RegStopped; //only pos stop, speed is not altered because in case of flying cut it work
            Rs->iBegSpd=R_CFG.FlyingCutSpeed;Rs->iEndSpd=R_CFG.FlyingCutSpeed;//  R_CFG.StartRampLowSpeed=R_CFG.FlyingCutSpeed;R_CFG.StopRampLowSpeed=R_CFG.FlyingCutSpeed;
        } //instr note!
        else
        {
            Rs->rMode&=~rMod_FlyngCut;
            Rs->rState&=~rSta_POSSPDWORKS; //stop position and speed regulators
        }

        if (R_CFG.BegStartRamp==0) R_CFG.BegStartRamp=1;
        if (R_CFG.RelaxTimeForDebit==0) R_CFG.RelaxTimeForDebit=1;
        if (R_CFG.RampStop1mm==0)  R_CFG.RampStop1mm=1;
        if (R_CFG.SpeedIntgTime<2) R_CFG.SpeedIntgTime=2;
        if (R_CFG.StartRampLowSpeed==0) R_CFG.StartRampLowSpeed=1;

       if ((R_CFG.Enc1RotPulsesX4<10)||(R_CFG.Enc1RotPulsesX4>32000)) R_CFG.Enc1RotPulsesX4=2500;
       Rs->fKmf=((float)R_CFG.Enc1RotPulsesX4)*400.0f/(((float)R_CFG.PullingRollDia001mm*3.1415926f)); //2500*400/(15000*pi)==21.2206
       if (R_CFG.IntPartMult16U==0) R_CFG.IntPartMult16U=100;
       Rs->fIntPartMult=((float)R_CFG.IntPartMult16U)/10000.0f; //100/10000=0.01 def

       CntToPostSpindleTime=R_CFG.PostSpindleTime;
       CFGLowSpeedLenImpulses=(int32_t)((float)R_CFG.LowSpeedLen1mm*Rs->fKmf);
       Rs->LimitPlus=(int32_t)(((float)R_CFG.LimitP_In01mm*Rs->fKmf)/10.0f);
       Rs->LimitMinus=(int32_t)(((float)R_CFG.LimitM_In01mm*Rs->fKmf)/10.0f);
       Rs->MaxBackMovLim=(int32_t)(((float)R_CFG.MaxBackwardMoveLimit01mm*Rs->fKmf)/10.0f);
       Rs->ESLimitPlusMinusM=(int32_t)(((float)R_CFG.ESLimitPM_In01mm*Rs->fKmf)/10.0f);

    Rs->f1divSpeedIntgTime=1.0f/(float)R_CFG.SpeedIntgTime;
    Rs->fBegSpd=((float)R_CFG.StartRampLowSpeed)/(float)R_CFG.SpeedIntgTime; //FltStartRampLowSpeed
    if (Rs->fBegSpd<0.01) Rs->fBegSpd=0.01;
    Rs->iBegSpd=R_CFG.StartRampLowSpeed;
    Rs->fEndSpd=((float)R_CFG.StopRampLowSpeed)/(float)R_CFG.SpeedIntgTime;  //FltStopRampLowSpeed
    Rs->iEndSpd=R_CFG.StopRampLowSpeed;

    Rs->iBegRamp=(int32_t)(Rs->fKmf*((float)R_CFG.BegStartRamp));
    Rs->iEndRamp=(int32_t)(Rs->fKmf*(float)R_CFG.RampStop1mm);   //kmf~=21
    Rs->FltBegStopLen100=(100.0f/(Rs->fKmf*((float)R_CFG.RampStop1mm)))*(10.0f/(float)R_CFG.SpeedIntgTime);

    smtmp16u=OlCo.Analog1In;
    if (smtmp16u<1) smtmp16u=1;
    Rs->fMaxMovingSpeed=((float)smtmp16u)/(float)R_CFG.SpeedIntgTime; // x2 ?????????????????
    Rs->fCurrentSpeed=Rs->fMaxMovingSpeed;

    if (R_CFG.SystB2&0x02) Rs->rMode|=rMod_Inverter;//inverter model
    NPNAfromSSPI=0;
    tmp16u=R_CFG.SpindlePullRatio;if (tmp16u>=10000){tmp16u=tmp16u-10000;}
    InverterSpeed=(uint16_t)(((float)smtmp16u)*(((float)tmp16u)/100.0f));
    fsmtmp=1.0f/(Rs->fKmf*((float)R_CFG.BegStartRamp));
    if (FlagFirstPiece==1)
    {
     	WrkRackCounter=0;
        if ((R_CFG.SystB2&0x40))
		{
		  Rs->iBegRamp=(int32_t)(16.0f*Rs->fKmf*((float)R_CFG.BegStartRamp));  //32.0
          fsmtmp=1.0f/(16.0f*Rs->fKmf*((float)R_CFG.BegStartRamp)); //32.0
		  Rs->fBegSpd=Rs->fBegSpd/2.0f; //3.0
		  InverterSpeed*=3;
		}
	}
    if (InverterSpeed>100) InverterSpeed=100;

    Rs->fBegStartLenXfMaxMovSpd=Rs->fMaxMovingSpeed*fsmtmp;

    Rs->i32RampAutoStopLen=(int32_t)( ((float)R_CFG.AutoStopRamp*((float)smtmp16u)) / ((float)R_CFG.SpeedIntgTime) );
    if (Rs->i32RampAutoStopLen==0) Rs->i32RampAutoStopLen++;
    Rs->flt1RampAutoStopLen=Rs->fMaxMovingSpeed/(float)Rs->i32RampAutoStopLen;

    Rs->FlagGetLastMovCurrentPosition=0;

    Rs->OldStartMovTargetPosition=(int32_t)(fx*Rs->fKmf);
        if (KeyAutoCorOn)
          { if ((R_CFG.SystemBits&0x01)==0)
            {//Rs->LastMovCurrentPosition=Rs->OldStartMovTargetPosition;Rs->CalculatedMovCorection=0;
             R_CFG.SystemBits|=0x01;R2_CFG.SystemBits|=0x01;}
          }
        else {R_CFG.SystemBits&=~0x01;R2_CFG.SystemBits&=~0x01;}

    i32tmp=Rs->OldStartMovTargetPosition-Rs->LastMovCurrentPosition;
    float mkmf;
     uint32_t limsum=i32sabs(R_CFG.LimitM_In01mm)+i32sabs(R_CFG.LimitP_In01mm);
    if (limsum<20) mkmf=10.0f;else mkmf=((float)limsum)*0.25f;
    if (i32sabs(i32tmp)>((int32_t)(mkmf*Rs->fKmf))) i32tmp=(i32tmp*8)/10;else i32tmp=i32tmp/2;
    if (R_CFG.SystemBits&0x01)
    {
      Rs->CalculatedMovCorection+=i32tmp; //+ or = ????????????????
    }
    else
    {
      Rs->CalculatedMovCorection=i32tmp; //!!!!without'+', otherwise endless
    }

    Rs->iTargetPos=Rs->OldStartMovTargetPosition;  //(int32_t)(fx*R_CFG.Enc1Coef);
    if (FlagFirstPiece==1)
    {
     	WrkRackCounter=0;Rs->CalculatedMovCorection=0;
        if ((R_CFG.ESLimitPM_In01mm!=0)) Rs->CalculatedMovCorection=R_CFG.ESLimitPM_In01mm;
    }

    if (Rs->iTargetPos>0)
	{
		if (Rs->CalculatedMovCorection>0){Rs->CalculatedMovCorection=0;}
	}

    if (i32sabs(Rs->CalculatedMovCorection)>i32sabs(Rs->OldStartMovTargetPosition))
    {
      if (Rs->OldStartMovTargetPosition>0) Rs->CalculatedMovCorection=45-Rs->OldStartMovTargetPosition;
      if (Rs->OldStartMovTargetPosition<0) Rs->CalculatedMovCorection=45+Rs->OldStartMovTargetPosition;
    }

    if (R_CFG.SystemBits&0x01)
    {
        Rs->iTargetPos=Rs->iTargetPos+Rs->CalculatedMovCorection; //+MState.CurrentPosition; is 0!!!!!
        if ((R_CFG.SystemBits&0x10)==0)
             {R2_CFG.ESLimitPM_In01mm=Rs->CalculatedMovCorection;R_CFG.ESLimitPM_In01mm=Rs->CalculatedMovCorection;}
    }

    Rs->iTargetPosCopy=Rs->iTargetPos;
    Rs->iCurrentPos=0;Rs->iCurrentPosCopy=Rs->iCurrentPos;


    if (R_CFG.SystemBits&0x10) Rs->rMode|=rMod_ESasCorector;

    SpindelStopTargetPos=Rs->iTargetPos-((((float)R_CFG.SubSpindle01mm)/10.0)*Rs->fKmf);
    MState.TargetPosition=Rs->iTargetPos;
    if (Rs->iTargetPos==0) goto exii;

    CFGBegin_Brake_Len1mm=Rs->iTargetPos-(int32_t)((float)R_CFG.Begin_Brake_Len1mm*Rs->fKmf);  //braking start point!!!!!!!
    if (R_CFG.Duration_Brake_TimeOrLen1mm<3000)
    {
      Duration_BrakeTime=0; //parametr assumed as len
      CFGDuration_Brake_TimeOrLen1mm=Rs->iTargetPos-(int32_t)((float)R_CFG.Duration_Brake_TimeOrLen1mm*Rs->fKmf);
    }
    else
    {
      CFGDuration_Brake_TimeOrLen1mm=0;
      Duration_BrakeTime=R_CFG.Duration_Brake_TimeOrLen1mm;//assumed as time
    }
    Rs->rState=rSta_UncondREINIT;
    if ((FlagFirstPiece!=1)&&(R_CFG.FlyingCutSpeed!=0)) {Rs->rState|=rSta_NoZIntegrREINIT;}
    CalcRegulator(Rs);

    //! ? if (Rs->rState&rSta_READY)
    //!! ?? Rs->rState=rSta_POSSPDWORKS;
    if (FlagFirstPiece==1) FlagFirstPiece++;

    if ((R_CFG.SystemBits&0x40)==0x00) {OnNPNCutOff;} else {OffNPNCutOff;}//up pressure normaly knife is always driven back
    TagLimitMaxRetrCnt=MAXTGR;
    if (R_CFG.SystB2&0x80) {TS_B_ON;FanTHY3OFF;}
    if (Rs->iTargetPos>0)
    {
      OffNPNBrakeMotor;
      RunFlagCopy=RM_MovingP;
      if ((R_CFG.SystemBits&0x10)==0x10) RunFlagCopy=RM_MovingPtoES;
//     MState.RunFlag=RM_MovingP;
      MState.RunFlag=RM_xStartMovPRS;

    }
     else
    {
//     goto exii;  //patch!!!!!!!!! `-` moves are ignored
      RunFlagCopy=RM_MovingM;
      MState.RunFlag=RM_xStartMovPRS;

//      MState.RunFlag=RM_MovingM;
    }

//    PosRegStatus=PRS_RegStopped; //stop reg in any case because new parameters will be feed

exii:;
}

//===========================================================================================//



/*
void SendQuestToEncX(void)
{
      if (CurrentEncAddr==0) goto exii;
      if (FlagSomeDataIsReceived!=0) //if (U2STAbits.TRMT==1)
      {
        OnRTS_U2;Dis_RxINT_U2;
        U2TXREG=CurrentEncAddr;
        U2TXREG=(CurrentEncAddr^0xff); //check sum
        IFS1bits.U2TXIF =0;IEC1bits.U2TXIE =1; //Enable TX2 int on end to deselect RTS
        FlagSomeDataIsReceived=0;EncCommRetrCnt=20;
      }
//      else
//      {
//        if (EncCommRetrCnt<14) //two bytes must be already send /on 19200/
//        {
//          OffRTS_U2;
//          En_RxINT_U2;
 //       }
 //     }
exii:;
}

void __attribute__((__interrupt__,no_auto_psv)) _U2RXInterrupt(void)
{
uint8_t bTmp,wcrc;
union  md168
{
  int16_t u16s;
  uint8_t  EncData[2];
} da;

  bTmp=U2RXREG;
  rxdata[0]=bTmp;
  rxdata[1]=U2RXREG;
  rxdata[2]=U2RXREG;
  rxdata[3]=U2RXREG;
  IFS1bits.U2RXIF =0; //clear rx interrupt flag
  wcrc=0xff;
  if (U2STAbits.OERR) { U2STAbits.OERR=0; rxdata[4]++;}
  if ( bTmp==CurrentEncAddr)
  {
    wcrc^=bTmp;
    da.EncData[0]=rxdata[1];
    da.EncData[1]=rxdata[2];
    wcrc^=da.EncData[0];
    wcrc^=da.EncData[1];
    wcrc^=rxdata[3];
    if ( wcrc==0)
    {
      if (bTmp==EncRotAddr)
       {
         PosCntRot=da.u16s;
         NewRotDataPresent=1;
       }
      if (bTmp==EncMovAddr)
       {
         PosCntMov=da.u16s;
         NewMovDataPresent=1;
       }
      FlagSomeDataIsReceived=1;
    }
  }
  else
  {
    wcrc=U2RXREG;wcrc=U2RXREG;wcrc=U2RXREG;wcrc=U2RXREG;wcrc=U2RXREG;U2STAbits.OERR=0;
  }

  Dis_RxINT_U2; //FlagSomeDataIsReceived=1;
}

void __attribute__((__interrupt__,no_auto_psv)) _U2TXInterrupt(void)
{
  uint8_t btmp;
  IFS1bits.U2TXIF =0; //clear tx interrupt flag

  Dis_TxINT_U2;   //disable transmit interrupt
pakk:
  if (U2STAbits.TRMT==0) goto pakk;
  OffRTS_U2;
  btmp=U2RXREG;btmp=U2RXREG;btmp=U2RXREG;btmp=U2RXREG;btmp=U2RXREG;U2STAbits.OERR=0;
//  U2STAbits.URXISEL =3; //11 or 4 data chars //i.e. full message
  En_RxINT_U2;
      // wErrorIntStatus=UART1TX_VECTOR*0x100;wSystemErrors|=ErrorIrq;

}
*/
//------------------------pos/speed regulator interrupt vars-------------------------------------
//!2016-10float fDPart;
//!2016-10float fIntPart;
//static float fPPart;
//!2016-10int32_t DifPos; //,OldDifPos;
//int32_t MovDifPos;
//!2016-10 float OldfTargetPosition,fTargetPosition;
//!2016-10float intruptfIntPartMult;
int16_t  CRTmp16s,invsp16s,itmp16s;

int32_t ZPosTarget=0,ZPosIntErr=0;
#define ZPGAIN 12
#define ZPMAXLIMIT 6    //100 orig
#define ZPMAXENDLIMIT 6
void ZeroSpeedReg(void)
{
int32_t tmp32s,otmp32s,stmp32s;

     stmp32s=(ZPosTarget-MState.CurrentPosition);
     tmp32s=stmp32s;if (tmp32s<0) tmp32s=0-tmp32s; //!ABS
     otmp32s=tmp32s;
     tmp32s=tmp32s>>7;
     if (tmp32s==0)
     {
       if (otmp32s<50) {OffIGBTBack;OffIGBTPull;goto ZSend_format_out;}
     }
     if ( stmp32s<0 ) {OnIGBTBack;OffIGBTPull;goto ZSend_format_out;}
     if ( stmp32s>0 ) {OffIGBTBack;OnIGBTPull;goto ZSend_format_out;}
     OffIGBTBack;OffIGBTPull;
ZSend_format_out:;
   if (tmp32s>ZPMAXENDLIMIT) tmp32s=ZPMAXENDLIMIT;  //!!!!!! again max speed in this mode
   AOSPPullSpeed=tmp32s;
}
uint16_t TRNAS_KnifeNOTinPos=0,TRNAS_KnifeInPos=0,SharedInvertorDelay=0;
void CutProc(void)
{
       if (R_CFG.FlyingCutSpeed==0){OffIGBTPull;OffIGBTBack;}

       if ((R_CFG.SystB2&0x80)&&(SharedInvertorDelay==0))  ////is 1 invertor roll+knife?
       {
            if ((Get_InACDC2==0)) {MState.ErrorFlags|=EF_CanNotStopRolls;MState.RunFlag=RM_Error;goto nochkcut;}
          TS_B_OFF;FanTHY3ON; //off rolls contactor //on knife contactor
          SharedInvertorDelay=2000;TRNAS_KnifeInPos=0; //wait contactor to on
          goto nochkcut;
       }

       if (SharedInvertorDelay>1) {RM_Delay++;SharedInvertorDelay--;goto nochkcut;}


       if (R_CFG.SystB2&0x80) {AOSPPullSpeed=150;}else{AOSPPullSpeed=0;}
//---------------------------
       if (RM_Delay==0)
       {
//No Transition detected! ???? CONTINUE???????
					 MState.ErrorFlags|= EF_KnifeBlockMayBe ;MState.RunFlag=RM_Error;
       }

         if (RM_Delay>(R_CFG.TimeForCutting-(R_CFG.TimeForCutting>>2))) {OnNPNCutOn;goto nochkcut;} //force moving (enable start)

         if (HiCutFBCnt>CutSwitchTimeCut) //(TRNAS_KnifeInPos)
         {
            TRNAS_KnifeInPos=1;OffNPNCutOff;OffNPNCutOn;
         }

         if (TRNAS_KnifeInPos)
         {
            if ((R_CFG.SystB2&0x80))  //is 1 invertor roll+knife?
            {
              if ((Get_InACDC2==0)) {RM_Delay++;AOSPPullSpeed=0;goto nochkcut;} //Knife not yet STOP
            }

            TRNAS_KnifeInPos=0;SharedInvertorDelay=0;if ((R_CFG.SystB2&0x80)){TS_B_ON;FanTHY3OFF;} //on rolls contactor //off knife contactor
            if (MState.RunFlag==RM_Manual){RM_Delay=0;}else
            {
               MState.RunFlag=RM_CutOff;   //knife is in Z pos
               RM_Delay=R_CFG.TimeForCuttOff; //some delay for hi-speed, long stop time cutters
            }
         }



nochkcut:;


    //next lines may produce false errors if knife is not on right positions
    //so comment it
//       if (RM_Delay==(R_CFG.TimeForCutting-3000))  // i.e. 0.3s from begin cutting
//       {
//         if ((MState.ErrorFlags&WF_CutterNotInZPos)==0)
//         {
//           if ((R_CFG.ReservSystBits&EF_KnifeNotMoving)==0)
//           { MState.ErrorFlags|=EF_KnifeNotMoving ;MState.RunFlag=RM_Error;goto  not_enought_cut_time;}
//         }
//       }  //knife STILL is in Z position! No cutting???


//       if (RM_Delay==0)
//       {
//           if ((R_CFG.ReservSystBits& EF_KnifeBlockMayBe)==0)
//           {MState.ErrorFlags|= EF_KnifeBlockMayBe ;MState.RunFlag=RM_Error;}
//       }  //No Transition detected! knife Not reached Z position? Blocking???
//       if (TRNAS_KnifeInPos)
//       {
//           TRNAS_KnifeInPos=0;OffNPNCutOff;OffNPNCutOn;RM_Delay=0;   //no more delay is needed
//       }
}
void SpindedltoCutSpeedOROFF(void)
{
  if ( ((R_CFG.SystemBits&0x80)==0x80)&&(R_CFG.CutPeriodSpindelSpeed==0) )
    {LoTR6_OFF;OffThySpindelENBL;}
  else
    {HiTR6_ON;NPNAOutDuty=R_CFG.CutPeriodSpindelSpeed;NPNAfromSSPI=R_CFG.CutPeriodSpindelSpeed;}
}
void MovingLR(RegStructure *Rs)
{
   if (MState.RunFlag==RM_MovingP) //L
   {
     FlagRegForceBellowZero=0;

     if ( R_CFG.SystemBits&0x04)
     {
       invsp16s=NewIntgCurrentSpeed; // *tstSPCmult; //Analog1In; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  		 if (invsp16s<0) invsp16s=0-invsp16s;
       NPNAOutDuty=invsp16s;if (NPNAOutDuty<R_CFG.CutPeriodSpindelSpeed) NPNAOutDuty=R_CFG.CutPeriodSpindelSpeed;
     }
      else
     {
       if (TagLimitMaxRetrCnt!=MAXTGR){invsp16s=R_CFG.CutPeriodSpindelSpeed;}else{invsp16s=InverterSpeed;}
       NPNAOutDuty=invsp16s;
     }
     if ((R_CFG.SystemBits&0x20)==0x20)   //ES as corector
     {
       if (MState.CurrentPosition<=(Rs->iTargetPos+Rs->ESLimitPlusMinusM))       // ??? C ES??     R_CFG.Limit_In01mm
       {
          OnIGBTPull;if ((R_CFG.SystemBits&0x40)==0x00) {OnNPNCutOff;}

          if (MState.CurrentPosition<(Rs->iTargetPos-Rs->ESLimitPlusMinusM))
          {
            if (LowEndSwitchCnt>ESwitchTimeCut)   //same as (MState.CurrentPosition>MState.TargetPosition)
            {
              if ((R_CFG.ReservSystBits&EF_EndSwitchTooEarly)==0x0000) {MState.ErrorFlags|=EF_EndSwitchTooEarly;MState.RunFlag=RM_Error; goto endofmovp;}
            }
          }

          if (MState.CurrentPosition>=(Rs->iTargetPos-Rs->ESLimitPlusMinusM))
          {
            if (LowEndSwitchCnt>ESwitchTimeCut)   //same as (MState.CurrentPosition>MState.TargetPosition)
            {
        	SpindedltoCutSpeedOROFF();
        	MState.RunFlag=RM_MovStoppingFullP;RM_Delay=100;
        	goto endofmovp;
            }
          }



          if (SpindelStopTargetPos>MState.CurrentPosition)
          {
            if (CntToPostSpindleTime!=0) {CntToPostSpindleTime--;} else { HiTR6_ON;OnThySpindelENBL;}  //orig OnThySpindelENBL;
          }
          else
          {
              SpindedltoCutSpeedOROFF();
          }

       }
       else
       {
         if ((R_CFG.ReservSystBits&EF_EndSwitchNoEnd)==0x0000) {MState.ErrorFlags|=EF_EndSwitchNoEnd;MState.RunFlag=RM_Error;goto endofmovp;}
       }
       goto endofmovp;
     }  //end if ((R_CFG.SystemBits&0x20)==0x20)   //ES as corector


     if (Rs->iTargetPos>MState.CurrentPosition)
     {
        OffIGBTBack;OnIGBTPull;if ((R_CFG.SystemBits&0x40)==0x00) {OnNPNCutOff;}
//---
        if (R_CFG.Begin_Brake_Len1mm)
        {
          if (CFGBegin_Brake_Len1mm>MState.CurrentPosition)
          {
            if (CFGDuration_Brake_TimeOrLen1mm==0)
            {
               if (Duration_BrakeTime==0) {OffNPNBrakeMotor;} else {Duration_BrakeTime--;OnNPNBrakeMotor;}
            }
            else
            {
               if (CFGDuration_Brake_TimeOrLen1mm>MState.CurrentPosition) {OffNPNBrakeMotor;}else{OnNPNBrakeMotor;}
            }
          }
        }
//---
        if (SpindelStopTargetPos>MState.CurrentPosition)
         {
           if (CntToPostSpindleTime!=0) {CntToPostSpindleTime--;} else { HiTR6_ON;OnThySpindelENBL;}  //orig OnThySpindelENBL;
         }
         else
         {
             SpindedltoCutSpeedOROFF();
         }
     }
     else
     {
	 SpindedltoCutSpeedOROFF();
	 MState.RunFlag=RM_MovStoppingFullP;RM_Delay=R_CFG.PreWaitFullStop;if ((R_CFG.SystB2&0x08)==0) RM_Delay=2;
     }

   }
endofmovp:;
//-------------------------------------------
   if (MState.RunFlag==RM_MovingM) //R
   {
     FlagRegForceBellowZero=0;

     if ( R_CFG.SystemBits&0x04)
     {
       invsp16s=NewIntgCurrentSpeed; // *tstSPCmult; //Analog1In; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  		 if (invsp16s<0) invsp16s=0-invsp16s;
       NPNAOutDuty=invsp16s; if (NPNAOutDuty<R_CFG.CutPeriodSpindelSpeed) NPNAOutDuty=R_CFG.CutPeriodSpindelSpeed;
     }
      else
     {
       if (TagLimitMaxRetrCnt!=MAXTGR){invsp16s=R_CFG.CutPeriodSpindelSpeed;}else{invsp16s=InverterSpeed;}
       NPNAOutDuty=invsp16s;
     }

     if (Rs->iTargetPos<MState.CurrentPosition) { OnIGBTBack;OffIGBTPull;} else {MState.RunFlag=RM_MovStoppingFullM;RM_Delay=R_CFG.PreWaitFullStop;if ((R_CFG.SystB2&0x08)==0) RM_Delay=2;} //R_CFG.MovRevDelay;}

   }

//-------------------------------------------
   if (MState.RunFlag==RM_MovingPtoES) //L
   {

     FlagRegForceBellowZero=0;
     if ( R_CFG.SystemBits&0x04)
     {
       invsp16s=NewIntgCurrentSpeed; // *tstSPCmult; //Analog1In; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  		 if (invsp16s<0) invsp16s=0-invsp16s;
       NPNAOutDuty=invsp16s; if (NPNAOutDuty<R_CFG.CutPeriodSpindelSpeed) NPNAOutDuty=R_CFG.CutPeriodSpindelSpeed;
     }
      else
     {
       if (TagLimitMaxRetrCnt!=MAXTGR){invsp16s=R_CFG.CutPeriodSpindelSpeed;}else{invsp16s=InverterSpeed;}
       NPNAOutDuty=invsp16s;
     }


     if (HiEndSwitchCnt>ESwitchTimeCut)   //same as (MState.TargetPosition>MState.CurrentPosition)
     {
//---
        if (R_CFG.Begin_Brake_Len1mm)
        {
          if (CFGBegin_Brake_Len1mm>MState.CurrentPosition)
          {
            if (CFGDuration_Brake_TimeOrLen1mm==0)
            {
               if (Duration_BrakeTime==0) {OffNPNBrakeMotor;} else {Duration_BrakeTime--;OnNPNBrakeMotor;}
            }
            else
            {
               if (CFGDuration_Brake_TimeOrLen1mm>MState.CurrentPosition) {OffNPNBrakeMotor;}else{OnNPNBrakeMotor;}
            }
          }
        }
//---
        OnIGBTPull;if ((R_CFG.SystemBits&0x40)==0x00) {OnNPNCutOff;}
        if (CntToPostSpindleTime!=0) {CntToPostSpindleTime--;} else { HiTR6_ON;}
        if (Rs->rState&rSta_SPDWORK) Rs->rState&=~rSta_POSWORK;
        Rs->fIntgTargetPosition+=(double)Rs->fMaxMovingSpeed;  //not fcursped because in begin pos reg and ramps are active
     }
     else
     {
        MState.RunFlag=RM_MovStoppingFullPtoES;
        Rs->fIntgTargetPosition=(double)MState.CurrentPosition;    //zeroing integral part of speed regulator
     }

   }
//--------------------------------------------------
   if (MState.RunFlag==RM_MovStoppingFullPtoES)
   {
     if ( (LowEndSwitchCnt>ESwitchTimeCut)&&((R_CFG.SystemBits&0x30)==0x30) )
     {
         OnIGBTPull;if ((R_CFG.SystemBits&0x40)==0x00) {OnNPNCutOff;}
         HiTR6_ON;
         if (R_CFG.StopRampLowSpeed==0) Rs->fIntgTargetPosition+=0.2;  else Rs->fIntgTargetPosition+=Rs->fEndSpd;
     }
     else
     {
	 SpindedltoCutSpeedOROFF();
        if (R_CFG.FlyingCutSpeed==0)
        {
            OffIGBTPull;OffIGBTBack; //OffIGBTPull;
            FlagRegForceBellowZero=1;//CopyOfSP_Current=MState.SP_Current;
            Rs->rState&=~rSta_POSSPDWORKS; //SpdRegStatus=SPD_RegStopped; PosRegStatus=PRS_RegStopped;
        }
        else
        {
          Rs->rState|=(rSta_UncondREINIT|rSta_NoZIntegrREINIT);CalcRegulator(Rs);Rs->rState&=~rSta_POSWORK;// SpdRegStatus=SPD_RegREStarted;PosRegStatus=PRS_RegStopped;
          if (R_CFG.SystB2&0x01)
          {
           MState.SP_Current=R_CFG.FlyingCutSpeed;
           Rs->rState&=~rSta_POSSPDWORKS; //SpdRegStatus=SPD_RegStopped; PosRegStatus=PRS_RegStopped;
          }

        }

//!!??        Rs->iBegRamp; //BegStartLen=0;
        RM_Delay=2;//MState.RunFlag=RM_PreWait2;
        MState.RunFlag=RM_CutOn;
        if (R_CAL.SPPieces==0) {LoTR6_OFF;if (R_CFG.CutPeriodSpindelSpeed==0){OffThySpindelENBL;}}      //KRYPKAAA

     }
   }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   if (MState.RunFlag==RM_MovStoppingFullM)
   {
     OffIGBTBack;OffIGBTPull;
     // Off Speed and pos regulators & force current regulator bellow zero
     Rs->rState&=~rSta_POSSPDWORKS; //SpdRegStatus=SPD_RegStopped; PosRegStatus=PRS_RegStopped;
     FlagRegForceBellowZero=1;            //now 4 times
//-------------------------------------------------------------------
         RM_Delay--;
//-----------------autocheck full stop--------------------
        if ((R_CFG.SystB2&0x08)==0)  //is autcheck disabled
        {
          if (NewIntgCurrentSpeed) {RM_Delay++;RM_Delay++;if (RM_Delay>42768) {if ((R_CFG.ReservSystBits&EF_CanNotStopRolls)==0x0000) {MState.ErrorFlags|=EF_CanNotStopRolls;MState.RunFlag=RM_Error;goto exiimovcor;}}} else {RM_Delay=0;}
        }
        if ((R_CFG.SystB2&0x80))  ////is 1 invertor roll+knife?
        {
          if ((Get_InACDC2==0)) {RM_Delay++;RM_Delay++;if (RM_Delay>42768) {if ((R_CFG.ReservSystBits&EF_CanNotStopRolls)==0x0000) {MState.ErrorFlags|=EF_CanNotStopRolls;MState.RunFlag=RM_Error;goto exiimovcor;}}} else {RM_Delay=RM_Delay>>1;}
        }
//--------------------------------------------------------
     if (RM_Delay==0)
     {
       MState.LastPosErr=(int16_t)(MState.CurrentPosition-Rs->OldStartMovTargetPosition);
       MState.ErrorFlags&=~WF_LimitIsSkipped;

       if (Rs->FlagGetLastMovCurrentPosition==0) {Rs->LastMovCurrentPosition=MState.CurrentPosition;Rs->FlagGetLastMovCurrentPosition=1;}

       if (Rs->LimitPlus!=0)
       {
         if ((MState.CurrentPosition-Rs->OldStartMovTargetPosition)>Rs->LimitPlus)
         {
            if (TagLimitMaxRetrCnt)
            {
              TagLimitMaxRetrCnt--;
              if ((MState.CurrentPosition-Rs->OldStartMovTargetPosition)>Rs->MaxBackMovLim) goto skip_m_lim;
//              fCurrentSpeed=3.0;fMaxMovingSpeed=fCurrentSpeed;  //4.0?????????????????????????
              MState.RunFlag=RM_StartMovRotPRS_W2007delay; Rs->iTargetPos=Rs->OldStartMovTargetPosition;
              RunFlagCopy=RM_MovingM;goto exiimovcor;
            }
            else
            {
skip_m_lim:;
              MState.ErrorFlags|=WF_LimitIsSkipped;
            }
         }
       }
       if (Rs->LimitMinus!=0)
       {
         if ((MState.CurrentPosition-Rs->OldStartMovTargetPosition)<Rs->LimitMinus)
         {
            if (TagLimitMaxRetrCnt)
            {
              TagLimitMaxRetrCnt--;
//              fCurrentSpeed=3.0;fMaxMovingSpeed=fCurrentSpeed;  //4.0?????????????????????????
              MState.RunFlag=RM_StartMovRotPRS_W2007delay; Rs->iTargetPos=Rs->OldStartMovTargetPosition;
              RunFlagCopy=RM_MovingP;goto exiimovcor;
            }
            else
            {
              MState.ErrorFlags|=WF_LimitIsSkipped;
            }
         }
       }
       MState.RunFlag=RM_CutOn;
       if (R_CAL.SPPieces==0) {LoTR6_OFF;if (R_CFG.CutPeriodSpindelSpeed==0){OffThySpindelENBL;}}      //KRYPKAAA
     }
   }
//exiimovcor: ;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   if (MState.RunFlag==RM_MovStoppingFullP)
   {
//?? ALTERNATIVE NO TO OFF MOVP AND SPEEDPOS REGS????
        if (R_CFG.FlyingCutSpeed==0)
        {
           OffIGBTPull;OffIGBTBack; ////OffIGBTPull; only Ain1Bias hold inverter //LoTR6_OFF; //OnSliv; //fast stop????
         // Off Speed and pos regulators
            Rs->rState&=~rSta_POSSPDWORKS; //SpdRegStatus=SPD_RegStopped; PosRegStatus=PRS_RegStopped;
            FlagRegForceBellowZero=1;//CopyOfSP_Current=MState.SP_Current;

        }
        else
        {
         Rs->rState|=(rSta_UncondREINIT|rSta_NoZIntegrREINIT);CalcRegulator(Rs);Rs->rState&=~rSta_POSWORK;// SpdRegStatus=SPD_RegREStarted;PosRegStatus=PRS_RegStopped;
          if (R_CFG.SystB2&0x01)
          {
            MState.SP_Current=R_CFG.FlyingCutSpeed;
            Rs->rState&=~rSta_POSSPDWORKS; //SpdRegStatus=SPD_RegStopped; PosRegStatus=PRS_RegStopped;
          }
        }


//!!???      Rs->iBegRamp; //BegStartLen=0;

//-------------------------------------------------------------------
         RM_Delay--;
//-----------------autocheck full stop--------------------
        if ((R_CFG.SystB2&0x08)==0)  //is autcheck disabled
        {
          if (NewIntgCurrentSpeed) {RM_Delay++;RM_Delay++;if (RM_Delay>42768){if ((R_CFG.ReservSystBits&EF_CanNotStopRolls)==0x0000) {MState.ErrorFlags|=EF_CanNotStopRolls;MState.RunFlag=RM_Error;goto exiimovcor;}}} else {RM_Delay=0;}
        }
        if ((R_CFG.SystB2&0x80))  //is 1 invertor roll+knife?
        {
          if ((Get_InACDC2==0)) {RM_Delay++;RM_Delay++;if (RM_Delay>42768){if ((R_CFG.ReservSystBits&EF_CanNotStopRolls)==0x0000) {MState.ErrorFlags|=EF_CanNotStopRolls;MState.RunFlag=RM_Error;goto exiimovcor;}}} else {RM_Delay=RM_Delay>>1;}
        }
//--------------------------------------------------------
     if (RM_Delay==0)
     {

       MState.LastPosErr=(int16_t)(MState.CurrentPosition-Rs->OldStartMovTargetPosition);
       MState.ErrorFlags&=~WF_LimitIsSkipped;

       if (Rs->FlagGetLastMovCurrentPosition==0) {Rs->LastMovCurrentPosition=MState.CurrentPosition;Rs->FlagGetLastMovCurrentPosition=1;}

       if (Rs->LimitPlus!=0)
       {
         if ((MState.CurrentPosition-Rs->OldStartMovTargetPosition)>Rs->LimitPlus)
         {
            if (TagLimitMaxRetrCnt)
            {
              TagLimitMaxRetrCnt--;
              if ((MState.CurrentPosition-Rs->OldStartMovTargetPosition)>Rs->MaxBackMovLim) goto skip_p_lim;
//              fCurrentSpeed=3.0;fMaxMovingSpeed=fCurrentSpeed;  //4.0?????????????????????????
              MState.RunFlag=RM_StartMovRotPRS_W2007delay; Rs->iTargetPos=Rs->OldStartMovTargetPosition;
              RunFlagCopy=RM_MovingM;goto exiimovcor;
            }
            else
            {
skip_p_lim:;
              MState.ErrorFlags|=WF_LimitIsSkipped;
            }
         }
       }
       if (Rs->LimitMinus!=0)
       {
         if ((MState.CurrentPosition-Rs->OldStartMovTargetPosition)<Rs->LimitMinus)
         {
            if (TagLimitMaxRetrCnt)
            {
              TagLimitMaxRetrCnt--;
//              fCurrentSpeed=3.0;fMaxMovingSpeed=fCurrentSpeed;  //4.0?????????????????????????
              MState.RunFlag=RM_StartMovRotPRS_W2007delay; Rs->iTargetPos=Rs->OldStartMovTargetPosition;
              RunFlagCopy=RM_MovingP;goto exiimovcor;
            }
            else
            {
              MState.ErrorFlags|=WF_LimitIsSkipped;
            }
         }
       }
       MState.RunFlag=RM_CutOn;
       if (R_CAL.SPPieces==0) {LoTR6_OFF;if (R_CFG.CutPeriodSpindelSpeed==0){OffThySpindelENBL;}}      //KRYPKAAA
      }
   }
exiimovcor: ;
}
//----------------------------------------------------------------------------------------------------
void Anstat100uS(void)
{
//float tmp32f;
//int32_t tmp32s;
static uint16_t itmp16u;
//static uint32_t itmp32u;

/* Interrupt Service Routine code goes here         */
//IFS0bits.T1IF = 0; /* clear interrupt flag */
OneSecondCounter++;
//IndicRefreshTime++;
//BlinkTime++;TETime++;
IntgSpdCnt++;
//go to nochkmods;
//--------------check QEI-------------------------------------------------------------------
		itmp16u=(uint16_t)TIMER_CNT(TIMER1);
       //itmp16u=(uint16_t)TIM2->CNT;
       CurrentSpeed=(int16_t)(itmp16u-OldPosCnt);
       if (R_CFG.SystemBits&0x8000) CurrentSpeed=0-CurrentSpeed;    //NEGATE if need
       IntgCurrentSpeed+=CurrentSpeed;
       MState.CurrentPosition+=CurrentSpeed;
       OldPosCnt=itmp16u;

      if (IntgSpdCnt>(R_CFG.SpeedIntgTime-1)) {IntgSpdCnt=0;OldIntgCurrentSpeed=NewIntgCurrentSpeed;NewIntgCurrentSpeed=IntgCurrentSpeed;IntgCurrentSpeed=0;}
//      if (NewIntgCurrentSpeed<0) NewIntgCurrentSpeed=0;


    if ( ((MState.RunFlag&0xfff0)==0x6610)&&(R_CFG.LowSpeedTime!=0) )     //movp or movm
    {
       if (ZeroSpdTimeCnt<R_CFG.LowSpeedTime)
       {
         ZeroSpdTimeCnt++;IntgSpeedLenImpulses+=CurrentSpeed;
       }
       else
       {
         if (i32sabs(IntgSpeedLenImpulses)<CFGLowSpeedLenImpulses)
         {
           MState.ErrorFlags|=EF_NoEncFeedback;MState.RunFlag=RM_Error;//goto inidle;
         }
         ZeroSpdTimeCnt=0;IntgSpeedLenImpulses=0;
       }
    }
    else
    {
       ZeroSpdTimeCnt=0;IntgSpeedLenImpulses=0;
    }
//------------------------------------------------------------------------------------------


//-----------------------------------Refresh Dinamic LEDs-----------------------------------------------------------------------

//----------------------------------Check&Refresh Dinamic Kbd--------------------------------------------------------------------


 //-----------------------------------Check&Filter Input Signals ------------------------------------------------------------------
 //if (GetNPNRackDown==0)    //fix electrical tumblers wiring bug, if no diode present on rackdown output;   UGLY HACK!!!!
 //{                        //if no diode present, rackdown cmd returns zero on man input and produses temprary switch to man mode!!!!!
                         //as result no switch in man mode is posible during rack is open!!!!!!!!!!!!!!!!

   if (ModeManualSig) //from master board
   {
     HiMManCnt++;if (HiMManCnt>MManTimeCut) HiMManCnt=MManTimeCut+GlitchTime1; //glitch up to 7 periods long will be rejected
     LowMManCnt=0;
   }
   else
   {
     LowMManCnt++;if (LowMManCnt>MManTimeCut) LowMManCnt=MManTimeCut+GlitchTime1; //glitch up to 7 periods long will be rejected
     HiMManCnt=0;
   }

if ((R_CFG.SystB2&0x20)==0) //0x20->machine without spindel!!!!!!!!!!!
{
   if (Get_SpindelEnable==0)
   {
      LowCntSpindelEnable++;HiCntSpindelEnable>>=1;
   }
   else
   {
      HiCntSpindelEnable++;LowCntSpindelEnable>>=1;
   }
 }

 //}
 if (ModeStopSig)
 {
   HiMStopCnt++;if (HiMStopCnt>MStopTimeCut) HiMStopCnt=MStopTimeCut+GlitchTime1; //glitch up to 7 periods long will be rejected
   if (HiMStopCnt==MStopTimeCut){MState.ErrorFlags|=WF_StopIsPressed;}
   if (LowMStopCnt!=0) LowMStopCnt--;
 }
 else
 {
   LowMStopCnt++;if (LowMStopCnt>MStopTimeCut) LowMStopCnt=MStopTimeCut+GlitchTime1; //glitch up to 7 periods long will be rejected
   if (LowMStopCnt==MStopTimeCut){MState.ErrorFlags&=~WF_StopIsPressed;}
   if (HiMStopCnt!=0) HiMStopCnt--;
 }

 if (ModeAutoSig) //from master board
 {
   HiMAutoCnt++;if (HiMAutoCnt>MAutoTimeCut) HiMAutoCnt=MAutoTimeCut+GlitchTime1; //glitch up to 7 periods long will be rejected
   LowMAutoCnt=0;
 }
 else
 {
   LowMAutoCnt++;if (LowMAutoCnt>MAutoTimeCut) LowMAutoCnt=MAutoTimeCut+GlitchTime1; //glitch up to 7 periods long will be rejected
   HiMAutoCnt=0;
 }

 if (EndSwitchSig)
 {
   HiEndSwitchCnt++;if (HiEndSwitchCnt>ESwitchTimeCut) HiEndSwitchCnt=ESwitchTimeCut+GlitchTime2; //glitch up to 10 periods long will be rejected
   if (LowEndSwitchCnt!=0) LowEndSwitchCnt--;
 }
 else
 {
   LowEndSwitchCnt++;if (LowEndSwitchCnt>ESwitchTimeCut) LowEndSwitchCnt=ESwitchTimeCut+GlitchTime2; //glitch up to 10 periods long will be rejected
   if (HiEndSwitchCnt!=0) HiEndSwitchCnt--;
 }

 if ((R_CFG.SysB3&0x01)==0)
 {
 if (IndSnzNozhFiltered>2000)
 {
   HiCutFBCnt++;if (HiCutFBCnt>CutSwitchTimeCut)
   {
     HiCutFBCnt=CutSwitchTimeCut+GlitchTime2;
   } //glitch up to 10 periods long will be rejected
   if (LowCutFBCnt!=0) LowCutFBCnt--;
 }
 else
 {
   LowCutFBCnt++;if (LowCutFBCnt>CutSwitchTimeCut)
   {
    LowCutFBCnt=CutSwitchTimeCut+GlitchTime2;
   } //glitch up to 10 periods long will be rejected
   if (HiCutFBCnt!=0) HiCutFBCnt--;
 }
 }
 else
 {
 if (IndSnzNozhFiltered<2000) //!inverted sensor on knife
 {
   HiCutFBCnt++;if (HiCutFBCnt>CutSwitchTimeCut)
   {
     HiCutFBCnt=CutSwitchTimeCut+GlitchTime2;
   } //glitch up to 10 periods long will be rejected
   if (LowCutFBCnt!=0) LowCutFBCnt--;
 }
 else
 {
   LowCutFBCnt++;if (LowCutFBCnt>CutSwitchTimeCut)
   {
    LowCutFBCnt=CutSwitchTimeCut+GlitchTime2;
   } //glitch up to 10 periods long will be rejected
   if (HiCutFBCnt!=0) HiCutFBCnt--;
 }
 }

 if (Get_InACDC0)
 {
  if (HiACDC0Cnt<1000) HiACDC0Cnt++;
 }
 else
 {
  HiACDC0Cnt-=20;//! SIGNED 16bit
  if (HiACDC0Cnt<(-15000)) {HiACDC0Cnt+=20; if ((R_CFG.ReservSystBits&EF_ChainLax)==0){MState.ErrorFlags|=EF_ChainLax;} }
 }

   if (( OlCo.CurrentMenu==MenuTestIOs ))
   {
     if (HiMStopCnt>MStopTimeCut) {OlCo.CurrentMenu=0;MState.RunFlag=RM_Error;goto justerror;}
     NPNAOutDuty=OlCo.Analog2In;
     AOSPPullSpeed=OlCo.Analog1In;
     if (NPNAOutDuty>100) NPNAOutDuty=100;
     itmp16u=NPNAOutDuty*10; //SPINDEL SPEED!!!!!!
     itmp16u+=Ain2Bias;
     if (itmp16u>1000) {itmp16u=1000;}
     TIMER_CH0CV(TIMER3)=itmp16u;
//     TIM4->CCR1 =itmp16u;

     if (AOSPPullSpeed&0x8000) {itmp16u=Ain1Bias; goto tmsetpwmao;}  //pull speed
     itmp16u=AOSPPullSpeed*10+Ain1Bias;
     if (itmp16u>1000) {itmp16u=1000;}
     tmsetpwmao:;

     TIMER_CH1CV(TIMER2)=itmp16u;
//     TIM3->CCR2 = itmp16u;   //CCR2 max 1000

     MState.SP_Current=AOSPPullSpeed;
     MState.RunFlag=RM_Neutral;
     goto curregbegin;
   }

//-----------------------------------------------------------------------------------------------------------------------------------
  if (FlagFirstPiece>1)  MovingLR(pRgM); //speed regulator must be init first
  if (MState.RunFlag==RM_Error) goto justerror;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   if (MState.RunFlag==RM_CutOn)
   {
     MState.CurrentPosition=0; //<<<<<<<<<<<<<<<<<!!!!!!!!!!!!!!!!!!!!!!!!?????????????????????????????????HERE
     ZPosTarget=MState.CurrentPosition;ZPosIntErr=0;
     if (R_CFG.SystB2&0x10)
     {
      if (GetSpinRev)
      LoSpinRev_OFF;
      else
      HiSpinRev_ON;
     }

   // ??????? if (R_CFG.SystemBits&0x40) {OffNPNCutOff;OffNPNCutOn;OffNPNRackDown;} //bit 6 test

     RM_Delay=R_CFG.TimeToWaitFullStop;MState.RunFlag=RM_CutWaitStop; //in orig 200ms wait to full stop
     WrkRackCounter++;
     if (WrkRackCounter==R_CFG.RackCounter)
     {
        WrkRackCounter=0;CntRackDelayToOpen=R_CFG.RackDelayToOpen;
        CntRackDelayToOpen++;  //zero value cor
     }
   }

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 if ((LowMAutoCnt>MAutoTimeCut)) //now in auto mode
 {
    if ((R_CFG.SystB2&0x100)==0) goto ZSalgNo; //Not enabled zero speed mode
    if (R_CFG.SystB2&0x80) goto ZSalgNo; //check if shred inverter is used
    if ((MState.RunFlag&0xff00)==0x6600) goto ZSalgNo; // moving in progress
    ZeroSpeedReg();
  }
ZSalgNo:;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   if (MState.RunFlag==RM_CutWaitStop)
   {
     if (RM_Delay==0)
     {
       if (R_CFG.SystB2&0x02)  goto met_is_sel_CUT;
       if (R_CFG.FlyingCutSpeed!=0) goto met_is_sel_CUT; //1st variant Flyingcut machine

       if (R_CFG.HydrRegMethod==1) //new
       {
         FlagRegForceBellowZero=0;
         MState.SP_Current=20; //5;
         goto met_is_sel_CUT;
       }
       if (R_CFG.HydrRegMethod==2) //old
       {
//         if (R_CFG.FlyingCutSpeed==0)
//         {
           OffIGBTPull;       //mov p is off above, here again for security    !!!ANTI SHAKING
           FlagRegForceBellowZero=1;
//         }                                //!!!ANTI SHAKING
         goto met_is_sel_CUT;
       }
       if (R_CFG.HydrRegMethod==3) //new delay-RelaxTimeForDebit
       {
         FlagRegForceBellowZero=0;
         MState.SP_Current=20;
         goto met_is_sel_CUT;
       }
       if (R_CFG.HydrRegMethod==4) //flyng cut, especialy for rack opening
       {
//         if (R_CFG.FlyingCutSpeed==0)
//         {
           OffIGBTPull;       //mov p is off above, here again for security    !!!ANTI SHAKING
           MState.SP_Current=R_CFG.RackDebitFlyngCut;
           FlagRegForceBellowZero=0;
//         }                                //!!!ANTI SHAKING
         goto met_is_sel_CUT;
       }

//       if (R_CFG.HydrRegMethod==0) //universal
//       {
         FlagRegForceBellowZero=0;
         MState.SP_Current=110;
         goto met_is_sel_CUT;
//       }

met_is_sel_CUT:;
       OffNPNCutOff;RM_Delay=R_CFG.TimeForCutting;MState.RunFlag=RM_Cutting;
//compatibilty heavy noice 			 if (R_CFG.SystB2&0x02) RM_Delay*=2; //incr time *2 only for inverter models
//from timer       OffNPNRackDown;
     }
     else
     {
       RM_Delay--;
     }
   }

   if (MState.RunFlag==RM_Cutting)
   {
     OffNPNCutOff; //
//--------inverter model check ------------
     if (R_CFG.SystB2&0x02)
     {
       RM_Delay--;
			 CutProc();if (MState.RunFlag==RM_Error) goto  justerror;
     }
//------------------------end inverter check
     else  //hydraulic models.............
     {
       OnNPNCutOn;
       RM_Delay--;
       if (RM_Delay==0)
       {
         MState.RunFlag=RM_CutOff;
         RM_Delay=R_CFG.TimeForCuttOff;
       }
     }
   }
//rack timers only
//---------------------------------
//if ((MState.RunFlag&0xfff0)==0xc300) goto nochkrt;
//if (MState.RunFlag==RM_Neutral) goto nochkrt;
//if (MState.RunFlag==RM_Manual) goto nochkrt;
//if (MState.RunFlag==RM_Error) goto nochkrt;

if (CntRackDelayToOpen)
{
  CntRackDelayToOpen--;
  if (CntRackDelayToOpen==0)
  {
	  OnNPNRackDown;CntRackHoldTime=R_CFG.RackHoldTime;
  }
}
else
{
  if (CntRackHoldTime)
  {
     if ((R_CFG.SysB3&0x02))
     {
         if (CntRackHoldTime<(R_CFG.RackHoldTime>>1))
         {
          if (Get_InACDC2==0) {CntRackHoldTime=1;}else{CntRackHoldTime++;}
         }
     }
    CntRackHoldTime--;
    if (CntRackHoldTime==0)
    {
		  OffNPNRackDown; DropPressureInMan=1500;
    }
  }
}
//nochkrt:;
//---------------------------------
   if (MState.RunFlag==RM_CutOff)
   {
     OnNPNCutOff;OffNPNCutOn;// MState.SP_Current=SPC_DebitMaximum;
//from timer     if (WrkRackCounter==0) {OnNPNRackDown;}
     RM_Delay--;
     if (RM_Delay==0)
     {
        OffNPNCutOff;OffNPNCutOn;  //i.e. OnSliv
        SharedInvertorDelay=0;if (R_CFG.SystB2&0x80) {TS_B_ON;FanTHY3OFF;} //on rolls contactor //off knife contactor
        if (R_CFG.FlyingCutSpeed==0) FlagRegForceBellowZero=1;                                       //!!!ANTI SHAKING

        MState.RunFlag=RM_PreWait;
            R_CAL.TotalPieces++;

            #ifdef ARCHIVES_ON
            R_FLASH.TotalHPieces++;R_FLASH.TotalDPieces++;R_FLASH.TotalMPieces++;R_FLASH.TotalYPieces++;
            #endif

        if (R_CAL.SPPieces==0)
        {
             MState.RunFlag=RM_AutoCompleted;      //KRYPKAAA
        }
     }
   }


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   if (MState.RunFlag==RM_PreWait)
   {
     //OnSliv;
//?? ALTERNATIVE NO TO OFF MOVP AND SPEEDPOS REGS????
     MState.SP_Current=SPC_DebitMinimum;
     pRgM->fCurrentSpeed=SPC_SpeedMinimum;
     pRgM->rState&=~rSta_POSWORK;if (R_CFG.FlyingCutSpeed==0) pRgM->rState&=~rSta_SPDWORK; //     PosRegStatus=PRS_RegStopped;if (R_CFG.FlyingCutSpeed==0) SpdRegStatus=SPD_RegStopped;
     MState.RunFlag=RM_PreWait2;RM_Delay=1;
   }

   if (MState.RunFlag==RM_PreWait2)
   {
     //OnSliv;
//??ALTERNATIVE NO TO OFF MOVP AND SPEEDPOS REGS????
     MState.SP_Current=SPC_DebitMinimum;
     pRgM->fCurrentSpeed=SPC_SpeedMinimum;
     pRgM->rState&=~rSta_POSWORK;if (R_CFG.FlyingCutSpeed==0) pRgM->rState&=~rSta_SPDWORK; //     PosRegStatus=PRS_RegStopped;if (R_CFG.FlyingCutSpeed==0) SpdRegStatus=SPD_RegStopped;
     OffNPNCutOn;
     RM_Delay--;
     if (RM_Delay==0)
      {
          MState.RunFlag=RM_Wait;
      }
   }

   if (MState.RunFlag==RM_Wait)
   {
//?? ALTERNATIVE NO TO OFF MOVP AND SPEEDPOS REGS????
//     if ((R_CFG.FlyingCutSpeed==0)&&((R_CFG.SystB2&0x40)==0)) OffIGBTPull; //&&((R_CFG.SystB2&0x40)==0)

     if (FlagAutoIsCompleted)
     {
        FlagRegForceBellowZero=1;
        OffNPNHidraulicHOLD;OffIGBTPull;
        OffNPNCutOff;OffNPNCutOn;
     }
     else
     {
        OnNPNHidraulicHOLD;
     }
   }

   if (MState.RunFlag==RM_AutoCompleted) { RM_Delay=R_CFG.PostPullTime;MState.RunFlag=RM_AutoCompleted_wait;}
   if (MState.RunFlag==RM_AutoCompleted_wait)
   {
     NPNAOutDuty=0;// NewIntgCurrentSpeed=0;//????????
     if (R_CFG.CutPeriodSpindelSpeed==0){OffThySpindelENBL;}   //???
     pRgM->rState&=~rSta_POSSPDWORKS; //     PosRegStatus=PRS_RegStopped; SpdRegStatus=SPD_RegStopped;
     MState.SP_Current=R_CFG.PPULSpeed;  //15 percent
     AOSPPullSpeed=R_CFG.PPULSpeed;
     FlagRegForceBellowZero=0;
     pRgM->fCurrentSpeed=SPC_SpeedMinimum;
     OnIGBTPull;OffNPNCutOn;OffNPNRackDown; //chk
     if ((R_CFG.SystemBits&0x40)==0x00) {OnNPNCutOff;}
     RM_Delay--;
     if (RM_Delay==0)
     {
       LoTR6_OFF;OffThySpindelENBL;OffIGBTPull;
       FlagAutoIsCompleted=1;
       MState.RunFlag=RM_PreWait;
     }
   }


      if (MState.RunFlag==RM_StartMovRotPRS_W2007delay)
   {
    FlagRegForceBellowZero=1;FlagRemovDPart=1;
    MState.SP_Current=0;//fIntPartMult=0.001*fSecondIntPartMult;         //-0
     pRgM->rState&=~rSta_POSSPDWORKS; //     PosRegStatus=PRS_RegStopped; SpdRegStatus=SPD_RegStopped;
     MState.RunFlag=RM_StartMovRotPRS_2_W2007delay;
     RM_Delay=2007; //now 4 times //PreWaitDelay;
//checked in analis program     if (RM_Delay<10) RM_Delay=10;
   }

   if (MState.RunFlag==RM_StartMovRotPRS_2_W2007delay)
   {
     RM_Delay--;
     if (RM_Delay==2)
     {
           if (TagLimitMaxRetrCnt>MAXTGR) TagLimitMaxRetrCnt=MAXTGR;
//           pRgM->fCurrentSpeed=5.0f/((float)R_CFG.SpeedIntgTime)/(float)((MAXTGR+1)-TagLimitMaxRetrCnt);
           pRgM->fCurrentSpeed=pRgM->fMaxMovingSpeed/(float)((MAXTGR+1)-TagLimitMaxRetrCnt);

                 //!-----------^ 5% speed
           pRgM->fMaxMovingSpeed=pRgM->fCurrentSpeed;
           FlagRemovDPart=0;
     }
     //---------------------------------------------------------------------

     if (RM_Delay==0)
      {
        FlagRegForceBellowZero=0;  //OffSliv;  //OffCutOff;
        pRgM->rState=rSta_UncondREINIT;CalcRegulator(pRgM);pRgM->rState|=rSta_POSSPDWORKS; //PosRegStatus=PRS_RegStarted;
        MState.RunFlag=RunFlagCopy;
      }
   }

   if (MState.RunFlag==RM_xStartMovPRS)
   {
//--------inverter check ------------
     if (R_CFG.SystB2&0x02)
     {
       FlagRegForceBellowZero=0;
       MState.SP_Current=0;  //-20;
       FlagRemovDPart=0;
//       PosRegStatus=PRS_RegStarted;
//       RM_Delay=1;   //10=1ms PreWaitDelay;    }
//       CorCycleCnt=0;
//       MState.RunFlag=RunFlagCopy;
       if ( R_CFG.SystemBits&0x04)
       {
	  NPNAOutDuty=R_CFG.CutPeriodSpindelSpeed;
       }
       else
       {
	  NPNAOutDuty=InverterSpeed;
       }

       MState.RunFlag=RM_StartMovPRS_2;
       AOSPPullSpeed=0; //prevent move "jerk"
       RM_Delay=R_CFG.RelaxTimeForDebit;   //used as delay to spindel acceleration
       RM_Delay++; //if ==0 then 1
       CorCycleCnt=RM_Delay>>1;CorCycleCnt++;
       R_CFG.FlyingCutSpeed=0;

       goto met_is_sel_StartReg;

     }
//------------------------end inverter check


     if (R_CFG.FlyingCutSpeed!=0)
     {
       FlagRegForceBellowZero=0;
       //intact MState.SP_Current=0; // CopyOfSP_Current;
       FlagRemovDPart=0;
       if ((R_CFG.SystemBits&0x40)==0x00) {OnNPNCutOff;}// ili OnNPNCutOff; ili NOTHING/i.e Off Off/ ???//up pressure

       pRgM->rState|=(rSta_UncondREINIT|rSta_NoZIntegrREINIT);CalcRegulator(pRgM);pRgM->rState|=rSta_POSSPDWORKS; //PosRegStatus=PRS_RegWorking; SpdRegStatus=SPD_RegREStarted;

       if (R_CFG.SystB2&0x01)
       {
         MState.SP_Current=R_CFG.FlyingCutSpeed;
         pRgM->rState=rSta_UncondREINIT;CalcRegulator(pRgM);pRgM->rState|=rSta_POSSPDWORKS; //PosRegStatus=PRS_RegStarted;
       }


       NPNAOutDuty=InverterSpeed;
       MState.RunFlag=RM_StartMovPRS_2;
       RM_Delay=0; //R_CFG.RelaxTimeForDebit;   //10=1ms PreWaitDelay;
       RM_Delay++;
       CorCycleCnt=0xffff; //>>2;CorCycleCnt*=3;
       goto met_is_sel_StartReg;
     }

     if (R_CFG.HydrRegMethod==1) //new
     {
       //OffSliv;
       FlagRegForceBellowZero=1;
       MState.SP_Current=0; // CopyOfSP_Current;
       FlagRemovDPart=0;
       if ((R_CFG.SystemBits&0x40)==0x00) {OnNPNCutOff;}// ili OnNPNCutOff; ili NOTHING/i.e Off Off/ ???//up pressure
       pRgM->rState&=~rSta_POSSPDWORKS; //SpdRegStatus=SPD_RegStopped; PosRegStatus=PRS_RegStopped;
		     NPNAOutDuty=InverterSpeed;
       MState.RunFlag=RM_StartMovPRS_2;
       RM_Delay=R_CFG.RelaxTimeForDebit;   //10=1ms PreWaitDelay;
       RM_Delay++;
       CorCycleCnt=0xffff; //>>2;CorCycleCnt*=3;
       goto met_is_sel_StartReg;
     }

     if (R_CFG.HydrRegMethod==2) //old
     {
       FlagRegForceBellowZero=0;
       MState.SP_Current=0;  //-20;
       FlagRemovDPart=0; CorCycleCnt=0xffff;
       //OnNPNCutOn;// ili OnNPNCutOff; ili NOTHING/i.e Off Off/ ???//up pressure
      pRgM->rState&=~rSta_POSSPDWORKS; //SpdRegStatus=SPD_RegStopped; PosRegStatus=PRS_RegStopped;
		     NPNAOutDuty=InverterSpeed;
			 MState.RunFlag=RM_StartMovPRS_2;
       RM_Delay=R_CFG.RelaxTimeForDebit;   //10=1ms PreWaitDelay;    }
       RM_Delay++;
       goto met_is_sel_StartReg;
     }

     if (R_CFG.HydrRegMethod==3) //new delay-R_CFG.RelaxTimeForDebit
     {
       FlagRegForceBellowZero=0;
       MState.SP_Current=0; //-20;
       FlagRemovDPart=0; CorCycleCnt=0xffff;
       //OnNPNCutOn;// ili OnNPNCutOff; ili NOTHING/i.e Off Off/ ???//up pressure
       pRgM->rState&=~rSta_POSSPDWORKS; //SpdRegStatus=SPD_RegStopped; PosRegStatus=PRS_RegStopped;
		     NPNAOutDuty=InverterSpeed;
       MState.RunFlag=RM_StartMovPRS_2;
       RM_Delay=R_CFG.RelaxTimeForDebit;   //10=1ms PreWaitDelay;    }
       RM_Delay++;
       goto met_is_sel_StartReg;
     }
     if (R_CFG.HydrRegMethod==4) //flyng cut
     {
       FlagRegForceBellowZero=0;
       MState.SP_Current=R_CFG.RackDebitFlyngCut;
       FlagRemovDPart=0; CorCycleCnt=0xffff;
       //OnNPNCutOn;// ili OnNPNCutOff; ili NOTHING/i.e Off Off/ ???//up pressure
      pRgM->rState&=~rSta_POSSPDWORKS; //SpdRegStatus=SPD_RegStopped; PosRegStatus=PRS_RegStopped;
		     NPNAOutDuty=InverterSpeed;
       MState.RunFlag=RM_StartMovPRS_2;
       RM_Delay=R_CFG.RelaxTimeForDebit;   //10=1ms PreWaitDelay;    }
       RM_Delay++;
       goto met_is_sel_StartReg;
     }

// 0 or any    if (R_CFG.HydrRegMethod==0) //universal
//     {
       //OffSliv;
       //FlagRegForceBellowZero=0; CorCycleCnt=0;
       MState.SP_Current=0; //-20;
       FlagRemovDPart=0;
//     OnNPNCutOn;// ili OnNPNCutOff; ili NOTHING/i.e Off Off/ ???//up pressure
		     NPNAOutDuty=InverterSpeed;
      pRgM->rState&=~rSta_POSSPDWORKS; //SpdRegStatus=SPD_RegStopped; PosRegStatus=PRS_RegStopped;
       MState.RunFlag=RM_StartMovPRS_2;
       RM_Delay=R_CFG.RelaxTimeForDebit;   //10=1ms PreWaitDelay;
       if (RM_Delay==0) RM_Delay=1;
       CorCycleCnt=RM_Delay>>1;CorCycleCnt++;
 //    }
met_is_sel_StartReg:;
   }

   if (MState.RunFlag==RM_StartMovPRS_2)
   {
     CorCycleCnt--; if ( CorCycleCnt==0) { FlagRegForceBellowZero=0; }
     RM_Delay--;if (RM_Delay==0)
      {
        if ((R_CFG.SystemBits&0x40)==0x00) {OnNPNCutOff;} else {OffNPNCutOff;}//up pressure normaly knife is always driven back
        if (R_CFG.FlyingCutSpeed==0) {pRgM->rState=rSta_UncondREINIT;CalcRegulator(pRgM);pRgM->rState=rSta_POSSPDWORKS;} //PosRegStatus=PRS_RegStarted; } //66=moving
        CorCycleCnt=0;
        MState.RunFlag=RunFlagCopy;
      }
   }


   if (LowCntSpindelEnable>200)
   {
      if (SpindelContactorDelay<SPINDEL_CONTACTOR_DELAY)
      {
        SpindelContactorDelay++;
        if (SpindelContactorDelay==700){OnThySpindelENBL;MState.ErrorFlags|=WF_SpindelIsEnabled;}
      }
      else
      {if (MState.RunFlag==RM_Manual) {HiTR6_ON;}} //here SpindelContactorDelay=5000;
   }


   if (HiCntSpindelEnable>3000) //much time, because time is need to switch in manual from main
   {
     LoTR6_OFF;
     if ((LowMAutoCnt>MAutoTimeCut)&&(FlagAutoIsCompleted==0)) //i.e. automatic mode in execution
     {
       MState.RunFlag=RM_Error;
	MState.ErrorFlags|=EF_SpindelNotEnbldInA;
     }
     else
     {
       if (SpindelContactorDelay>3) { SpindelContactorDelay-=2;if (SpindelContactorDelay<=3) {SpindelContactorDelay=0;}}
       else { OffThySpindelENBL;MState.ErrorFlags&=~WF_SpindelIsEnabled;}
     }
   }


 if (MState.RunFlag==RM_Neutral)
 {
     FlagRegForceBellowZero=1;
     MState.SP_Current=SPC_DebitMinimum;
     //fCurrentSpeed=SPC_SpeedMinimum;
      pRgM->rState&=~rSta_POSSPDWORKS; //SpdRegStatus=SPD_RegStopped; PosRegStatus=PRS_RegStopped;
     OffNPNRackDown;
     OffIGBTPull;OffNPNCutOff;OffNPNCutOn;LoTR6_OFF; //i.e. OnSliv
 //    OffNPNBrakeMotor;//OffNPNBrakeMotorForward;
     OffNPNHidraulicHOLD;OffNPNCutOff;OffNPNCutOn;OffIGBTPull;OffNPNRackDown;OffIGBTBack; //OffNPNBrakeMotorBack;OffNPNBrakeMotorForward
     OnNPNBrakeMotor;                                                           //  !!!!!!!!!
     AinDivider6=4;
 }

 if (MState.RunFlag==RM_Manual)
 {
  	 if (DropPressureInMan) {DropPressureInMan--;if (DropPressureInMan==0) OffNPNCutOff;}//after cut drop pressure always in man mode
      pRgM->rState&=~rSta_POSSPDWORKS; //SpdRegStatus=SPD_RegStopped; PosRegStatus=PRS_RegStopped;
     if ( ManualModTimer>=600)  //increment every 1s from backgroung, auto off if no rotaiton for 10min
     {
       OffNPNHidraulicHOLD;MState.ErrorFlags|=WF_10minutesHydrOff;
       if (ManualModTimer>=602) {ManualModTimer=0;MState.ErrorFlags&=~WF_10minutesHydrOff;}
     }
     else
     {
       OnNPNHidraulicHOLD;
     }
//     // OffIGBTPull;OffNPNCutOff;OffNPNCutOn;
//     OffThySpindelENBL;OffNPNBrakeMotor;
     FlagRegForceBellowZero=0;
     NPNAOutDuty=OlCo.Analog1In/AinDivider6;
     MState.SP_Current=OlCo.Analog1In/AinDivider6;
     AOSPPullSpeed=OlCo.Analog1In/AinDivider6;
     AinDivider6=4;

  if ((KeyCutSig)&&(RM_Delay==0))
  {
    if (R_CFG.SystB2&0x02)
    {
      OffNPNCutOff; RM_Delay=R_CFG.TimeForCutting;
      if (R_CFG.SystB2&0x80)  ////is 1 invertor roll+knife?
      {
        if ((Get_InACDC2==0)) {OffIGBTPull;OffIGBTBack;goto justerror;} //wait rolls to stop, because inverter is shared!!!!!!
      }
      ResetKeyCutSig;CntRackDelayToOpen=10; //open rack after 1ms
      goto InvManCutStartedDone;
    }
//hydraulic models.........
    if (SubMRunFlag==0x0000) {SubMRunFlag++;OffNPNCutOff;OnNPNCutOn;RM_Delay=R_CFG.TimeForCutting;CntRackDelayToOpen=10;}
  }  //next lines outside because Indicator board may clear KeyCutSig

    if (SubMRunFlag==0x0001) {if (RM_Delay) {RM_Delay--;}else{SubMRunFlag++;OnNPNCutOff;OffNPNCutOn;RM_Delay=R_CFG.TimeForCuttOff;} }
    if (SubMRunFlag==0x0002)
    {
		  if (RM_Delay)
		  {RM_Delay--;}
		  else
            {
			 SubMRunFlag=0;OffNPNCutOff;OffNPNCutOn;DropPressureInMan=1500;ResetKeyCutSig;
             if ((R_CFG.SystemBits&0x40)==0x00) {OnNPNCutOff;}//up pressure for rack open
            }
    }

InvManCutStartedDone:;
 //--------inverter check ------------
     if ((RM_Delay!=0)&&(R_CFG.SystB2&0x02))
     {
        RM_Delay--;
		CutProc();
        if (R_CFG.SystB2&0x80)  goto nomanZSp; ////is 1 inverter roll+knife? Inv Shared->no moving allowed if cut working
     }
//------------------------end inverter check
// if (MState.RunFlag==RM_Manual)
// {
	 if ((KeyForwardSig)||(KeyBackwardSig))
	 {
	   DropPressureInMan=1500;
	   if ((R_CFG.SystemBits&0x40)==0x00) {OnNPNCutOff;}//up pressure normaly knife is always driven back
     }
   if ((SysTickCntr-iOlCoLastTimeSet)>7000){ResetKeyForwardSig;ResetKeyCutSig;ResetKeyBackwardSig;} //if Comm is lost->stop


   if (KeyForwardSig) {OnIGBTPull;ZPosTarget=MState.CurrentPosition;ZPosIntErr=0;goto nomanZSp;}//else{OffIGBTPull;}
   if (KeyBackwardSig) {OnIGBTBack;ZPosTarget=MState.CurrentPosition;ZPosIntErr=0;goto nomanZSp;}//else{OffIGBTBack;}
   if (RM_Delay==0) //means NO CutProc at work
   {
     if ((R_CFG.SystB2&0x200)) {ZeroSpeedReg();} else {OffIGBTBack;OffIGBTPull;} //Not enabled zero speed mode in manual
   }
nomanZSp:;
// }


 }

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   CalcRegulator(pRgM); //pRgM->fRegTmp=100.0f;
if (pRgM->rState&rSta_SPDWORK)
{
   MState.SP_Current=(int16_t)(pRgM->fRegTmp*fScaleCurInp);
   if(MState.SP_Current<0) {AOSPPullSpeed=0;}else{AOSPPullSpeed=MState.SP_Current;}

   if (R_CFG.SpindlePullRatio==0)
   {
     if (MState.SP_Current<=0) {NPNAOutDuty=1;} else {NPNAOutDuty=MState.SP_Current;}
     if (NPNAOutDuty<R_CFG.CutPeriodSpindelSpeed)   NPNAOutDuty=R_CFG.CutPeriodSpindelSpeed; //?????!!!!!!!!!!!
   }
}
else //i.e. Speed Regulator is OFF
{
  if ((MState.RunFlag!=RM_StartMovPRS_2)&&(MState.RunFlag!=RM_Manual)&&(MState.RunFlag!=RM_AutoCompleted_wait))
       { NPNAOutDuty=R_CFG.CutPeriodSpindelSpeed; } //RM_Manual&&RM_StartMovPRS sets the NPNAOutDuty
 // if (MState.RunFlag!=RM_Manual) AOSPPullSpeed=0; // set in RelaxTimeForDebit pause
}


//tpr1=(float)MState.CurrentPosition;tpr2=(float)MState.TargetPosition;
//tpr1=(float)(TargetPosition-CurrentPosition);
//nnnnnnnnnnn tpr2=fRegTmp; if (MState.RunFlag==RM_MovingM) tpr2=0-tpr2;

//tpr1=(float)MState.CurrentPosition;tpr2=(float)MState.TargetPosition; tpr3=fDPart;
//nnnnnnnnnnn  tpr1=(float)MState.CurrentPosition;//tpr2=PosRegStatus;//tpr2=fTargetPosition; //tpr2=fIntPart;

justerror:
curregbegin:
 CR_SP=(uint16_t)(MState.SP_Current*5+R_CFG.DebitDeadZeroCorection); //tstSPCmult;  // 4
// if (( OlCo.CurrentMenu==MenuTestIOs ))      //test mode
// {
//   if (CR_SP>0x7ff0) CR_SP=0; // NEGATIVE CASE!!!!!! v3.3 need also some autocalc for these coeff
// }
// else
// {
   if (CR_SP>0x4000) CR_SP=0; // NEGATIVE CASE!!!!!! v3.3 need also some autocalc for these coeff
// }
   if (( OlCo.CurrentMenu==MenuTestIOs )&&((R_CFG.SystB2&0x02))) {goto nochkmods;} //inverter model

   if (FlagRegForceBellowZero)
   {
     if (( OlCo.CurrentMenu==MenuTestIOs )) {goto crtstmod;}
     OffIGBTAOut;
   }
   else
   {
crtstmod:;
     CR_Error=CR_SP-ADCValue;
     CR_IntError=CR_IntError+(CR_Error);
//     if (( OlCo.CurrentMenu==MenuTestIOs ))      //test mode
//     {
//       if (CR_IntError>30000) CR_IntError=30000;
//       if (CR_IntError<-30000) CR_IntError=-30000;
//     }
//     else
//     {
       if (CR_IntError>5000) CR_IntError=5000;      //20000 in org!!!!!!!!!
       if (CR_IntError<-5000) CR_IntError=-5000;   //20000 in org!!!!!!!!!
//     }
//   RegTmp=(CR_Error>>2)+(CR_IntError); //01234????????? Gain=1/(2^n)
 //ORIG!!!!   CRTmp16s=(CR_Error)+(CR_IntError>>3); //01234????????? Gain=1/(2^n)   with +-20000

    //CRTmp16s=((CR_Error*10)/10)+(CR_IntError/8); //kp=10 ti=8
     CRTmp16s=CR_Error+(CR_IntError>>2); //orig /4      kp=10 ti=8

    //         CRTmp16s=(SHIMPER/2)+(CRTmp16s*10);
     if (CRTmp16s<0) {OffIGBTAOut;} else {OnIGBTAOut;}
    //          if (CRTmp16s>(SHIMPER-250)) CRTmp16s=(SHIMPER-250);
    //         P1DC4=CRTmp16s;
    if (( OlCo.CurrentMenu==MenuTestIOs ))  {goto nochkmods;}
  }

//-------------------------------------------------------------------------------------------------------------------------------
//nothing, just branch
 if (MState.RunFlag==RM_Error)
 {
     FlagRegForceBellowZero=1;
     MState.SP_Current=SPC_DebitMinimum;
     //fCurrentSpeed=SPC_SpeedMinimum;
     CntRackDelayToOpen=0;CntRackHoldTime=0;    //block timers controled rack
      pRgM->rState&=~rSta_POSSPDWORKS; //SpdRegStatus=SPD_RegStopped; PosRegStatus=PRS_RegStopped;
     OffIGBTPull;OffNPNCutOff;OffNPNCutOn;OffThySpindelENBL;LoTR6_OFF; //i.e. OnSliv
     OffNPNRackDown;FanTHY3OFF;TS_B_OFF;
     //?  OnNPNBrakeMotor;      //force Brake mode???????????????????
     OffThySpindelENBL;OffNPNHidraulicHOLD;OffNPNCutOff;OffNPNCutOn;OffIGBTPull;OffNPNRackDown;OffIGBTBack; //OffNPNBrakeMotorBack;OffNPNBrakeMotorForward
     AinDivider6=4;
 }


// if (MState.RunFlag==RM_Neutral){TIM4->CCR1 =10;TIM3->CCR2 =10;goto noupdPWM;}
 if (MState.RunFlag==RM_Neutral){TIMER_CH0CV(TIMER3)=10;TIMER_CH1CV(TIMER2)=10;goto noupdPWM;} //TIM4->CCR1 =10;TIM3->CCR2 =10;

 if ((R_CFG.SpindlePullRatio>=10000)||(R_CFG.SysB3&0x08))
 {
     if (MState.RunFlag==RM_Cutting)
     {
       itmp16u=10*R_CFG.CutPeriodSpindelSpeed;
     }
     else
     {
	if ((R_CFG.SubSpindle01mm)&&(NPNAfromSSPI))
	  {
	    itmp16u=10*NPNAfromSSPI;
	  }
	else
	  {
	    itmp16u=10*OlCo.Analog2In;
	  }
     }
 }
 else
 {
     if (NPNAOutDuty>100) NPNAOutDuty=100;
     itmp16u=(NPNAOutDuty*OlCo.Analog2In)/10; //SPINDEL SPEED!!!!!!
 }

 if (R_CAL.SPPieces==0)
 {
   if ((MState.RunFlag==RM_AutoCompleted_wait)||((MState.RunFlag&0xfff0)==0x0060))
   {
     HiTR6_ON;  //!KRYPKA because some PI9100 have ONLY Freewheel STOP (function for decelerated STOP have bug)
     itmp16u=0; //!KRYPKAAA
   }
 }
 if (MState.RunFlag==RM_Manual){if (itmp16u>200) itmp16u=200;}
 itmp16u+=Ain2Bias;
 if (itmp16u>1000) {itmp16u=1000;}
  TIMER_CH0CV(TIMER3)=(uint32_t)itmp16u;
//TIM4->CCR1 =itmp16u;


 if (AOSPPullSpeed&0x8000) {itmp16u=Ain1Bias; goto setpwmao;}  //pull speed
 itmp16u=AOSPPullSpeed*10+Ain1Bias;
 if (MState.RunFlag==RM_Manual){if (itmp16u>200) itmp16u=200;}
 if (itmp16u>1000) {itmp16u=1000;}
 setpwmao:;
 TIMER_CH1CV(TIMER2)=(uint32_t)itmp16u;
// TIM3->CCR2 = itmp16u;   //CCR2 max 1000
noupdPWM:
//------------------------------------------------------------------------------
//??move in main, IRQ slows down ADC DMA??? AnalisRS();
//------------------------------------------------------------------------------
//------------------CHECK FOR UNDERVOLTAGE STOP GO INSLEEP??--------------------
// move in IRQ????????????? some irq??????
 if (ADC_ArrRaw[MPWR_24In]>VoltageToSleep) {LowPwrCnt=0;} else { if ((R_CFG.ReservSystBits&EF_UnderVoltage)==0x0000) LowPwrCnt++; }
     //PVDIN analog CMP on interrupt???????????????
 if (LowPwrCnt>(R_CFG.DelayToSleep-5))
 {
    MState.RunFlag=RM_Error;OlCo.CurrentMenu=MenuMain;MState.ErrorFlags|=EF_UnderVoltage;
 }
//------------------CHECK FOR OVERCURRENT---------------------------------------
    if (ADC_ArrRaw[TotalCurrent]<((uint16_t)4000)) //~16A ~3.2V on AIN, shunt0.01 x gain 19.8
    {OverCurrentCnt=0;}
    else
    {
      OverCurrentCnt++;
      if (OverCurrentCnt>2) {
			  if ((R_CFG.ReservSystBits&EF_OverCurrent)==0x0000){
			    OffIGBTPull;OffNPNCutOff;OffNPNCutOn;OffThySpindelENBL;LoTR6_OFF; //i.e. OnSliv
          OffNPNRackDown;OffIGBTAOut;OffIGBTBack; //OffNPNBrakeMotor;
			    MState.RunFlag=RM_Error;OlCo.CurrentMenu=MenuMain;MState.ErrorFlags|=EF_OverCurrent;}
			}
    }
//------------------------------------------------------------------------------
if (HiMStopCnt>MStopTimeCut)
{
  if (((R_CFG.ReservSystBits&WF_StopIsPressed)==0x0000))
  {
    MState.RunFlag=RM_Error;FanTHY3OFF;
    MState.ErrorFlags|=WF_StopIsPressed;
    OlCo.CurrentMenu=MenuMain;
  }
}
else
{
      if (MState.RunFlag==RM_Error)
      {
        if (LowMManCnt==MManTimeCut) MState.RunFlag=RM_ClearedError; //transition state
        if (LowMAutoCnt==MAutoTimeCut) MState.RunFlag=RM_ClearedError; //transition
        goto nochkmods;
      }

//    if ( OlCo.CurrentMenu==MenuEncTest ) goto nochkmods;
//    if ( OlCo.CurrentMenu==MenuAutoCalcCReg ) goto nochkmods;
      if (( OlCo.CurrentMenu==MenuTestIOs )) goto nochkmods;

      if ( (LowMAutoCnt>MAutoTimeCut)&&((MState.RunFlag==RM_Manual)||(MState.RunFlag==RM_Neutral)) )
      {

        if ((SpindelContactorDelay>=SPINDEL_CONTACTOR_DELAY)||((R_CFG.SystB2&0x20)==0x20)) //0x20 no spindel at all
        {
          FlagAutoIsCompleted=0;OnNPNHidraulicHOLD;WrkRackCounter=0xf000; //prevent rack from accidentally open
          FlagFirstPiece=1;OffNPNRackDown;MState.CurrentPosition=0;MState.RunFlag=RM_CutOn; OlCo.CurrentMenu=MenuMain;
          SpindelContactorDelay=0;
          //spindel is enabled from if (LowCntSpindelEnable>200)
        }

      }

      if (LowMManCnt>MManTimeCut)
      {
         if (MState.RunFlag!=RM_Manual)
         {
           SubMRunFlag=0;MState.CurrentPosition=0;ZPosTarget=MState.CurrentPosition;ZPosIntErr=0;RM_Delay=0;
           SharedInvertorDelay=0;TS_B_ON;FanTHY3OFF; //on rolls contactor //off knife contactor
					 OffThySpindelENBL;OffNPNHidraulicHOLD;OffNPNCutOff;OffNPNCutOn;OffIGBTPull;OffNPNRackDown;OffIGBTBack; //OffNPNBrakeMotorBack;OffNPNBrakeMotorForward
           MState.ErrorFlags&=0x00000f00; //!! some INTERNAL SysHardware erros may not be cleared HERE????????????????????
         }  //first time when in man mode, current position will be cleared
         MState.RunFlag=RM_Manual;FlagAutoIsCompleted=0;
      }

      if ((HiMAutoCnt>MAutoTimeCut)&&(HiMManCnt>MManTimeCut)) { MState.RunFlag=RM_Neutral; /*MState.ErrorFlags=0;*/FlagAutoIsCompleted=0;}
nochkmods:;
}
//------------------CHECK FOR OVERVOLTAGE ??ON all coils?? ALARM??--------------

//------------------------------------------------------------------------------
}
