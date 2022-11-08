
#ifndef __ANSTAT_H
#define __ANSTAT_H
void AdcsInt50uS(void);
void Anstat100uS(void);
//void AllOutsOFF(void);

#define rSta_READY  0x8000
#define rSta_FROZEN 0x0001
#define rSta_POSWORK   0x0200
#define rSta_SPDWORK   0x0400
#define rSta_POSSPDWORKS   0x0600
#define rSta_UncondREINIT 0x0004
#define rSta_NOCheckForRECALC 0x0008
#define rSta_NoZIntegrREINIT 0x0010

#define rMod_AutoStopRamp 0x0100
#define rMod_ESasCorector 0x0200
#define rMod_FlyngCut     0x0400
#define rMod_Inverter     0x0800

typedef struct {
//---------input parameters----------
   int32_t iTargetPos,iTargetPosCopy;  //imps/SpeedIntgTime
   int32_t iCurrentPos,iCurrentPosCopy; //imps
   float   fBegSpd,fBegSpdCopy; //[%]
   int32_t iBegSpd,iBegSpdCopy; //[%]
   float   fEndSpd,fEndSpdCopy; //[%]
   int32_t iEndSpd,iEndSpdCopy; //[%]
   float   fBegRamp,fBegRampCopy;//imps
   int32_t iBegRamp,iBegRampCopy;//imps
   float   fEndRamp,fEndRampCopy;//imps
   int32_t iEndRamp,iEndRampCopy;//imps

   float   FltBegStopLen100; //imps/mm
   int32_t ESLimitPlusMinusM;
   int32_t CalculatedMovCorection,LimitPlus,LimitMinus,OldStartMovTargetPosition,LastMovCurrentPosition,MaxBackMovLim;
//--------intermediate values----------
   float   fbq,feq; //add&subtract to/from current speed for BegRamp&EndRamp [%/imps]
   float   fbqMSE,feqMSE; //?or get speeds from table ArrStopPow05[MaxStopElemnts(MSE)]
   float   fCurrentSpeed,fCurrentSpeedOld,fMaxMovingSpeed,fMaxMovingSpeedCopy;
   float   fBegStartLenXfMaxMovSpd;
   float   fIntPart,fDPart,fRegTmp;
   float   fIntPartMult; //,intruptfIntPartMult;
   double  fIntgTargetPosition;
   float   fKmf,f1divSpeedIntgTime;
//----------work variables-------------
   int32_t i32cpM,i32tpM,DifPosM;
   int32_t i32RampAutoStopLen;
   int32_t ImpLenght;
   uint16_t tmpu16m,tmpu16m2;
   int16_t tmpi16m,tmpi16m2;
   int16_t rState,rMode,FlagGetLastMovCurrentPosition;
   float ftmpm,flt1RampAutoStopLen;
//----------output---------------------
 //  int32_t IntOutValPercent; //[%]

} RegStructure;

extern RegStructure  dRgM;      //!<-Main reg structure
extern RegStructure *pRgM; //=&dRgM;

void Scmoving(float fx,RegStructure *Rs);

#endif
