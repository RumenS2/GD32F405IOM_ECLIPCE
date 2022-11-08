#ifndef __VARIABLES_H
#define __VARIABLES_H

#include "gd32f4xx.h" //types

#define EF_NoEncFeedback      0x00000001
#define EF_EndSwitchNoEnd     0x00000002
#define EF_EndSwitchTooEarly  0x00000004
#define EF_EndSwitchBlock     0x00000008
#define EF_Inv1Fault          0x00000010
#define EF_Inv2Fault          0x00000020
#define EF_Inv3Fault          0x00000040
#define EF_ChainLax           0x00000080
#define EF_CALisLost          0x00000100
#define EF_CFGisLost          0x00000200 //for current profile
#define EF_HSE_FailedToStart  0x00000400
#define EF_WWG_Reset          0x00000400  //wdg reset
#define EF_ADCNotReady        0x00000800  //some hard err
#define EF_ARS_SomeErr        0x00000800  //some hard err
#define EF_HW_SomeErr         0x00000800  //some hard err
#define EF_KnifeNotMoving     0x00001000
#define EF_KnifeBlockMayBe    0x00002000
#define EF_CanNotStopRolls    0x00004000 //?????????????????
#define EF_OverCurrent        0x00008000
#define EF_OvrVoltage         0x00010000
#define EF_OvrHeatPump        0x00020000
#define EF_OvrHeatSpindel     0x00040000
#define EF_OvrHeatOil         0x00080000
#define EF_UnderVoltage       0x00100000
#define EF_UnderThrPump       0x00200000
#define EF_UnderThrSpindel    0x00400000
#define EF_UnderThrOil        0x00800000
#define EF_SpindelEnbldInNEUT 0x01000000
#define EF_SpindelNotEnbldInA 0x02000000

//#define EF_EmergencySTOP      0x80000000

#define WF_StopIsPressed      0x80000000
#define WF_SpindelIsEnabled   0x40000000
#define WF_10minutesHydrOff   0x20000000
#define WF_CutterNotInZPos    0x10000000
#define WF_LimitIsSkipped     0x08000000
#define WF_IntTempHeaterON    0x04000000

struct OldComands{
uint16_t ButtonsState;   //set from touch display board
uint16_t Analog1In;      //set from touch display board
uint16_t Analog2In;
uint16_t CurrentMenu;
};
extern volatile struct OldComands OlCo;
#define ModeManualSig   (OlCo.ButtonsState&0x0001)
#define ModeAutoSig     (OlCo.ButtonsState&0x0004)
#define KeyForwardSig   (OlCo.ButtonsState&0x0010)
#define ResetKeyForwardSig  (OlCo.ButtonsState&=~(0x0010))
#define KeyBackwardSig  (OlCo.ButtonsState&0x0020)
#define ResetKeyBackwardSig  (OlCo.ButtonsState&=~(0x0020))
#define KeyCutSig       (OlCo.ButtonsState&0x0040)
#define ResetKeyCutSig  (OlCo.ButtonsState&=~(0x0040))
#define KeyAutoCorOn    (OlCo.ButtonsState&0x1000)
#define SetKeyAutoCorOn OlCo.ButtonsState|=0x1000
#define ClrKeyAutoCorOn OlCo.ButtonsState&=~0x1000

#define MManTimeCut (2000) //must be >7 (see bellow)
#define MStopTimeCut (30*2) //must be >7 (see bellow)
#define MAutoTimeCut (2000) //must be >7 (see bellow)
#define ESwitchTimeCut (27) //must be >7 (see bellow)
#define CutSwitchTimeCut (27) //must be >7 (see bellow)
#define GlitchTime1 (7)
#define GlitchTime2 (7)




//extern uint16_t EEOutsSTOPState[4];

struct OutputsInputsState
{
 uint16_t  AODR,BODR,CODR,DODR;
 uint16_t  AIDR,BIDR,CIDR,DIDR;
};
struct Main_State{
 uint16_t  USERO;
 uint16_t  USERI;
 uint32_t  ErrorFlags;
 int32_t  CurrentPosition,TargetPosition;
 uint16_t  RunFlag;
 int16_t  SP_Current;
 int16_t  LastPosErr;
};
#define ReadyRstUSERI         1
#define ReadyLastCmdUSERI     2
#define WriteR2ConfigUSERI      0x1001
#define SetOutsA32USERI         0x10a5
#define SetOutsB32USERI         0x10b5
#define SetOutsC32USERI         0x10c5
#define SetOutsD32USERI         0x10d5

#define BadCprNLastCmdUSERI   0xffed
#define SomeErrLastCmdUSERI   0xffee
#define EnaChgEFUSERI          0xfffc

#define AllADCCh 7
extern volatile uint16_t ADC_Arr[AllADCCh]; //filtered
extern volatile uint16_t ADC_ArrRaw[AllADCCh];
extern uint32_t   Old32ADC_Arr[AllADCCh];

#define OilNozhTemp     0  //or nozh
#define SpindelTemp     1
#define PumpRolikiTemp  2  //or roliki
#define FlowRegCurrent  3
#define TotalCurrent    4 //(may be pressure)
#define SecFlowRegCurr  5
#define MPWR_24In       6 //may be input for IOM with KI for length
#define VoltageToSleep 1600 //raw adc value ~=14V

extern float fADC_ArrVolt[AllADCCh];
extern float fADC_ArrSpec[AllADCCh];

extern volatile struct Main_State MState;
extern volatile struct OutputsInputsState OISt;
extern volatile uint16_t PWM1_Val,PWM2_Val;
extern volatile uint32_t OutsA32;

#define MaxCFGProfiles 10
struct ACFG {
uint32_t    wID_ConfigValid;
int16_t      LimitM_In01mm;
int16_t      LimitP_In01mm;
int16_t      ESLimitPM_In01mm;
uint16_t      SystemBits;    //+1->Autocor ON  +2->Linear Stop +4->Inverter Work with speed measured
                           //+8->iom12/s&d/ model(4hyswitches) +16->END SWITCH +32 SecEndSwitch present
                           //+64 knife IS OFF in moving(!CLEAR! FOR 16d UZINSKII) +128 Motor Stops at every Cut
                           //+0x8000 negate encoder sign
uint32_t      ReservSystBits;
uint16_t      PullingRollDia001mm;  //about 15000 iom6&iom12D, about 7000 iom12S
uint16_t      TimeForCutting;   //in fact FullTime=CutOn/TimeForCutting/ + (CutOff/TimeForCutting/ Or RackHoldTime/Only If Rack is in Action/)
uint16_t      RampStop1mm;
uint16_t      BegStartRamp;
uint16_t      RelaxTimeForDebit;
uint16_t      CutPeriodSpindelSpeed;     //spindel speed in cut period
uint16_t      SpindlePullRatio;    //0 recomended, in this case NPNAOutDuty follows SPCurrent*Analog2In/100
uint16_t      PostPullTime;
uint16_t      RackCounter;     //MUST be 1 for iom12
uint16_t      RackHoldTime;    //about 5000 iom6, about 3000 iom12?????
uint16_t      SpeedIntgTime;  //about 43 iom6, about 22-24 iom12S, about 52 iom12D
uint16_t      LowSpeedTime;
uint16_t      LowSpeedLen1mm;
uint16_t      SubSpindle01mm;
uint16_t      StopRampLowSpeed;
uint16_t      StartRampLowSpeed;
uint16_t      PostSpindleTime; //0.1ms
uint16_t      IntPartMult16U;
uint16_t      TimeToWaitFullStop;
uint16_t      DebitDeadZeroCorection;
uint16_t      CcMaxSpdRegout;
uint16_t      PPULSpeed;
uint16_t      HydrRegMethod;
uint16_t      Begin_Brake_Len1mm;
uint16_t      Duration_Brake_TimeOrLen1mm; //if ==0, ending is Start of new piece!!!
uint16_t      RackDelayToOpen,TimeForCuttOff;
uint16_t      FlyingCutSpeed;
int16_t      RackDebitFlyngCut;
uint16_t      SystB2;                      //Secondary SystemBits
                                         //+1->Fixed flow on cutting/iom16/
                                         //+2->Excentric knife WITH FB
																				 //+4->spind revers
																				 //+8->Speed=0 from prewaitfullstop
																				 //+16->spindel revers at every cut
																				 //+32->Machine WITHOUT Spindell
																				 //+64-> no off roll inverter at cut

uint16_t      DelayToSleep,PreWaitFullStop;
int16_t      MaxOilTemp,MinFanTemp,MaxMotorTemp,MaxIntTemp;
int32_t      AIN1hAIN2lOffset,MaxBackwardMoveLimit01mm,rsvd323;
uint16_t      rsuMT,Enc1RotPulsesX4,AutoStopRamp;
int16_t      SysB3,rss43,rss44;

//float clrzerotest;
uint32_t    CS32;

uint32_t     ct0xa559c3b7;  //ct used for UNzeroing CS !!!
          };

struct ALL_Calib {
uint32_t    wID_CalibValid;
uint64_t      TotalLenght;
uint32_t      TotalPieces;
uint16_t      TotalPowerCicles;
uint16_t      RSCmdCod;
uint16_t      CProfileN,MachineType;
uint32_t      SPLenght;
uint32_t      SPPieces;
uint32_t    CS32;
uint32_t    ct0xa559c3b7;  //ct used for UNzeroing CS !!!
                 };
#define RSCmdCod_SetWhat 0x5a00
#define RSCmdCodMask_SetSPLenght  0x0001
#define RSCmdCodMask_SetSPPieces  0x0002
#define RSCmdCodMask_SetCProfileN 0x0004
#define RSCmdCodMask_SetMachineType 0x0008
#define RSCmdCodMask_GetR2_CFGMType 0x0010
#define RSCmdCodMask_GetR2_CFGProfileN 0x0020


#define MT_CurrnetR_CFG 0x00
#define MT_iom12S       0x10  //70mm roll
#define MT_iom6         0x11  //without spindle, only rools, dia 145mm
#define MT_iom6S        0x12 //spinedl russian hydraulic version
#define MT_iom12D       0x13 //spindel russian hydraulic version
#define MT_iom16D       0x14 //Uzinskii- SUBTRACT 64(0x40) knife must be ON in moving for UZINSKII
#define MT_iom16F       0x15
#define MT_iom10S       0x16
#define MT_iom12INV1    0x17
#define MT_iom8INV      0x18
#define MT_iom16INV1    0x19
#define MT_iom8INV2     0x1a

//===========================================================================================//
//===========================================================================================//
//extern  struct ACFG F_CFG __attribute__((space(eedata),  aligned(2)));
//extern  struct ALL_Calib F_CAL __attribute__((space(eedata),  aligned(2)));
//extern const struct ACFG C_CFG;

extern const struct ALL_Calib C_CAL;

//===========================================================================================//
//===========================================================================================//

//extern volatile int16_t IndicRefreshTime;

//extern volatile uint16_t BlockKeyScanTime;
//extern volatile uint8_t LastKeyPressed;
//extern volatile uint16_t TmpLastKeyPressedCnt;
//extern volatile uint8_t TmpLastKeyPressed;
//extern volatile uint16_t BlinkTime,TETime;

extern volatile uint16_t LowPwrCnt,OverCurrentCnt;

extern struct ACFG E_CFG[MaxCFGProfiles];
extern struct ACFG R_CFG,R2_CFG;
extern struct ALL_Calib R_CAL;
extern struct ALL_Calib E_CAL;
extern uint32_t* E_LastProfNumC0;
extern uint32_t* E_LastProfNumC1;


extern const struct ACFG C_CFGiom6;
extern const struct ACFG C_CFGiom6S;
extern const struct ACFG C_CFGiom12S;
extern const struct ACFG C_CFGiom12D;
extern const struct ACFG C_CFGiom16D;
extern const struct ACFG C_CFGiom16F;
extern const struct ACFG C_CFGiom10S;
extern const struct ACFG C_CFGiom12INV1;
extern const struct ACFG C_CFGiom8INV;
extern const struct ACFG C_CFGiom16INV1;
extern const struct ACFG C_CFGiom8INV2;
//==========================================================================================//
#define MenuMain 0x00                     // 0
#define MenuEncTest      (0x25+0x03)             // 4
// not used #define MenuDebitDeadZeroCorection (0x27+0x03)   // 4
//#define MenuRackDebitFlyngCut ((0x2f)+0x05)   //3
#define MenuTestIOs 355             //0x55



#define RM_xStartMovPRS  0xe011
#define RM_StartMovPRS_2  0xe012
#define RM_StartMovPRS_W2000delay 0xe013
#define RM_StartMovPRS_2_W2000delay 0xe014
#define RM_StartMovRotPRS_W2007delay 0xe015
#define RM_StartMovRotPRS_2_W2007delay 0xe016

#define RM_Neutral       0x2000

#define RM_Manual       0x1000

#define RM_AutoCompleted 0x1100
#define RM_AutoCompleted_wait 0x1101


#define RM_Error        0x00ff
#define RM_Wait         0x0000
#define RM_PreWait         0x0001
#define RM_PreWait2         0x0002
#define RM_Test1            0x0355
#define RM_ClearedError     0x0113
//!!!!!!!!!!!!!!!!!!!!!! if MState.RunFlag&0xff00==6600 then moving in progress
#define RM_MovingM       0x6611
#define RM_MovingP       0x6612
#define RM_MovingPtoES   0x6613
#define RM_MovStoppingM   0x6623
#define RM_MovStoppingP   0x6624
#define RM_MovStoppingFullM   0x6625
#define RM_MovStoppingFullP   0x6626
#define RM_MovStoppingFullPtoES   0x6627


#define RM_CutOn         0x0060
#define RM_CutWaitStop   0x0061
#define RM_Cutting       0x0062
#define RM_CutOff        0x0063
#define RM_CutStopping   0x0064


#define PRS_RegStopped 0x00
#define PRS_RegStarted 0x01
#define PRS_RegWorking 0x02

#define SPD_RegStopped 0x00
#define SPD_RegStarted 0x01
#define SPD_RegWorking 0x02
#define SPD_RegREStarted 0x11



#define RM_AC_WaitManMode 0xc301
#define RM_Wait_KBC2_NoFeAck 0xc302
#define RM_AC_WaitPositiveSpeed_01p 0xc303  //UpDn DebitDeadZeroCorection
#define RM_AC_WaitPositiveSpeed_100p 0xc304 //UPDn CcMaxSpdRegout SpeedIntgTime
#define RM_AC_WaitPositiveSpeed_100p_SIT 0xc305
#define RM_AC_WaitPositiveSpeed_END  0xc306

//==========================================================================================//

//#define ImpMaxMovColdSpeed  2.0        ????
//#define ImpMaxMovHotSpeed   5.0    ????//3.6

//#define RealMaxSpeed 100.0 //we make all in percent

#define SPC_DebitMinimum   -20
//#define SPC_DebitNominalMin 1 //100 //     15ob/min
//#define SPC_DebitNominal20  20
//#define SPC_DebitNominal30  30
//#define SPC_DebitNominal40  40  //250 //     40ob/min
//#define SPC_DebitNominalMax  100 //600 //400 500 600??       90ob/min
//#define SPC_DebitMaximum   110 //1000
//#define SPC_DebitPPH       200

//#define SpdToDebit   (SPC_DebitNominalMax/RealMaxSpeed)

#define SPC_SpeedMinimum 0
//#define SPC_SpeedNominal01 (0.01*RealMaxSpeed)
//#define SPC_SpeedNominal03 (0.03*RealMaxSpeed)
//#define SPC_SpeedNominal07 (0.07*RealMaxSpeed)
//#define SPC_SpeedNominal10 (0.1*RealMaxSpeed)
//#define SPC_SpeedNominal20 (0.2*RealMaxSpeed)
//#define SPC_SpeedNominal30 (0.3*RealMaxSpeed)
//#define SPC_SpeedNominal40 (0.4*RealMaxSpeed)
//#define SPC_SpeedNominal50 (0.5*RealMaxSpeed)
//#define SPC_SpeedNominal100 RealMaxSpeed
//#define SPC_SpeedNominal120 (1.2*RealMaxSpeed)
//#define SPC_SpeedNominal200 (RealMaxSpeed*2)
//#define SPC_SpeedNominalPPH (RealMaxSpeed*2)


extern volatile uint16_t HiMManCnt,LowMManCnt,HiMAutoCnt,LowMAutoCnt\
               ,HiMStopCnt,LowMStopCnt,HiEndSwitchCnt,LowEndSwitchCnt,HiCutFBCnt,LowCutFBCnt;
extern volatile uint16_t NPNAOutDuty,AinDivider6,NPNAfromSSPI;
extern volatile uint16_t WrkRackCounter,CntToPostSpindleTime;
extern volatile int16_t MSpeed,SPSpeed,kerr,HiACDC0Cnt;


extern volatile int32_t ESLimitPlusMinus,CopyOfCurrentPosition,CopyOfTargetPosition;
extern volatile int32_t CorCycleForceMov,SpindelStopTargetPos,CFGLowSpeedLenImpulses,IntgSpeedLenImpulses,CFGBegin_Brake_Len1mm,CFGDuration_Brake_TimeOrLen1mm;
extern volatile uint16_t OldPosCnt,BegStopLen,CorCycleCnt,Duration_BrakeTime,DropPressureInMan;
extern volatile uint16_t RunFlagCopy,SubMRunFlag;
extern volatile int16_t CurrentSpeed,IntgCurrentSpeed,NewIntgCurrentSpeed,OldIntgCurrentSpeed;
extern volatile uint16_t RM_Delay,ZeroSpdTimeCnt,IntgSpdCnt,ADCValue;
extern volatile uint16_t FlagRegForceBellowZero,FlagRemovDPart,FlagFirstPiece,FlagAutoIsCompleted;
extern volatile uint16_t InverterSpeed;
extern volatile float  fScaleCurInp;

extern volatile uint16_t ManualModTimer,HackIntPartFlag,TagLimitMaxRetrCnt,AOSPPullSpeed;
extern volatile int16_t prntint16s,VarArchivesMenu;

 //------------------------pos/speed regulator--------------------------------------------------
extern int16_t RegTmp,CR_Error,i16scs;
//extern float  fRegTmp,fSpeedClassicError,fDPart;

extern int16_t CR_IntError;
extern uint16_t CSPD_Cnt;
extern uint16_t CR_SP;
extern int32_t MovtmpOldCalculatedCorection;
extern uint16_t Ain1Bias,Ain2Bias;

//==========================================================================================//
extern volatile uint32_t SpindelContactorDelay,iOlCoLastTimeSet;
extern uint32_t HiCntSpindelEnable,LowCntSpindelEnable;
extern uint32_t Inv1FaultCnt,Inv2FaultCnt,Inv3FaultCnt;


extern int16_t AutoModePrntSel;
extern volatile uint16_t OneSecondCounter,HS_TimeCnt;
extern volatile uint16_t HiMMCnt,LowMMCnt;

void   GetMTDefaultR2CFG(uint8_t  MachineType); //from main;
void   GetCProfileNR2CFG (uint8_t PN); //from main
void   SetCProfileInRCFG (uint8_t PN); //from main
#endif
