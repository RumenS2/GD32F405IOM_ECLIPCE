#include "variables.h"

volatile struct OldComands OlCo={0};

volatile struct Main_State MState={0};
volatile struct OutputsInputsState OISt={0};

volatile uint16_t ADC_Arr[AllADCCh]={0};
volatile uint16_t ADC_ArrRaw[AllADCCh]={0};
uint32_t   Old32ADC_Arr[AllADCCh]={0};

float fADC_ArrVolt[AllADCCh];
float fADC_ArrSpec[AllADCCh];

volatile uint16_t PWM1_Val = 2,PWM2_Val = 2;
volatile uint32_t OutsA32=0;

const struct ACFG C_CFGiom6={0xabcd55aa,
                        0,0,0,  //1.5mm def limit
                        0x0002,
                        0x0000,
                        14500,  //Pulling roller diameter /in 0,01mm/
                        2200,
                        300, //ramp stop in mm
                        50, //First ramp start in mm   400 for mini spindel
                        666,
                        0,
                        100,   //percent
                        100,
                        10,
                        5000,
                        38,     //36-38 iom6mm   22-24 iom12mm    //if (this_par--) then real_speed ++ /if speed can not reach 100%, regulator is saturated!!!/
                                                               //if (this_par++) then real_speed --
                        20000,
                        3,
                        0,
                        1,  //stop ramp low speed
                        10,
                        0,
                        100,
                        333,
                        490,
                        100,
                        20,
                        0,
                        0,0,
                        2200,2200,
                        0,
                        1,
                        0,  //SystB2;
                        115,100,
                        80,40,130,-10,
                        0,0,0,  //AIN1hAIN2lOffset,MaxBackwardMoveLimit01mm,rsvd323;
                        17,2500,4400, //rsuMT,Enc1RotPulsesX4,AutoStopRamp;
                        0,0,0, //SysB3,rss43,rss44;
                        0x0000, //cs32
                        0xa559c3b7
                       };

const struct ACFG C_CFGiom6S={0xabcd55aa,
                        0,0,0,  //1.5mm def limit
                        0x0002+0x0080,
                        0x0000,
                        15000,  //Pulling roller diameter /in 0,01mm/
                        2200,
                        300, //ramp stop in mm
                        50, //First ramp start in mm   400 for mini spindel
                        666,
                        0,
                        100,   //percent
                        5000,
                        10,
                        5000,
                        38,     //36-38 iom6mm   22-24 iom12mm    //if (this_par--) then real_speed ++ /if speed can not reach 100%, regulator is saturated!!!/
                                                               //if (this_par++) then real_speed --
                        20000,
                        3,
                        0,
                        1, //stop ramp low spees
                        10,
                        0,
                        100,
                        333,
                        470,
                        110,
                        20,
                        0, //HydrRegMethod
                        0,0,
                        2200,2200,
                        0,
                        1,
                        0,  //SystB2;
                        115,100,
                        80,40,130,-10,
                        0,0,0,
                        18,2500,4400,
                        0,0,0,
                        0x0000,
                        0xa559c3b7
                       };

const struct ACFG C_CFGiom12S={0xabcd55aa,
                        0,0,0,  //1.5mm def limit
                        0x000a+0x0080,
                        0x0000,
                        7000,  //Pulling roller diameter /in 0,01mm/
                        3700,
                        280, //ramp stop in mm
                        1, //First ramp start in mm
                        666,
                        0,
                        100,   //percent
                        3500,
                        1,   //rack counter must be 1
                        3000,  //rach hold time
                        24,     //36-38 iom6mm   22-24 iom12mm    //if (this_par--) then real_speed ++ /if speed can not reach 100%, regulator is saturated!!!/
                                                               //if (this_par++) then real_speed --
                        20000,
                        3,
                        5,
                        10,
                        99,
                        3000,
                        100,
                        333,
                        490,
                        110,
                        20,
                        0,
                        0,0,
                        3700,3700,
                        0,
                        1,
                        0,  //SystB2;
                        115,100,
                        80,40,130,-10,
                        0,0,0,
                        16,2500,4400,
                        0,0,0,
                        0x0000,
                        0xa559c3b7
                       };
const struct ACFG C_CFGiom12D={0xabcd55aa,
                        0,0,0,  //1.5mm def limit
                        0x0003, //+0x0080+0xa,
                        0x0000,
                        15000,  //Pulling roller diameter /in 0,01mm/
                        4700,
                        0, //280..ramp stop in mm
                        1, //First ramp start in mm
                        666,
                        20,    //spindel speed in cut period
                        0,   //0% recomended, in this case NPNAOutDuty follows SPCurrent*Analog2In/100
                        3500,
                        1,   //rack counter must be 1
                        3000,  //rach hold time
                        52,     //36-38 iom6mm   22-24 iom12mm    //if (this_par--) then real_speed ++ /if speed can not reach 100%, regulator is saturated!!!/
                        20000,
                        3,
                        5,
                        2,
                        99,
                        3000,
                        100,
                        333,
                        480,
                        110,
                        20,
                        0,
                        0,0,
                        3700,3700,
                        0,
                        1,
                        0,  //SystB2;
                        115,100,
                        80,40,130,-10,
                        0x00050060,0x00000000,0x00000000,
                        19,2500,4400,
                        0,0,0,
                        0x0000,
                        0xa559c3b7
                       };

const struct ACFG C_CFGiom16D={0xabcd55aa,
                        0,0,0,  //1.5mm def limit
                        0x004a+0x0080, //sys word
                        0x0000,
                        15000,  //Pulling roller diameter /in 0,01mm/
                        6000,  //time for cuting
                        0, //350..ramp stop in mm
                        50, //start ramp in mm   400 for mini spindel
                        666, //relax time for debit
                        0, //sub impulses 01mm
                        100,   //percent  spindel/pull ratio
                        5000, //postpull time
                        1,    //rackcnt
                        5000, //rack hold time
                        30,     //36-38 iom6mm   22-24 iom12mm   30 iom16(full speed) //if (this_par--) then real_speed ++ /if speed can not reach 100%, regulator is saturated!!!/
                                                               //if (this_par++) then real_speed --
                        20000, //low speed time for err
                        3,  //low speed len1mm for err
                        0,  //subspindel 01mm
                        1, //stop ramp low speed
                        10, //start ramp low speed
                        0,  //post spindle time
                        100,  //integral part mul16u 100=1.00
                        333,     //time to wait full stop
                        470,  //debit dead zero correction
                        110, //cc max speed regout
                        20,  //ppulspeed
                        0,
                        0,0,
                        6000,6000,
                        0,
                        1,
                        0, //SystB2;
                        115,100,
                        80,40,130,-10,
                        0,0,0,
                        20,2500,4400,
                        0,0,0,
                        0x0000,
                        0xa559c3b7
                       };
const struct ACFG C_CFGiom16F={0xabcd55aa,
                        0,0,0,  //1.5mm def limit
                        0x004a,  //def:no cutoff at pulling, no stop spindel at flycut +0x0080, //sys word
                        0x0000,
                        15000,  //Pulling roller diameter /in 0,01mm/
                        2400,  //time for cuting
                        400, //350.. ramp stop in mm
                        400, //start ramp in mm   400 for mini spindel
                        666, //relax time for debit
                        0, //sub impulses 01mm
                        100,   //percent  spindel/pull ratio
                        5000, //postpull time
                        1,    //rackcnt
                        8000, //rack hold time
                        30,     //36-38 iom6mm   22-24 iom12mm   30 iom16(full speed) //if (this_par--) then real_speed ++ /if speed can not reach 100%, regulator is saturated!!!/
                                                               //if (this_par++) then real_speed --
                        20000, //low speed time for err
                        3,  //low speed len1mm for err
                        0,  //subspindel 01mm
                        1, //stop ramp low speed
                        10, //start ramp low speed
                        0,  //post spindle time
                        100,  //integral part mul16u 100=1.00
                        333,  //time to wait full stop
                        440,  //debit dead zero correction
                        110, //cc max speed regout
                        20,  //ppulspeed
                        4,   //HydrRegMethod
                        0,0,  //Begin_Brake_Len1mm,Duration_Brake_TimeOrLen1mm;
                        4000,4400, //RackDelayToOpen,TimeForCuttOff;
                        22, //FlyingCutSpeed;
                        1,
                        0,  //SystB2;
                        115,100,
                        80,40,130,-10,
                        0,0,0,
                        21,2500,4400,
                        0,0,0,
                        0x0000,
                        0xa559c3b7
                       };

const struct ACFG C_CFGiom10S={0xabcd55aa,
                        0,0,0,  //1.5mm def limit
                        0x0002+0x0080,
                        0x0000,
                        15000,  //Pulling roller diameter /in 0,01mm/
                        2200,
                        0, //240.. ramp stop in mm
                        50, //First ramp start in mm   400 for mini spindel
                        666, //Second ramp start in mm
                        0,
                        100,   //percent
                        5000,
                        10,
                        5000,
                        40,     //37-39 iom3-10mm   22-24 iom12mm    //if (this_par--) then real_speed ++ /if speed can not reach 100%, regulator is saturated!!!/
                                                               //if (this_par++) then real_speed --
                        20000,
                        3,
                        0,
                        1, //stop ramp low spees
                        10,
                        0,
                        100,
                        333,
                        490,
                        110,
                        20,
                        0,
                        0,0,
                        2200,2200,
                        0,
                        1,
                        0,  //SystB2;
                        115,100,
                        80,40,130,-10,
                        0,0,0,
                        22,2500,4400,
                        0,0,0,
                        0x0000,
                        0xa559c3b7
                       };

const struct ACFG C_CFGiom12INV1={0xabcd55aa,
                        -50,50,0,  //1.5mm def limit
                        0x0002,
                        0x0000,
                        15000,  //Pulling roller diameter /in 0,01mm/
                        10600,
                        0, //240.. ramp stop in mm
                        50, //First ramp start in mm   400 for mini spindel
                        666, //relax time for debit
                        12,  //CutPeriodSpindelSpeed
                        100,   //percent
                        2,
                        1,
                        5000,
                        34,     //37-39 iom3-10mm   22-24 iom12mm    //if (this_par--) then real_speed ++ /if speed can not reach 100%, regulator is saturated!!!/
                                                               //if (this_par++) then real_speed --
                        20000,
                        3,
                        0,
                        1, //stop ramp low speed
                        10,
                        0,
                        100,
                        333,
                        490,
                        110,
                        20,
                        0,
                        0,0,
                        2200,2200,
                        0,
                        1,
                        0x02,  //SystB2;
                        115,100,
                        80,20,130,-10,
                        0x00010001,10000,0x00000000, //AIN1hAIN2lOffset,MaxBackwardMoveLimit01mm,rsvd323;
                        23,2500,7400, //rsuMT,Enc1RotPulsesX4,AutoStopRamp;
                        0,0,0, //SysB3,rss43,rss44;
                        0x0000, //cs32
                        0xa559c3b7
                       };

const struct ACFG C_CFGiom8INV={0xabcd55aa,
                        -10,10,0,  //1.5mm def limit
                        0x0002+0x0001,
                        0x0000,
                        7000,  //Pulling roller diameter /in 0,01mm/
                        10600,
                        0, //240.. ramp stop in mm
                        50, //First ramp start in mm   400 for mini spindel
                        666, //relax time for debit
                        12,  //CutPeriodSpindelSpeed
                        100,   //percent
                        2,
                        1,
                        5000,
                        13,     //37-39 iom3-10mm   22-24 iom12mm    //if (this_par--) then real_speed ++ /if speed can not reach 100%, regulator is saturated!!!/
                                                               //if (this_par++) then real_speed --
                        20000,
                        3,
                        0,
                        1, //stop ramp low speed
                        10,
                        0,
                        100,
                        333,
                        490,
                        110,
                        20,
                        0,
                        0,0,
                        2200,2200,
                        0,
                        1,
                        0x02+0x20,  //SystB2;
                        115,1500,
                        80,20,130,-10,
                        0x00050010,99999,0x00000000,
                        23,2500,4400,
                        0,0,0,
                        0x0000,
                        0xa559c3b7
                       };

const struct ACFG C_CFGiom16INV1={0xabcd55aa,
                        -500,500,0,  //1.5mm def limit
                        0x0082,
                        0x0000,
                        15000,  //Pulling roller diameter /in 0,01mm/
                        10600,
                        0, //240.. ramp stop in mm
                        30, //First ramp start in mm   400 for mini spindel
                        666, //relax time for debit
                        12,  //CutPeriodSpindelSpeed
                        100,   //percent
                        2,
                        1,
                        5000,
                        32,     //37-39 iom3-10mm   22-24 iom12mm    //if (this_par--) then real_speed ++ /if speed can not reach 100%, regulator is saturated!!!/
                                                               //if (this_par++) then real_speed --
                        20000,
                        3,
                        4500,
                        1, //stop ramp low speed
                        17,
                        0,
                        100,
                        333,
                        490,
                        110,
                        20,
                        0,
                        0,0,
                        2200,1500,
                        0,
                        1,
                        0x02+0x80,  //SystB2;
                        115,100,
                        80,20,130,-10,
                        0x00050010,60000,0x00000000,
                        25,2500,7000,
                        0,0,0,
                        0x0000,
                        0xa559c3b7
                       };

const struct ACFG C_CFGiom8INV2={0xabcd55aa,
                        -15,15,0,  //1.5mm def limit
                        0x0003,  //SystemBits
                        0x0000,  //ReseervSystembits
                        15000,  //Pulling roller diameter /in 0,01mm/
                        10600,  //time for cuting
                        0, //240.. ramp stop in mm
                        30, //First ramp start in mm   400 for mini spindel
                        666, //relax time for debit
                        30,  //CutPeriodSpindelSpeed
                        100,   //percent
                        0,
                        10,  ////RackCounter
                        2200,
                        30,     //37-39 iom3-10mm   22-24 iom12mm    //if (this_par--) then real_speed ++ /if speed can not reach 100%, regulator is saturated!!!/
                                                               //if (this_par++) then real_speed --
                        20000,
                        3, //low speed len
                        4500,
                        10, //stop ramp low speed
                        17,
                        0, //PostSpindleTime
                        80,
                        333,
                        490,
                        110,
                        20,
                        0,
                        0,0,
                        2200,1500,
                        0,
                        1,
                        0x02,  //SystB2;
                        115,100,
                        80,20,130,-10,
                        0x00050010,60000,0x00000000,
                        25,2500,7000,
                        0,0,0,
                        0x0000,
                        0xa559c3b7
                       };


const struct ALL_Calib C_CAL={0xabcd55aa,
                            0x0000000000000000,
                            0x00000000,
                            0x0000,
                            0x0000,
                            0x00,MT_iom6S,
                            0,
                            0,
                            0x0000,
                            0xa559c3b7
                              };

///*Variables placed in backup sram */

struct ACFG E_CFG[MaxCFGProfiles] __attribute__ ((section ("__SCFG"))); //__attribute__((at(0x08080000)));
struct ACFG R_CFG,R2_CFG;
struct ALL_Calib R_CAL;
struct ALL_Calib E_CAL __attribute__ ((section ("__SCAL")));  //__attribute__((at(0x08080f00)));
uint32_t* E_LastProfNumC0=(uint32_t*)0x40024ff0;  //0x08080ff0;
uint32_t* E_LastProfNumC1=(uint32_t*)0x40024ff4;  //0x08080ff4;
uint32_t  EEformalEnd __attribute__ ((section ("__EEEND")));  //=0x08080ffc;//=0 //this will produce eeror if overlaped
//!!!!!!!!! sections must be defined in linker stm32xxxxxx.ld
/*
	SCFG (NOLOAD) : AT (0x40024000)
	{
		*(__SCFG*)
	} > BKPRAM

	SCAL (NOLOAD) : AT (0x40024f00)
	{
		*(__SCAL*)
	} > BKPRAM

    EEND (NOLOAD) : AT (0x40024ffc)
	{
		*(__EEND*)
	} > BKPRAM

*/

//===========================================================================================//
//#pragma dataseg=MY_INIT_RAM



//volatile int16_t IndicRefreshTime;

//volatile uint16_t BlockKeyScanTime;
//volatile uint8_t LastKeyPressed;
//volatile uint16_t TmpLastKeyPressedCnt;
//volatile uint8_t TmpLastKeyPressed;
//volatile uint16_t BlinkTime,TETime;

volatile uint16_t LowPwrCnt,OverCurrentCnt;

volatile uint16_t HiMManCnt=0,LowMManCnt=0,HiMAutoCnt=0,LowMAutoCnt=0\
        ,HiMStopCnt=0,LowMStopCnt=0,HiEndSwitchCnt=0,LowEndSwitchCnt=0,HiCutFBCnt=0,LowCutFBCnt=0;
volatile uint16_t NPNAOutDuty,AinDivider6,NPNAfromSSPI;
volatile uint16_t WrkRackCounter,CntToPostSpindleTime;
volatile int16_t MSpeed,SPSpeed,kerr,HiACDC0Cnt=0;

volatile int32_t ESLimitPlusMinus,CopyOfCurrentPosition,CopyOfTargetPosition;
volatile int32_t CorCycleForceMov,SpindelStopTargetPos,CFGLowSpeedLenImpulses,IntgSpeedLenImpulses,CFGBegin_Brake_Len1mm,CFGDuration_Brake_TimeOrLen1mm;
volatile uint16_t OldPosCnt,BegStopLen,CorCycleCnt,Duration_BrakeTime,DropPressureInMan=0;
volatile uint16_t RunFlagCopy,SubMRunFlag=0;
volatile int16_t CurrentSpeed,IntgCurrentSpeed,NewIntgCurrentSpeed,OldIntgCurrentSpeed;
volatile uint16_t RM_Delay,ZeroSpdTimeCnt,IntgSpdCnt,ADCValue;
volatile uint16_t FlagRegForceBellowZero,FlagRemovDPart,FlagFirstPiece,FlagAutoIsCompleted;
volatile uint16_t InverterSpeed;
volatile float  fScaleCurInp;

volatile uint16_t ManualModTimer,HackIntPartFlag,TagLimitMaxRetrCnt,AOSPPullSpeed;
volatile int16_t prntint16s,VarArchivesMenu;

 //------------------------pos/speed regulator--------------------------------------------------
int16_t RegTmp,CR_Error,i16scs;
//float  fRegTmp,fSpeedClassicError,fDPart;
int16_t CR_IntError=0;
uint16_t CSPD_Cnt;
uint16_t CR_SP;
int32_t MovtmpOldCalculatedCorection;
uint16_t Ain1Bias=0,Ain2Bias=0;
//==========================================================================================//
volatile uint32_t SpindelContactorDelay=0,iOlCoLastTimeSet=0;
uint32_t HiCntSpindelEnable=0,LowCntSpindelEnable=0;
uint32_t Inv1FaultCnt=0,Inv2FaultCnt=0,Inv3FaultCnt=0;

int16_t AutoModePrntSel=0;
volatile uint16_t OneSecondCounter,HS_TimeCnt;
volatile uint16_t HiMMCnt=0,LowMMCnt=0;
















