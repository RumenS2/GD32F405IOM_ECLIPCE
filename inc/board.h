/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __BOARD_H
#define __BOARD_H

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define MODE_SW_DEBUG 0    //1
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define CurrentSVpBV (uint16_t)(0x2211)

#define ENCODER_USED  1

/* Includes ------------------------------------------------------------------*/

//#include "gd32f4xx.h"

//!4  Port A
//#define RX485_3_AS_A GPIO_PIN_0
//#define GetRX485_3_AS_A (GPIOA->IDR&RX485_3_AS_A)  //for 485EncoderRcv A&A~
//#define RX485_2_AS_B GPIO_PIN_1                                          //T2ch1&T2ch2 pair
//#define GetRX485_2_AS_B (GPIOA->IDR&RX485_2_AS_B)  //for 485EncoderRcv B&B~
//GPIOpin2=tx, GPIOpin3=rx
//#define TX_485_2_AS_EMPTY_PD_INPUT GPIO_PIN_2
//#define RX_485_2_AS_EMPTY_PD_INPUT GPIO_PIN_3 //!!short conected with GPIOA_Pin_1 in this board

//#define ADIn0_Pin GPIO_PIN_4
#define Get_ADIn0 (GPIO_ISTAT(GPIOA)&GPIO_PIN_4)  //analog input 0 == current
//#define ADIn1_Pin GPIO_PIN_5
#define Get_ADIn1 (GPIO_ISTAT(GPIOA)&GPIO_PIN_5)  //analog input 1 == voltage
//#define ADIn2_Pin GPIO_PIN_6
#define Get_ADIn2 (GPIO_ISTAT(GPIOA)&GPIO_PIN_6)  //analog input 2 == hydr pump t
//#define ADInTR2_Pin GPIO_PIN_7
#define Get_ADInTR2 (GPIO_ISTAT(GPIOA)&GPIO_PIN_7)  //FlowRegCurrent
//#define RE_485_1_Pin GPIO_PIN_8                   //>>>>>>
#define LoRCV_Enbl_485_0 GPIO_BC(GPIOA)=(uint32_t)GPIO_PIN_8   //000000
#define HiTRN_Enbl_485_0 GPIO_BOP(GPIOA)=(uint32_t)GPIO_PIN_8  //111111
//GPIOpin9=tx, GPIOpin10=rx
//#define TX_485_1 GPIO_PIN_9
//#define RX_485_1 GPIO_PIN_10
//#define InACDC0_Pin GPIO_PIN_11
#define Get_InACDC0 (GPIO_ISTAT(GPIOA)&GPIO_PIN_11)  //may be USB or DI optocoupler
//#define InACDC1_Pin GPIO_PIN_12
#define Get_InACDC1 (GPIO_ISTAT(GPIOA)&GPIO_PIN_12) //may be USB DI on optocoupler
//GPIOpin13&14==debug
//#define SW_A_Pin GPIO_PIN_13                   //>>>>>>
#define SW_A_OFF GPIO_BC(GPIOA)=(uint32_t)GPIO_PIN_13 //000000
#define SW_A_ON GPIO_BOP(GPIOA)=(uint32_t)GPIO_PIN_13 //111111
//#define CK_A_Pin GPIO_PIN_14                   //>>>>>>
#define OffNPNHeater GPIO_BC(GPIOA)=(uint32_t)GPIO_PIN_14 //000000
#define OnNPNHeater  GPIO_BOP(GPIOA)=(uint32_t)GPIO_PIN_14 //111111

//#define THY3_REL4_Pin GPIO_PIN_15                     //>>>>>>
#define FanTHY3OFF GPIO_BC(GPIOA)=(uint32_t)GPIO_PIN_15
#define FanTHY3ON  GPIO_BOP(GPIOA)=(uint32_t)GPIO_PIN_15
#define GetFanThy  (GPIO_OCTL(GPIOA)&GPIO_PIN_15)

//!4---------------  Port B  ---------------------
//#define ADInTR0_Pin GPIO_PIN_0
#define Get_ADInTR0 (GPIO_ISTAT(GPIOB)&GPIO_PIN_0)  //analog input TR0/Total Current/ CurrentFeedback

//#define RE_485_2_Pin GPIO_PIN_1
#define LoRCV_Enbl_485_1 GPIO_BC(GPIOB)=(uint32_t)GPIO_PIN_1    //000000
//in encoder mode only LOW is perm!!!
#if ENCODER_USED==0
#define HiTRN_Enbl_485_1 GPIO_BOP(GPIOB)=(uint32_t)GPIO_PIN_1   //111111
#else
#define HiTRN_Enbl_485_1
#endif
//#define THY2_Pin GPIO_PIN_2                 //>>>>>>
#define OffThySpindelENBL GPIO_BC(GPIOB)=(uint32_t)GPIO_PIN_2 //000000
#define OnThySpindelENBL GPIO_BOP(GPIOB)=(uint32_t)GPIO_PIN_2 //111111


//GPIOpin3&4==debug
//#define TS_B_Pin GPIO_PIN_3                   //>>>>>>
#define TS_B_OFF GPIO_BC(GPIOB)=(uint32_t)GPIO_PIN_3 //000000
#define TS_B_ON GPIO_BOP(GPIOB)=(uint32_t)GPIO_PIN_3 //111111
//#define TRST_B_Pin GPIO_PIN_4                   //>>>>>>
#define OffC9_PB4 GPIO_BC(GPIOB)=(uint32_t)GPIO_PIN_4 //000000 LoLED1_B_OFF
#define OnC9_PB4 GPIO_BOP(GPIOB)=(uint32_t)GPIO_PIN_4 //111111 HiLED1_B_ON

//#define PWMREL0_Pin GPIO_PIN_5                     //>>>>>>
#define LoPWMREL0_OFF GPIO_BC(GPIOB)=(uint32_t)GPIO_PIN_5     //000000
#define HiPWMREL0_ON GPIO_BOP(GPIOB)=(uint32_t)GPIO_PIN_5     //111111
//#define PWMREL1_Pin GPIO_PIN_6      //>>>>>>
#define LoPWMREL1_OFF GPIO_BC(GPIOB)=(uint32_t)GPIO_PIN_6     //000000
#define HiPWMREL1_ON GPIO_BOP(GPIOB)=(uint32_t)GPIO_PIN_6     //111111


//#define SHRO_Pin GPIO_PIN_7
#define OffSHRO GPIO_BC(GPIOB)=(uint32_t)GPIO_PIN_7       //Second hydraulic regulator out
#define OnSHRO GPIO_BOP(GPIOB)=(uint32_t)GPIO_PIN_7       //Second hydraulic regulator out
//#define TR0_Pin GPIO_PIN_8                     //>>>>>>
#define OffIGBTPull GPIO_BC(GPIOB)=(uint32_t)GPIO_PIN_8       //000000
#define OnIGBTPull  GPIO_BOP(GPIOB)=(uint32_t)GPIO_PIN_8       //111111
//#define SecLED GPIO_PIN_9
#define OnCPUSecLED GPIO_BC(GPIOB)=(uint32_t)GPIO_PIN_9
#define OffCPUSecLED GPIO_BOP(GPIOB)=(uint32_t)GPIO_PIN_9
//#define TR4_Pin GPIO_PIN_10                     //>>>>>>
#define OffNPNCutOff GPIO_BC(GPIOB)=(uint32_t)GPIO_PIN_10       //000000
#define OnNPNCutOff GPIO_BOP(GPIOB)=(uint32_t)GPIO_PIN_10       //111111 #define GetStateBackward_C4 (GPIOB->ODR&TR4_Pin)
//#define TR5_Pin GPIO_PIN_11                     //>>>>>>
#define OffNPNCutOn GPIO_BC(GPIOB)=(uint32_t)GPIO_PIN_11       //000000
#define OnNPNCutOn GPIO_BOP(GPIOB)=(uint32_t)GPIO_PIN_11       //111111

//#define P24VC1_Pin GPIO_PIN_12
#define Get_P24VC1 (GPIO_ISTAT(GPIOB)&GPIO_PIN_12)     //DIGITAL From 24V in

// #define OnAd0_Pin GPIO_PIN_13
 #define Get_F1err (GPIO_ISTAT(GPIOB)&GPIO_PIN_13)     //Get_OnAd0
// #define OnAd1_Pin GPIO_PIN_14
 #define Get_F2err (GPIO_ISTAT(GPIOB)&GPIO_PIN_14)     //Get_OnAd1
// #define OnAd2_Pin GPIO_PIN_15
 #define Get_F3err (GPIO_ISTAT(GPIOB)&GPIO_PIN_15)     //Get_OnAd2

//!4----Port C------------


// #define TR6_Pin GPIO_PIN_0                     //>>>>>>
 #define LoTR6_OFF GPIO_BC(GPIOC)=(uint32_t)GPIO_PIN_0       //000000
 #define HiTR6_ON GPIO_BOP(GPIOC)=(uint32_t)GPIO_PIN_0       //111111
// #define TR7_Pin  GPIO_PIN_1                     //>>>>>>
 #define OffNPNBrakeMotor GPIO_BC(GPIOC)=(uint32_t)GPIO_PIN_1       //000000
 #define OnNPNBrakeMotor GPIO_BOP(GPIOC)=(uint32_t)GPIO_PIN_1       //111111

//#define ADIn3_Pin GPIO_PIN_2
#define EndSwitchSig (GPIO_ISTAT(GPIOC)&GPIO_PIN_2)  //NO!!! analog fn

//#define OnAd3_Pin GPIO_PIN_3
#define Gettoil (GPIO_ISTAT(GPIOC)&GPIO_PIN_3)       //t oil termistor CutFBSig !!!!!!!! Analog->digital?

//#define InACDC3_Pin GPIO_PIN_4
#define ModeStopSig (GPIO_ISTAT(GPIOC)&GPIO_PIN_4)   //may be analog or DI on optocoupler

//#define InACDC2_Pin GPIO_PIN_5
#define anowGet_InACDC2 (GPIO_ISTAT(GPIOC)&GPIO_PIN_5)   // Analog on optocupuler U13

//#define TR1_Pin GPIO_PIN_6                   //>>>>>>
#define OffIGBTBack GPIO_BC(GPIOC)=(uint32_t)GPIO_PIN_6    //000000
#define OnIGBTBack GPIO_BOP(GPIOC)=(uint32_t)GPIO_PIN_6    //111111
//#define REL2_SpindRevers GPIO_PIN_7                  //>>>>>>
#define LoSpinRev_OFF GPIO_BC(GPIOC)=(uint32_t)GPIO_PIN_7  //000000
#define HiSpinRev_ON GPIO_BOP(GPIOC)=(uint32_t)GPIO_PIN_7  //111111
#define GetSpinRev   (GPIO_OCTL(GPIOC)&GPIO_PIN_7)
//#define TR2_Pin GPIO_PIN_8                   //>>>>>>
#define OffNPNRackDown GPIO_BC(GPIOC)=(uint32_t)GPIO_PIN_8    //000000
#define OnNPNRackDown GPIO_BOP(GPIOC)=(uint32_t)GPIO_PIN_8    //111111
//#define TR3_Pin GPIO_PIN_9                   //>>>>>>
#define OffIGBTAOut GPIO_BC(GPIOC)=(uint32_t)GPIO_PIN_9    //000000
#define OnIGBTAOut GPIO_BOP(GPIOC)=(uint32_t)GPIO_PIN_9    //111111
//GPIOpin10=tx, GPIOpin11=rx
//#define TX_485_3_AS_EMPTY_PD_INPUT GPIO_PIN_10
//#define RX_485_3_AS_EMPTY_PD_INPUT GPIO_PIN_11//!!short conected with GPIOA_Pin_0 in this board

//#define RE_485_3_Pin GPIO_PIN_12                           //>>>>>>
#define LoRCV_Enbl_485_2 GPIO_BC(GPIOC)=(uint32_t)GPIO_PIN_12      //000000
//in encoder mode only LOW is perm!!!
#if ENCODER_USED==0
#define HiTRN_Enbl_485_2 GPIO_BOP(GPIOC)=(uint32_t)GPIO_PIN_12)     //111111
#else
#define HiTRN_Enbl_485_2
#endif
//#define InACDC4_Pin GPIO_PIN_13
#define Get_SpindelEnable (GPIO_ISTAT(GPIOC)&GPIO_PIN_13)

//Port D
//#define REL3_Pin GPIO_PIN_2                 //>>>>>>
#define OffNPNHidraulicHOLD GPIO_BC(GPIOD)=(uint32_t)GPIO_PIN_2 //000000
#define OnNPNHidraulicHOLD GPIO_BOP(GPIOD)=(uint32_t)GPIO_PIN_2 //111111


//#define ADC1_Ch_PressureSens    ((uint8_t)ADC_CHANNEL_8) //PB0 adc12 ->1
//#define ADC2_Ch_FlowReg    ((uint8_t)ADC_CHANNEL_7)  //PA7 adc12->2
//#define ADC1_TransI        ((uint8_t)ADC_CHANNEL_4) //PA4 adc12 ->adc1
//#define ADC2_TransU        ((uint8_t)ADC_CHANNEL_5) //PA5 adc12 ->adc2
//#define ADC3_Ch_Water    ((uint8_t)ADC_CHANNEL_13)  //PC3 adc123 ->3
//#define ADC2_Ch_Thyr       ((uint8_t)ADC_CHANNEL_6) //PA6 adc12 ->2
//#define ADC3_Ch_Oil        ((uint8_t)ADC_CHANNEL_12) //PC2 adc123 ->3
//#define ADC1_PressureRegCur   ((uint8_t)ADC_CHANNEL_15)  //PC5 adc12->1
////#define NoneADC_Ch_24VIn        ((uint8_t)ADC_Channel_18)
//// some Other defs


#define SPINDEL_CONTACTOR_DELAY 4000

#endif










/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
