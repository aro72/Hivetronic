/*
 * hivetronic.h
 *
 *  Created on: 16 janv. 2017
 *      Author: frq09465
 */

#ifndef HIVETRONIC_H_
#define HIVETRONIC_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// INCLUDES
#include "SX1272.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <String.h>
#include <HX711.h>
#include <time.h>
#include "stm32l4xx_hal.h"
#include "error.h"
#include "power.h"
#include "sorting.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL CONFIGURATIONS
// LoRa CONFIGURATION
#define DEFAULT_DEST_ADDR 		1
#define LORAMODE  				4
#define LORAPOWER 				'x'
#define LORA_NODE_ADDR 			8
#define LORA_REPORTING_PERIOD	10 // in second
#define PROCESSING_DURATION 	5 // FIX THIS: to be measured and updated
// HX711  CONFIGURATION
#define HX711_GAIN				128


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// PIN ASSIGNMENT
#define HX711_DOUT1   		A1
#define HX711_PD_SCK2 		A2
#define HX711_DOUT2   		3
#define HX711_PD_SCK3 		4
#define HX711_DOUT3   		5
#define HX711_PD_SCK4 		6
#define HX711_DOUT4   		7
#define HX711_PD_SCK1 		8
#define DHTPIN 				9
/* Other pins used for LoRa
		SPI CS/NSS 	--> 	10
		SPI MOSI 	--> 	11
		SPI MISO 	--> 	12
		SPI SCK  	--> 	13
		LoRa RESET 	--> 	A0
*/



///////////////////////////////////////////////////////////////////////////////////////////////////////////
// POWER MANAGEMENT DEFINES
#define PM_RUN1				1
#define PM_RUN2				2
#define PM_LPRUN			3
#define PM_SLEEP			4
#define PM_LPSLEEP			5
#define PM_STOP1			6
#define PM_STOP2			7
#define PM_STANDBY			8
#define PM_SHUTDOWN			9
#define PM_VBAT				10


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// OTHER DEFINES
#define DHTTYPE 				DHT22
/* Calibration at 21.0�C with 8s delay after power up */
#define ADC_OFFSET_FRONT_LEFT 	-163050
#define ADC_OFFSET_FRONT_RIGHT 	86050
#define ADC_OFFSET_REAR_RIGHT 	160110
#define ADC_OFFSET_REAR_LEFT 	-34350
#define ADC_CAL_TEMP			21
#define ADC_SCALE 				1
/*
 * 0.05% FS (10�C) = 50000 (50kg) x 0.0005 = 25g
 * Weight = sum of 4 load cell ==> 100g / 10�C ==> 10 g/�C
 * Temp_Effect = 10x ADC_SCALE
 */
#define	LOADCELL_TEMP_EFFECT	330*4
#define ADC_AVG_NB				32
#define ADC_NB_SAMPLES			32
#define ADC_DROPPED_SAMPLES		16
#define ADC_POWER_UP_MS			600
#define LORA_CORRECT_PACKET		0
#define ACK_NTP_CODE			0x01
#define ACK_PERIOD_CODE			0x02



///////////////////////////////////////////////////////////////////////////////////////////////////////////
// TYPE DEFINITION
typedef struct  {
	uint8_t  gwAckSize;
	uint8_t  gwSNR;
	uint32_t gwDate;
	uint32_t gwTime;
	uint32_t gwReportingPeriod;
} AckData_t;

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// CORTEX-M4 REGISTERS DEFINITION
#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */
#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct */


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// STM32L476 REGISTERS DEFINITION
#if 1
//#define UID_BASE 		0x1FFF7590
#define UID1			(*(volatile uint32_t *) UID_BASE)
#define UID2			(*(volatile uint32_t *) (UID_BASE+0x04))
#define UID3			(*(volatile uint32_t *) (UID_BASE+0x08))
//#define FLASH_SIZE		(*(volatile uint32_t *) (0x1FFF75E0))
//#define PWR_BASE 		0x40007000
#define PWR_CR1			(*(volatile uint32_t *) PWR_BASE)
#define PWR_CR2			(*(volatile uint32_t *) (PWR_BASE+0x04))
#define PWR_CR3			(*(volatile uint32_t *) (PWR_BASE+0x08))
#define PWR_CR4			(*(volatile uint32_t *) (PWR_BASE+0x0C))
#define PWR_SR1			(*(volatile uint32_t *) (PWR_BASE+0x10))
#define PWR_SR2			(*(volatile uint32_t *) (PWR_BASE+0x14))
#define PWR_SCR			(*(volatile uint32_t *) (PWR_BASE+0x18))
#define PWR_PUCRA		(*(volatile uint32_t *) (PWR_BASE+0x20))
#define PWR_PDCRA		(*(volatile uint32_t *) (PWR_BASE+0x24))
#define PWR_PUCRB		(*(volatile uint32_t *) (PWR_BASE+0x28))
#define PWR_PDCRB		(*(volatile uint32_t *) (PWR_BASE+0x2C))
#define PWR_PUCRC		(*(volatile uint32_t *) (PWR_BASE+0x30))
#define PWR_PDCRC		(*(volatile uint32_t *) (PWR_BASE+0x34))
#define PWR_PUCRD		(*(volatile uint32_t *) (PWR_BASE+0x38))
#define PWR_PDCRD		(*(volatile uint32_t *) (PWR_BASE+0x3C))
#define PWR_PUCRE		(*(volatile uint32_t *) (PWR_BASE+0x40))
#define PWR_PDCRE		(*(volatile uint32_t *) (PWR_BASE+0x44))
#define PWR_PUCRF		(*(volatile uint32_t *) (PWR_BASE+0x48))
#define PWR_PDCRF		(*(volatile uint32_t *) (PWR_BASE+0x4C))
#define PWR_PUCRG		(*(volatile uint32_t *) (PWR_BASE+0x50))
#define PWR_PDCRG		(*(volatile uint32_t *) (PWR_BASE+0x54))
#define PWR_PUCRH		(*(volatile uint32_t *) (PWR_BASE+0x58))
#define PWR_PDCRH		(*(volatile uint32_t *) (PWR_BASE+0x5C))
//#define RCC_BASE 		0x40021000
#define RCC_CR			(*(volatile uint32_t *) RCC_BASE)
#define RCC_ICSCR		(*(volatile uint32_t *) (RCC_BASE+0x04))
#define RCC_CFGR		(*(volatile uint32_t *) (RCC_BASE+0x08))
#define RCC_PLLCFGR		(*(volatile uint32_t *) (RCC_BASE+0x0C))
#define RCC_PLLSAI1CFGR	(*(volatile uint32_t *) (RCC_BASE+0x10))
#define RCC_PLLSAI2CFGR	(*(volatile uint32_t *) (RCC_BASE+0x14))
#define RCC_CIER		(*(volatile uint32_t *) (RCC_BASE+0x18))
#define RCC_CIFR		(*(volatile uint32_t *) (RCC_BASE+0x1C))
#define RCC_CICR		(*(volatile uint32_t *) (RCC_BASE+0x20))
#define RCC_AHB1RSTR	(*(volatile uint32_t *) (RCC_BASE+0x28))
#define RCC_AHB2RSTR	(*(volatile uint32_t *) (RCC_BASE+0x2C))
#define RCC_AHB3RSTR	(*(volatile uint32_t *) (RCC_BASE+0x30))
#define RCC_APB1RSTR1	(*(volatile uint32_t *) (RCC_BASE+0x38))
#define RCC_APB1RSTR2	(*(volatile uint32_t *) (RCC_BASE+0x3C))
#define RCC_APB2RSTR	(*(volatile uint32_t *) (RCC_BASE+0x40))
#define RCC_AHB1ENR		(*(volatile uint32_t *) (RCC_BASE+0x48))
#define RCC_AHB2ENR		(*(volatile uint32_t *) (RCC_BASE+0x4C))
#define RCC_AHB3ENR		(*(volatile uint32_t *) (RCC_BASE+0x50))
#define RCC_APB1ENR1	(*(volatile uint32_t *) (RCC_BASE+0x58))
#define RCC_APB1ENR2	(*(volatile uint32_t *) (RCC_BASE+0x5C))
#define RCC_APB2ENR		(*(volatile int32_t *) (RCC_BASE+0x60))
#define RCC_CCIPR		(*(volatile uint32_t *) (RCC_BASE+0x88))
#define RCC_BDCR		(*(volatile uint32_t *) (RCC_BASE+0x90))
#define RCC_CSR			(*(volatile uint32_t *) (RCC_BASE+0x94))
//#define RTC_BASE 		0x40002800
#define RTC_TR			(*(volatile uint32_t *) RTC_BASE)
#define RTC_DR 			(*(volatile uint32_t *) (RTC_BASE+0x04))
#define RTC_CR 			(*(volatile uint32_t *) (RTC_BASE+0x08))
#define RTC_ISR 		(*(volatile uint32_t *) (RTC_BASE+0x0C))
#define RTC_PRER 		(*(volatile uint32_t *) (RTC_BASE+0x10))
#define RTC_WUTR	 	(*(volatile uint32_t *) (RTC_BASE+0x14))
#define RTC_ALARMAR 	(*(volatile uint32_t *) (RTC_BASE+0x1C))
#define RTC_ALARMBR 	(*(volatile uint32_t *) (RTC_BASE+0x20))
#define RTC_WPR 		(*(volatile uint32_t *) (RTC_BASE+0x24))
#define RTC_SSR 		(*(volatile uint32_t *) (RTC_BASE+0x28))
#define RTC_SHITFR 		(*(volatile uint32_t *) (RTC_BASE+0x2C))
//#define EXTI_BASE 		0x40010400
#define EXTI_IMR1		(*(volatile uint32_t *) EXTI_BASE)
#define EXTI_EMR1 		(*(volatile uint32_t *) (EXTI_BASE+0x04))
#define EXTI_SWIER1		(*(volatile uint32_t *) (EXTI_BASE+0x10))
#define EXTI_PR1 		(*(volatile uint32_t *) (EXTI_BASE+0x14))
#define EXTI_IMR2 		(*(volatile uint32_t *) (EXTI_BASE+0x20))
#define EXTI_EMR2 		(*(volatile uint32_t *) (EXTI_BASE+0x24))
#define EXTI_SWIER2		(*(volatile uint32_t *) (EXTI_BASE+0x30))
#define EXTI_PR2 		(*(volatile uint32_t *) (EXTI_BASE+0x34))
//#define TIM5_BASE 		0x40000C00
#define TIM5_CR1		(*(volatile uint32_t *) TIM5_BASE)
#define TIM5_CR2 		(*(volatile uint32_t *) (TIM5_BASE+0x04))
#define TIM5_SCMR 		(*(volatile uint32_t *) (TIM5_BASE+0x08))
#define TIM5_DIER 		(*(volatile uint32_t *) (TIM5_BASE+0x0C))
#define TIM5_SR 		(*(volatile uint32_t *) (TIM5_BASE+0x10))
#define TIM5_EGR	 	(*(volatile uint32_t *) (TIM5_BASE+0x14))
#define TIM5_CCMR1	 	(*(volatile uint32_t *) (TIM5_BASE+0x18))
#define TIM5_CCMR2	 	(*(volatile uint32_t *) (TIM5_BASE+0x1C))
#define TIM5_CCER	 	(*(volatile uint32_t *) (TIM5_BASE+0x20))
#define TIM5_CNT 		(*(volatile uint32_t *) (TIM5_BASE+0x24))
#define TIM5_PSC 		(*(volatile uint32_t *) (TIM5_BASE+0x28))
#define TIM5_ARR 		(*(volatile uint32_t *) (TIM5_BASE+0x2C))
#define TIM5_CCR1 		(*(volatile uint32_t *) (TIM5_BASE+0x34))
#define TIM5_CCR2 		(*(volatile uint32_t *) (TIM5_BASE+0x38))
#define TIM5_CCR3 		(*(volatile uint32_t *) (TIM5_BASE+0x3C))
#define TIM5_CCR4 		(*(volatile uint32_t *) (TIM5_BASE+0x40))
#define TIM5_DCR 		(*(volatile uint32_t *) (TIM5_BASE+0x48))
#define TIM5_DMAR 		(*(volatile uint32_t *) (TIM5_BASE+0x4C))
#define TIM5_OR1 		(*(volatile uint32_t *) (TIM5_BASE+0x50))
#define TIM5_OR 		(*(volatile uint32_t *) (TIM5_BASE+0x60))
#define SYST_CSR 		(*(volatile uint32_t *) (0xE000E010))
#define SYST_RVR 		(*(volatile uint32_t *) (0xE000E014))
#define SYST_CVR 		(*(volatile uint32_t *) (0xE000E018))
#endif /* 0 */

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// PROTOTYPES
uint32_t initDHT(void);
uint32_t initADC(void);
uint32_t initLoRa(void);
uint32_t measureTempHum(float* T, float* H);
uint32_t measureHX711(float* Weight);
uint32_t adjustWeight(float* Weight, float T);
uint32_t handleAckData(uint8_t *AckMessage, uint8_t *AckSize, AckData_t *gwAckData);
uint32_t bcd_to_decimal(uint32_t bcd);
uint32_t decimal_to_bcd(uint32_t d);
uint32_t initRTC(void);
uint32_t setRTCDateTime(tm* time);
uint32_t getRTCDateTime(tm* time);
uint32_t configureClock(void);
uint32_t enterLowPower(uint32_t mode, uint32_t duration);
uint32_t addDateTime(tm* endtime, tm starttime, uint32_t duration);
uint32_t setAlarm(tm alrm);
void Error_Handler(uint32_t error_code);

#endif /* HIVETRONIC_H_ */
