/*
 * power.h
 *
 *  Created on: 18 janv. 2017
 *      Author: Arnaud ROSAY
 */

#ifndef POWER_H_
#define POWER_H_

//#include <stdint.h>
#include "Arduino.h"
#include "error.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// PROTOTYPES

uint32_t delay_WFI(uint32_t duration_ms);

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

#endif /* POWER_H_ */
