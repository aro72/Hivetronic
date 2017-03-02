
/*
 *****************************************************************************
    Hive monitor program using HX711 and LoRa transceiver

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the program.  If not, see <http://www.gnu.org/licenses/>.

 *****************************************************************************
 */


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// INCLUDES
#include "hivetronic.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL CONFIGURATIONS
#define DEBUG_HIVETRONIC
#define DEBUG_HX711
#define LORA_ENABLED
#define FAST_REPORTING

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES

DHT dht(DHTPIN, DHTTYPE);
HX711 adcFrontLeft, adcFrontRight, adcRearLeft, adcRearRight;
int loraMode = LORAMODE;
uint32_t seq = 0;
uint32_t inactiveDuration = LORA_REPORTING_PERIOD-PROCESSING_DURATION;
uint32_t WakeUpFlag=0, PullUpGPIOA;
ADC_HandleTypeDef hadc1;
RTC_HandleTypeDef hrtc;
float Temp = NAN;
float Hum = NAN;
Weight_t Weight;
float Vbat=0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup function

void setup() {

	// Open serial communications and wait for port to open:
	Serial.begin(115200);
	delay(10);
	// Print a start message
#ifdef DEBUG_HIVETRONIC
	if (WakeUpFlag==0) {
		printf("... Wake-up from Shutdown\r\n");
	} else {
		printf("... Wake-up from Standby\r\n");
	}
	printf("\r\n\r\n\n\r");
	printf("////////////////////////////////////\r\n");
	printf("\r\n");
	printf("Hive monitor using HX711 and LoRa\r\n");
	printf("ARM (STM32)\r\n");
#endif /* DEBUG_HIVETRONIC */
	WakeUp();
  	initGPIO();
	initRTC();
	initDHT();
  	MX_ADC1_Init();
	initADC();

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Loop function

void loop(void)
{
	uint32_t r_size;
	uint16_t VbatADC;
	int ret;
	uint8_t message[100]={0}, AckMessage[100]={0};
	AckData_t gwAckData;
	uint8_t AckSize;
	tm time;
	char cDateTime[26];

	/* measure weight, temperature/humidity, Vbat */
	measureTempHum(&Temp, &Hum); /* must be done at least 1s after power-up */
	measureVbat(&VbatADC); /* VbatADC = Vbat/3 */
	Vbat = 3*3.3*VbatADC/4095; /* 12-bit resolution, Vref+=3.3V */
	measureHX711(&Weight);

#ifdef DEBUG_HIVETRONIC
	printf("\r\n");
	printf("-------------\r\n");
#endif /* DEBUG_HIVETRONIC */
	/*
	* remove temp adjustment as long as calibration and 
	* recording of a fixed weight at different temperature is not available
	*/
	// adjustWeight(&Weight, Temp);
	getRTCDateTime(&time);
	sprintf(cDateTime, "%02d/%02d/%4d-%02d:%02d:%02d",
			time.tm_mday,
			time.tm_mon+1,
			(time.tm_year+1900),
			time.tm_hour,
			time.tm_min,
			time.tm_sec);
   	String strTime(cDateTime);
   	String strVbat(Vbat,3);
   	String strT(Temp, 2);
   	String strH(Hum, 2);
   	String strWeightFL(Weight.FrontLeft, 3);
   	String strWeightFR(Weight.FrontRight, 3);
   	String strWeightRL(Weight.RearLeft, 3);
   	String strWeightRR(Weight.RearRight, 3);
   	String strWeightTotal(Weight.Total, 3);
   	String messageData;

    messageData = strTime + " - Vbat=" + strVbat + " - T=" + strT + " - H=" + strH + " - FL=" + strWeightFL + " - FR=" + strWeightFR + " - RL=" + strWeightRL + " - RR=" + strWeightRR + " - W=" + strWeightTotal;
   	r_size = strlen(messageData.c_str());
    	for (uint32_t i = 0; i < r_size; i++) {
			message[i] = (uint8_t) messageData.c_str()[i];
		}
    	message[r_size]='\r';
    	message[r_size+1]='\n';
#ifdef DEBUG_HIVETRONIC
    	printf("\nSending Message (length=%d) : %s\r\n", (int)r_size,message);
#endif /* DEBUG_HIVETRONIC*/

#ifdef LORA_ENABLED
	initLoRa();
	// FIX THIS - Is it really needed ???
	// sx1272.CarrierSense();
	// END OF FIX THIS
	sx1272.setPacketType(PKT_TYPE_DATA);
    ret = sx1272.sendPacketTimeoutACK(DEFAULT_DEST_ADDR, message, r_size);
    // Power OFF the module
	sx1272.OFF();
#ifdef DEBUG_HIVETRONIC
    printf("Packet sent - state %d\r\n", ret);
    if (ret == 3)
		printf("No Ack!\r\n");
	if (ret == 0)
		printf("Ack received from gateway!\r\n");
#endif /* DEBUG_HIVETRONIC*/
	if (ret==0) {
		sx1272.getAckPacket(DEFAULT_DEST_ADDR, AckMessage, &AckSize);
		handleAckData(AckMessage, &AckSize, &gwAckData);
	}
#endif
	enterLowPower(LOW_POWER_MODE, inactiveDuration);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS IMPLEMENTATION
void GotoLowPower(uint32_t LowPowerMode) {
	if (LowPowerMode==PM_SLEEP) {
		uint32_t systick_csr;
		/* Stop TIM5 and SysTick */
		TIM5_CR1 &= 0xFFFFFFFE;
		systick_csr = SYST_CSR;
		SYST_CSR &=0xFFFFFFFD;
	   	__WFI();
		/* Stop TIM5 and SysTick */
		TIM5_CR1 |= 0x01;
		SYST_CSR = systick_csr;
	}
	else {
		RCC_ClkInitTypeDef LowPowerClkConfig;
		RCC_OscInitTypeDef LowPowerOscConfig;
		/* Stop SysTick */
		HAL_SuspendTick();
		/* Set new clocks configuration to enable low power mode */
		LowPowerOscConfig.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
		LowPowerOscConfig.LSEState = RCC_LSE_ON;
		LowPowerOscConfig.MSIState = RCC_MSI_ON;
		LowPowerOscConfig.MSIClockRange = RCC_MSIRANGE_4; /* MSI at 1MHz */
		LowPowerOscConfig.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
		LowPowerOscConfig.PLL.PLLState = RCC_PLL_ON;
		LowPowerClkConfig.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
		LowPowerClkConfig.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
		LowPowerClkConfig.AHBCLKDivider = RCC_SYSCLK_DIV1;
		LowPowerClkConfig.APB1CLKDivider = RCC_HCLK_DIV1;
		LowPowerClkConfig.APB2CLKDivider = RCC_HCLK_DIV1;
		HAL_RCC_OscConfig(&LowPowerOscConfig); /* Configure clocks */
		HAL_RCC_ClockConfig(&LowPowerClkConfig, FLASH_LATENCY_0); /* Switch to MSI */
		/* Disable PLL */
		LowPowerOscConfig.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
		LowPowerOscConfig.LSEState = RCC_LSE_ON;
		LowPowerOscConfig.MSIState = RCC_MSI_ON;
		LowPowerOscConfig.MSIClockRange = RCC_MSIRANGE_4; /* MSI at 1MHz */
		LowPowerOscConfig.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
		LowPowerOscConfig.PLL.PLLState = RCC_PLL_OFF;
		HAL_RCC_OscConfig(&LowPowerOscConfig); /* Configure clocks */

		/* Enter low power mode */
		WakeUpFlag = 0;
		switch (LowPowerMode) {
		case PM_STOP2:
			HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
			break;
		case PM_STANDBY:
			HAL_PWR_EnterSTANDBYMode();
			for(;;);
			break;
		case PM_SHUTDOWN:
			HAL_PWREx_EnterSHUTDOWNMode();
			for(;;);
			break;
		default:
			Error_Handler(ERROR_PMMODE);
			break;
		}

		/* Switch back to main regulator */
		//HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
		/* Restore clocks configuration */
		SystemClock_Config();
		/* Restart SysTick and TIM5 */
		HAL_ResumeTick();
	}
}
uint32_t enterLowPower(uint32_t mode, uint32_t duration) {
	tm time, alarm;
	getRTCDateTime(&time);
	getNextAlarm(&alarm, time);
	//addDateTime(&alarm, time, duration);
#ifdef DEBUG_HIVETRONIC
	printf("Alarm set: %02d:%02d:%02d\r\n", alarm.tm_hour, alarm.tm_min, alarm.tm_sec);
	printf("Low Power entry ...\r\n");
#endif /* DEBUG_HIVETRONIC */
	//delay(10);
	setAlarm(alarm);
	GotoLowPower(mode);
#ifdef DEBUG_HIVETRONIC
	printf("... Low Power exit\r\n");
#endif /* DEBUG_HIVETRONIC */
	return NO_ERROR;
}

uint32_t getNextAlarm(tm* alarm, tm time) {
	daylight_t daylight;
	tm previous_alarm;
	uint32_t reporting_period;
	/* Get sunrise/sunset depending on month */
	switch (time.tm_mon) {
		case 0: /* january */
			daylight.sunrise.hour 	=	8;
			daylight.sunrise.min 	=	30;
			daylight.sunset.hour 	=	18;
			daylight.sunset.min 	= 	00;
			break;
		case 1: /* february */
			daylight.sunrise.hour 	=	7;
			daylight.sunrise.min 	=	40;
			daylight.sunset.hour 	= 	18;
			daylight.sunset.min 	= 	40;
			break;
		case 2: /* march */
			daylight.sunrise.hour 	=	7;
			daylight.sunrise.min 	= 	00;
			daylight.sunset.hour 	= 	20;
			daylight.sunset.min 	= 	30;
			break;
		case 3: /* april */
			daylight.sunrise.hour 	=	6;
			daylight.sunrise.min 	= 	40;
			daylight.sunset.hour 	= 	21;
			daylight.sunset.min 	= 	10;
			break;
		case 4: /* may */
			daylight.sunrise.hour 	= 	6;
			daylight.sunrise.min 	= 	00;
			daylight.sunset.hour 	= 	21;
			daylight.sunset.min 	= 	50;
			break;
		case 5: /* june */
			daylight.sunrise.hour 	= 	6;
			daylight.sunrise.min 	= 	00;
			daylight.sunset.hour 	= 	22;
			daylight.sunset.min 	= 	00;
			break;
		case 6: /* july */
			daylight.sunrise.hour 	= 	6;
			daylight.sunrise.min 	= 	00;
			daylight.sunset.hour 	= 	22;
			daylight.sunset.min 	= 	00;
			break;
		case 7: /* august */
			daylight.sunrise.hour 	= 	6;
			daylight.sunrise.min 	= 	30;
			daylight.sunset.hour 	= 	21;
			daylight.sunset.min 	= 	30;
			break;
		case 8: /* september */
			daylight.sunrise.hour 	= 	7;
			daylight.sunrise.min 	= 	20;
			daylight.sunset.hour 	= 	20;
			daylight.sunset.min 	= 	40;
			break;
		case 9: /* october */
			daylight.sunrise.hour 	= 	7;
			daylight.sunrise.min 	= 	40;
			daylight.sunset.hour 	= 	19;
			daylight.sunset.min 	= 	40;
			break;
		case 10: /* november */
			daylight.sunrise.hour 	= 	7;
			daylight.sunrise.min 	= 	40;
			daylight.sunset.hour 	= 	17;
			daylight.sunset.min 	= 	40;
			break;
		case 11: /* december */
			daylight.sunrise.hour 	= 	8;
			daylight.sunrise.min 	= 	30;
			daylight.sunset.hour 	= 	17;
			daylight.sunset.min 	= 	00;
			break;
		default:
			Error_Handler(ERROR_BAD_DATE);
	}

	/* Select reporting period depending on time and sunrise/sunset */
	reporting_period = NIGHTTIME_REPORTING;
	if ((time.tm_hour>daylight.sunrise.hour) and (time.tm_min>daylight.sunrise.min)) {
		/* sunrise already happened */
		if ((time.tm_hour<daylight.sunset.hour) and (time.tm_min<daylight.sunset.min)) {
			/* not yet sunset: stil day time */
			reporting_period = DAYTIME_REPORTING;
		}
	} 

	/* calculate next alarm */
#ifndef FAST_REPORTING
	memcpy(&previous_alarm, &time, sizeof(tm));
	previous_alarm.tm_sec = 0;
	addDateTime(alarm, previous_alarm, reporting_period);
	alarm->tm_min = alarm->tm_min | WAKEUP_ALIGN;
#else /* FAST_REPORTING*/
	addDateTime(alarm, time, reporting_period);
#endif /* FAST_REPORTING */
	return NO_ERROR; 
}

uint32_t addDateTime(tm* endtime, tm starttime, uint32_t duration) {
	/* duration is in seconds */
	uint32_t hour, min, sec, carry;
	hour = (uint32_t) (duration/3600);
	min = (uint32_t) ((duration/60)%60);
	sec = (uint32_t) (duration%60);
	endtime->tm_sec = starttime.tm_sec + sec;
	carry = 0;
	if (endtime->tm_sec>59) {
		// add one minute
		endtime->tm_sec = endtime->tm_sec-60;
		carry=1;
	}
	endtime->tm_min = starttime.tm_min + min + carry;
	carry = 0;
	if (endtime->tm_min>59) {
		// add one minute
		endtime->tm_min = endtime->tm_min-60;
		carry=1;
	}
	endtime->tm_hour = starttime.tm_hour + hour + carry;
	if (endtime->tm_hour>23) {
		// add one minute
		endtime->tm_hour = endtime->tm_hour-24;
		// carry=1;
	}
	return NO_ERROR;
}

uint32_t setAlarm(tm alrm) {
	uint8_t st, su, mnt, mnu, ht, hu;
	//uint8_t dt, du, mt, mu, wdu, yt, yu;
	st = (uint8_t) decimal_to_bcd((uint8_t)(alrm.tm_sec/10));
	su = (uint8_t) decimal_to_bcd((uint8_t)(alrm.tm_sec - 10*st));
	mnt = (uint8_t) decimal_to_bcd((uint8_t)(alrm.tm_min/10));
	mnu = (uint8_t)	decimal_to_bcd((uint8_t)(alrm.tm_min - 10*mnt));
	ht = (uint8_t) decimal_to_bcd((uint8_t)(alrm.tm_hour/10));
	hu = (uint8_t) decimal_to_bcd((uint8_t)(alrm.tm_hour - 10*ht));	// remove write protection
	RTC_WPR = 0xCA;
	RTC_WPR = 0x53;
	while ((RTC_ISR&0x00000001)!=0x00000001) {
		// wait for ALARAWF set
	}
	// clear ALRAE to disable Alarm A
	RTC_CR &= ~0x00100100;
	// enable RTC_ALARM output: OSEL=0b01
	RTC_CR |= 0x00201000;
	// clear ALARAF flag
	RTC_ISR &= ~0x00000100;
	// set alarm hour:min:sec
	RTC_ALARMAR = ht<<20 | hu<<16 | mnt<<12 | mnu<<8 | st<<4 | su;
	// date mask doesn't care
	RTC_ALARMAR |= 0x80000000;
	// Configure EXTI line 18 in Rising Edge as RTC Alarm interrupt
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN5_HIGH); /* Wake-up 5 = PC5 */
	//HAL_PWREx_DisableInternalWakeUpLine();
	//__HAL_RTC_ALARM_EXTI_CLEAR_FLAG();
	//__HAL_RTC_ALARM_EXTI_ENABLE_RISING_EDGE();
	//__HAL_RTC_ALARM_EXTI_ENABLE_IT();
	// Enable RTC Alarm in NVIC
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
	// set ALRAE to enable Alarm A
	RTC_CR |= 0x00000100;
	// reactivate write protection
	RTC_WPR = 0xDE;
	RTC_WPR = 0xAD;
	return NO_ERROR;
}


uint32_t setRTCDateTime(tm time) {
#ifndef RTC_HAL
	uint8_t st, su, mnt, mnu, ht, hu, dt, du, mt, mu, wdu, yt, yu;
	// remove write protection
	RTC_WPR = 0xCA;
	RTC_WPR = 0x53,
	delay(50);
	// Set INIT bit to 1 in the RTC_ISR register to enter initialization mode
	RTC_ISR |= 0x80;
	// Poll INITF bit of in the RTC_ISR register.
	while((RTC_ISR & 0x40)!=0x40) {
	    // The initialization phase mode is entered when INITF is set to 1 (takes 2RTCCLK)
	}

	RTC_PRER = 0x007F00FF;

	// Load the initial time and date values in the shadow registers (RTC_TR and RTC_DR)
	st = (uint8_t) decimal_to_bcd((uint8_t)(time.tm_sec/10));
	su = (uint8_t) decimal_to_bcd((uint8_t)(time.tm_sec - 10*st));
	mnt = (uint8_t) decimal_to_bcd((uint8_t)(time.tm_min/10));
	mnu = (uint8_t)	decimal_to_bcd((uint8_t)(time.tm_min - 10*mnt));
	ht = (uint8_t) decimal_to_bcd((uint8_t)(time.tm_hour/10));
	hu = (uint8_t) decimal_to_bcd((uint8_t)(time.tm_hour - 10*ht));
	dt = (uint8_t) decimal_to_bcd((uint8_t)(time.tm_mday/10));
	du = (uint8_t) decimal_to_bcd((uint8_t)(time.tm_mday - 10*dt));
	mt = (uint8_t) decimal_to_bcd((uint8_t)((time.tm_mon+1)/10)); /* need to correct month (from 0 to 11) */
	mu = (uint8_t) decimal_to_bcd((uint8_t)((time.tm_mon+1) - 10*mt));
	wdu = (uint8_t) decimal_to_bcd((uint8_t)(time.tm_wday));
	yt = (uint8_t) decimal_to_bcd((uint8_t)(time.tm_year/10));
	yu = (uint8_t) decimal_to_bcd((uint8_t)(time.tm_year - 10*yt));
	// printf("set date : %d%d-%d%d-%d%d\r\n", dt, du, mt, mu, yt, yu);
  	if (dt<3) {
  		RTC_TR = ht<<20 | hu<<16 | mnt<<12 | mnu<<8 | st<<4 | su;
  		/* FIX THIS: month not handle correctly */
		RTC_DR = yt<<20 | yu<<16 | wdu<<13 | mt << 12 | mu<<8 | dt<<4 | du;
		// configure the time format (24 hours) through the FMT bit in the RTC_CR register
		RTC_CR &= 0xFFFFFFBF;
	} else {
#ifdef DEBUG_HIVETRONIC
		printf("\tWrong date/time\r\n");
#endif /* DEBUG_HIVETRONIC*/
	}
	// Exit the initialization mode by clearing the INIT bit
	RTC_ISR &= 0xFFFFFF7F;
	// reactivate write protection
	RTC_WPR = 0xDE;
	RTC_WPR = 0xAD;
	delay(50);
	return NO_ERROR;
#else /* !RTC_HAL */
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	sTime.Seconds = time.tm_sec;
	sTime.Minutes = time.tm_min;
	sTime.Hours = time.tm_hour;
	sDate.Date = time.tm_mday;
	sDate.Month = time.tm_mon + 1;
	sDate.Year = time.tm_year - 100;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
	  Error_Handler(ERROR_SETDATE);
	}
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
	  Error_Handler(ERROR_SETDATE);
	}
#endif /* !RTC_HAL */
#ifdef DEBUG_HIVETRONIC
		printf("Set Time/Date\r\n");
		printf("RTC_TR: %ld\r\n", RTC_TR);
		printf("RTC_DR: %ld\r\n", RTC_DR);
#endif /* DEBUG_HIVETRONIC*/
	return NO_ERROR;
}

uint32_t getRTCDateTime(tm* time) {
#ifndef RTC_HAL
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
	  Error_Handler(ERROR_GETDATE);
	}
	if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
	  Error_Handler(ERROR_GETTIME);
	}
	time->tm_sec = sTime.Seconds;
	time->tm_min = sTime.Minutes;
	time->tm_hour = sTime.Hours;
	time->tm_mday = sDate.Date;
	time->tm_mon = sDate.Month-1;
	time->tm_year = sDate.Year+100;
	time->tm_wday = 1;
	time->tm_yday = 1;
	time->tm_isdst = 0;
#ifdef DEBUG
	char charDateTime[100];
	sprintf(charDateTime, "Get Date/Time - %02d/%02d/%4d-%02d:%02d:%02d",
			time->tm_mday,
			time->tm_mon+1,
			(time->tm_year+1900),
			time->tm_hour,
			time->tm_min,
			time->tm_sec);
	printf("%s\n\r", charDateTime);
#endif /* DEBUG */
#else /* RTC_HAL */
	uint32_t tr, dr;
	uint32_t st, su, mnt, mnu, ht, hu, dt, du, mt, mu, wdu, yt, yu;
	tr = RTC_TR;
	dr = RTC_DR;
	su = (tr & 0x0000000F);
	st = (tr & 0x000000F0) >> 4;
	mnu = (tr & 0x00000F00) >> 8;
	mnt = (tr & 0x0000F000) >> 12;
	hu = (tr & 0x000F0000) >> 16;
	ht = (tr & 0x00F00000) >> 20;
	time->tm_sec = 10*st + su;
	time->tm_min = 10*mnt + mnu;
	time->tm_hour = 10*ht + hu;
	du = (dr & 0x0000000F);
	dt = (dr & 0x00000030) >> 4;
	mu = (dr & 0x00000F00) >> 8;
	mt = (dr & 0x00001000) >> 12;
	//wdu = (dr & 0x0000E000) >> 3;
	yu = (dr & 0x000F0000) >> 16;
	yt = (dr & 0x00F00000) >> 20;
	time->tm_mday = 10*dt + du;
	time->tm_mon = 10*mt + mu;
	time->tm_year = 10*yt + yu;
#endif /* RTC_HAL */
	return NO_ERROR;

}

uint32_t bcd_to_decimal(uint32_t bcd) {
    return ((bcd & 0xf0) >> 4) * 10 + (bcd & 0x0f);
}
uint32_t decimal_to_bcd(uint32_t d) {
    return ((d / 10) << 4) + (d % 10);
}

uint32_t handleAckData(uint8_t *AckMessage, uint8_t *AckSize, AckData_t *gwAckData) {
	uint32_t ret=ERROR_INCORRECT_ACK;
	uint8_t snr;
	uint8_t opcode;
	uint32_t i;
	tm time;
	if (AckMessage[0]==LORA_CORRECT_PACKET) {
		gwAckData->gwReportingPeriod = 0xFFFFFFFF;
		gwAckData->gwDate= 0xFFFFFFFF;
		gwAckData->gwTime= 0xFFFFFFFF;
		snr = AckMessage[1];
		// extract and decode SNR measured by the gateway
    	if( snr & 0x80 ) // The SNR sign bit is 1
   		{
        	// Invert and divide by 4
        	snr = ( ( ~snr + 1 ) & 0xFF ) >> 2;
        	gwAckData->gwSNR = -snr;
    	} else {
	        // Divide by 4
        	gwAckData->gwSNR = ( snr & 0xFF ) >> 2;
    	}
    	if (*AckSize>2) {
	    	i=1;
    		do{
    		    i++;
    			opcode=AckMessage[i++];
    			if ((opcode&ACK_NTP_CODE)==ACK_NTP_CODE) {
    				// need to update date and time
    				time.tm_sec=AckMessage[i];
    				time.tm_min=AckMessage[i+1];
    				time.tm_hour=AckMessage[i+2];
    				time.tm_mday=AckMessage[i+3];
    				time.tm_mon=AckMessage[i+4];
    				time.tm_year=AckMessage[i+5]-100;
    				time.tm_wday=AckMessage[i+6];
    				i+=7;
    				setRTCDateTime(time);
    			}
			} while (i<*AckSize);
    	}
    	ret=NO_ERROR;
    }
	return ret;
}

uint32_t measureHX711(Weight_t* Weight) {
	int32_t FrontLeftTab[ADC_NB_SAMPLES];
	int32_t FrontRightTab[ADC_NB_SAMPLES];
	int32_t RearLeftTab[ADC_NB_SAMPLES];
	int32_t RearRightTab[ADC_NB_SAMPLES];
	uint32_t i, start_idx, stop_idx;
	float FrontLeft_avg=0, FrontRight_avg=0, RearLeft_avg=0, RearRight_avg=0;

	// Read ADC for all load cells
	for (i=0;i<ADC_NB_SAMPLES;i++) {
		FrontLeftTab[i]  = (int32_t)  adcFrontLeft.get_units();
		if (i==(ADC_NB_SAMPLES-1)) {
			adcFrontLeft.power_down();
		}
		FrontRightTab[i] = (int32_t) adcFrontRight.get_units();
		if (i==(ADC_NB_SAMPLES-1)) {
			adcFrontRight.power_down();
		}
		RearLeftTab[i]   = (int32_t) adcRearLeft.get_units();
		if (i==(ADC_NB_SAMPLES-1)) {
			adcRearLeft.power_down();
		}
		RearRightTab[i]  = (int32_t) adcRearRight.get_units();
		if (i==(ADC_NB_SAMPLES-1)) {
			adcRearRight.power_down();
		} else {
			delay(100);
		}

	}

	// Power down all ADC to decrease power consumption
	/*
	adcFrontLeft.power_down();
	adcFrontRight.power_down();
	adcRearLeft.power_down();
	adcRearRight.power_down();
	*/

#ifdef DEBUG_HX711
	// Display unsorted tables
	printf("\n\rFrontLeft :\t");
	for (i=0;i<ADC_NB_SAMPLES;i++) {
		printf("  %ld",FrontLeftTab[i]);
	}
	printf("\n\rFrontRight :\t");
	for (i=0;i<ADC_NB_SAMPLES;i++) {
		printf("  %ld",FrontRightTab[i]);
	}
	printf("\n\rRearLeft :\t");
	for (i=0;i<ADC_NB_SAMPLES;i++) {
		printf("  %ld",RearLeftTab[i]);
	}
	printf("\n\rRearRight :\t");
	for (i=0;i<ADC_NB_SAMPLES;i++) {
		printf("  %ld",RearRightTab[i]);
	}
	printf("\n\r");
#endif /* DEBUG_HX711 */

	// Sort tables
	merge_sort(FrontLeftTab, ADC_NB_SAMPLES);
	merge_sort(FrontRightTab, ADC_NB_SAMPLES);
	merge_sort(RearLeftTab, ADC_NB_SAMPLES);
	merge_sort(RearRightTab, ADC_NB_SAMPLES);

#ifdef DEBUG_HX711
	// Display sorted tables
	printf("\n\rFrontLeft :\t");
	for (i=0;i<ADC_NB_SAMPLES;i++) {
		printf("  %ld",FrontLeftTab[i]);
	}
	printf("\n\rFrontRight :\t");
	for (i=0;i<ADC_NB_SAMPLES;i++) {
		printf("  %ld",FrontRightTab[i]);
	}
	printf("\n\rRearLeft :\t");
	for (i=0;i<ADC_NB_SAMPLES;i++) {
		printf("  %ld",RearLeftTab[i]);
	}
	printf("\n\rRearRight :\t");
	for (i=0;i<ADC_NB_SAMPLES;i++) {
		printf("  %ld",RearRightTab[i]);
	}
	printf("\n\r");
#endif /* DEBUG_HX711 */

	// Calculate average by removing first quarter and last quarter of sorted tables
	start_idx = ADC_DROPPED_SAMPLES/2;
	stop_idx = ADC_NB_SAMPLES - (ADC_DROPPED_SAMPLES/2);
	for (i=start_idx;i<stop_idx;i++) {
			FrontLeft_avg += FrontLeftTab[i];
			FrontRight_avg += FrontRightTab[i];
			RearLeft_avg += RearLeftTab[i];
			RearRight_avg += RearRightTab[i];
	}
	FrontLeft_avg /= ADC_NB_SAMPLES-ADC_DROPPED_SAMPLES;
	FrontRight_avg /= ADC_NB_SAMPLES-ADC_DROPPED_SAMPLES;
	RearLeft_avg /= ADC_NB_SAMPLES-ADC_DROPPED_SAMPLES;
	RearRight_avg /= ADC_NB_SAMPLES-ADC_DROPPED_SAMPLES;
#ifdef DEBUG_HX711
	printf("Average: \t%.0f\t%.0f\t%.0f\t%.0f\r\n", FrontLeft_avg, FrontRight_avg, RearLeft_avg, RearRight_avg);
#endif /* DEBUG_HX711 */

	// Update Structure
	Weight->FrontLeft = FrontLeft_avg;
	Weight->FrontRight = FrontRight_avg;
	Weight->RearLeft = RearLeft_avg;
	Weight->RearRight = RearRight_avg;
	Weight->Total = FrontLeft_avg + FrontRight_avg + RearLeft_avg + RearRight_avg;
#ifdef DEBUG_HIVETRONIC_N
	printf("Weight: %.0f\r\n", Weight->Total);
#endif /* DEBUG_HIVETRONIC */

	return NO_ERROR;
}

uint32_t adjustWeight(Weight_t* Weight, float T) {
	uint32_t ret=NO_ERROR;
#if 0
	float deltaTemp, deltaWeight;
	deltaTemp = T-ADC_CAL_TEMP;
	deltaWeight = LOADCELL_TEMP_EFFECT*deltaTemp;
#ifdef DEBUG_HIVETRONIC
	printf("Weight before adjustment: %.0f\r\n", *Weight);
	printf("Weight after adjustment: %.0f\r\n", *Weight-deltaWeight);
#endif /* DEBUG_HIVETRONIC */
	*Weight = *Weight - deltaWeight;
#endif /* #if 0 */
	return ret;
}

uint32_t measureTempHum(float* T, float* H) {
	float t=0, h=0;
	uint32_t ret=NO_ERROR;
	// Reading temperature or humidity takes about 250 milliseconds!
	// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
	h = dht.readHumidity();
	// Read temperature as Celsius (the default)
	t = dht.readTemperature();
	// Check if any reads failed and exit early (to try again).
	if (isnan(h) || isnan(t)) {
#ifdef DEBUG_HIVETRONIC
		printf("Failed to read from DHT sensor!\r\n");
#endif /* DEBUG_HIVETRONIC*/
		h=0;
		t=0;
		ret=ERROR_DHT_FAILED;
	}
	*T = t;
	*H = h;
#ifdef DEBUG_HIVETRONIC
	printf("DHT22: %3.1f°C - %3.1f%%\r\n", *T, *H);
#endif /* DEBUG_HIVETRONIC*/
	return ret;
}

uint32_t initDHT(void) {
	Wire.begin();
	dht.begin();
	return NO_ERROR;
}

uint32_t initRTC(void) {
#ifdef RTC_HAL
	/* if RTC is not enabled, then enable/initialize RTC */
	if ((RCC_BDCR&0x00008000)==0) {
		RTC_TimeTypeDef sTime;
		RTC_DateTypeDef sDate;

		/**Initialize RTC Only
	  	*/
		hrtc.Instance = RTC;
		hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
		/* Prescaler values are set to get 1Hz clock from LSE (32.768KHz) */
		hrtc.Init.AsynchPrediv = 127;
		hrtc.Init.SynchPrediv = 255;
		hrtc.Init.OutPut = RTC_OUTPUT_ALARMA;
		hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
		hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
		hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
		if (HAL_RTC_Init(&hrtc) != HAL_OK)
		{
	  	Error_Handler(ERROR_INITRTC);
		}

		// set SYSCFGEN
		(*(volatile uint32_t *) (RCC_BASE+0x4C)) |= 0x01;
		// set PWREN
		(*(volatile uint32_t *) (RCC_BASE+0x58)) |= 0x10000000;
		delay(100);
		// set DBP in PWR_CR1
		(*(volatile uint32_t *) PWR_BASE) |= 0x0100;
	    // set RTCSEL=LSE and set LSEON in RCC_BDCR
		(*(volatile uint32_t *) (RCC_BASE+0x90)) |= 0x000000101;
	    // set RTCEN in RCC_BDCR
		(*(volatile uint32_t *) (RCC_BASE+0x90)) |= 0x00008000;

		/* default date : May 16, 2016 - 12:00:00 */
		sTime.Hours = 0x12;
		sTime.Minutes = 0x0;
		sTime.Seconds = 0x0;
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
		{
	  	Error_Handler(ERROR_SETTIME);
		}
		sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
		sDate.Month = RTC_MONTH_MAY;
		sDate.Date = 0x16;
		sDate.Year = 0x16;

		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
		{
	  	Error_Handler(ERROR_SETDATE);
		}
	}
#else /* RTC_HAL */
	/* if RTC is not enabled, then enable/initialize RTC */
	if ((RCC_BDCR&0x00008000)==0) {
		uint32_t value;
		tm time;    time.tm_sec=0;
    	time.tm_min=0;
    	time.tm_hour=12;
    	time.tm_mday=16;
    	time.tm_mon=5;
    	time.tm_year=16;
    	time.tm_wday=6;
    	hrtc.Instance = RTC;
      	// set SYSCFGEN
    	RCC_APB2ENR |= 0x01;
    	// set PWREN
    	RCC_APB1ENR1 |= 0x10000000;
    	delay(100);
    	// set DBP
    	PWR_CR1 |= 0x0100;
    	// set RTCSEL=LSE and set LSEON
    	RCC_BDCR |= 0x000000101;
    	// set RTCEN
    	RCC_BDCR |= 0x00008000;
    	// update date and time in RTC if DR contains the reset value
		if (RTC_DR==0x00002101) {
			setRTCDateTime(time);
		}
	}

#endif /* RTC_HAL */

	return NO_ERROR;
}


uint32_t initADC(void) {
	// Notes about HX711
	// RATE = 0 --> 10 samples/second
	// internal oscillator used --> XI to GND, XO not connected
	// internal voltage regulator used
	// Power consumption: <1.5mA in normal mode ; <1uA in power down mode
	// Might need temperature compensation:
	// 		Offset drift:	+/- 6nV/°C
	//		Gain drift:		+/- 5ppm/°C
	int32_t adc[4];
#ifdef DEBUG_HIVETRONIC_N
	printf("initializing HX711 converters ...\r\n");
#endif /* DEBUG_HIVETRONIC*/
	// initialize each HX711
	adcFrontRight.begin(HX711_DOUT1, HX711_PD_SCK1, HX711_GAIN);
	adcFrontLeft.begin(HX711_DOUT2, HX711_PD_SCK2, HX711_GAIN);
	adcRearLeft.begin(HX711_DOUT3, HX711_PD_SCK3, HX711_GAIN);
	adcRearRight.begin(HX711_DOUT4, HX711_PD_SCK4, HX711_GAIN);
	// Power up all ADC
	adcFrontLeft.power_up();
	adcFrontRight.power_up();
	adcRearLeft.power_up();
	adcRearRight.power_up();
	// Settling time is 400ms at 10Hz to get a stable output
	// As first readings are executed to configure next readings, no need to have stable output
	// During initial phase of the project, it is anyway better to wait for stable output
	delay(400);
	// Set offset
	adcFrontLeft.set_offset(ADC_OFFSET_FRONT_LEFT);
	adcFrontRight.set_offset(ADC_OFFSET_FRONT_RIGHT);
	adcRearLeft.set_offset(ADC_OFFSET_REAR_LEFT);
	adcRearRight.set_offset(ADC_OFFSET_REAR_RIGHT);
	// Set scale to a value obtained by calibrating the scale with known weight
  	adcFrontLeft.set_scale(ADC_SCALE);
	adcFrontRight.set_scale(ADC_SCALE);
	adcRearLeft.set_scale(ADC_SCALE);
	adcRearRight.set_scale(ADC_SCALE);
	// Read once all ADC to initialize next read access
	adc[0]=adcFrontLeft.get_units();
	adc[1]=adcFrontRight.get_units();
	adc[2]=adcRearLeft.get_units();
	adc[3]=adcRearRight.get_units();
	// Power down all ADC
	/* No need to power down as init will be done after each and every standby-shutdown
	adcFrontLeft.power_down();
	adcFrontRight.power_down();
	adcRearLeft.power_down();
	adcRearRight.power_down();
	*/
#ifdef DEBUG_HIVETRONIC_N
	printf("\tFrontLeft:\t%ld\r\n",adc[0]);
	printf("\tFrontRight:\t%ld\r\n",adc[1]);
	printf("\tRearLeft:\t%ld\r\n",adc[2]);
	printf("\tRearRight:\t%ld\r\n",adc[3]);
	printf("... init done\r\n");
#endif /* DEBUG_HIVETRONIC*/
	return NO_ERROR;
}


uint32_t initLoRa(void) {
	int ret;
#ifdef DEBUG_HIVETRONIC
	printf("initializing LoRa transceiver ...\r\n");
#endif /* DEBUG_HIVETRONIC*/
	// Power ON the module
	sx1272.ON();
	// Set transmission mode and print the result
	ret = sx1272.setMode(loraMode);
#ifdef DEBUG_HIVETRONIC_N
	printf("Lora mode: %d\r\n", loraMode);
	printf("Setting Mode: state %d\r\n", ret);
#endif /* DEBUG_HIVETRONIC*/
	// enable carrier sense
	sx1272._enableCarrierSense = true;
	// Select frequency channel
	ret = sx1272.setChannel(CH_10_868);
#ifdef DEBUG_HIVETRONIC_N
	printf("Setting Channel: state %d\r\n", ret);
#endif /* DEBUG_HIVETRONIC*/
	// Select output power (Max, High or Low)
	ret = sx1272.setPower(LORAPOWER);
#ifdef DEBUG_HIVETRONIC
	printf("Setting Power: state %d\r\n", ret);
#endif /* DEBUG_HIVETRONIC*/
	// Set the node address and print the result
	ret = sx1272.setNodeAddress(LORA_NODE_ADDR);
#ifdef DEBUG_HIVETRONIC
	//printf("Setting node addr: state %d\r\n", ret);
	printf("SX1272/76 successfully configured\r\n");
#endif /* DEBUG_HIVETRONIC*/
	return NO_ERROR;
}

void Error_Handler(uint32_t error_code) {
#ifdef DEBUG_HIVETRONIC
	printf("\n\n !!!!!! Error_Handler - Code %2ld !!!!!!\r\n",error_code);
#endif /* DEBUG_HIVETRONIC*/
	/* TODO send error code using LoRa */
	while(1) {
		/* infinite loop */
	}
}

void WakeUp() {
	if (LL_PWR_IsActiveFlag_SB()) {
		LL_PWR_ClearFlag_SB();
		WakeUpFlag |= 1;
	}
	if (LL_PWR_IsActiveFlag_InternWU()) {
		WakeUpFlag |= 2;
	}
	/* Get the pending status of the AlarmA Interrupt */
	if(__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF) != RESET) {
	  /* AlarmA callback */
#ifdef DEBUG_HIVETRONIC
	  printf("AlarmA IT flag at wake-up\r\n");
#endif /* DEBUG_HIVETRONIC*/
	  /* Clear the AlarmA interrupt pending bit */
	  __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
	 }
	__HAL_RTC_ALARM_EXTI_CLEAR_FLAG();
	PullUpGPIOA = LL_PWR_ReadReg(PUCRA);
#ifdef DEBUG_HIVETRONIC
	if (WakeUpFlag==0) {
		printf("... Wake-up from Shutdown\r\n");
	} else {
		printf("PU %ld -  WU %ld", PullUpGPIOA, WakeUpFlag);
		printf("... Wake-up from Standby\r\n");
	}
#endif /* DEBUG_HIVETRONIC*/
}

void initGPIO(void) {
	HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, PWR_GPIOA_PULLUP);
	HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_B, PWR_GPIOB_PULLUP);
	HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_C, PWR_GPIOC_PULLUP);
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, PWR_GPIOC_PULLDOWN);
	HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_D, PWR_GPIOD_PULLUP);
	LL_PWR_EnablePUPDCfg();
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

  __HAL_RCC_ADC_CLK_ENABLE();

    /**Common config
    */
  hadc1.Instance = ADC1;
  if (HAL_ADC_DeInit(&hadc1) != HAL_OK)
  {
    /* ADC de-initialization Error */
    Error_Handler(ERROR_ADC_INIT);
  }
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler(ERROR_ADC_INIT);
  }

    /**Configure the ADC multi-mode
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler(ERROR_ADC_INIT);
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  //sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler(ERROR_ADC_INIT);
  }

}

uint32_t measureVbat(uint16_t *VbatADC) {
  uint16_t ADCvalue;
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) !=  HAL_OK)
  {
    /* ADC Calibration Error */
    Error_Handler(ERROR_ADC_CAL);
  }

  __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOS);

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    /* Start Conversation Error */
    Error_Handler(ERROR_ADC_START);
  }
  if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK)
  {
    /* End Of Conversion flag not set on time */
    Error_Handler(ERROR_ADC_CONV);
  }
  ADCvalue = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  *VbatADC = ADCvalue;
  return NO_ERROR;
}

