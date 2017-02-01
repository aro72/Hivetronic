
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
#define STM32_RTC
#define LORA_ENABLED_N
#define WFI
#define ADC_POWER_OPTIM

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES

DHT dht(DHTPIN, DHTTYPE);
float weight[4] = {0};
HX711 adcFrontLeft, adcFrontRight, adcRearLeft, adcRearRight;
int loraMode = LORAMODE;
uint32_t seq = 0;
uint32_t inactiveDuration = LORA_REPORTING_PERIOD-PROCESSING_DURATION;
RTC_HandleTypeDef hrtc;

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup function

void setup() {
	// Set clock config
	// SYSCLK = 48MHz from MSI
	// AHB = 1MHz
	// APB = 1MHz

	// Open serial communications and wait for port to open:
	Serial.begin(38400);
	delay(100);
	// Print a start message
#ifdef DEBUG_HIVETRONIC
	printf("\r\n\r\n");
	printf("////////////////////////////////////\r\n");
	printf("\r\n");
	printf("Hive monitor using HX711 and LoRa\r\n");
	printf("ARM (STM32)\r\n");
#endif /* DEBUG_HIVETRONIC */
	configureClock();
	initDHT();
	initADC();
	initRTC();
#ifdef LORA_ENABLED
	initLoRa();
#endif

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Loop function

void loop(void)
{
	uint32_t r_size;
	int ret;
	uint8_t message[100]={0}, AckMessage[100]={0};
	AckData_t gwAckData;
	uint8_t AckSize;
	float Temp = NAN;
	float Hum = NAN;
	float Weight = NAN;
	tm time;
	char cDateTime[26];
	// FIX THIS - Is it really needed ???
	// sx1272.CarrierSense();
	// END OF FIX THIS
	sx1272.setPacketType(PKT_TYPE_DATA);
	while (1) {
#ifdef DEBUG_HIVETRONIC
		printf("\r\n");
		printf("////////////////////////////////////\r\n");
#endif /* DEBUG_HIVETRONIC */
		// measure temperature
		measureTempHum(&Temp, &Hum);
		measureHX711(&Weight);
		/*
		* remove temp adjustment as long as calibration and 
		* recording of a fixed weight at different temperature is not available
		*/
		// adjustWeight(&Weight, Temp);
		getRTCDateTime(&time);
		sprintf(cDateTime, "%02d/%02d/%4d-%02d:%02d:%02d",
				time.tm_mday,
				time.tm_mon,
				(time.tm_year+1900),
				time.tm_hour,
				time.tm_min,
				time.tm_sec);
    	String strT(Temp, 2);
    	String strH(Hum, 2);
    	String strW(Weight, 3);
    	String strSeq(seq, 10);
    	String strTime(cDateTime);
    	String messageData;

      	messageData = strTime + " - Seq: " + strSeq + " - T=" + strT + " - H=" + strH + " - W=" + strW;
    	r_size = strlen(messageData.c_str());
    	for (uint32_t i = 0; i < r_size; i++) {
			message[i] = (uint8_t) messageData.c_str()[i];
		}
    	message[r_size]='\r';
    	message[r_size+1]='\n';
#ifdef DEBUG_HIVETRONIC
    	printf("\nSending Message (length=%d) : %s\r\n", (int)r_size,message);
#endif /* DEBUG_HIVETRONIC*/
		seq++;
	    ret = sx1272.sendPacketTimeoutACK(DEFAULT_DEST_ADDR, message, r_size);
#ifdef DEBUG_HIVETRONIC
	    printf("Packet sent, state %d\r\n", ret);
	    if (ret == 3)
			printf("No Ack!\r\n");
		if (ret == 0)
			printf("Ack received from gateway!\r\n");
#endif /* DEBUG_HIVETRONIC*/
		if (ret==0) {
			sx1272.getAckPacket(DEFAULT_DEST_ADDR, AckMessage, &AckSize);
			handleAckData(AckMessage, &AckSize, &gwAckData);
		}
		//delay(inactiveDuration);
		enterLowPower(PM_SLEEP, inactiveDuration);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS IMPLEMENTATION
#define STDBY_MODE_N
void GotoLowPower() {
#ifdef STDBY_MODE
	uint32_t newSysTickLOAD;
	RCC_ClkInitTypeDef LowPowerClkConfig;
	RCC_OscInitTypeDef LowPowerOscConfig;
	RCC_PeriphCLKInitTypeDef LowPowerPerighClkConfig;

	/* Stop SysTick */
	HAL_SuspendTick();
	/* Set new clocks configuration to enable LPSLEEP mode */
	LowPowerOscConfig.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	LowPowerOscConfig.LSEState = RCC_LSE_ON;
	LowPowerOscConfig.MSIState = RCC_MSI_ON;
	LowPowerOscConfig.MSIClockRange = RCC_MSIRANGE_4; /* MSI at 1MHz */
	LowPowerOscConfig.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	LowPowerOscConfig.PLL.PLLState = RCC_PLL_OFF;
	LowPowerClkConfig.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	LowPowerClkConfig.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	LowPowerClkConfig.AHBCLKDivider = RCC_SYSCLK_DIV512;
	LowPowerClkConfig.APB1CLKDivider = RCC_HCLK_DIV16;
	LowPowerClkConfig.APB2CLKDivider = RCC_HCLK_DIV16;
	HAL_RCC_OscConfig(&LowPowerOscConfig);
	HAL_RCC_ClockConfig(&LowPowerClkConfig, FLASH_LATENCY_0);
	/* Switch to Low Power regulator */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);


#else /* STDBY_MODE */
	uint32_t systick_csr;
	/* Stop TIM5 and SysTick */
	TIM5_CR1 &= 0xFFFFFFFE;
	systick_csr = SYST_CSR;
	SYST_CSR &=0xFFFFFFFD;
   	__WFI();
	/* Stop TIM5 and SysTick */
	TIM5_CR1 |= 0x01;
	SYST_CSR = systick_csr;
#endif /* STDBY_MODE */
}
uint32_t enterLowPower(uint32_t mode, uint32_t duration) {
	tm time, alarm;
	getRTCDateTime(&time);
	addDateTime(&alarm, time, duration);
	setAlarm(alarm);
#ifdef DEBUG_HIVETRONIC
	printf("Alarm set: %02d:%02d:%02d\r\n", alarm.tm_hour, alarm.tm_min, alarm.tm_sec);
	printf("Low Power entry ...\r\n");
#endif /* DEBUG_HIVETRONIC */
	delay(10);
	GotoLowPower();
#ifdef DEBUG_HIVETRONIC
	printf("\t... Low Power exit\r\n");
#endif /* DEBUG_HIVETRONIC */
	return NO_ERROR;
}

uint32_t configureClock(void) {
	uint32_t ret=NO_ERROR;
	// set SYSCLK
	//RCC_CR &= ˜0xFFFFFF0F;
	//RCC_CR |= =0x0000005F, // MSI range: 2MHz
	RCC_PLLSAI1CFGR = 0x00000000;
	RCC_PLLSAI2CFGR = 0x00000000;
	// set Cortex-M4 SysTick
	// set Range 2 voltage
    RCC_APB1ENR1 |= 0x10000000;
	//PWR_CR1 = 0x00004700;
	return ret;
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
		carry=1;
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
	__HAL_RTC_ALARM_EXTI_CLEAR_FLAG();
	__HAL_RTC_ALARM_EXTI_ENABLE_RISING_EDGE();
	__HAL_RTC_ALARM_EXTI_ENABLE_IT();
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
#else /* 0 */
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
#endif
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
			time->tm_mon,
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
    			if ((opcode&ACK_PERIOD_CODE)==ACK_PERIOD_CODE) {
	    			// need to update date and time
    				gwAckData->gwReportingPeriod = (uint32_t) ((AckMessage[i]<<24)+(AckMessage[i+1]<<16)+(AckMessage[i+2]<<8)+(AckMessage[i+3])) ;
    				inactiveDuration = gwAckData->gwReportingPeriod - PROCESSING_DURATION;
    				i=+4;
    			}
			} while (i<*AckSize);
    	}
    	ret=NO_ERROR;
    }
	return ret;
}

uint32_t measureHX711(float* Weight) {
	int32_t FrontLeftTab[ADC_NB_SAMPLES];
	int32_t FrontRightTab[ADC_NB_SAMPLES];
	int32_t RearLeftTab[ADC_NB_SAMPLES];
	int32_t RearRightTab[ADC_NB_SAMPLES];
	uint32_t i, start_idx, stop_idx;
	float FrontLeft_avg, FrontRight_avg, RearLeft_avg, RearRight_avg;

	// Power up all ADC
	adcFrontLeft.power_up();
	adcFrontRight.power_up();
	adcRearLeft.power_up();
	adcRearRight.power_up();
	delay(ADC_POWER_UP_MS);

	// Read ADC for all load cells
	for (i=0;i<ADC_NB_SAMPLES;i++) {
		FrontLeftTab[i]  = (int32_t)  adcFrontLeft.get_units();
		FrontRightTab[i] = (int32_t) adcFrontRight.get_units();
		RearLeftTab[i]   = (int32_t) adcRearLeft.get_units();
		RearRightTab[i]  = (int32_t) adcRearRight.get_units();
		delay(100);
	}

	// Power down all ADC
	adcFrontLeft.power_down();
	adcFrontRight.power_down();
	adcRearLeft.power_down();
	adcRearRight.power_down();

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
#ifdef DEBUG_HX711
	printf("Start_index: %ld\r\n", start_idx);
	printf("Stop_index: %ld\r\n", stop_idx);
#endif /* DEBUG_HX711 */
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
	printf("Average: %.0f\t%.0f\t%.0f\t%.0f\r\n", FrontLeft_avg, FrontRight_avg, RearLeft_avg, RearRight_avg);
#endif /* DEBUG_HX711 */

	// Calculate weight
	*Weight = FrontLeft_avg + FrontRight_avg + RearLeft_avg + RearRight_avg;
#ifdef DEBUG_HIVETRONIC
	printf("Weight: %.0f\r\n", *Weight);
#endif /* DEBUG_HIVETRONIC */

	return NO_ERROR;
}

uint32_t adjustWeight(float* Weight, float T) {
	uint32_t ret=NO_ERROR;
	float deltaTemp, deltaWeight;
	deltaTemp = T-ADC_CAL_TEMP;
	deltaWeight = LOADCELL_TEMP_EFFECT*deltaTemp;
#ifdef DEBUG_HIVETRONIC
	printf("Weight before adjustment: %.0f\r\n", *Weight);
	printf("Weight after adjustment: %.0f\r\n", *Weight-deltaWeight);
#endif /* DEBUG_HIVETRONIC */
	*Weight = *Weight - deltaWeight;
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
#ifdef DEBUG_HIVETRONIC
	printf("initializing DHT22 sensor ...\r\n");
#endif /* DEBUG_HIVETRONIC*/
	Wire.begin();
	dht.begin();
#ifdef DEBUG_HIVETRONIC
	printf("... init done\r\n");
#endif /* DEBUG_HIVETRONIC*/
	return NO_ERROR;
}

uint32_t initRTC(void) {
#ifdef RTC_HAL
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
#else /* RTC_HAL */
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
    // update date and time in RTC
    setRTCDateTime(time);
#endif /* RTC_HAL */
    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
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
#ifdef DEBUG_HIVETRONIC
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
	delay(500);
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
	adcFrontLeft.power_down();
	adcFrontRight.power_down();
	adcRearLeft.power_down();
	adcRearRight.power_down();
#ifdef DEBUG_HIVETRONIC
	printf("\tFrontLeft:\t%ld\r\n",adc[0]);
	printf("\tFrontRight:\t%ld\r\n",adc[1]);
	printf("\tRearLeft:\t%ld\r\n",adc[2]);
	printf("\tRearRight:\t%ld\r\n",adc[3]);
	printf("... init done");
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
#ifdef DEBUG_HIVETRONIC
	printf("Lora mode: %d\r\n", loraMode);
	printf("Setting Mode: state %d\r\n", ret);
#endif /* DEBUG_HIVETRONIC*/
	// enable carrier sense
	sx1272._enableCarrierSense = true;
	// Select frequency channel
	ret = sx1272.setChannel(CH_10_868);
#ifdef DEBUG_HIVETRONIC
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
	printf("Setting node addr: state %d\r\n", ret);
	// Print a success message
	printf("SX1272/76 successfully configured\r\n");
#endif /* DEBUG_HIVETRONIC*/
	return NO_ERROR;
}

void Error_Handler(uint32_t error_code) {
	printf("\n\n !!!!!! Error_Handler - Code %2ld !!!!!!\r\n",error_code);
	while(1) {
		/* infinite loop */
	}
}
