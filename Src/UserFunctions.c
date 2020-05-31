/*
 * UserFunctions.c
 *
 *  Created on: Nov 5, 2019
 *      Author: BAY
 */
#include "main.h"

extern const uint16_t userConfig[16];

void InitUserVariables (void)
{
	_sysTickFlagScan=FALSE;
	_5msFlagScan=FALSE;
	_10msFlagScan=FALSE;
	_50msFlagScan=FALSE;
	_100msFlagScan=FALSE;
	_1000msFlagScan=FALSE;

	Tickin_1ms		=1/uwTickFreq;			/*Period in ms */
	Tickin_5ms		=5/uwTickFreq;
	Tickin_10ms		=10/uwTickFreq;
	Tickin_100ms 	=100/uwTickFreq;
	Tickin_1000ms	=1000/uwTickFreq;

	Duty=0;
	DutyMAX	=800;						 	/*Define this parametric */
	DutyMAX_Reverse= 800;
	StateSandClock=0;						/* Uninitialized vars stored in zero init area unnecassary*/
	Pot1_ConvAvg=100;
	Pot2_ConvAvg=100;

	StrArr[0] = "Ýleri Yönde Gerilim-(%):";	// curl bracket aralara virgül de olmaý lazým
	StrArr[1] = "Geri Yönde Gerilim-(%):";
	StrArr[2] = "Max Ýleri Hareket Süresi-(ms):";
	StrArr[3] = "Max Geride Bekleme Süresi(ms):";

	StrArr[4] = "YENÝ Ileri Yonde Gerilim %:";	//
	StrArr[5] = "YENÝ Geri Yonde Gerilim %:";
	StrArr[6] = "YENÝ Max Ýleri Hareket Süresi (0-5000ms):";
	StrArr[7] = "YENÝ Max Geride Bekleme Süresi (0-5000ms):";
	StrArr[8] = "Lütfen Tanýmlý Aralýkta Bir Deger Giriniz! ";	//
	StrArr[9] = "AYARLAR KAYDEDÝLSÝN MÝ? y/n ";	//
	StrArr[10] = "AYARLAR KAYDEDÝLDÝ! ";	//


	ParamArray[0] = userConfig[1];
	ParamArray[1] = userConfig[2];
	ParamArray[2] = userConfig[3];
	ParamArray[3] = userConfig[4];

    if (userConfig[0]==0xABCD)	{ 		// Otherwise Defaults will be used
		ScaleArray[0] = (uint16_t)(ParamArray[0]/TIM2PERIOD*100);
		ScaleArray[1] = (uint16_t)(ParamArray[1]/TIM2PERIOD*100);
		ScaleArray[2] = (uint16_t)(ParamArray[2]);
		ScaleArray[3] = (uint16_t)(ParamArray[3]);
	}



}

// Button timing variables
#define DebouncePeriod  40 // ms debounce period to prevent flickering when pressing or releasing the button
#define HoldTime  500 // ms hold period: how long to wait for press+hold event
#define LongHoldTime  3000 // ms long hold period: how long to wait for press+hold event

