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

	StrArr[0] = "�leri Y�nde Gerilim-(%):";	// curl bracket aralara virg�l de olma� laz�m
	StrArr[1] = "Geri Y�nde Gerilim-(%):";
	StrArr[2] = "Max �leri Hareket S�resi-(ms):";
	StrArr[3] = "Max Geride Bekleme S�resi(ms):";

	StrArr[4] = "YEN� Ileri Yonde Gerilim %:";	//
	StrArr[5] = "YEN� Geri Yonde Gerilim %:";
	StrArr[6] = "YEN� Max �leri Hareket S�resi (0-5000ms):";
	StrArr[7] = "YEN� Max Geride Bekleme S�resi (0-5000ms):";
	StrArr[8] = "L�tfen Tan�ml� Aral�kta Bir Deger Giriniz! ";	//
	StrArr[9] = "AYARLAR KAYDED�LS�N M�? y/n ";	//
	StrArr[10] = "AYARLAR KAYDED�LD�! ";	//


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

