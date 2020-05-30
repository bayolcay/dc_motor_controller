/*
 * USER_Definitions.h
 *
 *  Created on: Nov 3, 2019
 *      Author: BAY
 */

#ifndef USER_DEFINITIIONS_H_
#define USER_DEFINITIIONS_H_

#include "stm32f1xx.h"
#include "stdbool.h"

#define VREFINT_CAL_ADDR  ((uint16_t*)((uint32_t) 0xFFFFF7BA))
#define EEPROM_start_add         (uint32_t)0x4100



/*********************************************************************************************/
/*********************************************************************************************/

//#define         SysTickPeriod           ((TIM4_PERIOD+1)*(1<<TIM4_PRESCALER)/TIM4_CLOCK)   // seconds

/*********************************************************************************************/
/*********************************************************************************************/
#define TRUE	1
#define FALSE	0

bool _sysTickFlagScan;
bool _5msFlagScan;
bool _10msFlagScan;
bool _50msFlagScan;
bool _100msFlagScan;
bool _1000msFlagScan;
bool FlashSaveFlag;

uint16_t Tickin_1ms;
uint16_t Tickin_5ms;
uint16_t Tickin_10ms;
uint16_t Tickin_100ms;
uint16_t Tickin_1000ms;


extern __IO uint32_t uwTick;
extern HAL_TickFreqTypeDef uwTickFreq;			/*Period in ms */
extern char ReceivedData[100];
extern uint8_t Rxcount;
extern uint32_t dataSize;
extern volatile uint8_t check;
extern uint32_t time;


#define NUMBER_OF_PARAMETERS 4
#define NUMBER_OF_STRING (2*NUMBER_OF_PARAMETERS+3)
#define MAX_STRING_SIZE 20


uint16_t TIM2PERIOD;

uint16_t ParamArray[NUMBER_OF_PARAMETERS];
uint16_t ScaleArray[NUMBER_OF_PARAMETERS];

uint16_t Duty;
uint16_t DutyMAX;
uint16_t DutyMAX_Reverse;

uint16_t StateSandClock;

uint16_t Pot1_ConvAvg;
uint16_t Pot2_ConvAvg;



//const char StrArr[NUMBER_OF_STRING][MAX_STRING_SIZE];
const char* StrArr[NUMBER_OF_STRING];


#define FLASH_PAGE_63 (uint32_t) 0x0800FC00

//static enum	DriveState { geri_bekleme, ileri, ileri_bekleme, geri } DriveState;



void InitUserVariables (void);
void SysTickCountersUpdate(void);
void CheckButtons (void);
void SaveandShutdown (void);
extern void FLASH_PageErase(uint32_t PageAddress);

void (*Alpha_State_Ptr)(void);	// Base States pointer

void Idle_Works (void);	//state A0

void Wait_Reverse (void);	//state
void Reverse (void);	//state
void Forward (void);	//state
void Wait_Forward (void);	//state

void SaveandExit (uint16_t * Parameters);




#endif /* USER_DEFINITIIONS_H_ */
