 /*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic lora driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    10-July-2018
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"
#include "timeServer.h"
#include "version.h"
#include "command.h"
#include "at.h"
#include "lora.h"
#include "bsp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/**
 * @brief LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1
#define APP_TX_DUTYCYCLE                            20000
#define SETALAB_CTRL_PORT				   222
// for thingplus
//#define __USE_THINGPLUS__
#define THINGPLUS_CTRL_PORT				   11
#define PWOFF 0
#define PR_PORT 10		
static uint32_t pr_cnt = 0;

/**
 * When fast wake up is enabled, the mcu wakes up in ~20us and
 * does not wait for the VREFINT to be settled. THis is ok for
 * most of the case except when adc must be used in this case before
 * starting the adc, you must make sure VREFINT is settled
 */
#define ENABLE_FAST_WAKEUP

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa has received a frame*/
static void LoraRxData(lora_AppData_t *AppData);
/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );
/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded ( void );

/* Private variables ---------------------------------------------------------*/
/* load call backs*/
/*!
 * User application data
 */
static uint8_t AppDataBuff[256];

/*!
 * User application data structure
 */
lora_AppData_t AppData={ AppDataBuff,  0 ,0 };

static TimerEvent_t TxTimer;

static LoRaMainCallback_t LoRaMainCallbacks = { HW_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LoraRxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded};

/**
 * Initialises the Lora Parameters
 */
static LoRaParam_t LoRaParamInit = {LORAWAN_ADR_ON,
                                    DR_0,
                                    LORAWAN_PUBLIC_NETWORK};

/* Private functions ---------------------------------------------------------*/

																		/* LoRa endNode send request*/
static void Send( void );

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

/* tx timer callback function*/
static void OnTxTimerEvent( void );

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main( void )
{
    /* STM32 HAL library initialization*/
  HAL_Init();

  /* Configure the system clock*/
  SystemClock_Config();

  /* Configure the hardware*/
  HW_Init();

  /* Configure Debug mode */
  DBG_Init();
  
  /* USER CODE BEGIN 1 */
  CMD_Init();
  /*Disable standby mode*/
  LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);
  
  PRINTF("ATtention command interface\n\r");
  /* USER CODE END 1 */

  /* Configure the Lora Stack*/
  LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);

	LoraStartTx( TX_ON_TIMER) ;

  /* main loop*/
  while (1)
  {
    /* Handle UART commands */
    CMD_Process();
    
    LoRaMacProcess( );
    /*
     * low power section
     */
    DISABLE_IRQ();
    /*
     * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway
     * don't go in low power mode if we just received a char
     */
#ifndef LOW_POWER_DISABLE
    LPM_EnterLowPower();
#endif
    ENABLE_IRQ();

    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */
  }
}


static void LoraRxData(lora_AppData_t *AppData)
{
	if(AppData == 0)
	{
		PRINTF("LoraRxData but AppData is NULL\r\n");
		return;
	}
	
	PRINTF("AppData Port=%d AppData->BuffSize=%d\r\n", AppData->Port, AppData->BuffSize);

	switch (AppData->Port)
	{
		case THINGPLUS_CTRL_PORT:
		{
			PRINTF("THINGPLUS_CTRL_PORT\n"); 
			for(int j=0; j<AppData->BuffSize; j++) 
				PRINTF("%02X ", AppData->Buff[j]); 
			PRINTF("\n"); 
			
			if(AppData->BuffSize > 4)
			{
				uint32_t utc = 0;
				for(int i=0; i<4; i++)
						utc |=  (uint32_t)((AppData->Buff[1+i])<<(8*i));
				PRINTF("status=%d utc=%d \n", AppData->Buff[0], utc);
				if(AppData->BuffSize > 7)
				{
					PRINTF("Type = %1X SeqLen=%1X Val=%d\n", AppData->Buff[5], AppData->Buff[6], AppData->Buff[7]);
					switch(AppData->Buff[5])
					{
						// Power Off
						case 0x3D:
							if(AppData->Buff[7]==PWOFF)
							{
								PRINTF("Reset Device!!\n");
								TimerStop( &TxTimer ); 
								HAL_Delay(1000);
								HAL_NVIC_SystemReset();
							}
							break;
					}
				}
			}
		}
		break;
	}
	set_at_receive(AppData->Port, AppData->Buff, AppData->BuffSize);
}

static void Send( void )
{
  /* USER CODE BEGIN 3 */
  uint16_t pressure = 0;
  int16_t temperature = 0;
  uint16_t humidity = 0;
  uint8_t batteryLevel;
  sensor_t sensor_data;

  uint32_t i = 0;
	
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    return;
  }
	
  BSP_sensor_Read( &sensor_data );
	
  temperature = ( int16_t )( sensor_data.temperature*100); 
  pressure    = ( uint16_t )( sensor_data.pressure ); 
  humidity    = ( uint16_t )( sensor_data.humidity*100 ); 
	
  /* 1 (very low) to 254 (fully charged) */
  batteryLevel = HW_GetBatteryLevel( );                     
  PRINTF("pr_cnt=%d temperature = %d Pressure=%d Humidity=%d batteryLevel=%d\n", pr_cnt, temperature, pressure, humidity, batteryLevel);

  AppData.Port = PR_PORT;

//#ifdef __USE_THINGPLUS__
  AppData.Buff[i++]  = 0; // status
  
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 0; // utc
	AppData.Buff[i++]  = 1; // temperature type
	AppData.Buff[i++]  = 0x12; //  the same type num & data size
	AppData.Buff[i++] = ( temperature >> 8 ) & 0xFF;
	AppData.Buff[i++] = temperature & 0xFF;

  AppData.Buff[i++]  = 2; // humidity type
	AppData.Buff[i++]  = 0x12; //  the same type num & data size
  AppData.Buff[i++] = ( humidity >> 8 ) & 0xFF;
	AppData.Buff[i++] = humidity & 0xFF;
	
  AppData.Buff[i++]  = 28; // pressure type
	AppData.Buff[i++]  = 0x12; //  the same type num & data size
  AppData.Buff[i++] = ( pressure >> 8 ) & 0xFF;
  AppData.Buff[i++] = pressure & 0xFF;
//#endif

  AppData.BuffSize = i;
	pr_cnt++;
  LORA_send( &AppData, lora_config_reqack_get());
  
  /* USER CODE END 3 */
}

static void OnTxTimerEvent( void )
{
  /*Wait for next tx slot*/
  TimerStart( &TxTimer);
  /*Send*/
  Send( );
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
    OnTxTimerEvent();
  }
  else
  {
    /* send everytime button is pushed */
    GPIO_InitTypeDef initStruct={0};
  
    initStruct.Mode =GPIO_MODE_IT_RISING;
    initStruct.Pull = GPIO_PULLUP;
    initStruct.Speed = GPIO_SPEED_HIGH;

    HW_GPIO_Init( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct );
    HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, Send );
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  Error_Handler();
}
#endif

static void LORA_HasJoined( void )
{
  PRINTF("JOINED\n\r");
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF("switch to class %c done\n\r","ABC"[Class] );
}

static void LORA_TxNeeded ( void )
{
  PRINTF("Network Server is asking for an uplink transmission\n\r");
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
