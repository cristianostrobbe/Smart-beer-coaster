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
  * @version V1.1.2
  * @date    08-September-2017
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
#include "low_power.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include "radio.h"
//#include "stm32l0xx_hal.h"
//#include "stm32l0xx_hal_lptim.h"

/* Private typedef -----------------------------------------------------------*/
uint32_t adc_int = 0 ;
char msg_int[50] = "bla bla";
uint32_t gpioA3_int = 0 ;
char gpioA3_msg[50] = "bla bla";
uint32_t adc_batt = 0 ;
char msg_batt[50] = "bla bla";

char dbug[50] = "bla bla";

//Format data
char s0[]= "{\"D\":\"SMARTGLASS\"";
char s1[]= ",\"S\":";
char s2[]= ",\"B\":";
char s3[]= ",\"W\":";
char s4[]= "}";

//LPTIM_HandleTypeDef hlptim1;
/* Private define ------------------------------------------------------------*/
/*!
 * CAYENNE_LPP is myDevices Application server.
 */
//#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_HUMIDITY       0x68
#define LPP_DATATYPE_TEMPERATURE    0x67
#define LPP_DATATYPE_BAROMETER      0x73

#define LPP_APP_PORT 99

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            100
/*!
 * LoRaWAN Adaptive Data Rate (ADR) is a mechanism for optimizing data rates, airtime and energy consumption in the network
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              0
/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG                   false
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * Number of trials for the join request.
 */
#define JOINREQ_NBTRIALS                            3

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void communication(void);
void toggle_led(int GPIOx,int GPIO_PIN_x,int time);
//static void MX_LPTIM1_Init(void);
//void buzzer(void);

/* call back when LoRa will transmit a frame*/
static void LoraTxData( lora_AppData_t *AppData, FunctionalState* IsTxConfirmed);

/* call back when LoRa has received a frame*/
static void LoraRxData( lora_AppData_t *AppData);

/* Private variables ---------------------------------------------------------*/
//extern static DeviceState_t DeviceState;

/* load call backs*/
static LoRaMainCallback_t LoRaMainCallbacks ={ HW_GetBatteryLevel,
                                               HW_GetUniqueId,
                                               HW_GetRandomSeed,
                                               LoraTxData,
                                               LoraRxData};

/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;


#ifdef USE_B_L072Z_LRWAN1
/*!
 * Timer to handle the application Tx Led to toggle
 */
static TimerEvent_t TxLedTimer;
static void OnTimerLedEvent( void );
#endif
/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {TX_ON_TIMER,
                                    APP_TX_DUTYCYCLE,
                                    CLASS_A,
                                    LORAWAN_ADR_ON,
                                    DR_5,
                                    LORAWAN_PUBLIC_NETWORK,
                                    JOINREQ_NBTRIALS};

/* Private functions ---------------------------------------------------------*/

 void EXTI2_3_IRQHandler( void )
 {
	 int lb = 300; // lower boundary adc
	 int ub = 1500; // upper boundary adc
	 int lbat = 2500; // lower boundary bat
	 // Voltage divider = 0.5
	 int discharged = 0;
	 int empty = 0;
	 int callwaiter=0;

	 gpioA3_int = HW_GPIO_Read(GPIOA,  GPIO_PIN_3 );
	 sprintf(gpioA3_msg, "GPIO A3: %lu\t\r\n", gpioA3_int);
	 PRINTF(gpioA3_msg);

	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	 adc_batt = HW_AdcReadChannel( ADC_CHANNEL_4 );
	 if( adc_batt < lbat ) {
		 discharged = 1;
	 }
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	 sprintf(dbug, "battery value: %lu, battery state: %u \t\r\n", adc_batt,discharged);
	 PRINTF(dbug);

	 switch (gpioA3_int){
	 	 case( 1 ):
				 HAL_Delay(500);
				 adc_int = HW_AdcReadChannel( ADC_CHANNEL_0 );

	 		 if( adc_int < ub && adc_int > lb ) {
	 			//Empty Glass
	 			empty=1;
	 			sprintf(msg_int, "%s%s%u%s%u%s%u%s", s0, s1,empty, s2, discharged,s3 , callwaiter,s4);
	 			sprintf(dbug, "EMPTY GLASS adc: %lu \t\r\n", adc_int);
	 			communication();
	 			toggle_led(GPIOB,GPIO_PIN_7,500);
	 			}
	 		 else {
	 			//Full or NO Glass
	 			sprintf(msg_int, "%s%s%u%s%u%s%u%s", s0, s1,empty, s2, discharged,s3 , callwaiter,s4);
	 			sprintf(dbug, "FULL or NO GLASS adc: %lu \t\r\n", adc_int);
	 		 }
	 		PRINTF(dbug);
	 		//PRINTF(msg_int);
	 	break;

	 	case( 0 ):
			callwaiter=1;
	 		sprintf(msg_int, "%s%s%u%s%u%s%u%s", s0, s1,empty, s2, discharged,s3 , callwaiter,s4);
			PRINTF("CALL WAITER \t\r\n");
			communication();
			toggle_led(GPIOB,GPIO_PIN_7,500);
		break;
	 }

   HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_2 );

   HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_3 );
 }

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void )
{
  /* STM32 HAL library initialization*/
  HAL_Init( );
  
  /* Configure the system clock*/
  SystemClock_Config( );
  
  /* Configure the debug mode*/
  DBG_Init( );
  
  /* Configure the hardware*/
  HW_Init( );

  /* USER CODE BEGIN 1 */
  MX_GPIO_Init();
//  MX_LPTIM1_Init();
  /* USER CODE END 1 */
  
  /* Configure the Lora Stack*/
  lora_Init( &LoRaMainCallbacks, &LoRaParamInit);
  
//  HAL_LPTIM_PWM_Start(&hlptim1, 1000, 100);

  PRINTF("VERSION: %X\n\r", VERSION);

  /* main loop*/
  while( 1 )
  {

	  HW_EnterSleepMode();
  }
}

static void LoraTxData( lora_AppData_t *AppData, FunctionalState* IsTxConfirmed)
{
	  /* USER CODE BEGIN 3 */
	  PRINTF("LoraTxData\r\n ");
	  //batteryLevel = HW_GetBatteryLevel( );                     /* 1 (very low) to 254 (fully charged) */

	  AppData->Port = LORAWAN_APP_PORT;

	  *IsTxConfirmed =  LORAWAN_CONFIRMED_MSG;
	  sprintf(AppData->Buff, msg_int);
	  AppData->BuffSize = strlen(AppData->Buff);
	  /* USER CODE END 3 */

}
    
static void LoraRxData( lora_AppData_t *AppData )
{
  /* USER CODE BEGIN 4 */
  switch (AppData->Port)
  {
  case LORAWAN_APP_PORT:
    if( AppData->BuffSize == 1 )
    {
      AppLedStateOn = AppData->Buff[0] & 0x01;
      if ( AppLedStateOn == RESET )
      {
        PRINTF("LED OFF\n\r");
        LED_Off( LED_BLUE ) ; 
        
      }
      else
      {
        PRINTF("LED ON\n\r");
        LED_On( LED_BLUE ) ; 
      }
      //GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 0 : 1 );
    }
    break;
  case LPP_APP_PORT:
  {
    AppLedStateOn= (AppData->Buff[2] == 100) ?  0x01 : 0x00;
      if ( AppLedStateOn == RESET )
      {
        PRINTF("LED OFF\n\r");
        LED_Off( LED_BLUE ) ; 
        
      }
      else
      {
        PRINTF("LED ON\n\r");
        LED_On( LED_BLUE ) ; 
      }
    break;
  }
  default:
    break;
  }
  /* USER CODE END 4 */
}

#ifdef USE_B_L072Z_LRWAN1
static void OnTimerLedEvent( void )
{
  LED_Off( LED_RED1 ) ; 
}
#endif

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
  /*Configure GPIO pin : PA3 interupt piezo*/
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  /*Configure GPIO pin : PB2 intterrupt botton */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 interruttore batteria */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

}

static void communication(void){

	for (int j=0; j<4; j++)
	{

//		Radio.SetChannel( 868100000 ); // frequency of gateway to receive and endnote to send message
//		Radio.SetTxConfig( MODEM_LORA, TX_POWER_1, 0, 0, 12, 1, 8, 0, true, 1, 0, 0, 3000 );
//		LoRaMacChannelRemove(2);
//		LoRaMacChannelRemove(3);
//		LoRaMacChannelRemove(4);
//		LoRaMacChannelRemove(5);
//		LoRaMacChannelRemove(6);
//		LoRaMacChannelRemove(7);
//		LoRaMacChannelRemove(8);
//		LoRaMacChannelRemove(9);
		/* run the LoRa class A state machine*/
		lora_fsm( );
	 }

}

void toggle_led(int GPIOx,int GPIO_PIN_x,int time){
	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);  //LED on
	HAL_Delay(time);
	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET); //LED off
}
//
///* LPTIM1 init function */
//static void MX_LPTIM1_Init(void)
//{
//
//  hlptim1.Instance = LPTIM1;
//  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
//  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV16;
//  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
//  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
//  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
//  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
//  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
//  {
//    while(1){
//
//    }
//  }
//
//}
//
//void buzzer(void){
//	for(int i=0; i<100000; i++){
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0));
//	PRINTF("sound\n\r");
//	}
//}
