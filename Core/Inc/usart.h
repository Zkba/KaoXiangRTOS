/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

//UART ReceiveBuffer Structure definition
typedef struct {

	uint16_t								RxBufSize;			//UARTx receive buffer size

	uint16_t								prevDataPos;		//Previous data position, initialize as RxBufSize. And -1 when receive a byte data, similar to DMAx->CNDTR

	uint16_t								nextDataPos;		//Current data position, = DMAx->CNDTR. When prev != next, UARTx receive new data

	uint8_t									*RxBufPtr;			//Pointer to RXBUFFER, an array of receive buffer

	uint8_t									*DataPtr;				//Pointer to current receive data

	uint8_t									RxState;				//Recieve state

} UART_RXBuffTypeDef;

extern UART_RXBuffTypeDef	G_UART2_RXBUF_STRUCT;

// define of RXState
#define RXState_NODATA										0x0U
#define RxState_NEWDATA										0x1U
#define RxState_ERR												0x2U

// Size of Trasmission buffer
#define TXBUFFERSIZE                      1

// Size of Reception buffer
#define RXBUFFERSIZE                      500

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

extern uint8_t G_UART2_TXBUFFER[TXBUFFERSIZE];
extern uint8_t G_UART2_RXBUFFER[RXBUFFERSIZE];
extern uint8_t G_RX_BUFF_HANDLE[RXBUFFERSIZE];

#define RX_DATA_HEAD1											0x55U
#define RX_DATA_HEAD2											0xAAU
#define RX_DATA_ADDRESS										0x00U
#define RX_DATA_TAIL1											0x0DU
#define RX_DATA_TAIL2											0x0AU

#define MOTOR_ENABLE											0xFFU
#define MOTOR_DISABLE											0x00U

#define MOTOR_DIR_CW											0x01U
#define MOTOR_DIR_CCW											0x00U

#define MOTOR_MAX_SPEED										0x07FFU
#define MOTOR_MIN_SPEED										0x0100U

extern uint8_t G_UART1_TXBUFFER[8];

/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

void UART_BSP_INIT(void);
void UART_RXBuffTypeDef_Create(uint8_t *bufptr, uint16_t bufsize, UART_RXBuffTypeDef *UART_RXBuff);
uint8_t UART_RECEIVE_DATA(uint8_t *data, UART_RXBuffTypeDef *UART_RXBuff, UART_HandleTypeDef *huart);
uint8_t UART2_GET_DATA(void);

void UART2_SEND_DATA(uint8_t *txBuff, uint16_t length);
void MOTOR_CONTROL(uint8_t enable, uint8_t dir, uint16_t speed);
void UART1_SEND_DATA(uint8_t *txBuff, uint16_t length);
uint8_t* u8_strcat(uint8_t *str, uint8_t *ptr);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
