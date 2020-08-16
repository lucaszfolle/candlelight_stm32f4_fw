/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <gs_usb.h>

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

typedef struct {
	CAN_TypeDef *instance;
	uint16_t brp;
	uint8_t phase_seg1;
	uint8_t phase_seg2;
	uint8_t sjw;
} can_data_t;

void can_init(can_data_t *hcan, CAN_TypeDef *instance);
bool can_set_bittiming(can_data_t *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw);
void can_enable(can_data_t *hcan, bool loop_back, bool listen_only, bool one_shot);
void can_disable(can_data_t *hcan);
bool can_is_enabled(can_data_t *hcan);

bool can_receive(can_data_t *hcan, struct gs_host_frame *rx_frame);
bool can_is_rx_pending(can_data_t *hcan);

bool can_send(can_data_t *hcan, struct gs_host_frame *frame);

uint32_t can_get_error_status(can_data_t *hcan);
bool can_parse_error_status(uint32_t err, struct gs_host_frame *frame);

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
