/**
  ******************************************************************************
  * @file    usbd_cdc_eem_if.h
  * @author  AL94
  * @brief   Header for usbd_cdc_eem_if.c
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
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
#ifndef __USBD_CDC_EEM_IF_H
#define __USBD_CDC_EEM_IF_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_eem.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Ethernet Maximum Segment size, typically 1514 bytes */
#define CDC_EEM_ETH_MAX_SEGSZE_IF               1514U

/* Max number of Tx trials waiting for EEM to become ready */
#define CDC_EEM_MAX_TX_WAIT_TRIALS              1000000U

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

extern USBD_CDC_EEM_ItfTypeDef USBD_CDC_EEM_fops;

#endif /* __USBD_CDC_EEM_IF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
