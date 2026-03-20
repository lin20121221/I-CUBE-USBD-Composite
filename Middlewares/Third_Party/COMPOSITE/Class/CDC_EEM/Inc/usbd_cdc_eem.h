/**
  ******************************************************************************
  * @file    usbd_cdc_eem.h
  * @author  AL94
  * @brief   Header file for the usbd_cdc_eem.c file.
  *
  *          CDC EEM (Ethernet Emulation Model) - USB CDC Subclass 0x0C
  *          Protocol 0x07, per USB CDC 1.2 spec Table 17.
  *
  *          EEM is simpler than ECM: a single bulk-only interface carries
  *          both data and control information using in-band framing.
  *          No notification endpoint is needed.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_CDC_EEM_H
#define __USB_CDC_EEM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup usbd_cdc_eem
  * @brief This file is the Header file for usbd_cdc_eem.c
  * @{
  */

/** @defgroup usbd_cdc_eem_Exported_Defines
  * @{
  */

#define CDC_EEM_STR_DESC                    "STM32 CDC EEM"

/* EEM endpoint packet sizes */
#ifndef CDC_EEM_DATA_HS_MAX_PACKET_SIZE
#define CDC_EEM_DATA_HS_MAX_PACKET_SIZE     512U  /* Bulk IN/OUT max packet size (HS) */
#endif
#ifndef CDC_EEM_DATA_FS_MAX_PACKET_SIZE
#define CDC_EEM_DATA_FS_MAX_PACKET_SIZE     64U   /* Bulk IN/OUT max packet size (FS) */
#endif

/* EEM descriptor total size:
 *   9 (config) + 8 (IAD) + 9 (interface) + 7 (EP OUT) + 7 (EP IN) = 40
 */
#define CDC_EEM_CONFIG_DESC_SIZE            40U

/* Data buffer size – must accommodate one full Ethernet frame + 2-byte EEM header */
#define CDC_EEM_DATA_BUFFER_SIZE            2002U   /* 1500 payload + 14 Eth hdr + 2 EEM hdr + margin */

#define CDC_EEM_DATA_HS_IN_PACKET_SIZE      CDC_EEM_DATA_HS_MAX_PACKET_SIZE
#define CDC_EEM_DATA_HS_OUT_PACKET_SIZE     CDC_EEM_DATA_HS_MAX_PACKET_SIZE
#define CDC_EEM_DATA_FS_IN_PACKET_SIZE      CDC_EEM_DATA_FS_MAX_PACKET_SIZE
#define CDC_EEM_DATA_FS_OUT_PACKET_SIZE     CDC_EEM_DATA_FS_MAX_PACKET_SIZE

/* EEM in-band command types (bits[14:11] of the EEM command header) */
#define CDC_EEM_CMD_ECHO                    0x00U
#define CDC_EEM_CMD_ECHO_RESPONSE           0x01U
#define CDC_EEM_CMD_SUSPEND_HINT            0x02U
#define CDC_EEM_CMD_RESPONSE_HINT          0x03U
#define CDC_EEM_CMD_RESPONSE_COMPLETE_HINT 0x04U
#define CDC_EEM_CMD_TICKLE                  0x05U

/* EEM header bit definitions */
#define CDC_EEM_HDR_TYPE_DATA               0x0000U   /* bit15 = 0 → data packet  */
#define CDC_EEM_HDR_TYPE_CMD                0x8000U   /* bit15 = 1 → EEM command  */
#define CDC_EEM_HDR_DATA_LEN_MASK           0x3FFFU   /* bits[13:0] payload length */
#define CDC_EEM_HDR_CRC_NOT_CALCULATED      0x4000U   /* bit14: CRC sentinel flag  */

/* Ethernet maximum segment size */
#ifndef CDC_EEM_ETH_MAX_SEGSZE
#define CDC_EEM_ETH_MAX_SEGSZE              1514U
#endif

/**
  * @}
  */

/** @defgroup usbd_cdc_eem_Exported_TypesDefinitions
  * @{
  */

typedef struct
{
  int8_t (*Init)(void);
  int8_t (*DeInit)(void);
  int8_t (*Receive)(uint8_t *Buf, uint32_t *Len);
  int8_t (*TransmitCplt)(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
  int8_t (*Process)(USBD_HandleTypeDef *pdev);
} USBD_CDC_EEM_ItfTypeDef;

typedef struct
{
  /* Raw receive buffer (one MaxPacket chunk at a time) */
  uint8_t  RxBuffer[CDC_EEM_DATA_BUFFER_SIZE];
  /* Assembled Ethernet frame accumulator */
  uint8_t  EthRxBuffer[CDC_EEM_DATA_BUFFER_SIZE];
  uint32_t EthRxLength;           /* bytes accumulated so far in EthRxBuffer  */
  uint32_t EthFrameLength;        /* total expected frame length from EEM hdr  */

  uint8_t *TxBuffer;
  uint32_t TxLength;

  __IO uint32_t TxState;
  __IO uint32_t RxState;
  __IO uint32_t MaxPcktLen;
} USBD_CDC_EEM_HandleTypeDef;

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef USBD_CDC_EEM;

extern uint8_t CDC_EEM_IN_EP;
extern uint8_t CDC_EEM_OUT_EP;
extern uint8_t CDC_EEM_ITF_NBR;
extern uint8_t CDC_EEM_STR_DESC_IDX;

/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */

uint8_t USBD_CDC_EEM_RegisterInterface(USBD_HandleTypeDef *pdev,
                                       USBD_CDC_EEM_ItfTypeDef *fops);

uint8_t USBD_CDC_EEM_SetTxBuffer(USBD_HandleTypeDef *pdev,
                                 uint8_t *pbuff, uint32_t length);

uint8_t USBD_CDC_EEM_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff);

uint8_t USBD_CDC_EEM_ReceivePacket(USBD_HandleTypeDef *pdev);

uint8_t USBD_CDC_EEM_TransmitPacket(USBD_HandleTypeDef *pdev);

void    USBD_Update_CDC_EEM_DESC(uint8_t *desc,
                                 uint8_t itf,
                                 uint8_t in_ep,
                                 uint8_t out_ep,
                                 uint8_t str_idx);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USB_CDC_EEM_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
