/**
  ******************************************************************************
  * @file    usbd_cdc_ncm.h
  * @author  AL94
  * @brief   Header file for the usbd_cdc_ncm.c file.
  *
  *          CDC NCM (Network Control Model) - USB CDC Subclass 0x0D
  *          Protocol 0x01, per USB CDC 1.2 spec and CDC NCM 1.0 spec.
  *
  *          NCM uses two interfaces:
  *            - Interface 0: Communication class (control) with a notification EP
  *            - Interface 1: Data class with bulk IN + OUT endpoints
  *
  *          Data is framed in NTBs (Network Transfer Blocks), which can carry
  *          multiple Ethernet frames per USB transfer, making NCM significantly
  *          more efficient than ECM or EEM for high-throughput applications.
  *          This makes it the preferred choice when connecting to LwIP.
  *
  *          USB descriptor topology:
  *            IAD → Control Interface (CDC Header + Union + ECM Functional)
  *                    Interrupt IN notification endpoint
  *              → Data Interface (alternate 0 = no EPs, alternate 1 = bulk IN+OUT)
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
#ifndef __USB_CDC_NCM_H
#define __USB_CDC_NCM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup usbd_cdc_ncm
  * @brief This file is the Header file for usbd_cdc_ncm.c
  * @{
  */

/** @defgroup usbd_cdc_ncm_Exported_Defines
  * @{
  */

#define CDC_NCM_STR_DESC                    "STM32 CDC NCM"

/* ---- Notification endpoint interval ---- */
#ifndef CDC_NCM_HS_BINTERVAL
#define CDC_NCM_HS_BINTERVAL                0x10U
#endif
#ifndef CDC_NCM_FS_BINTERVAL
#define CDC_NCM_FS_BINTERVAL                0x10U
#endif

/* ---- Endpoint max-packet sizes ---- */
#ifndef CDC_NCM_DATA_HS_MAX_PACKET_SIZE
#define CDC_NCM_DATA_HS_MAX_PACKET_SIZE     512U
#endif
#ifndef CDC_NCM_DATA_FS_MAX_PACKET_SIZE
#define CDC_NCM_DATA_FS_MAX_PACKET_SIZE     64U
#endif
#define CDC_NCM_CMD_PACKET_SIZE             16U

/* ---- Descriptor total size (bytes) ----
 *   9   Configuration Descriptor
 *   8   IAD
 *   9   Control Interface Descriptor
 *   5   CDC Header Functional Descriptor
 *   5   CDC Union Functional Descriptor     (bLen+bType+bSubtype+bMaster+bSlave0)
 *   6   NCM Functional Descriptor           (NCM spec §5.2.1)
 *  13   CDC Ethernet Functional Descriptor  (full 13-byte form with MAC string idx)
 *   7   Interrupt IN EP (notification)
 *   9   Data Interface alt-0 (no endpoints)
 *   9   Data Interface alt-1 (bulk IN + OUT)
 *   7   Bulk OUT endpoint
 *   7   Bulk IN  endpoint
 * ----
 *  94 bytes total
 */
#define CDC_NCM_CONFIG_DESC_SIZE            94U

/* ---- NTB (Network Transfer Block) sizes ----
 * Keep NTB_IN_MAX_SIZE large enough for multiple datagrams per NTB.
 * LwIP default MTU is 1500 bytes; a 4 KB NTB can carry ~2-3 frames.
 * Increase if you need higher throughput.
 */
#ifndef CDC_NCM_NTB_IN_MAX_SIZE
#define CDC_NCM_NTB_IN_MAX_SIZE             4096U
#endif
#ifndef CDC_NCM_NTB_OUT_MAX_SIZE
#define CDC_NCM_NTB_OUT_MAX_SIZE            4096U
#endif

/* Ethernet receive buffer: one assembled Ethernet frame */
#define CDC_NCM_ETH_MAX_SEGSZE              1514U

/* ---- NTB-16 header / pointer structures (NCM spec §3.2, §3.3) ---- */
/* NTH16: NTB Transfer Header (16-bit) */
#define CDC_NCM_NTH16_SIGNATURE             0x484D434EU  /* "NCMH" */
/* NDP16: NTB Datagram Pointer (16-bit) */
#define CDC_NCM_NDP16_SIGNATURE_NCM0        0x304D434EU  /* "NCM0" */
#define CDC_NCM_NDP16_SIGNATURE_NCM1        0x314D434EU  /* "NCM1" */

/* Maximum number of datagrams per NTB (IN direction) */
#ifndef CDC_NCM_MAX_DATAGRAMS_PER_NTB
#define CDC_NCM_MAX_DATAGRAMS_PER_NTB       4U
#endif

/* ---- CDC NCM class-specific requests (NCM spec §6.2) ---- */
#define CDC_NCM_SET_ETHERNET_MULTICAST_FILTERS      0x40U
#define CDC_NCM_SET_ETHERNET_POWER_MGMT_FILTER      0x41U
#define CDC_NCM_GET_ETHERNET_POWER_MGMT_FILTER      0x42U
#define CDC_NCM_SET_ETHERNET_PACKET_FILTER          0x43U
#define CDC_NCM_GET_ETHERNET_STATISTIC              0x44U
#define CDC_NCM_GET_NTB_PARAMETERS                  0x80U
#define CDC_NCM_GET_NET_ADDRESS                     0x81U
#define CDC_NCM_SET_NET_ADDRESS                     0x82U
#define CDC_NCM_GET_NTB_FORMAT                      0x83U
#define CDC_NCM_SET_NTB_FORMAT                      0x84U
#define CDC_NCM_GET_NTB_INPUT_SIZE                  0x85U
#define CDC_NCM_SET_NTB_INPUT_SIZE                  0x86U
#define CDC_NCM_GET_MAX_DATAGRAM_SIZE               0x87U
#define CDC_NCM_SET_MAX_DATAGRAM_SIZE               0x88U
#define CDC_NCM_GET_CRC_MODE                        0x89U
#define CDC_NCM_SET_CRC_MODE                        0x8AU

/* ---- CDC notifications ---- */
#define CDC_NCM_NOTIFY_NETWORK_CONNECTION           0x00U
#define CDC_NCM_NOTIFY_RESPONSE_AVAILABLE           0x01U
#define CDC_NCM_NOTIFY_CONNECTION_SPEED_CHANGE      0x2AU

#define CDC_NCM_NET_DISCONNECTED                    0x00U
#define CDC_NCM_NET_CONNECTED                       0x01U

/* MAC String index in composite string table */
#define CDC_NCM_MAC_STRING_INDEX                    0x06U

/* bmRequestType for ECM/NCM notifications */
#define CDC_NCM_BMREQUEST_TYPE_NCM                  0xA1U

/**
  * @}
  */

/** @defgroup usbd_cdc_ncm_Exported_TypesDefinitions
  * @{
  */

/* NTB Parameters structure returned by GET_NTB_PARAMETERS (NCM spec Table 6-3) */
typedef struct __PACKED
{
  uint16_t wLength;               /* Size of this structure = 28 */
  uint16_t bmNtbFormatsSupported; /* bit0 = NTB-16; bit1 = NTB-32 */
  uint32_t dwNtbInMaxSize;        /* Max NTB size, IN direction  */
  uint16_t wNdpInDivisor;         /* NDP alignment divisor (IN)  */
  uint16_t wNdpInPayloadRemainder;/* NDP alignment remainder (IN) */
  uint16_t wNdpInAlignment;       /* Must be power of 2, >= 4    */
  uint16_t wReserved;
  uint32_t dwNtbOutMaxSize;       /* Max NTB size, OUT direction */
  uint16_t wNdpOutDivisor;        /* NDP alignment divisor (OUT) */
  uint16_t wNdpOutPayloadRemainder;
  uint16_t wNdpOutAlignment;
  uint16_t wNtbOutMaxDatagrams;   /* Max datagrams per OUT NTB   */
} USBD_CDC_NCM_NtbParametersTypeDef;

/* NTH16: 16-bit NTB Transfer Header (NCM spec §3.2.1) */
typedef struct __PACKED
{
  uint32_t dwSignature;   /* 0x484D434E "NCMH" */
  uint16_t wHeaderLength; /* = 12 */
  uint16_t wSequence;
  uint16_t wBlockLength;  /* Total NTB length in bytes */
  uint16_t wNdpIndex;     /* Offset of first NDP16     */
} USBD_CDC_NCM_NTH16TypeDef;

/* NDP16: 16-bit Datagram Pointer entry (one per datagram + terminating zero) */
typedef struct __PACKED
{
  uint16_t wDatagramIndex;  /* Offset of datagram from start of NTB; 0 = end */
  uint16_t wDatagramLength; /* Length of datagram; 0 = end                   */
} USBD_CDC_NCM_NDP16DatgramTypeDef;

/* NDP16 header + pointer table */
typedef struct __PACKED
{
  uint32_t dwSignature;   /* NCM0 or NCM1 */
  uint16_t wLength;       /* Header length = 8 + 4*wDatagramCount */
  uint16_t wNextNdpIndex; /* 0 = no more NDPs */
  USBD_CDC_NCM_NDP16DatgramTypeDef Datagram[CDC_NCM_MAX_DATAGRAMS_PER_NTB + 1U]; /* +1 terminator */
} USBD_CDC_NCM_NDP16TypeDef;

/* Application interface callbacks */
typedef struct
{
  int8_t (*Init)(void);
  int8_t (*DeInit)(void);
  int8_t (*Control)(uint8_t cmd, uint8_t *pbuf, uint16_t length);
  int8_t (*Receive)(uint8_t *Buf, uint32_t *Len);
  int8_t (*TransmitCplt)(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
  int8_t (*Process)(USBD_HandleTypeDef *pdev);
} USBD_CDC_NCM_ItfTypeDef;

/* Internal class handle */
typedef struct
{
  /* OUT (host→device) NTB assembly */
  uint8_t  RxBuffer[CDC_NCM_NTB_OUT_MAX_SIZE];
  uint32_t RxLength;

  /* IN (device→host) NTB build buffer */
  uint8_t  TxNtbBuffer[CDC_NCM_NTB_IN_MAX_SIZE];
  uint32_t TxNtbLength;

  /* Notification */
  uint8_t  NotifyBuffer[16];
  uint32_t NotifyLength;

  /* Host-negotiated NTB input size */
  uint32_t NtbInMaxSize;

  uint16_t TxSequence;

  __IO uint32_t TxState;
  __IO uint32_t RxState;
  __IO uint32_t MaxPcktLen;
} USBD_CDC_NCM_HandleTypeDef;

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef USBD_CDC_NCM;

extern uint8_t CDC_NCM_IN_EP;
extern uint8_t CDC_NCM_OUT_EP;
extern uint8_t CDC_NCM_CMD_EP;
extern uint8_t CDC_NCM_CMD_ITF_NBR;
extern uint8_t CDC_NCM_COM_ITF_NBR;
extern uint8_t CDC_NCM_STR_DESC_IDX;

/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */

uint8_t USBD_CDC_NCM_RegisterInterface(USBD_HandleTypeDef *pdev,
                                       USBD_CDC_NCM_ItfTypeDef *fops);

uint8_t USBD_CDC_NCM_SetTxBuffer(USBD_HandleTypeDef *pdev,
                                  uint8_t *pbuff, uint32_t length);

uint8_t USBD_CDC_NCM_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff);

uint8_t USBD_CDC_NCM_ReceivePacket(USBD_HandleTypeDef *pdev);

uint8_t USBD_CDC_NCM_TransmitPacket(USBD_HandleTypeDef *pdev);

uint8_t USBD_CDC_NCM_SendNotification(USBD_HandleTypeDef *pdev,
                                       uint8_t value);

void    USBD_Update_CDC_NCM_DESC(uint8_t *desc,
                                  uint8_t cmd_itf,
                                  uint8_t com_itf,
                                  uint8_t in_ep,
                                  uint8_t out_ep,
                                  uint8_t cmd_ep,
                                  uint8_t str_idx);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USB_CDC_NCM_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
