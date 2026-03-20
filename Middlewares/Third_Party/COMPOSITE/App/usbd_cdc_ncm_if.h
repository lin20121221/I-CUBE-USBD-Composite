/**
  ******************************************************************************
  * @file    usbd_cdc_ncm_if.h
  * @author  AL94
  * @brief   Header for usbd_cdc_ncm_if.c
  *
  *          This file is the glue layer between the USB CDC NCM class driver
  *          and the application TCP/IP stack (LwIP).
  *
  *          ── Quick LwIP integration guide ──────────────────────────────────
  *
  *          1. Add these files to your build:
  *               Middlewares/Third_Party/COMPOSITE/Class/CDC_NCM/Src/usbd_cdc_ncm.c
  *               Middlewares/Third_Party/COMPOSITE/Class/CDC_NCM/Inc/usbd_cdc_ncm.h
  *               Middlewares/Third_Party/COMPOSITE/App/usbd_cdc_ncm_if.c
  *               Middlewares/Third_Party/COMPOSITE/App/usbd_cdc_ncm_if.h
  *
  *          2. In AL94.I-CUBE-USBD-COMPOSITE_conf.h set:
  *               #define _USBD_USE_CDC_NCM   1
  *
  *          3. In usbd_cdc_ncm_if.c, fill in the four LwIP placeholders:
  *               CDC_NCM_Itf_Init()       – call lwip_init() / netif_add()
  *               CDC_NCM_Itf_DeInit()     – notify link down
  *               CDC_NCM_Itf_Receive()    – pbuf_alloc + netif.input()
  *               CDC_NCM_Itf_Process()    – sys_check_timeouts() + TX pump
  *
  *          4. To transmit a frame from LwIP (netif->linkoutput):
  *               a. Call CDC_NCM_BuildNTB(frame_buf, frame_len) to wrap the
  *                  Ethernet frame in an NTB-16.
  *               b. Call USBD_CDC_NCM_TransmitPacket(&hUsbDevice).
  *
  *          5. Call USBD_CDC_NCM_RegisterInterface(&hUsbDevice, &USBD_CDC_NCM_fops)
  *             before USBD_Start() (already done in usb_device.c if NCM is
  *             enabled via the composite macro).
  *
  *          6. Call CDC_NCM_Itf_Process() periodically (e.g. from a FreeRTOS
  *             task or the main loop) to poll LwIP timers and drain the TX
  *             queue.
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
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_NCM_IF_H
#define __USBD_CDC_NCM_IF_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_ncm.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/** Maximum Ethernet segment size (excluding 4-byte CRC) */
#define CDC_NCM_ETH_MAX_SEGSZE_IF               1514U

/** Maximum number of TX trials before giving up when NCM is busy */
#define CDC_NCM_MAX_TX_WAIT_TRIALS              1000000U

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Build a minimal single-datagram NTB-16 in the internal TX buffer
  *         and schedule it for transmission.
  *
  *         Call this from your netif->linkoutput() LwIP callback:
  *
  *           err_t usb_ncm_linkoutput(struct netif *netif, struct pbuf *p)
  *           {
  *             uint8_t  frame[CDC_NCM_ETH_MAX_SEGSZE_IF];
  *             uint16_t len = 0U;
  *             struct pbuf *q;
  *             for (q = p; q != NULL; q = q->next) {
  *               memcpy(frame + len, q->payload, q->len);
  *               len += q->len;
  *             }
  *             CDC_NCM_Transmit(frame, len);
  *             return ERR_OK;
  *           }
  *
  * @param  Buf  Pointer to raw Ethernet frame (no EEM/NTB header).
  * @param  Len  Frame length in bytes.
  * @retval USBD_OK or USBD_BUSY
  */
uint8_t CDC_NCM_Transmit(uint8_t *Buf, uint32_t Len);

/**
  * @brief  Callback table registered with the CDC NCM class driver.
  *         Pass this to USBD_CDC_NCM_RegisterInterface().
  */
extern USBD_CDC_NCM_ItfTypeDef USBD_CDC_NCM_fops;

#endif /* __USBD_CDC_NCM_IF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
