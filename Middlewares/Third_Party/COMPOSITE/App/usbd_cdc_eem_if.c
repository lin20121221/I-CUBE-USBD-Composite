/**
  ******************************************************************************
  * @file    usbd_cdc_eem_if.c
  * @author  AL94
  * @brief   Source file for USBD CDC EEM interface.
  *
  *          This file is the glue between the USB CDC EEM class driver and the
  *          application (typically a TCP/IP stack such as LwIP).
  *
  *          Replace the placeholder comments with real TCP/IP stack calls.
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_eem_if.h"

/* The USB device handle is defined in usb_device.c */
extern USBD_HandleTypeDef hUsbDevice;

/*
  Include LwIP or other TCP/IP stack headers here if needed.
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
/* Receive buffer: one complete Ethernet frame (class driver delivers assembled frames) */
__ALIGN_BEGIN static uint8_t UserRxBuffer[CDC_EEM_ETH_MAX_SEGSZE_IF + 100U] __ALIGN_END;

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
/* Transmit buffer: caller fills this then calls USBD_CDC_EEM_TransmitPacket() */
__ALIGN_BEGIN static uint8_t UserTxBuffer[CDC_EEM_ETH_MAX_SEGSZE_IF + 100U] __ALIGN_END;

static uint8_t CDC_EEMInitialized = 0U;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_EEM_Itf_Init(void);
static int8_t CDC_EEM_Itf_DeInit(void);
static int8_t CDC_EEM_Itf_Receive(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_EEM_Itf_TransmitCplt(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);
static int8_t CDC_EEM_Itf_Process(USBD_HandleTypeDef *pdev);

USBD_CDC_EEM_ItfTypeDef USBD_CDC_EEM_fops =
{
  CDC_EEM_Itf_Init,
  CDC_EEM_Itf_DeInit,
  CDC_EEM_Itf_Receive,
  CDC_EEM_Itf_TransmitCplt,
  CDC_EEM_Itf_Process,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_EEM_Itf_Init
  *         Initialises the CDC EEM media low layer.
  * @retval USBD_OK
  */
static int8_t CDC_EEM_Itf_Init(void)
{
  if (CDC_EEMInitialized == 0U)
  {
    /*
      Initialise the TCP/IP stack here (e.g. lwip_init(), netif_add() …).
    */
    CDC_EEMInitialized = 1U;
  }

  /* Point the class driver at the application Tx buffer.
     RxBuffer is managed internally by the class driver. */
  (void)USBD_CDC_EEM_SetTxBuffer(&hUsbDevice, UserTxBuffer, 0U);
  (void)USBD_CDC_EEM_SetRxBuffer(&hUsbDevice, UserRxBuffer);

  return (0);
}

/**
  * @brief  CDC_EEM_Itf_DeInit
  *         DeInitialises the CDC EEM media low layer.
  * @retval USBD_OK
  */
static int8_t CDC_EEM_Itf_DeInit(void)
{
  /*
    Notify application / TCP/IP stack that the link is down.
  */
  return (0);
}

/**
  * @brief  CDC_EEM_Itf_Receive
  *         Called by the class driver when a complete Ethernet frame has been
  *         received and assembled.  The 4-byte CRC/sentinel has already been
  *         stripped by the driver.
  * @param  Buf : pointer to the Ethernet frame
  * @param  Len : frame length in bytes (in/out; driver sets *Len before call)
  * @retval USBD_OK
  */
static int8_t CDC_EEM_Itf_Receive(uint8_t *Buf, uint32_t *Len)
{
  /*
    Pass the Ethernet frame to the TCP/IP stack.
    Example for LwIP:
      struct pbuf *p = pbuf_alloc(PBUF_RAW, (uint16_t)*Len, PBUF_POOL);
      if (p != NULL) {
        pbuf_take(p, Buf, (uint16_t)*Len);
        if (netif.input(p, &netif) != ERR_OK) { pbuf_free(p); }
      }
  */
  UNUSED(Buf);
  UNUSED(Len);

  return (0);
}

/**
  * @brief  CDC_EEM_Itf_TransmitCplt
  *         Called when an IN transfer (Ethernet frame to host) has completed.
  * @param  Buf   : the buffer that was transmitted
  * @param  Len   : length of the transmitted data
  * @param  epnum : endpoint number
  * @retval USBD_OK
  */
static int8_t CDC_EEM_Itf_TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);

  return (0);
}

/**
  * @brief  CDC_EEM_Itf_Process
  *         Periodic processing hook called from the USB device task / main loop.
  *         Use this to poll the TCP/IP stack and initiate outgoing transfers.
  * @param  pdev: pointer to the USB Device Handle
  * @retval USBD_OK
  */
static int8_t CDC_EEM_Itf_Process(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_EEM_HandleTypeDef *heem =
      (USBD_CDC_EEM_HandleTypeDef *)(pdev->pClassData_CDC_EEM);

  if (heem != NULL)
  {
    /*
      Poll the TCP/IP stack here.
      Example for LwIP: sys_check_timeouts();

      To transmit a frame:
        1. Copy frame data into UserTxBuffer
        2. USBD_CDC_EEM_SetTxBuffer(pdev, UserTxBuffer, frameLen);
        3. USBD_CDC_EEM_TransmitPacket(pdev);
    */
  }

  return (0);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
