/**
  ******************************************************************************
  * @file    usbd_cdc_ncm_if.c
  * @author  AL94
  * @brief   Source file for USBD CDC NCM interface.
  *
  *          This file is the glue between the USB CDC NCM class driver and
  *          the application TCP/IP stack (LwIP).
  *
  *          Every placeholder is clearly marked with:
  *            /* >>>  USER CODE: <description>  <<< */
  *
  *          Search for "USER CODE" to find all integration points.
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
#include "usbd_cdc_ncm_if.h"

/*
  ============================================================================
  LwIP integration – include the headers below when you are ready to connect.
  ============================================================================

  #include "lwip/netif.h"
  #include "lwip/pbuf.h"
  #include "lwip/timeouts.h"
  #include "lwip/etharp.h"
  #include "netif/ethernet.h"

  extern struct netif gnetif;   // your LwIP netif instance
*/

/* The USB device handle defined in usb_device.c */
extern USBD_HandleTypeDef hUsbDevice;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/*
  NTB-16 layout built by CDC_NCM_Transmit():

    ┌─────────────────────────────────┐  ← byte 0
    │  NTH16  (12 bytes)              │    NCMH signature + seq + block len
    ├─────────────────────────────────┤  ← byte 12
    │  Ethernet frame  (variable)     │    raw layer-2 frame
    ├─────────────────────────────────┤  ← aligned to 4 bytes
    │  NDP16  (8 + 4×2 bytes)         │    NCM0 + one datagram pointer + terminator
    └─────────────────────────────────┘
*/

/* Offset of the NDP16 block inside the NTB when there is exactly one frame.
 * NTH16 (12 bytes) + frame (variable, padded to 4-byte boundary) */
#define NCM_NTH16_SIZE    12U
#define NCM_NDP16_SIZE    (8U + 4U * 2U)   /* header + 1 datagram + terminator */

/* Private macro -------------------------------------------------------------*/
#define ALIGN4(x)  (((x) + 3U) & ~3U)

/* Private variables ---------------------------------------------------------*/

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
/* Internal NTB transmit buffer – shared between CDC_NCM_Transmit() and the
 * class driver.  Must be at least CDC_NCM_NTB_IN_MAX_SIZE bytes. */
__ALIGN_BEGIN static uint8_t NcmTxNtbBuf[CDC_NCM_NTB_IN_MAX_SIZE] __ALIGN_END;

static uint16_t NcmTxSequence = 0U;
static uint8_t  CDC_NCMInitialized = 0U;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_NCM_Itf_Init(void);
static int8_t CDC_NCM_Itf_DeInit(void);
static int8_t CDC_NCM_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_NCM_Itf_Receive(uint8_t *Buf, uint32_t *Len);
static int8_t CDC_NCM_Itf_TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
static int8_t CDC_NCM_Itf_Process(USBD_HandleTypeDef *pdev);

USBD_CDC_NCM_ItfTypeDef USBD_CDC_NCM_fops =
{
  CDC_NCM_Itf_Init,
  CDC_NCM_Itf_DeInit,
  CDC_NCM_Itf_Control,
  CDC_NCM_Itf_Receive,
  CDC_NCM_Itf_TransmitCplt,
  CDC_NCM_Itf_Process,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_NCM_Itf_Init
  *         Initialise the CDC NCM media low layer and the TCP/IP stack.
  *
  *         ── LwIP integration ──────────────────────────────────────────────
  *
  *         Replace the USER CODE block with something like:
  *
  *           lwip_init();
  *
  *           ip4_addr_t ipaddr, netmask, gw;
  *           IP4_ADDR(&ipaddr,  192, 168, 7, 1);
  *           IP4_ADDR(&netmask, 255, 255, 255, 0);
  *           IP4_ADDR(&gw,      0, 0, 0, 0);
  *
  *           netif_add(&gnetif, &ipaddr, &netmask, &gw,
  *                     NULL, usb_ncm_netif_init, ethernet_input);
  *           netif_set_default(&gnetif);
  *           netif_set_up(&gnetif);
  *
  *           // Send NETWORK_CONNECTION = connected notification
  *           USBD_CDC_NCM_SendNotification(&hUsbDevice, CDC_NCM_NET_CONNECTED);
  *
  * @retval 0 (USBD_OK)
  */
static int8_t CDC_NCM_Itf_Init(void)
{
  if (CDC_NCMInitialized == 0U)
  {
    /* >>> USER CODE: initialise LwIP stack and add USB NCM netif <<< */

    CDC_NCMInitialized = 1U;
  }

  return (0);
}

/**
  * @brief  CDC_NCM_Itf_DeInit
  *         De-initialise the CDC NCM media low layer.
  *
  *         ── LwIP integration ──────────────────────────────────────────────
  *           netif_set_link_down(&gnetif);
  *           netif_set_down(&gnetif);
  *
  * @retval 0 (USBD_OK)
  */
static int8_t CDC_NCM_Itf_DeInit(void)
{
  /* >>> USER CODE: notify TCP/IP stack that the link is down <<< */
  return (0);
}

/**
  * @brief  CDC_NCM_Itf_Control
  *         Handle CDC NCM class-specific control requests.
  *
  *         The class driver already handles GET_NTB_PARAMETERS and
  *         SET/GET_NTB_INPUT_SIZE internally.  Override here only if you need
  *         to act on other requests (e.g. SET_ETHERNET_PACKET_FILTER).
  *
  * @param  cmd     bRequest value
  * @param  pbuf    Data buffer (host→device phase, or NULL for no-data)
  * @param  length  Buffer length
  * @retval 0 (USBD_OK)
  */
static int8_t CDC_NCM_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  UNUSED(length);

  switch (cmd)
  {
    case CDC_NCM_SET_ETHERNET_PACKET_FILTER:
      /* >>> USER CODE: configure multicast/promiscuous filters <<< */
      /* pbuf[0..1] contains the filter bitmap (wValue from setup packet) */
      UNUSED(pbuf);
      break;

    case CDC_NCM_SET_NTB_INPUT_SIZE:
      /* Handled by the class driver; nothing to do here unless you want
       * to resize application buffers dynamically. */
      break;

    default:
      UNUSED(pbuf);
      break;
  }
  return (0);
}

/**
  * @brief  CDC_NCM_Itf_Receive
  *         Called by the class driver for each individual Ethernet datagram
  *         extracted from a received NTB.  The NTB framing has already been
  *         stripped; Buf points to a raw layer-2 Ethernet frame.
  *
  *         ── LwIP integration ──────────────────────────────────────────────
  *
  *           struct pbuf *p = pbuf_alloc(PBUF_RAW, (uint16_t)*Len, PBUF_POOL);
  *           if (p != NULL)
  *           {
  *             pbuf_take(p, Buf, (uint16_t)*Len);
  *             if (gnetif.input(p, &gnetif) != ERR_OK)
  *             {
  *               pbuf_free(p);
  *             }
  *           }
  *
  *         If you use NO_SYS=0 (FreeRTOS), call tcpip_input() instead:
  *           tcpip_input(p, &gnetif);
  *
  * @param  Buf  Pointer to the raw Ethernet frame
  * @param  Len  Pointer to frame length (set by driver before call)
  * @retval 0 (USBD_OK)
  */
static int8_t CDC_NCM_Itf_Receive(uint8_t *Buf, uint32_t *Len)
{
  /* >>> USER CODE: pass Ethernet frame to LwIP <<< */
  UNUSED(Buf);
  UNUSED(Len);

  return (0);
}

/**
  * @brief  CDC_NCM_Itf_TransmitCplt
  *         Called when an IN transfer (NTB to host) has completed.
  *         Use this to signal a TX-done semaphore if needed.
  *
  * @param  Buf   Buffer that was transmitted
  * @param  Len   Length of transmitted data
  * @param  epnum Endpoint number
  * @retval 0 (USBD_OK)
  */
static int8_t CDC_NCM_Itf_TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);

  /* >>> USER CODE: release TX semaphore / signal tx_done event <<< */

  return (0);
}

/**
  * @brief  CDC_NCM_Itf_Process
  *         Periodic processing hook – called from the USB device task or main
  *         loop by the composite driver.
  *
  *         ── LwIP integration ──────────────────────────────────────────────
  *
  *         Bare-metal (NO_SYS=1):
  *           sys_check_timeouts();
  *
  *         FreeRTOS (NO_SYS=0):
  *           This callback is called from the USB IRQ context.
  *           Do NOT call LwIP functions directly.  Instead, post a message
  *           to your LwIP task:
  *             osMessageQueuePut(lwipEventQueue, &evt, 0, 0);
  *
  *         TX pump example (bare-metal, NO_SYS=1):
  *           The netif->linkoutput hook calls CDC_NCM_Transmit(), which
  *           builds the NTB and calls USBD_CDC_NCM_TransmitPacket().
  *           No additional TX pumping is needed here for bare-metal.
  *
  * @param  pdev  USB device handle
  * @retval 0 (USBD_OK)
  */
static int8_t CDC_NCM_Itf_Process(USBD_HandleTypeDef *pdev)
{
  UNUSED(pdev);

  /* >>> USER CODE: poll TCP/IP stack timers (bare-metal only) <<< */
  /* Example: sys_check_timeouts(); */

  return (0);
}

/* ========================================================================== */
/* Public helper: build a single-datagram NTB-16 and transmit it              */
/* ========================================================================== */

/**
  * @brief  CDC_NCM_Transmit
  *         Wrap a single raw Ethernet frame in a minimal NTB-16 and send it
  *         to the host via USBD_CDC_NCM_TransmitPacket().
  *
  *         NTB-16 structure built here (single datagram, NCM0 NDP):
  *
  *           [0..11]   NTH16 header (signature NCMH, seq, block-len, NDP offset)
  *           [12..12+Len-1]  Raw Ethernet frame
  *           [padded to 4-byte boundary]
  *           [ndp_offset..ndp_offset+15]  NDP16 (NCM0, 1 datagram + terminator)
  *
  *         This function blocks up to CDC_NCM_MAX_TX_WAIT_TRIALS loop
  *         iterations waiting for the previous IN transfer to finish.
  *         In a FreeRTOS environment, replace the spin-wait with a semaphore
  *         pend in CDC_NCM_Itf_TransmitCplt().
  *
  * @param  Buf  Raw Ethernet frame (caller-owned, copied internally)
  * @param  Len  Frame length in bytes (must be <= CDC_NCM_ETH_MAX_SEGSZE_IF)
  * @retval USBD_OK on success, USBD_BUSY if the previous transfer timed out
  */
uint8_t CDC_NCM_Transmit(uint8_t *Buf, uint32_t Len)
{
  if (Len > CDC_NCM_ETH_MAX_SEGSZE_IF || Buf == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Spin-wait for previous NTB to finish (simple bare-metal approach) */
  uint32_t trials = 0U;
  USBD_CDC_NCM_HandleTypeDef *hncm =
      (USBD_CDC_NCM_HandleTypeDef *)(hUsbDevice.pClassData_CDC_NCM);

  while (hncm != NULL && hncm->TxState != 0U)
  {
    trials++;
    if (trials >= CDC_NCM_MAX_TX_WAIT_TRIALS)
    {
      return (uint8_t)USBD_BUSY;
    }
  }

  /* ---- Build NTB-16 ---- */
  (void)memset(NcmTxNtbBuf, 0, sizeof(NcmTxNtbBuf));

  /* Ethernet frame starts immediately after the NTH16 */
  uint16_t frame_offset = (uint16_t)NCM_NTH16_SIZE;
  /* NDP16 starts after the frame, 4-byte aligned */
  uint16_t ndp_offset   = (uint16_t)ALIGN4((uint32_t)frame_offset + Len);
  /* Total NTB block length */
  uint16_t block_len    = (uint16_t)(ndp_offset + NCM_NDP16_SIZE);

  if (block_len > CDC_NCM_NTB_IN_MAX_SIZE)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* NTH16 */
  USBD_CDC_NCM_NTH16TypeDef *nth = (USBD_CDC_NCM_NTH16TypeDef *)NcmTxNtbBuf;
  nth->dwSignature   = CDC_NCM_NTH16_SIGNATURE;
  nth->wHeaderLength = (uint16_t)NCM_NTH16_SIZE;
  nth->wSequence     = NcmTxSequence++;
  nth->wBlockLength  = block_len;
  nth->wNdpIndex     = ndp_offset;

  /* Ethernet frame payload */
  (void)memcpy(NcmTxNtbBuf + frame_offset, Buf, Len);

  /* NDP16 */
  USBD_CDC_NCM_NDP16TypeDef *ndp =
      (USBD_CDC_NCM_NDP16TypeDef *)(NcmTxNtbBuf + ndp_offset);
  ndp->dwSignature       = CDC_NCM_NDP16_SIGNATURE_NCM0;
  ndp->wLength           = (uint16_t)(8U + 4U * 2U); /* 1 datagram + terminator */
  ndp->wNextNdpIndex     = 0U;
  ndp->Datagram[0].wDatagramIndex  = frame_offset;
  ndp->Datagram[0].wDatagramLength = (uint16_t)Len;
  ndp->Datagram[1].wDatagramIndex  = 0U; /* terminator */
  ndp->Datagram[1].wDatagramLength = 0U;

  /* Hand the NTB to the class driver and transmit */
  (void)USBD_CDC_NCM_SetTxBuffer(&hUsbDevice, NcmTxNtbBuf, (uint32_t)block_len);
  return USBD_CDC_NCM_TransmitPacket(&hUsbDevice);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
