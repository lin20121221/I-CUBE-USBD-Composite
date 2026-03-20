/**
  ******************************************************************************
  * @file    usbd_cdc_eem.c
  * @author  AL94
  * @brief   USB CDC EEM (Ethernet Emulation Model) class driver.
  *
  *          CDC EEM uses a single bulk interface (no notification endpoint).
  *          Every USB transfer carries one or more EEM packets, each prefixed
  *          by a 2-byte EEM header:
  *
  *          Data packet  (bit15 = 0):
  *            [15]   = 0
  *            [14]   = bmCRC  (1 = CRC not calculated / sentinel 0xDEADBEEF)
  *            [13:0] = length of the following Ethernet frame in bytes
  *
  *          Command packet (bit15 = 1):
  *            [15]   = 1
  *            [14:11]= EEM command code
  *            [10:0] = command-specific parameter
  *
  *          Reference: USB Communications Class Subclass Specification for
  *          Ethernet Emulation Model Devices, Revision 1.0 (February 2, 2005).
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_eem.h"
#include "usbd_ctlreq.h"
#include "usbd_cdc_eem_if.h"

/* Default endpoint / interface assignments (overridden by USBD_Update_CDC_EEM_DESC) */
#define _CDC_EEM_IN_EP       0x81U
#define _CDC_EEM_OUT_EP      0x01U
#define _CDC_EEM_ITF_NBR     0x00U
#define _CDC_EEM_STR_DESC_IDX 0x00U

uint8_t CDC_EEM_IN_EP        = _CDC_EEM_IN_EP;
uint8_t CDC_EEM_OUT_EP       = _CDC_EEM_OUT_EP;
uint8_t CDC_EEM_ITF_NBR      = _CDC_EEM_ITF_NBR;
uint8_t CDC_EEM_STR_DESC_IDX = _CDC_EEM_STR_DESC_IDX;

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CDC_EEM
  * @brief usbd CDC EEM module
  * @{
  */

/** @defgroup USBD_CDC_EEM_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_CDC_EEM_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_EEM_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_EEM_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_CDC_EEM_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_EEM_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t *USBD_CDC_EEM_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_EEM_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_EEM_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_EEM_GetDeviceQualifierDescriptor(uint16_t *length);

/**
  * @}
  */

/** @defgroup USBD_CDC_EEM_Private_Variables
  * @{
  */

/* Static handle – avoids heap allocation like other classes in this framework */
static USBD_CDC_EEM_HandleTypeDef CDC_EEM_Instance;

/* CDC EEM class callbacks structure */
USBD_ClassTypeDef USBD_CDC_EEM =
{
  USBD_CDC_EEM_Init,
  USBD_CDC_EEM_DeInit,
  USBD_CDC_EEM_Setup,
  NULL,                                   /* EP0_TxSent    */
  NULL,                                   /* EP0_RxReady   */
  USBD_CDC_EEM_DataIn,
  USBD_CDC_EEM_DataOut,
  NULL,                                   /* SOF           */
  NULL,                                   /* IsoINIncomplete  */
  NULL,                                   /* IsoOUTIncomplete */
  USBD_CDC_EEM_GetHSCfgDesc,
  USBD_CDC_EEM_GetFSCfgDesc,
  USBD_CDC_EEM_GetOtherSpeedCfgDesc,
  USBD_CDC_EEM_GetDeviceQualifierDescriptor,
};

/* --------------------------------------------------------------------------
 * Descriptor layout (FS & HS share the same template):
 *
 *  [0..8]   Configuration Descriptor  (9 bytes)
 *  [9..16]  IAD Descriptor            (8 bytes)
 *  [17..25] Interface Descriptor       (9 bytes)
 *  [26..32] Endpoint OUT Descriptor    (7 bytes)
 *  [33..39] Endpoint IN  Descriptor    (7 bytes)
 *  Total: 40 bytes  = CDC_EEM_CONFIG_DESC_SIZE
 * --------------------------------------------------------------------------
 *
 *  IAD offsets used by USBD_Update_CDC_EEM_DESC:
 *    desc[9+2]  = bFirstInterface  → ITF_NBR
 *    desc[9+7]  = iFunction        → str_idx
 *  Interface descriptor offsets (base = 17):
 *    desc[17+2] = bInterfaceNumber → ITF_NBR
 *    desc[17+8] = iInterface       → str_idx
 *  EP OUT (base = 26):
 *    desc[26+2] = bEndpointAddress → OUT_EP
 *  EP IN  (base = 33):
 *    desc[33+2] = bEndpointAddress → IN_EP
 * --------------------------------------------------------------------------
 */

/* HS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_EEM_CfgHSDesc[CDC_EEM_CONFIG_DESC_SIZE] __ALIGN_END =
{
  /* Configuration Descriptor */
  0x09,                              /* bLength */
  USB_DESC_TYPE_CONFIGURATION,       /* bDescriptorType */
  LOBYTE(CDC_EEM_CONFIG_DESC_SIZE),  /* wTotalLength lo */
  HIBYTE(CDC_EEM_CONFIG_DESC_SIZE),  /* wTotalLength hi */
  0x01,                              /* bNumInterfaces: 1 */
  0x01,                              /* bConfigurationValue */
  0x00,                              /* iConfiguration */
#if (USBD_SELF_POWERED == 1U)
  0xC0,                              /* bmAttributes: Self + Bus powered */
#else
  0x80,                              /* bmAttributes: Bus powered */
#endif
  USBD_MAX_POWER,                    /* MaxPower */

  /* IAD Descriptor */
  0x08,                              /* bLength */
  0x0B,                              /* bDescriptorType: IAD */
  _CDC_EEM_ITF_NBR,                  /* bFirstInterface */
  0x01,                              /* bInterfaceCount */
  0x02,                              /* bFunctionClass: CDC */
  0x0C,                              /* bFunctionSubClass: EEM */
  0x07,                              /* bFunctionProtocol: EEM */
  _CDC_EEM_STR_DESC_IDX,             /* iFunction */

  /* Interface Descriptor */
  0x09,                              /* bLength */
  USB_DESC_TYPE_INTERFACE,           /* bDescriptorType */
  _CDC_EEM_ITF_NBR,                  /* bInterfaceNumber */
  0x00,                              /* bAlternateSetting */
  0x02,                              /* bNumEndpoints: 2 (bulk IN + bulk OUT) */
  0x02,                              /* bInterfaceClass: CDC */
  0x0C,                              /* bInterfaceSubClass: EEM */
  0x07,                              /* bInterfaceProtocol: EEM */
  _CDC_EEM_STR_DESC_IDX,             /* iInterface */

  /* Endpoint OUT Descriptor */
  0x07,                              /* bLength */
  USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType */
  _CDC_EEM_OUT_EP,                   /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_EEM_DATA_HS_MAX_PACKET_SIZE),
  HIBYTE(CDC_EEM_DATA_HS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: N/A for Bulk */

  /* Endpoint IN Descriptor */
  0x07,                              /* bLength */
  USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType */
  _CDC_EEM_IN_EP,                    /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_EEM_DATA_HS_MAX_PACKET_SIZE),
  HIBYTE(CDC_EEM_DATA_HS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: N/A for Bulk */
};

/* FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_EEM_CfgFSDesc[CDC_EEM_CONFIG_DESC_SIZE] __ALIGN_END =
{
  /* Configuration Descriptor */
  0x09,
  USB_DESC_TYPE_CONFIGURATION,
  LOBYTE(CDC_EEM_CONFIG_DESC_SIZE),
  HIBYTE(CDC_EEM_CONFIG_DESC_SIZE),
  0x01,
  0x01,
  0x00,
#if (USBD_SELF_POWERED == 1U)
  0xC0,
#else
  0x80,
#endif
  USBD_MAX_POWER,

  /* IAD */
  0x08,
  0x0B,
  _CDC_EEM_ITF_NBR,
  0x01,
  0x02,
  0x0C,
  0x07,
  _CDC_EEM_STR_DESC_IDX,

  /* Interface */
  0x09,
  USB_DESC_TYPE_INTERFACE,
  _CDC_EEM_ITF_NBR,
  0x00,
  0x02,
  0x02,
  0x0C,
  0x07,
  _CDC_EEM_STR_DESC_IDX,

  /* EP OUT */
  0x07,
  USB_DESC_TYPE_ENDPOINT,
  _CDC_EEM_OUT_EP,
  0x02,
  LOBYTE(CDC_EEM_DATA_FS_MAX_PACKET_SIZE),
  HIBYTE(CDC_EEM_DATA_FS_MAX_PACKET_SIZE),
  0x00,

  /* EP IN */
  0x07,
  USB_DESC_TYPE_ENDPOINT,
  _CDC_EEM_IN_EP,
  0x02,
  LOBYTE(CDC_EEM_DATA_FS_MAX_PACKET_SIZE),
  HIBYTE(CDC_EEM_DATA_FS_MAX_PACKET_SIZE),
  0x00,
};

/* Other-speed Configuration Descriptor (same layout as FS) */
__ALIGN_BEGIN static uint8_t USBD_CDC_EEM_OtherSpeedCfgDesc[CDC_EEM_CONFIG_DESC_SIZE] __ALIGN_END =
{
  0x09,
  USB_DESC_TYPE_CONFIGURATION,
  LOBYTE(CDC_EEM_CONFIG_DESC_SIZE),
  HIBYTE(CDC_EEM_CONFIG_DESC_SIZE),
  0x01,
  0x01,
  0x00,
#if (USBD_SELF_POWERED == 1U)
  0xC0,
#else
  0x80,
#endif
  USBD_MAX_POWER,

  0x08,
  0x0B,
  _CDC_EEM_ITF_NBR,
  0x01,
  0x02,
  0x0C,
  0x07,
  _CDC_EEM_STR_DESC_IDX,

  0x09,
  USB_DESC_TYPE_INTERFACE,
  _CDC_EEM_ITF_NBR,
  0x00,
  0x02,
  0x02,
  0x0C,
  0x07,
  _CDC_EEM_STR_DESC_IDX,

  0x07,
  USB_DESC_TYPE_ENDPOINT,
  _CDC_EEM_OUT_EP,
  0x02,
  0x40,
  0x00,
  0x00,

  0x07,
  USB_DESC_TYPE_ENDPOINT,
  _CDC_EEM_IN_EP,
  0x02,
  0x40,
  0x00,
  0x00,
};

/* Device Qualifier Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_EEM_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */

/** @defgroup USBD_CDC_EEM_Private_Functions
  * @{
  */

/**
  * @brief  USBD_CDC_EEM_Init
  *         Initialize the CDC EEM interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_EEM_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  USBD_CDC_EEM_HandleTypeDef *heem = &CDC_EEM_Instance;

  pdev->pClassData_CDC_EEM = (void *)heem;

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    (void)USBD_LL_OpenEP(pdev, CDC_EEM_IN_EP,  USBD_EP_TYPE_BULK, CDC_EEM_DATA_HS_IN_PACKET_SIZE);
    (void)USBD_LL_OpenEP(pdev, CDC_EEM_OUT_EP, USBD_EP_TYPE_BULK, CDC_EEM_DATA_HS_OUT_PACKET_SIZE);
    heem->MaxPcktLen = CDC_EEM_DATA_HS_MAX_PACKET_SIZE;
  }
  else
  {
    (void)USBD_LL_OpenEP(pdev, CDC_EEM_IN_EP,  USBD_EP_TYPE_BULK, CDC_EEM_DATA_FS_IN_PACKET_SIZE);
    (void)USBD_LL_OpenEP(pdev, CDC_EEM_OUT_EP, USBD_EP_TYPE_BULK, CDC_EEM_DATA_FS_OUT_PACKET_SIZE);
    heem->MaxPcktLen = CDC_EEM_DATA_FS_MAX_PACKET_SIZE;
  }

  pdev->ep_in[CDC_EEM_IN_EP   & 0x0FU].is_used = 1U;
  pdev->ep_out[CDC_EEM_OUT_EP & 0x0FU].is_used = 1U;

  /* Reset frame-assembly state */
  heem->EthRxLength    = 0U;
  heem->EthFrameLength = 0U;
  heem->TxState        = 0U;
  heem->RxState        = 0U;

  /* Call application Init */
  ((USBD_CDC_EEM_ItfTypeDef *)pdev->pUserData_CDC_EEM)->Init();

  /* Arm first OUT transfer */
  (void)USBD_LL_PrepareReceive(pdev, CDC_EEM_OUT_EP, heem->RxBuffer, heem->MaxPcktLen);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_EEM_DeInit
  *         Deinitialise the CDC EEM layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_EEM_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  (void)USBD_LL_CloseEP(pdev, CDC_EEM_IN_EP);
  (void)USBD_LL_CloseEP(pdev, CDC_EEM_OUT_EP);

  pdev->ep_in[CDC_EEM_IN_EP   & 0x0FU].is_used = 0U;
  pdev->ep_out[CDC_EEM_OUT_EP & 0x0FU].is_used = 0U;

  if (pdev->pClassData_CDC_EEM != NULL)
  {
    ((USBD_CDC_EEM_ItfTypeDef *)pdev->pUserData_CDC_EEM)->DeInit();
    pdev->pClassData_CDC_EEM = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_EEM_Setup
  *         Handle CDC EEM specific requests.
  *         EEM defines no class-specific control requests; we only respond to
  *         the mandatory standard interface requests.
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_CDC_EEM_Setup(USBD_HandleTypeDef *pdev,
                                  USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;
  uint16_t status_info   = 0U;
  uint8_t  ifalt         = 0U;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_STATUS:
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
      }
      else
      {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_GET_INTERFACE:
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        (void)USBD_CtlSendData(pdev, &ifalt, 1U);
      }
      else
      {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_SET_INTERFACE:
      if (pdev->dev_state != USBD_STATE_CONFIGURED)
      {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_CLEAR_FEATURE:
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
    }
    break;

  default:
    USBD_CtlError(pdev, req);
    ret = USBD_FAIL;
    break;
  }

  return (uint8_t)ret;
}

/**
  * @brief  USBD_CDC_EEM_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_CDC_EEM_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_EEM_HandleTypeDef *heem = (USBD_CDC_EEM_HandleTypeDef *)pdev->pClassData_CDC_EEM;
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;

  if (heem == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (epnum == (CDC_EEM_IN_EP & 0x7FU))
  {
    /* If we just sent a full-size packet, send a ZLP to terminate */
    if ((pdev->ep_in[epnum].total_length > 0U) &&
        ((pdev->ep_in[epnum].total_length % hpcd->IN_ep[epnum].maxpacket) == 0U))
    {
      pdev->ep_in[epnum].total_length = 0U;
      (void)USBD_LL_Transmit(pdev, epnum, NULL, 0U);
    }
    else
    {
      heem->TxState = 0U;
      if (((USBD_CDC_EEM_ItfTypeDef *)pdev->pUserData_CDC_EEM)->TransmitCplt != NULL)
      {
        ((USBD_CDC_EEM_ItfTypeDef *)pdev->pUserData_CDC_EEM)->TransmitCplt(
            heem->TxBuffer, &heem->TxLength, epnum);
      }
    }
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_EEM_DataOut
  *         Data received on non-control Out endpoint.
  *         Parse EEM in-band framing and deliver complete Ethernet frames
  *         to the application via the Receive callback.
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_CDC_EEM_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_EEM_HandleTypeDef *heem = (USBD_CDC_EEM_HandleTypeDef *)pdev->pClassData_CDC_EEM;

  if (heem == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (epnum != CDC_EEM_OUT_EP)
  {
    return (uint8_t)USBD_FAIL;
  }

  uint32_t usbLen = USBD_LL_GetRxDataSize(pdev, epnum);
  uint8_t *pkt    = heem->RxBuffer;
  uint32_t pos    = 0U;

  /* A single USB transfer may contain multiple concatenated EEM packets.
     Walk through all of them. */
  while (pos + 1U < usbLen)
  {
    uint16_t hdr = (uint16_t)pkt[pos] | ((uint16_t)pkt[pos + 1U] << 8U);
    pos += 2U;

    if ((hdr & 0x8000U) != 0U)
    {
      /* ---- EEM command packet ---- */
      /* We acknowledge Echo but ignore other commands silently.
         No data payload follows a command header. */
      uint8_t cmd = (uint8_t)((hdr >> 11U) & 0x0FU);
      if (cmd == CDC_EEM_CMD_ECHO)
      {
        /* Echo: the 11-bit param carries the echo data length.
           For simplicity we skip those bytes. */
        uint32_t echoLen = (uint32_t)(hdr & 0x07FFU);
        pos += echoLen;
      }
      /* All other commands: no payload, nothing to skip */
    }
    else
    {
      /* ---- EEM data packet ---- */
      uint32_t frameLen = (uint32_t)(hdr & 0x3FFFU);  /* bits[13:0] */

      if (frameLen == 0U)
      {
        /* Zero-length EEM data packet = ZLP sentinel; skip */
        continue;
      }

      /* Guard against runaway length */
      if (frameLen > CDC_EEM_ETH_MAX_SEGSZE + 4U)   /* +4 for optional CRC */
      {
        /* Malformed; reset accumulator and bail */
        heem->EthRxLength    = 0U;
        heem->EthFrameLength = 0U;
        break;
      }

      /* How many bytes of this frame are in the current USB packet? */
      uint32_t avail = usbLen - pos;
      uint32_t chunk = (avail < frameLen) ? avail : frameLen;

      /* Copy chunk into assembly buffer */
      if ((heem->EthRxLength + chunk) <= sizeof(heem->EthRxBuffer))
      {
        (void)USBD_memcpy(heem->EthRxBuffer + heem->EthRxLength,
                          pkt + pos,
                          chunk);
        heem->EthRxLength += chunk;
      }

      pos += chunk;

      if (heem->EthRxLength >= frameLen)
      {
        /* Complete Ethernet frame received – strip 4-byte sentinel CRC if present.
           The EEM spec says the CRC field is always present; when bmCRC=1 it
           contains the sentinel value 0xDEADBEEF (we simply drop it). */
        uint32_t ethPayload = frameLen;
        if (ethPayload >= 4U)
        {
          ethPayload -= 4U;  /* remove trailing CRC / sentinel */
        }

        /* Deliver to application */
        ((USBD_CDC_EEM_ItfTypeDef *)pdev->pUserData_CDC_EEM)->Receive(
            heem->EthRxBuffer, &ethPayload);

        /* Reset accumulator */
        heem->EthRxLength    = 0U;
        heem->EthFrameLength = 0U;
      }
    }
  }

  /* Re-arm OUT endpoint for next USB packet */
  (void)USBD_LL_PrepareReceive(pdev, CDC_EEM_OUT_EP,
                               heem->RxBuffer,
                               heem->MaxPcktLen);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_EEM_GetFSCfgDesc
  * @param  length : pointer to data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_EEM_GetFSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_EEM_CfgFSDesc);
  return USBD_CDC_EEM_CfgFSDesc;
}

/**
  * @brief  USBD_CDC_EEM_GetHSCfgDesc
  * @param  length : pointer to data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_EEM_GetHSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_EEM_CfgHSDesc);
  return USBD_CDC_EEM_CfgHSDesc;
}

/**
  * @brief  USBD_CDC_EEM_GetOtherSpeedCfgDesc
  * @param  length : pointer to data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_EEM_GetOtherSpeedCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_EEM_OtherSpeedCfgDesc);
  return USBD_CDC_EEM_OtherSpeedCfgDesc;
}

/**
  * @brief  USBD_CDC_EEM_GetDeviceQualifierDescriptor
  * @param  length : pointer to data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_EEM_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_EEM_DeviceQualifierDesc);
  return USBD_CDC_EEM_DeviceQualifierDesc;
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

/**
  * @brief  USBD_CDC_EEM_RegisterInterface
  * @param  pdev   : device instance
  * @param  fops   : interface callbacks
  * @retval status
  */
uint8_t USBD_CDC_EEM_RegisterInterface(USBD_HandleTypeDef *pdev,
                                       USBD_CDC_EEM_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }
  pdev->pUserData_CDC_EEM = fops;
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_EEM_SetTxBuffer
  *         Set the pointer and length of data to transmit.
  *         The driver prepends the 2-byte EEM data header automatically.
  * @param  pdev   : device instance
  * @param  pbuff  : pointer to Ethernet frame (without EEM header)
  * @param  length : frame length in bytes (excluding 4-byte CRC sentinel)
  * @retval status
  */
uint8_t USBD_CDC_EEM_SetTxBuffer(USBD_HandleTypeDef *pdev,
                                 uint8_t *pbuff, uint32_t length)
{
  USBD_CDC_EEM_HandleTypeDef *heem = (USBD_CDC_EEM_HandleTypeDef *)pdev->pClassData_CDC_EEM;
  if (heem == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }
  heem->TxBuffer = pbuff;
  heem->TxLength = length;
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_EEM_SetRxBuffer
  * @param  pdev  : device instance
  * @param  pbuff : RX buffer (unused – driver uses internal buffer; kept for API symmetry)
  * @retval status
  */
uint8_t USBD_CDC_EEM_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff)
{
  UNUSED(pbuff);
  if (pdev->pClassData_CDC_EEM == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_EEM_ReceivePacket
  *         Re-arm the OUT endpoint (call after processing a received frame).
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_CDC_EEM_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_EEM_HandleTypeDef *heem = (USBD_CDC_EEM_HandleTypeDef *)pdev->pClassData_CDC_EEM;
  if (heem == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }
  return (uint8_t)USBD_LL_PrepareReceive(pdev, CDC_EEM_OUT_EP,
                                         heem->RxBuffer, heem->MaxPcktLen);
}

/**
  * @brief  USBD_CDC_EEM_TransmitPacket
  *         Transmit an Ethernet frame wrapped in an EEM data header.
  *         The caller must have set TxBuffer / TxLength via USBD_CDC_EEM_SetTxBuffer.
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_CDC_EEM_TransmitPacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_EEM_HandleTypeDef *heem = (USBD_CDC_EEM_HandleTypeDef *)pdev->pClassData_CDC_EEM;
  USBD_StatusTypeDef ret = USBD_BUSY;

  if (heem == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (heem->TxState == 0U)
  {
    /* Include 4-byte sentinel CRC (0xDEADBEEF) in the EEM length field */
    uint32_t eemPayloadLen = heem->TxLength + 4U;

    /* Build 2-byte EEM header in the two bytes immediately before TxBuffer.
       The application MUST reserve 2 bytes before the Ethernet frame for
       the EEM header (or use a separate header buffer – here we use a small
       static staging area to avoid requiring the application to pre-allocate). */
    static uint8_t eemTxStaging[CDC_EEM_DATA_BUFFER_SIZE + 6U];  /* header + frame + CRC */

    if (eemPayloadLen > CDC_EEM_ETH_MAX_SEGSZE + 4U)
    {
      return (uint8_t)USBD_FAIL;
    }

    /* EEM data header: bit15=0 (data), bit14=1 (CRC not calculated), bits[13:0]=length */
    uint16_t eemHdr = (uint16_t)(0x4000U | (eemPayloadLen & 0x3FFFU));
    eemTxStaging[0] = (uint8_t)(eemHdr & 0xFFU);
    eemTxStaging[1] = (uint8_t)((eemHdr >> 8U) & 0xFFU);

    /* Copy Ethernet frame */
    (void)USBD_memcpy(&eemTxStaging[2], heem->TxBuffer, heem->TxLength);

    /* Sentinel CRC = 0xDEADBEEF (little-endian) */
    eemTxStaging[2U + heem->TxLength + 0U] = 0xEFU;
    eemTxStaging[2U + heem->TxLength + 1U] = 0xBEU;
    eemTxStaging[2U + heem->TxLength + 2U] = 0xADU;
    eemTxStaging[2U + heem->TxLength + 3U] = 0xDEU;

    uint32_t totalLen = 2U + eemPayloadLen;

    heem->TxState = 1U;

    pdev->ep_in[CDC_EEM_IN_EP & 0x0FU].total_length = totalLen;

    (void)USBD_LL_Transmit(pdev, CDC_EEM_IN_EP, eemTxStaging, totalLen);

    ret = USBD_OK;
  }

  return (uint8_t)ret;
}

/**
  * @brief  USBD_Update_CDC_EEM_DESC
  *         Patch the EEM configuration descriptor with the interface/endpoint
  *         numbers assigned by the composite framework.
  *
  *         Descriptor byte offsets (base = start of the configuration
  *         descriptor array, i.e. includes the 9-byte config header):
  *
  *           IAD (base 9):
  *             [9+2]  bFirstInterface  = itf
  *             [9+7]  iFunction        = str_idx
  *           Interface (base 17):
  *             [17+2] bInterfaceNumber = itf
  *             [17+8] iInterface       = str_idx
  *           EP OUT (base 26):
  *             [26+2] bEndpointAddress = out_ep
  *           EP IN (base 33):
  *             [33+2] bEndpointAddress = in_ep
  *
  * @param  desc    : pointer to the descriptor buffer (full config desc)
  * @param  itf     : interface number assigned by composite
  * @param  in_ep   : IN endpoint address
  * @param  out_ep  : OUT endpoint address
  * @param  str_idx : string index assigned by composite
  */
void USBD_Update_CDC_EEM_DESC(uint8_t *desc,
                              uint8_t  itf,
                              uint8_t  in_ep,
                              uint8_t  out_ep,
                              uint8_t  str_idx)
{
  /* IAD */
  desc[11] = itf;      /* bFirstInterface */
  desc[16] = str_idx;  /* iFunction       */

  /* Interface descriptor */
  desc[19] = itf;      /* bInterfaceNumber */
  desc[25] = str_idx;  /* iInterface       */

  /* EP OUT */
  desc[28] = out_ep;

  /* EP IN */
  desc[35] = in_ep;

  /* Update global EP/ITF tracking variables */
  CDC_EEM_IN_EP        = in_ep;
  CDC_EEM_OUT_EP       = out_ep;
  CDC_EEM_ITF_NBR      = itf;
  CDC_EEM_STR_DESC_IDX = str_idx;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
