/**
  ******************************************************************************
  * @file    usbd_midi.c
  * @author  AL94 / Community
  * @brief   USB MIDI 1.0 Class driver for STM32 Composite USB framework.
  *
  *          This driver implements USB MIDI 1.0 (USB Audio Class, MIDI Streaming
  *          subclass) for use within the AL94 I-CUBE-USBD-COMPOSITE framework.
  *
  *          The device presents itself as:
  *            - Audio Control Interface (informational, required by spec)
  *            - MIDI Streaming Interface with:
  *                - 1 Embedded MIDI IN Jack  (device receives MIDI from host)
  *                - 1 Embedded MIDI OUT Jack (device sends MIDI to host)
  *                - 1 External MIDI IN Jack  (connected to Embedded OUT)
  *                - 1 External MIDI OUT Jack (connected to Embedded IN)
  *                - 1 Bulk OUT endpoint (host -> device)
  *                - 1 Bulk IN endpoint  (device -> host)
  *
  *          MIDI data is transmitted as 4-byte USB-MIDI event packets per the
  *          USB MIDI 1.0 specification (Universal Serial Bus Device Class
  *          Definition for MIDI Devices, Release 1.0, November 1, 1999).
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_midi.h"
#include "usbd_ctlreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_MIDI
  * @brief usbd MIDI core module
  * @{
  */

/* Default endpoint/interface assignments (overridden by USBD_Update_MIDI_DESC) */
#define _MIDI_IN_EP_DEFAULT     0x81U
#define _MIDI_OUT_EP_DEFAULT    0x01U
#define _MIDI_AC_ITF_DEFAULT    0x00U
#define _MIDI_MS_ITF_DEFAULT    0x01U
#define _MIDI_STR_IDX_DEFAULT   0x00U

/* Assigned values (updated by USBD_Update_MIDI_DESC) */
uint8_t MIDI_IN_EP      = _MIDI_IN_EP_DEFAULT;
uint8_t MIDI_OUT_EP     = _MIDI_OUT_EP_DEFAULT;
uint8_t MIDI_AC_ITF_NBR = _MIDI_AC_ITF_DEFAULT;
uint8_t MIDI_MS_ITF_NBR = _MIDI_MS_ITF_DEFAULT;
uint8_t MIDI_STR_DESC_IDX = _MIDI_STR_IDX_DEFAULT;

/** @defgroup USBD_MIDI_Private_FunctionPrototypes
  * @{
  */
static uint8_t  USBD_MIDI_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_MIDI_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_MIDI_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t  USBD_MIDI_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_MIDI_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t *USBD_MIDI_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_MIDI_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_MIDI_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_MIDI_GetDeviceQualifierDescriptor(uint16_t *length);
/**
  * @}
  */

/** @defgroup USBD_MIDI_Private_Variables
  * @{
  */

/* Internal class handle */
static USBD_MIDI_HandleTypeDef MIDI_Class_Data;

/* Application-layer callback pointer */
static USBD_MIDI_ItfTypeDef *MIDI_fops = NULL;

/* USB Device Qualifier Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MIDI_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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

/*
 * Full USB MIDI Configuration Descriptor
 *
 * Layout (101 bytes total):
 *  [0x00] Configuration Descriptor (9 bytes)
 *  [0x09] Standard AC Interface Descriptor (9 bytes)
 *  [0x12] Class-Specific AC Interface Header Descriptor (9 bytes)
 *  [0x1B] Standard MS Interface Descriptor (9 bytes)
 *  [0x24] Class-Specific MS Interface Header Descriptor (7 bytes)
 *  [0x2B] MIDI IN Jack Descriptor - Embedded (6 bytes)
 *  [0x31] MIDI IN Jack Descriptor - External (6 bytes)
 *  [0x37] MIDI OUT Jack Descriptor - Embedded (9 bytes)
 *  [0x40] MIDI OUT Jack Descriptor - External (9 bytes)
 *  [0x49] Standard Bulk OUT Endpoint Descriptor (9 bytes)
 *  [0x52] Class-Specific MS Bulk OUT Endpoint Descriptor (5 bytes)
 *  [0x57] Standard Bulk IN Endpoint Descriptor (9 bytes)
 *  [0x60] Class-Specific MS Bulk IN Endpoint Descriptor (5 bytes)
 */
__ALIGN_BEGIN static uint8_t USBD_MIDI_CfgFSDesc[USB_MIDI_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* ----------------------------------------------------------------------- */
  /* Configuration Descriptor */
  0x09,                           /* bLength */
  USB_DESC_TYPE_CONFIGURATION,    /* bDescriptorType: Configuration */
  LOBYTE(USB_MIDI_CONFIG_DESC_SIZ),
  HIBYTE(USB_MIDI_CONFIG_DESC_SIZ),
  0x02,                           /* bNumInterfaces: 2 (AC + MS) */
  0x01,                           /* bConfigurationValue */
  0x00,                           /* iConfiguration */
#if (USBD_SELF_POWERED == 1U)
  0xC0,                           /* bmAttributes: Self-powered */
#else
  0x80,                           /* bmAttributes: Bus-powered */
#endif
  USBD_MAX_POWER,                 /* MaxPower */

  /* ----------------------------------------------------------------------- */
  /* Standard Audio Control (AC) Interface Descriptor */
  /* Interface 0 - AudioControl (required, zero-bandwidth) */
  0x09,                           /* bLength */
  USB_DESC_TYPE_INTERFACE,        /* bDescriptorType: Interface */
  _MIDI_AC_ITF_DEFAULT,           /* bInterfaceNumber (patched by Update) */
  0x00,                           /* bAlternateSetting */
  0x00,                           /* bNumEndpoints: no endpoints */
  USB_AUDIO_CLASS,                /* bInterfaceClass: Audio */
  USB_AUDIO_SUBCLASS_AUDIOCONTROL,/* bInterfaceSubClass: AudioControl */
  USB_AUDIO_PROTOCOL_UNDEFINED,   /* bInterfaceProtocol */
  _MIDI_STR_IDX_DEFAULT,          /* iInterface (patched by Update) */

  /* ----------------------------------------------------------------------- */
  /* Class-Specific AC Interface Header Descriptor */
  0x09,                           /* bLength */
  CS_INTERFACE,                   /* bDescriptorType: CS_INTERFACE */
  HEADER_SUBTYPE,                 /* bDescriptorSubtype: HEADER */
  0x00, 0x01,                     /* bcdADC: 1.0 */
  0x09, 0x00,                     /* wTotalLength: 9 (just this header) */
  0x01,                           /* bInCollection: 1 streaming interface */
  _MIDI_MS_ITF_DEFAULT,           /* baInterfaceNr(1) (patched by Update) */

  /* ----------------------------------------------------------------------- */
  /* Standard MIDI Streaming (MS) Interface Descriptor */
  /* Interface 1 - MIDIStreaming */
  0x09,                           /* bLength */
  USB_DESC_TYPE_INTERFACE,        /* bDescriptorType: Interface */
  _MIDI_MS_ITF_DEFAULT,           /* bInterfaceNumber (patched by Update) */
  0x00,                           /* bAlternateSetting */
  0x02,                           /* bNumEndpoints: 2 (bulk IN + bulk OUT) */
  USB_AUDIO_CLASS,                /* bInterfaceClass: Audio */
  USB_AUDIO_SUBCLASS_MIDISTREAMING,/* bInterfaceSubClass: MIDIStreaming */
  USB_AUDIO_PROTOCOL_UNDEFINED,   /* bInterfaceProtocol */
  _MIDI_STR_IDX_DEFAULT,          /* iInterface (patched by Update) */

  /* ----------------------------------------------------------------------- */
  /* Class-Specific MS Interface Header Descriptor */
  0x07,                           /* bLength */
  CS_INTERFACE,                   /* bDescriptorType: CS_INTERFACE */
  MS_HEADER_SUBTYPE,              /* bDescriptorSubtype: MS_HEADER */
  0x00, 0x01,                     /* BcdMSC: 1.0 */
  /* wTotalLength: sum of all class-specific MS descriptors:
   * 7 + 6 + 6 + 9 + 9 + 5 + 5 = 47 bytes */
  0x2F, 0x00,

  /* ----------------------------------------------------------------------- */
  /* MIDI IN Jack Descriptor - Embedded (ID=1) */
  /* Represents the MIDI data coming IN to the device FROM the host */
  0x06,                           /* bLength */
  CS_INTERFACE,                   /* bDescriptorType: CS_INTERFACE */
  MIDI_IN_JACK_SUBTYPE,           /* bDescriptorSubtype: MIDI_IN_JACK */
  JACK_TYPE_EMBEDDED,             /* bJackType: Embedded */
  MIDI_IN_JACK_EMB_ID,            /* bJackID: 1 */
  0x00,                           /* iJack */

  /* ----------------------------------------------------------------------- */
  /* MIDI IN Jack Descriptor - External (ID=2) */
  /* Represents a physical MIDI IN connector on the device */
  0x06,                           /* bLength */
  CS_INTERFACE,                   /* bDescriptorType: CS_INTERFACE */
  MIDI_IN_JACK_SUBTYPE,           /* bDescriptorSubtype: MIDI_IN_JACK */
  JACK_TYPE_EXTERNAL,             /* bJackType: External */
  MIDI_IN_JACK_EXT_ID,            /* bJackID: 2 */
  0x00,                           /* iJack */

  /* ----------------------------------------------------------------------- */
  /* MIDI OUT Jack Descriptor - Embedded (ID=3) */
  /* Represents MIDI data going OUT from the device TO the host */
  0x09,                           /* bLength */
  CS_INTERFACE,                   /* bDescriptorType: CS_INTERFACE */
  MIDI_OUT_JACK_SUBTYPE,          /* bDescriptorSubtype: MIDI_OUT_JACK */
  JACK_TYPE_EMBEDDED,             /* bJackType: Embedded */
  MIDI_OUT_JACK_EMB_ID,           /* bJackID: 3 */
  0x01,                           /* bNrInputPins: 1 */
  MIDI_IN_JACK_EXT_ID,            /* BaSourceID(1): connected to External IN Jack */
  0x01,                           /* BaSourcePin(1) */
  0x00,                           /* iJack */

  /* ----------------------------------------------------------------------- */
  /* MIDI OUT Jack Descriptor - External (ID=4) */
  /* Represents a physical MIDI OUT connector on the device */
  0x09,                           /* bLength */
  CS_INTERFACE,                   /* bDescriptorType: CS_INTERFACE */
  MIDI_OUT_JACK_SUBTYPE,          /* bDescriptorSubtype: MIDI_OUT_JACK */
  JACK_TYPE_EXTERNAL,             /* bJackType: External */
  MIDI_OUT_JACK_EXT_ID,           /* bJackID: 4 */
  0x01,                           /* bNrInputPins: 1 */
  MIDI_IN_JACK_EMB_ID,            /* BaSourceID(1): connected to Embedded IN Jack */
  0x01,                           /* BaSourcePin(1) */
  0x00,                           /* iJack */

  /* ----------------------------------------------------------------------- */
  /* Standard Bulk OUT Endpoint Descriptor */
  /* Host sends MIDI data to device via this endpoint */
  0x09,                           /* bLength */
  USB_DESC_TYPE_ENDPOINT,         /* bDescriptorType: Endpoint */
  _MIDI_OUT_EP_DEFAULT,           /* bEndpointAddress (patched by Update) */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(MIDI_DATA_FS_MAX_PACKET_SIZE),
  HIBYTE(MIDI_DATA_FS_MAX_PACKET_SIZE),
  0x00,                           /* bInterval: ignored for Bulk */
  0x00,                           /* bRefresh */
  0x00,                           /* bSynchAddress */

  /* ----------------------------------------------------------------------- */
  /* Class-Specific MS Bulk OUT Endpoint Descriptor */
  0x05,                           /* bLength */
  CS_ENDPOINT,                    /* bDescriptorType: CS_ENDPOINT */
  MS_GENERAL_SUBTYPE,             /* bDescriptorSubtype: MS_GENERAL */
  0x01,                           /* bNumEmbMIDIJack: 1 */
  MIDI_IN_JACK_EMB_ID,            /* BaAssocJackID(1): Embedded IN Jack */

  /* ----------------------------------------------------------------------- */
  /* Standard Bulk IN Endpoint Descriptor */
  /* Device sends MIDI data to host via this endpoint */
  0x09,                           /* bLength */
  USB_DESC_TYPE_ENDPOINT,         /* bDescriptorType: Endpoint */
  _MIDI_IN_EP_DEFAULT,            /* bEndpointAddress (patched by Update) */
  0x02,                           /* bmAttributes: Bulk */
  LOBYTE(MIDI_DATA_FS_MAX_PACKET_SIZE),
  HIBYTE(MIDI_DATA_FS_MAX_PACKET_SIZE),
  0x00,                           /* bInterval: ignored for Bulk */
  0x00,                           /* bRefresh */
  0x00,                           /* bSynchAddress */

  /* ----------------------------------------------------------------------- */
  /* Class-Specific MS Bulk IN Endpoint Descriptor */
  0x05,                           /* bLength */
  CS_ENDPOINT,                    /* bDescriptorType: CS_ENDPOINT */
  MS_GENERAL_SUBTYPE,             /* bDescriptorSubtype: MS_GENERAL */
  0x01,                           /* bNumEmbMIDIJack: 1 */
  MIDI_OUT_JACK_EMB_ID,           /* BaAssocJackID(1): Embedded OUT Jack */
};

/* HS descriptor is identical except endpoint max packet size */
__ALIGN_BEGIN static uint8_t USBD_MIDI_CfgHSDesc[USB_MIDI_CONFIG_DESC_SIZ] __ALIGN_END;

/**
  * @}
  */

/* USBD_ClassTypeDef registration structure */
USBD_ClassTypeDef USBD_MIDI =
{
  USBD_MIDI_Init,
  USBD_MIDI_DeInit,
  USBD_MIDI_Setup,
  NULL,                           /* EP0_TxSent */
  NULL,                           /* EP0_RxReady */
  USBD_MIDI_DataIn,
  USBD_MIDI_DataOut,
  NULL,                           /* SOF */
  NULL,                           /* IsoINIncomplete */
  NULL,                           /* IsoOutIncomplete */
  USBD_MIDI_GetHSCfgDesc,
  USBD_MIDI_GetFSCfgDesc,
  USBD_MIDI_GetOtherSpeedCfgDesc,
  USBD_MIDI_GetDeviceQualifierDescriptor,
};

/** @defgroup USBD_MIDI_Private_Functions
  * @{
  */

/**
  * @brief  USBD_MIDI_Init
  *         Initialize the MIDI interface
  * @param  pdev: device instance
  * @param  cfgidx: configuration index
  * @retval status
  */
static uint8_t USBD_MIDI_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  USBD_MIDI_HandleTypeDef *hmidi = &MIDI_Class_Data;

  /* Open Bulk OUT EP */
  (void)USBD_LL_OpenEP(pdev,
                        MIDI_OUT_EP,
                        USBD_EP_TYPE_BULK,
                        MIDI_DATA_FS_MAX_PACKET_SIZE);
  pdev->ep_out[MIDI_OUT_EP & 0x0FU].is_used = 1U;

  /* Open Bulk IN EP */
  (void)USBD_LL_OpenEP(pdev,
                        MIDI_IN_EP,
                        USBD_EP_TYPE_BULK,
                        MIDI_DATA_FS_MAX_PACKET_SIZE);
  pdev->ep_in[MIDI_IN_EP & 0x0FU].is_used = 1U;

  /* Initialize handle */
  hmidi->TxState = 0U;
  hmidi->RxState = 0U;

  /* Set default RX buffer */
  (void)USBD_LL_PrepareReceive(pdev, MIDI_OUT_EP,
                                hmidi->RxBuffer,
                                MIDI_DATA_FS_MAX_PACKET_SIZE);

  /* Call application Init */
  if (MIDI_fops != NULL && MIDI_fops->Init != NULL)
  {
    (void)MIDI_fops->Init();
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_MIDI_DeInit
  *         De-Initialize the MIDI interface
  * @param  pdev: device instance
  * @param  cfgidx: configuration index
  * @retval status
  */
static uint8_t USBD_MIDI_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  /* Close endpoints */
  (void)USBD_LL_CloseEP(pdev, MIDI_OUT_EP);
  pdev->ep_out[MIDI_OUT_EP & 0x0FU].is_used = 0U;

  (void)USBD_LL_CloseEP(pdev, MIDI_IN_EP);
  pdev->ep_in[MIDI_IN_EP & 0x0FU].is_used = 0U;

  /* Call application DeInit */
  if (MIDI_fops != NULL && MIDI_fops->DeInit != NULL)
  {
    (void)MIDI_fops->DeInit();
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_MIDI_Setup
  *         Handle the MIDI class-specific requests.
  *         USB MIDI 1.0 does not define class-specific requests beyond
  *         standard Audio Class control, so most requests are acknowledged.
  * @param  pdev: device instance
  * @param  req: USB request
  * @retval status
  */
static uint8_t USBD_MIDI_Setup(USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
  UNUSED(pdev);

  /* MIDI 1.0 has no mandatory class-specific control requests.
   * Stall unsupported requests gracefully. */
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      /* No class-specific requests defined for MIDI 1.0 */
      USBD_CtlError(pdev, req);
      break;

    case USB_REQ_TYPE_STANDARD:
      /* Standard requests handled by core */
      break;

    default:
      USBD_CtlError(pdev, req);
      break;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_MIDI_DataIn
  *         Data sent on IN endpoint (device -> host transfer complete)
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_MIDI_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(pdev);
  USBD_MIDI_HandleTypeDef *hmidi = &MIDI_Class_Data;

  if ((MIDI_IN_EP & 0x7FU) == epnum)
  {
    hmidi->TxState = 0U;

    if (MIDI_fops != NULL && MIDI_fops->TransmitCplt != NULL)
    {
      (void)MIDI_fops->TransmitCplt(hmidi->TxBuffer,
                                     &hmidi->TxLength,
                                     epnum);
    }
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_MIDI_DataOut
  *         Data received on OUT endpoint (host -> device)
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_MIDI_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_MIDI_HandleTypeDef *hmidi = &MIDI_Class_Data;

  if ((MIDI_OUT_EP & 0x0FU) == epnum)
  {
    hmidi->RxLength = USBD_LL_GetRxDataSize(pdev, epnum);

    if (MIDI_fops != NULL && MIDI_fops->Receive != NULL)
    {
      (void)MIDI_fops->Receive(hmidi->RxBuffer, hmidi->RxLength);
    }
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_MIDI_GetFSCfgDesc
  * @param  length: pointer to descriptor length
  * @retval pointer to configuration descriptor
  */
static uint8_t *USBD_MIDI_GetFSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_MIDI_CfgFSDesc);
  return USBD_MIDI_CfgFSDesc;
}

/**
  * @brief  USBD_MIDI_GetHSCfgDesc
  * @param  length: pointer to descriptor length
  * @retval pointer to configuration descriptor
  */
static uint8_t *USBD_MIDI_GetHSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_MIDI_CfgHSDesc);
  return USBD_MIDI_CfgHSDesc;
}

/**
  * @brief  USBD_MIDI_GetOtherSpeedCfgDesc
  * @param  length: pointer to descriptor length
  * @retval pointer to configuration descriptor (reuse FS for other-speed)
  */
static uint8_t *USBD_MIDI_GetOtherSpeedCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_MIDI_CfgFSDesc);
  return USBD_MIDI_CfgFSDesc;
}

/**
  * @brief  USBD_MIDI_GetDeviceQualifierDescriptor
  * @param  length: pointer to descriptor length
  * @retval pointer to device qualifier descriptor
  */
static uint8_t *USBD_MIDI_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_MIDI_DeviceQualifierDesc);
  return USBD_MIDI_DeviceQualifierDesc;
}

/** @defgroup USBD_MIDI_Exported_Functions
  * @{
  */

/**
  * @brief  USBD_MIDI_RegisterInterface
  *         Register MIDI application callbacks
  * @param  pdev: device instance
  * @param  fops: MIDI interface callbacks
  * @retval status
  */
uint8_t USBD_MIDI_RegisterInterface(USBD_HandleTypeDef *pdev,
                                    USBD_MIDI_ItfTypeDef *fops)
{
  UNUSED(pdev);

  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  MIDI_fops = fops;
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_MIDI_SetTxBuffer
  *         Set the transmit (IN) buffer
  * @param  pdev: device instance
  * @param  pbuff: pointer to data
  * @param  length: length in bytes
  * @retval status
  */
uint8_t USBD_MIDI_SetTxBuffer(USBD_HandleTypeDef *pdev,
                               uint8_t *pbuff,
                               uint32_t length)
{
  UNUSED(pdev);
  USBD_MIDI_HandleTypeDef *hmidi = &MIDI_Class_Data;

  hmidi->TxBuffer = pbuff;
  hmidi->TxLength = length;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_MIDI_SetRxBuffer
  *         Set the receive (OUT) buffer
  * @param  pdev: device instance
  * @param  pbuff: pointer to buffer
  * @retval status
  */
uint8_t USBD_MIDI_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff)
{
  UNUSED(pdev);
  USBD_MIDI_HandleTypeDef *hmidi = &MIDI_Class_Data;
  hmidi->RxBuffer = pbuff;
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_MIDI_ReceivePacket
  *         Re-arm the OUT endpoint to receive the next USB-MIDI packet
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_MIDI_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  USBD_MIDI_HandleTypeDef *hmidi = &MIDI_Class_Data;

  return (uint8_t)USBD_LL_PrepareReceive(pdev,
                                          MIDI_OUT_EP,
                                          hmidi->RxBuffer,
                                          MIDI_DATA_FS_MAX_PACKET_SIZE);
}

/**
  * @brief  USBD_MIDI_TransmitPacket
  *         Transmit a USB-MIDI packet over the IN endpoint
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_MIDI_TransmitPacket(USBD_HandleTypeDef *pdev)
{
  USBD_MIDI_HandleTypeDef *hmidi = &MIDI_Class_Data;
  uint8_t ret = USBD_BUSY;

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (hmidi->TxState == 0U)
    {
      hmidi->TxState = 1U;
      pdev->ep_in[MIDI_IN_EP & 0x0FU].total_length = hmidi->TxLength;

      (void)USBD_LL_Transmit(pdev,
                              MIDI_IN_EP,
                              hmidi->TxBuffer,
                              hmidi->TxLength);
      ret = USBD_OK;
    }
  }

  return ret;
}

/**
  * @brief  USBD_Update_MIDI_DESC
  *         Patch the MIDI configuration descriptors with assigned
  *         interface numbers, endpoint addresses, and string index.
  *         Called by USBD_COMPOSITE_Mount_Class().
  *
  * @param  desc:    pointer to FS or HS config descriptor
  * @param  ac_itf:  Audio Control interface number
  * @param  ms_itf:  MIDI Streaming interface number
  * @param  in_ep:   Bulk IN endpoint address  (e.g. 0x81)
  * @param  out_ep:  Bulk OUT endpoint address (e.g. 0x01)
  * @param  str_idx: String descriptor index for iInterface fields
  */
void USBD_Update_MIDI_DESC(uint8_t *desc,
                           uint8_t ac_itf,
                           uint8_t ms_itf,
                           uint8_t in_ep,
                           uint8_t out_ep,
                           uint8_t str_idx)
{
  /* Save assigned values to global variables used by composite routing */
  MIDI_AC_ITF_NBR   = ac_itf;
  MIDI_MS_ITF_NBR   = ms_itf;
  MIDI_IN_EP        = in_ep;
  MIDI_OUT_EP       = out_ep;
  MIDI_STR_DESC_IDX = str_idx;

  /* Skip the 9-byte configuration descriptor header */
  uint8_t *p = desc + 9U;

  /* --- Standard AC Interface Descriptor (offset 9, length 9) --- */
  /* bInterfaceNumber @ p+2 */
  p[2] = ac_itf;
  /* iInterface @ p+8 */
  p[8] = str_idx;

  /* --- CS AC Interface Header (offset 18, length 9) --- */
  p += 9U;
  /* baInterfaceNr(1) @ p+8: the MIDI Streaming interface number */
  p[8] = ms_itf;

  /* --- Standard MS Interface Descriptor (offset 27, length 9) --- */
  p += 9U;
  /* bInterfaceNumber @ p+2 */
  p[2] = ms_itf;
  /* iInterface @ p+8 */
  p[8] = str_idx;

  /* --- CS MS Interface Header (offset 36, length 7) --- */
  p += 9U;
  /* No interface numbers to patch here */

  /* --- MIDI IN Jack Embedded (offset 43, length 6) --- */
  p += 7U;
  /* No interface/endpoint fields to patch */

  /* --- MIDI IN Jack External (offset 49, length 6) --- */
  p += 6U;

  /* --- MIDI OUT Jack Embedded (offset 55, length 9) --- */
  p += 6U;

  /* --- MIDI OUT Jack External (offset 64, length 9) --- */
  p += 9U;

  /* --- Standard Bulk OUT Endpoint (offset 73, length 9) --- */
  p += 9U;
  /* bEndpointAddress @ p+2 */
  p[2] = out_ep;

  /* --- CS MS Bulk OUT Endpoint (offset 82, length 5) --- */
  p += 9U;

  /* --- Standard Bulk IN Endpoint (offset 87, length 9) --- */
  p += 5U;
  /* bEndpointAddress @ p+2 */
  p[2] = in_ep;

  /* Also update HS descriptor (copy FS, then fix packet sizes) */
  uint8_t *fs_start = USBD_MIDI_CfgFSDesc;
  uint8_t *hs_start = USBD_MIDI_CfgHSDesc;
  for (uint16_t i = 0U; i < USB_MIDI_CONFIG_DESC_SIZ; i++)
  {
    hs_start[i] = fs_start[i];
  }
  /* Patch HS max packet sizes for both endpoints */
  /* Bulk OUT EP wMaxPacketSize at offset 73+4 = 77 from start */
  hs_start[77] = LOBYTE(MIDI_DATA_HS_MAX_PACKET_SIZE);
  hs_start[78] = HIBYTE(MIDI_DATA_HS_MAX_PACKET_SIZE);
  /* Bulk IN EP wMaxPacketSize at offset 87+4 = 91 from start */
  hs_start[91] = LOBYTE(MIDI_DATA_HS_MAX_PACKET_SIZE);
  hs_start[92] = HIBYTE(MIDI_DATA_HS_MAX_PACKET_SIZE);
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
