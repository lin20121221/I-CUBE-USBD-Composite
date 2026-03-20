/**
  ******************************************************************************
  * @file    usbd_cdc_ncm.c
  * @author  AL94
  * @brief   USB CDC NCM (Network Control Model) class driver.
  *
  *          CDC NCM uses two interfaces:
  *            Interface 0  – CDC Communication class, with:
  *              • CDC Header Functional Descriptor
  *              • CDC Union Functional Descriptor
  *              • NCM Functional Descriptor  (NCM spec §5.2.1)
  *              • CDC Ethernet Functional Descriptor (MAC address string)
  *              • Interrupt IN endpoint (network connection / speed notifications)
  *            Interface 1  – CDC Data class, alternate 0 (no EPs) and
  *                           alternate 1 (Bulk IN + Bulk OUT).
  *
  *          Host→Device data (OUT) is received as NTB-16 blocks.  The driver
  *          assembles each USB transfer into the RxBuffer, then calls the
  *          application Receive() callback with each individual datagram it
  *          finds in the NDP16 pointer table.
  *
  *          Device→Host data (IN) is built by the application layer calling
  *          USBD_CDC_NCM_TransmitPacket() after filling the TxNtbBuffer with
  *          a well-formed NTB-16 (or by using the helper in the _if layer).
  *
  *          Reference: USB Communications Device Class, Subclass Specification
  *          for Network Control Model Devices, Revision 1.0 (November 24, 2010).
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
#include "usbd_cdc_ncm.h"
#include "usbd_ctlreq.h"
#include "usbd_cdc_ncm_if.h"

/* Default endpoint / interface assignments (overridden by USBD_Update_CDC_NCM_DESC) */
#define _CDC_NCM_IN_EP        0x82U
#define _CDC_NCM_OUT_EP       0x02U
#define _CDC_NCM_CMD_EP       0x83U
#define _CDC_NCM_CMD_ITF_NBR  0x00U
#define _CDC_NCM_COM_ITF_NBR  0x01U
#define _CDC_NCM_STR_DESC_IDX 0x00U

uint8_t CDC_NCM_IN_EP        = _CDC_NCM_IN_EP;
uint8_t CDC_NCM_OUT_EP       = _CDC_NCM_OUT_EP;
uint8_t CDC_NCM_CMD_EP       = _CDC_NCM_CMD_EP;
uint8_t CDC_NCM_CMD_ITF_NBR  = _CDC_NCM_CMD_ITF_NBR;
uint8_t CDC_NCM_COM_ITF_NBR  = _CDC_NCM_COM_ITF_NBR;
uint8_t CDC_NCM_STR_DESC_IDX = _CDC_NCM_STR_DESC_IDX;

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CDC_NCM
  * @brief usbd CDC NCM module
  * @{
  */

/* Private function prototypes -----------------------------------------------*/
static uint8_t USBD_CDC_NCM_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_NCM_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_NCM_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_CDC_NCM_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_CDC_NCM_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_NCM_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t *USBD_CDC_NCM_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_NCM_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_NCM_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_NCM_GetDeviceQualifierDescriptor(uint16_t *length);

/* Static handle */
static USBD_CDC_NCM_HandleTypeDef CDC_NCM_Instance;

/* NTB parameters reported to host */
static const USBD_CDC_NCM_NtbParametersTypeDef CDC_NCM_NtbParameters =
{
  .wLength               = sizeof(USBD_CDC_NCM_NtbParametersTypeDef),
  .bmNtbFormatsSupported = 0x0001U,  /* NTB-16 only */
  .dwNtbInMaxSize        = CDC_NCM_NTB_IN_MAX_SIZE,
  .wNdpInDivisor         = 4U,
  .wNdpInPayloadRemainder= 0U,
  .wNdpInAlignment       = 4U,
  .wReserved             = 0U,
  .dwNtbOutMaxSize       = CDC_NCM_NTB_OUT_MAX_SIZE,
  .wNdpOutDivisor        = 4U,
  .wNdpOutPayloadRemainder = 0U,
  .wNdpOutAlignment      = 4U,
  .wNtbOutMaxDatagrams   = CDC_NCM_MAX_DATAGRAMS_PER_NTB,
};

USBD_ClassTypeDef USBD_CDC_NCM =
{
  USBD_CDC_NCM_Init,
  USBD_CDC_NCM_DeInit,
  USBD_CDC_NCM_Setup,
  NULL,                                   /* EP0_TxSent       */
  USBD_CDC_NCM_EP0_RxReady,
  USBD_CDC_NCM_DataIn,
  USBD_CDC_NCM_DataOut,
  NULL,                                   /* SOF              */
  NULL,                                   /* IsoINIncomplete  */
  NULL,                                   /* IsoOUTIncomplete */
  USBD_CDC_NCM_GetHSCfgDesc,
  USBD_CDC_NCM_GetFSCfgDesc,
  USBD_CDC_NCM_GetOtherSpeedCfgDesc,
  USBD_CDC_NCM_GetDeviceQualifierDescriptor,
};

/* --------------------------------------------------------------------------
 * Descriptor layout (CDC NCM, 94 bytes total):
 *
 *  [  0..  8]  Configuration Descriptor          9 bytes
 *  [  9.. 16]  IAD                               8 bytes
 *  [ 17.. 25]  Control Interface Descriptor       9 bytes
 *  [ 26.. 30]  CDC Header Functional Descriptor   5 bytes
 *  [ 31.. 35]  CDC Union Functional Descriptor    5 bytes  ← 5, not 4
 *  [ 36.. 41]  NCM Functional Descriptor          6 bytes  ← per NCM spec §5.2.1
 *  [ 42.. 54]  CDC Ethernet Functional Descriptor 13 bytes
 *  [ 55.. 61]  Interrupt IN EP (notification)     7 bytes
 *  [ 62.. 70]  Data Interface alt-0 (no EPs)      9 bytes
 *  [ 71.. 79]  Data Interface alt-1 (bulk EPs)    9 bytes
 *  [ 80.. 86]  Bulk OUT EP                        7 bytes
 *  [ 87.. 93]  Bulk IN  EP                        7 bytes
 *
 *  Total = 9+8+9+5+5+6+13+7+9+9+7+7 = 94
 *
 *  Fields patched by USBD_Update_CDC_NCM_DESC (absolute byte index):
 *    desc[11]  IAD bFirstInterface    → cmd_itf
 *    desc[16]  IAD iFunction          → str_idx
 *    desc[19]  CtrlItf bItfNumber     → cmd_itf
 *    desc[25]  CtrlItf iInterface     → str_idx
 *    desc[34]  Union bMasterInterface → cmd_itf
 *    desc[35]  Union bSlaveInterface0 → com_itf
 *    desc[57]  Interrupt EP address   → cmd_ep
 *    desc[64]  DataItf-0 bItfNumber   → com_itf
 *    desc[73]  DataItf-1 bItfNumber   → com_itf
 *    desc[82]  Bulk OUT EP address    → out_ep
 *    desc[89]  Bulk IN  EP address    → in_ep
 * --------------------------------------------------------------------------
 */

/* HS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_NCM_CfgHSDesc[CDC_NCM_CONFIG_DESC_SIZE] __ALIGN_END =
{
  /* [0..8] Configuration Descriptor (9) */
  0x09,                                /* bLength */
  USB_DESC_TYPE_CONFIGURATION,         /* bDescriptorType */
  LOBYTE(CDC_NCM_CONFIG_DESC_SIZE),    /* wTotalLength lo */
  HIBYTE(CDC_NCM_CONFIG_DESC_SIZE),    /* wTotalLength hi */
  0x02,                                /* bNumInterfaces */
  0x01,                                /* bConfigurationValue */
  0x00,                                /* iConfiguration */
#if (USBD_SELF_POWERED == 1U)
  0xC0,
#else
  0x80,
#endif
  USBD_MAX_POWER,

  /* [9..16] IAD (8) */
  0x08,                                /* bLength */
  0x0B,                                /* bDescriptorType: IAD */
  _CDC_NCM_CMD_ITF_NBR,               /* [11] bFirstInterface ← patched */
  0x02,                                /* bInterfaceCount */
  0x02,                                /* bFunctionClass: CDC */
  0x0D,                                /* bFunctionSubClass: NCM */
  0x00,                                /* bFunctionProtocol */
  _CDC_NCM_STR_DESC_IDX,              /* [16] iFunction ← patched */

  /* [17..25] Control Interface Descriptor (9) */
  0x09,                                /* bLength */
  USB_DESC_TYPE_INTERFACE,             /* bDescriptorType */
  _CDC_NCM_CMD_ITF_NBR,               /* [19] bInterfaceNumber ← patched */
  0x00,                                /* bAlternateSetting */
  0x01,                                /* bNumEndpoints: 1 interrupt IN */
  0x02,                                /* bInterfaceClass: CDC */
  0x0D,                                /* bInterfaceSubClass: NCM */
  0x00,                                /* bInterfaceProtocol */
  _CDC_NCM_STR_DESC_IDX,              /* [25] iInterface ← patched */

  /* [26..30] CDC Header Functional Descriptor (5) */
  0x05,                                /* bFunctionLength */
  0x24,                                /* bDescriptorType: CS_INTERFACE */
  0x00,                                /* bDescriptorSubtype: Header */
  0x20, 0x01,                          /* bcdCDC: 1.20 */

  /* [31..35] CDC Union Functional Descriptor (5) */
  0x05,                                /* bFunctionLength */
  0x24,                                /* bDescriptorType: CS_INTERFACE */
  0x06,                                /* bDescriptorSubtype: Union */
  _CDC_NCM_CMD_ITF_NBR,               /* [34] bMasterInterface ← patched */
  _CDC_NCM_COM_ITF_NBR,               /* [35] bSlaveInterface0 ← patched */

  /* [36..41] NCM Functional Descriptor (6) — NCM spec §5.2.1 */
  0x06,                                /* bFunctionLength */
  0x24,                                /* bDescriptorType: CS_INTERFACE */
  0x1A,                                /* bDescriptorSubtype: NCM */
  0x00, 0x01,                          /* bcdNcmVersion: 1.00 */
  0x00,                                /* bmNetworkCapabilities */

  /* [42..54] CDC Ethernet Functional Descriptor (13) */
  0x0D,                                /* bFunctionLength */
  0x24,                                /* bDescriptorType: CS_INTERFACE */
  0x0F,                                /* bDescriptorSubtype: Ethernet Networking */
  CDC_NCM_MAC_STRING_INDEX,            /* iMACAddress (Unicode string, 12 hex digits) */
  0x00, 0x00, 0x00, 0x00,             /* bmEthernetStatistics */
  LOBYTE(CDC_NCM_ETH_MAX_SEGSZE),      /* wMaxSegmentSize lo */
  HIBYTE(CDC_NCM_ETH_MAX_SEGSZE),      /* wMaxSegmentSize hi */
  0x00, 0x00,                          /* wNumberMCFilters */
  0x00,                                /* bNumberPowerFilters */

  /* [55..61] Interrupt IN Endpoint (7) */
  0x07,                                /* bLength */
  USB_DESC_TYPE_ENDPOINT,              /* bDescriptorType */
  _CDC_NCM_CMD_EP,                    /* [57] bEndpointAddress ← patched */
  0x03,                                /* bmAttributes: Interrupt */
  LOBYTE(CDC_NCM_CMD_PACKET_SIZE),
  HIBYTE(CDC_NCM_CMD_PACKET_SIZE),
  CDC_NCM_HS_BINTERVAL,

  /* [62..70] Data Interface alt-0: no endpoints (9) */
  0x09,                                /* bLength */
  USB_DESC_TYPE_INTERFACE,             /* bDescriptorType */
  _CDC_NCM_COM_ITF_NBR,               /* [64] bInterfaceNumber ← patched */
  0x00,                                /* bAlternateSetting: 0 */
  0x00,                                /* bNumEndpoints: 0 */
  0x0A,                                /* bInterfaceClass: CDC Data */
  0x00,                                /* bInterfaceSubClass */
  0x01,                                /* bInterfaceProtocol: NTB */
  0x00,                                /* iInterface */

  /* [71..79] Data Interface alt-1: with bulk endpoints (9) */
  0x09,                                /* bLength */
  USB_DESC_TYPE_INTERFACE,             /* bDescriptorType */
  _CDC_NCM_COM_ITF_NBR,               /* [73] bInterfaceNumber ← patched */
  0x01,                                /* bAlternateSetting: 1 */
  0x02,                                /* bNumEndpoints: 2 */
  0x0A,                                /* bInterfaceClass: CDC Data */
  0x00,                                /* bInterfaceSubClass */
  0x01,                                /* bInterfaceProtocol: NTB */
  0x00,                                /* iInterface */

  /* [80..86] Bulk OUT Endpoint (7) */
  0x07,                                /* bLength */
  USB_DESC_TYPE_ENDPOINT,              /* bDescriptorType */
  _CDC_NCM_OUT_EP,                    /* [82] bEndpointAddress ← patched */
  0x02,                                /* bmAttributes: Bulk */
  LOBYTE(CDC_NCM_DATA_HS_MAX_PACKET_SIZE),
  HIBYTE(CDC_NCM_DATA_HS_MAX_PACKET_SIZE),
  0x00,

  /* [87..93] Bulk IN Endpoint (7) */
  0x07,                                /* bLength */
  USB_DESC_TYPE_ENDPOINT,              /* bDescriptorType */
  _CDC_NCM_IN_EP,                     /* [89] bEndpointAddress ← patched */
  0x02,                                /* bmAttributes: Bulk */
  LOBYTE(CDC_NCM_DATA_HS_MAX_PACKET_SIZE),
  HIBYTE(CDC_NCM_DATA_HS_MAX_PACKET_SIZE),
  0x00,
};

/* FS Configuration Descriptor – same layout, FS bulk packet sizes */
__ALIGN_BEGIN static uint8_t USBD_CDC_NCM_CfgFSDesc[CDC_NCM_CONFIG_DESC_SIZE] __ALIGN_END =
{
  /* Config */
  0x09, USB_DESC_TYPE_CONFIGURATION,
  LOBYTE(CDC_NCM_CONFIG_DESC_SIZE), HIBYTE(CDC_NCM_CONFIG_DESC_SIZE),
  0x02, 0x01, 0x00,
#if (USBD_SELF_POWERED == 1U)
  0xC0,
#else
  0x80,
#endif
  USBD_MAX_POWER,
  /* IAD */
  0x08, 0x0B, _CDC_NCM_CMD_ITF_NBR, 0x02, 0x02, 0x0D, 0x00, _CDC_NCM_STR_DESC_IDX,
  /* Control Interface */
  0x09, USB_DESC_TYPE_INTERFACE, _CDC_NCM_CMD_ITF_NBR, 0x00, 0x01,
  0x02, 0x0D, 0x00, _CDC_NCM_STR_DESC_IDX,
  /* CDC Header FD */
  0x05, 0x24, 0x00, 0x20, 0x01,
  /* CDC Union FD (5 bytes) */
  0x05, 0x24, 0x06, _CDC_NCM_CMD_ITF_NBR, _CDC_NCM_COM_ITF_NBR,
  /* NCM Functional FD (6 bytes) */
  0x06, 0x24, 0x1A, 0x00, 0x01, 0x00,
  /* CDC Ethernet FD (13 bytes) */
  0x0D, 0x24, 0x0F, CDC_NCM_MAC_STRING_INDEX,
  0x00, 0x00, 0x00, 0x00,
  LOBYTE(CDC_NCM_ETH_MAX_SEGSZE), HIBYTE(CDC_NCM_ETH_MAX_SEGSZE),
  0x00, 0x00, 0x00,
  /* Interrupt IN EP */
  0x07, USB_DESC_TYPE_ENDPOINT, _CDC_NCM_CMD_EP, 0x03,
  LOBYTE(CDC_NCM_CMD_PACKET_SIZE), HIBYTE(CDC_NCM_CMD_PACKET_SIZE),
  CDC_NCM_FS_BINTERVAL,
  /* Data Interface alt-0 */
  0x09, USB_DESC_TYPE_INTERFACE, _CDC_NCM_COM_ITF_NBR, 0x00, 0x00, 0x0A, 0x00, 0x01, 0x00,
  /* Data Interface alt-1 */
  0x09, USB_DESC_TYPE_INTERFACE, _CDC_NCM_COM_ITF_NBR, 0x01, 0x02, 0x0A, 0x00, 0x01, 0x00,
  /* Bulk OUT */
  0x07, USB_DESC_TYPE_ENDPOINT, _CDC_NCM_OUT_EP, 0x02,
  LOBYTE(CDC_NCM_DATA_FS_MAX_PACKET_SIZE), HIBYTE(CDC_NCM_DATA_FS_MAX_PACKET_SIZE), 0x00,
  /* Bulk IN */
  0x07, USB_DESC_TYPE_ENDPOINT, _CDC_NCM_IN_EP, 0x02,
  LOBYTE(CDC_NCM_DATA_FS_MAX_PACKET_SIZE), HIBYTE(CDC_NCM_DATA_FS_MAX_PACKET_SIZE), 0x00,
};

/* OtherSpeed = FS copy */
__ALIGN_BEGIN static uint8_t USBD_CDC_NCM_OtherSpeedCfgDesc[CDC_NCM_CONFIG_DESC_SIZE] __ALIGN_END;

/* Device qualifier */
__ALIGN_BEGIN static uint8_t USBD_CDC_NCM_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00, 0x02,   /* bcdUSB 2.00 */
  0x00,         /* bDeviceClass */
  0x00,         /* bDeviceSubClass */
  0x00,         /* bDeviceProtocol */
  0x40,         /* bMaxPacketSize0 */
  0x01,         /* bNumConfigurations */
  0x00,
};

/* EP0 receive buffer for class-specific control transfers */
__ALIGN_BEGIN static uint8_t CDC_NCM_EP0_Buf[8] __ALIGN_END;

/* ========================================================================== */
/* Private helper: parse an incoming NTB-16 and deliver each datagram         */
/* ========================================================================== */
static void USBD_CDC_NCM_ProcessNTB(USBD_HandleTypeDef *pdev,
                                     uint8_t *ntb, uint32_t ntb_len)
{
  USBD_CDC_NCM_ItfTypeDef *fops =
      (USBD_CDC_NCM_ItfTypeDef *)(pdev->pUserData_CDC_NCM);

  if (fops == NULL || ntb_len < sizeof(USBD_CDC_NCM_NTH16TypeDef))
  {
    return;
  }

  USBD_CDC_NCM_NTH16TypeDef *nth = (USBD_CDC_NCM_NTH16TypeDef *)ntb;

  /* Validate signature */
  if (nth->dwSignature != CDC_NCM_NTH16_SIGNATURE)
  {
    return;
  }

  uint16_t ndp_offset = nth->wNdpIndex;

  while (ndp_offset != 0U && (ndp_offset + 8U) <= ntb_len)
  {
    USBD_CDC_NCM_NDP16TypeDef *ndp =
        (USBD_CDC_NCM_NDP16TypeDef *)(ntb + ndp_offset);

    /* Walk datagram pointer array */
    uint32_t i = 0U;
    while (i < CDC_NCM_MAX_DATAGRAMS_PER_NTB)
    {
      uint16_t dg_idx = ndp->Datagram[i].wDatagramIndex;
      uint16_t dg_len = ndp->Datagram[i].wDatagramLength;

      if (dg_idx == 0U && dg_len == 0U)
      {
        break; /* terminator */
      }

      if ((uint32_t)dg_idx + dg_len <= ntb_len)
      {
        uint32_t len32 = (uint32_t)dg_len;
        (void)fops->Receive(ntb + dg_idx, &len32);
      }
      i++;
    }

    ndp_offset = ndp->wNextNdpIndex;
  }
}

/* ========================================================================== */
/* Class driver callbacks                                                      */
/* ========================================================================== */

static uint8_t USBD_CDC_NCM_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_CDC_NCM_HandleTypeDef *hncm = &CDC_NCM_Instance;
  (void)memset(hncm, 0, sizeof(USBD_CDC_NCM_HandleTypeDef));

  hncm->NtbInMaxSize = CDC_NCM_NTB_IN_MAX_SIZE;
  pdev->pClassData_CDC_NCM = (void *)hncm;

  uint16_t mps = (pdev->dev_speed == USBD_SPEED_HIGH)
                 ? CDC_NCM_DATA_HS_MAX_PACKET_SIZE
                 : CDC_NCM_DATA_FS_MAX_PACKET_SIZE;
  hncm->MaxPcktLen = mps;

  /* Open data endpoints (only after host selects alternate 1, but we open
   * them here; Windows/Linux drivers send SET_INTERFACE 1 automatically) */
  (void)USBD_LL_OpenEP(pdev, CDC_NCM_IN_EP,  USBD_EP_TYPE_BULK, mps);
  pdev->ep_in[CDC_NCM_IN_EP  & 0xFU].bInterval = 0U;
  (void)USBD_LL_OpenEP(pdev, CDC_NCM_OUT_EP, USBD_EP_TYPE_BULK, mps);
  pdev->ep_out[CDC_NCM_OUT_EP & 0xFU].bInterval = 0U;

  /* Open notification endpoint */
  (void)USBD_LL_OpenEP(pdev, CDC_NCM_CMD_EP, USBD_EP_TYPE_INTR, CDC_NCM_CMD_PACKET_SIZE);
  pdev->ep_in[CDC_NCM_CMD_EP & 0xFU].bInterval = CDC_NCM_FS_BINTERVAL;

  /* Call application init */
  USBD_CDC_NCM_ItfTypeDef *fops = (USBD_CDC_NCM_ItfTypeDef *)pdev->pUserData_CDC_NCM;
  if (fops != NULL)
  {
    (void)fops->Init();
  }

  /* Prime bulk OUT */
  (void)USBD_LL_PrepareReceive(pdev, CDC_NCM_OUT_EP, hncm->RxBuffer, mps);

  return (uint8_t)USBD_OK;
}

static uint8_t USBD_CDC_NCM_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  (void)USBD_LL_CloseEP(pdev, CDC_NCM_IN_EP);
  (void)USBD_LL_CloseEP(pdev, CDC_NCM_OUT_EP);
  (void)USBD_LL_CloseEP(pdev, CDC_NCM_CMD_EP);

  USBD_CDC_NCM_ItfTypeDef *fops = (USBD_CDC_NCM_ItfTypeDef *)pdev->pUserData_CDC_NCM;
  if (fops != NULL)
  {
    (void)fops->DeInit();
  }

  pdev->pClassData_CDC_NCM = NULL;
  return (uint8_t)USBD_OK;
}

static uint8_t USBD_CDC_NCM_Setup(USBD_HandleTypeDef *pdev,
                                    USBD_SetupReqTypedef *req)
{
  USBD_CDC_NCM_HandleTypeDef *hncm =
      (USBD_CDC_NCM_HandleTypeDef *)(pdev->pClassData_CDC_NCM);
  USBD_CDC_NCM_ItfTypeDef    *fops =
      (USBD_CDC_NCM_ItfTypeDef *)(pdev->pUserData_CDC_NCM);

  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    /* ---- Class-specific requests ---- */
    case USB_REQ_TYPE_CLASS:
      if (req->wLength != 0U)
      {
        if ((req->bmRequest & 0x80U) != 0U)
        {
          /* Device-to-host */
          if (fops != NULL)
          {
            (void)fops->Control(req->bRequest, CDC_NCM_EP0_Buf, (uint16_t)req->wLength);
          }

          if (req->bRequest == CDC_NCM_GET_NTB_PARAMETERS)
          {
            uint16_t len = (uint16_t)MIN(req->wLength,
                                         sizeof(USBD_CDC_NCM_NtbParametersTypeDef));
            (void)USBD_CtlSendData(pdev,
                                   (uint8_t *)&CDC_NCM_NtbParameters,
                                   len);
          }
          else if (req->bRequest == CDC_NCM_GET_NTB_INPUT_SIZE)
          {
            uint8_t buf[4];
            buf[0] = LOBYTE(LOWORD(hncm->NtbInMaxSize));
            buf[1] = HIBYTE(LOWORD(hncm->NtbInMaxSize));
            buf[2] = LOBYTE(HIWORD(hncm->NtbInMaxSize));
            buf[3] = HIBYTE(HIWORD(hncm->NtbInMaxSize));
            (void)USBD_CtlSendData(pdev, buf, 4U);
          }
          else
          {
            if (fops != NULL)
            {
              (void)fops->Control(req->bRequest, CDC_NCM_EP0_Buf, (uint16_t)req->wLength);
            }
            (void)USBD_CtlSendData(pdev, CDC_NCM_EP0_Buf, req->wLength);
          }
        }
        else
        {
          /* Host-to-device with data phase */
          hncm->NotifyLength = req->wLength;
          (void)USBD_CtlPrepareRx(pdev, CDC_NCM_EP0_Buf, req->wLength);
        }
      }
      else
      {
        /* No data phase – handle inline */
        if (req->bRequest == CDC_NCM_SET_NTB_INPUT_SIZE && req->wLength == 0U)
        {
          /* Some hosts send wLength=0; ignore (NtbInMaxSize already set) */
        }
        if (fops != NULL)
        {
          (void)fops->Control(req->bRequest, NULL, 0U);
        }
      }
      break;

    /* ---- Standard requests ---- */
    case USB_REQ_TYPE_STANDARD:
      if (req->bRequest == USB_REQ_SET_INTERFACE)
      {
        /* Host selects alternate 0 (no EPs) or alternate 1 (active).
         * We keep endpoints open for simplicity; a full implementation
         * would close/reopen on alt-0. */
        if (fops != NULL)
        {
          (void)fops->Control(req->bRequest, NULL, 0U);
        }
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }
  return (uint8_t)ret;
}

static uint8_t USBD_CDC_NCM_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_NCM_HandleTypeDef *hncm =
      (USBD_CDC_NCM_HandleTypeDef *)(pdev->pClassData_CDC_NCM);
  USBD_CDC_NCM_ItfTypeDef    *fops =
      (USBD_CDC_NCM_ItfTypeDef *)(pdev->pUserData_CDC_NCM);

  if (hncm == NULL || fops == NULL)
  {
    return (uint8_t)USBD_OK;
  }

  /* If this is a SET_NTB_INPUT_SIZE response (4 bytes) */
  if (hncm->NotifyLength == 4U)
  {
    uint32_t new_size =
        (uint32_t)CDC_NCM_EP0_Buf[0]        |
        ((uint32_t)CDC_NCM_EP0_Buf[1] << 8)  |
        ((uint32_t)CDC_NCM_EP0_Buf[2] << 16) |
        ((uint32_t)CDC_NCM_EP0_Buf[3] << 24);

    if (new_size <= CDC_NCM_NTB_IN_MAX_SIZE)
    {
      hncm->NtbInMaxSize = new_size;
    }
  }

  (void)fops->Control(0xFFU, CDC_NCM_EP0_Buf, (uint16_t)hncm->NotifyLength);
  hncm->NotifyLength = 0U;
  return (uint8_t)USBD_OK;
}

static uint8_t USBD_CDC_NCM_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_NCM_HandleTypeDef *hncm =
      (USBD_CDC_NCM_HandleTypeDef *)(pdev->pClassData_CDC_NCM);
  USBD_CDC_NCM_ItfTypeDef    *fops =
      (USBD_CDC_NCM_ItfTypeDef *)(pdev->pUserData_CDC_NCM);

  if (hncm == NULL)
  {
    return (uint8_t)USBD_OK;
  }

  if (epnum == (CDC_NCM_IN_EP & 0x7FU))
  {
    PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
    if (hpcd->IN_ep[epnum].xfer_len == 0U)
    {
      /* Zero-length packet already sent if needed by lower layer */
    }
    hncm->TxState = 0U;

    if (fops != NULL)
    {
      (void)fops->TransmitCplt(hncm->TxNtbBuffer, &hncm->TxNtbLength, epnum);
    }
  }
  return (uint8_t)USBD_OK;
}

static uint8_t USBD_CDC_NCM_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_NCM_HandleTypeDef *hncm =
      (USBD_CDC_NCM_HandleTypeDef *)(pdev->pClassData_CDC_NCM);

  if (hncm == NULL)
  {
    return (uint8_t)USBD_OK;
  }

  if (epnum == CDC_NCM_OUT_EP)
  {
    uint32_t len = USBD_LL_GetRxDataSize(pdev, epnum);
    hncm->RxLength += len;

    /* A complete NTB is received when we get a short packet or the
     * accumulated size matches the NTH wBlockLength field. */
    uint8_t done = 0U;
    if (hncm->RxLength >= sizeof(USBD_CDC_NCM_NTH16TypeDef))
    {
      USBD_CDC_NCM_NTH16TypeDef *nth = (USBD_CDC_NCM_NTH16TypeDef *)hncm->RxBuffer;
      if (nth->dwSignature == CDC_NCM_NTH16_SIGNATURE &&
          hncm->RxLength >= (uint32_t)nth->wBlockLength)
      {
        done = 1U;
      }
    }

    if (done || len < hncm->MaxPcktLen)
    {
      /* Parse and deliver */
      USBD_CDC_NCM_ProcessNTB(pdev, hncm->RxBuffer, hncm->RxLength);
      hncm->RxLength = 0U;
    }

    /* Re-arm OUT endpoint */
    uint32_t remaining = CDC_NCM_NTB_OUT_MAX_SIZE - hncm->RxLength;
    uint32_t rx_size   = MIN(remaining, hncm->MaxPcktLen);
    (void)USBD_LL_PrepareReceive(pdev,
                                  CDC_NCM_OUT_EP,
                                  hncm->RxBuffer + hncm->RxLength,
                                  rx_size);
  }
  return (uint8_t)USBD_OK;
}

/* ========================================================================== */
/* Descriptor accessors                                                        */
/* ========================================================================== */

static uint8_t *USBD_CDC_NCM_GetFSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_NCM_CfgFSDesc);
  return USBD_CDC_NCM_CfgFSDesc;
}

static uint8_t *USBD_CDC_NCM_GetHSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_NCM_CfgHSDesc);
  return USBD_CDC_NCM_CfgHSDesc;
}

static uint8_t *USBD_CDC_NCM_GetOtherSpeedCfgDesc(uint16_t *length)
{
  (void)memcpy(USBD_CDC_NCM_OtherSpeedCfgDesc,
               USBD_CDC_NCM_CfgFSDesc,
               sizeof(USBD_CDC_NCM_CfgFSDesc));
  *length = (uint16_t)sizeof(USBD_CDC_NCM_OtherSpeedCfgDesc);
  return USBD_CDC_NCM_OtherSpeedCfgDesc;
}

static uint8_t *USBD_CDC_NCM_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_NCM_DeviceQualifierDesc);
  return USBD_CDC_NCM_DeviceQualifierDesc;
}

/* ========================================================================== */
/* Public API                                                                  */
/* ========================================================================== */

/**
  * @brief  USBD_CDC_NCM_RegisterInterface
  *         Register the application-layer callbacks.
  */
uint8_t USBD_CDC_NCM_RegisterInterface(USBD_HandleTypeDef *pdev,
                                        USBD_CDC_NCM_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }
  pdev->pUserData_CDC_NCM = fops;
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_NCM_SetTxBuffer
  *         Set the transmit NTB buffer and length (pre-built by application).
  */
uint8_t USBD_CDC_NCM_SetTxBuffer(USBD_HandleTypeDef *pdev,
                                   uint8_t *pbuff, uint32_t length)
{
  USBD_CDC_NCM_HandleTypeDef *hncm =
      (USBD_CDC_NCM_HandleTypeDef *)(pdev->pClassData_CDC_NCM);
  if (hncm == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }
  /* If pbuff points to the internal NTB buffer, just record length */
  if (pbuff != hncm->TxNtbBuffer)
  {
    uint32_t copy_len = MIN(length, CDC_NCM_NTB_IN_MAX_SIZE);
    (void)memcpy(hncm->TxNtbBuffer, pbuff, copy_len);
    hncm->TxNtbLength = copy_len;
  }
  else
  {
    hncm->TxNtbLength = MIN(length, CDC_NCM_NTB_IN_MAX_SIZE);
  }
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_NCM_SetRxBuffer  (kept for API symmetry)
  */
uint8_t USBD_CDC_NCM_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff)
{
  UNUSED(pdev);
  UNUSED(pbuff);
  /* RxBuffer is managed internally; this exists for API compatibility */
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_NCM_ReceivePacket
  *         Re-arm the OUT endpoint to receive the next NTB chunk.
  */
uint8_t USBD_CDC_NCM_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_NCM_HandleTypeDef *hncm =
      (USBD_CDC_NCM_HandleTypeDef *)(pdev->pClassData_CDC_NCM);
  if (hncm == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }
  return USBD_LL_PrepareReceive(pdev, CDC_NCM_OUT_EP,
                                 hncm->RxBuffer, hncm->MaxPcktLen);
}

/**
  * @brief  USBD_CDC_NCM_TransmitPacket
  *         Initiate sending the NTB currently in TxNtbBuffer.
  *         Call USBD_CDC_NCM_SetTxBuffer() first.
  */
uint8_t USBD_CDC_NCM_TransmitPacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_NCM_HandleTypeDef *hncm =
      (USBD_CDC_NCM_HandleTypeDef *)(pdev->pClassData_CDC_NCM);
  if (hncm == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }
  if (hncm->TxState != 0U)
  {
    return (uint8_t)USBD_BUSY;
  }

  hncm->TxState = 1U;
  pdev->ep_in[CDC_NCM_IN_EP & 0xFU].total_length = hncm->TxNtbLength;

  return USBD_LL_Transmit(pdev, CDC_NCM_IN_EP,
                           hncm->TxNtbBuffer, hncm->TxNtbLength);
}

/**
  * @brief  USBD_CDC_NCM_SendNotification
  *         Send a NETWORK_CONNECTION notification to the host.
  * @param  value  CDC_NCM_NET_CONNECTED or CDC_NCM_NET_DISCONNECTED
  */
uint8_t USBD_CDC_NCM_SendNotification(USBD_HandleTypeDef *pdev, uint8_t value)
{
  USBD_CDC_NCM_HandleTypeDef *hncm =
      (USBD_CDC_NCM_HandleTypeDef *)(pdev->pClassData_CDC_NCM);
  if (hncm == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* USB CDC notification: NETWORK_CONNECTION (§6.3.1) */
  hncm->NotifyBuffer[0] = 0xA1U; /* bmRequestType */
  hncm->NotifyBuffer[1] = CDC_NCM_NOTIFY_NETWORK_CONNECTION; /* bNotification */
  hncm->NotifyBuffer[2] = value; /* wValue lo */
  hncm->NotifyBuffer[3] = 0x00U; /* wValue hi */
  hncm->NotifyBuffer[4] = CDC_NCM_CMD_ITF_NBR; /* wIndex lo */
  hncm->NotifyBuffer[5] = 0x00U; /* wIndex hi */
  hncm->NotifyBuffer[6] = 0x00U; /* wLength lo */
  hncm->NotifyBuffer[7] = 0x00U; /* wLength hi */
  hncm->NotifyLength    = 8U;

  return USBD_LL_Transmit(pdev, CDC_NCM_CMD_EP,
                           hncm->NotifyBuffer, hncm->NotifyLength);
}

/* ========================================================================== */
/* Descriptor patch function                                                   */
/* ========================================================================== */

/**
  * @brief  USBD_Update_CDC_NCM_DESC
  *         Patch endpoint addresses, interface numbers and string index into
  *         a CDC NCM configuration descriptor.
  *
  *  Verified absolute byte offsets for the 94-byte descriptor layout:
  *    desc[11]  IAD bFirstInterface    → cmd_itf
  *    desc[16]  IAD iFunction          → str_idx
  *    desc[19]  CtrlItf bItfNumber     → cmd_itf
  *    desc[25]  CtrlItf iInterface     → str_idx
  *    desc[34]  Union bMasterInterface → cmd_itf
  *    desc[35]  Union bSlaveInterface0 → com_itf
  *    desc[57]  Interrupt EP address   → cmd_ep
  *    desc[64]  DataItf-0 bItfNumber   → com_itf
  *    desc[73]  DataItf-1 bItfNumber   → com_itf
  *    desc[82]  Bulk OUT EP address    → out_ep
  *    desc[89]  Bulk IN  EP address    → in_ep
  */
void USBD_Update_CDC_NCM_DESC(uint8_t *desc,
                               uint8_t cmd_itf,
                               uint8_t com_itf,
                               uint8_t in_ep,
                               uint8_t out_ep,
                               uint8_t cmd_ep,
                               uint8_t str_idx)
{
  /* Cache global EP/ITF numbers used by Setup/DataIn/DataOut callbacks */
  CDC_NCM_CMD_ITF_NBR  = cmd_itf;
  CDC_NCM_COM_ITF_NBR  = com_itf;
  CDC_NCM_IN_EP        = in_ep;
  CDC_NCM_OUT_EP       = out_ep;
  CDC_NCM_CMD_EP       = cmd_ep;
  CDC_NCM_STR_DESC_IDX = str_idx;

  /* IAD */
  desc[11] = cmd_itf;   /* bFirstInterface */
  desc[16] = str_idx;   /* iFunction       */

  /* Control Interface Descriptor */
  desc[19] = cmd_itf;   /* bInterfaceNumber */
  desc[25] = str_idx;   /* iInterface       */

  /* CDC Union Functional Descriptor */
  desc[34] = cmd_itf;   /* bMasterInterface */
  desc[35] = com_itf;   /* bSlaveInterface0 */

  /* Interrupt IN Endpoint */
  desc[57] = cmd_ep;    /* bEndpointAddress */

  /* Data Interface alt-0 */
  desc[64] = com_itf;   /* bInterfaceNumber */

  /* Data Interface alt-1 */
  desc[73] = com_itf;   /* bInterfaceNumber */

  /* Bulk OUT Endpoint */
  desc[82] = out_ep;    /* bEndpointAddress */

  /* Bulk IN Endpoint */
  desc[89] = in_ep;     /* bEndpointAddress */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
