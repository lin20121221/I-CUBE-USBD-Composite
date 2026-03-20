/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_midi_if.c
  * @brief          : USB MIDI Device application-layer interface.
  *
  *          This file provides the MIDI application callbacks and the
  *          MIDI_Transmit() function for sending MIDI data to the host.
  *
  *          USB-MIDI Event Packet format (4 bytes per event):
  *            Byte 0: Cable Number [7:4] | Code Index Number [3:0]
  *            Byte 1: MIDI Status byte
  *            Byte 2: MIDI Data byte 1
  *            Byte 3: MIDI Data byte 2
  *
  *          Usage example (send Note On then Note Off):
  *            uint8_t note_on[4]  = {0x09, 0x90, 60, 100};
  *            uint8_t note_off[4] = {0x08, 0x80, 60,   0};
  *            MIDI_Transmit(note_on,  4);
  *            HAL_Delay(500);
  *            MIDI_Transmit(note_off, 4);
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_midi_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_MIDI_IF
  * @{
  */

/** @defgroup USBD_MIDI_IF_Private_TypesDefinitions
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_MIDI_IF_Private_Defines
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_MIDI_IF_Private_Macros
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_MIDI_IF_Private_Variables
  * @{
  */

/* USER CODE BEGIN PRIVATE_VARIABLES */

/** Receive buffer for incoming USB-MIDI packets from host */
static uint8_t MIDI_RxBuf[MIDI_RX_BUFFER_SIZE];

/** Transmit buffer for outgoing USB-MIDI packets to host */
static uint8_t MIDI_TxBuf[MIDI_TX_BUFFER_SIZE];

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_MIDI_IF_Exported_Variables
  * @{
  */

extern USBD_HandleTypeDef hUsbDevice;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_MIDI_IF_Private_FunctionPrototypes
  * @{
  */

static int8_t MIDI_Init(void);
static int8_t MIDI_DeInit(void);
static int8_t MIDI_Receive(uint8_t *pbuf, uint32_t Len);
static int8_t MIDI_TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_MIDI_ItfTypeDef USBD_MIDI_fops =
{
  MIDI_Init,
  MIDI_DeInit,
  MIDI_Receive,
  MIDI_TransmitCplt
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  MIDI_Init
  *         Initialize the MIDI low layer.
  *         Called when USB is enumerated and the MIDI interface is configured.
  * @retval USBD_OK
  */
static int8_t MIDI_Init(void)
{
  /* USER CODE BEGIN MIDI_Init */

  /* Set the receive buffer so the driver can arm the OUT endpoint */
  USBD_MIDI_SetRxBuffer(&hUsbDevice, MIDI_RxBuf);

  return (USBD_OK);
  /* USER CODE END MIDI_Init */
}

/**
  * @brief  MIDI_DeInit
  *         De-Initialize the MIDI low layer.
  * @retval USBD_OK
  */
static int8_t MIDI_DeInit(void)
{
  /* USER CODE BEGIN MIDI_DeInit */

  return (USBD_OK);
  /* USER CODE END MIDI_DeInit */
}

/**
  * @brief  MIDI_Receive
  *         Called when USB-MIDI data is received from the host (OUT endpoint).
  *
  *         Data arrives as 4-byte USB-MIDI event packets.
  *         Parse them here or forward to your MIDI engine.
  *
  *         Example parsing:
  *           for (uint32_t i = 0; i < Len; i += 4) {
  *             uint8_t cin    = pbuf[i] & 0x0F;   // Code Index Number
  *             uint8_t status = pbuf[i+1];         // MIDI status
  *             uint8_t data1  = pbuf[i+2];         // MIDI data byte 1
  *             uint8_t data2  = pbuf[i+3];         // MIDI data byte 2
  *             // process...
  *           }
  *
  * @param  pbuf: pointer to received USB-MIDI packet(s)
  * @param  Len:  number of bytes received (multiple of 4)
  * @retval USBD_OK
  */
static int8_t MIDI_Receive(uint8_t *pbuf, uint32_t Len)
{
  /* USER CODE BEGIN MIDI_Receive */

  /*
   * Process incoming USB-MIDI packets.
   * Each packet is 4 bytes:
   *   pbuf[0] = Cable Number | Code Index Number
   *   pbuf[1] = MIDI status byte
   *   pbuf[2] = MIDI data byte 1
   *   pbuf[3] = MIDI data byte 2
   */

  UNUSED(pbuf);
  UNUSED(Len);

  /* Re-arm the OUT endpoint to receive the next packet */
  USBD_MIDI_ReceivePacket(&hUsbDevice);

  return (USBD_OK);
  /* USER CODE END MIDI_Receive */
}

/**
  * @brief  MIDI_TransmitCplt
  *         Called when a USB-MIDI IN transfer is complete (data sent to host).
  * @param  Buf:   pointer to transmitted buffer
  * @param  Len:   pointer to length
  * @param  epnum: endpoint number
  * @retval USBD_OK
  */
static int8_t MIDI_TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  /* USER CODE BEGIN MIDI_TransmitCplt */

  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);

  return (USBD_OK);
  /* USER CODE END MIDI_TransmitCplt */
}

/**
  * @brief  MIDI_Transmit
  *         Transmit USB-MIDI event packet(s) to the host.
  *
  *         Buf must point to one or more 4-byte USB-MIDI event packets.
  *         Len must be a multiple of 4.
  *
  *         Example – send a single Note On (channel 1, note 60, velocity 100):
  *           uint8_t msg[4] = {0x09, 0x90, 60, 100};
  *           MIDI_Transmit(msg, 4);
  *
  * @param  Buf: pointer to USB-MIDI packet buffer
  * @param  Len: number of bytes (must be multiple of 4, max MIDI_TX_BUFFER_SIZE)
  * @retval USBD_OK if transmission started, USBD_BUSY if previous TX pending
  */
uint8_t MIDI_Transmit(uint8_t *Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;

  /* USER CODE BEGIN MIDI_Transmit */

  extern USBD_MIDI_HandleTypeDef MIDI_Class_Data;

  if (MIDI_Class_Data.TxState != 0U)
  {
    return USBD_BUSY;
  }

  /* Copy to internal TX buffer (required for DMA safety on some STM32 families) */
  if (Len > MIDI_TX_BUFFER_SIZE)
  {
    Len = MIDI_TX_BUFFER_SIZE;
  }

  for (uint16_t i = 0U; i < Len; i++)
  {
    MIDI_TxBuf[i] = Buf[i];
  }

  USBD_MIDI_SetTxBuffer(&hUsbDevice, MIDI_TxBuf, Len);
  result = USBD_MIDI_TransmitPacket(&hUsbDevice);

  /* USER CODE END MIDI_Transmit */

  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
