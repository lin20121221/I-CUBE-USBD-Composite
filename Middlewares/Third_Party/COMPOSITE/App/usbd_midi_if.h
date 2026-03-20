/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_midi_if.h
  * @brief          : Header for usbd_midi_if.c file.
  *
  *                   Application-level interface for USB MIDI device.
  *                   Users should implement the callback functions and use
  *                   MIDI_Transmit() to send MIDI data to the host.
  *
  *          USB-MIDI Event Packet format (4 bytes per event):
  *          Byte 0: Cable Number (upper 4 bits) | Code Index Number (lower 4 bits)
  *          Byte 1: MIDI Status byte  (e.g. 0x90 = Note On, channel 1)
  *          Byte 2: MIDI Data byte 1  (e.g. note number)
  *          Byte 3: MIDI Data byte 2  (e.g. velocity)
  *
  *          Common Code Index Numbers (CIN):
  *            0x08 = Note Off
  *            0x09 = Note On
  *            0x0B = Control Change
  *            0x0C = Program Change
  *            0x0E = Pitch Bend
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_MIDI_IF_H__
#define __USBD_MIDI_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_midi.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief For Usb device.
  * @{
  */

/** @defgroup USBD_MIDI_IF
  * @brief USB MIDI device module
  * @{
  */

/** @defgroup USBD_MIDI_IF_Exported_Defines
  * @{
  */

/* USER CODE BEGIN EXPORTED_DEFINES */

/** Size of the internal receive buffer (must be multiple of 4 for MIDI packets) */
#define MIDI_RX_BUFFER_SIZE    64U

/** Size of the internal transmit buffer */
#define MIDI_TX_BUFFER_SIZE    64U

/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_MIDI_IF_Exported_Types
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */

/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_MIDI_IF_Exported_Macros
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACRO */

/* USER CODE END EXPORTED_MACRO */

/**
  * @}
  */

/** @defgroup USBD_MIDI_IF_Exported_Variables
  * @{
  */

/** MIDI Interface callback structure */
extern USBD_MIDI_ItfTypeDef USBD_MIDI_fops;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_MIDI_IF_Exported_FunctionsPrototype
  * @{
  */

/**
  * @brief  Transmit a USB-MIDI event packet to the host.
  *
  *         Data must be formatted as USB-MIDI event packets (4 bytes each).
  *         Example - send Note On (channel 1, note 60, velocity 100):
  *           uint8_t msg[4] = {0x09, 0x90, 60, 100};
  *           MIDI_Transmit(msg, 4);
  *
  * @param  Buf  Pointer to USB-MIDI packet buffer
  * @param  Len  Number of bytes to transmit (must be multiple of 4)
  * @retval USBD_OK, USBD_BUSY, or USBD_FAIL
  */
uint8_t MIDI_Transmit(uint8_t *Buf, uint16_t Len);

/* USER CODE BEGIN EXPORTED_FUNCTIONS */

/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_MIDI_IF_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
