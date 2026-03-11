/**
  ******************************************************************************
  * @file    usbd_midi.h
  * @author  AL94 / Community
  * @brief   Header file for the usbd_midi.c file.
  *
  *          USB MIDI 1.0 Class implementation for STM32 Composite USB framework.
  *          Implements USB Audio Class (Class 0x01) with MIDI Streaming subclass.
  *
  ******************************************************************************
  * @attention
  *
  * This software component is provided AS-IS, without any warranty.
  * Compatible with the AL94 I-CUBE-USBD-COMPOSITE framework.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_MIDI_H
#define __USB_MIDI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_ioreq.h"
#include "AL94.I-CUBE-USBD-COMPOSITE_conf.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_MIDI
  * @brief This file is the Header file for usbd_midi.c
  * @{
  */

/** @defgroup USBD_MIDI_Exported_Defines
  * @{
  */

#define MIDI_STR_DESC                           "STM32 USB MIDI"

/* USB MIDI uses Audio Class (0x01) with MIDI Streaming subclass (0x03) */
#define USB_AUDIO_CLASS                         0x01U
#define USB_AUDIO_SUBCLASS_AUDIOCONTROL         0x01U
#define USB_AUDIO_SUBCLASS_MIDISTREAMING        0x03U
#define USB_AUDIO_PROTOCOL_UNDEFINED            0x00U

/* Audio Class-Specific Descriptor Types */
#define CS_INTERFACE                            0x24U
#define CS_ENDPOINT                             0x25U

/* Audio Class-Specific AC Interface Subtypes */
#define HEADER_SUBTYPE                          0x01U

/* MIDI Streaming Interface Subtypes */
#define MS_HEADER_SUBTYPE                       0x01U
#define MIDI_IN_JACK_SUBTYPE                    0x02U
#define MIDI_OUT_JACK_SUBTYPE                   0x03U

/* MIDI Jack Types */
#define JACK_TYPE_EMBEDDED                      0x01U
#define JACK_TYPE_EXTERNAL                      0x02U

/* MIDI Streaming Endpoint Subtype */
#define MS_GENERAL_SUBTYPE                      0x01U

/* Endpoint packet size */
#define MIDI_DATA_FS_MAX_PACKET_SIZE            64U
#define MIDI_DATA_HS_MAX_PACKET_SIZE            512U

/* Descriptor sizes:
 * Config Header (9) + AC Interface (9) + AC Header CS (9)
 * + MS Interface (9) + MS Header CS (7)
 * + MIDI IN Jack Embedded (6) + MIDI IN Jack External (6)
 * + MIDI OUT Jack Embedded (9) + MIDI OUT Jack External (9)
 * + Bulk OUT Endpoint (9) + MS Bulk OUT CS Endpoint (5)
 * + Bulk IN Endpoint (9)  + MS Bulk IN CS Endpoint (5)
 * Total without config header (9): 83 bytes
 */
#define MIDI_AC_HEADER_SIZE                     9U
#define MIDI_AC_CS_HEADER_SIZE                  9U
#define MIDI_MS_INTERFACE_SIZE                  9U
#define MIDI_MS_CS_HEADER_SIZE                  7U
#define MIDI_IN_JACK_SIZE                       6U
#define MIDI_OUT_JACK_SIZE                      9U
#define MIDI_BULK_EP_SIZE                       9U
#define MIDI_MS_BULK_EP_SIZE                    5U

/* Total config descriptor size (excluding the 9-byte config descriptor header):
 * = AC_ITF(9) + AC_CS_HDR(9) + MS_ITF(9) + MS_CS_HDR(7)
 *   + MIDI_IN_EMB(6) + MIDI_IN_EXT(6) + MIDI_OUT_EMB(9) + MIDI_OUT_EXT(9)
 *   + EP_OUT(9) + EP_OUT_CS(5) + EP_IN(9) + EP_IN_CS(5)
 * = 92 bytes (plus config header = 101 total)
 */
#define USB_MIDI_CONFIG_DESC_SIZ                101U

/* MIDI Jack IDs */
#define MIDI_IN_JACK_EMB_ID                     0x01U
#define MIDI_IN_JACK_EXT_ID                     0x02U
#define MIDI_OUT_JACK_EMB_ID                    0x03U
#define MIDI_OUT_JACK_EXT_ID                    0x04U

/* Interface numbers (updated by USBD_Update_MIDI_DESC) */
#define _MIDI_AC_ITF_NBR                        0x00U
#define _MIDI_MS_ITF_NBR                        0x01U

/* Endpoints (updated by USBD_Update_MIDI_DESC) */
#define _MIDI_IN_EP                             0x81U
#define _MIDI_OUT_EP                            0x01U

/* String descriptor index */
#define _MIDI_STR_DESC_IDX                      0x00U

/**
  * @}
  */

/** @defgroup USBD_MIDI_Exported_TypesDefinitions
  * @{
  */

/**
  * @brief  USBD_MIDI_ItfTypeDef
  *         Application callbacks interface structure for MIDI
  */
typedef struct _USBD_MIDI_Itf
{
  int8_t (*Init)(void);
  int8_t (*DeInit)(void);
  int8_t (*Receive)(uint8_t *msg, uint32_t len);
  int8_t (*TransmitCplt)(uint8_t *msg, uint32_t *len, uint8_t epnum);
} USBD_MIDI_ItfTypeDef;

/**
  * @brief  USBD_MIDI_HandleTypeDef
  *         Internal handle structure
  */
typedef struct
{
  uint8_t  *RxBuffer;
  uint8_t  *TxBuffer;
  uint32_t RxLength;
  uint32_t TxLength;
  __IO uint32_t TxState;
  __IO uint32_t RxState;
} USBD_MIDI_HandleTypeDef;

/**
  * @}
  */

/** @defgroup USBD_MIDI_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  USBD_MIDI;

/* Interface numbers (assigned by composite layer) */
extern uint8_t MIDI_AC_ITF_NBR;
extern uint8_t MIDI_MS_ITF_NBR;

/* Endpoint numbers (assigned by composite layer) */
extern uint8_t MIDI_IN_EP;
extern uint8_t MIDI_OUT_EP;

/* String descriptor index */
extern uint8_t MIDI_STR_DESC_IDX;

/**
  * @}
  */

/** @defgroup USBD_MIDI_Exported_Functions
  * @{
  */

uint8_t USBD_MIDI_RegisterInterface(USBD_HandleTypeDef *pdev,
                                    USBD_MIDI_ItfTypeDef *fops);

uint8_t USBD_MIDI_SetTxBuffer(USBD_HandleTypeDef *pdev,
                              uint8_t *pbuff,
                              uint32_t length);

uint8_t USBD_MIDI_SetRxBuffer(USBD_HandleTypeDef *pdev,
                              uint8_t *pbuff);

uint8_t USBD_MIDI_ReceivePacket(USBD_HandleTypeDef *pdev);

uint8_t USBD_MIDI_TransmitPacket(USBD_HandleTypeDef *pdev);

/**
  * @brief  Update MIDI descriptor with assigned interface/endpoint numbers.
  *         Called by USBD_COMPOSITE_Mount_Class().
  */
void USBD_Update_MIDI_DESC(uint8_t *desc,
                           uint8_t ac_itf,
                           uint8_t ms_itf,
                           uint8_t in_ep,
                           uint8_t out_ep,
                           uint8_t str_idx);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USB_MIDI_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
