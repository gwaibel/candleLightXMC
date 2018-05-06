
#ifndef USBD_WINUSB_H
#define USBD_WINUSB_H


/***********************************************************************************************************************
 * HEADER FILES                                                                                                      
 **********************************************************************************************************************/
#include <xmc_common.h>
#include <DAVE_Common.h>
#include "USBD/usbd.h"
#include "USBD/usb/core/endpoint_stream.h"
#include "USBD/usb/core/usb_task.h"
#include "USBD/usb/core/endpoint.h"
#include "USBD/usbd_conf.h"
#include "queue.h"
#include "led.h"
#include "gs_usb.h"

/**********************************************************************************************************************
 * MACROS                                                                                                            
 **********************************************************************************************************************/
//TODO prüfen:
#define USBD_OK                          0
#define USBD_BUSY                        1



/**********************************************************************************************************************
 * ENUMS
 **********************************************************************************************************************/

/**
 *WinUSB error codes
 */
typedef enum {
	USBD_GS_CAN_STATUS_SUCCESS, /**< Operation was successful */
	USBD_GS_CAN_STATUS_FAILURE, /**< Operation has raised an error */
	USBD_GS_CAN_STATUS_NOT_READY /**< Device/Endpoint is not ready for operation */
} USBD_GS_CAN_Status_t;




/**********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/






USBD_GS_CAN_Status_t USBD_GS_CAN_Init(queue_t *q_frame_pool, queue_t *q_from_host, led_data_t *leds);
uint8_t USBD_GS_CAN_GetProtocolVersion(void);
uint8_t USBD_GS_CAN_TxReady(void);
uint8_t USBD_GS_CAN_Transmit(uint8_t *buf, uint16_t len);
uint8_t USBD_GS_CAN_SendFrame(struct gs_host_frame *frame);

uint8_t USBD_GS_CAN_IsEnumDone(void);
void USBD_GS_CAN_Receive(void);



#endif
