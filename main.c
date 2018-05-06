/*
 * main.c
 *
 *  Created on: 2018 Mar 17 21:28:38
 *  Author: GW
 */




#include <DAVE.h>
#include <string.h>
#include "usbd_gs_can.h"
#include "queue.h"
#include "led.h"
#include "can.h"
#include "hal.h"


#define CAN_QUEUE_SIZE     64

#define LED1_GPIO_Port     XMC_GPIO_PORT5
#define LED1_Pin           9
#define LED2_GPIO_Port     XMC_GPIO_PORT5
#define LED2_Pin           8


led_data_t hLED;

queue_t *q_frame_pool;
queue_t *q_from_host;
queue_t *q_to_host;

//can_data_t hCAN;

uint8_t send_to_host_or_enqueue(struct gs_host_frame *frame);
void send_to_host(void);

/*
void test_can()
{
   struct gs_host_frame t_Frame;
   static int Send = 0;
   static int Recv = 1;
   static int Return = 0;

   can_init(0);
   can_init(1);

   can_enable(0, 1, 0, 0);
   can_enable(1, 1, 0, 0);


   while (1)
   {
      t_Frame.can_id = 0x100 | CAN_EFF_FLAG;
      t_Frame.can_dlc = 1;
      t_Frame.data[0] = 99;
      if (Send) {
         Return = can_send(0, &t_Frame);
         //Return = can_send(0, &t_Frame);
      }

      memset(&t_Frame, 0, sizeof(struct gs_host_frame));
      if (can_is_rx_pending(1)) {
         if (Recv)
         {
            while (can_is_rx_pending(1))
               Return = can_receive(1, &t_Frame);
         }
      }
   }
}
*/

int main(void)
{
   int16_t s16_Error = 0;
   DAVE_STATUS_t t_DaveStatus;
   USBD_GS_CAN_Status_t t_GsCanStatus;
   uint32_t last_can_error_status = 0;


   t_DaveStatus = DAVE_Init();

   if(t_DaveStatus != DAVE_STATUS_SUCCESS)
   {
      XMC_DEBUG("DAVE APPs initialization failed\n");
      s16_Error = -1;
   }

   //test_can();

   if (s16_Error == 0)
   {
      struct gs_host_frame *pt_MsgBuf = calloc(CAN_QUEUE_SIZE, sizeof(struct gs_host_frame));

      q_frame_pool = queue_create(CAN_QUEUE_SIZE);
      q_from_host  = queue_create(CAN_QUEUE_SIZE);
      q_to_host    = queue_create(CAN_QUEUE_SIZE);
      for (unsigned i=0; i < CAN_QUEUE_SIZE; i++)
      {
         queue_push_back(q_frame_pool, &pt_MsgBuf[i]);
      }
   }

   if (s16_Error == 0)
   {
      led_init(&hLED, LED1_GPIO_Port, LED1_Pin, true, LED2_GPIO_Port, LED2_Pin, true);
      led_set_mode(&hLED, led_mode_off);
   }

   if (s16_Error == 0)
   {
      t_GsCanStatus = USBD_GS_CAN_Init(NULL, NULL, NULL);
      if (t_GsCanStatus != USBD_GS_CAN_STATUS_SUCCESS)
      {
         s16_Error = -1;
      }
   }

   if (s16_Error != 0)
   {
      XMC_DEBUG("Target Initialization failed\n");
      while (1)
      {
      }
   }

   XMC_DEBUG("Starting application\n");


   while(!USBD_GS_CAN_IsEnumDone())
   {
      // Wait until we are enumerated
   }


   while (1)
   {
      USBD_GS_CAN_Receive();

      struct gs_host_frame *frame = queue_pop_front(q_from_host);
      if (frame != 0)
      { // send can message from host
         if (can_send(frame->channel, frame))
         {
            frame->timestamp_us = (uint32_t)HAL_GetTick_us();
            send_to_host_or_enqueue(frame);
            led_indicate_trx(&hLED, led_2);
         }
         else
         {
            queue_push_front(q_from_host, frame); // retry later
         }
      }

      if (USBD_GS_CAN_TxReady())
      {
         send_to_host();
      }

      if (can_is_rx_pending(0)) {
         struct gs_host_frame *frame = queue_pop_front(q_frame_pool);
         if ((frame != 0) && can_receive(0, frame)) {

            frame->timestamp_us = (uint32_t)HAL_GetTick_us();
            frame->echo_id = 0xFFFFFFFF; // not a echo frame
            frame->channel = 0;
            frame->flags = 0;
            frame->reserved = 0;
            send_to_host_or_enqueue(frame);

            led_indicate_trx(&hLED, led_1);

         } else {
            queue_push_back(q_frame_pool, frame);
         }

      }

      uint32_t can_err = can_get_error_status(0);
      if (can_err != last_can_error_status)
      {
         struct gs_host_frame *frame = queue_pop_front(q_frame_pool);
         if (frame != 0)
         {
            frame->timestamp_us = (uint32_t)HAL_GetTick_us();
            if (can_parse_error_status(can_err, frame))
            {
               send_to_host_or_enqueue(frame);
               last_can_error_status = can_err;
            }
            else
            {
               queue_push_back(q_frame_pool, frame);
            }
         }
      }

      led_update(&hLED);

   }
}


uint8_t send_to_host_or_enqueue(struct gs_host_frame *frame)
{
   if (USBD_GS_CAN_GetProtocolVersion() == 2)
   {
      queue_push_back(q_to_host, frame);
      return true;
   }
   else
   {
      bool retval = false;
      if ( USBD_GS_CAN_SendFrame(frame) == USBD_OK )
      {
         queue_push_back(q_frame_pool, frame);
         retval = true;
      }
      else
      {
         queue_push_back(q_to_host, frame);
      }
      return retval;
   }
}


void send_to_host(void)
{
   uint8_t buf[5*sizeof(struct gs_host_frame)];
   struct gs_host_frame *frame;

   unsigned num_msgs = queue_size(q_to_host);

   if (num_msgs>0)
   {
      // send received message or echo message to host
      if (USBD_GS_CAN_GetProtocolVersion() == 2)
      {
         if (num_msgs>1)
         {
            num_msgs = 1;
         }

         for (unsigned i=0; i<num_msgs; i++)
         {
            frame = queue_pop_front(q_to_host);
            memcpy(&buf[i*sizeof(struct gs_host_frame)], frame, sizeof(struct gs_host_frame));
            queue_push_back(q_frame_pool, frame);
         }
         USBD_GS_CAN_Transmit(buf, num_msgs * sizeof(struct gs_host_frame));
      }
      else
      {
         struct gs_host_frame *frame = queue_pop_front(q_to_host);
         if (USBD_GS_CAN_SendFrame(frame) == USBD_OK)
         {
            queue_push_back(q_frame_pool, frame);
         }
         else
         {
            queue_push_front(q_to_host, frame);
         }
      }
   }
}
