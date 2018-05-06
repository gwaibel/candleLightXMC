/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "xmc_can.h"
#include "can.h"

// 2*Tx+2*31*Rx = 64 objects (maximum for xmc4100)
#define MSGOBJ_TX_CH0              0
#define MSGOBJ_TX_CH1              1
#define MSGOBJ_RX_FIFO_SIZE        4   //=>30 FIFO entries + base object
#define MSGOBJ_RX_FIFO_BASE_CH0    2
#define MSGOBJ_RX_FIFO_BASE_CH1    (MSGOBJ_RX_FIFO_BASE_CH0 + MSGOBJ_RX_FIFO_SIZE)


static XMC_CAN_t* mpt_CanBase = (XMC_CAN_t*)CAN_BASE;
static XMC_CAN_NODE_t* mapt_CanNode[2] = { (XMC_CAN_NODE_t*)CAN_NODE0_BASE, (XMC_CAN_NODE_t*)CAN_NODE1_BASE };
static XMC_CAN_MO_t mat_RxMo[2];
static XMC_CAN_MO_t mat_TxMo[2];
static uint8_t mau8_NodeEnabled[2];
static uint8_t mu8_RxOverflow[2];


void can_init(uint8_t channel)
{
   uint32_t freq;
   uint8_t u8_RxBaseMoNumber;
   uint8_t u8_TxMoNumber;
   XMC_CAN_NODE_t* pt_Node = mapt_CanNode[channel];
   XMC_CAN_MO_t* pt_RxBaseMo = &mat_RxMo[channel];
   XMC_CAN_MO_t* pt_TxMo = &mat_TxMo[channel];
   XMC_CAN_FIFO_CONFIG_t t_FifoCfg;
   static uint8_t hu8_FirstRun = 0;

   if (hu8_FirstRun == 0)
   {
      hu8_FirstRun = 1;

      freq = XMC_CAN_InitEx(mpt_CanBase, XMC_CAN_CANCLKSRC_FPERI, CAN_MODULE_FREQUENCY);
      (void)freq ;
   }

   /* TODO Configure CAN NODE pins */
   //XMC_GPIO_SetMode(CAN_RXD, XMC_GPIO_MODE_INPUT_TRISTATE);
   //XMC_CAN_NODE_SetReceiveInput(pt_Node, XMC_CAN_NODE_RECEIVE_INPUT_RXDCA);
   /* Configure CAN NODE output pin */
   //XMC_GPIO_SetMode(CAN_TXD, XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2);


/* TODO Temporary bitrate setting, only for testing yet... */
   XMC_CAN_NODE_NOMINAL_BIT_TIME_CONFIG_t t_Bittiming;
   t_Bittiming.can_frequency = freq;
   t_Bittiming.baudrate = 125000;
   t_Bittiming.sample_point = 8750;
   t_Bittiming.sjw = 1;
   XMC_CAN_NODE_NominalBitTimeConfigure(pt_Node, &t_Bittiming);

   XMC_CAN_NODE_EnableConfigurationChange(pt_Node);
   XMC_CAN_NODE_SetInitBit(pt_Node);

   u8_RxBaseMoNumber = (channel == 0) ? MSGOBJ_RX_FIFO_BASE_CH0 : MSGOBJ_RX_FIFO_BASE_CH1;
   pt_RxBaseMo->can_mo_ptr = (CAN_MO_TypeDef *)&(CAN_MO->MO[u8_RxBaseMoNumber]);
   pt_RxBaseMo->can_priority = XMC_CAN_ARBITRATION_MODE_IDE_DIR_BASED_PRIO_2;
   pt_RxBaseMo->can_mo_type = XMC_CAN_MO_TYPE_RECMSGOBJ;
   t_FifoCfg.fifo_bottom = u8_RxBaseMoNumber + 1;
   t_FifoCfg.fifo_top = u8_RxBaseMoNumber + MSGOBJ_RX_FIFO_SIZE;
   t_FifoCfg.fifo_base = u8_RxBaseMoNumber + 1;
   XMC_CAN_MO_Config(pt_RxBaseMo);
   XMC_CAN_FIFO_SetSELMO(pt_RxBaseMo, t_FifoCfg.fifo_bottom);
   for (uint8_t u8_ObjNum = t_FifoCfg.fifo_bottom; u8_ObjNum <= t_FifoCfg.fifo_top; u8_ObjNum++)
   {
      XMC_CAN_MO_t t_Mo;
      t_Mo.can_mo_ptr = (CAN_MO_TypeDef *)&(CAN_MO->MO[u8_ObjNum]);
      t_Mo.can_mo_type = XMC_CAN_MO_TYPE_RECMSGOBJ;
      XMC_CAN_MO_Config(&t_Mo); // ???
      XMC_CAN_RXFIFO_ConfigMOSlaveObject(&t_Mo);
      XMC_CAN_AllocateMOtoNodeList(mpt_CanBase, channel, u8_ObjNum); // ???
   }
   XMC_CAN_RXFIFO_ConfigMOBaseObject(pt_RxBaseMo, t_FifoCfg);
   XMC_CAN_AllocateMOtoNodeList(mpt_CanBase, channel, u8_RxBaseMoNumber);


   // Mo0 => channel0_tx, // Mo1 => channel1_tx
   u8_TxMoNumber = (channel == 0) ? MSGOBJ_TX_CH0 : MSGOBJ_TX_CH1;
   pt_TxMo->can_mo_ptr = (CAN_MO_TypeDef *)&(CAN_MO->MO[u8_TxMoNumber]);
   pt_TxMo->can_priority = XMC_CAN_ARBITRATION_MODE_IDE_DIR_BASED_PRIO_2;
   pt_TxMo->can_mo_type = XMC_CAN_MO_TYPE_TRANSMSGOBJ;
   XMC_CAN_MO_Config(pt_TxMo);
   XMC_CAN_AllocateMOtoNodeList(mpt_CanBase, channel, u8_TxMoNumber);

   XMC_CAN_NODE_DisableConfigurationChange(pt_Node);
   // Leave Init bit set; will be cleared by can_enable().
   mau8_NodeEnabled[channel] = 0;
}


/* 0: ok, 1: error
 * Assume that the CAN node is disabled (INIT flag set)
 * */
bool can_set_bittiming(uint8_t channel, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
   uint8_t u8_Ret = 1;
   uint8_t div8;
   XMC_CAN_NODE_t* pt_Node = mapt_CanNode[channel];

   if ( (brp>0) && (brp<=64)
     && (phase_seg1>0) && (phase_seg1<=16)
     && (phase_seg2>0) && (phase_seg2<=8)
     && (sjw>0) && (sjw<=4))
   {
      if (brp > 64)
      {
         div8 = 1;
         brp = brp / 8;
      }
      else
      {
         div8 = 0;
      }

      XMC_CAN_NODE_EnableConfigurationChange(pt_Node);

      /* Configure bit timing register */
      pt_Node->NBTR = ((((phase_seg2 - 1u) << CAN_NODE_NBTR_TSEG2_Pos) & (uint32_t)CAN_NODE_NBTR_TSEG2_Msk) |
                       (((sjw - 1U) << CAN_NODE_NBTR_SJW_Pos) & (uint32_t)CAN_NODE_NBTR_SJW_Msk) |
                       (((phase_seg1 - 1U) << CAN_NODE_NBTR_TSEG1_Pos) & (uint32_t)CAN_NODE_NBTR_TSEG1_Msk) |
                       (((brp - 1U) << CAN_NODE_NBTR_BRP_Pos) & (uint32_t)CAN_NODE_NBTR_BRP_Msk) |
                       ((div8 << CAN_NODE_NBTR_DIV8_Pos) & (uint32_t)CAN_NODE_NBTR_DIV8_Msk));

      XMC_CAN_NODE_DisableConfigurationChange(pt_Node);
      u8_Ret = 0;
   }
   return u8_Ret;
}


void can_enable(uint8_t channel, bool loop_back, bool listen_only, bool one_shot)
{
   XMC_CAN_NODE_t* pt_Node = mapt_CanNode[channel];

   // TODO: listen_only
   // TODO: one_shot

   XMC_CAN_NODE_EnableConfigurationChange(pt_Node);

   if (loop_back)
      XMC_CAN_NODE_EnableLoopBack(pt_Node);
   else
      XMC_CAN_NODE_DisableLoopBack(pt_Node);

   XMC_CAN_NODE_DisableConfigurationChange(pt_Node);

   XMC_CAN_NODE_Enable(pt_Node);
   mau8_NodeEnabled[channel] = 1;
}


void can_disable(uint8_t channel)
{
   XMC_CAN_NODE_t* pt_Node = mapt_CanNode[channel];

   XMC_CAN_NODE_Disable(pt_Node);
   mau8_NodeEnabled[channel] = 0;
}


bool can_is_enabled(uint8_t channel)
{
   return mau8_NodeEnabled[channel];
}


bool can_is_rx_pending(uint8_t channel)
{
   XMC_CAN_MO_t* pt_RxBaseMo = &mat_RxMo[channel];
   XMC_CAN_MO_t t_RxMo;
   uint32_t u32_MoStat;
   uint8_t u8_SelPos;
   uint8_t u8_CurPos;

   u8_SelPos = XMC_CAN_FIFO_GetSELMO(pt_RxBaseMo);
   u8_CurPos = XMC_CAN_FIFO_GetCurrentMO(pt_RxBaseMo);
   t_RxMo.can_mo_ptr = (CAN_MO_TypeDef *)&(CAN_MO->MO[u8_SelPos]);

   u32_MoStat = XMC_CAN_MO_GetStatus(&t_RxMo);
   if (u32_MoStat & CAN_MO_MOSTAT_MSGLST_Msk)
   {
      // On overfow we loose a complete FIFI buffer.
      // TODO: rework to just loose the messages that are really overwritten
      mu8_RxOverflow[channel] |= 1;
      XMC_CAN_MO_SetStatus(&t_RxMo, CAN_MO_MOCTR_RESMSGLST_Msk);
   }

   return (u8_SelPos == u8_CurPos) ? false : true;
}


bool can_receive(uint8_t channel, struct gs_host_frame *rx_frame)
{
   XMC_CAN_STATUS_t t_Error;
   XMC_CAN_MO_t* pt_RxBaseMo = &mat_RxMo[channel];
   XMC_CAN_MO_t t_RxMo;
   uint8_t u8_SelPos;

   u8_SelPos = XMC_CAN_FIFO_GetIncSELMO(pt_RxBaseMo);
   t_RxMo.can_mo_ptr = (CAN_MO_TypeDef *)&(CAN_MO->MO[u8_SelPos]);

   t_Error = XMC_CAN_MO_Receive(&t_RxMo);
   if (t_Error == XMC_CAN_STATUS_SUCCESS)
   {
      rx_frame->can_id = t_RxMo.can_identifier;
      rx_frame->can_id |= (t_RxMo.can_id_mode == XMC_CAN_FRAME_TYPE_EXTENDED_29BITS) ? CAN_EFF_FLAG : 0;
      rx_frame->can_dlc = t_RxMo.can_data_length;
      memcpy(rx_frame->data, t_RxMo.can_data_byte, 8);

      // TODO: RTR frames
      return true;
   }
   else
   {
      return false;
   }
}


bool can_send(uint8_t channel, struct gs_host_frame *frame)
{
   XMC_CAN_MO_t* pt_TxMo = &mat_TxMo[channel];

   // TODO: RTR frames

   pt_TxMo->can_data_length = frame->can_dlc;
   memcpy(pt_TxMo->can_data_byte, frame->data, 8);
   XMC_CAN_MO_UpdateData(pt_TxMo);

   if (frame->can_id & CAN_EFF_FLAG)
      XMC_CAN_MO_SetExtendedID(pt_TxMo);
   else
      XMC_CAN_MO_SetStandardID(pt_TxMo);
   XMC_CAN_MO_SetIdentifier(pt_TxMo, frame->can_id);

   return (XMC_CAN_MO_Transmit(pt_TxMo) == XMC_CAN_STATUS_SUCCESS) ? true : false;
}


uint32_t can_get_error_status(uint8_t channel)
{
   XMC_CAN_NODE_t* pt_Node = mapt_CanNode[channel];
   return XMC_CAN_NODE_GetStatus(pt_Node);
}


bool can_parse_error_status(uint32_t err, struct gs_host_frame *frame)
{
   frame->echo_id = 0xFFFFFFFF;
   frame->can_id  = CAN_ERR_FLAG | CAN_ERR_CRTL;
   frame->can_dlc = CAN_ERR_DLC;
   frame->data[0] = CAN_ERR_LOSTARB_UNSPEC;
   frame->data[1] = CAN_ERR_CRTL_UNSPEC;
   frame->data[2] = CAN_ERR_PROT_UNSPEC;
   frame->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
   frame->data[4] = CAN_ERR_TRX_UNSPEC;
   frame->data[5] = 0;
   frame->data[6] = 0;
   frame->data[7] = 0;

   if ((err & CAN_NODE_NSR_BOFF_Msk) != 0) {
      frame->can_id |= CAN_ERR_BUSOFF;
   }

   // No "error passiv" flag in NSR
   //if (err & ) {
   //   frame->data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
   //} else
   if (err & CAN_NODE_NSR_EWRN_Msk) {
      frame->data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
   }

   uint8_t lec = (uint8_t)((err & CAN_NODE_NSR_LEC_Msk) >> CAN_NODE_NSR_LEC_Pos);
   if (lec!=0) { /* protocol error */
      switch (lec) {
         case 0x01: /* stuff error */
            frame->can_id |= CAN_ERR_PROT;
            frame->data[2] |= CAN_ERR_PROT_STUFF;
            break;
         case 0x02: /* form error */
            frame->can_id |= CAN_ERR_PROT;
            frame->data[2] |= CAN_ERR_PROT_FORM;
            break;
         case 0x03: /* ack error */
            frame->can_id |= CAN_ERR_ACK;
            break;
         case 0x04: /* bit recessive error */
            frame->can_id |= CAN_ERR_PROT;
            frame->data[2] |= CAN_ERR_PROT_BIT1;
            break;
         case 0x05: /* bit dominant error */
            frame->can_id |= CAN_ERR_PROT;
            frame->data[2] |= CAN_ERR_PROT_BIT0;
            break;
         case 0x06: /* CRC error */
            frame->can_id |= CAN_ERR_PROT;
            frame->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
            break;
         default:
            break;
      }
   }

   return true;
}
