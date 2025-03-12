/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025, Ennebi Elettronica (https://ennebielettronica.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of twinhe Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include <USBFS.h>
#include <USBFS_Dp.h>

#include "tusb_option.h"

#if CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_PSOC5LP

#include "device/dcd.h"

#define TU_LOG_PORT(...)   TU_LOG(CFG_TUD_LOG_LEVEL, __VA_ARGS__)

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

typedef struct {
  uint16_t offset;
  uint16_t max_packet_size;
  tusb_dir_t dir;
} xfer_ep_info_t;

typedef struct {
  uint8_t *buffer;
  tu_fifo_t *ff;
  uint16_t total_len;
  uint16_t queued_len;
  xfer_ep_info_t *ep_info;
} xfer_ctl_t;

static volatile xfer_ctl_t xfer_ctl[CFG_TUD_ENDPPOINT_MAX];
static xfer_ep_info_t xfer_ep[CFG_TUD_ENDPPOINT_MAX];
static uint8_t xfer_ep_alloc = 0;
static uint8_t set_dev_addr = 0;

//--------------------------------------------------------------------+
// Interrupt prototypes
//--------------------------------------------------------------------+

CY_ISR_PROTO(USBFS_EP_0_ISR);
#ifdef CFG_TUD_EP1_ACTIVE
  CY_ISR_PROTO(USBFS_EP_1_ISR);
#endif
#ifdef CFG_TUD_EP2_ACTIVE
  CY_ISR_PROTO(USBFS_EP_2_ISR);
#endif
#ifdef CFG_TUD_EP3_ACTIVE
  CY_ISR_PROTO(USBFS_EP_3_ISR);
#endif
#ifdef CFG_TUD_EP4_ACTIVE
  CY_ISR_PROTO(USBFS_EP_4_ISR);
#endif
#ifdef CFG_TUD_EP5_ACTIVE
  CY_ISR_PROTO(USBFS_EP_5_ISR);
#endif
#ifdef CFG_TUD_EP6_ACTIVE
  CY_ISR_PROTO(USBFS_EP_6_ISR);
#endif
#ifdef CFG_TUD_EP7_ACTIVE
  CY_ISR_PROTO(USBFS_EP_7_ISR);
#endif
#ifdef CFG_TUD_EP8_ACTIVE
  CY_ISR_PROTO(USBFS_EP_8_ISR);
#endif

CY_ISR_PROTO(USBFS_BUS_RESET_ISR);
CY_ISR_PROTO(USBFS_DP_ISR);

//--------------------------------------------------------------------+
// Internal functions
//--------------------------------------------------------------------+
bool edpt0_xfer(tusb_dir_t dir)
{
  uint8_t len;
  volatile xfer_ctl_t *xfer = &xfer_ctl[0];

  if (dir == TUSB_DIR_IN)
  {
    len = xfer->total_len - xfer->queued_len;
    // Endpoint size check
    if (len > 8)
      len = 8;

    if (xfer->buffer != NULL)
    {
      for (uint16_t i = 0; i < len; i++)
      {
        USBFS_EP0_DR_BASE.epData[i] = xfer->buffer[i];
      }
      xfer->queued_len += len;
    }
    USBFS_EP0_CNT_REG = (USBFS_EP0_CNT_REG & ~USBFS_EPX_CNT0_MASK) | len;
    // Update the data toggle and count, then set mode
    USBFS_EP0_CNT_REG ^= USBFS_EP0_CNT_DATA_TOGGLE;
    USBFS_EP0_CR_REG = (USBFS_EP0_CR_REG & ~USBFS_MODE_MASK) | USBFS_MODE_ACK_IN_STATUS_OUT;
  }
  else /* if (dir == TUSB_DIR_OUT) */
  {
    // Reset data toggle and mode
    USBFS_EP0_CNT_REG = USBFS_EP0_CNT_DATA_TOGGLE;
    USBFS_EP0_CR_REG = (USBFS_EP0_CR_REG & ~USBFS_MODE_MASK) | USBFS_MODE_ACK_OUT_STATUS_IN;
  }
  return true;
}

bool edpt_xfer(uint8_t epn)
{
  uint16_t len;
  volatile xfer_ctl_t *xfer = &xfer_ctl[epn];
  tusb_dir_t dir = xfer->ep_info->dir;
  volatile USBFS_sie_ep_struct *sie_ep = &USBFS_SIE_EP_BASE.sieEp[epn];
  volatile USBFS_arb_ep_struct *arb_ep = &USBFS_ARB_EP_BASE.arbEp[epn];
  // Read control register. Altough documented for EP0 only, this seems to be necessary
  (void) sie_ep->epCr0;

  // Write WA register based on required memory allocation
  arb_ep->rwWa = LO8(xfer->ep_info->offset);
  arb_ep->rwWaMsb = HI8(xfer->ep_info->offset);

  if (dir == TUSB_DIR_IN)
  {
    len = xfer->total_len - xfer->queued_len;
    if (len > xfer->ep_info->max_packet_size)
      len = xfer->ep_info->max_packet_size;

    // Write packet size to Byte Count register (toggle data_toggle bit)
    sie_ep->epCnt0 = (sie_ep->epCnt0 & USBFS_EP0_CNT_DATA_TOGGLE) | ( HI8(len) & 0x07);
    sie_ep->epCnt1 = LO8(len);

    if (xfer->buffer != NULL)
    {
      for (uint16_t i = 0; i < len; i++)
      {
        arb_ep->rwDr = xfer->buffer[xfer->queued_len];
        xfer->queued_len++;
      }
    }

    arb_ep->rwRa = LO8(xfer->ep_info->offset);
    arb_ep->rwRaMsb = HI8(xfer->ep_info->offset);

    // Set mode valut in CR0 register
    // TODO supporti isochronous EPs
    sie_ep->epCr0 = /* ep_is_iso() ? USBFS_MODE_ISO_IN : */USBFS_MODE_ACK_IN;
  }
  else /* if (dir == TUSB_DIR_OUT) */
  {
    // Write maximum packet size to byte count register
    sie_ep->epCnt0 = (sie_ep->epCnt0 & 0xF8) | ( HI8(xfer->total_len) & 0x07);
    sie_ep->epCnt1 = LO8(xfer->total_len);

    // Set mode value in CR0 register
    // TODO supporti isochronous EPs
    sie_ep->epCr0 = /* ep_is_iso() ? USBFS_MODE_ISO_OUT : */USBFS_MODE_ACK_OUT;
  }
  return true;
}

//--------------------------------------------------------------------+
// ISR routines
//--------------------------------------------------------------------+
void edpt0_isr(void)
{
  uint8_t len;
  volatile xfer_ctl_t *xfer = &xfer_ctl[0];

  // Read CR0 register to clear SIE lock.
  uint8_t cr = USBFS_EP0_CR_REG;
  uint8_t cnt = USBFS_EP0_CNT_REG;
  bool valid = (cnt & 0x40) != 0;
  len = cnt & USBFS_EPX_CNT0_MASK;

  // At the end of an ACK transaction
  if (cr & USBFS_MODE_ACKD)
  {
    // Setup stage from host sets NAK_IN_OUT
    if (cr & USBFS_MODE_SETUP_RCVD && ((cr & USBFS_MODE_MASK) == USBFS_MODE_NAK_IN_OUT))
    {
      dcd_event_setup_received(BOARD_TUD_RHPORT, (const uint8_t*)USBFS_EP0_DR_BASE.epData, true);
    }

    // Data sent to host
    else if (cr & USBFS_MODE_IN_RCVD)
    {
      if (!valid)
      {
        dcd_event_xfer_complete(BOARD_TUD_RHPORT, tu_edpt_addr(0, TUSB_DIR_IN), 0, XFER_RESULT_FAILED, true);
      }
      else if (xfer->queued_len < xfer->total_len)
      {
        edpt0_xfer(TUSB_DIR_IN);
      }
      else
      {
        dcd_event_xfer_complete(BOARD_TUD_RHPORT, tu_edpt_addr(0, TUSB_DIR_IN), xfer->queued_len, XFER_RESULT_SUCCESS, true);
      }
    }

    // Data received from host
    else if (cr & USBFS_MODE_OUT_RCVD)
    {
      if (!valid)
      {
        dcd_event_xfer_complete(BOARD_TUD_RHPORT, tu_edpt_addr(0, TUSB_DIR_OUT), 0, XFER_RESULT_FAILED, true);
      }
      else
      {
        // Subtract CRC from len
        len -= USBFS_EPX_CNTX_CRC_COUNT;

        if (xfer->buffer != NULL)
        {
          for (uint8_t i = 0; i < len; i++)
          {
            xfer->buffer[i] = USBFS_EP0_DR_BASE.epData[i];
          }
        }

        // Send confirmation to the stack
        dcd_event_xfer_complete(BOARD_TUD_RHPORT, tu_edpt_addr(0, TUSB_DIR_OUT), len, XFER_RESULT_SUCCESS, true);
      }
    }
    // Try to reset CR status by writing to it
    // New transmission/mode will be enabled by the stack
    cr = USBFS_EP0_CR_REG;
    USBFS_EP0_CR_REG = cr;
  }
}

// Perform actions after and USB EP interrupt
// IN EP: host has read data and/or endpoint buffer can be reloaded
// OUT EP: data ready to be read from EP buffer
void edpt_isr(uint8_t epn)
{
  uint16_t len;
  volatile xfer_ctl_t *xfer = &xfer_ctl[epn];
  tusb_dir_t dir = xfer->ep_info->dir;
  volatile USBFS_sie_ep_struct *sie_ep = &USBFS_SIE_EP_BASE.sieEp[epn];
  volatile USBFS_arb_ep_struct *arb_ep = &USBFS_ARB_EP_BASE.arbEp[epn];
  bool error_in_txn = (sie_ep->epCr0 & 0x40) != 0;
  TU_ATTR_UNUSED bool acked_txn = (sie_ep->epCr0 & 0x10) != 0;

  // Reset error flags
  sie_ep->epCr0 |= 0x50;
  // Toggle for next transaction
  sie_ep->epCnt0 ^= USBFS_EPX_CNT_DATA_TOGGLE;

  // Obtain transaction data length. Subtract two CRC bytes
  len  = sie_ep->epCnt0 & USBFS_EPX_CNT0_MASK;
  len <<= 8;
  len |= sie_ep->epCnt1;

  if (dir == TUSB_DIR_IN)
  {
    // Verify error flags
    if (error_in_txn)
    {
      dcd_event_xfer_complete(BOARD_TUD_RHPORT, tu_edpt_addr(epn, dir), 0, XFER_RESULT_FAILED, true);
    }
    else if (xfer->queued_len < xfer->total_len)
    {
      edpt_xfer(epn);
    }
    else
    {
      dcd_event_xfer_complete(BOARD_TUD_RHPORT, tu_edpt_addr(epn, dir), xfer->queued_len, XFER_RESULT_SUCCESS, true);
    }
  }
  else /* if (dir == TUSB_DIR_OUT) */
  {
    // Verify data_valid
    if (error_in_txn)
    {
      dcd_event_xfer_complete(BOARD_TUD_RHPORT, tu_edpt_addr(epn, dir), 0, XFER_RESULT_FAILED, true);
    }
    else
    {
      // Write the RA value same as initial WA
      arb_ep->rwRa = LO8(xfer->ep_info->offset);
      arb_ep->rwRaMsb = HI8(xfer->ep_info->offset);

      // Subtract CRC from len
      len -= USBFS_EPX_CNTX_CRC_COUNT;

      // Read data
      if (xfer->buffer != NULL)
      {
        for (uint16_t i = 0; i < len; i++)
        {
          xfer->buffer[i] = arb_ep->rwDr;
        }
      }

      // Send confirmation to the stack
      dcd_event_xfer_complete(BOARD_TUD_RHPORT, tu_edpt_addr(epn, dir), len, XFER_RESULT_SUCCESS, true);
    }
  }
}

//--------------------------------------------------------------------+
// Device API
//--------------------------------------------------------------------+

// Initialize controller to device mode
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init)
{
  (void) rhport;
  (void) rh_init;

  // Initialization block follows USBFS_Init()

  /* Enable USB block. */
  USBFS_PM_ACT_CFG_REG |= USBFS_PM_ACT_EN_FSUSB;
  /* Enable USB block for Standby Power Mode. */
  USBFS_PM_STBY_CFG_REG |= USBFS_PM_STBY_EN_FSUSB;

  /* Enable core clock. */
  USBFS_USB_CLK_EN_REG = USBFS_USB_CLK_ENABLE;
  USBFS_CR1_REG = USBFS_CR1_ENABLE_LOCK;

  /* ENABLING USBIO PADS IN USB MODE FROM I/O MODE. */
  /* Ensure USB transmit enable is low (USB_USBIO_CR0.ten). - Manual Transmission - Disabled. */
  USBFS_USBIO_CR0_REG &= (uint8) ~USBFS_USBIO_CR0_TEN;
  CyDelayUs(USBFS_WAIT_REG_STABILITY_50NS);  /* ~50ns delay. */
  /* Disable USBIO by asserting PM.USB_CR0.fsusbio_pd_n(Inverted.
  *  high. These bits will be set low by the power manager out-of-reset.
  *  Also confirm USBIO pull-up is disabled.
  */
  USBFS_PM_USB_CR0_REG &= (uint8) ~(USBFS_PM_USB_CR0_PD_N |
                                               USBFS_PM_USB_CR0_PD_PULLUP_N);

  /* Select IOMODE to USB mode. */
  USBFS_USBIO_CR1_REG &= (uint8) ~USBFS_USBIO_CR1_IOMODE;

  /* Enable USBIO reference by setting PM.USB_CR0.fsusbio_ref_en. */
  USBFS_PM_USB_CR0_REG |= USBFS_PM_USB_CR0_REF_EN;
  /* Reference is available for 1us after regulator is enabled. */
  CyDelayUs(USBFS_WAIT_REG_STABILITY_1US);
  /* OR 40us after power is restored. */
  CyDelayUs(USBFS_WAIT_VREF_RESTORE);
  /* Ensure single-ended disable bits are low (PRT15.INP_DIS[7:6])(input receiver enabled). */
  USBFS_DM_INP_DIS_REG &= (uint8) ~USBFS_DM_MASK;
  USBFS_DP_INP_DIS_REG &= (uint8) ~USBFS_DP_MASK;

  /* Enable USBIO. */
  USBFS_PM_USB_CR0_REG |= USBFS_PM_USB_CR0_PD_N;
  CyDelayUs(USBFS_WAIT_PD_PULLUP_N_ENABLE);
  /* Set USBIO pull-up enable. */
  USBFS_PM_USB_CR0_REG |= USBFS_PM_USB_CR0_PD_PULLUP_N;

  /* Reset Arbiter Write Address register for endpoint 1. */
  CY_SET_REG8(USBFS_ARB_RW1_WA_PTR,     0u);
  CY_SET_REG8(USBFS_ARB_RW1_WA_MSB_PTR, 0u);

#if (USBFS_EP_MANAGEMENT_DMA)
  /* Initialize transfer descriptor. This will be used to detect DMA state - initialized or not. */
  for (i = 0u; i < USBFS_MAX_EP; ++i)
  {
    USBFS_DmaTd[i] = DMA_INVALID_TD;

    #if (USBFS_EP_MANAGEMENT_DMA_AUTO && (USBFS_EP_DMA_AUTO_OPT == 0u))
        USBFS_DmaNextTd[i] = DMA_INVALID_TD;
    #endif /* (USBFS_EP_MANAGEMENT_DMA_AUTO && (USBFS_EP_DMA_AUTO_OPT == 0u)) */
  }
#endif /* (USBFS_EP_MANAGEMENT_DMA) */

  /* Set bus reset interrupt. */
  CyIntSetPriority(USBFS_BUS_RESET_VECT_NUM, USBFS_BUS_RESET_PRIOR);
  (void) CyIntSetVector(USBFS_BUS_RESET_VECT_NUM,   &USBFS_BUS_RESET_ISR);

  /* Set Control Endpoint Interrupt. */
  CyIntSetPriority(USBFS_EP_0_VECT_NUM, USBFS_EP_0_PRIOR);
  (void) CyIntSetVector(USBFS_EP_0_VECT_NUM,   &USBFS_EP_0_ISR);

  /* Endpoint interrupts */
#ifdef CFG_TUD_EP1_ACTIVE
  CyIntSetPriority     (USBFS_EP_1_VECT_NUM,  USBFS_EP_1_PRIOR);
  (void) CyIntSetVector(USBFS_EP_1_VECT_NUM,  &USBFS_EP_1_ISR);
#endif
#ifdef CFG_TUD_EP2_ACTIVE
  CyIntSetPriority     (USBFS_EP_2_VECT_NUM,  USBFS_EP_2_PRIOR);
  (void) CyIntSetVector(USBFS_EP_2_VECT_NUM,  &USBFS_EP_2_ISR);
#endif
#ifdef CFG_TUD_EP3_ACTIVE
  CyIntSetPriority     (USBFS_EP_3_VECT_NUM,  USBFS_EP_3_PRIOR);
  (void) CyIntSetVector(USBFS_EP_3_VECT_NUM,  &USBFS_EP_3_ISR);
#endif
#ifdef CFG_TUD_EP4_ACTIVE
  CyIntSetPriority     (USBFS_EP_4_VECT_NUM,  USBFS_EP_4_PRIOR);
  (void) CyIntSetVector(USBFS_EP_4_VECT_NUM,  &USBFS_EP_4_ISR);
#endif
#ifdef CFG_TUD_EP5_ACTIVE
  CyIntSetPriority     (USBFS_EP_5_VECT_NUM,  USBFS_EP_5_PRIOR);
  (void) CyIntSetVector(USBFS_EP_5_VECT_NUM,  &USBFS_EP_5_ISR);
#endif
#ifdef CFG_TUD_EP6_ACTIVE
  CyIntSetPriority     (USBFS_EP_6_VECT_NUM,  USBFS_EP_6_PRIOR);
  (void) CyIntSetVector(USBFS_EP_6_VECT_NUM,  &USBFS_EP_6_ISR);
#endif
#ifdef CFG_TUD_EP7_ACTIVE
  CyIntSetPriority     (USBFS_EP_7_VECT_NUM,  USBFS_EP_7_PRIOR);
  (void) CyIntSetVector(USBFS_EP_7_VECT_NUM,  &USBFS_EP_7_ISR);
#endif
#ifdef CFG_TUD_EP8_ACTIVE
  CyIntSetPriority     (USBFS_EP_8_VECT_NUM,  USBFS_EP_8_PRIOR);
  (void) CyIntSetVector(USBFS_EP_8_VECT_NUM,  &USBFS_EP_8_ISR);
#endif

  /* Common: Configure GPIO interrupt for wakeup. */
  CyIntSetPriority     (USBFS_DP_INTC_VECT_NUM,  USBFS_DP_INTC_PRIORITY);
  (void) CyIntSetVector(USBFS_DP_INTC_VECT_NUM, &USBFS_DP_ISR);

#if (USBFS_EP_MANAGEMENT_DMA)
  /* Configure Arbiter for Manual or Auto DMA operation and clear configuration completion. */
  USBFS_ARB_CFG_REG = USBFS_DEFAULT_ARB_CFG;

  #if (CY_PSOC4)
    /* Enable DMA operation. */
    CyDmaEnable();

    #if (USBFS_EP_MANAGEMENT_DMA_AUTO)
      /* Change DMA priority to be highest. */
      CyIntSetPriority(CYDMA_INTR_NUMBER, USBFS_DMA_AUTO_INTR_PRIO);
    #endif /* (USBFS_EP_MANAGEMENT_DMA_AUTO) */
  #endif /* (CY_PSOC4) */

  #if (USBFS_EP_MANAGEMENT_DMA_AUTO)
    #if (CY_PSOC4)
      /* Enable DMA interrupt to handle DMA management. */
      CyIntEnable(CYDMA_INTR_NUMBER);
    #else
      #if (USBFS_EP_DMA_AUTO_OPT == 0u)
        /* Initialize interrupts which handle verification of successful DMA transaction. */
        USBFS_EP_DMA_Done_isr_StartEx(&USBFS_EP_DMA_DONE_ISR);
        USBFS_EP17_DMA_Done_SR_InterruptEnable();
        USBFS_EP8_DMA_Done_SR_InterruptEnable();
      #endif /* (USBFS_EP_DMA_AUTO_OPT == 0u) */
    #endif /* (CY_PSOC4) */
  #endif /* (USBFS_EP_MANAGEMENT_DMA_AUTO) */
#endif /* (USBFS_EP_MANAGEMENT_DMA) */

  /* Check DWR settings of USB power supply. */
  #if (USBFS_VDDD_MV < USBFS_3500MV)
    /* Disable regulator for 3V operation. */
    USBFS_CR1_REG &= (uint8) ~USBFS_CR1_REG_ENABLE;
  #else
    /* Enable regulator for 5V operation. */
    USBFS_CR1_REG |= (uint8)  USBFS_CR1_REG_ENABLE;
  #endif /* (USBFS_VDDD_MV < USBFS_3500MV) */

  /* Set EP0.CR: ACK Setup, STALL IN/OUT. */
  USBFS_EP0_CR_REG = USBFS_MODE_STALL_IN_OUT;

  /* Enable device to respond to USB traffic with address 0. */
  USBFS_CR0_REG = USBFS_DEFUALT_CR0;
  CyDelayCycles(USBFS_WAIT_CR0_REG_STABILITY);

  /* Enable D+ pull-up and keep USB control on IO. */
  USBFS_USBIO_CR1_REG = USBFS_USBIO_CR1_USBPUEN;

  return true;
}

// Enable device interrupt
void dcd_int_enable (uint8_t rhport)
{
  (void) rhport;

  CyIntEnable(USBFS_BUS_RESET_VECT_NUM);
  CyIntEnable(USBFS_EP_0_VECT_NUM);
#ifdef CFG_TUD_EP1_ACTIVE
  CyIntEnable(USBFS_EP_1_VECT_NUM);
#endif
#ifdef CFG_TUD_EP2_ACTIVE
  CyIntEnable(USBFS_EP_2_VECT_NUM);
#endif
#ifdef CFG_TUD_EP3_ACTIVE
  CyIntEnable(USBFS_EP_3_VECT_NUM);
#endif
#ifdef CFG_TUD_EP4_ACTIVE
  CyIntEnable(USBFS_EP_4_VECT_NUM);
#endif
#ifdef CFG_TUD_EP5_ACTIVE
  CyIntEnable(USBFS_EP_5_VECT_NUM);
#endif
#ifdef CFG_TUD_EP6_ACTIVE
  CyIntEnable(USBFS_EP_6_VECT_NUM);
#endif
#ifdef CFG_TUD_EP7_ACTIVE
  CyIntEnable(USBFS_EP_7_VECT_NUM);
#endif
#ifdef CFG_TUD_EP8_ACTIVE
  CyIntEnable(USBFS_EP_8_VECT_NUM);
#endif
}

// Disable device interrupt
void dcd_int_disable (uint8_t rhport)
{
  (void) rhport;

  CyIntDisable(USBFS_BUS_RESET_VECT_NUM);
  CyIntDisable(USBFS_EP_0_VECT_NUM);
#ifdef CFG_TUD_EP1_ACTIVE
  CyIntDisable(USBFS_EP_1_VECT_NUM);
#endif
#ifdef CFG_TUD_EP2_ACTIVE
  CyIntDisable(USBFS_EP_2_VECT_NUM);
#endif
#ifdef CFG_TUD_EP3_ACTIVE
  CyIntDisable(USBFS_EP_3_VECT_NUM);
#endif
#ifdef CFG_TUD_EP4_ACTIVE
  CyIntDisable(USBFS_EP_4_VECT_NUM);
#endif
#ifdef CFG_TUD_EP5_ACTIVE
  CyIntDisable(USBFS_EP_5_VECT_NUM);
#endif
#ifdef CFG_TUD_EP6_ACTIVE
  CyIntDisable(USBFS_EP_6_VECT_NUM);
#endif
#ifdef CFG_TUD_EP7_ACTIVE
  CyIntDisable(USBFS_EP_7_VECT_NUM);
#endif
#ifdef CFG_TUD_EP8_ACTIVE
  CyIntDisable(USBFS_EP_8_VECT_NUM);
#endif

  /* Clear active mode Dp interrupt source history. */
  (void) USBFS_Dp_ClearInterrupt();
  CyIntClearPending(USBFS_DP_INTC_VECT_NUM);
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
  // Managed by the component
  (void) rhport;
  (void) dev_addr;

  // DCD can only set address after status for this request is complete.
  // do it at dcd_edpt0_status_complete()
  set_dev_addr = dev_addr;

  // Respond with status
  dcd_edpt_xfer(rhport, TUSB_DIR_IN_MASK | 0x00, NULL, 0);
}

void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const * request)
{
  (void)rhport;
  (void)request;

  if (set_dev_addr != (USBFS_CR0_REG & USBFS_CR0_DEVICE_ADDRESS_MASK))
  {
    USBFS_CR0_REG = set_dev_addr | USBFS_CR0_ENABLE;
  }
}

// Wake up host
void dcd_remote_wakeup (uint8_t rhport)
{
  // Managed by the component, unused
  (void) rhport;
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport)
{
  (void) rhport;

  USBFS_USBIO_CR1_REG |= USBFS_USBIO_CR1_USBPUEN;
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;

  USBFS_USBIO_CR1_REG &= ~USBFS_USBIO_CR1_USBPUEN;
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * ep_desc)
{
  (void) rhport;

  if (xfer_ep_alloc >= CFG_TUD_ENDPPOINT_MAX)
    return false;

  // Hardware limitation
  TU_ASSERT(ep_desc->wMaxPacketSize <= 64);

  const unsigned epn = tu_edpt_number(ep_desc->bEndpointAddress);
  const unsigned dir = tu_edpt_dir(ep_desc->bEndpointAddress);

  // Initialize offset in SRAM according to maximum packet size (depending on EP size)
  TU_ATTR_UNUSED xfer_ep_info_t *ep = &xfer_ep[xfer_ep_alloc];
  if (xfer_ep_alloc == 0)
    xfer_ep[xfer_ep_alloc].offset = 0;
  else
    xfer_ep[xfer_ep_alloc].offset = xfer_ep[xfer_ep_alloc - 1].offset + xfer_ep[xfer_ep_alloc - 1].max_packet_size;
  xfer_ep[xfer_ep_alloc].max_packet_size = ep_desc->wMaxPacketSize;
  xfer_ep[xfer_ep_alloc].dir = dir;
  xfer_ctl[epn].ep_info = &xfer_ep[xfer_ep_alloc];
  xfer_ep_alloc++;

  if (epn > 0)
  {
    // Enable interrupt
    USBFS_SIE_EP_INT_EN_REG |= 1 << (epn - 1);
    // Set EP to NAK
    USBFS_SIE_EP_BASE.sieEp[epn].epCr0 = (dir == TUSB_DIR_IN ? USBFS_MODE_NAK_IN : USBFS_MODE_NAK_OUT);
    USBFS_SIE_EP_BASE.sieEp[epn].epCnt0 &= ~USBFS_EPX_CNT_DATA_TOGGLE;
  }

  return true;
}

#ifdef TUP_DCD_EDPT_ISO_ALLOC

// Allocate packet buffer used by ISO endpoints
// Some MCU need manual packet buffer allocation, we allocate the largest size to avoid clustering
bool dcd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t largest_packet_size) {
  (void) rhport;
  (void) ep_addr;
  (void) largest_packet_size;
  return false;
}

// Configure and enable an ISO endpoint according to descriptor
bool dcd_edpt_iso_activate(uint8_t rhport, tusb_desc_endpoint_t const * desc_ep) {
  (void) rhport;
  (void) desc_ep;
  return false;
}

#endif

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;

  // Clear the list of allocated endpoints
  xfer_ep_alloc = 0;
  USBFS_SIE_EP_INT_EN_REG = 0;
  // Clear current device address
  set_dev_addr = 0;
  USBFS_CR0_REG =  set_dev_addr | USBFS_CR0_ENABLE;
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
  (void) rhport;
  const unsigned epn = tu_edpt_number(ep_addr);
  const unsigned dir = tu_edpt_dir(ep_addr);

  volatile xfer_ctl_t *xfer = &xfer_ctl[epn];
  xfer->buffer = buffer;
  xfer->total_len = total_bytes;
  xfer->queued_len = 0;
  xfer->ff = NULL;

  if (epn == 0)
  {
    return edpt0_xfer(dir);
  }
  else
  {
    TU_ASSERT(dir == xfer->ep_info->dir);
    return edpt_xfer(epn);
  }
}

// Submit a transfer where is managed by FIFO, When complete dcd_event_xfer_complete() is invoked to notify the stack - optional, however, must be listed in usbd.c
bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t *ff, uint16_t total_bytes)
{
  (void) rhport;

const unsigned epn = tu_edpt_number(ep_addr);
  const unsigned dir = tu_edpt_dir(ep_addr);

  volatile xfer_ctl_t *xfer = &xfer_ctl[epn];
  xfer->buffer = NULL;
  xfer->total_len = total_bytes;
  xfer->queued_len = 0;
  xfer->ff = ff;

  TU_ASSERT(dir == xfer->ep_info->dir);

  return edpt_xfer(epn);
}

// Stall endpoint
void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  const unsigned epn = tu_edpt_number(ep_addr);

  if (epn == 0)
    USBFS_EP0_CR_REG = USBFS_MODE_STALL_IN_OUT;
  else
    USBFS_SIE_EP_BASE.sieEp[epn].epCr0 = USBFS_MODE_STALL_DATA_EP;
}

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  const unsigned epn = tu_edpt_number(ep_addr);

  if (epn == 0)
  {
    USBFS_EP0_CR_REG = USBFS_MODE_ACK_OUT_STATUS_IN;
    USBFS_EP0_CNT_REG &= ~USBFS_EP0_CNT_DATA_TOGGLE;
  }
  else
  {
    USBFS_SIE_EP_BASE.sieEp[epn].epCr0 = USBFS_MODE_NAK_IN;
    USBFS_SIE_EP_BASE.sieEp[epn].epCnt0 &= ~USBFS_EPX_CNT_DATA_TOGGLE;
  }
}

//--------------------------------------------------------------------+
// Interrupts
//--------------------------------------------------------------------+

CY_ISR(USBFS_EP_0_ISR)
{
  edpt0_isr();
  USBFS_ClearSieInterruptSource(USBFS_INTR_SIE_EP0_INTR);
}

#ifdef CFG_TUD_EP1_ACTIVE
CY_ISR(USBFS_EP_1_ISR)
{
  USBFS_ClearSieEpInterruptSource(USBFS_SIE_INT_EP1_INTR);
  ///@todo CY firmware reads CR0 to clear SIE lock, but this seems necessary for EP0 only
  edpt_isr(1);
}
#endif

#ifdef CFG_TUD_EP2_ACTIVE
CY_ISR(USBFS_EP_2_ISR)
{
  USBFS_ClearSieEpInterruptSource(USBFS_SIE_INT_EP2_INTR);
  ///@todo CY firmware reads CR0 to clear SIE lock, but this seems necessary for EP0 only
  edpt_isr(2);
}
#endif

#ifdef CFG_TUD_EP3_ACTIVE
CY_ISR(USBFS_EP_3_ISR)
{
  USBFS_ClearSieEpInterruptSource(USBFS_SIE_INT_EP3_INTR);
  edpt_isr(3);
}
#endif

#ifdef CFG_TUD_EP4_ACTIVE
CY_ISR(USBFS_EP_4_ISR)
{
  USBFS_ClearSieEpInterruptSource(USBFS_SIE_INT_EP4_INTR);
  edpt_isr(4);
}
#endif

#ifdef CFG_TUD_EP5_ACTIVE
CY_ISR(USBFS_EP_5_ISR)
{
  USBFS_ClearSieEpInterruptSource(USBFS_SIE_INT_EP5_INTR);
  edpt_isr(5);
}
#endif

#ifdef CFG_TUD_EP6_ACTIVE
CY_ISR(USBFS_EP_6_ISR)
{
  USBFS_ClearSieEpInterruptSource(USBFS_SIE_INT_EP6_INTR);
  edpt_isr(6);
}
#endif

#ifdef CFG_TUD_EP7_ACTIVE
CY_ISR(USBFS_EP_7_ISR)
{
  // OUT Endpoint, HOST -> DEVICE
  USBFS_ClearSieEpInterruptSource(USBFS_SIE_INT_EP7_INTR);
  edpt_isr(7);
}
#endif

#ifdef CFG_TUD_EP8_ACTIVE
CY_ISR(USBFS_EP_8_ISR)
{
  // OUT Endpoint, HOST -> DEVICE
  USBFS_ClearSieEpInterruptSource(USBFS_SIE_INT_EP8_INTR);
  edpt_isr(8);
}
#endif

CY_ISR(USBFS_BUS_RESET_ISR)
{
  USBFS_ClearSieInterruptSource(USBFS_INTR_SIE_BUS_RESET_INTR);
  // Restore configuration mode. Accept SETUP packet on default address
  USBFS_EP0_CR_REG = USBFS_MODE_STALL_IN_OUT;
  USBFS_CR0_REG = USBFS_DEFUALT_CR0;
  dcd_event_bus_reset(BOARD_TUD_RHPORT, TUSB_SPEED_FULL, true);
  xfer_ep_alloc = 0;
}

CY_ISR(USBFS_DP_ISR)
{
  // Clear interrupt status
  TU_ATTR_UNUSED uint8_t sts = (USBFS_Dp_INTSTAT & USBFS_Dp_MASK) >> USBFS_Dp_SHIFT;
}

#endif /* CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_PSOC5LP */
