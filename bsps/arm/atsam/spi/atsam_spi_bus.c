/* ---------------------------------------------------------------------------- */
/*                  Atmel Microcontroller Software Support                      */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) 2015, Atmel Corporation                                        */
/* Copyright (c) 2016, embedded brains GmbH                                     */
/*                                                                              */
/* All rights reserved.                                                         */
/*                                                                              */
/* Redistribution and use in source and binary forms, with or without           */
/* modification, are permitted provided that the following condition is met:    */
/*                                                                              */
/* - Redistributions of source code must retain the above copyright notice,     */
/* this list of conditions and the disclaimer below.                            */
/*                                                                              */
/* Atmel's name may not be used to endorse or promote products derived from     */
/* this software without specific prior written permission.                     */
/*                                                                              */
/* DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR   */
/* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE   */
/* DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,      */
/* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
/* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,  */
/* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    */
/* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING         */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/* ---------------------------------------------------------------------------- */

#include <bsp/atsam-clock-config.h>
#include <bsp/atsam-spi.h>
#include <bsp/iocopy.h>

#include <rtems/thread.h>

#include <dev/spi/spi.h>

#include <string.h>

#define MAX_SPI_FREQUENCY 50000000

#define DMA_NR_DESC_PER_DIR 3
#define DMA_DESC_ALLIGNMENT 4

#define DMA_BUF_RX 0
#define DMA_BUF_TX 1
#define DMA_BUF_DIRS 2

struct atsam_spi_xdma_buf {
  LinkedListDescriporView0 desc[DMA_NR_DESC_PER_DIR];
  uint8_t leadbuf[CPU_CACHE_LINE_BYTES];
  uint8_t trailbuf[CPU_CACHE_LINE_BYTES];
};

typedef struct {
  spi_bus base;
  rtems_binary_semaphore sem;
  const spi_ioc_transfer *msg_current;
  uint32_t msg_todo;
  int msg_error;
  Spi *spi_regs;
  uint32_t dma_tx_channel;
  uint32_t dma_rx_channel;
  struct atsam_spi_xdma_buf *dma_bufs;
  size_t leadbuf_rx_buffered_len;
  size_t trailbuf_rx_buffered_len;
  int transfer_in_progress;
  bool chip_select_decode;
  uint8_t spi_id;
  uint32_t peripheral_clk_per_us;
  uint32_t spi_mr;
  uint32_t spi_csr[4];
} atsam_spi_bus;

static void atsam_spi_wakeup_task(atsam_spi_bus *bus)
{
  rtems_binary_semaphore_post(&bus->sem);
}

static uint8_t atsam_calculate_dlybcs(const atsam_spi_bus *bus)
{
  uint32_t dlybcs = bus->base.delay_usecs * bus->peripheral_clk_per_us;

  if (dlybcs > 0xff) {
    dlybcs = 0xff;
  }

  return dlybcs;
}

static uint32_t atsam_calculate_scbr(uint32_t speed_hz)
{
  uint32_t scbr;

  scbr = BOARD_MCK / speed_hz;
  if (scbr > 0x0FF) {
    /* Best estimation we can offer with the hardware. */
    scbr = 0x0FF;
  }
  if (scbr == 0) {
    /* SCBR = 0 isn't allowed. */
    scbr = 1;
  }

  return scbr;
}

static void atsam_set_phase_and_polarity(uint32_t mode, uint32_t *csr)
{
  uint32_t mode_mask = mode & SPI_MODE_3;

  switch(mode_mask) {
    case SPI_MODE_0:
      *csr |= SPI_CSR_NCPHA;
      break;
    case SPI_MODE_1:
      break;
    case SPI_MODE_2:
      *csr |= SPI_CSR_NCPHA;
      *csr |= SPI_CSR_CPOL;
      break;
    case SPI_MODE_3:
      *csr |= SPI_CSR_CPOL;
      break;
  }
  *csr |= SPI_CSR_CSAAT;
}

static void atsam_configure_spi(atsam_spi_bus *bus)
{
  uint32_t scbr;
  uint32_t csr = 0;
  uint32_t mr;
  uint32_t cs = bus->base.cs;

  scbr = atsam_calculate_scbr(bus->base.speed_hz);

  mr = bus->spi_mr;

  if (bus->chip_select_decode) {
    mr |= SPI_MR_PCS(bus->base.cs);
    mr |= SPI_MR_PCSDEC;
    cs /= 4;
  } else {
    mr |= SPI_PCS(bus->base.cs);
  }

  bus->spi_regs->SPI_MR = mr;

  csr = bus->spi_csr[cs]
    | SPI_CSR_SCBR(scbr)
    | SPI_CSR_BITS(bus->base.bits_per_word - 8);

  atsam_set_phase_and_polarity(bus->base.mode, &csr);

  SPI_ConfigureNPCS(bus->spi_regs, cs, csr);
}

static void atsam_reset_spi(atsam_spi_bus *bus)
{
  bus->spi_regs->SPI_CR = SPI_CR_SPIDIS;
  bus->spi_regs->SPI_CR = SPI_CR_SWRST;
  bus->spi_regs->SPI_CR = SPI_CR_SWRST;
  bus->spi_regs->SPI_CR = SPI_CR_SPIEN;
}

static void atsam_spi_check_alignment_and_set_up_dma_descriptors(
  atsam_spi_bus *bus,
  struct atsam_spi_xdma_buf *buf,
  const uint8_t *start,
  size_t len,
  bool tx
)
{
  LinkedListDescriporView0 *curdesc = buf->desc;
  size_t misaligned_begin;
  size_t misaligned_end;
  size_t len_main;
  const uint8_t *start_main;
  const uint8_t *start_trail;

  /* Check alignments. */
  if (len < CPU_CACHE_LINE_BYTES) {
    misaligned_begin = len;
    misaligned_end = 0;
    len_main = 0;
  } else {
    misaligned_begin = ((uint32_t) start) % CPU_CACHE_LINE_BYTES;
    misaligned_end = (((uint32_t) start) + len) % CPU_CACHE_LINE_BYTES;
    len_main = len - misaligned_begin - misaligned_end;
  }
  start_main = start + misaligned_begin;
  start_trail = start_main + len_main;

  /* Store length for copying data back. */
  if (!tx) {
    bus->leadbuf_rx_buffered_len = misaligned_begin;
    bus->trailbuf_rx_buffered_len = misaligned_end;
  }

  /* Handle misalignment on begin. */
  if (misaligned_begin != 0) {
    if (tx) {
      atsam_copy_to_io(buf->leadbuf, start, misaligned_begin);
    }
    curdesc->mbr_nda = (uint32_t) (&curdesc[1]);
    curdesc->mbr_ta = (uint32_t) buf->leadbuf;
    curdesc->mbr_ubc = misaligned_begin;
  }

  /* Main part */
  if (len_main > 0) {
    curdesc->mbr_ubc |= tx ? XDMA_UBC_NSEN_UPDATED : XDMA_UBC_NDEN_UPDATED;
    curdesc->mbr_ubc |= XDMA_UBC_NVIEW_NDV0;
    curdesc->mbr_ubc |= XDMA_UBC_NDE_FETCH_EN;
    ++curdesc;

    curdesc->mbr_nda = (uint32_t) (&curdesc[1]);
    curdesc->mbr_ta = (uint32_t) start_main;
    curdesc->mbr_ubc = len_main;
    if (tx) {
      rtems_cache_flush_multiple_data_lines(start_main, len_main);
    } else {
      rtems_cache_invalidate_multiple_data_lines(start_main, len_main);
    }
  }

  /* Handle misalignment on end */
  if (misaligned_end != 0) {
    curdesc->mbr_ubc |= tx ? XDMA_UBC_NSEN_UPDATED : XDMA_UBC_NDEN_UPDATED;
    curdesc->mbr_ubc |= XDMA_UBC_NVIEW_NDV0;
    curdesc->mbr_ubc |= XDMA_UBC_NDE_FETCH_EN;
    ++curdesc;

    if (tx) {
      atsam_copy_to_io(buf->trailbuf, start_trail, misaligned_end);
    }
    curdesc->mbr_nda = 0;
    curdesc->mbr_ta = (uint32_t) buf->trailbuf;
    curdesc->mbr_ubc = misaligned_end;
    curdesc->mbr_ubc |= XDMA_UBC_NDE_FETCH_DIS;
  }
}

static void atsam_spi_copy_back_rx_after_dma_transfer(
  atsam_spi_bus *bus
)
{
  if (bus->leadbuf_rx_buffered_len != 0) {
    atsam_copy_from_io(
      bus->msg_current->rx_buf,
      bus->dma_bufs[DMA_BUF_RX].leadbuf,
      bus->leadbuf_rx_buffered_len
    );
  }
  if (bus->trailbuf_rx_buffered_len != 0) {
    atsam_copy_from_io(
      bus->msg_current->rx_buf + bus->msg_current->len -
        bus->trailbuf_rx_buffered_len,
      bus->dma_bufs[DMA_BUF_RX].trailbuf,
      bus->trailbuf_rx_buffered_len
    );
  }
}

static void atsam_spi_start_dma_transfer(
  atsam_spi_bus *bus,
  const spi_ioc_transfer *msg
)
{
  Xdmac *pXdmac = XDMAC;

  atsam_spi_check_alignment_and_set_up_dma_descriptors(
    bus,
    &bus->dma_bufs[DMA_BUF_RX],
    msg->rx_buf,
    msg->len,
    false
  );
  atsam_spi_check_alignment_and_set_up_dma_descriptors(
    bus,
    &bus->dma_bufs[DMA_BUF_TX],
    msg->tx_buf,
    msg->len,
    true
  );

  XDMAC_SetDescriptorAddr(
    pXdmac,
    bus->dma_rx_channel,
    (uint32_t) bus->dma_bufs[DMA_BUF_RX].desc,
    0
  );
  XDMAC_SetDescriptorControl(
    pXdmac,
    bus->dma_rx_channel,
    XDMAC_CNDC_NDVIEW_NDV0 |
    XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED |
    XDMAC_CNDC_NDE_DSCR_FETCH_EN
  );
  XDMAC_SetDescriptorAddr(
    pXdmac,
    bus->dma_tx_channel,
    (uint32_t) bus->dma_bufs[DMA_BUF_TX].desc,
    0
  );
  XDMAC_SetDescriptorControl(
    pXdmac,
    bus->dma_tx_channel,
    XDMAC_CNDC_NDVIEW_NDV0 |
    XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED |
    XDMAC_CNDC_NDE_DSCR_FETCH_EN
  );

  XDMAC_StartTransfer(pXdmac, bus->dma_rx_channel);
  XDMAC_StartTransfer(pXdmac, bus->dma_tx_channel);
}

static int atsam_check_configure_spi(atsam_spi_bus *bus, const spi_ioc_transfer *msg)
{
  if (
    msg->mode != bus->base.mode
      || msg->speed_hz != bus->base.speed_hz
      || msg->bits_per_word != bus->base.bits_per_word
      || msg->cs != bus->base.cs
  ) {
    if (
      (msg->bits_per_word != 8 && msg->bits_per_word != 16)
        || msg->mode > 3
        || msg->speed_hz > bus->base.max_speed_hz
    ) {
      return -EINVAL;
    }

    bus->base.mode = msg->mode;
    bus->base.speed_hz = msg->speed_hz;
    bus->base.bits_per_word = msg->bits_per_word;
    bus->base.cs = msg->cs;
    atsam_configure_spi(bus);
  }

  return 0;
}

static void atsam_spi_setup_transfer(atsam_spi_bus *bus)
{
  uint32_t msg_todo = bus->msg_todo;

  bus->transfer_in_progress = 2;

  if (msg_todo > 0) {
    const spi_ioc_transfer *msg;
    int error;

    msg = bus->msg_current;
    error = atsam_check_configure_spi(bus, msg);
    if (error == 0) {
      atsam_spi_start_dma_transfer(bus, msg);
    } else {
      bus->msg_error = error;
      atsam_spi_wakeup_task(bus);
    }
  } else {
    atsam_spi_wakeup_task(bus);
  }
}

static void atsam_spi_dma_callback(uint32_t channel, void *arg)
{
  atsam_spi_bus *bus = (atsam_spi_bus *)arg;

  --bus->transfer_in_progress;

  if (bus->transfer_in_progress == 0) {
    const spi_ioc_transfer *msg = bus->msg_current;

    if (msg->delay_usecs != bus->base.delay_usecs) {
      uint32_t mr;
      uint32_t mr_dlybcs;

      bus->base.delay_usecs = msg->delay_usecs;
      mr_dlybcs = SPI_MR_DLYBCS(atsam_calculate_dlybcs(bus));
      bus->spi_mr = mr_dlybcs | SPI_MR_MSTR | SPI_MR_MODFDIS;

      mr = bus->spi_regs->SPI_MR;
      mr &= ~SPI_MR_DLYBCS_Msk;
      mr |= mr_dlybcs;
      bus->spi_regs->SPI_MR = mr;
    }

    if (msg->cs_change) {
      bus->spi_regs->SPI_CR = SPI_CR_LASTXFER;
    }

    atsam_spi_copy_back_rx_after_dma_transfer(bus);

    bus->msg_current = msg + 1;
    --bus->msg_todo;

    atsam_spi_setup_transfer(bus);
  }
}

static int atsam_spi_transfer(
  spi_bus *base,
  const spi_ioc_transfer *msgs,
  uint32_t msg_count
)
{
  atsam_spi_bus *bus = (atsam_spi_bus *)base;

  bus->msg_current = msgs;
  bus->msg_todo = msg_count;
  bus->msg_error = 0;
  atsam_spi_setup_transfer(bus);
  rtems_binary_semaphore_wait(&bus->sem);
  return bus->msg_error;
}


static void atsam_spi_destroy(spi_bus *base)
{
  atsam_spi_bus *bus = (atsam_spi_bus *)base;
  eXdmadRC rc;

  rc = XDMAD_SetCallback(
    &XDMAD_Instance,
    bus->dma_rx_channel,
    XDMAD_DoNothingCallback,
    NULL
  );
  assert(rc == XDMAD_OK);

  rc = XDMAD_SetCallback(
    &XDMAD_Instance,
    bus->dma_tx_channel,
    XDMAD_DoNothingCallback,
    NULL
  );
  assert(rc == XDMAD_OK);

  XDMAD_FreeChannel(&XDMAD_Instance, bus->dma_rx_channel);
  XDMAD_FreeChannel(&XDMAD_Instance, bus->dma_tx_channel);

  SPI_Disable(bus->spi_regs);
  PMC_DisablePeripheral(bus->spi_id);

  rtems_cache_coherent_free(bus->dma_bufs);
  rtems_binary_semaphore_destroy(&bus->sem);
  spi_bus_destroy_and_free(&bus->base);
}

static int atsam_spi_setup(spi_bus *base)
{
  atsam_spi_bus *bus = (atsam_spi_bus *)base;

  if (
    bus->base.speed_hz > MAX_SPI_FREQUENCY
      || (bus->base.bits_per_word != 8 && bus->base.bits_per_word != 16)
  ) {
      return -EINVAL;
  }
  atsam_configure_spi(bus);
  return 0;
}

static void atsam_spi_init_xdma(atsam_spi_bus *bus)
{
  sXdmadCfg cfg;
  uint32_t xdmaInt;
  uint8_t channel;
  eXdmadRC rc;
  uint32_t xdma_cndc;

  bus->dma_bufs = rtems_cache_coherent_allocate(
    DMA_BUF_DIRS * sizeof(*(bus->dma_bufs)),
    DMA_DESC_ALLIGNMENT,
    0
  );
  assert(bus->dma_bufs != NULL);

  bus->dma_tx_channel = XDMAD_AllocateChannel(
    &XDMAD_Instance,
    XDMAD_TRANSFER_MEMORY,
    bus->spi_id
  );
  assert(bus->dma_tx_channel != XDMAD_ALLOC_FAILED);

  bus->dma_rx_channel = XDMAD_AllocateChannel(
    &XDMAD_Instance,
    bus->spi_id,
    XDMAD_TRANSFER_MEMORY
  );
  assert(bus->dma_rx_channel != XDMAD_ALLOC_FAILED);

  rc = XDMAD_SetCallback(
    &XDMAD_Instance,
    bus->dma_rx_channel,
    atsam_spi_dma_callback,
    bus
  );
  assert(rc == XDMAD_OK);

  rc = XDMAD_SetCallback(
    &XDMAD_Instance,
    bus->dma_tx_channel,
    atsam_spi_dma_callback,
    bus
  );
  assert(rc == XDMAD_OK);

  rc = XDMAD_PrepareChannel(&XDMAD_Instance, bus->dma_rx_channel);
  assert(rc == XDMAD_OK);

  rc = XDMAD_PrepareChannel(&XDMAD_Instance, bus->dma_tx_channel);
  assert(rc == XDMAD_OK);

  /* Put all relevant interrupts on */
  xdmaInt =  (
    XDMAC_CIE_BIE |
    XDMAC_CIE_DIE |
    XDMAC_CIE_FIE |
    XDMAC_CIE_RBIE |
    XDMAC_CIE_WBIE |
    XDMAC_CIE_ROIE);

  /* Setup RX */
  memset(&cfg, 0, sizeof(cfg));
  channel = XDMAIF_Get_ChannelNumber(bus->spi_id, XDMAD_TRANSFER_RX);
  cfg.mbr_sa = (uint32_t)&bus->spi_regs->SPI_RDR;
  cfg.mbr_cfg =
    XDMAC_CC_TYPE_PER_TRAN |
    XDMAC_CC_MBSIZE_SINGLE |
    XDMAC_CC_DSYNC_PER2MEM |
    XDMAC_CC_CSIZE_CHK_1 |
    XDMAC_CC_DWIDTH_BYTE |
    XDMAC_CC_SIF_AHB_IF1 |
    XDMAC_CC_DIF_AHB_IF1 |
    XDMAC_CC_SAM_FIXED_AM |
    XDMAC_CC_DAM_INCREMENTED_AM |
    XDMAC_CC_PERID(channel);
  xdma_cndc = XDMAC_CNDC_NDVIEW_NDV0 |
    XDMAC_CNDC_NDE_DSCR_FETCH_EN |
    XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED |
    XDMAC_CNDC_NDSUP_SRC_PARAMS_UNCHANGED;
  rc = XDMAD_ConfigureTransfer(
    &XDMAD_Instance,
    bus->dma_rx_channel,
    &cfg,
    xdma_cndc,
    (uint32_t) bus->dma_bufs[DMA_BUF_RX].desc,
    xdmaInt
  );
  assert(rc == XDMAD_OK);

  /* Setup TX  */
  memset(&cfg, 0, sizeof(cfg));
  channel = XDMAIF_Get_ChannelNumber(bus->spi_id, XDMAD_TRANSFER_TX);
  cfg.mbr_da = (uint32_t)&bus->spi_regs->SPI_TDR;
  cfg.mbr_cfg =
    XDMAC_CC_TYPE_PER_TRAN |
    XDMAC_CC_MBSIZE_SINGLE |
    XDMAC_CC_DSYNC_MEM2PER |
    XDMAC_CC_CSIZE_CHK_1 |
    XDMAC_CC_DWIDTH_BYTE |
    XDMAC_CC_SIF_AHB_IF1 |
    XDMAC_CC_DIF_AHB_IF1 |
    XDMAC_CC_SAM_INCREMENTED_AM |
    XDMAC_CC_DAM_FIXED_AM |
    XDMAC_CC_PERID(channel);
  xdma_cndc = XDMAC_CNDC_NDVIEW_NDV0 |
    XDMAC_CNDC_NDE_DSCR_FETCH_EN |
    XDMAC_CNDC_NDDUP_DST_PARAMS_UNCHANGED |
    XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED;
  rc = XDMAD_ConfigureTransfer(
    &XDMAD_Instance,
    bus->dma_tx_channel,
    &cfg,
    xdma_cndc,
    (uint32_t) bus->dma_bufs[DMA_BUF_TX].desc,
    xdmaInt
  );
  assert(rc == XDMAD_OK);
}

int spi_bus_register_atsam(
  const char *bus_path,
  const atsam_spi_config *config
)
{
  atsam_spi_bus *bus;
  size_t i;

  bus = (atsam_spi_bus *) spi_bus_alloc_and_init(sizeof(*bus));
  if (bus == NULL) {
    return -1;
  }

  bus->base.transfer = atsam_spi_transfer;
  bus->base.destroy = atsam_spi_destroy;
  bus->base.setup = atsam_spi_setup;
  bus->base.max_speed_hz = MAX_SPI_FREQUENCY;
  bus->base.bits_per_word = 8;
  bus->base.speed_hz = bus->base.max_speed_hz;
  bus->base.cs = 1;
  bus->spi_id = config->spi_peripheral_id;
  bus->spi_regs = config->spi_regs;
  bus->chip_select_decode = config->chip_select_decode;
  bus->peripheral_clk_per_us = BOARD_MCK / 1000000;
  bus->spi_mr = SPI_MR_MSTR | SPI_MR_MODFDIS;

  for (i = 0; i < RTEMS_ARRAY_SIZE(bus->spi_csr); ++i) {
    if (config->dlybs_in_ns[i] != 0) {
      bus->spi_csr[i] |= SPI_DLYBS(config->dlybs_in_ns[i], BOARD_MCK);
    }

    if (config->dlybct_in_ns[i] != 0) {
      bus->spi_csr[i] |= SPI_DLYBCT(config->dlybct_in_ns[i], BOARD_MCK);
    }
  }

  rtems_binary_semaphore_init(&bus->sem, "ATSAM SPI");
  PIO_Configure(config->pins, config->pin_count);
  PMC_EnablePeripheral(config->spi_peripheral_id);
  atsam_reset_spi(bus);
  atsam_configure_spi(bus);
  atsam_spi_init_xdma(bus);

  return spi_bus_register(&bus->base, bus_path);
}
