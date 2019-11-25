/*-
 * Copyright (c) 2017 George Abdelmalik <gabdelmalik@fork.id.au>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Xilinx Quad-SPI Flash controller. 
 *
 * Refer to chapter 12 of the Xilinx Zynq-7000 All programmable SoC
 * techincal reference manual, ug586-Zynq-7000-TRM.pdf.
 *
 * Note: Only Flash memory I/O mode is supported by this driver implementation.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/conf.h>

#include <sys/malloc.h>
#include <sys/rman.h>

#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>

#include "spibus_if.h"

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/xilinx/zy7_slcr.h>

#include <machine/bus.h>
#include <machine/intr.h>

#define	RD4(_sc, _reg) \
	bus_read_4((_sc)->mem_res, (_reg))
#define	WR4(_sc, _reg, _val) \
	bus_write_4((_sc)->mem_res, (_reg), (_val))
#define BW(_sc) \
	bus_barrier((_sc)->mem_res, 0, 0xff, BUS_SPACE_BARRIER_WRITE)
#define BR(_sc) \
	bus_barrier((_sc)->mem_res, 0, 0xff, BUS_SPACE_BARRIER_READ)
#define BRW(_sc) \
	bus_barrier((_sc)->mem_res, 0, 0xff, \
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE)
#define QSPI_LOCK(_sc) \
	mtx_lock(&(_sc)->sc_mtx)
#define QSPI_UNLOCK(_sc) \
	mtx_unlock(&(_sc)->sc_mtx)
#define QSPI_LOCK_INIT(_sc) \
	mtx_init(&(_sc)->sc_mtx, device_get_nameunit((_sc)->dev), NULL, MTX_DEF)
#define QSPI_LOCK_DESTROY(_sc) \
	mtx_destroy(&(_sc)->sc_mtx)
#define QSPI_ASSERT_LOCKED(_sc) \
	mtx_assert(&(_sc)->sc_mtx, MA_OWNED)


#define FIFO_DEPTH 252 /* byte depth of Rx and Tx FIFO queues */

#define QSPI_CR  	0x00 /* Configuration register */
#define  CR_FLASH  (1 << 31)       /* Enable Flash IO Interface */
#define  CR_BIG_ENDIAN  (1 << 26)  /* Litte otherwise */
#define  CR_SELF_HOLD  (1 << 19)   /* Don't rely on exernal pull-up of unused pins. MUST */
#define  CR_MSTC (1 << 16)         /* Start command SPI transfer */
#define  CR_MSE  (1 << 15)         /* Manual start enable, 0 means auto */
#define  CR_MCS  (1 << 14)         /* Manual chip-select enable, 0 means auto */
/* Chip select line, 0 means asserted, 1 means de-asserted */
#define  CR_CSL_SHIFT   10 
#define  CR_CSL_MASK (1 << CR_CSL_SHIFT)
#define  CR_CSL_FLAG (1 << CR_CSL_SHIFT)
#define   CR_CSL_ASSERT    (~(1 << CR_CSL_SHIFT))  /* assert */
   /* To assert the chip-select line, it MUST be ANDed
    * since active is '0' (zero), e.g.
    * reg |= CR_CSL_MASK;
    * reg &= CR_CSL_ASSERT;
    */  
#define  CR_FIFO_WITDH (0x3 << 6)  /* FIFO width, 0b11 means be 32 bits, MUST be this value */
   /* QSPI line clock baud rate divisor */
#define  CR_BR_DIV_SHIFT 3
#define  CR_BR_DIV_MIN 2
#define  CR_BR_DIV_MAX 256
#define  CR_BR_DIV_MASK  (7 << CR_BR_DIV_SHIFT)
   /* Exactly one of these shall be specified.
	* reg &= ~CR_BR_DIV_MASK;
	* reg |= CR_BR_DIV_4;
    */	
#define   CR_BR_DIV_2   (0 << CR_BR_DIV_SHIFT)  /* divisor of 2 */
#define   CR_BR_DIV_4   (1 << CR_BR_DIV_SHIFT)  /* " " 4 */
#define   CR_BR_DIV_8   (2 << CR_BR_DIV_SHIFT)  /* " " 8 */
#define   CR_BR_DIV_16  (3 << CR_BR_DIV_SHIFT)  /* " " 16 */
#define   CR_BR_DIV_32  (4 << CR_BR_DIV_SHIFT)  /* " " 32 */
#define   CR_BR_DIV_64  (5 << CR_BR_DIV_SHIFT)  /* " " 64 */
#define   CR_BR_DIV_128 (6 << CR_BR_DIV_SHIFT)  /* " " 128 */
#define   CR_BR_DIV_256 (7 << CR_BR_DIV_SHIFT)  /* " " 256 */
/* CLK_PH and CLK_POL bits combined */
#define  CR_CLK_PH_POL (0 << 1) /* Either 0 or 0b11 are valid values */
#define  CR_MODE_MASTER (1)  /* Select master mode */
#define QSPI_EN  	0x14 /* Enable register */
#define  EN_ENABLE  1   /* Enable */
#define  EN_DISABLE 0   /* Disable */
#define QSPI_LCR  	0xA0 /* Linear Configuration register */
#define QSPI_ISR 	0x04 /* Interrupt status register */
#define QSPI_IER 	0x08 /* Interrupt enable register */
#define QSPI_IDR 	0x0c /* Interrupt disable register */
#define QSPI_IMR 	0x10 /* Interrupt mask register */
	/* Interrupt flags are common to all interrupt registers */
#define  IR_MASK   (  0x7f)    /* interrupt bits mask */
#define  IR_ALL    IR_MASK     /* all interrupts */
#define  IR_TXUF   (1 << 6)    /* Tx FIFO underflow */
#define  IR_RXFULL (1 << 5)    /* Rx FIFO is full */
#define  IR_RXNE   (1 << 4)    /* Rx FIFO not empty, or >= threshold */
#define  IR_TXFULL (1 << 3)    /* Tx FIFO is full */
#define  IR_TXNOTFULL (1 << 2) /* Tx FIFO is < threshold */
#define  IR_UNUSED    (1 << 1) /* bit 1 is not used */
#define  IR_RXOF      (1)      /* Rx FIFO overflow */
#define QSPI_TXD  	0x1C /* Tx fifo for 4-byte instruction, in LSB first order */
#define QSPI_TXD1  	0x80 /* Tx fifo for 1-byte instruction, in LSB first order */
#define QSPI_TXD2  	0x84 /* Tx fifo for 2-byte instruction, in LSB first order */
#define QSPI_TXD3  	0x88 /* Tx fifo for 3-byte instruction, in LSB first order */
#define QSPI_RXD 	0x20   /* Rx fifo, in MSB first order */
#if 0
#define SPI_ER  	0x14 /* SPI enable register */
#define  ER_MASK    (1)         /* enable bit mask */
#define  ER_ENABLE  (1)         /* Enable SPI */
#define SPI_DR  	0x18 /* Delay register, see doco */
#define  DR_DNSS_POS  24 /* bit offset */
#define  DR_DNSS_MASK  (0xff << DR_DNSS_POS)
#define  DR_BTWN_POS  16 /* bit offset */
#define  DR_BTWN_MASK  (0xff << 16)
#define  DR_AFTER_POS 8 /* bit offset */
#define  DR_AFTER_MASK (0xff << DR_AFTER_POS)
#define  DR_INIT_MASK  (0xff)
#define SPI_TXD 	0x1C /* Tx data register, [7:0] valid bits */
#define SPI_RXD 	0x20 /* Rx data register, [7:0] valid bits */
#define SPI_SICR	0x24 /* Slave idle count register, only for slave mode */
#define  SICR_COUNT_MASK  (0xff) /* see doco */
#define SPI_TXTHLD	0x28 /* Tx FIFO threshold register. Defines when the */
                         /* TXNOTFULL interrupt is triggered */
//#define  TXTHLD_RESET_VAL  1 /* */
//#define  TXTHLD_MIN_VAL    TXTHLD_RESET_VAL
//#define  TXTHLD_MAX_VAL    (FIFO_DEPTH - 1)
#define SPI_RXTHLD	0x2C /* Rx FIFO threshold register. Defines when the */
                         /* RXNE interrupt is triggered */
#define SPI_MODID	0xFC /* Module ID register */
#define  MODID_MASK  (0xffffff)
#endif

#define QSPI_LINE_CLK_MAX  100000000   /* 100 MHz */
#define QSPI_LINE_CLK_DEFAULT  QSPI_LINE_CLK_MAX
#define QSPI_REF_CLK_DEFAULT 200000000   /* 200 Mhz */

static struct ofw_compat_data qspi_matches[] = {
	/* found in vendor Linux upstreamed DTS files */
	{ "xlnx,zynq-qspi-1.0" , true }, 
	/* found in FreeBSD zedboard DTS file */
	{ "xlnx,zy7_qspi"      , true },
	{ NULL                 , false }
};

/* QSPI clocks have been initialised */
static bool clocks_done = false;

/* Completion status of an SPI transfer */
enum proto_xfer_state {
	/* Contoller is available to begin a new transfer. */
	xfer_state_idle,

	/* Transmission of tx fifo is in progress */
	xfer_state_tx_progress,

	/* Waiting on the rx fifo to contain the complete response */
	xfer_state_awaiting_rx,
};

/* SPI transfer control block */
struct xfer_control {
	enum proto_xfer_state	xfer_state;
	struct spi_command	*xfer_cmd;
	int			xfer_size;
	int			xfer_read;
	int			xfer_written;
	const char		*err_msg;
};

struct qspi_softc {
	struct mtx		sc_mtx;
	device_t	 	dev;
	device_t	 	spibus; /* the attached child device */
	struct resource		*mem_res;
	struct resource		*irq_res;
	void	 *ih;
	int		 qspi_clk_freq;
	struct xfer_control	pcb;
};

#if 0
static void
dump_registers(struct cspi_softc *sc)
{
	CSPI_ASSERT_LOCKED(sc);
	int regs[] = { SPI_CR, SPI_IMR, SPI_ER, SPI_DR, SPI_TXTHLD, SPI_RXTHLD,
	    SPI_MODID };
	for (int i=0; i < nitems(regs); ++i) {
		BRW(sc);
		device_printf(sc->dev, "reg[0x%02x] = 0x%08x\n",
		    regs[i], RD4(sc, regs[i]));
	}
}
#endif

static void
qspi_modifyreg(struct qspi_softc *sc, uint32_t off, uint32_t mask,
    uint32_t value)
{
	QSPI_ASSERT_LOCKED(sc);
	BRW(sc);
	uint32_t reg = RD4(sc, off);
	reg &= ~mask;
	reg |= value;
	BRW(sc);
	WR4(sc, off, reg);
}

static bool
xfer_in_error(struct qspi_softc *sc)
{
	QSPI_ASSERT_LOCKED(sc);
	struct xfer_control *pcb = &sc->pcb;
	return pcb->err_msg;
}

static const char *
xfer_get_error_message(struct qspi_softc *sc)
{
	QSPI_ASSERT_LOCKED(sc);
	struct xfer_control *pcb = &sc->pcb;
	return pcb->err_msg;
}

static void
xfer_set_error_message(struct qspi_softc *sc, const char *err_msg)
{
	QSPI_ASSERT_LOCKED(sc);
	struct xfer_control *pcb = &sc->pcb;
	pcb->err_msg = err_msg;
}

static bool
xfer_is_busy(struct qspi_softc *sc)
{
	QSPI_ASSERT_LOCKED(sc);
	struct xfer_control *pcb = &sc->pcb;
	return pcb->xfer_state != xfer_state_idle;
}
#if 0
static bool
xfer_is_awaiting_rx(struct cspi_softc *sc)
{
	CSPI_ASSERT_LOCKED(sc);
	struct xfer_control *pcb = &sc->pcb;
	return pcb->xfer_state == xfer_state_awaiting_rx;
}

static bool
xfer_has_more_to_send(struct cspi_softc *sc)
{
	CSPI_ASSERT_LOCKED(sc);
	struct xfer_control *pcb = &sc->pcb;
	return pcb->xfer_written < pcb->xfer_size;
}
#endif
static void
qspi_assert_chip_select(struct qspi_softc *sc)
{
	QSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	qspi_modifyreg(sc, QSPI_CR, CR_CSL_MASK, 0);
}

static void
qspi_enable_controller(struct qspi_softc *sc)
{
	QSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	WR4(sc, QSPI_EN, EN_ENABLE);
}

static int
qspi_fill_txfifo(struct qspi_softc *sc, const char *begin, int how_many)
{
	QSPI_ASSERT_LOCKED(sc);
	/* Don't overflow the depth of the FIFO */
	const int depth = (how_many < FIFO_DEPTH) ? how_many : FIFO_DEPTH;
//GA device_printf(sc->dev, "**GA %s to depth of %d bytes\n", __func__, depth);
	for (int i=0; i < depth; ++i) {
		BRW(sc);
		//GA TODO Which TX keyhole to use ???
		//GA TODO 32bit register value is filled in LSB first order
		WR4(sc, QSPI_TXD, begin[i]);
	}
	return depth;
}

static bool
qspi_drain_rxfifo(struct qspi_softc *sc)
{
	QSPI_ASSERT_LOCKED(sc);
	bool result = true;
	struct xfer_control *pcb = &sc->pcb;
	const int to_read = pcb->xfer_written - pcb->xfer_read;
//GA device_printf(sc->dev, "**GA %s of %d bytes\n", __func__, to_read);
	KASSERT( to_read > 0 && to_read <= FIFO_DEPTH,
	    ("%s: Invalid read size, %d bytes", __func__, to_read));
	char *cmd_ptr = (char *)pcb->xfer_cmd->rx_cmd;
	const int cmd_size = (int)pcb->xfer_cmd->rx_cmd_sz;
	/* XXX - rx_data is optional and may hence be NULL */
	char *data_ptr = (char *)pcb->xfer_cmd->rx_data;
	const int data_size = (int)pcb->xfer_cmd->rx_data_sz;
	const bool belongings_to_cmd = (pcb->xfer_read < cmd_size)
	    ? true : false;

	/* Set to store rx bytes into the correct location. */
	char *location = belongings_to_cmd
	    ? cmd_ptr + pcb->xfer_read
	    : data_ptr + (pcb->xfer_read - cmd_size);
	/* limit the bytes to store to the size remaining in the location */
	const int remaining_space = belongings_to_cmd
	    ? cmd_size - pcb->xfer_read
	    : (cmd_size + data_size) - pcb->xfer_read;
	const int to_store = (remaining_space < to_read)
	    ? remaining_space
	    : to_read;
	if (to_read != to_store) {
		xfer_set_error_message(sc,
		    "Rx buffer too small, expect dropped data");
		result = false;
	}
	/* Drain the RX FIFO of its data, store what we can into the
	 * rx buffer.
	 */
	 // GA TODO - serialise the 4 bytes that are returned by each read
		//GA TODO 32bit register value is populated by the controller in MSB first order
	for (int i=0; i < to_read; ++i) {
		BRW(sc);
		uint32_t d = RD4(sc, SPI_RXD);
		if (i < to_store) 
			*location++ = d;
	}
	pcb->xfer_read += to_read;
	return result;
}

static void
qspi_enable_interrupts(struct qspi_softc *sc)
{
	QSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	const uint32_t intr_flags =
	    IR_TXUF | IR_RXOF /* error conditions */
	    | IR_TXNOTFULL /* tx fifo has been put out on the wire */
      ;
	WR4(sc, QSPI_IER, intr_flags);
}

static void
qspi_start_transfer(struct qspi_softc *sc)
{
	QSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	qspi_modifyreg(sc, QSPI_CR, CR_MSTC, CR_MSTC);
}

static void
qspi_stop_transfer(struct qspi_softc *sc)
{
	QSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	qspi_modifyreg(sc, QSPI_CR, CR_MSTC, 0);
}

static void
qspi_disable_interrupts(struct qspi_softc *sc, unsigned intr_flags)
{
	QSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s, flags 0x%02x\n", __func__, intr_flags);
	WR4(sc, QSPI_IDR, intr_flags);
}

static void
qspi_disable_controller(struct qspi_softc *sc)
{
	QSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	WR4(sc, QSPI_EN, EN_DISABLE);
}

static void
qspi_deassert_chip_select(struct qspi_softc *sc)
{
	QSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	qspi_modifyreg(sc, QSPI_CR, CR_CSL_MASK, CR_CSL_FLAG);
}

static void
qspi_clear_interrupts(struct qspi_softc *sc, unsigned intr_flags)
{
	QSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s, flags 0x%02x\n", __func__, intr_flags);
	WR4(sc, QSPI_ISR, intr_flags);
}


static uint32_t
qspi_read_interrupt_status(struct qspi_softc *sc)
{
	QSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	return RD4(sc, QSPI_ISR);
}

static void
xfer_move_to_idle(struct qspi_softc *sc)
{
	QSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	struct xfer_control *pcb = &sc->pcb;
	pcb->xfer_state = xfer_state_idle;
	pcb->xfer_cmd = NULL;
	pcb->xfer_size = pcb->xfer_read = pcb->xfer_written = 0;
 	pcb->err_msg = NULL;
}

static void
xfer_move_to_tx_progress(struct qspi_softc *sc)
{
	QSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	struct xfer_control *pcb = &sc->pcb;
	KASSERT(pcb->xfer_cmd != NULL,
	    ("%s: Missing transfer command ", __func__));
	pcb->xfer_state = xfer_state_tx_progress;
	int how_many = 0;
	const char *cmd_ptr = (const char *)pcb->xfer_cmd->tx_cmd;
	const int cmd_size = (int)pcb->xfer_cmd->tx_cmd_sz;
	/* XXX - tx_data is optional and may hence be NULL */
	const char *data_ptr = (const char *)pcb->xfer_cmd->tx_data;
	const int data_size = (int)pcb->xfer_cmd->tx_data_sz;

	const bool doing_cmd = pcb->xfer_written < cmd_size ? true : false;

	/* either transferring portions of the cmd or data */
	const char *start = NULL;
	if (doing_cmd) {
		start = &cmd_ptr[pcb->xfer_written];
		how_many = cmd_size - pcb->xfer_written;
	} else { /* doing data */
		const int begin = pcb->xfer_written - cmd_size;
		start = &data_ptr[begin];
		how_many = data_size - begin;
	}
	const int queued = qspi_fill_txfifo(sc, start, how_many);
	pcb->xfer_written += queued;
	qspi_enable_interrupts(sc);
	qspi_start_transfer(sc);
}

static void
qspi_intr(void *arg)
{
	struct qspi_softc *sc = (struct qspi_softc *)arg;
	bool done = false;

	QSPI_LOCK(sc);
	
	uint32_t stat = qspi_read_interrupt_status(sc);
//GA device_printf(sc->dev, "**GA %s status 0x%08x\n", __func__, stat);

	/* Ignore the interrupt if we're not performing a transfer */
	if (!xfer_is_busy(sc)) {
//GA device_printf(sc->dev, "**GA %s - interrupt ignored\n", __func__);
		qspi_disable_interrupts(sc, IR_ALL);
		qspi_clear_interrupts(sc, IR_ALL);
		QSPI_UNLOCK(sc);
		return;
	}

	if ((stat & IR_TXNOTFULL) && !xfer_in_error(sc)) {
//GA device_printf(sc->dev, "**GA %s - IR_TXNOTFULL\n", __func__);
	  qspi_stop_transfer(sc);
		qspi_disable_interrupts(sc, IR_TXNOTFULL);
		qspi_clear_interrupts(sc, IR_TXNOTFULL);
		/* all rx data has arrived. */
		bool ok = qspi_drain_rxfifo(sc);
		if (ok && xfer_has_more_to_send(sc)) {
//GA device_printf(sc->dev, "**GA %s - Has more to send\n", __func__);
			xfer_move_to_tx_progress(sc);
		} else
			done = true;
	}
	if ((stat & IR_TXUF) && !xfer_in_error(sc))
		xfer_set_error_message(sc, "Tx underflow occurred");
	if ((stat & IR_RXOF) && !xfer_in_error(sc))
		xfer_set_error_message(sc, "Rx overflow occurred");

	if (done || xfer_in_error(sc)) {
		/* awake the requester of this completed transfer */
		wakeup(sc->dev);
	}

	QSPI_UNLOCK(sc);
}

static int
qspi_transfer(device_t dev, device_t child, struct spi_command *cmd)
{
	struct qspi_softc *sc = device_get_softc(dev);

	if (cmd->tx_cmd_sz != cmd->rx_cmd_sz
	    || cmd->tx_data_sz != cmd->rx_data_sz) {
		device_printf(dev,
		    "Tx/Rx command(%u/%u) and/or data(%u/%u) "
		    "buffer size mismatch\n",
		    cmd->tx_cmd_sz, cmd->rx_cmd_sz,
		    cmd->tx_data_sz, cmd->rx_data_sz);
		return EIO;
	}

	QSPI_LOCK(sc);

	/* A new transfer requester shall wait for any outstanding 
	 * transfer to complete before proceeding.
	 */
	while (xfer_is_busy(sc))
		mtx_sleep(dev, &sc->sc_mtx, 0, "qspi - pending", 0);

	/* Now it's our turn. */
	sc->pcb.xfer_cmd = cmd;
	sc->pcb.xfer_size = (int)cmd->tx_cmd_sz + (int)cmd->tx_data_sz;
//GA dump_registers(sc);
	qspi_assert_chip_select(sc);
	qspi_enable_controller(sc);
	xfer_move_to_tx_progress(sc);
//GA dump_registers(sc);
	/* Sleep Waiting for the trasfer to complete, or timeout */
	int err = mtx_sleep(dev, &sc->sc_mtx, 0, "qspi - inprogress", hz * 2);

	/* Check for transfer timeout. */
	if (err == EWOULDBLOCK) {
		if (!xfer_in_error(sc))
			xfer_set_error_message(sc, "QSPI transfer timeout");
	}

	/* Report any error */
	if (xfer_in_error(sc)) {
		device_printf(dev, "%s\n", xfer_get_error_message(sc));
		err = EIO;
	}

	qspi_stop_transfer(sc);
	qspi_disable_interrupts(sc, IR_ALL);
	qspi_clear_interrupts(sc, IR_ALL);
	qspi_disable_controller(sc);
	qspi_deassert_chip_select(sc);
/* TODO also clear tx/rx fifo incase of aborted transfer */
	xfer_move_to_idle(sc);
	
	/* Awake the next requester which would have slept waiting for
	 * this transfer to complete.
	 */
	wakeup_one(dev);
//GA dump_registers(sc);
	QSPI_UNLOCK(sc);

	return err;
}

static int
qspi_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return ENXIO;

	if (!ofw_bus_search_compatible(dev, qspi_matches)->ocd_data)
		return ENXIO;

	device_set_desc(dev, "Quad-SPI Flash Controller");
	return BUS_PROBE_DEFAULT;
}

static void
qspi_softc_init(struct qspi_softc *sc, device_t dev)
{
	sc->dev = dev;
	QSPI_LOCK_INIT(sc);
	sc->mem_res = NULL;
	sc->irq_res = NULL;
	sc->ih = NULL;
	sc->spibus = NULL;
	sc->qspi_clk_freq = QSPI_LINE_CLK_DEFAULT;
	QSPI_LOCK(sc);
	xfer_move_to_idle(sc);
	QSPI_UNLOCK(sc);
}

static uint8_t
divisor_as_field(int divisor)
{
	switch (divisor) {
	case 2: return CR_BR_DIV_2;
	case 4: return CR_BR_DIV_4;
	case 8: return CR_BR_DIV_8;
	case 16: return CR_BR_DIV_16;
	case 32: return CR_BR_DIV_32;
	case 64: return CR_BR_DIV_64;
	case 128: return CR_BR_DIV_128;
	default: return CR_BR_DIV_256;
	}
}

static int
config_init(device_t dev)
{
	struct qspi_softc *sc = device_get_softc(dev);
	int ref_freq;
	int error = qspi_get_ref_clk_freq(&ref_freq);
	if (error)
		return error;

	/* Find the dividor that when applied to the SPI reference
	 * clock, will give an SPI line rate <= the desired SPI signal
   * clock frequency.
	 */
	int divisor = CR_BR_DIV_MIN; 
	while (divisor <= CR_BR_DIV_MAX){
		if (ref_freq/divisor > sc->qspi_clk_freq)
			divisor *= 2;   /* divisor not large enough yet */
		else
			break;
	}

	if( divisor > CR_BR_DIV_MAX )
		return error;

	int reg = 0;
	reg |= CR_FLASH | CR_SELF_HOLD | CR_FIFO_WITDH;
	reg |= CR_MSE | CR_MCS;
	reg |= CR_CSL_MASK; /* De-assert chip-select line */
	reg |= divisor_as_field(divisor);
	reg |= CR_CLK_PH_POL;
	reg |= CR_MODE_MASTER; /* Only master mode is supported */
	WR4(sc, QSPI_CR, reg);

	if (bootverbose)
		device_printf(dev, "master clock frequency %dHz\n",
		    ref_freq / divisor);

	return 0;
}


static int
init_clocks()
{
	int error = qspi_set_ref_clk_source(zy7_clk_src_iopll);
	if (!error)
		return qspi_set_ref_clk_freq(QSPI_REF_CLK_DEFAULT);
	return error;
}


static int qspi_detach(device_t dev);

static int
qspi_attach(device_t dev)
{
	int error;

	/* Only do once on the first instance attachment, as setting
	 * apply to all instances.
	 */
	if (!clocks_done) {
		error = init_clocks();
		if (error) {
			device_printf(dev, "clock init failed\n");
			return ENXIO;
		}
		clocks_done = true;
		if (bootverbose) {
			int ref_freq = 0;
			enum zy7_clk_src ref_source;
			(void)qspi_get_ref_clk_freq(&ref_freq);
			(void)qspi_get_ref_clk_source(&ref_source);
			device_printf(dev, "reference clock source %s\n",
			    zy7_clk_src_as_string(ref_source));
			device_printf(dev, "reference clock frequency %dHz\n",
			    ref_freq);
		}

		/* Reset this controller instance */
		error = qspi_clk_reset();
		if (error) {
			device_printf(dev, "hardware reset failed\n");
			return ENXIO;
		}
	}

	struct qspi_softc *sc = device_get_softc(dev);

	/* Initialise software context parameters. */
	qspi_softc_init(sc, dev);

	/* Acquire the memory resource */
	int rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->mem_res) {
		device_printf(dev, "could not allocate memory resource\n");
		goto err;
	}

	/* Aquire the IRQ resource. */
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (!sc->irq_res) {
		device_printf(dev, "could not allocate interrupt resource.\n");
		goto err;
	}

	/* Hook up our interrupt handler. */
	if (bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, qspi_intr, sc, &sc->ih)) {
		device_printf(dev, "cannot setup the interrupt handler\n");
		goto err;
	}

	/* SPI clock freq may be overridden via DTB */
	pcell_t cell;
	phandle_t node = ofw_bus_get_node(dev);
	if (OF_getprop(node, "spi-clock", &cell, sizeof(cell)) > 0)
		sc->qspi_clk_freq = fdt32_to_cpu(cell);

	/* Initialise the config of the controller */
	error = config_init(dev);
	if (error) {
		device_printf(dev, "config failed\n");
		goto err;
	}

	sc->spibus = device_add_child(dev, "spibus", -1);
	if (!sc->spibus) {
		device_printf(dev, "attach of spibus failed\n");
		goto err;
	}

	return bus_generic_attach(dev);

err:
	qspi_detach(dev);
	return ENXIO;
}

static int
qspi_detach(device_t dev)
{
	struct qspi_softc *sc = device_get_softc(dev);

	QSPI_LOCK(sc);

	/* Place the controller into a sane state.
	 * - interrupts disabled.
	 * - interrupt status register cleared
	 * - reset any tx/rx fifo threshold levels
	 * - tx/rx fifo queues cleared
	 */

	/* release interrupt resource */
	if (sc->irq_res) {
		if (sc->ih)
			bus_teardown_intr(dev, sc->irq_res, sc->ih);
		bus_release_resource(dev, SYS_RES_IRQ,
		    rman_get_rid(sc->irq_res), sc->irq_res);
		sc->irq_res = NULL;
	}

	/* release memory resource */
	if (sc->mem_res) {
		bus_release_resource(dev,SYS_RES_MEMORY,
		    rman_get_rid(sc->mem_res), sc->mem_res);
		sc->mem_res = NULL;
	}

	if (sc->spibus)
		device_delete_child(dev, sc->spibus);

	QSPI_UNLOCK(sc);
	QSPI_LOCK_DESTROY(sc);
	return bus_generic_detach(dev);
}

static int
qspi_driver_load(module_t mod, int what, void *arg)
{
	switch (what) {
	case MOD_LOAD:
		uprintf("qspi: loaded.\n");
		break;
	case MOD_UNLOAD:
		uprintf("qspi: unloaded.\n");
		break;
	case MOD_SHUTDOWN:
		uprintf("qspi: system shutting down.\n");
		break;
	case MOD_QUIESCE:
		uprintf("qspi: unload coming.\n");
		break;
	}
	return (0);
}

static device_method_t qspi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		qspi_probe),
	DEVMETHOD(device_attach,	qspi_attach),
	DEVMETHOD(device_detach,	qspi_detach),
	/* SPI interface */
	DEVMETHOD(spibus_transfer,	qspi_transfer),
	/* end sentinel */
	DEVMETHOD_END
};

static driver_t qspi_driver = {
	"spi", // so spibus will attach to it
	qspi_methods,
	sizeof(struct qspi_softc),
};

static devclass_t qspi_devclass;

DRIVER_MODULE(qspi, simplebus, qspi_driver, qspi_devclass,
    qspi_driver_load, NULL);
MODULE_VERSION(qspi, 1);
