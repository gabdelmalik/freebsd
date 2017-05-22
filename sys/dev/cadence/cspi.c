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
 * Xilinx SPI controller based on the Cadence SIP core, 
 *
 * Refer to chapter 17 of the Xilinx Zynq-7000 All programmable SoC
 * techincal reference manual, ug586-Zynq-7000-TRM.pdf.
 *
 * Note: Only SPI Master mode is supported by this driver implementation.
 */

//#include <sys/cdefs.h>

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
	bus_barrier((_sc)->mem_res, 0, 0, BUS_SPACE_BARRIER_WRITE)
#define BR(_sc) \
	bus_barrier((_sc)->mem_res, 0, 0, BUS_SPACE_BARRIER_READ)
#define BRW(_sc) \
	bus_barrier((_sc)->mem_res, 0, 0, \
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE)
#define CSPI_LOCK(_sc) \
	mtx_lock(&(_sc)->sc_mtx)
#define CSPI_UNLOCK(_sc) \
	mtx_unlock(&(_sc)->sc_mtx)
#define CSPI_LOCK_INIT(_sc) \
	mtx_init(&(_sc)->sc_mtx, device_get_nameunit((_sc)->dev), NULL, MTX_DEF)
#define CSPI_LOCK_DESTROY(_sc) \
	mtx_destroy(&(_sc)->sc_mtx)
#define CSPI_ASSERT_LOCKED(_sc) \
	mtx_assert(&(_sc)->sc_mtx, MA_OWNED)


#define FIFO_DEPTH 128 /* byte depth of Rx and Tx FIFO queues */

#define SPI_CR  	0x00 /* Configuration register */
#define  CR_MFG  (1 << 17)       /* ModeFail generation enable */
#define  CR_MSTC (1 << 16)       /* Manual start command, 0 means auto */
#define  CR_MSE  (1 << 15)       /* Manual start enable, 0 means auto */
#define  CR_MCS  (1 << 14)       /* Manual chip-select enable, 0 means auto */
#define  CR_CSL_SHIFT	10 
#define  CR_CSL_NONE (0xf << CR_CSL_SHIFT) /* No slave selected */
#define  CR_CSL_MASK CR_CSL_NONE /* mask slave select bits */
   /* At most one of these chip-select line values, and it MUST be ANDed
    * since active bit is '0' (zero), e.g.
	* reg |= CR_CSL_MASK;
	* reg &= CR_CLS_1;
    */	
//#define   CR_CSL_0	(~(1 << 10))  /* select slave 0 */
//#define   CR_CSL_1	(~(1 << 11))  /* select slave 1 */
//#define   CR_CSL_2	(~(1 << 12))  /* select slave 2 */
//GA #define   CR_CSL(num)	(~(1 << (10 + (num)))) /* select slave line number */
   /* SPI line clock baud rate divisor */
#define  CR_BR_DIV_SHIFT 3
#define  CR_BR_DIV_MIN 4
#define  CR_BR_DIV_MAX 256
#define  CR_BR_DIV_MASK  (7 << CR_BR_DIV_SHIFT)
   /* Exactly one of these shall be specified.
	* reg &= ~CR_BR_DIV_MASK;
	* reg |= CR_BR_DIV_4;
    */	
#define   CR_BR_DIV_4   (1 << CR_BR_DIV_SHIFT)  /* divisor of 4 */
#define   CR_BR_DIV_8   (2 << CR_BR_DIV_SHIFT)  /* " " 8 */
#define   CR_BR_DIV_16  (3 << CR_BR_DIV_SHIFT)  /* " " 16 */
#define   CR_BR_DIV_32  (4 << CR_BR_DIV_SHIFT)  /* " " 32 */
#define   CR_BR_DIV_64  (5 << CR_BR_DIV_SHIFT)  /* " " 64 */
#define   CR_BR_DIV_128 (6 << CR_BR_DIV_SHIFT)  /* " " 128 */
#define   CR_BR_DIV_256 (7 << CR_BR_DIV_SHIFT)  /* " " 256 */
#define  CR_CLK_PH_FALLING (1 << 2) /* Clock phase falling edge active, */
                                  /* otherwise rising edge active, */
#define  CR_CLK_POL_HIGH (1 << 1) /* Clock inactive polarity is high */
				  /* otherwise it is low. */
#define  CR_MODE_MASTER (1)  /* Is SPI Master, otherwise slave */
#define SPI_ISR 	0x04 /* Interrupt status register */
#define SPI_IER 	0x08 /* Interrupt enable register */
#define SPI_IDR 	0x0c /* Interrupt disable register */
#define SPI_IMR 	0x10 /* Interrupt mask register */
	/* Interrupt flags are common to all interrupt registers */
#define  IR_MASK   (  0x7f)    /* interrupt bits mask */
#define  IR_ALL    IR_MASK     /* all interrupts */
#define  IR_TXUF   (1 << 6)    /* Tx FIFO underflow */
#define  IR_RXFULL (1 << 5)    /* Rx FIFO is full */
#define  IR_RXNE   (1 << 4)    /* Rx FIFO not empty, or >= threshold */
#define  IR_TXFULL (1 << 3)    /* Tx FIFO is full */
#define  IR_TXNOTFULL (1 << 2) /* Tx FIFO is < threshold */
#define  IR_MODF   (1 << 1)    /* Mode fault */
#define  IR_RXOF   (1)         /* Rx FIFO overflow */
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

#define CSPI_LINE_CLK_MIO_DEFAULT  50000000
#define CSPI_LINE_CLK_EMIO_DEFAULT 25000000
#define CSPI_LINE_CLK_MIO_MAX  50000000
#define CSPI_LINE_CLK_EMIO_MAX 25000000
#define CSPI_LINE_CLK_DEFAULT  CSPI_LINE_CLK_MIO_DEFAULT
#define CSPI_REF_CLK_DEFAULT 100000000   /* 100 Mhz */

static struct ofw_compat_data spi_matches[] = {
	/* found in vendor Linux upstreamed DTS files */
	{ "xlnx,zynq-spi-r1p6" , true }, 
	{ "cdns,spi-r1p6"      , true },   
	/* found in FreeBSD zedboard DTS file */
	{ "cadence,spi"        , true },
	{ NULL                 , false }
};

/* SPI clocks have been initialised */
static bool clocks_done = false;

/* Completion status of an SPI transfer */
enum proto_xfer_state {
	/* Contoller is available to begin a new transfer. */
	xfer_state_idle,

	/* Transmission of tx fifo is in progress */
	xfer_state_tx_progress,

	/* Waiting on the rx fifo to contain the complete response */
	xfer_state_awaiting_rx,

//  	/* A transfer has been initiated and remains in progress.
//	 *
//	 * Slave select line is automatically asserted/de-asserted
//	 * during transfer, and tranfer is automatically started and
//	 * remains in progress while there is data in the txfifo.
//	 * This mode requires the use of tx/rx fifo threshold
//	 * interrupts. */
//	xfer_state_busy,
//
//	/* Transfer has completed, readying for return to idle. */
//	xfer_state_complete
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

struct cspi_softc {
	struct mtx		sc_mtx;
	device_t	 	dev;
	device_t	 	spibus; /* the attached child device */
	struct resource		*mem_res;
	struct resource		*irq_res;
	void			*ih;
	int			spi_clk_freq;
	struct xfer_control	pcb;
};


static void
dump_registers(struct cspi_softc *sc)
{
	CSPI_ASSERT_LOCKED(sc);
	int regs[] = { SPI_CR, SPI_IMR, SPI_ER, SPI_DR, SPI_TXTHLD, SPI_RXTHLD,
	    SPI_MODID };
	for (int i=0; i < nitems(regs); ++i)
		device_printf(sc->dev, "reg[0x%02x] = 0x%08x\n",
		    regs[i], RD4(sc, regs[i]));
}

static void
cspi_modifyreg(struct cspi_softc *sc, uint32_t off, uint32_t mask,
    uint32_t value)
{
	CSPI_ASSERT_LOCKED(sc);
	uint32_t reg = RD4(sc, off);
	reg &= ~mask;
	reg |= value;
	WR4(sc, off, reg);
}

static bool
xfer_in_error(struct cspi_softc *sc)
{
	CSPI_ASSERT_LOCKED(sc);
	struct xfer_control *pcb = &sc->pcb;
	return pcb->err_msg;
}

static const char *
xfer_get_error_message(struct cspi_softc *sc)
{
	CSPI_ASSERT_LOCKED(sc);
	struct xfer_control *pcb = &sc->pcb;
	return pcb->err_msg;
}

static void
xfer_set_error_message(struct cspi_softc *sc, const char *err_msg)
{
	CSPI_ASSERT_LOCKED(sc);
	struct xfer_control *pcb = &sc->pcb;
	pcb->err_msg = err_msg;
}


static bool
xfer_is_busy(struct cspi_softc *sc)
{
	CSPI_ASSERT_LOCKED(sc);
	struct xfer_control *pcb = &sc->pcb;
	return pcb->xfer_state != xfer_state_idle;
}

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

static void
cspi_assert_slave(struct cspi_softc *sc, int cs)
{
	CSPI_ASSERT_LOCKED(sc);
	unsigned cs_field;
	switch(cs){
	case 0:
		cs_field = 0xe;
		break;
	case 1:
		cs_field = 0xd;
		break;
	case 2:
		cs_field = 0xb;
		break;
	default:
		cs_field = 0xf;
	}
//GA device_printf(sc->dev, "**GA cspi_assert_slave %d\n", cs);
	cspi_modifyreg(sc, SPI_CR, CR_CSL_MASK, (cs_field << CR_CSL_SHIFT));
}

static void
cspi_enable_controller(struct cspi_softc *sc)
{
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	cspi_modifyreg(sc, SPI_ER, ER_MASK, ER_ENABLE);
}

static int
cspi_fill_txfifo(struct cspi_softc *sc, const char *begin, int how_many)
{
	CSPI_ASSERT_LOCKED(sc);
	/* Don't overflow the depth of the FIFO */
	const int depth = (how_many < FIFO_DEPTH) ? how_many : FIFO_DEPTH;
//GA device_printf(sc->dev, "**GA %s to depth of %d bytes\n", __func__, depth);
	for (int i=0; i < depth; ++i)
		WR4(sc, SPI_TXD, begin[i]);
	return depth;
}

static bool
cspi_drain_rxfifo(struct cspi_softc *sc)
{
	CSPI_ASSERT_LOCKED(sc);
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
	for (int i=0; i < to_read; ++i) {
		uint32_t d = RD4(sc, SPI_RXD);
		if (i < to_store) 
			*location++ = d;
	}
	pcb->xfer_read += to_read;
	return result;
}

static void
cspi_set_rxfifo_threshold(struct cspi_softc *sc, int how_many)
{
	CSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s to %d bytes\n", __func__, how_many);
	WR4(sc, SPI_RXTHLD, how_many);
}

static void
cspi_enable_interrupts(struct cspi_softc *sc)
{
	CSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	const uint32_t intr_flags =
	    IR_TXUF | IR_RXOF | IR_MODF /* error conditions */
	    | IR_TXNOTFULL /* tx fifo has been put out on the wire */
;//GA	    | IR_RXNE;  /* rx fifo contains the entire response */
	WR4(sc, SPI_IER, intr_flags);
}

//GA static void
//GA cspi_enable_interrupt(struct cspi_softc *sc, unsigned intr_flags)
//GA {
//GA 	CSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA cspi_enable_interrupt, flags 0x%02x\n", intr_flags);
//GA 	WR4(sc, SPI_IER, intr_flags);
//GA }

static void
cspi_start_transfer(struct cspi_softc *sc)
{
	cspi_enable_interrupts(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
}


static void
cspi_disable_interrupts(struct cspi_softc *sc, unsigned intr_flags)
{
	CSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s, flags 0x%02x\n", __func__, intr_flags);
	WR4(sc, SPI_IDR, intr_flags);
}

static void
cspi_disable_controller(struct cspi_softc *sc)
{
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	cspi_modifyreg(sc, SPI_ER, ER_MASK, 0);
}

static void
cspi_deassert_slaves(struct cspi_softc *sc)
{
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	cspi_modifyreg(sc, SPI_CR, CR_CSL_MASK, CR_CSL_NONE);
}

static void
cspi_clear_interrupts(struct cspi_softc *sc, unsigned intr_flags)
{
	CSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s, flags 0x%02x\n", __func__, intr_flags);
	WR4(sc, SPI_ISR, intr_flags);
}

static uint32_t
cspi_read_interrupt_status(struct cspi_softc *sc)
{
	CSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	return RD4(sc, SPI_ISR);
}

static void xfer_move_to_idle(struct cspi_softc *sc);

static void
xfer_move_to_idle(struct cspi_softc *sc)
{
	CSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	struct xfer_control *pcb = &sc->pcb;
	pcb->xfer_state = xfer_state_idle;
	pcb->xfer_cmd = NULL;
	pcb->xfer_size = pcb->xfer_read = pcb->xfer_written = 0;
 	pcb->err_msg = NULL;
}

static void
xfer_move_to_tx_progress(struct cspi_softc *sc)
{
	CSPI_ASSERT_LOCKED(sc);
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
	const int queued = cspi_fill_txfifo(sc, start, how_many);
//GA	cspi_set_rxfifo_threshold(sc, queued);
	pcb->xfer_written += queued;
	cspi_start_transfer(sc);
}

static void
xfer_move_to_awaiting_rx(struct cspi_softc *sc)
{
	CSPI_ASSERT_LOCKED(sc);
//GA device_printf(sc->dev, "**GA %s\n", __func__);
	struct xfer_control *pcb = &sc->pcb;
	KASSERT(pcb->xfer_state == xfer_state_tx_progress,
	    ("%s: transition to awaiting_rx is only permitted from "
	     "from tx_progress", __func__));
	pcb->xfer_state = xfer_state_awaiting_rx;
}

static void
cspi_intr(void *arg)
{
	struct cspi_softc *sc = (struct cspi_softc *)arg;
	bool done = false;
	CSPI_LOCK(sc);

	
	uint32_t stat = cspi_read_interrupt_status(sc);
//GA device_printf(sc->dev, "**GA %s status 0x%08x\n", __func__, stat);

	/* Ignore the interrupt if we're not performing a transfer */
	if (!xfer_is_busy(sc)) {
//GA device_printf(sc->dev, "**GA %s - interrupt ignored\n", __func__);
		cspi_disable_interrupts(sc, IR_ALL);
		cspi_clear_interrupts(sc, IR_ALL);
		CSPI_UNLOCK(sc);
		return;
	}

	if ((stat & IR_TXNOTFULL) && !xfer_in_error(sc)) {
//GA device_printf(sc->dev, "**GA %s - IR_TXNOTFULL\n", __func__);
		cspi_disable_interrupts(sc, IR_TXNOTFULL);
		cspi_clear_interrupts(sc, IR_TXNOTFULL);
//GA		xfer_move_to_awaiting_rx(sc);
//GA	}
//GA	if ((stat & IR_RXNE) && !xfer_in_error(sc)
//GA	    && xfer_is_awaiting_rx(sc)) {
//GAdevice_printf(sc->dev, "**GA %s - IR_RXNE\n", __func__);
//GA		cspi_disable_interrupts(sc, IR_RXNE);
//GA		cspi_clear_interrupts(sc, IR_RXNE);
		/* all rx data has arrived. */
		bool ok = cspi_drain_rxfifo(sc);
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
	if ((stat & IR_MODF) && !xfer_in_error(sc))
		xfer_set_error_message(sc, "Mode fault occurred");

	if (done || xfer_in_error(sc)) {
//GA		cspi_end_transfer(sc);
		/* awake the requester of this completed transfer */
		wakeup(sc->dev);
	}

	CSPI_UNLOCK(sc);
}

static int
cspi_transfer(device_t dev, device_t child, struct spi_command *cmd)
{
	uint32_t cs;
	struct cspi_softc *sc = device_get_softc(dev);

	if (cmd->tx_cmd_sz != cmd->rx_cmd_sz
	    || cmd->tx_data_sz != cmd->rx_data_sz) {
		device_printf(dev,
		    "Tx/Rx command(%u/%u) and/or data(%u/%u) "
		    "buffer size mismatch\n",
		    cmd->tx_cmd_sz, cmd->rx_cmd_sz,
		    cmd->tx_data_sz, cmd->rx_data_sz);
		return EIO;
	}

	/* get the proper chip select */
	spibus_get_cs(child, &cs);
	cs &= ~SPIBUS_CS_HIGH;

	if (cs > 2) {
		device_printf(dev,
		    "Invalid chip select %d requested by %s\n", cs,
		    device_get_nameunit(child));
		return EINVAL;
	}

	CSPI_LOCK(sc);

	/* A new transfer requester shall wait for any outstanding 
	 * transfer to complete before proceeding.
	 */
	  while (xfer_is_busy(sc))
		mtx_sleep(dev, &sc->sc_mtx, 0, "cspi - pending", 0);

	/* Now it's our turn. */
	sc->pcb.xfer_cmd = cmd;
	sc->pcb.xfer_size = (int)cmd->tx_cmd_sz + (int)cmd->tx_data_sz;
//GA dump_registers(sc);
	cspi_assert_slave(sc, cs);
	cspi_enable_controller(sc);
	xfer_move_to_tx_progress(sc);
//GA dump_registers(sc);
	/* Sleep Waiting for the trasfer to complete, or timeout */
	int err = mtx_sleep(dev, &sc->sc_mtx, 0, "cspi - inprogress", hz * 2);

	/* Check for transfer timeout. */
	if (err == EWOULDBLOCK) {
		if (!xfer_in_error(sc))
			xfer_set_error_message(sc, "SPI transfer timeout");
	}

	/* Report any error */
	if (xfer_in_error(sc)) {
		device_printf(dev, "%s\n", xfer_get_error_message(sc));
		err = EIO;
	}

	cspi_disable_interrupts(sc, IR_ALL);
	cspi_clear_interrupts(sc, IR_ALL);
	cspi_disable_controller(sc);
	cspi_deassert_slaves(sc);
/* TODO also clear tx/rx fifo incase of aborted transfer */
	xfer_move_to_idle(sc);
	
	/* Awake the next requester which would have slept waiting for
	 * this transfer to complete.
	 */
	wakeup_one(dev);
//GA dump_registers(sc);
	CSPI_UNLOCK(sc);

	return err;
}

static int
cspi_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return ENXIO;

	if (!ofw_bus_search_compatible(dev, spi_matches)->ocd_data)
		return ENXIO;

	device_set_desc(dev, "Cadence SPI controller");
	return BUS_PROBE_DEFAULT;
}

static void
cspi_softc_init(struct cspi_softc *sc, device_t dev)
{
	sc->dev = dev;
	CSPI_LOCK_INIT(sc);
	sc->mem_res = NULL;
	sc->irq_res = NULL;
	sc->ih = NULL;
	sc->spibus = NULL;
	sc->spi_clk_freq = CSPI_LINE_CLK_DEFAULT;
	CSPI_LOCK(sc);
	xfer_move_to_idle(sc);
	CSPI_UNLOCK(sc);
}

static uint8_t
divisor_as_field(int divisor)
{
	switch (divisor) {
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
	struct cspi_softc *sc = device_get_softc(dev);
	int ref_freq;
	int error = cspi_get_ref_clk_freq(&ref_freq);
	if (error)
		return error;

/*  TODO - verfiy that the spi frequency is valid for the case
 * of MIO or EMIO. */
/* Whether the SPI is routed to MIO or EMIO pins will determine what the max
 * line rate can be.
 * For MIO its 50 MHz, while for EMIO it's 25 MHz
 */
	/* Find the dividor that when applied to the SPI reference
	 * clock, will give an SPI line rate <= the desired SPI signal
         * clock frequency.*/
	int divisor = CR_BR_DIV_MIN; 
	while (divisor <= CR_BR_DIV_MAX){
		if (ref_freq/divisor > sc->spi_clk_freq)
			divisor *= 2;   /* divisor not large enough yet */
		else
			break;
	}

	if( divisor > CR_BR_DIV_MAX )
		return error;

	int reg = 0;
	reg |= CR_MFG;
//GA	reg |= CR_MSE;
	reg |= CR_MCS;
	reg |= CR_CSL_NONE; /* no slaves selected at this time */
	reg |= divisor_as_field(divisor);
	reg |= CR_CLK_PH_FALLING;
	reg |= CR_MODE_MASTER; /* Only master mode is supported */
	WR4(sc, SPI_CR, reg);

	if (bootverbose)
		device_printf(dev, "master clock frequency %dHz\n",
		    ref_freq / divisor);

	return 0;
}


static int
reset_unit(int unit)
{
	return cspi_clk_reset(unit);
}

static int
init_clocks()
{
	int error = cspi_set_ref_clk_source(zy7_clk_src_iopll);
	if (!error)
		return cspi_set_ref_clk_freq(CSPI_REF_CLK_DEFAULT);
	return error;
}

static int cspi_detach(device_t dev);

static bool
cspi_map_spi0_pins()
{
	return 
	    zy7_mio_set_pin_register(28, 0x22a0) /* serial clock */
	    && zy7_mio_set_pin_register(29, 0x02A0) /* Master In Slave Out */
	    && zy7_mio_set_pin_register(30, 0x32A0) /* slave select 0 */
	    && zy7_mio_set_pin_register(31, 0x32A0) /* slave select 1 */
	    && zy7_mio_set_pin_register(32, 0x32A0) /* slave select 2 */
	    && zy7_mio_set_pin_register(33, 0x22A0) /* Master Out Slave In */
	;
}

static bool
cspi_map_spi1_pins()
{
	return 
	    zy7_mio_set_pin_register(34, 0x22a0) /* serial clock */
	    && zy7_mio_set_pin_register(35, 0x02A0) /* Master In Slave Out */
	    && zy7_mio_set_pin_register(36, 0x32A0) /* slave select 0 */
	    && zy7_mio_set_pin_register(37, 0x32A0) /* slave select 1 */
	    && zy7_mio_set_pin_register(38, 0x32A0) /* slave select 2 */
	    && zy7_mio_set_pin_register(39, 0x22A0) /* Master Out Slave In */
	;
}

static int
cspi_attach(device_t dev)
{
	int error;

	/* Only do once on the first instance attachment, as setting
	 * apply to all instances.
	 */
	if (!clocks_done) {
		/* SPI 0 and 1 occupy the MIO pin region 28-39. */
/*		if (!zy7_mio_unmap_pin_range(28,39)
		    || !cspi_map_spi0_pins()
		    || !cspi_map_spi1_pins()) {
			device_printf(dev, "MIO routing failed\n");
			return ENXIO;
		}*/

		error = init_clocks();
		if (error) {
			device_printf(dev, "clock init failed\n");
			return ENXIO;
		}
		clocks_done = true;
		if (bootverbose) {
			int ref_freq = 0;
			enum zy7_clk_src ref_source;
			(void)cspi_get_ref_clk_freq(&ref_freq);
			(void)cspi_get_ref_clk_source(&ref_source);
			device_printf(dev, "reference clock source %s\n",
			    zy7_clk_src_as_string(ref_source));
			device_printf(dev, "reference clock frequency %dHz\n",
			    ref_freq);
			zy7_dump_mio_pin_control_registers();
		}
	}

        /* Reset this controller instance */
	error = reset_unit(device_get_unit(dev));
	if (error) {
		device_printf(dev, "hardware reset failed\n");
		return ENXIO;
	}

	struct cspi_softc *sc = device_get_softc(dev);

	/* Initialise software context parameters. */
	cspi_softc_init(sc, dev);

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
	    NULL, cspi_intr, sc, &sc->ih)) {
		device_printf(dev, "cannot setup the interrupt handler\n");
		goto err;
	}

	/* SPI clock freq may be overridden via DTB */
	pcell_t cell;
	phandle_t node = ofw_bus_get_node(dev);
	if (OF_getprop(node, "spi-clock", &cell, sizeof(cell)) > 0)
		sc->spi_clk_freq = fdt32_to_cpu(cell);

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
	cspi_detach(dev);
	return ENXIO;
	
}

static int
cspi_detach(device_t dev)
{
	struct cspi_softc *sc = device_get_softc(dev);

	CSPI_LOCK(sc);

	/* Place the controller into a sane state.
	 * - interrupts disabled.
	 * - interrupt status register cleared
	 * - reset any tx/rx fifo threshold levels
	 * - tx/rx fifo queues cleared
	 */
	(void) reset_unit(device_get_unit(dev));

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

	CSPI_UNLOCK(sc);       /* FIXME theoretical race here between UNLOCK */
	CSPI_LOCK_DESTROY(sc); /* and DESTROY. */
	return bus_generic_detach(dev);
}

static int
cspi_driver_load(module_t mod, int what, void *arg)
{
	switch (what) {
	case MOD_LOAD:
		uprintf("cspi: loaded.\n");
		break;
	case MOD_UNLOAD:
		uprintf("cspi: unloaded.\n");
		break;
	case MOD_SHUTDOWN:
		uprintf("cspi: system shutting down.\n");
		break;
	case MOD_QUIESCE:
		uprintf("cspi: unload coming.\n");
		break;
	}
	return (0);
}

static device_method_t cspi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		cspi_probe),
	DEVMETHOD(device_attach,	cspi_attach),
	DEVMETHOD(device_detach,	cspi_detach),
	/* SPI interface */
	DEVMETHOD(spibus_transfer,	cspi_transfer),
	/* end sentinel */
	DEVMETHOD_END
};

static driver_t cspi_driver = {
	"spi",
	cspi_methods,
	sizeof(struct cspi_softc),
};

static devclass_t cspi_devclass;

DRIVER_MODULE(cspi, simplebus, cspi_driver, cspi_devclass,
    cspi_driver_load, NULL);
MODULE_VERSION(cspi, 1);
