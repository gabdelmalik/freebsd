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
//#include <sys/timeet.h>
//#include <sys/timetc.h>
//#include <sys/watchdog.h>

#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>

#include "spibus_if.h"

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/xilinx/zy7_slcr.h>

#include <machine/bus.h>
//#include <machine/cpu.h>
//#include <machine/intr.h>

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


#define FIFO_DEPTH 128 /* byte depth of Rx and Tx FIFO queues */

#define SPI_CR  	0x00 /* Configuration register */
#define  CR_MFG  (1 << 17)       /* ModeFail generation enable */
#define  CR_MSTC (1 << 16)       /* Manual start command, 0 means auto */
#define  CR_MSE  (1 << 15)       /* Manual start enable, 0 means auto */
#define  CR_MCS  (1 << 14)       /* Manual chip-select enable, 0 means auto */
#define  CR_CSL_NONE (0xf << 10) /* No slave selected */
#define  CR_CSL_MASK CR_CSL_NONE /* mask slave select bits */
   /* At most one of these chip-select line values, and it MUST be ANDed
    * since active bit is '0' (zero), e.g.
	* reg |= CR_CSL_MASK;
	* reg &= CR_CLS_1;
    */	
#define   CR_CSL_0	(~(1 << 10))  /* select slave 0 */
#define   CR_CSL_1	(~(1 << 11))  /* select slave 1 */
#define   CR_CSL_2	(~(1 << 12))  /* select slave 2 */
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
#define  CR_CLK_PH_INACT (1 << 2) /* Clock phase inactive outside SPI word, */
                                  /* otherwise it will be active, */
#define  CR_CLK_POL_HIGH (1 << 1) /* Clock polarity is high outside SPI word */
								  /* otherwise it will be low. */
#define  CR_MODE_MASTER (1)  /* Is SPI Master, otherwise slave */
#define SPI_ISR 	0x04 /* Interrupt status register */
#define  ISR_MASK   (  0xef)    /* ISR bits mask */
#define  ISR_TXUF   (1 << 6)    /* Tx FIFO underflow detected*/
#define  ISR_RXFULL (1 << 5)    /* Rx FIFO is full */
#define  ISR_RXNE   (1 << 4)    /* Rx FIFO not empty, or == lower threshold */
#define  ISR_TXFULL (1 << 3)    /* Tx FIFO is full*/
#define  ISR_TXNOTFULL (1 << 2) /* Tx FIFO is < threshold */
#define  ISR_MODF   (1 << 1)    /* Mode fault detected */
#define  ISR_RXOF   (1)         /* Rx FIFO overflow detected */
#define SPI_IER 	0x08 /* Interrupt enable register */
#define  IER_MASK   ISR_MASK    /* IER bits mask */
#define  IER_TXUF   (1 << 6)    /* Enable Tx FIFO underflow  interrupt */
#define  IER_RXFULL (1 << 5)    /* Enable Rx FIFO is full interrupt */
#define  IER_RXNE   (1 << 4)    /* Enable Rx FIFO not empty, */
							    /* or == lower threshold interrupt */
#define  IER_TXFULL (1 << 3)    /* Enable Tx FIFO is full interrupt */
#define  IER_TXNOTFULL (1 << 2) /* Enable Tx FIFO is < threshold interrupt */
#define  IER_MODF   (1 << 1)    /* Enable Mode fault interrupt */
#define  IER_RXOF   (1)         /* Enable FIFO overflow interrupt */
#define SPI_IDR 	0x0c /* Interrupt disable register */
#define  IDR_MASK   ISR_MASK    /* IDR bits mask */
#define  IDR_TXUF   (1 << 6)    /* Disable Tx FIFO underflow  interrupt */
#define  IDR_RXFULL (1 << 5)    /* Disable Rx FIFO is full interrupt */
#define  IDR_RXNE   (1 << 4)    /* Disable Rx FIFO not empty, */
							    /* or == lower threshold interrupt */
#define  IDR_TXFULL (1 << 3)    /* Disable Tx FIFO is full interrupt */
#define  IDR_TXNOTFULL (1 << 2) /* Disable Tx FIFO is < threshold interrupt */
#define  IDR_MODF   (1 << 1)    /* Disable Mode fault interrupt */
#define  IDR_RXOF   (1)         /* Disable FIFO overflow interrupt */
#define SPI_IMR 	0x10 /* Interrupt mask register */
#define  IMR_MASK   ISR_MASK    /* IMR bits mask */
#define  IMR_TXUF   (1 << 6)    /* Int. Tx FIFO underflow is disabled */
#define  IMR_RXFULL (1 << 5)    /* Int. Rx FIFO is full is disabled */
#define  IMR_RXNE   (1 << 4)    /* Int. Rx FIFO not empty, */
							    /* or == lower threshold is disabled */
#define  IMR_TXFULL (1 << 3)    /* Int. Tx FIFO is full is disabled */
#define  IMR_TXNOTFULL (1 << 2) /* Int. Tx FIFO is < threshold is disabled */
#define  IMR_MODF   (1 << 1)    /* Int. Mode fault is disabled */
#define  IMR_RXOF   (1)         /* Int. FIFO overflow is disabled */
#define SPI_ER  	0x14 /* SPI enable register */
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
#define  TXTHLD_RESET_VAL  1 /* */
#define  TXTHLD_MIN_VAL    TXTHLD_RESET_VAL
#define  TXTHLD_MAX_VAL    (FIFO_DEPTH - 1)
#define SPI_RXTHLD	0x2C /* Rx FIFO threshold register. Defines when the */
                         /* RXNE interrupt is triggered */
#define SPI_MODID	0x2C /* Module ID register */
#define  MODID_MASK  (0xffffff)

enum master_xfer_mode {
	/* Manual assertion/de-assertion of slave select line,
	 * and manual start of data transfer.
	 * This mode is the default. */
	xfer_manual,
  	/* Slave select line is automatically asserted/de-asserted
	 * during transfer, and tranfer is automatically started and
	 * remains in progress while there is data in the txfifo.
	 * This mode requires the use of tx/rx fifo threshold
	 * interrupts. */
	xfer_automatic
};

struct spi_softc {
	struct resource		*mem_res;
	void			*ih;
	int			spi_clk_freq;
	enum master_xfer_mode	xfer_mode;
};

static struct ofw_compat_data spi_matches[] = {
	/* found in vendor Linux upstreamed DTS files */
	{ "xlnx,zynq-spi-r1p6" , 1 }, 
	{ "cdns,spi-r1p6"      , 1 },   
	/* found in FreeBSD zedboard DTS file */
	{ "cadence,spi"        , 1 },
	{ NULL                 , 0 }
};

/* SPI clocks have been initialised */
static bool clocks_done = false;

//static void
//set_interrupts(struct spi_softc *sc)
//{
//}

#define CSPI_LINE_CLK_MIO_DEFAULT  50000000
#define CSPI_LINE_CLK_EMIO_DEFAULT 25000000
#define CSPI_LINE_CLK_MIO_MAX  50000000
#define CSPI_LINE_CLK_EMIO_MAX 25000000
#define CSPI_LINE_CLK_DEFAULT  CSPI_LINE_CLK_MIO_DEFAULT

static int
config_spi(device_t dev)
{
	int ref_freq;
	int error = cspi_get_ref_clk_freq(&ref_freq);
	if (error)
		return error;

	struct spi_softc *sc = device_get_softc(dev);
	pcell_t cell;
	phandle_t node = ofw_bus_get_node(dev);

	/* SPI clock freq may be overridden via DTB */
	if (OF_getprop(node, "spi-clock", &cell, sizeof(cell)) > 0)
		sc->spi_clk_freq = fdt32_to_cpu(cell);

	/* SPI transfer mode may be overridden via DTB */
	char xfer_mode[32] = {0};
	if (OF_getprop(node, "spi-xfer-mode", &xfer_mode,
	    sizeof(xfer_mode)) > 0
	    && strncmp(xfer_mode, "automatic", 9) == 0)
		sc->xfer_mode = xfer_automatic;

	/* FIXME : automatic transfer isn't yet suppported.
	 * Revert to manual, and tell the operator about it. 
	 */
	if (sc->xfer_mode == xfer_automatic) {
		device_printf(dev,
		    "automatic transfer mode is not supported at this time, "
		    "falling back to manual mode.\n");
		sc->xfer_mode = xfer_manual;
	}

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
	reg |= CR_CSL_NONE;
	reg |= ((divisor >> 1) << CR_BR_DIV_SHIFT);
	reg |= CR_CLK_PH_INACT;
	reg |= CR_CLK_POL_HIGH;
	reg |= CR_MODE_MASTER;
	WR4(sc, SPI_CR, reg);
	BW(sc); /* ensure config is written before any later data transfer
	         * attempts that may occur. */

	if (bootverbose)
		device_printf(dev, "master clock frequency %dHz\n",
		    ref_freq / divisor);

	return 0;
}

 /*TODO - allow override via DTS */
#define CSPI_REF_CLK_DEFAULT 100000000   /* 100 Mhz */

static int
reset_unit(int unit)
{
	int error = cspi_clk_reset(unit);
	if (error)
		return error;

	return 0;
}

static int
init_clocks()
{
	int error = cspi_set_ref_clk_source(zy7_clk_src_iopll);
	if (error)
		return error;

	error = cspi_set_ref_clk_freq(CSPI_REF_CLK_DEFAULT);
	if (error)
		return error;

	return 0;
}

static int
spi_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return ENXIO;

	if (ofw_bus_search_compatible(dev, spi_matches)->ocd_data == 0)
		return ENXIO;

	device_set_desc(dev, "Cadence SPI controller");
	return BUS_PROBE_DEFAULT;
}

static int
spi_attach(device_t dev)
{
	int error;

	/* Only do once on the first instance attachment. */
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
			(void)cspi_get_ref_clk_freq(&ref_freq);
			(void)cspi_get_ref_clk_source(&ref_source);
			device_printf(dev, "reference clock source %s\n",
			    zy7_clk_src_as_string(ref_source));
			device_printf(dev, "reference clock frequency %dHz\n",
			    ref_freq);
		}
	}
        /* Reset is per controller instance */
	error = reset_unit(device_get_unit(dev));
	if (error) {
		device_printf(dev, "reset failed\n");
		return ENXIO;
	}

	struct spi_softc *sc = device_get_softc(dev);

	/* Initialise operating parameters. */
	sc->xfer_mode = xfer_manual;
	sc->spi_clk_freq = CSPI_LINE_CLK_DEFAULT;

	/* acquire a bus presence */
	int rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->mem_res) {
		device_printf(dev, "could not allocate resources\n");
		return ENXIO;
	}

	error = config_spi(dev);
	if (error) {
		bus_release_resource(dev,SYS_RES_MEMORY,
		    rman_get_rid(sc->mem_res), sc->mem_res);
		device_printf(dev, "config failed\n");
		return ENXIO;
	}

	device_add_child(dev, "spibus", -1);
	return bus_generic_attach(dev);
}

static int
spi_txrx(struct spi_softc *sc, uint8_t *out_buf,
    uint8_t *in_buf, int bufsz, int cs)
{
	uprintf("SPI transfer: dry-run of %d bytes.\n", bufsz);
#if 0
	uint32_t data;
	uint32_t i;

	for (i = 0; i < bufsz; i++) {
		WR4(sc, SPI_DTR, out_buf[i]);

		while(!(RD4(sc, SPI_SR) & SR_TX_EMPTY))
			continue;

		data = RD4(sc, SPI_DRR);
		if (in_buf)
			in_buf[i] = (data & 0xff);
	}
#endif
	return 0;
}

static int
spi_transfer(device_t dev, device_t child, struct spi_command *cmd)
{
	uint32_t cs;
	struct spi_softc *sc = device_get_softc(dev);
#if 0

	KASSERT(cmd->tx_cmd_sz == cmd->rx_cmd_sz,
	    ("%s: TX/RX command sizes should be equal", __func__));
	KASSERT(cmd->tx_data_sz == cmd->rx_data_sz,
	    ("%s: TX/RX data sizes should be equal", __func__));
#endif

	/* get the proper chip select */
	spibus_get_cs(child, &cs);

#if 0
	/* Assert CS */
	uint32_t reg = RD4(sc, SPI_SSR);
	reg &= ~(1 << cs);
	WR4(sc, SPI_SSR, reg);
#endif

	/* Command */
	spi_txrx(sc, cmd->tx_cmd, cmd->rx_cmd, cmd->tx_cmd_sz, cs);

	/* Data */
	spi_txrx(sc, cmd->tx_data, cmd->rx_data, cmd->tx_data_sz, cs);

#if 0
	/* Deassert CS */
	reg = RD4(sc, SPI_SSR);
	reg |= (1 << cs);
	WR4(sc, SPI_SSR, reg);
#endif
	return (0);
}

static int
spi_driver_load(module_t mod, int what, void *arg)
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

static device_method_t spi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		spi_probe),
	DEVMETHOD(device_attach,	spi_attach),
	/* SPI interface */
	DEVMETHOD(spibus_transfer,	spi_transfer),
	/* end sentinel */
	DEVMETHOD_END
};

static driver_t spi_driver = {
	"spi",
	spi_methods,
	sizeof(struct spi_softc),
};

static devclass_t spi_devclass;

DRIVER_MODULE(cspi, simplebus, spi_driver, spi_devclass, spi_driver_load, NULL);
MODULE_VERSION(cspi, 1);
