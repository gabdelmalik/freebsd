/*-
 * Copyright (c) 2017 George Abd-El-Malik <gabdelmalik@fork.id.au>
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
 */

//#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/conf.h>

//#include <sys/malloc.h>
//#include <sys/rman.h>
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
#include <machine/cpu.h>
#include <machine/intr.h>

#define	RD4(_sc, _reg)	\
	bus_space_read_4(_sc->bst, _sc->bsh, _reg)
#define	WR4(_sc, _reg, _val)	\
	bus_space_write_4(_sc->bst, _sc->bsh, _reg, _val)

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
#define  CR_BR_DIV_MASK (7 << 3)  /* mask baud rate divisor bits*/
   /* Exactly one of these shall be specified.
	* reg &= ~CR_BR_DIV_MASK;
	* reg |= CR_BR_DIV_4;
    */	
#define   CR_BR_DIV_4   (1 << 3)  /* divisor of 4 */
#define   CR_BR_DIV_8   (2 << 3)  /* " " 8 */
#define   CR_BR_DIV_16  (3 << 3)  /* " " 16 */
#define   CR_BR_DIV_32  (4 << 3)  /* " " 32 */
#define   CR_BR_DIV_64  (5 << 3)  /* " " 64 */
#define   CR_BR_DIV_128 (6 << 3)  /* " " 128 */
#define   CR_BR_DIV_256 (7 << 3)  /* " " 256 */
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

struct spi_softc {
	struct resource		*res;
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	void			*ih;
};

static struct resource_spec spi_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, 0 }
};

static struct ofw_compat_data spi_matches[] = {
	{ "xlnx,zy7_spi"       , 1 },
	{ "xlnx,zynq-spi-r1p6" , 1 },
	{ "cdns,spi-r1p6"      , 1 },
	{ NULL                 , 0 }
};

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

static void
set_clocks(struct spi_softc *sc)
{
}

static void
set_interrupts(struct spi_softc *sc)
{
}

#define SPI0_REF_RST_SHIFT 2
#define SPI0_CPU1X_RST_SHIFT 0
static void
spi_reset(struct spi_softc *sc)
{
	/* ABP and PS interface reset */
	WR4(sc, ZY7_SLCR_SPI_RST_CTRL,
	   (1 << SPI0_REF_RST_SHIFT) | (1 << SPI0_CPU1X_RST_SHIFT));
	DELAY(1000);
	WR4(sc, ZY7_SLCR_SPI_RST_CTRL, 0);
}

#define SPI_CLK_DIVISOR_SHIFT  8
#define SPI_CLK_DIVISOR_10     0x0a 
#define SPI_CLK_SRCSEL_SHIFT   4
#define SPI_CLK_SRCSEL_IOPLL   0x1
#define SPI_CLK_ACT0           1
static void
program_clocks(struct spi_softc *sc)
{
	/* SPI_Ref_Clk to 100 MHz, Assumes I/O PLL is at 1,000 MHz.
	 * SPI_Ref_Clk must be >= CPU_1x clock frequency.
	 * TODO who to read the values of I/O PLL and CPU_1x frequencies
	 * to assert that this condition holds.
	 */
	WR4(sc, ZY7_SLCR_SPI_CLK_CTRL,
	   (SPI_CLK_DIVISOR_10 << SPI_CLK_DIVISOR_SHIFT) 
	      | (SPI_CLK_SRCSEL_IOPLL << SPI_CLK_SRCSEL_SHIFT)
	      | (SPI_CLK_ACT0));
}

static void
config_spi(struct spi_softc *sc)
{
	int reg = 0;
	reg |= CR_MFG;
	reg |= CR_CSL_NONE;
	reg |= CR_BR_DIV_4; /* to give a SCLK of 25 MHz, assuming SPI_Ref_Clk was */
	                    /* 100MHz */
	reg |= CR_CLK_PH_INACT;
	reg |= CR_CLK_POL_HIGH;
	reg |= CR_MODE_MASTER;
	WR4(sc, SPI_CR, reg);
}

static void
hw_init(struct spi_softc *sc)
{
	spi_reset(sc);
	program_clocks(sc);
	config_spi(sc);
}

static int
spi_attach(device_t dev)
{
	struct spi_softc *sc = device_get_softc(dev);

	if (bus_alloc_resources(dev, spi_spec, &sc->res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	/* Memory interface */
	sc->bst = rman_get_bustag(sc->res);
	sc->bsh = rman_get_bushandle(sc->res);

    hw_init(sc);

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
