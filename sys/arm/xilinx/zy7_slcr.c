/*-
 * Copyright (c) 2013 Thomas Skibo
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

/*
 * Zynq-700 SLCR driver.  Provides hooks for cpu_reset and PL control stuff.
 * In the future, maybe MIO control, clock control, etc. could go here.
 *
 * Reference: Zynq-7000 All Programmable SoC Technical Reference Manual.
 * (v1.4) November 16, 2012.  Xilinx doc UG585.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/sysctl.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/stdarg.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/xilinx/zy7_slcr.h>

struct zy7_slcr_softc {
	device_t	dev;
	struct mtx	sc_mtx;
	struct resource	*mem_res;
};

static struct zy7_slcr_softc *zy7_slcr_softc_p;
extern void (*zynq7_cpu_reset);

#define ZSLCR_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define	ZSLCR_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)
#define ZSLCR_LOCK_INIT(sc) \
	mtx_init(&(sc)->sc_mtx, device_get_nameunit((sc)->dev),	\
	    "zy7_slcr", MTX_DEF)
#define ZSLCR_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);

#define RD4(sc, off) 		(bus_read_4((sc)->mem_res, (off)))
#define WR4(sc, off, val) 	(bus_write_4((sc)->mem_res, (off), (val)))

#define ZYNQ_DEFAULT_PS_CLK_FREQUENCY	33333333	/* 33.3 Mhz */

SYSCTL_NODE(_hw, OID_AUTO, zynq, CTLFLAG_RD, 0, "Xilinx Zynq-7000");

static char zynq_bootmode[64];
SYSCTL_STRING(_hw_zynq, OID_AUTO, bootmode, CTLFLAG_RD, zynq_bootmode, 0,
	      "Zynq boot mode");

static char zynq_pssid[100];
SYSCTL_STRING(_hw_zynq, OID_AUTO, pssid, CTLFLAG_RD, zynq_pssid, 0,
	   "Zynq PSS IDCODE");

static uint32_t zynq_reboot_status;
SYSCTL_INT(_hw_zynq, OID_AUTO, reboot_status, CTLFLAG_RD, &zynq_reboot_status,
	   0, "Zynq REBOOT_STATUS register");

static int ps_clk_frequency;
SYSCTL_INT(_hw_zynq, OID_AUTO, ps_clk_frequency, CTLFLAG_RD, &ps_clk_frequency,
	   0, "Zynq PS_CLK Frequency");

static int io_pll_frequency;
SYSCTL_INT(_hw_zynq, OID_AUTO, io_pll_frequency, CTLFLAG_RD, &io_pll_frequency,
	   0, "Zynq IO PLL Frequency");

static int arm_pll_frequency;
SYSCTL_INT(_hw_zynq, OID_AUTO, arm_pll_frequency, CTLFLAG_RD,
	   &arm_pll_frequency, 0, "Zynq ARM PLL Frequency");

static int ddr_pll_frequency;
SYSCTL_INT(_hw_zynq, OID_AUTO, ddr_pll_frequency, CTLFLAG_RD,
	   &ddr_pll_frequency, 0, "Zynq DDR PLL Frequency");

static void
zy7_slcr_unlock(struct zy7_slcr_softc *sc)
{

	/* Unlock SLCR with magic number. */
	WR4(sc, ZY7_SLCR_UNLOCK, ZY7_SLCR_UNLOCK_MAGIC);
}

static void
zy7_slcr_lock(struct zy7_slcr_softc *sc)
{

	/* Lock SLCR with magic number. */
	WR4(sc, ZY7_SLCR_LOCK, ZY7_SLCR_LOCK_MAGIC);
}

static void
zy7_slcr_cpu_reset(void)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;

	/* Unlock SLCR registers. */
	zy7_slcr_unlock(sc);

	/* This has something to do with a work-around so the fsbl will load
	 * the bitstream after soft-reboot.  It's very important.
	 */
	WR4(sc, ZY7_SLCR_REBOOT_STAT,
	    RD4(sc, ZY7_SLCR_REBOOT_STAT) & 0xf0ffffff);

	/* Soft reset */
	WR4(sc, ZY7_SLCR_PSS_RST_CTRL, ZY7_SLCR_PSS_RST_CTRL_SOFT_RESET);

	for (;;)
		;
}

/* Assert PL resets and disable level shifters in preparation of programming
 * the PL (FPGA) section.  Called from zy7_devcfg.c.
 */
void
zy7_slcr_preload_pl(void)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;

	if (!sc)
		return;

	ZSLCR_LOCK(sc);

	/* Unlock SLCR registers. */
	zy7_slcr_unlock(sc);

	/* Assert top level output resets. */
	WR4(sc, ZY7_SLCR_FPGA_RST_CTRL, ZY7_SLCR_FPGA_RST_CTRL_RST_ALL);

	/* Disable all level shifters. */
	WR4(sc, ZY7_SLCR_LVL_SHFTR_EN, 0);

	/* Lock SLCR registers. */
	zy7_slcr_lock(sc);

	ZSLCR_UNLOCK(sc);
}

/* After PL configuration, enable level shifters and deassert top-level
 * PL resets.  Called from zy7_devcfg.c.  Optionally, the level shifters
 * can be left disabled but that's rare of an FPGA application. That option
 * is controlled by a sysctl in the devcfg driver.
 */
void
zy7_slcr_postload_pl(int en_level_shifters)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;

	if (!sc)
		return;

	ZSLCR_LOCK(sc);

	/* Unlock SLCR registers. */
	zy7_slcr_unlock(sc);

	if (en_level_shifters)
		/* Enable level shifters. */
		WR4(sc, ZY7_SLCR_LVL_SHFTR_EN, ZY7_SLCR_LVL_SHFTR_EN_ALL);

	/* Deassert top level output resets. */
	WR4(sc, ZY7_SLCR_FPGA_RST_CTRL, 0);

	/* Lock SLCR registers. */
	zy7_slcr_lock(sc);

	ZSLCR_UNLOCK(sc);
}

/* Override cgem_set_refclk() in gigabit ethernet driver
 * (sys/dev/cadence/if_cgem.c).  This function is called to
 * request a change in the gem's reference clock speed.
 */
int
cgem_set_ref_clk(int unit, int frequency)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	int div0, div1;

	if (!sc)
		return (-1);

	/* Find suitable divisor pairs.  Round result to nearest khz
	 * to test for match.
	 */
	for (div1 = 1; div1 <= ZY7_SLCR_GEM_CLK_CTRL_DIVISOR1_MAX; div1++) {
		div0 = (io_pll_frequency + div1 * frequency / 2) /
			div1 / frequency;
		if (div0 > 0 && div0 <= ZY7_SLCR_GEM_CLK_CTRL_DIVISOR_MAX &&
		    ((io_pll_frequency / div0 / div1) + 500) / 1000 ==
		    (frequency + 500) / 1000)
			break;
	}

	if (div1 > ZY7_SLCR_GEM_CLK_CTRL_DIVISOR1_MAX)
		return (-1);

	ZSLCR_LOCK(sc);

	/* Unlock SLCR registers. */
	zy7_slcr_unlock(sc);

	/* Modify GEM reference clock. */
	WR4(sc, unit ? ZY7_SLCR_GEM1_CLK_CTRL : ZY7_SLCR_GEM0_CLK_CTRL,
	    (div1 << ZY7_SLCR_GEM_CLK_CTRL_DIVISOR1_SHIFT) |
	    (div0 << ZY7_SLCR_GEM_CLK_CTRL_DIVISOR_SHIFT) |
	    ZY7_SLCR_GEM_CLK_CTRL_SRCSEL_IO_PLL |
	    ZY7_SLCR_GEM_CLK_CTRL_CLKACT);

	/* Lock SLCR registers. */
	zy7_slcr_lock(sc);

	ZSLCR_UNLOCK(sc);

	return (0);
}

/* 
 * PL clocks management function
 */
int 
zy7_pl_fclk_set_source(int unit, int source)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	uint32_t reg;

	if (!sc)
		return (-1);

	ZSLCR_LOCK(sc);

	/* Unlock SLCR registers. */
	zy7_slcr_unlock(sc);

	/* Modify FPGAx source. */
	reg = RD4(sc, ZY7_SLCR_FPGA_CLK_CTRL(unit));
	reg &= ~(ZY7_SLCR_FPGA_CLK_CTRL_SRCSEL_MASK);
	reg |= (source << ZY7_SLCR_FPGA_CLK_CTRL_SRCSEL_SHIFT);
	WR4(sc, ZY7_SLCR_FPGA_CLK_CTRL(unit), reg);

	/* Lock SLCR registers. */
	zy7_slcr_lock(sc);

	ZSLCR_UNLOCK(sc);

	return (0);
}

int 
zy7_pl_fclk_get_source(int unit)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	uint32_t reg;
	int source;

	if (!sc)
		return (-1);

	ZSLCR_LOCK(sc);

	/* Modify GEM reference clock. */
	reg = RD4(sc, ZY7_SLCR_FPGA_CLK_CTRL(unit));
	source = (reg & ZY7_SLCR_FPGA_CLK_CTRL_SRCSEL_MASK) >> 
	    ZY7_SLCR_FPGA_CLK_CTRL_SRCSEL_SHIFT;

	/* ZY7_PL_FCLK_SRC_IO is actually b0x */
	if ((source & 2) == 0)
		source = ZY7_PL_FCLK_SRC_IO;

	ZSLCR_UNLOCK(sc);

	return (source);
}

int
zy7_pl_fclk_set_freq(int unit, int frequency)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	int div0, div1;
	int base_frequency;
	uint32_t reg;
	int source;

	if (!sc)
		return (-1);

	source = zy7_pl_fclk_get_source(unit);
	switch (source) {
		case ZY7_PL_FCLK_SRC_IO:
			base_frequency = io_pll_frequency;
			break;

		case ZY7_PL_FCLK_SRC_ARM:
			base_frequency = arm_pll_frequency;
			break;

		case ZY7_PL_FCLK_SRC_DDR:
			base_frequency = ddr_pll_frequency;
			break;

		default:
			return (-1);
	}

	/* Find suitable divisor pairs.  Round result to nearest khz
	 * to test for match.
	 */
	for (div1 = 1; div1 <= ZY7_SLCR_FPGA_CLK_CTRL_DIVISOR_MAX; div1++) {
		div0 = (base_frequency + div1 * frequency / 2) /
			div1 / frequency;
		if (div0 > 0 && div0 <= ZY7_SLCR_FPGA_CLK_CTRL_DIVISOR_MAX &&
		    ((base_frequency / div0 / div1) + 500) / 1000 ==
		    (frequency + 500) / 1000)
			break;
	}

	if (div1 > ZY7_SLCR_FPGA_CLK_CTRL_DIVISOR_MAX)
		return (-1);

	ZSLCR_LOCK(sc);

	/* Unlock SLCR registers. */
	zy7_slcr_unlock(sc);

	/* Modify FPGAx reference clock. */
	reg = RD4(sc, ZY7_SLCR_FPGA_CLK_CTRL(unit));
	reg &= ~(ZY7_SLCR_FPGA_CLK_CTRL_DIVISOR1_MASK |
	    ZY7_SLCR_FPGA_CLK_CTRL_DIVISOR0_MASK);
	reg |= (div1 << ZY7_SLCR_FPGA_CLK_CTRL_DIVISOR1_SHIFT) |
	    (div0 << ZY7_SLCR_FPGA_CLK_CTRL_DIVISOR0_SHIFT);
	WR4(sc, ZY7_SLCR_FPGA_CLK_CTRL(unit), reg);

	/* Lock SLCR registers. */
	zy7_slcr_lock(sc);

	ZSLCR_UNLOCK(sc);

	return (base_frequency / div0 / div1);
}

int
zy7_pl_fclk_get_freq(int unit)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	int div0, div1;
	int base_frequency;
	int frequency;
	uint32_t reg;
	int source;

	if (!sc)
		return (-1);

	source = zy7_pl_fclk_get_source(unit);
	switch (source) {
		case ZY7_PL_FCLK_SRC_IO:
			base_frequency = io_pll_frequency;
			break;

		case ZY7_PL_FCLK_SRC_ARM:
			base_frequency = arm_pll_frequency;
			break;

		case ZY7_PL_FCLK_SRC_DDR:
			base_frequency = ddr_pll_frequency;
			break;

		default:
			return (-1);
	}

	ZSLCR_LOCK(sc);

	/* Modify FPGAx reference clock. */
	reg = RD4(sc, ZY7_SLCR_FPGA_CLK_CTRL(unit));
	div1 = (reg & ZY7_SLCR_FPGA_CLK_CTRL_DIVISOR1_MASK) >>
	    ZY7_SLCR_FPGA_CLK_CTRL_DIVISOR1_SHIFT;
	div0 = (reg & ZY7_SLCR_FPGA_CLK_CTRL_DIVISOR0_MASK) >>
	    ZY7_SLCR_FPGA_CLK_CTRL_DIVISOR0_SHIFT;

	ZSLCR_UNLOCK(sc);

	if (div0 == 0)
		div0 = 1;

	if (div1 == 0)
		div1 = 1;

	frequency = (base_frequency / div0 / div1);
	/* Round to KHz */
	frequency = (frequency + 500) / 1000;
	frequency = frequency * 1000;

	return (frequency);
}

int 
zy7_pl_fclk_enable(int unit)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;

	if (!sc)
		return (-1);

	ZSLCR_LOCK(sc);

	/* Unlock SLCR registers. */
	zy7_slcr_unlock(sc);

	WR4(sc, ZY7_SLCR_FPGA_THR_CTRL(unit), 0);
	WR4(sc, ZY7_SLCR_FPGA_THR_CNT(unit), 0);

	/* Lock SLCR registers. */
	zy7_slcr_lock(sc);

	ZSLCR_UNLOCK(sc);

	return (0);
}

int 
zy7_pl_fclk_disable(int unit)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;

	if (!sc)
		return (-1);

	ZSLCR_LOCK(sc);

	/* Unlock SLCR registers. */
	zy7_slcr_unlock(sc);

	WR4(sc, ZY7_SLCR_FPGA_THR_CTRL(unit), 0);
	WR4(sc, ZY7_SLCR_FPGA_THR_CNT(unit), 1);

	/* Lock SLCR registers. */
	zy7_slcr_lock(sc);

	ZSLCR_UNLOCK(sc);

	return (0);
}

int 
zy7_pl_fclk_enabled(int unit)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	uint32_t reg;

	if (!sc)
		return (-1);

	ZSLCR_LOCK(sc);
	reg = RD4(sc, ZY7_SLCR_FPGA_THR_CNT(unit));
	ZSLCR_UNLOCK(sc);

	return !(reg & 1);
}

int
zy7_pl_level_shifters_enabled(void)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;

	uint32_t reg;

	if (!sc)
		return (-1);

	ZSLCR_LOCK(sc);
	reg = RD4(sc, ZY7_SLCR_LVL_SHFTR_EN);
	ZSLCR_UNLOCK(sc);

	return (reg == ZY7_SLCR_LVL_SHFTR_EN_ALL);
}

void
zy7_pl_level_shifters_enable(void)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;

	if (!sc)
		return;

	ZSLCR_LOCK(sc);
	zy7_slcr_unlock(sc);
	WR4(sc, ZY7_SLCR_LVL_SHFTR_EN, ZY7_SLCR_LVL_SHFTR_EN_ALL);
	zy7_slcr_lock(sc);
	ZSLCR_UNLOCK(sc);
}

void
zy7_pl_level_shifters_disable(void)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;

	if (!sc)
		return;

	ZSLCR_LOCK(sc);
	zy7_slcr_unlock(sc);
	WR4(sc, ZY7_SLCR_LVL_SHFTR_EN, 0);
	zy7_slcr_lock(sc);
	ZSLCR_UNLOCK(sc);
}


/* Reset the interfaces of the SPI controller */
int
cspi_clk_reset(int unit)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	uint32_t reg = unit
	    ? ZY7_SLCR_SPI1_REF_RST | ZY7_SLCR_SPI1_CPU1X_RST
	    : ZY7_SLCR_SPI0_REF_RST | ZY7_SLCR_SPI0_CPU1X_RST;

	if (!sc || unit < 0 || unit > 1)
		return (-1);

	ZSLCR_LOCK(sc);
	zy7_slcr_unlock(sc);

	/* It is required that the reset flags be asserted, then after some
	 * delay be deasserted.
	 */
	WR4(sc, ZY7_SLCR_SPI_RST_CTRL, reg);
	DELAY(1000);
	WR4(sc, ZY7_SLCR_SPI_RST_CTRL, 0);

	zy7_slcr_lock(sc);
	ZSLCR_UNLOCK(sc);

	return (0);
}

int
qspi_clk_reset(void)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;

	ZSLCR_LOCK(sc);
	zy7_slcr_unlock(sc);

	/* It is required that the reset flags be asserted, then after some
	 * delay be deasserted.
	 */
	WR4(sc, ZY7_SLCR_LQSPI_RST_CTRL, ZY7_SLCR_LQSPI_REF_RST | ZY7_SLCR_LQSPI_CPU1X_RST);
	DELAY(1000);
	WR4(sc, ZY7_SLCR_LQSPI_RST_CTRL, 0);

	zy7_slcr_lock(sc);
	ZSLCR_UNLOCK(sc);

	return (0);
}

static int
get_cpu_clk_params(int *src_freq, int *divisor)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	uint32_t reg;

	ZSLCR_LOCK(sc);
	reg = RD4(sc, ZY7_SLCR_ARM_CLK_CTRL);
	ZSLCR_UNLOCK(sc);

	/* Determine which base clock source is being used. */
	if (reg & ZY7_SLCR_ARM_CLK_CTRL_SRCSEL_ARM_PLL) {
		*src_freq = arm_pll_frequency;
	} else if (reg & ZY7_SLCR_ARM_CLK_CTRL_SRCSEL_DDR_PLL) {
		*src_freq = ddr_pll_frequency;
	} else
		*src_freq = io_pll_frequency;

	*divisor = (reg & ZY7_SLCR_ARM_CLK_CTRL_DIVISOR_MASK)
	    >> ZY7_SLCR_ARM_CLK_CTRL_DIVISOR_SHIFT;
	KASSERT(*divisor > 0,
	    ("get_cpu_clk_params: divisor too small"));
	return (0);
}

static int
cpu_freqs(int *cpu1x, int *cpu2x, int *cpu3x2x, int * cpu6x4x)
{
	int src_freq;
	int divisor;
	int constant;
	int err = get_cpu_clk_params(&src_freq, &divisor);
	if (err) return err;
	constant = src_freq/divisor * 1/6;
	if (cpu1x) *cpu1x = constant * 1;
	if (cpu2x) *cpu2x = constant * 2;
	if (cpu3x2x) *cpu3x2x = constant * 3;
	if (cpu6x4x) *cpu6x4x = constant * 6;
	return (0);
}

int
cspi_get_ref_clk_source(enum zy7_clk_src *source)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	uint32_t reg;

	if (!sc)
		return (-1);

	ZSLCR_LOCK(sc);
	reg = RD4(sc, ZY7_SLCR_SPI_CLK_CTRL);
	ZSLCR_UNLOCK(sc);

	switch (reg & ZY7_SLCR_SPI_CLK_SRCSEL_MASK) {
		case ZY7_SLCR_SPI_CLK_SRCSEL_IOPLL :
			*source = zy7_clk_src_iopll;
			break;
		case ZY7_SLCR_SPI_CLK_SRCSEL_ARMPLL:
			*source = zy7_clk_src_armpll;
			break;
		case ZY7_SLCR_SPI_CLK_SRCSEL_DDRPLL:
			*source = zy7_clk_src_ddrpll;
			break;
		default:
			return (-1);
	}
	return (0);
}

int
qspi_get_ref_clk_source(enum zy7_clk_src *source)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	uint32_t reg;

	if (!sc)
		return (-1);

	ZSLCR_LOCK(sc);
	reg = RD4(sc, ZY7_SLCR_LQSPI_CLK_CTRL);
	ZSLCR_UNLOCK(sc);

	switch (reg & ZY7_SLCR_LQSPI_CLK_SRCSEL_MASK) {
		case ZY7_SLCR_LQSPI_CLK_SRCSEL_IOPLL :
			*source = zy7_clk_src_iopll;
			break;
		case ZY7_SLCR_LQSPI_CLK_SRCSEL_ARMPLL:
			*source = zy7_clk_src_armpll;
			break;
		case ZY7_SLCR_LQSPI_CLK_SRCSEL_DDRPLL:
			*source = zy7_clk_src_ddrpll;
			break;
		default:
			return (-1);
	}
	return (0);
}

const char*
 zy7_clk_src_as_string(enum zy7_clk_src source)
{
	const char *result = "???";
	switch (source) {
		case zy7_clk_src_iopll:
			result = "IO PLL";
			break;
		case zy7_clk_src_armpll:
			result = "ARM PLL";
			break;
		case zy7_clk_src_ddrpll:
			result = "DDR PLL";
			break;
	}
	return result;
}

int
cspi_set_ref_clk_source(enum zy7_clk_src source)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	uint32_t reg;
	uint32_t src_sel_flags;

	if (!sc)
		return (-1);

	switch (source) {
		case zy7_clk_src_iopll:
			src_sel_flags = ZY7_SLCR_SPI_CLK_SRCSEL_IOPLL;
			break;
		case zy7_clk_src_armpll:
			src_sel_flags = ZY7_SLCR_SPI_CLK_SRCSEL_ARMPLL;
			break;
		case zy7_clk_src_ddrpll:
			src_sel_flags = ZY7_SLCR_SPI_CLK_SRCSEL_DDRPLL;
			break;
		default:
			return (-1);
	}

	/* Apply the selected source */
	ZSLCR_LOCK(sc);
	reg = RD4(sc, ZY7_SLCR_SPI_CLK_CTRL);
	reg &= ZY7_SLCR_SPI_CLK_SRCSEL_MASK;
	reg |= src_sel_flags;
	reg |= ZY7_SLCR_SPI_CLK_ACT(0); /* As the reference clock is shared, */
	reg |= ZY7_SLCR_SPI_CLK_ACT(1); /* activate both. */
	zy7_slcr_unlock(sc);
	WR4(sc, ZY7_SLCR_SPI_CLK_CTRL, reg);
	zy7_slcr_lock(sc);
	ZSLCR_UNLOCK(sc);

	return (0);
}

int
qspi_set_ref_clk_source(enum zy7_clk_src source)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	uint32_t reg;
	uint32_t src_sel_flags;

	if (!sc)
		return (-1);

	switch (source) {
		case zy7_clk_src_iopll:
			src_sel_flags = ZY7_SLCR_LQSPI_CLK_SRCSEL_IOPLL;
			break;
		case zy7_clk_src_armpll:
			src_sel_flags = ZY7_SLCR_LQSPI_CLK_SRCSEL_ARMPLL;
			break;
		case zy7_clk_src_ddrpll:
			src_sel_flags = ZY7_SLCR_LQSPI_CLK_SRCSEL_DDRPLL;
			break;
		default:
			return (-1);
	}

	/* Apply the selected source */
	ZSLCR_LOCK(sc);
	reg = RD4(sc, ZY7_SLCR_LQSPI_CLK_CTRL);
	reg &= ZY7_SLCR_LQSPI_CLK_SRCSEL_MASK;
	reg |= src_sel_flags;
	zy7_slcr_unlock(sc);
	WR4(sc, ZY7_SLCR_LQSPI_CLK_CTRL, reg);
	zy7_slcr_lock(sc);
	ZSLCR_UNLOCK(sc);

	return (0);
}

static int
freq_of_source(enum zy7_clk_src source, int *freq)
{
	switch (source) {
		case zy7_clk_src_iopll:
			*freq = io_pll_frequency;
			break;
		case zy7_clk_src_armpll:
			*freq = arm_pll_frequency;
			break;
		case zy7_clk_src_ddrpll:
			*freq = ddr_pll_frequency;
			break;
		default:
			return (-1);
	}
	return (0);
}

int
cspi_set_ref_clk_freq(int desired_freq)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	uint32_t reg;
	int divisor;  /* attempt to find which yields desired_freq */
	int spi_ref_freq; /* resultant freq after applying the divisor */
	int cpu1x_freq;
	int err;
	enum zy7_clk_src src_clk;
	int src_freq;

	if (!sc)
		return (-1);

	err = cspi_get_ref_clk_source(&src_clk);
	if (err) return err;
	err = freq_of_source(src_clk, &src_freq);
	if (err) return err;
	err = cpu_freqs(&cpu1x_freq, NULL, NULL, NULL);
	if (err) return err;

	/* Find the divisor which yields a SPI reference frequency
	 * which is closest to, but not greater, than what is desired. */
	divisor = ZY7_SLCR_SPI_CLK_DIVISOR_MIN;
	while (divisor <= ZY7_SLCR_SPI_CLK_DIVISOR_MAX) {
		spi_ref_freq = src_freq / divisor;
		if (spi_ref_freq > desired_freq)
			++divisor; /* freq still too large. keep reaching */
		else
			break; /* divisor found */
	}
	if (divisor > ZY7_SLCR_SPI_CLK_DIVISOR_MAX
	    /* Clocking restrictions require that the SPI reference clock,
	     * SPI_Ref_Clk, be greater than the CPU_1x clock frequency.
	     */
	    || spi_ref_freq <= cpu1x_freq) {
		return (-1);
	}

	/* Apply computed divisor value */
	ZSLCR_LOCK(sc);
	reg = RD4(sc, ZY7_SLCR_SPI_CLK_CTRL);
	reg &= ~ZY7_SLCR_SPI_CLK_DIVISOR_MASK;
	reg |= (divisor << ZY7_SLCR_SPI_CLK_DIVISOR_SHIFT);
	zy7_slcr_unlock(sc);
	WR4(sc, ZY7_SLCR_SPI_CLK_CTRL, reg);
	zy7_slcr_lock(sc);
	ZSLCR_UNLOCK(sc);

	return (0);
}

int
qspi_set_ref_clk_freq(int desired_freq)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	uint32_t reg;
	int divisor;  /* attempt to find which yields desired_freq */
	int qspi_ref_freq; /* resultant freq after applying the divisor */
	int cpu1x_freq;
	int err;
	enum zy7_clk_src src_clk;
	int src_freq;

	if (!sc)
		return (-1);

	err = qspi_get_ref_clk_source(&src_clk);
	if (err) return err;
	err = freq_of_source(src_clk, &src_freq);
	if (err) return err;
	err = cpu_freqs(&cpu1x_freq, NULL, NULL, NULL);
	if (err) return err;

	/* Find the divisor which yields a QSPI reference frequency
	 * which is closest to, but not greater, than what is desired. */
	divisor = ZY7_SLCR_LQSPI_CLK_DIVISOR_MIN;
	while (divisor <= ZY7_SLCR_LQSPI_CLK_DIVISOR_MAX) {
		qspi_ref_freq = src_freq / divisor;
		if (qspi_ref_freq > desired_freq)
			++divisor; /* freq still too large. keep reaching */
		else
			break; /* divisor found */
	}
	if (divisor > ZY7_SLCR_LQSPI_CLK_DIVISOR_MAX
	    /* Clocking restrictions require that the SPI reference clock,
	     * SPI_Ref_Clk, be greater than the CPU_1x clock frequency.
	     */
	    || qspi_ref_freq <= cpu1x_freq) {
		return (-1);
	}

	/* Apply computed divisor value */
	ZSLCR_LOCK(sc);
	reg = RD4(sc, ZY7_SLCR_LQSPI_CLK_CTRL);
	reg &= ~ZY7_SLCR_LQSPI_CLK_DIVISOR_MASK;
	reg |= (divisor << ZY7_SLCR_LQSPI_CLK_DIVISOR_SHIFT);
	zy7_slcr_unlock(sc);
	WR4(sc, ZY7_SLCR_LQSPI_CLK_CTRL, reg);
	zy7_slcr_lock(sc);
	ZSLCR_UNLOCK(sc);

	return (0);
}

int
cspi_get_ref_clk_freq(int *freq)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	uint32_t reg;
	int err;
	int divisor;
	int src_freq;
	enum zy7_clk_src src_clk;

	if (!sc || !freq)
		return (-1);

	err = cspi_get_ref_clk_source(&src_clk);
	if (err) return err;
	err = freq_of_source(src_clk, &src_freq);
	if (err) return err;

	ZSLCR_LOCK(sc);
	reg = RD4(sc, ZY7_SLCR_SPI_CLK_CTRL);
	ZSLCR_UNLOCK(sc);

	divisor = (reg & ZY7_SLCR_SPI_CLK_DIVISOR_MASK)
	    >> ZY7_SLCR_SPI_CLK_DIVISOR_SHIFT;
	KASSERT(divisor >= ZY7_SLCR_SPI_CLK_DIVISOR_MIN,
	    ("cspi_get_ref_clk_freq: divisor too small"));
	*freq = src_freq / divisor;
	return (0);
}

int
qspi_get_ref_clk_freq(int *freq)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;
	uint32_t reg;
	int err;
	int divisor;
	int src_freq;
	enum zy7_clk_src src_clk;

	if (!sc || !freq)
		return (-1);

	err = qspi_get_ref_clk_source(&src_clk);
	if (err) return err;
	err = freq_of_source(src_clk, &src_freq);
	if (err) return err;

	ZSLCR_LOCK(sc);
	reg = RD4(sc, ZY7_SLCR_LQSPI_CLK_CTRL);
	ZSLCR_UNLOCK(sc);

	divisor = (reg & ZY7_SLCR_LQSPI_CLK_DIVISOR_MASK)
	    >> ZY7_SLCR_LQSPI_CLK_DIVISOR_SHIFT;
	KASSERT(divisor >= ZY7_SLCR_LQSPI_CLK_DIVISOR_MIN,
	    ("qspi_get_ref_clk_freq: divisor too small"));
	*freq = src_freq / divisor;
	return (0);
}

uint32_t
zy7_mio_get_pin_register(int pin)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;

	if (pin < ZY7_SLCR_MIO_PIN_MIN || pin > ZY7_SLCR_MIO_PIN_MAX)
		return false;

	ZSLCR_LOCK(sc);

	uint32_t result = RD4(sc, ZY7_SLCR_MIO_PIN(pin));

	ZSLCR_UNLOCK(sc);
	return result;
}

bool
zy7_mio_set_pin_register(int pin, uint32_t reg)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;

	if (pin < ZY7_SLCR_MIO_PIN_MIN || pin > ZY7_SLCR_MIO_PIN_MAX)
		return false;

	ZSLCR_LOCK(sc);
	zy7_slcr_unlock(sc);

	WR4(sc, ZY7_SLCR_MIO_PIN(pin), reg);

	zy7_slcr_lock(sc);
	ZSLCR_UNLOCK(sc);
	return true;
}

bool
zy7_mio_unmap_pin_range(int begin, int end)
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;

	if (begin < ZY7_SLCR_MIO_PIN_MIN
	    || end > ZY7_SLCR_MIO_PIN_MAX
	    || begin > end)
		return false;

	ZSLCR_LOCK(sc);
	zy7_slcr_unlock(sc);

	for (int i = begin; i <= end; ++i)
		WR4(sc, ZY7_SLCR_MIO_PIN(i), 0);

	zy7_slcr_lock(sc);
	ZSLCR_UNLOCK(sc);
	return true;
}

void
zy7_dump_mio_pin_control_registers()
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;

	device_printf(sc->dev, "MIO pin control register values\n");
	device_printf(sc->dev, "-------------------------------\n");

	ZSLCR_LOCK(sc);

	for (int i = ZY7_SLCR_MIO_PIN_MIN; i <= ZY7_SLCR_MIO_PIN_MAX; ++i)
		device_printf(sc->dev, "[%02d] 0x%08x\n", i, RD4(sc, ZY7_SLCR_MIO_PIN(i)));

	ZSLCR_UNLOCK(sc);
}

static void
zy7_dump_all_other_registers()
{
	struct zy7_slcr_softc *sc = zy7_slcr_softc_p;

	device_printf(sc->dev, "Other register values\n");
	device_printf(sc->dev, "---------------------\n");

	ZSLCR_LOCK(sc);

  unsigned reg_list[] = {
		ZY7_SCLR_SCL,
		ZY7_SLCR_LOCKSTA,
		ZY7_SLCR_ARM_PLL_CTRL,
		ZY7_SLCR_DDR_PLL_CTRL,
		ZY7_SLCR_IO_PLL_CTRL,
		ZY7_SLCR_PLL_STATUS,
		ZY7_SLCR_ARM_PLL_CFG,
		ZY7_SLCR_DDR_PLL_CFG,
		ZY7_SLCR_IO_PLL_CFG,
		ZY7_SLCR_ARM_CLK_CTRL,
		ZY7_SLCR_DDR_CLK_CTRL,
		ZY7_SLCR_DCI_CLK_CTRL,
		ZY7_SLCR_APER_CLK_CTRL,
		ZY7_SLCR_USB0_CLK_CTRL,
		ZY7_SLCR_USB1_CLK_CTRL,
		ZY7_SLCR_GEM0_RCLK_CTRL,
		ZY7_SLCR_GEM1_RCLK_CTRL,
		ZY7_SLCR_GEM0_CLK_CTRL,
		ZY7_SLCR_GEM1_CLK_CTRL,
		ZY7_SLCR_SMC_CLK_CTRL,
		ZY7_SLCR_LQSPI_CLK_CTRL,
		ZY7_SLCR_SDIO_CLK_CTRL,
		ZY7_SLCR_UART_CLK_CTRL,
		ZY7_SLCR_SPI_CLK_CTRL,
		ZY7_SLCR_CAN_CLK_CTRL,
		ZY7_SLCR_CAN_MIOCLK_CTRL,
		ZY7_SLCR_DBG_CLK_CTRL,
		ZY7_SLCR_PCAP_CLK_CTRL,
		ZY7_SLCR_TOPSW_CLK_CTRL,
		ZY7_SLCR_FPGA_CLK_CTRL(0),
		ZY7_SLCR_FPGA_CLK_CTRL(1),
		ZY7_SLCR_FPGA_CLK_CTRL(2),
		ZY7_SLCR_FPGA_CLK_CTRL(3),
		ZY7_SLCR_FPGA_THR_CTRL(0),
		ZY7_SLCR_FPGA_THR_CTRL(1),
		ZY7_SLCR_FPGA_THR_CTRL(2),
		ZY7_SLCR_FPGA_THR_CTRL(3),
		ZY7_SLCR_FPGA_THR_CNT(0),
		ZY7_SLCR_FPGA_THR_CNT(1),
		ZY7_SLCR_FPGA_THR_CNT(2),
		ZY7_SLCR_FPGA_THR_CNT(3),
		ZY7_SLCR_FPGA_THR_STA(0),
		ZY7_SLCR_FPGA_THR_STA(1),
		ZY7_SLCR_FPGA_THR_STA(2),
		ZY7_SLCR_FPGA_THR_STA(3),
		ZY7_SLCR_CLK_621_TRUE,
		ZY7_SLCR_PSS_RST_CTRL,
		ZY7_SLCR_DDR_RST_CTRL,
		ZY7_SLCR_TOPSW_RST_CTRL,
		ZY7_SLCR_DMAC_RST_CTRL,
		ZY7_SLCR_USB_RST_CTRL,
		ZY7_SLCR_GEM_RST_CTRL,
		ZY7_SLCR_SDIO_RST_CTRL,
		ZY7_SLCR_SPI_RST_CTRL,
		ZY7_SLCR_CAN_RST_CTRL,
		ZY7_SLCR_I2C_RST_CTRL,
		ZY7_SLCR_UART_RST_CTRL,
		ZY7_SLCR_GPIO_RST_CTRL,
		ZY7_SLCR_LQSPI_RST_CTRL,
		ZY7_SLCR_SMC_RST_CTRL,
		ZY7_SLCR_OCM_RST_CTRL,
		ZY7_SLCR_DEVCI_RST_CTRL,
		ZY7_SLCR_FPGA_RST_CTRL,
		ZY7_SLCR_A9_CPU_RST_CTRL,
		ZY7_SLCR_RS_AWDT_CTRL,
		ZY7_SLCR_REBOOT_STAT,
		ZY7_SLCR_BOOT_MODE,
		ZY7_SLCR_APU_CTRL,
		ZY7_SLCR_WDT_CLK_SEL,
		ZY7_SLCR_PSS_IDCODE,
		ZY7_SLCR_DDR_URGENT,
		ZY7_SLCR_DDR_CAL_START,
		ZY7_SLCR_DDR_REF_START,
		ZY7_SLCR_DDR_CMD_STA,
		ZY7_SLCR_DDR_URGENT_SEL,
		ZY7_SLCR_DDR_DFI_STATUS,
		ZY7_SLCR_MIO_LOOPBACK,
		ZY7_SLCR_MIO_MST_TRI0,
		ZY7_SLCR_MIO_MST_TRI1,
		ZY7_SLCR_SD0_WP_CD_SEL,
		ZY7_SLCR_SD1_WP_CD_SEL,
		ZY7_SLCR_LVL_SHFTR_EN,
		ZY7_SLCR_OCM_CFG,
		ZY7_SLCR_GPIOB_CTRL,
		ZY7_SLCR_GPIOB_CFG_CMOS18,
		ZY7_SLCR_GPIOB_CFG_CMOS25,
		ZY7_SLCR_GPIOB_CFG_CMOS33,
		ZY7_SLCR_GPIOB_CFG_LVTTL,
		ZY7_SLCR_GPIOB_CFG_HSTL,
		ZY7_SLCR_GPIOB_DRVR_BIAS_CTRL,
		ZY7_SLCR_DDRIOB_ADDR0,
		ZY7_SLCR_DDRIOB_ADDR1,
		ZY7_SLCR_DDRIOB_DATA0,
		ZY7_SLCR_DDRIOB_DATA1,
		ZY7_SLCR_DDRIOB_DIFF0,
		ZY7_SLCR_DDRIOB_DIFF1,
		ZY7_SLCR_DDRIOB_CLK,
		ZY7_SLCR_DDRIOB_DRIVE_SLEW_ADDR,
		ZY7_SLCR_DDRIOB_DRIVE_SLEW_DATA,
		ZY7_SLCR_DDRIOB_DRIVE_SLEW_DIFF,
		ZY7_SLCR_DDRIOB_DRIVE_SLEW_CLK ,
		ZY7_SLCR_DDRIOB_DDR_CTRL,
		ZY7_SLCR_DDRIOB_DCI_CTRL,
		ZY7_SLCR_DDRIOB_DCI_STATUS
	};
	for (int i = 0; i < nitems(reg_list); ++i)
		device_printf(sc->dev, "[0x%08x] 0x%08x\n", reg_list[i],
				RD4(sc, reg_list[i]));

	ZSLCR_UNLOCK(sc);
}


static int
zy7_slcr_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "xlnx,zy7_slcr"))
		return (ENXIO);

	device_set_desc(dev, "Zynq-7000 slcr block");
	return (0);
}

static int
zy7_slcr_attach(device_t dev)
{
	struct zy7_slcr_softc *sc = device_get_softc(dev);
	int rid;
	phandle_t node;
	pcell_t cell;
	uint32_t bootmode;
	uint32_t pss_idcode;
	uint32_t arm_pll_ctrl;
	uint32_t ddr_pll_ctrl;
	uint32_t io_pll_ctrl;
	static char *bootdev_names[] = {
		"JTAG", "Quad-SPI", "NOR", "(3?)",
		"NAND", "SD Card", "(6?)", "(7?)"
	};

	/* Allow only one attach. */
	if (zy7_slcr_softc_p != NULL)
		return (ENXIO);

	sc->dev = dev;

	ZSLCR_LOCK_INIT(sc);

	/* Get memory resource. */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
					     RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "could not allocate memory resources.\n");
		return (ENOMEM);
	}

	/* Hook up cpu_reset. */
	zy7_slcr_softc_p = sc;
	zynq7_cpu_reset = zy7_slcr_cpu_reset;

	/* Read info and set sysctls. */
	bootmode = RD4(sc, ZY7_SLCR_BOOT_MODE);
	snprintf(zynq_bootmode, sizeof(zynq_bootmode),
		 "0x%x: boot device: %s", bootmode,
		 bootdev_names[bootmode & ZY7_SLCR_BOOT_MODE_BOOTDEV_MASK]);

	pss_idcode = RD4(sc, ZY7_SLCR_PSS_IDCODE);
	snprintf(zynq_pssid, sizeof(zynq_pssid),
		 "0x%x: manufacturer: 0x%x device: 0x%x "
		 "family: 0x%x sub-family: 0x%x rev: 0x%x",
		 pss_idcode,
		 (pss_idcode & ZY7_SLCR_PSS_IDCODE_MNFR_ID_MASK) >>
		 ZY7_SLCR_PSS_IDCODE_MNFR_ID_SHIFT,
		 (pss_idcode & ZY7_SLCR_PSS_IDCODE_DEVICE_MASK) >>
		 ZY7_SLCR_PSS_IDCODE_DEVICE_SHIFT,
		 (pss_idcode & ZY7_SLCR_PSS_IDCODE_FAMILY_MASK) >>
		 ZY7_SLCR_PSS_IDCODE_FAMILY_SHIFT,
		 (pss_idcode & ZY7_SLCR_PSS_IDCODE_SUB_FAMILY_MASK) >>
		 ZY7_SLCR_PSS_IDCODE_SUB_FAMILY_SHIFT,
		 (pss_idcode & ZY7_SLCR_PSS_IDCODE_REVISION_MASK) >>
		 ZY7_SLCR_PSS_IDCODE_REVISION_SHIFT);

	zynq_reboot_status = RD4(sc, ZY7_SLCR_REBOOT_STAT);

	/* Derive PLL frequencies from PS_CLK. */
	node = ofw_bus_get_node(dev);
	if (OF_getencprop(node, "clock-frequency", &cell, sizeof(cell)) > 0)
		ps_clk_frequency = cell;
	else
		ps_clk_frequency = ZYNQ_DEFAULT_PS_CLK_FREQUENCY;

	arm_pll_ctrl = RD4(sc, ZY7_SLCR_ARM_PLL_CTRL);
	ddr_pll_ctrl = RD4(sc, ZY7_SLCR_DDR_PLL_CTRL);
	io_pll_ctrl = RD4(sc, ZY7_SLCR_IO_PLL_CTRL);

	/* Determine ARM PLL frequency. */
	if (((arm_pll_ctrl & ZY7_SLCR_PLL_CTRL_BYPASS_QUAL) == 0 &&
	     (arm_pll_ctrl & ZY7_SLCR_PLL_CTRL_BYPASS_FORCE) != 0) ||
	    ((arm_pll_ctrl & ZY7_SLCR_PLL_CTRL_BYPASS_QUAL) != 0 &&
	     (bootmode & ZY7_SLCR_BOOT_MODE_PLL_BYPASS) != 0))
		/* PLL is bypassed. */
		arm_pll_frequency = ps_clk_frequency;
	else
		arm_pll_frequency = ps_clk_frequency *
			((arm_pll_ctrl & ZY7_SLCR_PLL_CTRL_FDIV_MASK) >>
			 ZY7_SLCR_PLL_CTRL_FDIV_SHIFT);

	/* Determine DDR PLL frequency. */
	if (((ddr_pll_ctrl & ZY7_SLCR_PLL_CTRL_BYPASS_QUAL) == 0 &&
	     (ddr_pll_ctrl & ZY7_SLCR_PLL_CTRL_BYPASS_FORCE) != 0) ||
	    ((ddr_pll_ctrl & ZY7_SLCR_PLL_CTRL_BYPASS_QUAL) != 0 &&
	     (bootmode & ZY7_SLCR_BOOT_MODE_PLL_BYPASS) != 0))
		/* PLL is bypassed. */
		ddr_pll_frequency = ps_clk_frequency;
	else
		ddr_pll_frequency = ps_clk_frequency *
			((ddr_pll_ctrl & ZY7_SLCR_PLL_CTRL_FDIV_MASK) >>
			 ZY7_SLCR_PLL_CTRL_FDIV_SHIFT);

	/* Determine IO PLL frequency. */
	if (((io_pll_ctrl & ZY7_SLCR_PLL_CTRL_BYPASS_QUAL) == 0 &&
	     (io_pll_ctrl & ZY7_SLCR_PLL_CTRL_BYPASS_FORCE) != 0) ||
	    ((io_pll_ctrl & ZY7_SLCR_PLL_CTRL_BYPASS_QUAL) != 0 &&
	     (bootmode & ZY7_SLCR_BOOT_MODE_PLL_BYPASS) != 0))
		/* PLL is bypassed. */
		io_pll_frequency = ps_clk_frequency;
	else
		io_pll_frequency = ps_clk_frequency *
			((io_pll_ctrl & ZY7_SLCR_PLL_CTRL_FDIV_MASK) >>
			 ZY7_SLCR_PLL_CTRL_FDIV_SHIFT);

	/* Lock SLCR registers. */
	zy7_slcr_lock(sc);

	if (bootverbose) {
		zy7_dump_mio_pin_control_registers();
		zy7_dump_all_other_registers();
	}

	return (0);
}

static int
zy7_slcr_detach(device_t dev)
{
	struct zy7_slcr_softc *sc = device_get_softc(dev);

	bus_generic_detach(dev);

	/* Release memory resource. */
	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
			     rman_get_rid(sc->mem_res), sc->mem_res);

	zy7_slcr_softc_p = NULL;
	zynq7_cpu_reset = NULL;

	ZSLCR_LOCK_DESTROY(sc);

	return (0);
}

static device_method_t zy7_slcr_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe, 	zy7_slcr_probe),
	DEVMETHOD(device_attach, 	zy7_slcr_attach),
	DEVMETHOD(device_detach, 	zy7_slcr_detach),

	DEVMETHOD_END
};

static driver_t zy7_slcr_driver = {
	"zy7_slcr",
	zy7_slcr_methods,
	sizeof(struct zy7_slcr_softc),
};
static devclass_t zy7_slcr_devclass;

DRIVER_MODULE(zy7_slcr, simplebus, zy7_slcr_driver, zy7_slcr_devclass, 0, 0);
MODULE_VERSION(zy7_slcr, 1);
