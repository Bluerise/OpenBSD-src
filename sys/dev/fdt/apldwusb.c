/*	$OpenBSD: apldwusb.c,v 1.4 2020/12/19 01:21:35 patrick Exp $	*/
/*
 * Copyright (c) 2017, 2018 Mark Kettenis <kettenis@openbsd.org>
 * Copyright (c) 2020, 2021 Patrick Wildt <patrick@blueri.se>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/malloc.h>

#include <machine/bus.h>
#include <machine/fdt.h>

#include <arm64/dev/simplebusvar.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_clock.h>
#include <dev/ofw/ofw_power.h>
#include <dev/ofw/fdt.h>

#define APLDWUSB_TUNABLE_ADDR(x)		(((x) >>  0) & 0xfffffff)
#define APLDWUSB_TUNABLE_RANGE(x)		(((x) >> 28) & 0xf)

#define USBCORE_PIPEPHY_STATUS			0x200020
#define   USBCORE_PIPEPHY_STATUS_READY			(1 << 30)
#define USBCORE_FORCE_CLK_ON			0x2000f0
#define USBCORE_AUSBEVT_USB2CTL			0x800000
#define   USBCORE_AUSBEVT_USB2CTL_EVT_EN		(1 << 0)
#define   USBCORE_AUSBEVT_USB2CTL_LOAD_CNT		(1 << 3)
#define USBCORE_AUSBEVT_UTMIACT_EVTCNT		0x800020
#define USBCORE_PIPEHDLR_MUXSEL			0xa8400c
#define   USBCORE_PIPEHDLR_MUXSEL_MODE_MASK		(3 << 0)
#define     USBCORE_PIPEHDLR_MUXSEL_MODE_USB2		(2 << 0)
#define   USBCORE_PIPEHDLR_MUXSEL_CLKEN_MASK		(3 << 3)
#define USBCORE_PIPEHDLR_PIPE_IF_REQ		0xa84010
#define USBCORE_PIPEHDLR_PIPE_IF_ACK		0xa84014
#define USBCORE_PIPEHDLR_AON_GEN		0xa8401c
#define   USBCORE_PIPEHDLR_AON_GEN_DRD_FORCE_CLAMP_EN	(1 << 4)
#define   USBCORE_PIPEHDLR_AON_GEN_DRD_SW_VCC_RESET	(1 << 0)
#define USBCORE_PIPEHDLR_NONSEL_OVRD		0xa84020
#define   USBCORE_PIPEHDLR_NONSEL_OVRD_DUMMY_PHY_READY	(1 << 15)
#define USBCORE_USB2PHY_USBCTL			0xa90000
#define   USBCORE_USB2PHY_USBCTL_MODE_MASK		(7 << 0)
#define     USBCORE_USB2PHY_USBCTL_MODE_USB2		(2 << 0)
#define USBCORE_USB2PHY_CTL			0xa90004
#define   USBCORE_USB2PHY_CTL_RESET			(1 << 0)
#define   USBCORE_USB2PHY_CTL_PORT_RESET		(1 << 1)
#define   USBCORE_USB2PHY_CTL_APB_RESETN		(1 << 2)
#define   USBCORE_USB2PHY_CTL_SIDDQ			(1 << 3)
#define USBCORE_USB2PHY_SIG			0xa90008
#define   USBCORE_USB2PHY_SIG_VBUSDET_FORCE_VAL		(1 << 0)
#define   USBCORE_USB2PHY_SIG_VBUSDET_FORCE_EN		(1 << 1)
#define   USBCORE_USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL	(1 << 2)
#define   USBCORE_USB2PHY_SIG_VBUSVLDEXT_FORCE_EN	(1 << 3)
#define   USBCORE_USB2PHY_SIG_MODE_HOST			(7 << 12)
#define USBCORE_USB2PHY_MISCTUNE		0xa9001c
#define   USBCORE_USB2PHY_MISCTUNE_APBCLK_GATE_OFF	(1 << 29)
#define   USBCORE_USB2PHY_MISCTUNE_REFCLK_GATE_OFF	(1 << 30)

#define USB3_GCTL				0x28c110
#define  USB3_GCTL_GBLHIBERNATIONEN			(1 << 1)
#define  USB3_GCTL_PRTCAPDIR_MASK			(0x3 << 12)
#define  USB3_GCTL_PRTCAPDIR_HOST			(0x1 << 12)
#define  USB3_GCTL_PRTCAPDIR_DEVICE			(0x2 << 12)
#define  USB3_GCTL_PWRDNSCALE_MASK			(0x1fff << 19)
#define  USB3_GCTL_PWRDNSCALE(n)			((n) << 19)
#define USB3_GUCTL1				0x28c11c
#define  USB3_GUCTL1_TX_IPGAP_LINECHECK_DIS		(1 << 28)
#define USB3_GUSB2PHYCFG0			0x28c200
#define  USB3_GUSB2PHYCFG0_U2_FREECLK_EXISTS		(1 << 30)
#define  USB3_GUSB2PHYCFG0_USBTRDTIM(n)			((n) << 10)
#define  USB3_GUSB2PHYCFG0_ENBLSLPM			(1 << 8)
#define  USB3_GUSB2PHYCFG0_SUSPENDUSB20			(1 << 6)
#define  USB3_GUSB2PHYCFG0_PHYIF			(1 << 3)
#define USB3_GUSB3PIPECTL0			0x28c2c0
#define  USB3_GUSB3PIPECTL0_SUSPENDENABLE		(1 << 17)

/*
 * This driver is based on preliminary device tree bindings and will
 * almost certainly need changes once the official bindings land in
 * mainline Linux.  Support for these preliminary bindings will be
 * dropped as soon as official bindings are available.
 */

struct apldwusb_softc {
	struct simplebus_softc	sc_sbus;
	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_phy_ioh;
	bus_space_handle_t	sc_usb_ioh;
	int			sc_node;
};

int	apldwusb_match(struct device *, void *, void *);
void	apldwusb_attach(struct device *, struct device *, void *);

int	apldwusb_init(struct apldwusb_softc *);
int	apldwusb_tunable(struct apldwusb_softc *, char *);

struct cfattach apldwusb_ca = {
	sizeof(struct apldwusb_softc), apldwusb_match, apldwusb_attach
};

struct cfdriver apldwusb_cd = {
	NULL, "apldwusb", DV_DULL
};

int
apldwusb_match(struct device *parent, void *match, void *aux)
{
	struct fdt_attach_args *faa = aux;

	return OF_is_compatible(faa->fa_node, "apple,dwc3-m1");
}

void
apldwusb_attach(struct device *parent, struct device *self, void *aux)
{
	struct apldwusb_softc *sc = (struct apldwusb_softc *)self;
	struct fdt_attach_args *faa = aux;

	KASSERT(faa->fa_nreg >= 2);

	power_domain_enable(faa->fa_node);
	clock_enable_all(faa->fa_node);
	reset_deassert_all(faa->fa_node);

	sc->sc_node = faa->fa_node;
	sc->sc_iot = faa->fa_iot;
	if (bus_space_map(sc->sc_iot, faa->fa_reg[0].addr,
	    faa->fa_reg[0].size, 0, &sc->sc_phy_ioh))
		panic("%s: bus_space_map failed!", __func__);
	if (bus_space_map(sc->sc_iot, faa->fa_reg[1].addr,
	    faa->fa_reg[1].size, 0, &sc->sc_usb_ioh))
		panic("%s: bus_space_map failed!", __func__);

	if (apldwusb_init(sc)) {
		printf(": init failed\n");
		return;
	}

	simplebus_attach(parent, &sc->sc_sbus.sc_dev, faa);
}

int
apldwusb_init(struct apldwusb_softc *sc)
{
	uint32_t reg;
	int error, i;

	error = apldwusb_tunable(sc, "tunable-ATC0AXI2AF");
	if (error)
		return error;

	reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_USB2PHY_SIG);
	reg |= USBCORE_USB2PHY_SIG_MODE_HOST |
	    USBCORE_USB2PHY_SIG_VBUSDET_FORCE_VAL |
	    USBCORE_USB2PHY_SIG_VBUSDET_FORCE_EN |
	    USBCORE_USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL |
	    USBCORE_USB2PHY_SIG_VBUSVLDEXT_FORCE_EN;
	bus_space_write_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_USB2PHY_SIG, reg);

	reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_USB2PHY_CTL);
	reg &= ~USBCORE_USB2PHY_CTL_SIDDQ;
	bus_space_write_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_USB2PHY_CTL, reg);
	delay(10);

	reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_USB2PHY_CTL);
	reg &= ~(USBCORE_USB2PHY_CTL_RESET | USBCORE_USB2PHY_CTL_PORT_RESET);
	bus_space_write_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_USB2PHY_CTL, reg);

	reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_AUSBEVT_USB2CTL);
	reg |= USBCORE_AUSBEVT_USB2CTL_EVT_EN;
	reg |= USBCORE_AUSBEVT_USB2CTL_LOAD_CNT;
	bus_space_write_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_AUSBEVT_USB2CTL, reg);

	reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_USB2PHY_CTL);
	reg |= USBCORE_USB2PHY_CTL_APB_RESETN;
	bus_space_write_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_USB2PHY_CTL, reg);

	reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_USB2PHY_MISCTUNE);
	reg &= ~USBCORE_USB2PHY_MISCTUNE_APBCLK_GATE_OFF;
	reg &= ~USBCORE_USB2PHY_MISCTUNE_REFCLK_GATE_OFF;
	bus_space_write_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_USB2PHY_MISCTUNE, reg);
	delay(30);

	for (i = 100; i > 0; i++) {
		reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh,
		    USBCORE_AUSBEVT_UTMIACT_EVTCNT);
		if (reg)
			break;
		delay(100);
	}
	if (i == 0) {
		printf("%s: UTMI clock active timeout\n",
		    sc->sc_sbus.sc_dev.dv_xname);
		return 1;
	}

	reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_USB2PHY_USBCTL);
	reg &= ~USBCORE_USB2PHY_USBCTL_MODE_MASK;
	reg |= USBCORE_USB2PHY_USBCTL_MODE_USB2;
	bus_space_write_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_USB2PHY_USBCTL, reg);

	reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_PIPEHDLR_AON_GEN);
	reg &= ~USBCORE_PIPEHDLR_AON_GEN_DRD_FORCE_CLAMP_EN;
	reg |= USBCORE_PIPEHDLR_AON_GEN_DRD_SW_VCC_RESET;
	bus_space_write_4(sc->sc_iot, sc->sc_usb_ioh, USBCORE_PIPEHDLR_AON_GEN, reg);

	reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh, USB3_GCTL);
	reg &= ~USB3_GCTL_PRTCAPDIR_MASK;
	reg |= USB3_GCTL_PRTCAPDIR_HOST;
	bus_space_write_4(sc->sc_iot, sc->sc_usb_ioh, USB3_GCTL, reg);

	reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh,
	    USBCORE_PIPEHDLR_MUXSEL);
	if ((reg & USBCORE_PIPEHDLR_MUXSEL_MODE_MASK) !=
	    USBCORE_PIPEHDLR_MUXSEL_MODE_USB2) {
		for (i = 100; i > 0; i++) {
			reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh,
			    USBCORE_PIPEPHY_STATUS);
			if ((reg & USBCORE_PIPEPHY_STATUS_READY) ==
			    USBCORE_PIPEPHY_STATUS_READY)
				break;
			delay(100);
		}
		if (i == 0) {
			printf("%s: PIPE PHY ready timeout\n",
			    sc->sc_sbus.sc_dev.dv_xname);
			return 1;
		}
	}
	for (i = 100; i > 0; i++) {
		reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh,
		    USBCORE_AUSBEVT_UTMIACT_EVTCNT);
		if (reg)
			break;
		delay(100);
	}
	if (i == 0) {
		printf("%s: UTMI clock active timeout\n",
		    sc->sc_sbus.sc_dev.dv_xname);
		return 1;
	}

	error = apldwusb_tunable(sc, "tunable");
	if (error)
		return error;

	reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh, USB3_GCTL);
	reg &= ~USB3_GCTL_PWRDNSCALE_MASK;
	reg |= USB3_GCTL_PWRDNSCALE(13);
	reg |= USB3_GCTL_GBLHIBERNATIONEN;
	bus_space_write_4(sc->sc_iot, sc->sc_usb_ioh, USB3_GCTL, reg);

	reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh, USB3_GUSB2PHYCFG0);
	reg &= ~USB3_GUSB2PHYCFG0_SUSPENDUSB20;
	bus_space_write_4(sc->sc_iot, sc->sc_usb_ioh, USB3_GUSB2PHYCFG0, reg);
	reg = bus_space_read_4(sc->sc_iot, sc->sc_usb_ioh, USB3_GUSB3PIPECTL0);
	reg &= ~USB3_GUSB3PIPECTL0_SUSPENDENABLE;
	bus_space_write_4(sc->sc_iot, sc->sc_usb_ioh, USB3_GUSB3PIPECTL0, reg);

	return 0;
}

int
apldwusb_tunable(struct apldwusb_softc *sc, char *tunable)
{
	bus_space_handle_t ioh;
	uint32_t *seq, reg;
	int error, i, len;

	len = OF_getproplen(sc->sc_node, tunable);
	if (len == 0)
		return 0;
	if (len <= 0 || (len % (3 * sizeof(uint32_t))) != 0)
		return 1;

	seq = malloc(len, M_DEVBUF, M_WAITOK);
	OF_getpropintarray(sc->sc_node, tunable, seq, len);

	for (i = 0; i < len / sizeof(uint32_t); i += 3) {
		switch (APLDWUSB_TUNABLE_RANGE(seq[i + 0])) {
		case 0:
			ioh = sc->sc_phy_ioh;
			break;
		case 1:
			ioh = sc->sc_usb_ioh;
			break;
		default:
			printf("%s: unknown range\n",
			    sc->sc_sbus.sc_dev.dv_xname);
			error = 1;
			goto out;
		}
		reg = bus_space_read_4(sc->sc_iot, ioh,
		    APLDWUSB_TUNABLE_ADDR(seq[i + 0]));
		reg &= ~seq[i + 1];
		reg |= seq[i + 2];
		bus_space_write_4(sc->sc_iot, ioh,
		    APLDWUSB_TUNABLE_ADDR(seq[i + 0]), reg);
	}

	error = 0;
out:
	free(seq, M_DEVBUF, len);
	return error;
}
