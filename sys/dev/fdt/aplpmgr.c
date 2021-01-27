/* $OpenBSD$ */
/*
 * Copyright (c) 2021 Patrick Wildt <patrick@blueri.se>
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
#include <sys/malloc.h>
#include <sys/device.h>
#include <sys/atomic.h>

#include <machine/bus.h>
#include <machine/fdt.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_clock.h>
#include <dev/ofw/ofw_misc.h>
#include <dev/ofw/fdt.h>

#define NREG			4

#define REG0			0x00
#define  REG0_ENABLE			(0xf << 0)
#define  REG0_OFF			(0x0 << 4)
#define  REG0_ON			(0xf << 4)
#define  REG0_STATUS			(0xf << 4)
#define  REG0_DISABLE			(0x3 << 8)
#define  REG0_POSTENABLE		(1 << 28)

#define HREAD4(sc, reg)							\
	(bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh[(reg)], 0))
#define HWRITE4(sc, reg, val)						\
	bus_space_write_4((sc)->sc_iot, (sc)->sc_ioh[(reg)], 0, (val))
#define HSET4(sc, reg, bits)						\
	HWRITE4((sc), (reg), HREAD4((sc), (reg)) | (bits))
#define HCLR4(sc, reg, bits)						\
	HWRITE4((sc), (reg), HREAD4((sc), (reg)) & ~(bits))

struct aplpmgr_softc {
	struct device		sc_dev;
	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_ioh[NREG];
	int			sc_nreg;
	int			sc_node;

	struct clock_device	sc_cd;
};

int	aplpmgr_match(struct device *, void *, void *);
void	aplpmgr_attach(struct device *parent, struct device *self, void *args);

struct cfattach	aplpmgr_ca = {
	sizeof (struct aplpmgr_softc), aplpmgr_match, aplpmgr_attach
};

struct cfdriver aplpmgr_cd = {
	NULL, "aplpmgr", DV_DULL
};

void aplpmgr_run_seq(struct aplpmgr_softc *, char *);
void aplpmgr_enable(void *, uint32_t *, int);
uint32_t aplclock_get_frequency(void *cookie, uint32_t *cells);

int
aplpmgr_match(struct device *parent, void *match, void *aux)
{
	struct fdt_attach_args *faa = aux;

	return OF_is_compatible(faa->fa_node, "apple,pmgr-clk-gate");
}

void
aplpmgr_attach(struct device *parent, struct device *self, void *aux)
{
	struct aplpmgr_softc *sc = (struct aplpmgr_softc *)self;
	struct fdt_attach_args *faa = aux;
	int i;

	KASSERT(faa->fa_nreg >= 1);
	KASSERT(faa->fa_nreg <= NREG);

	sc->sc_nreg = faa->fa_nreg;
	sc->sc_node = faa->fa_node;
	sc->sc_iot = faa->fa_iot;
	for (i = 0; i < sc->sc_nreg; i++)
		if (bus_space_map(sc->sc_iot, faa->fa_reg[i].addr,
		    faa->fa_reg[i].size, 0, &sc->sc_ioh[i]))
			panic("%s: bus_space_map failed!", __func__);

	printf("\n");

	sc->sc_cd.cd_node = sc->sc_node;
	sc->sc_cd.cd_cookie = sc;
	sc->sc_cd.cd_enable = aplpmgr_enable;
	sc->sc_cd.cd_get_frequency = aplclock_get_frequency;
	clock_register(&sc->sc_cd);
}

void
aplpmgr_enable(void *cookie, uint32_t *cells, int on)
{
	struct aplpmgr_softc *sc = cookie;
	uint32_t reg = HREAD4(sc, REG0);
	int i;

	if ((on && (reg & REG0_STATUS) == REG0_ON) ||
	    (!on && (reg & REG0_STATUS) == REG0_OFF))
		return;

	if (on) {
		clock_enable_all(sc->sc_node);
		aplpmgr_run_seq(sc, "pre-up");
		reg = HREAD4(sc, REG0);
		reg |= REG0_ENABLE;
		HWRITE4(sc, REG0, reg);
		for (i = 10000; i > 0; i--) {
			membar_sync();
			reg = HREAD4(sc, REG0);
			if ((reg & REG0_STATUS) == REG0_ON)
				break;
		}
		if (i == 0) {
			printf("%s: enable timeout\n", sc->sc_dev.dv_xname);
			return;
		}
		reg |= REG0_POSTENABLE;
		HWRITE4(sc, REG0, reg);
		aplpmgr_run_seq(sc, "post-up");
	} else {
		aplpmgr_run_seq(sc, "pre-down");
		reg = HREAD4(sc, REG0);
		reg |= REG0_DISABLE;
		reg &= ~REG0_ON;
		HWRITE4(sc, REG0, reg);
		for (i = 10000; i > 0; i--) {
			reg = HREAD4(sc, REG0);
			if ((reg & REG0_STATUS) == REG0_OFF)
				break;
		}
		if (i == 0) {
			printf("%s: disable timeout\n", sc->sc_dev.dv_xname);
			return;
		}
		aplpmgr_run_seq(sc, "post-down");
	}
}

void
aplpmgr_run_seq(struct aplpmgr_softc *sc, char *action)
{
	uint32_t *seq;
	int i, len;

	len = OF_getproplen(sc->sc_node, action);
	if (len <= 0 || (len % (3 * sizeof(uint32_t))) != 0) {
		printf("%s: invalid action '%s'\n",
		    sc->sc_dev.dv_xname, action);
		return;
	}

	seq = malloc(len, M_DEVBUF, M_WAITOK);
	OF_getpropintarray(sc->sc_node, action, seq, len);

	for (i = 0; i < len / sizeof(uint32_t); i += 3) {
		KASSERT((seq[i + 0] + seq[i + 1]) < sc->sc_nreg);
		HWRITE4(sc, seq[i + 0] + seq[i + 1], seq[i + 2]);
	}

	free(seq, M_DEVBUF, len);
}

uint32_t
aplclock_get_frequency(void *cookie, uint32_t *cells)
{
	struct aplpmgr_softc *sc = cookie;

	return clock_get_frequency(sc->sc_node, NULL);
}
