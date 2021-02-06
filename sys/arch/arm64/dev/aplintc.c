/* $OpenBSD$ */
/*
 * Copyright (c) 2007, 2009, 2011, 2017 Dale Rahn <drahn@dalerahn.com>
 * Copyright (c) 2018 Mark Kettenis <kettenis@openbsd.org>
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
#include <sys/queue.h>
#include <sys/malloc.h>
#include <sys/device.h>
#include <sys/evcount.h>
#include <sys/atomic.h>

#include <machine/bus.h>
#include <machine/cpufunc.h>
#include <machine/fdt.h>

#include <dev/ofw/fdt.h>
#include <dev/ofw/openfirm.h>

#define _STR(x) #x
#define STR(x) _STR(x)

#define AIC_INFO		0x0004
#define  AIC_INFO_NIRQ(x)		(((x) >> 0) & 0xffff)
#define AIC_IRQ_ACK		0x2004
#define  AIC_IRQ_ACK_IRQNO(x)		(((x) >> 0) & 0xffff)
#define  AIC_IRQ_ACK_TYPE(x)		(((x) >> 16) & 0xf)
#define  AIC_IRQ_ACK_TYPE_HW		1
#define  AIC_IRQ_ACK_TYPE_IPI		4
#define AIC_IRQ_AFFINITY(i)	(0x3000 + (i) * 4)
#define AIC_IRQ_CLEAR(i)	(0x4080 + (IRQ_TO_REG32(i) * 4))
#define AIC_IRQ_DISABLE(i)	(0x4100 + (IRQ_TO_REG32(i) * 4))
#define AIC_IRQ_ENABLE(i)	(0x4180 + (IRQ_TO_REG32(i) * 4))

#define AIC_NUM_FIQ	2
#define AIC_NUM_IPI	2

#define IRQ_TO_REG32(i)		(((i) >> 5) & 0x7)
#define IRQ_TO_REG32BIT(i)	((i) & 0x1f)

#define IRQ_TO_REG16(i)		(((i) >> 4) & 0xf)
#define IRQ_TO_REG16BIT(i)	((i) & 0xf)

#define IRQ_ENABLE	1
#define IRQ_DISABLE	0

struct aplintc_softc {
	struct device		 sc_dev;
	struct intrq		*sc_handler;
	bus_space_tag_t		 sc_iot;
	bus_space_handle_t	 sc_ioh;
	bus_dma_tag_t		 sc_dmat;
	int			 sc_nintr;
	int			 sc_hwnintr;
	struct evcount		 sc_spur;
	struct interrupt_controller sc_ic;
	int			 sc_ipi_num[2]; /* id for NOP and DDB ipi */
	int			 sc_ipi_reason[MAXCPUS]; /* NOP or DDB caused */
	void			*sc_ipi_irq[2]; /* irqhandle for each ipi */
	uint32_t		*sc_imask;
};
struct aplintc_softc *aplintc_sc;

struct intrhand {
	TAILQ_ENTRY(intrhand)	 ih_list;		/* link on intrq list */
	int			(*ih_func)(void *);	/* handler */
	void			*ih_arg;		/* arg for handler */
	int			 ih_ipl;		/* IPL_* */
	int			 ih_flags;
	int			 ih_irq;		/* IRQ number */
	struct evcount		 ih_count;
	char			*ih_name;
	struct cpu_info		*ih_ci;			/* CPU the IRQ runs on */
};

struct intrq {
	TAILQ_HEAD(, intrhand)	iq_list;	/* handler list */
	struct cpu_info		*iq_ci;		/* CPU the IRQ runs on */
	int			iq_irq_max;	/* IRQ to mask while handling */
	int			iq_irq_min;	/* lowest IRQ when shared */
	int			iq_ist;		/* share type */
	int			iq_route;
};

int		aplintc_match(struct device *, void *, void *);
void		aplintc_attach(struct device *, struct device *, void *);
void		aplintc_cpuinit(void);
int		aplintc_spllower(int);
void		aplintc_splx(int);
int		aplintc_splraise(int);
void		aplintc_setipl(int);
void		aplintc_calc_mask(struct aplintc_softc *);
void		aplintc_calc_irq(struct aplintc_softc *, int irq);
void		*aplintc_intr_establish(int, int, int, struct cpu_info *,
		    int (*)(void *), void *, char *);
void		*aplintc_intr_establish_fdt(void *cookie, int *cell, int level,
		    struct cpu_info *, int (*func)(void *), void *arg, char *name);
void		aplintc_intr_disestablish(void *);
void		aplintc_irq_handler(void *);
void		aplintc_fiq_handler(void *);
void		aplintc_eoi(struct aplintc_softc *, uint32_t);
void		aplintc_intr_enable(struct aplintc_softc *, int, int);
void		aplintc_intr_disable(struct aplintc_softc *, int, int);
void		aplintc_route(struct aplintc_softc *, int, int,
		    struct cpu_info *);
void		aplintc_route_irq(void *, int, struct cpu_info *);
void		aplintc_intr_barrier(void *);

int		aplintc_ipi_ddb(void *v);
int		aplintc_ipi_nop(void *v);
int		aplintc_ipi_combined(void *);
void		aplintc_send_ipi(struct cpu_info *, int);

struct cfattach	aplintc_ca = {
	sizeof (struct aplintc_softc), aplintc_match, aplintc_attach
};

struct cfdriver aplintc_cd = {
	NULL, "aplintc", DV_DULL
};

static char *aplintc_compatibles[] = {
	"apple,aic",
	NULL
};

int
aplintc_match(struct device *parent, void *cfdata, void *aux)
{
	struct fdt_attach_args *faa = aux;
	int i;

	for (i = 0; aplintc_compatibles[i]; i++)
		if (OF_is_compatible(faa->fa_node, aplintc_compatibles[i]))
			return (1);

	return (0);
}

static void
__isb(void)
{
	__asm volatile("isb");
}

void
aplintc_attach(struct device *parent, struct device *self, void *aux)
{
	struct aplintc_softc	*sc = (struct aplintc_softc *)self;
	struct fdt_attach_args	*faa = aux;
	int			 i, nintr;

	arm_init_smask();

	sc->sc_iot = faa->fa_iot;
	sc->sc_dmat = faa->fa_dmat;

	if (bus_space_map(sc->sc_iot, faa->fa_reg[0].addr,
	    faa->fa_reg[0].size, 0, &sc->sc_ioh))
		panic("%s: bus_space_map failed!", __func__);

	evcount_attach(&sc->sc_spur, "irq1023/spur", NULL);

	nintr = bus_space_read_4(sc->sc_iot, sc->sc_ioh, AIC_INFO);
	nintr = AIC_INFO_NIRQ(nintr);
	sc->sc_hwnintr = nintr;
	sc->sc_nintr = nintr + AIC_NUM_FIQ + AIC_NUM_IPI;

	aplintc_sc = sc; /* save this for global access */

	printf(" nirq %d\n", nintr);

	/* Disable all interrupts, clear all pending */
	for (i = 0; i < nintr / 32; i++) {
		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    AIC_IRQ_DISABLE(i * 32), ~0);
		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    AIC_IRQ_CLEAR(i * 32), ~0);
	}

	/* Target primary CPU */
	for (i = 0; i < nintr; i++) {
		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    AIC_IRQ_AFFINITY(i), 1 << 0);
	}

	aplintc_cpuinit();

	sc->sc_handler = mallocarray(sc->sc_nintr,
	    sizeof(*sc->sc_handler), M_DEVBUF, M_ZERO | M_WAITOK);
	for (i = 0; i < sc->sc_nintr; i++)
		TAILQ_INIT(&sc->sc_handler[i].iq_list);

	sc->sc_imask = mallocarray((roundup(nintr, 32) / 32) * NIPL,
	    sizeof(*sc->sc_imask), M_DEVBUF, M_ZERO | M_WAITOK);

	/* set priority to IPL_HIGH until configure lowers to desired IPL */
	aplintc_setipl(IPL_HIGH);

	/* initialize all interrupts as disabled */
	aplintc_calc_mask(sc);

	/* insert self as interrupt handler */
	arm_set_intr_handler(aplintc_splraise, aplintc_spllower, aplintc_splx,
	    aplintc_setipl, aplintc_irq_handler, aplintc_fiq_handler);

#ifdef MULTIPROCESSOR
	/* setup IPI interrupts */
	sc->sc_ipi_irq[0] = aplintc_intr_establish(
	    sc->sc_hwnintr + AIC_NUM_FIQ + 0,
	    IST_LEVEL_HIGH, IPL_IPI|IPL_MPSAFE, NULL,
	    aplintc_ipi_nop, sc, "ipinop");
	sc->sc_ipi_num[ARM_IPI_NOP] = 0;
	sc->sc_ipi_irq[1] = aplintc_intr_establish(
	    sc->sc_hwnintr + AIC_NUM_FIQ + 1,
	    IST_LEVEL_HIGH, IPL_IPI|IPL_MPSAFE, NULL,
	    aplintc_ipi_ddb, sc, "ipiddb");
	sc->sc_ipi_num[ARM_IPI_DDB] = 1;

	intr_send_ipi_func = aplintc_send_ipi;
#endif

	enable_interrupts(PSR_I | PSR_F);

	sc->sc_ic.ic_node = faa->fa_node;
	sc->sc_ic.ic_cookie = self;
	sc->sc_ic.ic_establish = aplintc_intr_establish_fdt;
	sc->sc_ic.ic_disestablish = aplintc_intr_disestablish;
	sc->sc_ic.ic_route = aplintc_route_irq;
	sc->sc_ic.ic_cpu_enable = aplintc_cpuinit;
	sc->sc_ic.ic_barrier = aplintc_intr_barrier;
	arm_intr_register_fdt(&sc->sc_ic);
}

/* XXX: Allocate a per-cpu interrupt mask? */
void
aplintc_cpuinit(void)
{
#if 0
	struct aplintc_softc *sc = aplintc_sc;

	if (sc->sc_ipi_irq[0] != NULL)
		aplintc_route_irq(sc->sc_ipi_irq[0], IRQ_ENABLE, curcpu());
	if (sc->sc_ipi_irq[1] != NULL)
		aplintc_route_irq(sc->sc_ipi_irq[1], IRQ_ENABLE, curcpu());
#endif

	enable_interrupts(PSR_I);
	if (curcpu()->ci_cpl <= IPL_CLOCK)
		enable_interrupts(PSR_F);
}

void
aplintc_setipl(int ipl)
{
	struct aplintc_softc	*sc = aplintc_sc;
	struct cpu_info		*ci = curcpu();
	uint32_t		*imask;
	int			 i, psw;

	/* disable here is only to keep hardware in sync with ci->ci_cpl */
	psw = disable_interrupts(PSR_I | PSR_F);
	ci->ci_cpl = ipl;

	imask = sc->sc_imask + ipl * (roundup(sc->sc_hwnintr, 32) / 32);

	for (i = 0; i < roundup(sc->sc_hwnintr, 32) / 32; i++) {
		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    AIC_IRQ_DISABLE(i * 32), ~imask[i]);
		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    AIC_IRQ_ENABLE(i * 32), imask[i]);
	}
	__isb();

	/*
	 * Hack, because we know these are timers and there's no other
	 * way to disable them.
	 */
	psw &= ~PSR_F;
	if (ipl > IPL_CLOCK)
		psw |= PSR_F;

	restore_interrupts(psw);
}

void
aplintc_intr_enable(struct aplintc_softc *sc, int irq, int ipl)
{
	uint32_t		*imask;

	if (irq >= sc->sc_hwnintr)
		return;

	imask = sc->sc_imask + ipl * (roundup(sc->sc_hwnintr, 32) / 32);

	imask[IRQ_TO_REG32(irq)] |= (1U << IRQ_TO_REG32BIT(irq));
}

void
aplintc_intr_disable(struct aplintc_softc *sc, int irq, int ipl)
{
	uint32_t		*imask;

	if (irq >= sc->sc_hwnintr)
		return;

	imask = sc->sc_imask + ipl * (roundup(sc->sc_hwnintr, 32) / 32);

	imask[IRQ_TO_REG32(irq)] &= ~(1U << IRQ_TO_REG32BIT(irq));
}

void
aplintc_calc_mask(struct aplintc_softc *sc)
{
	int			 irq;

	for (irq = 0; irq < sc->sc_nintr; irq++)
		aplintc_calc_irq(sc, irq);
}

void
aplintc_calc_irq(struct aplintc_softc *sc, int irq)
{
#ifdef notyet
	struct cpu_info	*ci = sc->sc_handler[irq].iq_ci;
#endif
	struct intrhand	*ih;
	int max = IPL_NONE;
	int min = IPL_HIGH;
	int i;

	TAILQ_FOREACH(ih, &sc->sc_handler[irq].iq_list, ih_list) {
		if (ih->ih_ipl > max)
			max = ih->ih_ipl;

		if (ih->ih_ipl < min)
			min = ih->ih_ipl;
	}

	if (max == IPL_NONE)
		min = IPL_NONE;

	if (sc->sc_handler[irq].iq_irq_max == max &&
	    sc->sc_handler[irq].iq_irq_min == min)
		return;

	sc->sc_handler[irq].iq_irq_max = max;
	sc->sc_handler[irq].iq_irq_min = min;

#ifdef DEBUG_APLINTC
	if (min != IPL_NONE)
		printf("irq %d to block at %d %d \n", irq, max, min );
#endif

	/* Enable interrupts at lower levels, clear -> enable */
	for (i = 0; i < min; i++)
		aplintc_intr_enable(sc, irq, i);
	for (; i <= IPL_HIGH; i++)
		aplintc_intr_disable(sc, irq, i);

	/* TODO: route interrupts */

	aplintc_setipl(curcpu()->ci_cpl);
}

void
aplintc_splx(int new)
{
	struct cpu_info *ci = curcpu();

	if (ci->ci_ipending & arm_smask[new])
		arm_do_pending_intr(new);

	aplintc_setipl(new);
}

int
aplintc_spllower(int new)
{
	struct cpu_info *ci = curcpu();
	int old = ci->ci_cpl;

	aplintc_splx(new);
	return (old);
}

int
aplintc_splraise(int new)
{
	struct cpu_info	*ci = curcpu();
	int old = ci->ci_cpl;

	/*
	 * setipl must always be called because there is a race window
	 * where the variable is updated before the mask is set
	 * an interrupt occurs in that window without the mask always
	 * being set, the hardware might not get updated on the next
	 * splraise completely messing up spl protection.
	 */
	if (old > new)
		new = old;

	aplintc_setipl(new);
	return (old);
}

void
aplintc_route_irq(void *v, int enable, struct cpu_info *ci)
{
#if 0
	struct aplintc_softc	*sc = aplintc_sc;
	struct intrhand		*ih = v;

	if (enable) {
		aplintc_route(sc, ih->ih_irq, IRQ_ENABLE, ci);
		aplintc_intr_enable(sc, ih->ih_irq);
	}
#endif
}

void
aplintc_route(struct aplintc_softc *sc, int irq, int enable, struct cpu_info *ci)
{
	/* XXX: Is the Affinity register a bitmasks of CPU to target? */
	/* XXX: Are CPUs targeted by a bit, or a number/mpidr? */
	/* XXX: If we want to move IRQs to another core, does each core need to have an intmask? */
}

void
aplintc_intr_barrier(void *cookie)
{
	struct intrhand		*ih = cookie;

	sched_barrier(ih->ih_ci);
}

void
aplintc_run_handler(struct intrhand *ih, void *frame, int s)
{
	void *arg;
	int handled;

#ifdef MULTIPROCESSOR
	int need_lock;

	if (ih->ih_flags & IPL_MPSAFE)
		need_lock = 0;
	else
		need_lock = s < IPL_SCHED;

	if (need_lock)
		KERNEL_LOCK();
#endif

	if (ih->ih_arg != 0)
		arg = ih->ih_arg;
	else
		arg = frame;

	enable_interrupts(PSR_I);
	if (curcpu()->ci_cpl <= IPL_CLOCK)
		enable_interrupts(PSR_F);
	handled = ih->ih_func(arg);
	disable_interrupts(PSR_I | PSR_F);
	if (handled)
		ih->ih_count.ec_count++;

#ifdef MULTIPROCESSOR
	if (need_lock)
		KERNEL_UNLOCK();
#endif
}

void
aplintc_irq_handler(void *frame)
{
	struct aplintc_softc	*sc = aplintc_sc;
	struct intrhand		*ih;
	uint32_t		 irq;
	int			 pri, s;

	irq = bus_space_read_4(sc->sc_iot, sc->sc_ioh, AIC_IRQ_ACK);
	__isb();

#ifdef DEBUG_APLINTC
	printf("irq %d fired\n", irq);
#endif

	if (AIC_IRQ_ACK_TYPE(irq) != AIC_IRQ_ACK_TYPE_HW &&
	    AIC_IRQ_ACK_TYPE(irq) != AIC_IRQ_ACK_TYPE_IPI) {
		sc->sc_spur.ec_count++;
		return;
	}

	if (AIC_IRQ_ACK_TYPE(irq) == AIC_IRQ_ACK_TYPE_IPI) {
		irq = sc->sc_hwnintr + AIC_NUM_FIQ + AIC_IRQ_ACK_IRQNO(irq);
		membar_consumer();
	}

	if (irq >= sc->sc_hwnintr + AIC_NUM_FIQ + AIC_NUM_IPI)
		return;

	pri = sc->sc_handler[irq].iq_irq_max;
	s = aplintc_splraise(pri);
	TAILQ_FOREACH(ih, &sc->sc_handler[irq].iq_list, ih_list) {
		aplintc_run_handler(ih, frame, s);
	}
	aplintc_eoi(sc, irq);

	aplintc_splx(s);
}

void
aplintc_fiq_handler(void *frame)
{
	struct aplintc_softc	*sc = aplintc_sc;
	struct intrhand		*ih;
	uint32_t		 irq;
	int			 pri, s;

	/* Assume it was the virtual timer */
	irq = 1;

#ifdef DEBUG_APLINTC
	static int cnt = 0;
	if ((cnt++ % 100) == 0) {
		printf("irq  %d fired * _100\n", irq);
#ifdef DDB
		db_enter();
#endif
	}
#endif

	pri = sc->sc_handler[irq].iq_irq_max;
	s = aplintc_splraise(pri); /* XXX: needed? */
	TAILQ_FOREACH(ih, &sc->sc_handler[irq].iq_list, ih_list) {
		aplintc_run_handler(ih, frame, s);
	}
	aplintc_splx(s);
}

void *
aplintc_intr_establish_fdt(void *cookie, int *cell, int level,
    struct cpu_info *ci, int (*func)(void *), void *arg, char *name)
{
	struct aplintc_softc	*sc = aplintc_sc;
	int			 irq;

	/* 2nd cell contains the interrupt number */
	irq = cell[1];

	/* 1st cell contains type: 0 IRQ (0-X), 1 FIQ (0-1) */
	if (cell[0] == 0) {
		if (irq >= sc->sc_hwnintr)
			panic("%s: bogus IRQ", sc->sc_dev.dv_xname);
	} else if (cell[0] == 1) {
		if (irq >= AIC_NUM_FIQ)
			panic("%s: bogus FIQ", sc->sc_dev.dv_xname);
		/* FIQs follow HW IRQs */
		irq += sc->sc_hwnintr;
	} else
		panic("%s: bogus interrupt type", sc->sc_dev.dv_xname);

	return aplintc_intr_establish(irq, IST_LEVEL_HIGH, level, ci, func, arg, name);
}

void *
aplintc_intr_establish(int irqno, int type, int level, struct cpu_info *ci,
    int (*func)(void *), void *arg, char *name)
{
	struct aplintc_softc	*sc = aplintc_sc;
	struct intrhand		*ih;
	int			 psw;

	if (irqno < 0 || irqno >= sc->sc_nintr)
		panic("%s: bogus irqnumber %d: %s", __func__, irqno, name);

	if (ci == NULL)
		ci = &cpu_info_primary;

	ih = malloc(sizeof *ih, M_DEVBUF, M_WAITOK);
	ih->ih_func = func;
	ih->ih_arg = arg;
	ih->ih_ipl = level & IPL_IRQMASK;
	ih->ih_flags = level & IPL_FLAGMASK;
	ih->ih_irq = irqno;
	ih->ih_name = name;
	ih->ih_ci = ci;

	psw = disable_interrupts(PSR_I | PSR_F);

	if (!TAILQ_EMPTY(&sc->sc_handler[irqno].iq_list) &&
	    sc->sc_handler[irqno].iq_ci != ci) {
		free(ih, M_DEVBUF, sizeof *ih);
		restore_interrupts(psw);
		return NULL;
	}
	TAILQ_INSERT_TAIL(&sc->sc_handler[irqno].iq_list, ih, ih_list);
	sc->sc_handler[irqno].iq_ci = ci;

	if (name != NULL)
		evcount_attach(&ih->ih_count, name, &ih->ih_irq);

#ifdef DEBUG_APLINTC
	printf("%s: irq %d level %d [%s]\n", __func__, irqno, level, name);
#endif

	aplintc_calc_irq(sc, irqno);

	restore_interrupts(psw);
	return (ih);
}

void
aplintc_intr_disestablish(void *cookie)
{
	struct aplintc_softc	*sc = aplintc_sc;
	struct intrhand		*ih = cookie;
	int			 irqno = ih->ih_irq;
	int			 psw;

	psw = disable_interrupts(PSR_I | PSR_F);

	TAILQ_REMOVE(&sc->sc_handler[irqno].iq_list, ih, ih_list);
	if (ih->ih_name != NULL)
		evcount_detach(&ih->ih_count);

	aplintc_calc_irq(sc, irqno);

	restore_interrupts(psw);

	free(ih, M_DEVBUF, 0);
}

void
aplintc_eoi(struct aplintc_softc *sc, uint32_t irq)
{
	if (irq < sc->sc_hwnintr)
		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    AIC_IRQ_ENABLE(irq), 1 << IRQ_TO_REG32BIT(irq));
}

#ifdef MULTIPROCESSOR
int
aplintc_ipi_ddb(void *v)
{
	/* XXX */
#ifdef DDB
	db_enter();
#endif
	return 1;
}

int
aplintc_ipi_nop(void *v)
{
	/* Nothing to do here, just enough to wake up from WFI */
	return 1;
}

int
aplintc_ipi_combined(void *v)
{
	struct aplintc_softc *sc = v;

	if (sc->sc_ipi_reason[cpu_number()] == ARM_IPI_DDB) {
		sc->sc_ipi_reason[cpu_number()] = ARM_IPI_NOP;
		return aplintc_ipi_ddb(v);
	} else {
		return aplintc_ipi_nop(v);
	}
}

void
aplintc_send_ipi(struct cpu_info *ci, int id)
{
	struct aplintc_softc	*sc = aplintc_sc;

	if (ci == curcpu() && id == ARM_IPI_NOP)
		return;

	/* never overwrite IPI_DDB with IPI_NOP */
	if (id == ARM_IPI_DDB)
		sc->sc_ipi_reason[ci->ci_cpuid] = id;

	membar_producer();
	/* TODO: send IPI */
}
#endif
