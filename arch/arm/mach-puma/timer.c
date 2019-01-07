/*
 * Board setup Timer routines for Ericsson PUMA-1 Board
 *
 * Copyright (C) 2010 Wind River Systems, Inc.
 * Copyright (C) 2014 - 2019 MontaVista Software, LLC.
 * (Modified by Niyas Ahamed Mydeen <nmydeen@mvista.com>
 * for MontaVista Software, LLC.)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/mach/time.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach/time.h>
#include <mach/timex.h>
#include <linux/cpumask.h>
#include <linux/sched_clock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

static struct clock_event_device clockevent_PUMA;

enum {
	T0_BOT = 0, T0_TOP, T1_BOT, T1_TOP, NUM_TIMERS,
};

#define IS_TIMER1(id)    (id & 0x2)
#define IS_TIMER0(id)    (!IS_TIMER1(id))
#define IS_TIMER_TOP(id) ((id & 0x1))
#define IS_TIMER_BOT(id) (!IS_TIMER_TOP(id))

#define ID_TO_TIMER(id)         (IS_TIMER1(id) != 0)

static int timer_irqs[NUM_TIMERS] = {
	IRQ_TINT0_TINT12,
	IRQ_TINT0_TINT34,
	IRQ_TINT1_TINT12,
	IRQ_TINT1_TINT34,
};

#define readlong(r) readl((volatile void *)r)
#define writelong(d, r) writel(d, (volatile void *)r)

/*
 * This driver configures the 2 64-bit count-up timers as 4 independent
 * 32-bit count-up timers used as follows:
 *
 * T0_BOT: Timer 0, bottom:  clockevent source for hrtimers
 * T0_TOP: Timer 0, top   :  clocksource for generic timekeeping
 * T1_BOT: Timer 1, bottom:  <unused>
 * T1_TOP: Timer 1, top   :  <unused>
 */

enum {
	TID_CLOCKEVENT,
	TID_CLOCKSOURCE,
};

/* Timer register offsets */
#define PID12                        0x0
#define TIM12                        0x10
#define TIM34                        0x14
#define PRD12                        0x18
#define PRD34                        0x1c
#define TCR                          0x20
#define TGCR                         0x24
#define WDTCR                        0x28
#define INTCTL_STAT		     0x44
/* Timer register bitfields */
#define TCR_ENAMODE_DISABLE          0x0
#define TCR_ENAMODE_ONESHOT          0x1
#define TCR_ENAMODE_PERIODIC         0x2
#define TCR_ENAMODE_MASK             0x3

#define TGCR_TIMMODE_SHIFT           2
#define TGCR_TIMMODE_64BIT_GP        0x0
#define TGCR_TIMMODE_32BIT_UNCHAINED 0x1
#define TGCR_TIMMODE_64BIT_WDOG      0x2
#define TGCR_TIMMODE_32BIT_CHAINED   0x3

#define TGCR_TIM12RS_SHIFT           0
#define TGCR_TIM34RS_SHIFT           1
#define TGCR_RESET                   0x0
#define TGCR_UNRESET                 0x1
#define TGCR_RESET_MASK              0x3

#define WDTCR_WDEN_SHIFT             14
#define WDTCR_WDEN_DISABLE           0x0
#define WDTCR_WDEN_ENABLE            0x1
#define WDTCR_WDKEY_SHIFT            16
#define WDTCR_WDKEY_SEQ0             0xa5c6
#define WDTCR_WDKEY_SEQ1             0xda7e

struct timer_s {
	char *name;
	unsigned int id;
	unsigned long period;
	unsigned long opts;
	unsigned long reg_base;
	unsigned long tim_reg;
	unsigned long prd_reg;
	unsigned long enamode_shift;
	struct irqaction irqaction;
};
static struct timer_s timers[];

/* values for 'opts' field of struct timer_s */
#define TIMER_OPTS_DISABLED   0x00
#define TIMER_OPTS_ONESHOT    0x01
#define TIMER_OPTS_PERIODIC   0x02
#define TIMER_OPTS_STATE_MASK           0x07

#define TIMER_OPTS_USE_COMPARE          0x80000000
#define USING_COMPARE(t)                ((t)->opts & TIMER_OPTS_USE_COMPARE)


static int timer32_config(struct timer_s *t)
{
	u32 tcr;

	tcr = readlong(t->reg_base + TCR);

	/* disable timer */
	tcr &= ~(TCR_ENAMODE_MASK << t->enamode_shift);
	writelong(tcr, t->reg_base + TCR);

	/* reset counter to zero, set new period */
	writelong(0, t->tim_reg);
	writelong(t->period, t->prd_reg);

	/* Set enable mode */
	if (t->opts & TIMER_OPTS_ONESHOT)
		tcr |= TCR_ENAMODE_ONESHOT << t->enamode_shift;
	else if (t->opts & TIMER_OPTS_PERIODIC)
		tcr |= TCR_ENAMODE_PERIODIC << t->enamode_shift;

	writelong(tcr, t->reg_base + TCR);

	return 0;
}

static inline u32 timer32_read(struct timer_s *t)
{
	return readlong(t->tim_reg);
}

static irqreturn_t timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &clockevent_PUMA;

	evt->event_handler(evt);
	return IRQ_HANDLED;
}

/* called when 32-bit counter wraps */
static irqreturn_t freerun_interrupt(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static struct timer_s timers[] = {
	[TID_CLOCKEVENT] = {
		.name      = "clockevent",
		.opts      = TIMER_OPTS_DISABLED,
		.irqaction = {
			.flags   = IRQD_IRQ_DISABLED | IRQF_TIMER,
			.handler = timer_interrupt,
		}
	},
	[TID_CLOCKSOURCE] = {
		.name       = "free-run counter",
		.period     = ~0,
		.opts       = TIMER_OPTS_PERIODIC,
		.irqaction = {
			.flags   = IRQD_IRQ_DISABLED | IRQF_TIMER,
			.handler = freerun_interrupt,
		}
	},
};

static void __init timer_init(u32 *bases)
{
	int i;
	/* Global init of each 64-bit timer as a whole */
	for (i = 0; i < 2; i++) {
		u32 tgcr, base = (u32)bases[i];

		/* Disabled, Internal clock source */
		writelong(0, base + TCR);

		/* reset both timers, no pre-scaler for timer34 */
		tgcr = 0;
		writelong(tgcr, base + TGCR);

		/* Set both timers to unchained 32-bit */
		tgcr = TGCR_TIMMODE_32BIT_UNCHAINED << TGCR_TIMMODE_SHIFT;
		writelong(tgcr, base + TGCR);

		/* Unreset timers */
		tgcr |= (TGCR_UNRESET << TGCR_TIM12RS_SHIFT) |
			(TGCR_UNRESET << TGCR_TIM34RS_SHIFT);
		writelong(tgcr, base + TGCR);

		/* Init both counters to zero */
		writelong(0, base + TIM12);
		writelong(0, base + TIM34);
	}

	/* Init of each timer as a 32-bit timer */
	for (i = 0; i < ARRAY_SIZE(timers); i++) {
		struct timer_s *t = &timers[i];

		if (t->name) {
			t->id = i;
			t->reg_base = (unsigned long) (IS_TIMER1(t->id) ?
			       PUMA_TIMER2_VIRT : PUMA_TIMER1_VIRT);

			if (IS_TIMER_BOT(t->id)) {
				t->enamode_shift = 6;
				t->tim_reg = t->reg_base + TIM12;
				t->prd_reg = t->reg_base + PRD12;
			} else {
				t->enamode_shift = 22;
				t->tim_reg = t->reg_base + TIM34;
				t->prd_reg = t->reg_base + PRD34;
			}

			/* Register interrupt */
			t->irqaction.name = t->name;
			t->irqaction.dev_id = (void *)t;
			if (t->irqaction.handler != NULL)
				setup_irq(timer_irqs[t->id], &t->irqaction);

		}
	}
}
/*
 * clocksource
 */
static u64 read_cycles(struct clocksource *cs)
{
	struct timer_s *t = &timers[TID_CLOCKSOURCE];

	return (cycles_t)timer32_read(t);
}

static struct clocksource clocksource_PUMA = {
	.name		= "timer0_1",
	.rating		= 200,
	.read		= read_cycles,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

/*
 * clockevent
 */
static int PUMA_set_next_event(unsigned long cycles,
				  struct clock_event_device *evt)
{
	struct timer_s *t = &timers[TID_CLOCKEVENT];

	t->period = cycles;
	timer32_config(t);
	return 0;
}

static int PUMA_shutdown(struct clock_event_device *evt)
{
	struct timer_s *t = &timers[TID_CLOCKEVENT];

	t->opts &= ~TIMER_OPTS_STATE_MASK;
	t->opts |= TIMER_OPTS_DISABLED;
	return 0;
}

static int PUMA_set_oneshot(struct clock_event_device *evt)
{
	struct timer_s *t = &timers[TID_CLOCKEVENT];

	t->opts &= ~TIMER_OPTS_STATE_MASK;
		t->opts |= TIMER_OPTS_ONESHOT;
	return 0;
}
static int PUMA_set_periodic(struct clock_event_device *evt)
{
	struct timer_s *t = &timers[TID_CLOCKEVENT];

	t->period = CLOCK_TICK_RATE / (HZ);
	t->opts &= ~TIMER_OPTS_STATE_MASK;
	t->opts |= TIMER_OPTS_PERIODIC;
	timer32_config(t);
	return 0;
}

static struct clock_event_device clockevent_PUMA = {
	.name		= "timer0_0",
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 32,
	.set_next_event	= PUMA_set_next_event,
	.rating		= 200,
	.irq		= 13,
	.set_state_shutdown	= PUMA_shutdown,
	.set_state_periodic	= PUMA_set_periodic,
	.set_state_oneshot	= PUMA_set_oneshot,
};

unsigned long long notrace puma_sched_clock(void)
{
	return (unsigned long long)(timer32_read(&timers[TID_CLOCKSOURCE]));
}

static void __init puma_clk_registier(void)
{
	static char err[] __initdata = KERN_ERR
		"%s: can't register clocksource!\n";

	if (clocksource_register_hz(&clocksource_PUMA, CLOCK_TICK_RATE))
		printk(err, clocksource_PUMA.name);

	sched_clock_register(puma_sched_clock, 32, CLOCK_TICK_RATE);

	/* setup clockevent */
	clockevent_PUMA.mult = div_sc(CLOCK_TICK_RATE, NSEC_PER_SEC,
					 clockevent_PUMA.shift);

	clockevent_PUMA.max_delta_ns =
		clockevent_delta2ns(0xfffffffe, &clockevent_PUMA);

	clockevent_PUMA.min_delta_ns = 50000; /* 50 usec */
	clockevent_PUMA.cpumask =  cpumask_of(0);
	clockevents_register_device(&clockevent_PUMA);
}

void __init PUMA_timer_init(void)
{
	u32 bases[] = {(u32) PUMA_TIMER1_VIRT, (u32) PUMA_TIMER2_VIRT};
	int i;
	/* init timer hw */
	timer_init(bases);
	puma_clk_registier();
	for (i = 0; i < ARRAY_SIZE(timers); i++)
		timer32_config(&timers[i]);
}
EXPORT_SYMBOL(PUMA_timer_init);

static int __init puma_of_init_timer(struct device_node *np)
{
	u32 bases[2], i;

	bases[0] = (u32)of_iomap(np, 0);
	if (!bases[0]) {
		pr_err("Can't map timer registers\n");
		BUG();
	}

	bases[1] = (u32)of_iomap(np, 1);
	if (!bases[1]) {
		pr_err("Can't map timer registers\n");
		BUG();
	}
	np = of_find_node_by_name(NULL, "timer");
	for (i = 0; i < NUM_TIMERS; i++) {
		timer_irqs[i] = irq_of_parse_and_map(np, i);
		if (timer_irqs[i] <= 0) {
			pr_err("Failed to map timer IRQ\n");
			BUG();
		}
	}

	/* init timer hw */
	timer_init(bases);
	puma_clk_registier();

	for (i = 0; i < ARRAY_SIZE(timers); i++)
		timer32_config(&timers[i]);

	return 0;
}
CLOCKSOURCE_OF_DECLARE(puma1_timer, "ericsson,puma1-timer", puma_of_init_timer);

extern void __iomem	*wdt_base;

/* reset board using watchdog timer */
void PUMA_watchdog_reset(void)
{
	u32 tgcr, wdtcr;

	void __iomem *base;

	base = ioremap(PUMA_TIMERWD_BASE, SZ_4K);
	if (WARN_ON(!base))
		return;

	/* disable, internal clock source */
	writelong(0, base + TCR);

	/* reset timer, set mode to 64-bit watchdog, and unreset */
	tgcr = 0;
	writelong(tgcr, base + TGCR);
	tgcr = TGCR_TIMMODE_64BIT_WDOG << TGCR_TIMMODE_SHIFT;
	tgcr |= (TGCR_UNRESET << TGCR_TIM12RS_SHIFT) |
		(TGCR_UNRESET << TGCR_TIM34RS_SHIFT);
	writelong(tgcr, base + TGCR);

	/* clear counter and period regs */
	writelong(0, base + TIM12);
	writelong(0, base + TIM34);
	writelong(0, base + PRD12);
	writelong(0, base + PRD34);

	/* enable */
	wdtcr = readlong(base + WDTCR);
	wdtcr |= WDTCR_WDEN_ENABLE << WDTCR_WDEN_SHIFT;
	writelong(wdtcr, base + WDTCR);

	/* put watchdog in pre-active state */
	wdtcr = readlong(base + WDTCR);
	wdtcr = (WDTCR_WDKEY_SEQ0 << WDTCR_WDKEY_SHIFT) |
		(WDTCR_WDEN_ENABLE << WDTCR_WDEN_SHIFT);
	writelong(wdtcr, base + WDTCR);

	/* put watchdog in active state */
	wdtcr = (WDTCR_WDKEY_SEQ1 << WDTCR_WDKEY_SHIFT) |
		(WDTCR_WDEN_ENABLE << WDTCR_WDEN_SHIFT);
	writelong(wdtcr, base + WDTCR);

	/*
	 * write an invalid value to the WDKEY field to trigger
	 * a watchdog reset
	 */
	wdtcr = 0x00004000;
	writelong(wdtcr, base + WDTCR);
}
