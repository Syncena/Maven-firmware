/*
 * Copyright (c) 2022, Steve C. Woodford.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <inttypes.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "rtos.h"
#include "timer.h"
#include "linked-lists.h"
#include "console.h"
#include "zone_alloc.h"

#if (__FPU_USED)
#error "FPU not supported."
#endif

/*
 * How many interrupt vectors on this CPU?
 */
#if defined(SCnSCB_ICTR_INTLINESNUM_Msk)
#define	IRQ_ICTR_NSOURCES	(32u * ((SCnSCB->ICTR & \
				 SCnSCB_ICTR_INTLINESNUM_Msk) + 1))
#elif defined(__CORTEX_M) && (__CORTEX_M == 0U)
#define	IRQ_ICTR_NSOURCES	32u
#else
#error "Number of interrupt sources unknown"
#endif

/*
 * Each interrupt handler has one of these
 */
typedef struct {
	void (*ih_func)(void *);
	void *ih_arg;
#if defined(__CORTEX_M) && (__CORTEX_M > 0U)
	uint32_t ih_priority;
#endif
#ifdef RTOS_OPTION_IRQ_STATS
	uint32_t ih_count;	/* XXX: Keep as 32-bits for performance... */
#endif
#ifndef NDEBUG
	const char *ih_name;
#endif
} rtos_irq_handler_t;

static rtos_irq_handler_t *rtos_irq_handlers;
extern int _stack_top;

/*
 * ============================================================================
 * FreeRTOS "port" functions follow...
 * ============================================================================
 */
static volatile uint32_t _rtos_critical_nesting = 0x12345678u;
static volatile rtos_saved_ipl_t _rtos_critical_saved_ipl;

#if (configUSE_IDLE_HOOK != 0)
void vApplicationIdleHook(void);
void
vApplicationIdleHook(void)
{

#ifdef CONSOLE_FLUSH_IN_IDLE_TASK
	console_flush(false);
#endif
	asm volatile("	wfi\n" ::: "memory");
}
#endif /* (configUSE_IDLE_HOOK != 0) */

void
portYIELD(void)
{
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	_rtos_need_context_switch = 1;
	SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
	rtos_ipl_restore(ipl);
}

void
vPortEnterCritical(void)
{
	uint32_t pm = __get_PRIMASK();

	__disable_irq();

#if (__CORTEX_M > 0U)
	if (_rtos_critical_nesting++ == 0)
		_rtos_critical_saved_ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	__set_PRIMASK(pm);
#else
	if (_rtos_critical_nesting++ == 0)
		_rtos_critical_saved_ipl = pm;
#endif
}

void
vPortExitCritical(void)
{

	configASSERT(_rtos_critical_nesting > 0);

#if (__CORTEX_M > 0U)
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	if (--_rtos_critical_nesting == 0)
		rtos_ipl_restore(_rtos_critical_saved_ipl);
	__set_PRIMASK(pm);
#else
	if (--_rtos_critical_nesting == 0)
		__set_PRIMASK(_rtos_critical_saved_ipl);
#endif
}

/*
 * The initial value for the 'lr' register for all task is the address
 * of this function. This ensures that if a task's entry point returns
 * then it has something to return to. Since this is not the canonical
 * way for a task to exit, we bail.
 */
static void
task_exit(void)
{

	portDISABLE_INTERRUPTS();
	configASSERT(0);
	for(;;);
}

/*
 * Port-specific function to initialise a new task's stack
 */
StackType_t *
pxPortInitialiseStack(StackType_t *pxTopOfStack,
#if (portHAS_STACK_OVERFLOW_CHECKING == 1)
    StackType_t *pxEndOfStack,
#endif
    TaskFunction_t pxCode, void *pvParameters)
{
	xPSR_Type xpsr;

	/*
	 * We need to set up the stack as if the task's context
	 * had been saved by xPortPendSVHandler().
	 */

	/*
	 * The saved PSR must have the Thumb bit set
	 */
	xpsr.w = 0;
	xpsr.b.T = 1;

	/*
	 * This is a standard ARM Cortex-M exception frame.
	 */
	*(--pxTopOfStack) = xpsr.w;				/* xPSR */
	*(--pxTopOfStack) = (StackType_t)pxCode;		/* PC */
	*(--pxTopOfStack) = (StackType_t)task_exit;		/* LR */
	*(--pxTopOfStack) = 0;					/* R12 */
	*(--pxTopOfStack) = 0;					/* R3 */
	*(--pxTopOfStack) = 0;					/* R2 */
	*(--pxTopOfStack) = 0;					/* R1 */
	*(--pxTopOfStack) = (StackType_t)pvParameters;		/* R0 */

	/* The magic return value to switch the CPU to back to Thread Mode */
	*(--pxTopOfStack) = 0xfffffffdul;			/* EXC_RET */

	/* The remaining callee-saved registers */
	*(--pxTopOfStack) = 0;					/* R11 */
	*(--pxTopOfStack) = 0;					/* R10 */
	*(--pxTopOfStack) = 0;					/* R9 */
	*(--pxTopOfStack) = 0;					/* R8 */
	*(--pxTopOfStack) = 0;					/* R7 */
	*(--pxTopOfStack) = 0;					/* R6 */
	*(--pxTopOfStack) = 0;					/* R5 */
	*(--pxTopOfStack) = 0;					/* R4 */

#if (portHAS_STACK_OVERFLOW_CHECKING == 1)
	uintptr_t base = (uintptr_t)pxEndOfStack;
	base = (base + 7u) & ~7u;	/* Align to 8 bytes */
	*(--pxTopOfStack) = (StackType_t)base;			/* PSPLIM */
#endif

	return pxTopOfStack;
}

/*
 * Port-specific function to kick off the scheduler.
 */
BaseType_t
xPortStartScheduler(void)
{

	__disable_irq();

	/* Configure low-priority PendSV interrupt */
	NVIC_SetPriority(PendSV_IRQn, HW_IPL_SOFTIRQ);

	/* Set SysTick priority and start it up */
	SysTick->LOAD = (uint32_t)(configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1;
	SysTick->VAL = (uint32_t)(configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1;
	NVIC_SetPriority(SysTick_IRQn, HW_IPL_TIMER);
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
	    SysTick_CTRL_ENABLE_Msk;

	_rtos_critical_nesting = 0;

	__asm volatile(
	    "	.syntax	unified\n"
	    "	mov	r2, %0\n"
	    "	ldr	r0, 1f\n"
	    "	ldr	r0, [r0]\n"
	    "	ldr	r0, [r0]\n"
#if (portHAS_STACK_OVERFLOW_CHECKING == 1)
	    "	ldr	r1, [r0]\n"	/* Fetch base of stack. */
	    "	msr	psplim, r1\n"	/* Use it to initialise PSPLIM. */
	    "	movs	r1, #40\n"	/* Ditch psplim,r4-r11, lr */
#else
	    "	movs	r1, #36\n"	/* Ditch r4-r11, lr */
#endif
	    "	add	r0, r0, r1\n"
	    "	msr	psp, r0\n"	/* Set new PSP */
	    "	isb\n"
	    "	movs	r1, #2\n"
	    "	mrs	r0,control\n"
	    "	orrs	r0, r1\n"
	    "	msr	control, r0\n"	/* Switch to PSP */
	    "	isb\n"
	    "	msr	msp, r2\n"	/* Reset MSP to top of irq stack */
	    "	isb\n"
	    "	pop	{r0-r7}\n"	/* r0=arg, r5=lr, r6=pc */
	    "	mov	lr, r5\n"
#if defined(__CORTEX_M) && (__CORTEX_M > 0U)
	    "	mov	r1, #0\n"
	    "	msr	basepri, r1\n"
	    "	isb\n"
	    "	cpsie	f\n"
	    "	isb\n"
#endif
	    "	cpsie	i\n"
	    "	bx	r6\n"
	    "	.align\n"
	    "1:	.word	pxCurrentTCB\n" :: "r"(&_stack_top)
	);

	/*NOTREACHED*/
	for (;;);
}

void
vPortEndScheduler(void)
{

	portDISABLE_INTERRUPTS();
	for(;;);
}


/*
 * ============================================================================
 * My own 'rtos' wrapper stuff follows.
 * ============================================================================
 */

static void
rtos_bad_interrupt(void *arg)
{
	uint32_t vecnum = __get_IPSR();

	(void) arg;

	NVIC_DisableIRQ(vecnum);
	NVIC_ClearPendingIRQ(vecnum);

#ifdef RTOS_DEBUG
	printf("Spurious interrupt: vector %" PRIu32 "\n", vecnum);
#endif /* RTOS_DEBUG */
}

void *
rtos_irq_register_impl(
#ifndef NDEBUG
    const char *name,
#endif
    int vecnum, uint32_t pri, void (*func)(void *), void *arg)
{
	rtos_irq_handler_t *ih;

	assert(pri < portINTERRUPT_PRIORITIES);
	assert(vecnum >= 0 && (unsigned int)vecnum < SOC_IRQ_NSOURCES);
	assert(rtos_irq_handlers != NULL);

	ih = &rtos_irq_handlers[vecnum];
	assert(ih->ih_func == rtos_bad_interrupt);

	ih->ih_func = func;
	ih->ih_arg = arg;
#if defined(__CORTEX_M) && (__CORTEX_M > 0U)
	ih->ih_priority = pri;
#else
	(void) pri;
#endif
#ifndef NDEBUG
	if (name != NULL)
		ih->ih_name = zone_strdup(name);
	else
		ih->ih_name = NULL;
#endif

	NVIC_SetPriority((IRQn_Type)vecnum, pri);
	NVIC_EnableIRQ((IRQn_Type)vecnum);

	return ih;
}

void
rtos_irq_unregister(void *handle)
{
	rtos_irq_handler_t *ih = handle;
	uint32_t vector;

	assert(ih != NULL);
	assert(ih >= rtos_irq_handlers);
	assert(ih < &rtos_irq_handlers[SOC_IRQ_NSOURCES]);

	/* Pointer arithmetic */
	vector = (uint32_t)(ih - rtos_irq_handlers);

	NVIC_DisableIRQ((IRQn_Type) vector);
	NVIC_ClearPendingIRQ((IRQn_Type) vector);

	ih->ih_func = rtos_bad_interrupt;
	ih->ih_arg = ih;
#ifndef NDEBUG
	if (ih->ih_name != NULL)
		zone_free((void *)(uintptr_t)ih->ih_name);
	ih->ih_name = NULL;
#endif
}

#ifdef RTOS_OPTION_SOFT_IRQ
static volatile bool _rtos_need_soft_interrupt;

typedef struct rtos_soft_irq_handler {
	LIST_ENTRY(rtos_soft_irq_handler) sih_entry;
	void (*sih_func)(void *);
	void *sih_arg;
	bool sih_pending;
#ifdef RTOS_OPTION_IRQ_STATS
	uint32_t sih_count;
#endif
#ifndef NDEBUG
	const char *sih_name;
#endif
} *rtos_soft_irq_handler_t;

static LIST_HEAD(, rtos_soft_irq_handler) soft_irq_head =
    LIST_HEAD_INITIALIZER(&soft_irq_head);

void *
rtos_soft_irq_register_impl(
#ifndef NDEBUG
    const char *name,
#endif
    void (*func)(void *), void *arg)
{
	rtos_soft_irq_handler_t sih;
	rtos_saved_ipl_t ipl;

	if ((sih = zone_malloc(sizeof(*sih))) == NULL)
		return NULL;

	sih->sih_func = func;
	sih->sih_arg = arg;
	sih->sih_pending = false;
#ifdef RTOS_OPTION_IRQ_STATS
	sih->sih_count = 0;
#endif
#ifndef NDEBUG
	if (name != NULL)
		sih->sih_name = zone_strdup(name);
	else
		sih->sih_name = NULL;
#endif
	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	LIST_INSERT_HEAD(&soft_irq_head, sih, sih_entry);
	rtos_ipl_restore(ipl);

	return sih;
}

void
rtos_soft_irq_unregister(void *cookie)
{
	rtos_soft_irq_handler_t sih = cookie;
	rtos_saved_ipl_t ipl;
#ifndef NDEBUG
	const char *name;
#endif

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	LIST_REMOVE(sih, sih_entry);
#ifndef NDEBUG
	name = sih->sih_name;
	sih->sih_name = NULL;
#endif
	rtos_ipl_restore(ipl);

#ifndef NDEBUG
	if (name != NULL)
		zone_free((void *)(uintptr_t)name);
#endif
	zone_free(sih);
}

void
rtos_soft_irq_schedule(void *cookie)
{
	rtos_soft_irq_handler_t sih = cookie;

	/*
	 * Soft interrupts are handled via the PendSV exception.
	 */
	sih->sih_pending = true;
	_rtos_need_soft_interrupt = true;
	SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

static void FAST_RAMFUNC rtos_handle_soft_interrupts(void);
static void
rtos_handle_soft_interrupts(void)
{
	rtos_soft_irq_handler_t sih;
	bool got_work;

	assert(__get_IPSR() != 0);

	for (;;) {
		__disable_irq();
		got_work = _rtos_need_soft_interrupt;
		_rtos_need_soft_interrupt = false;
		__enable_irq();

		if (got_work == false)
			break;

		/* Iterate the software interrupt queue. */
		LIST_FOREACH(sih, &soft_irq_head, sih_entry) {
			if (sih->sih_pending == false)
				continue;

			sih->sih_pending = false;
			sih->sih_func(sih->sih_arg);
#ifdef RTOS_OPTION_IRQ_STATS
			sih->sih_count++;
#endif
		}
	}
}
#endif	/* RTOS_OPTION_SOFT_IRQ */

#ifdef RTOS_OPTION_IRQ_STATS
void _rtos_print_irq_stats(FILE *);
void
_rtos_print_irq_stats(FILE *os)
{
	rtos_irq_handler_t *ih;
	unsigned int irq, cnt;
	struct timeval tv;

	cnt = SOC_IRQ_NSOURCES;
	timer_get_mono_time(&tv);

#ifndef NDEBUG
	fprintf(os, "\n%-10s%-14s%-11s %s\n",
	    "IRQNUM", "NAME", "COUNT", "RATE/Second");
#else
	fprintf(os, "\n%-10s%-11s %s\n",
	    "IrqNum", "Count", "Rate");
#endif

	for (irq = 0, ih = &rtos_irq_handlers[0];
	    irq < cnt; irq++, ih++) {
		unsigned int whole, frac;

		if (ih->ih_func == NULL || ih->ih_func == rtos_bad_interrupt)
			continue;

		if (tv.tv_sec != 0) {
			whole = (unsigned int)(ih->ih_count / tv.tv_sec);
			frac = (unsigned int)(((ih->ih_count % tv.tv_sec) *
			    100u) / tv.tv_sec);
		} else {
			whole = 0;
			frac = 0;
		}

		fprintf(os, "irq%03u    ", irq);

#ifndef NDEBUG
		if (ih->ih_name != NULL)
			fprintf(os, "%-14s", ih->ih_name);
#endif

		fprintf(os, "%-11" PRIu32 " %u.%02u\n", ih->ih_count, whole,
		    frac);
	}

#ifdef RTOS_OPTION_SOFT_IRQ
	rtos_soft_irq_handler_t sih;

	irq = 0;
	LIST_FOREACH(sih, &soft_irq_head, sih_entry) {
		unsigned int whole, frac;

		if (tv.tv_sec != 0) {
			whole = (unsigned int)(sih->sih_count / tv.tv_sec);
			frac = (unsigned int)(((sih->sih_count % tv.tv_sec) *
			    100u) / tv.tv_sec);
		} else {
			whole = 0;
			frac = 0;
		}

		fprintf(os, "softirq%u  ", irq++);

#ifndef NDEBUG
		if (sih->sih_name != NULL)
			fprintf(os, "%-14s", sih->sih_name);
#endif

		fprintf(os, "%-11" PRIu32 " %u.%02u\n", sih->sih_count, whole,
		    frac);
	}
#endif /* RTOS_OPTION_SOFT_IRQ */
}
#endif /* RTOS_OPTION_IRQ_STATS */

static void
init_irq_support(void)
{
	rtos_irq_handler_t *ih;

	assert((IRQ_ICTR_NSOURCES / 32u) <= (SOC_IRQ_NSOURCES + 31) / 32u);

	rtos_irq_handlers = ih = zone_calloc(SOC_IRQ_NSOURCES, sizeof(*ih));
	assert(ih != NULL);

	for (unsigned int i = 0; i < SOC_IRQ_NSOURCES; i++, ih++) {
		ih->ih_func = rtos_bad_interrupt;
		ih->ih_arg = ih;
	}
}

/*
 * Fault recovery support.
 */
static jmp_buf *cpu_fault_jmp_buf;

bool
rtos_catch_fault(void (*func)(void *), void *arg)
{
	jmp_buf jb;

	/* Save state in the jmp_buf */
	if (setjmp(jb) != 0) {
		/*
		 * A fault occurred while running 'func'.
		 * Scheduler was resumed in cpu_fault_recover().
		 */
		return true;
	}

	rtos_scheduler_suspend();
	cpu_fault_jmp_buf = &jb;

	/*
	 * If a fault occurs within 'func', control will return to the
	 * setjmp() above.
	 */
	(func)(arg);

	/*
	 * If we reach here, no fault occurred.
	 */
	cpu_fault_jmp_buf = NULL;
	rtos_scheduler_resume();

	return false;
}

static void
cpu_fault_recover_tramp(jmp_buf *jb)
{

	/* Couldn't be simpler... */
	rtos_scheduler_resume();
	longjmp(*jb, 1);
}

/*
 * Layout of registers automatically pushed on the stack following
 * an exception/interrupt.
 */
typedef struct {
	uint32_t	ef_r0;
	uint32_t	ef_r1;
	uint32_t	ef_r2;
	uint32_t	ef_r3;
	uint32_t	ef_r12;
	uint32_t	ef_lr;
	uint32_t	ef_pc;
	uint32_t	ef_xpsr;
} exception_trap_frame_t;

/*
 * This is the stack layout following a fatal exception
 */
typedef struct {
#if defined(__CORTEX_M) && (__CORTEX_M > 0U)
	uint32_t	tf_primask;
	uint32_t	tf_r4;
	uint32_t	tf_r5;
	uint32_t	tf_r6;
	uint32_t	tf_r7;
	uint32_t	tf_r8;
	uint32_t	tf_r9;
	uint32_t	tf_r10;
	uint32_t	tf_r11;
	uint32_t	tf_exc_return;
#else
	uint32_t	tf_r8;
	uint32_t	tf_r9;
	uint32_t	tf_r10;
	uint32_t	tf_r11;
	uint32_t	tf_exc_return;
	uint32_t	tf_primask;
	uint32_t	tf_r4;
	uint32_t	tf_r5;
	uint32_t	tf_r6;
	uint32_t	tf_r7;
#endif
} fatal_trap_frame_t;

static bool
cpu_fault_recover(exception_trap_frame_t *ef, fatal_trap_frame_t *tf)
{
	jmp_buf *jb;

	(void) tf;

	if ((jb = cpu_fault_jmp_buf) != NULL) {
		/*
		 * Attempt to recover from this fault.
		 * Note: We clear cpu_fault_jmp_buf here rather than in
		 * in cpu_fault_recover_tramp() because:
		 *  a) Interrupts are already disabled here,
		 *  b) Another fault might occur as soon as we return from
		 *     the exception due to stack overflow (armv8m:psplim),
		 *     for example. If we didn't unhook here, we'd end up
		 *     in an infinite loop.
		 */
		cpu_fault_jmp_buf = NULL;
		ef->ef_pc = (uint32_t)(uintptr_t)cpu_fault_recover_tramp;
		ef->ef_r0 = (uint32_t)(uintptr_t)jb;
		return true;
	}

	return false;
}

static void vector_cpu_exc_fatal(exception_trap_frame_t *,
		fatal_trap_frame_t *) __attribute__ ((used));
static void
vector_cpu_exc_fatal(exception_trap_frame_t *ef, fatal_trap_frame_t *tf)
{

	if (cpu_fault_recover(ef, tf))
		return;

#if (configQUIET_FATAL_EXCEPTION == 0)
	extern UBaseType_t _rtos_scheduler_started;
	const char *exc_string;
	uint32_t msp, psp;
	uint8_t on_psp;

	on_psp = (tf->tf_exc_return & 4) != 0;

	if (on_psp) {
		psp = (uint32_t)(tf + 1);
		if ((ef->ef_xpsr & (1u << 9)) != 0)
			psp += 4;
		msp = __get_MSP();
	} else {
		msp = (uint32_t)(tf + 1);
		if ((ef->ef_xpsr & (1u << 9)) != 0)
			msp += 4;
		psp = __get_PSP();
	}

	switch (__get_IPSR()) {
	case 2:
		exc_string = "NMI";
		break;
	case 3:
		exc_string = "HardFault";
		break;
	case 4:
		exc_string = "MemManage";
		break;
	case 5:
		exc_string = "BusFault";
		break;
	case 6:
		exc_string = "UsageFault";
		break;
	case 14:
		exc_string = "PendSV";
		break;
	default:
		exc_string = "Whit!?!";
		break;
	}

	printf("\n\nFatal exception %" PRIu32 " (%s) at PC %08" PRIx32 " in "
	    "task '%s'\n", __get_IPSR(), exc_string, ef->ef_pc,
	    _rtos_scheduler_started ? pcTaskGetName(NULL) : "_INIT_");
	printf("     xPSR %08" PRIx32 "  PRIMASK %" PRIu32
#if defined(__CORTEX_M) && (__CORTEX_M > 0U)
	    "  BASEPRI %08" PRIx32
#endif
	    "  EXC_RET %08" PRIx32 "\n", ef->ef_xpsr, tf->tf_primask,
#if defined(__CORTEX_M) && (__CORTEX_M > 0U)
	    __get_BASEPRI(),
#endif
	    tf->tf_exc_return);
#if defined(__CORTEX_M) && (__CORTEX_M > 0U)
	printf("    FRAME %08" PRIx32 "     CFSR %08" PRIx32
	    "     HFSR %08" PRIx32 "     BFAR %08" PRIx32 "\n",
	    (uint32_t)tf, SCB->CFSR, SCB->HFSR, SCB->BFAR);
	printf("     DFSR %08" PRIx32 "    MMFAR %08" PRIx32
	    "     AFSR %08" PRIx32 "\n", SCB->DFSR, SCB->MMFAR, SCB->AFSR);
#else
	printf("    FRAME %08" PRIx32 "\n", (uint32_t)tf);
#endif
	printf("       r0 %08" PRIx32 "       r1 %08" PRIx32
	    "       r2 %08" PRIx32 "       r3 %08" PRIx32 "\n",
	    ef->ef_r0, ef->ef_r1, ef->ef_r2, ef->ef_r3);
	printf("       r4 %08" PRIx32 "       r5 %08" PRIx32
	    "       r6 %08" PRIx32 "       r7 %08" PRIx32 "\n",
	    tf->tf_r4, tf->tf_r5, tf->tf_r6, tf->tf_r7);
	printf("       r8 %08" PRIx32 "       r9 %08" PRIx32
	    "      r10 %08" PRIx32 "      r11 %08" PRIx32 "\n",
	    tf->tf_r8, tf->tf_r9, tf->tf_r10, tf->tf_r11);
	printf("      r12 %08" PRIx32 "       lr %08" PRIx32
	    "     %cMSP %08" PRIx32 "     %cPSP %08" PRIx32 "\n",
	    ef->ef_r12, ef->ef_lr, on_psp ? ' ' : '*', msp,
	    on_psp ? '*' : ' ', psp);
#else
	(void) tf;
#endif /* FATAL_EXCEPTION_QUIET */

	for (;;);
}

/*
 * Allow application code to override the default behaviour of
 * fatal exceptions by defining the defaults as weak.
 */
static void vector_cpu_exc_fatal_trampoline(void) __attribute__ ((naked));
static void
vector_cpu_exc_fatal_trampoline(void)
{

#if defined(__CORTEX_M) && (__CORTEX_M > 0U)
	__asm __volatile(
	    "	.syntax	unified\n"
	    "	tst	lr, #0x4\n"
	    "	ite	eq\n"		/* 'eq' if MSP was active. */
	    "	moveq	r0, sp\n"	/* MSP -> R0 */
	    "	mrsne	r0, psp\n"	/* PSP -> R0 */
	    "	mrs	r3, primask\n"
	    "	cpsid	i\n"
	    "	push	{r3-r11,lr}\n"
	    "	mov	r1, sp\n"
	    "	bl	vector_cpu_exc_fatal\n"
	    "	pop	{r3-r11,lr}\n"
	    "	msr	primask, r3\n"
	    "	isb\n"
	    "	bx	lr\n"
	    : : : "memory");
#else
	__asm __volatile(
	    "	.syntax	unified\n"
	    "	mrs	r0, psp\n"	/* Assume PSP was active. */
	    "	subs	r0, r0, #40\n"
	    "	mov	r1, lr\n"
	    "	lsls	r1, r1, #29\n"	/* Were we right? */
	    "	bmi	1f\n"		/* Jump if so. */
	    "	mov	r0, sp\n"	/* MSP was active. */
	    "1:	mrs	r3, primask\n"
	    "	cpsid	i\n"
	    "	push	{r3-r7}\n"
	    "	mov	r1, r8\n"
	    "	mov	r2, r9\n"
	    "	mov	r3, r10\n"
	    "	mov	r4, r11\n"
	    "	mov	r5, lr\n"
	    "	push	{r1-r5}\n"
	    "	mov	r1, sp\n"
	    "	bl	vector_cpu_exc_fatal\n"
	    "	pop	{r1-r5}\n"
	    "	mov	lr, r5\n"
	    "	mov	r11, r4\n"
	    "	mov	r10, r3\n"
	    "	mov	r9, r2\n"
	    "	mov	r8, r1\n"
	    "	pop	{r3-r7}\n"
	    "	msr	primask, r3\n"
	    "	isb\n"
	    "	bx	lr\n"
	    : : : "memory");
#endif /* defined(__CORTEX_M) && (__CORTEX_M > 0U) */
}
#define	FATAL	__attribute__((weak, alias("vector_cpu_exc_fatal_trampoline")))
FATAL void vector_cpu_exc_nmi(void);
FATAL void vector_cpu_exc_hard_fault(void);
FATAL void vector_cpu_exc_mm_fault(void);
FATAL void vector_cpu_exc_bus_fault(void);
FATAL void vector_cpu_exc_usage_fault(void);
FATAL void vector_cpu_exc_resvd7(void);
FATAL void vector_cpu_exc_resvd8(void);
FATAL void vector_cpu_exc_resvd9(void);
FATAL void vector_cpu_exc_resvd10(void);
FATAL void vector_cpu_exc_svccall(void);
FATAL void vector_cpu_exc_debug_mon(void);
FATAL void vector_cpu_exc_resvd13(void);
#undef FATAL

struct tcb;
typedef struct tcb *TCB_t;

static StackType_t FAST_RAMFUNC
    cpu_exc_pendsv(StackType_t sp) __attribute__ ((used));
static StackType_t
cpu_exc_pendsv(StackType_t sp)
{
	extern TCB_t * volatile pxCurrentTCB;
	rtos_saved_ipl_t ipl;
	StackType_t rv = 0;
	TCB_t *t;

#ifdef	RTOS_OPTION_SOFT_IRQ
	if (_rtos_need_soft_interrupt)
		rtos_handle_soft_interrupts();
#endif	/* RTOS_OPTION_SOFT_IRQ */

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	if (_rtos_need_context_switch != 0) {
		_rtos_need_context_switch = 0;

		/* First element of pxCurrentTCB is the current SP. Save it. */
		t = pxCurrentTCB;
		*(volatile StackType_t *)t = sp;

		vTaskSwitchContext();

		/* Return the new task's stack pointer, if switch required. */
		if (t != pxCurrentTCB)
			rv = *(volatile StackType_t *)pxCurrentTCB;
	}

	rtos_ipl_restore(ipl);

	return rv;
}

static void FAST_RAMFUNC
    vector_cpu_exc_pendsv(void) __attribute__ ((naked));
static void
vector_cpu_exc_pendsv(void)
{

	/*
	 * Because PendSV is the lowest-priority interrupt in the
	 * system, we can only be entered if the processor was
	 * previously in thread mode. That is, we will never nest.
	 * Thus the automatically-saved processor state is already
	 * on the Process Stack.
	 */
	__asm volatile(
	    /*
	     * Grab Process Stack Pointer; we need to save the remaining
	     * state on it.
	     */
	    "	.syntax	unified\n"

	    /* We need to stash LR, and free up a callee-saved register. */
	    "	push	{r4,lr}\n"

	    "	mrs	r4, psp\n"
#if defined(__CORTEX_M) && (__CORTEX_M > 0U)
#if (portHAS_STACK_OVERFLOW_CHECKING == 1)
	    "	subs	r0, r4, #40\n"
#else
	    "	subs	r0, r4, #36\n"
#endif
#else
	    "	subs	r4, #36\n"
	    "	mov	r0, r4\n"
#endif

	    /*
	     * Handle switching in C. Parameter is interrupted task's SP,
	     * accounting for the frame we will save on it if we need to
	     * switch contexts.
	     */
	    "	bl	cpu_exc_pendsv\n"

	    /*
	     * If cpu_exc_pendsv() returned NULL, then a context-switch did
	     * not happen; we return to the interrupted task without any
	     * further ado.
	     */
#if defined(__CORTEX_M) && (__CORTEX_M > 0U)
	    "	cbnz	r0, 1f\n"	/* Jump if switch needed. */
#else
	    "	cmp	r0, #0\n"
	    "	bne	1f\n"		/* Jump if switch needed. */
#endif
	    "	pop	{r4,pc}\n"	/* Return from interrupt. */

	    /*
	     * We need to context-switch.
	     */
	    "1:\n"

	    /*
	     * Current state:
	     *      R0: New task's SP.
	     *      R4: On M0/M0+, this points to the start of the gap we
	     *          opened up on the interrupted task's stack. On other
	     *          CPU cores, this holds the task's current SP.
	     *  [SP+0]: Interrupted task's R4.
	     *  [SP+4]: Interrupt vector return code (LR).
	     *
	     *  We now need to save the interrupted task's remaining
	     *  registers; R4 to R11, and LR.
	     */
#if defined(__CORTEX_M) && (__CORTEX_M > 0U)
	    "	pop	{r2,lr}\n"		/* Saved R4->R2, and LR */
#if (portHAS_STACK_OVERFLOW_CHECKING == 1)
	    /* Fetch task's PSPLIM and stash it along with the state. */
	    "	mrs	r1, psplim\n"
	    "	stmdb	r4!, {r1,r2,r5-r11,lr}\n"
	    /*
	     * Update PSP here. If, by saving state, we blew the task's
	     * stack then this will force a hard fault.
	     */
	    "	msr	psp, r4\n"
	    "	isb\n"
#else
	    "	stmdb	r4!, {r2,r5-r11,lr}\n"
#endif
#else /* Cortex-M0/M0+ */
	    /* ARMv6M is a little more convoluted to get the same layout. */
	    "	pop	{r2}\n"			/* Saved R4->R2. */
	    "	stmia	r4!, {r2,r5-r7}\n"	/* Save R4 to R7. */
	    "	mov	r1, r8\n"
	    "	mov	r2, r9\n"
	    "	pop	{r7}\n"			/* Saved LR. */
	    "	mov	r5, r10\n"
	    "	mov	r6, r11\n"
	    "	stmia	r4!, {r1-r2,r5-r7}\n"	/* Save R8 to R11, and LR */
#endif

	    /*
	     * Interrupted task's context has been saved. Next task's SP
	     * is in R0, so go restore its context.
	     */

#if defined(__CORTEX_M) && (__CORTEX_M > 0U)
#if (portHAS_STACK_OVERFLOW_CHECKING == 1)
	    "	ldmia	r0!, {r1,r4-r11,lr}\n"
	    "	msr	psplim, r1\n"
#else
	    "	ldmia	r0!, {r4-r11,lr}\n"
#endif
#else /* defined(__CORTEX_M) && (__CORTEX_M > 0U) */
	    "	adds	r0, r0, #16\n"
	    "	ldmia	r0!, {r2-r6}\n"	/* Fetch saved R8 to R11, and LR */
	    "	mov	r8, r2\n"
	    "	mov	r9, r3\n"
	    "	mov	r10, r4\n"
	    "	mov	r11, r5\n"
	    "	mov	lr, r6\n"
	    "	movs	r1, #36\n"
	    "	subs	r1, r0, r1\n"
	    "	ldmia	r1!, {r4-r7}\n"	/* Fetch saved R4 to R7. */
#endif /* defined(__CORTEX_M) && (__CORTEX_M > 0U) */

	    /*
	     * New task's stack pointer -> Process Stack, sync the
	     * instruction stream and exit back to Thread mode.
	     */
	    "	msr	psp, r0\n"
	    "	isb\n"
	    "	bx	lr\n"
	    );
}

static void FAST_RAMFUNC vector_cpu_exc_systick(void);
static void
vector_cpu_exc_systick(void)
{
	rtos_saved_ipl_t ipl;

	assert(_rtos_critical_nesting == 0);

	timer_tick();

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	if (xTaskIncrementTick()) {
		_rtos_need_context_switch = 1;
		SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
	}

	rtos_ipl_restore(ipl);
}

static FAST_RAMFUNC void vector_cpu_irq_trampoline(void);
static void
vector_cpu_irq_trampoline(void)
{
	rtos_irq_handler_t *ih;
	uint32_t ipsr = __get_IPSR();

	assert(ipsr >= 16u);
	ipsr -= 16u;
	assert(ipsr < SOC_IRQ_NSOURCES);

	/*
	 * XXX: I'd assert that this is valid, but if the CPU is lying
	 * about the current IPSR, then we have other problems.
	 */
	ih = &rtos_irq_handlers[ipsr];

	assert(ih != NULL);

#if defined(__CORTEX_M) && (__CORTEX_M > 0U)
	if (ih->ih_priority >= HW_IPL_SCHEDULER)
		assert(_rtos_critical_nesting == 0);
#endif

	/* Invoke the handler  */
	(ih->ih_func)(ih->ih_arg);

#ifdef RTOS_OPTION_IRQ_STATS
	ih->ih_count++;
#endif

#if defined(__CORTEX_M) && (__CORTEX_M > 0U)
	/*
	 * Interrupts with a higher priority than HW_IPL_SCHEDULER are not
	 * permitted to invoke any rtos functions. Therefore it is not
	 * possible for _rtos_need_context_switch to change state.
	 * Note: ARM CortexM priority numbering is upside down...
	 */
	if (ih->ih_priority >= HW_IPL_SCHEDULER)
#endif
	{
		if (_rtos_need_context_switch)
			SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
	}
}

#if defined(configASSERT) && (configASSERT != 0)
void
vPortValidateInterruptPriority(void)
{
	rtos_irq_handler_t *ih;
	uint32_t ipsr = __get_IPSR();

	assert(ipsr != 0);

	if (ipsr < 16u) {
		uint32_t pri = __get_BASEPRI() >> (8u - __NVIC_PRIO_BITS);
		assert(pri >= HW_IPL_SCHEDULER);
		return;
	}

	ipsr -= 16u;
	assert(ipsr < SOC_IRQ_NSOURCES);

	ih = &rtos_irq_handlers[ipsr];
	assert(ih != NULL);

	assert(ih->ih_priority >= HW_IPL_SCHEDULER);
}
#endif /* configASSERT */

#define	HWIRQ	__attribute__((weak, alias("vector_cpu_irq_trampoline")))

#define	VECTOR_TABLE_ALIGN	128

HWIRQ void vector_cpu_irq0(void);
HWIRQ void vector_cpu_irq1(void);
HWIRQ void vector_cpu_irq2(void);
HWIRQ void vector_cpu_irq3(void);
HWIRQ void vector_cpu_irq4(void);
HWIRQ void vector_cpu_irq5(void);
HWIRQ void vector_cpu_irq6(void);
HWIRQ void vector_cpu_irq7(void);
#if (SOC_IRQ_NSOURCES > 8)
HWIRQ void vector_cpu_irq8(void);
HWIRQ void vector_cpu_irq9(void);
HWIRQ void vector_cpu_irq10(void);
HWIRQ void vector_cpu_irq11(void);
HWIRQ void vector_cpu_irq12(void);
HWIRQ void vector_cpu_irq13(void);
HWIRQ void vector_cpu_irq14(void);
HWIRQ void vector_cpu_irq15(void);
#endif
#if (SOC_IRQ_NSOURCES > 16)
HWIRQ void vector_cpu_irq16(void);
HWIRQ void vector_cpu_irq17(void);
HWIRQ void vector_cpu_irq18(void);
HWIRQ void vector_cpu_irq19(void);
HWIRQ void vector_cpu_irq20(void);
HWIRQ void vector_cpu_irq21(void);
HWIRQ void vector_cpu_irq22(void);
HWIRQ void vector_cpu_irq23(void);
#endif
#if (SOC_IRQ_NSOURCES > 24)
HWIRQ void vector_cpu_irq24(void);
HWIRQ void vector_cpu_irq25(void);
HWIRQ void vector_cpu_irq26(void);
HWIRQ void vector_cpu_irq27(void);
HWIRQ void vector_cpu_irq28(void);
HWIRQ void vector_cpu_irq29(void);
HWIRQ void vector_cpu_irq30(void);
HWIRQ void vector_cpu_irq31(void);
#endif
#if (SOC_IRQ_NSOURCES > 32)
#undef	VECTOR_TABLE_ALIGN
#define	VECTOR_TABLE_ALIGN	256
HWIRQ void vector_cpu_irq32(void);
HWIRQ void vector_cpu_irq33(void);
HWIRQ void vector_cpu_irq34(void);
HWIRQ void vector_cpu_irq35(void);
HWIRQ void vector_cpu_irq36(void);
HWIRQ void vector_cpu_irq37(void);
HWIRQ void vector_cpu_irq38(void);
HWIRQ void vector_cpu_irq39(void);
#endif
#if (SOC_IRQ_NSOURCES > 40)
HWIRQ void vector_cpu_irq40(void);
HWIRQ void vector_cpu_irq41(void);
HWIRQ void vector_cpu_irq42(void);
HWIRQ void vector_cpu_irq43(void);
HWIRQ void vector_cpu_irq44(void);
HWIRQ void vector_cpu_irq45(void);
HWIRQ void vector_cpu_irq46(void);
HWIRQ void vector_cpu_irq47(void);
#endif
#if (SOC_IRQ_NSOURCES > 48)
HWIRQ void vector_cpu_irq48(void);
HWIRQ void vector_cpu_irq49(void);
HWIRQ void vector_cpu_irq50(void);
HWIRQ void vector_cpu_irq51(void);
HWIRQ void vector_cpu_irq52(void);
HWIRQ void vector_cpu_irq53(void);
HWIRQ void vector_cpu_irq54(void);
HWIRQ void vector_cpu_irq55(void);
#endif
#if (SOC_IRQ_NSOURCES > 56)
HWIRQ void vector_cpu_irq56(void);
HWIRQ void vector_cpu_irq57(void);
HWIRQ void vector_cpu_irq58(void);
HWIRQ void vector_cpu_irq59(void);
HWIRQ void vector_cpu_irq60(void);
HWIRQ void vector_cpu_irq61(void);
HWIRQ void vector_cpu_irq62(void);
HWIRQ void vector_cpu_irq63(void);
#endif
#if (SOC_IRQ_NSOURCES > 64)
#undef	VECTOR_TABLE_ALIGN
#define	VECTOR_TABLE_ALIGN	512
HWIRQ void vector_cpu_irq64(void);
HWIRQ void vector_cpu_irq65(void);
HWIRQ void vector_cpu_irq66(void);
HWIRQ void vector_cpu_irq67(void);
HWIRQ void vector_cpu_irq68(void);
HWIRQ void vector_cpu_irq69(void);
HWIRQ void vector_cpu_irq70(void);
HWIRQ void vector_cpu_irq71(void);
#endif
#if (SOC_IRQ_NSOURCES > 72)
HWIRQ void vector_cpu_irq72(void);
HWIRQ void vector_cpu_irq73(void);
HWIRQ void vector_cpu_irq74(void);
HWIRQ void vector_cpu_irq75(void);
HWIRQ void vector_cpu_irq76(void);
HWIRQ void vector_cpu_irq77(void);
HWIRQ void vector_cpu_irq78(void);
HWIRQ void vector_cpu_irq79(void);
#endif
#if (SOC_IRQ_NSOURCES > 80)
HWIRQ void vector_cpu_irq80(void);
HWIRQ void vector_cpu_irq81(void);
HWIRQ void vector_cpu_irq82(void);
HWIRQ void vector_cpu_irq83(void);
HWIRQ void vector_cpu_irq84(void);
HWIRQ void vector_cpu_irq85(void);
HWIRQ void vector_cpu_irq86(void);
HWIRQ void vector_cpu_irq87(void);
#endif
#if (SOC_IRQ_NSOURCES > 88)
HWIRQ void vector_cpu_irq88(void);
HWIRQ void vector_cpu_irq89(void);
HWIRQ void vector_cpu_irq90(void);
HWIRQ void vector_cpu_irq91(void);
HWIRQ void vector_cpu_irq92(void);
HWIRQ void vector_cpu_irq93(void);
HWIRQ void vector_cpu_irq94(void);
HWIRQ void vector_cpu_irq95(void);
#endif
#if (SOC_IRQ_NSOURCES > 96)
HWIRQ void vector_cpu_irq96(void);
HWIRQ void vector_cpu_irq97(void);
HWIRQ void vector_cpu_irq98(void);
HWIRQ void vector_cpu_irq99(void);
HWIRQ void vector_cpu_irq100(void);
HWIRQ void vector_cpu_irq101(void);
HWIRQ void vector_cpu_irq102(void);
HWIRQ void vector_cpu_irq103(void);
#endif
#if (SOC_IRQ_NSOURCES > 104)
HWIRQ void vector_cpu_irq104(void);
HWIRQ void vector_cpu_irq105(void);
HWIRQ void vector_cpu_irq106(void);
HWIRQ void vector_cpu_irq107(void);
HWIRQ void vector_cpu_irq108(void);
HWIRQ void vector_cpu_irq109(void);
HWIRQ void vector_cpu_irq110(void);
HWIRQ void vector_cpu_irq111(void);
#endif
#if (SOC_IRQ_NSOURCES > 112)
HWIRQ void vector_cpu_irq112(void);
HWIRQ void vector_cpu_irq113(void);
HWIRQ void vector_cpu_irq114(void);
HWIRQ void vector_cpu_irq115(void);
HWIRQ void vector_cpu_irq116(void);
HWIRQ void vector_cpu_irq117(void);
HWIRQ void vector_cpu_irq118(void);
HWIRQ void vector_cpu_irq119(void);
#endif
#if (SOC_IRQ_NSOURCES > 120)
HWIRQ void vector_cpu_irq120(void);
HWIRQ void vector_cpu_irq121(void);
HWIRQ void vector_cpu_irq122(void);
HWIRQ void vector_cpu_irq123(void);
HWIRQ void vector_cpu_irq124(void);
HWIRQ void vector_cpu_irq125(void);
HWIRQ void vector_cpu_irq126(void);
HWIRQ void vector_cpu_irq127(void);
#endif
#if (SOC_IRQ_NSOURCES > 128)
#undef	VECTOR_TABLE_ALIGN
#define	VECTOR_TABLE_ALIGN	1024
HWIRQ void vector_cpu_irq128(void);
HWIRQ void vector_cpu_irq129(void);
HWIRQ void vector_cpu_irq130(void);
HWIRQ void vector_cpu_irq131(void);
HWIRQ void vector_cpu_irq132(void);
HWIRQ void vector_cpu_irq133(void);
HWIRQ void vector_cpu_irq134(void);
HWIRQ void vector_cpu_irq135(void);
#endif
#if (SOC_IRQ_NSOURCES > 136)
HWIRQ void vector_cpu_irq136(void);
HWIRQ void vector_cpu_irq137(void);
HWIRQ void vector_cpu_irq138(void);
HWIRQ void vector_cpu_irq139(void);
HWIRQ void vector_cpu_irq140(void);
HWIRQ void vector_cpu_irq141(void);
HWIRQ void vector_cpu_irq142(void);
HWIRQ void vector_cpu_irq143(void);
#endif
#if (SOC_IRQ_NSOURCES > 144)
#error "Need to extend the vectors Victor!"
#endif
#undef HWIRQ

#define	REAL_VECTORS_ATTR	__attribute__ ((used, \
						aligned(VECTOR_TABLE_ALIGN), \
						section(".realvectors")))
#define	RESET_VECTORS_ATTR	__attribute__ ((section(".resetvectors"),used))
#define	RESET_CRT_ATTR		__attribute__ ((naked, section(".resetcrt")))

void start(void) RESET_CRT_ATTR;

typedef void (*vector_entry_t)(void);

/* Cut down vector table used at reset only. */
const vector_entry_t reset_vectors[16] RESET_VECTORS_ATTR = {
	/* 0 - Initial Stack Pointer Value */
	(vector_entry_t)(uintptr_t)&_stack_top,
	/* 1 - Reset */
	start
};

static const vector_entry_t real_vectors[] REAL_VECTORS_ATTR = {
	/* 0 - Initial Stack Pointer Value */
	(vector_entry_t)(uintptr_t)&_stack_top,

	/* Fatal fault handlers */
	start,				/* 1 - Reset */
	vector_cpu_exc_nmi,		/* 2 - NMI */
	vector_cpu_exc_hard_fault,	/* 3 - Hard Fault */
	vector_cpu_exc_mm_fault,	/* 4 - MM Fault */
	vector_cpu_exc_bus_fault,	/* 5 - Bus Fault */
	vector_cpu_exc_usage_fault,	/* 6 - Usage Fault */
	vector_cpu_exc_resvd7,
	vector_cpu_exc_resvd8,
	vector_cpu_exc_resvd9,
	vector_cpu_exc_resvd10,
	vector_cpu_exc_svccall,		/* 11 - SVCall */
	vector_cpu_exc_debug_mon,	/* 12 - Debug */
	vector_cpu_exc_resvd13,
	vector_cpu_exc_pendsv,		/* 14 - PendSV */
	vector_cpu_exc_systick,		/* 15 - SysTick */

	/* Hardware interrupt handlers: 16 upwards */
	 vector_cpu_irq0, vector_cpu_irq1, vector_cpu_irq2, vector_cpu_irq3,
	 vector_cpu_irq4, vector_cpu_irq5, vector_cpu_irq6, vector_cpu_irq7,
#if (SOC_IRQ_NSOURCES > 8)
	 vector_cpu_irq8, vector_cpu_irq9, vector_cpu_irq10, vector_cpu_irq11,
	 vector_cpu_irq12, vector_cpu_irq13, vector_cpu_irq14, vector_cpu_irq15,
#endif
#if (SOC_IRQ_NSOURCES > 16)
	 vector_cpu_irq16, vector_cpu_irq17, vector_cpu_irq18, vector_cpu_irq19,
	 vector_cpu_irq20, vector_cpu_irq21, vector_cpu_irq22, vector_cpu_irq23,
#endif
#if (SOC_IRQ_NSOURCES > 24)
	 vector_cpu_irq24, vector_cpu_irq25, vector_cpu_irq26, vector_cpu_irq27,
	 vector_cpu_irq28, vector_cpu_irq29, vector_cpu_irq30, vector_cpu_irq31,
#endif
#if (SOC_IRQ_NSOURCES > 32)
	 vector_cpu_irq32, vector_cpu_irq33, vector_cpu_irq34, vector_cpu_irq35,
	 vector_cpu_irq36, vector_cpu_irq37, vector_cpu_irq38, vector_cpu_irq39,
#endif
#if (SOC_IRQ_NSOURCES > 40)
	 vector_cpu_irq40, vector_cpu_irq41, vector_cpu_irq42, vector_cpu_irq43,
	 vector_cpu_irq44, vector_cpu_irq45, vector_cpu_irq46, vector_cpu_irq47,
#endif
#if (SOC_IRQ_NSOURCES > 48)
	 vector_cpu_irq48, vector_cpu_irq49, vector_cpu_irq50, vector_cpu_irq51,
	 vector_cpu_irq52, vector_cpu_irq53, vector_cpu_irq54, vector_cpu_irq55,
#endif
#if (SOC_IRQ_NSOURCES > 56)
	 vector_cpu_irq56, vector_cpu_irq57, vector_cpu_irq58, vector_cpu_irq59,
	 vector_cpu_irq60, vector_cpu_irq61, vector_cpu_irq62, vector_cpu_irq63,
#endif
#if (SOC_IRQ_NSOURCES > 64)
	 vector_cpu_irq64, vector_cpu_irq65, vector_cpu_irq66, vector_cpu_irq67,
	 vector_cpu_irq68, vector_cpu_irq69, vector_cpu_irq70, vector_cpu_irq71,
#endif
#if (SOC_IRQ_NSOURCES > 72)
	 vector_cpu_irq72, vector_cpu_irq73, vector_cpu_irq74, vector_cpu_irq75,
	 vector_cpu_irq76, vector_cpu_irq77, vector_cpu_irq78, vector_cpu_irq79,
#endif
#if (SOC_IRQ_NSOURCES > 80)
	 vector_cpu_irq80, vector_cpu_irq81, vector_cpu_irq82, vector_cpu_irq83,
	 vector_cpu_irq84, vector_cpu_irq85, vector_cpu_irq86, vector_cpu_irq87,
#endif
#if (SOC_IRQ_NSOURCES > 88)
	 vector_cpu_irq88, vector_cpu_irq89, vector_cpu_irq90, vector_cpu_irq91,
	 vector_cpu_irq92, vector_cpu_irq93, vector_cpu_irq94, vector_cpu_irq95,
#endif
#if (SOC_IRQ_NSOURCES > 96)
	 vector_cpu_irq96, vector_cpu_irq97, vector_cpu_irq98, vector_cpu_irq99,
	 vector_cpu_irq100, vector_cpu_irq101, vector_cpu_irq102, vector_cpu_irq103,
#endif
#if (SOC_IRQ_NSOURCES > 104)
	 vector_cpu_irq104, vector_cpu_irq105, vector_cpu_irq106, vector_cpu_irq107,
	 vector_cpu_irq108, vector_cpu_irq109, vector_cpu_irq110, vector_cpu_irq111,
#endif
#if (SOC_IRQ_NSOURCES > 112)
	 vector_cpu_irq112, vector_cpu_irq113, vector_cpu_irq114, vector_cpu_irq115,
	 vector_cpu_irq116, vector_cpu_irq117, vector_cpu_irq118, vector_cpu_irq119,
#endif
#if (SOC_IRQ_NSOURCES > 120)
	 vector_cpu_irq120, vector_cpu_irq121, vector_cpu_irq122, vector_cpu_irq123,
	 vector_cpu_irq124, vector_cpu_irq125, vector_cpu_irq126, vector_cpu_irq127,
#endif
#if (SOC_IRQ_NSOURCES > 128)
	 vector_cpu_irq128, vector_cpu_irq129, vector_cpu_irq130, vector_cpu_irq131,
	 vector_cpu_irq132, vector_cpu_irq133, vector_cpu_irq134, vector_cpu_irq135,
#endif
#if (SOC_IRQ_NSOURCES > 136)
	 vector_cpu_irq136, vector_cpu_irq137, vector_cpu_irq138, vector_cpu_irq139,
	 vector_cpu_irq140, vector_cpu_irq141, vector_cpu_irq142, vector_cpu_irq143,
#endif
};

void platform_early_init(void) __attribute__((weak));
void platform_early_init(void) { }

/*
 * Enter here on hardware reset
 */
void
start(void)
{
#ifndef RTOS_NO_RAMFUNC
	extern unsigned int _siramfunc, _sramfunc, _eramfunc;
#endif
	extern unsigned int _sidata, _data, _edata;
	extern unsigned int _bss;
	extern int main(void);
	extern void __libc_init_array(void);
	volatile unsigned int *src, *dst;

	__disable_irq();
	__set_MSP((uint32_t)(uintptr_t)&_stack_top);
#if (defined(__CORTEX_M) && (__CORTEX_M >= 33U))
	extern unsigned int _stack_bottom;
	__set_MSPLIM((uint32_t)(uintptr_t)&_stack_bottom);
#endif
#if (defined(__CORTEX_M) && (__CORTEX_M > 0U)) || \
    (defined(__VTOR_PRESENT) && (__VTOR_PRESENT > 0U))
	SCB->VTOR = (uint32_t) real_vectors;
#endif

#ifndef RTOS_NO_RAMFUNC
	/* Copy .ramfuncs */
	src = &_siramfunc;
	dst = &_sramfunc;
	while (dst < &_eramfunc)
		*dst++ = *src++;
#endif

	/* Copy initialised data. */
	src = &_sidata;
	dst = &_data;
	while (dst < &_edata)
		*dst++ = *src++;

	/* Zero the BSS */
	dst = &_bss;
	while (dst < (unsigned int *)(uintptr_t)&_stack_top)
		*dst++ = 0;

	__asm __volatile("" ::: "memory");

	platform_early_init();

	__libc_init_array();

	__asm __volatile("" ::: "memory");

	init_irq_support();

	__enable_irq();

	(void) main();

	for (;;);
}

void
abort(void)
{

	__disable_irq();
	console_flush(true);
	for (;;)
		;
}
