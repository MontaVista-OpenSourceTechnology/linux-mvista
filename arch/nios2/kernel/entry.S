/*
 * linux/arch/nios2/kernel/entry.S
 *
 * Copyright (C) 2013-2014  Altera Corporation
 * Copyright (C) 2009, Wind River Systems Inc
 *
 * Implemented by fredrik.markstrom@gmail.com and ivarholmqvist@gmail.com
 *
 *  Copyright (C) 1999-2002, Greg Ungerer (gerg@snapgear.com)
 *  Copyright (C) 1998  D. Jeff Dionne <jeff@lineo.ca>,
 *                      Kenneth Albanowski <kjahds@kjahds.com>,
 *  Copyright (C) 2000  Lineo Inc. (www.lineo.com)
 *  Copyright (C) 2004  Microtronix Datacom Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Linux/m68k support by Hamish Macdonald
 *
 * 68060 fixes by Jesper Skov
 * ColdFire support by Greg Ungerer (gerg@snapgear.com)
 * 5307 fixes by David W. Miller
 * linux 2.4 support David McCullough <davidm@snapgear.com>
 */

#include <linux/sys.h>
#include <linux/linkage.h>
#include <asm/asm-offsets.h>
#include <asm/asm-macros.h>
#include <asm/thread_info.h>
#include <asm/errno.h>
#include <asm/setup.h>
#include <asm/entry.h>
#include <asm/unistd.h>
#include <asm/processor.h>

.macro GET_THREAD_INFO reg
.if THREAD_SIZE & 0xffff0000
	andhi	\reg, sp, %hi(~(THREAD_SIZE-1))
.else
	addi	\reg, r0, %lo(~(THREAD_SIZE-1))
	and	\reg, \reg, sp
.endif
.endm

.macro	kuser_cmpxchg_check
	/*
	 * Make sure our user space atomic helper is restarted if it was
	 * interrupted in a critical region.
	 * ea-4 = address of interrupted insn (ea must be preserved).
	 * sp = saved regs.
	 * cmpxchg_ldw = first critical insn, cmpxchg_stw = last critical insn.
	 * If ea <= cmpxchg_stw and ea > cmpxchg_ldw then saved EA is set to
	 * cmpxchg_ldw + 4.
	*/
	/* et = cmpxchg_stw + 4 */
	movui   et, (KUSER_BASE + 4 + (cmpxchg_stw - __kuser_helper_start))
	bgtu	ea, et, 1f

	subi	et, et, (cmpxchg_stw - cmpxchg_ldw) /* et = cmpxchg_ldw + 4 */
	bltu	ea, et, 1f
	stw	et, PT_EA(sp)	/* fix up EA */
	mov	ea, et
1:
.endm

.section .rodata
.align 4
exception_table:
	.word unhandled_exception	/* 0 - Reset */
	.word unhandled_exception	/* 1 - Processor-only Reset */
	.word external_interrupt	/* 2 - Interrupt */
	.word handle_trap		/* 3 - Trap Instruction */

	.word instruction_trap		/* 4 - Unimplemented instruction */
	.word handle_illegal		/* 5 - Illegal instruction */
	.word handle_unaligned		/* 6 - Misaligned data access */
	.word handle_unaligned		/* 7 - Misaligned destination address */

	.word handle_diverror		/* 8 - Division error */
	.word protection_exception_ba	/* 9 - Supervisor-only instr. address */
	.word protection_exception_instr /* 10 - Supervisor only instruction */
	.word protection_exception_ba	/* 11 - Supervisor only data address */

	.word unhandled_exception	/* 12 - Double TLB miss (data) */
	.word protection_exception_pte	/* 13 - TLB permission violation (x) */
	.word protection_exception_pte	/* 14 - TLB permission violation (r) */
	.word protection_exception_pte	/* 15 - TLB permission violation (w) */

	.word unhandled_exception	/* 16 - MPU region violation */

trap_table:
	.word	handle_system_call	/* 0  */
	.word	handle_trap_1		/* 1  */
	.word	handle_trap_2		/* 2  */
	.word	handle_trap_3		/* 3  */
	.word	handle_trap_reserved	/* 4  */
	.word	handle_trap_reserved	/* 5  */
	.word	handle_trap_reserved	/* 6  */
	.word	handle_trap_reserved	/* 7  */
	.word	handle_trap_reserved	/* 8  */
	.word	handle_trap_reserved	/* 9  */
	.word	handle_trap_reserved	/* 10 */
	.word	handle_trap_reserved	/* 11 */
	.word	handle_trap_reserved	/* 12 */
	.word	handle_trap_reserved	/* 13 */
	.word	handle_trap_reserved	/* 14 */
	.word	handle_trap_reserved	/* 15 */
	.word	handle_trap_reserved	/* 16 */
	.word	handle_trap_reserved	/* 17 */
	.word	handle_trap_reserved	/* 18 */
	.word	handle_trap_reserved	/* 19 */
	.word	handle_trap_reserved	/* 20 */
	.word	handle_trap_reserved	/* 21 */
	.word	handle_trap_reserved	/* 22 */
	.word	handle_trap_reserved	/* 23 */
	.word	handle_trap_reserved	/* 24 */
	.word	handle_trap_reserved	/* 25 */
	.word	handle_trap_reserved	/* 26 */
	.word	handle_trap_reserved	/* 27 */
	.word	handle_trap_reserved	/* 28 */
	.word	handle_trap_reserved	/* 29 */
#ifdef CONFIG_KGDB
	.word	handle_kgdb_breakpoint	/* 30 KGDB breakpoint */
#else
	.word	instruction_trap		/* 30 */
#endif
	.word	handle_breakpoint	/* 31 */

.text
.set noat
.set nobreak

ENTRY(inthandler)
	SAVE_ALL

	kuser_cmpxchg_check

	/* Clear EH bit before we get a new excpetion in the kernel
	 * and after we have saved it to the exception frame. This is done
	 * whether it's trap, tlb-miss or interrupt. If we don't do this
	 * estatus is not updated the next exception.
	 */
	rdctl	r24, status
	movi	r9, %lo(~STATUS_EH)
	and	r24, r24, r9
	wrctl	status, r24

	/* Read cause and vector and branch to the associated handler */
	mov	r4, sp
	rdctl	r5, exception
	movia	r9, exception_table
	add	r24, r9, r5
	ldw	r24, 0(r24)
	jmp	r24


/***********************************************************************
 * Handle traps
 ***********************************************************************
 */
ENTRY(handle_trap)
	ldwio	r24, -4(ea)	/* instruction that caused the exception */
	srli	r24, r24, 4
	andi	r24, r24, 0x7c
	movia	r9,trap_table
	add	r24, r24, r9
	ldw	r24, 0(r24)
	jmp	r24


/***********************************************************************
 * Handle system calls
 ***********************************************************************
 */
ENTRY(handle_system_call)
	/* Enable interrupts */
	rdctl	r10, status
	ori	r10, r10, STATUS_PIE
	wrctl	status, r10

	/* Reload registers destroyed by common code. */
	ldw	r4, PT_R4(sp)
	ldw	r5, PT_R5(sp)

local_restart:
	stw	r2, PT_ORIG_R2(sp)
	/* Check that the requested system call is within limits */
	movui	r1, __NR_syscalls
	bgeu	r2, r1, ret_invsyscall
	slli	r1, r2, 2
	movhi	r11, %hiadj(sys_call_table)
	add	r1, r1, r11
	ldw	r1, %lo(sys_call_table)(r1)

	/* Check if we are being traced */
	GET_THREAD_INFO r11
	ldw	r11,TI_FLAGS(r11)
	BTBNZ   r11,r11,TIF_SYSCALL_TRACE,traced_system_call

	/* Execute the system call */
	callr	r1

	/* If the syscall returns a negative result:
	 *   Set r7 to 1 to indicate error,
	 *   Negate r2 to get a positive error code
	 * If the syscall returns zero or a positive value:
	 *   Set r7 to 0.
	 * The sigreturn system calls will skip the code below by
	 * adding to register ra. To avoid destroying registers
	 */
translate_rc_and_ret:
	movi	r1, 0
	bge	r2, zero, 3f
	ldw	r1, PT_ORIG_R2(sp)
	addi	r1, r1, 1
	beq	r1, zero, 3f
	sub	r2, zero, r2
	movi	r1, 1
3:
	stw	r2, PT_R2(sp)
	stw	r1, PT_R7(sp)
end_translate_rc_and_ret:

ret_from_exception:
	ldw	r1, PT_ESTATUS(sp)
	/* if so, skip resched, signals */
	TSTBNZ	r1, r1, ESTATUS_EU, Luser_return

restore_all:
	rdctl	r10, status			/* disable intrs */
	andi	r10, r10, %lo(~STATUS_PIE)
	wrctl	status, r10
	RESTORE_ALL
	eret

	/* If the syscall number was invalid return ENOSYS */
ret_invsyscall:
	movi	r2, -ENOSYS
	br	translate_rc_and_ret

	/* This implements the same as above, except it calls
	 * do_syscall_trace_enter and do_syscall_trace_exit before and after the
	 * syscall in order for utilities like strace and gdb to work.
	 */
traced_system_call:
	SAVE_SWITCH_STACK
	call	do_syscall_trace_enter
	RESTORE_SWITCH_STACK

	/* Create system call register arguments. The 5th and 6th
	   arguments on stack are already in place at the beginning
	   of pt_regs. */
	ldw	r2, PT_R2(sp)
	ldw	r4, PT_R4(sp)
	ldw	r5, PT_R5(sp)
	ldw	r6, PT_R6(sp)
	ldw	r7, PT_R7(sp)

	/* Fetch the syscall function. */
	movui	r1, __NR_syscalls
	bgeu	r2, r1, traced_invsyscall
	slli	r1, r2, 2
	movhi	r11,%hiadj(sys_call_table)
	add	r1, r1, r11
	ldw	r1, %lo(sys_call_table)(r1)

	callr	r1

	/* If the syscall returns a negative result:
	 *   Set r7 to 1 to indicate error,
	 *   Negate r2 to get a positive error code
	 * If the syscall returns zero or a positive value:
	 *   Set r7 to 0.
	 * The sigreturn system calls will skip the code below by
	 * adding to register ra. To avoid destroying registers
	 */
translate_rc_and_ret2:
	movi	r1, 0
	bge	r2, zero, 4f
	ldw	r1, PT_ORIG_R2(sp)
	addi	r1, r1, 1
	beq	r1, zero, 4f
	sub	r2, zero, r2
	movi	r1, 1
4:
	stw	r2, PT_R2(sp)
	stw	r1, PT_R7(sp)
end_translate_rc_and_ret2:
	SAVE_SWITCH_STACK
	call	do_syscall_trace_exit
	RESTORE_SWITCH_STACK
	br	ret_from_exception

	/* If the syscall number was invalid return ENOSYS */
traced_invsyscall:
	movi	r2, -ENOSYS
	br	translate_rc_and_ret2

Luser_return:
	GET_THREAD_INFO	r11			/* get thread_info pointer */
	ldw	r10, TI_FLAGS(r11)		/* get thread_info->flags */
	ANDI32	r11, r10, _TIF_WORK_MASK
	beq	r11, r0, restore_all		/* Nothing to do */
	BTBZ	r1, r10, TIF_NEED_RESCHED, Lsignal_return

	/* Reschedule work */
	call	schedule
	br	ret_from_exception

Lsignal_return:
	ANDI32	r1, r10, _TIF_SIGPENDING | _TIF_NOTIFY_RESUME
	beq	r1, r0, restore_all
	mov	r4, sp			/* pt_regs */
	SAVE_SWITCH_STACK
	call	do_notify_resume
	beq	r2, r0, no_work_pending
	RESTORE_SWITCH_STACK
	/* prepare restart syscall here without leaving kernel */
	ldw	r2, PT_R2(sp)	/* reload syscall number in r2 */
	ldw 	r4, PT_R4(sp)	/* reload syscall arguments r4-r9 */
	ldw 	r5, PT_R5(sp)
	ldw 	r6, PT_R6(sp)
	ldw 	r7, PT_R7(sp)
	ldw 	r8, PT_R8(sp)
	ldw 	r9, PT_R9(sp)
	br	local_restart	/* restart syscall */

no_work_pending:
	RESTORE_SWITCH_STACK
	br	ret_from_exception

/***********************************************************************
 * Handle external interrupts.
 ***********************************************************************
 */
/*
 * This is the generic interrupt handler (for all hardware interrupt
 * sources). It figures out the vector number and calls the appropriate
 * interrupt service routine directly.
 */
external_interrupt:
	rdctl	r12, ipending
	rdctl	r9, ienable
	and	r12, r12, r9
	/* skip if no interrupt is pending */
	beq	r12, r0, ret_from_interrupt

	/*
	 * Process an external hardware interrupt.
	 */

	addi	ea, ea, -4	/* re-issue the interrupted instruction */
	stw	ea, PT_EA(sp)
2:	movi	r4, %lo(-1)	/* Start from bit position 0,
					highest priority */
				/* This is the IRQ # for handler call */
1:	andi	r10, r12, 1	/* Isolate bit we are interested in */
	srli	r12, r12, 1	/* shift count is costly without hardware
					multiplier */
	addi	r4, r4, 1
	beq	r10, r0, 1b
	mov	r5, sp		/* Setup pt_regs pointer for handler call */
	call	do_IRQ
	rdctl	r12, ipending	/* check again if irq still pending */
	rdctl	r9, ienable	/* Isolate possible interrupts */
	and	r12, r12, r9
	bne	r12, r0, 2b
	/* br	ret_from_interrupt */ /* fall through to ret_from_interrupt */

ENTRY(ret_from_interrupt)
	ldw	r1, PT_ESTATUS(sp)	/* check if returning to kernel */
	TSTBNZ	r1, r1, ESTATUS_EU, Luser_return

#ifdef CONFIG_PREEMPTION
	GET_THREAD_INFO	r1
	ldw	r4, TI_PREEMPT_COUNT(r1)
	bne	r4, r0, restore_all
	ldw	r4, TI_FLAGS(r1)		/* ? Need resched set */
	BTBZ	r10, r4, TIF_NEED_RESCHED, restore_all
	ldw	r4, PT_ESTATUS(sp)	/* ? Interrupts off */
	andi	r10, r4, ESTATUS_EPIE
	beq	r10, r0, restore_all
	call	preempt_schedule_irq
#endif
	br	restore_all

/***********************************************************************
 * A few syscall wrappers
 ***********************************************************************
 */
/*
 * int clone(unsigned long clone_flags, unsigned long newsp,
 *		int __user * parent_tidptr, int __user * child_tidptr,
 *		int tls_val)
 */
ENTRY(sys_clone)
	SAVE_SWITCH_STACK
	addi	sp, sp, -4
	stw	r7, 0(sp)	/* Pass 5th arg thru stack */
	mov	r7, r6		/* 4th arg is 3rd of clone() */
	mov	r6, zero	/* 3rd arg always 0 */
	call	do_fork
	addi	sp, sp, 4
	RESTORE_SWITCH_STACK
	ret

ENTRY(sys_rt_sigreturn)
	SAVE_SWITCH_STACK
	mov	r4, sp
	call	do_rt_sigreturn
	RESTORE_SWITCH_STACK
	addi	ra, ra, (end_translate_rc_and_ret - translate_rc_and_ret)
	ret

/***********************************************************************
 * A few other wrappers and stubs
 ***********************************************************************
 */
protection_exception_pte:
	rdctl	r6, pteaddr
	slli	r6, r6, 10
	call	do_page_fault
	br	ret_from_exception

protection_exception_ba:
	rdctl	r6, badaddr
	call	do_page_fault
	br	ret_from_exception

protection_exception_instr:
	call	handle_supervisor_instr
	br	ret_from_exception

handle_breakpoint:
	call	breakpoint_c
	br	ret_from_exception

#ifdef CONFIG_NIOS2_ALIGNMENT_TRAP
handle_unaligned:
	SAVE_SWITCH_STACK
	call	handle_unaligned_c
	RESTORE_SWITCH_STACK
	br	ret_from_exception
#else
handle_unaligned:
	call	handle_unaligned_c
	br	ret_from_exception
#endif

handle_illegal:
	call	handle_illegal_c
	br	ret_from_exception

handle_diverror:
	call	handle_diverror_c
	br	ret_from_exception

#ifdef CONFIG_KGDB
handle_kgdb_breakpoint:
	call	kgdb_breakpoint_c
	br	ret_from_exception
#endif

handle_trap_1:
	call	handle_trap_1_c
	br	ret_from_exception

handle_trap_2:
	call	handle_trap_2_c
	br	ret_from_exception

handle_trap_3:
handle_trap_reserved:
	call	handle_trap_3_c
	br	ret_from_exception

/*
 * Beware - when entering resume, prev (the current task) is
 * in r4, next (the new task) is in r5, don't change these
 * registers.
 */
ENTRY(resume)

	rdctl	r7, status			/* save thread status reg */
	stw	r7, TASK_THREAD + THREAD_KPSR(r4)

	andi	r7, r7, %lo(~STATUS_PIE)	/* disable interrupts */
	wrctl	status, r7

	SAVE_SWITCH_STACK
	stw	sp, TASK_THREAD + THREAD_KSP(r4)/* save kernel stack pointer */
	ldw	sp, TASK_THREAD + THREAD_KSP(r5)/* restore new thread stack */
	movia	r24, _current_thread		/* save thread */
	GET_THREAD_INFO r1
	stw	r1, 0(r24)
	RESTORE_SWITCH_STACK

	ldw	r7, TASK_THREAD + THREAD_KPSR(r5)/* restore thread status reg */
	wrctl	status, r7
	ret

ENTRY(ret_from_fork)
	call	schedule_tail
	br	ret_from_exception

ENTRY(ret_from_kernel_thread)
	call	schedule_tail
	mov	r4,r17	/* arg */
	callr	r16	/* function */
	br	ret_from_exception

/*
 * Kernel user helpers.
 *
 * Each segment is 64-byte aligned and will be mapped to the <User space>.
 * New segments (if ever needed) must be added after the existing ones.
 * This mechanism should be used only for things that are really small and
 * justified, and not be abused freely.
 *
 */

 /* Filling pads with undefined instructions. */
.macro	kuser_pad sym size
	.if	((. - \sym) & 3)
	.rept	(4 - (. - \sym) & 3)
	.byte	0
	.endr
	.endif
	.rept	((\size - (. - \sym)) / 4)
	.word	0xdeadbeef
	.endr
.endm

	.align	6
	.globl	__kuser_helper_start
__kuser_helper_start:

__kuser_helper_version:				/* @ 0x1000 */
	.word	((__kuser_helper_end - __kuser_helper_start) >> 6)

__kuser_cmpxchg:				/* @ 0x1004 */
	/*
	 * r4 pointer to exchange variable
	 * r5 old value
	 * r6 new value
	 */
cmpxchg_ldw:
	ldw	r2, 0(r4)			/* load current value */
	sub	r2, r2, r5			/* compare with old value */
	bne	r2, zero, cmpxchg_ret

	/* We had a match, store the new value */
cmpxchg_stw:
	stw	r6, 0(r4)
cmpxchg_ret:
	ret

	kuser_pad __kuser_cmpxchg, 64

	.globl	__kuser_sigtramp
__kuser_sigtramp:
	movi	r2, __NR_rt_sigreturn
	trap

	kuser_pad __kuser_sigtramp, 64

	.globl	__kuser_helper_end
__kuser_helper_end:
