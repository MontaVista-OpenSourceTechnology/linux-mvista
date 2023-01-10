/* CCSR base address */

#define PPC_T2XXX_CCSR_ADDR   0xfe000000

/* UART registers */
#define PPC_T2XXX_DUART1_BASE_ADDR     (PPC_T2XXX_CCSR_ADDR + 0x0011c500)

#define NS16550_URBR        0
#define NS16550_UTHR        0
#define NS16550_UDLB        0
#define NS16550_UIER        1
#define NS16550_UDMB        1
#define NS16550_UIIR        2
#define NS16550_UFCR        2
#define NS16550_UAFR        2
#define NS16550_ULCR        3
#define NS16550_UMCR        4
#define NS16550_ULSR        5
#define NS16550_UMSR        6
#define NS16550_USCR        7
#define NS16550_UDSR        10

.macro load32 reg data
    lis    \reg, (\data)@h
    ori    \reg, \reg, (\data)@l
.endm

/* CPU Registers */
#define r0  0
#define r1  1
#define r2  2
#define r3  3
#define r4  4
#define r5  5
#define r6  6
#define r7  7
#define r8  8
#define r9  9
#define r10 10
#define r11 11
#define r12 12
#define r13 13
#define r14 14
#define r15 15
#define r16 16
#define r17 17
#define r18 18
#define r19 19
#define r20 20
#define r21 21
#define r22 22
#define r23 23
#define r24 24
#define r25 25
#define r26 26
#define r27 27
#define r28 28
#define r29 29
#define r30 30
#define r31 31

/*
 * function: puts
 * Purpose:
 *    Print string on console UART1.
 * Registers:
 *    r3  - (I) Address of string to print
 *    r2  - (X) Address of next byte in string
 *    r4  - (X) used by asm_putc
 *    r8  - (X) Return address
 *    r30 - (X) UART base address
 */
.globl mmsi_puts
mmsi_puts:
    /* GID-996433 - PBIT results */
    mflr   r8              /* save return addr */
    load32 r30, PPC_T2XXX_DUART1_BASE_ADDR
    addi   r2,r3,-1
uart_loop:
    lbzu   r3,1(r2)        /* load content of addr(r2) and increment r2 */
    cmpwi  0, r3, 0
    beq    uart_end        /* if r3 == 0 exit loop */
    bl     asm_putc
    cmpwi  0, r3, 10
    bne    uart_loop       /* if r3 == '\n' then put '\r' on the console */
    li     r3, 13
    bl     asm_putc
    b      uart_loop
uart_end:
    mtlr    r8
    blr    /* end of puts */

/*
 * function: putc
 * Purpose:
 *    Print one character on console UART1.
 * Registers:
 *    r3  - (I) Address of string to print
 *    r4  - (X) register status
 *    r30 - (X) UART base address
 */
.globl putc
putc:
    load32 r30, PPC_T2XXX_DUART1_BASE_ADDR

asm_putc:
    /* GID-996433 - PBIT results */
    lbz    r4, NS16550_ULSR(r30)    /* read line status register */
    andi.  r4, r4, 0x20             /* mask */
    beq    asm_putc                 /* if value is 0 goto putc */
    stb    r3, NS16550_UTHR(r30)    /* send character */
    sync
    blr /* end of putc */
