/*
 * e6500 secondary cores release
 *
 * Initalize all secondary cores and keep them in the spin loop.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <asm/ppc_asm.h>

    /* Errata */
#ifdef INCLUDE_ERRATA_A004809
    ERRATA_A004809_APPLY(r3,r4)
#endif

#ifdef INCLUDE_ERRATA_A004792
    ERRATA_A004792_APPLY(r3,r4)
#endif

#ifdef INCLUDE_ERRATA_A004779
    ERRATA_A004779_APPLY(r3,r4)
#endif

#ifdef INCLUDE_ERRATA_A004786
    ERRATA_A004786_APPLY(r3,r4)
#endif

#ifdef INCLUDE_ERRATA_A004806
    ERRATA_A004806_APPLY(r3,r4)
#endif

    /* The address of this 0 is position independent */
    mflr    r4
    bl clear_fpr
zero_64:
    .long 0
    .long 0

clear_fpr:
        /* Reset fpr */
    mflr    r3
    mtlr    r4
    mfmsr   r5                          /* Get msr */
    xor     r4,r4,r4
    ori     r4,r5,0x2000                /* Set msr[fp] */
    mtmsr   r4
    isync
    lfd     0,0(r3)
/*    fcfid   0,0 */
    fmr     1,0
    fmr     2,0
    fmr     3,0
    fmr     4,0
    fmr     5,0
    fmr     6,0
    fmr     7,0
    fmr     8,0
    fmr     9,0
    fmr     10,0
    fmr     11,0
    fmr     12,0
    fmr     13,0
    fmr     14,0
    fmr     15,0
    fmr     16,0
    fmr     17,0
    fmr     18,0
    fmr     19,0
    fmr     20,0
    fmr     21,0
    fmr     22,0
    fmr     23,0
    fmr     24,0
    fmr     25,0
    fmr     26,0
    fmr     27,0
    fmr     28,0
    fmr     29,0
    fmr     30,0
    fmr     31,0
    /* keep msr[fp] on */

clear_altivec:
        /* reset VR */
    mfmsr   r5                  /* Get msr */
    oris    r4,r5,0x0200        /* set msr[spv] */
    mtmsr   r4
    isync

    vxor    0,0,0
    vxor    1,1,1
    vxor    2,2,2
    vxor    3,3,3
    vxor    4,4,4
    vxor    5,5,5
    vxor    6,6,6
    vxor    7,7,7
    vxor    8,8,8
    vxor    9,9,9
    vxor    10,10,10
    vxor    11,11,11
    vxor    12,12,12
    vxor    13,13,13
    vxor    14,14,14
    vxor    15,15,15
    vxor    16,16,16
    vxor    17,17,17
    vxor    18,18,18
    vxor    19,19,19
    vxor    20,20,20
    vxor    21,21,21
    vxor    22,22,22
    vxor    23,23,23
    vxor    24,24,24
    vxor    25,25,25
    vxor    26,26,26
    vxor    27,27,27
    vxor    28,28,28
    vxor    29,29,29
    vxor    30,30,30
    vxor    31,31,31

    mtmsr   r5                        /* restore msr */
    isync

clear_regs:
    xor     r0,  r0,  r0
    xor     r1,  r1,  r1
    xor     r2,  r2,  r2
    xor     r3,  r3,  r3
    xor     r4,  r4,  r4
    xor     r5,  r5,  r5
    xor     r6,  r6,  r6
    xor     r7,  r7,  r7
    xor     r8,  r8,  r9
    xor     r9,  r9,  r9
    xor     r10, r10, r10
    xor     r11, r11, r11
    xor     r12, r12, r12
    xor     r13, r13, r13
    xor     r14, r14, r14
    xor     r15, r15, r15
    xor     r16, r16, r16
    xor     r17, r17, r17
    xor     r18, r18, r18
    xor     r19, r19, r19
    xor     r20, r20, r20
    xor     r21, r21, r21
    xor     r22, r22, r22
    xor     r23, r23, r23
    xor     r24, r24, r24
    xor     r25, r25, r25
    xor     r26, r26, r26
    xor     r27, r27, r27
    xor     r28, r28, r28
    xor     r29, r29, r29
    xor     r30, r30, r30
    xor     r31, r31, r31

clear_sprs:
    msync
    isync
    mtspr   SPRN_HID0, r3        /* here we can't use accessor functions as we must stay within 4k bootpage! */
    mtmsr   r3
    mtspr   SPRN_TCR, r3    /* disable all timers */
    mtspr   SPRN_ESR, r3    /* clear exception sybdrome register */
    mtxer   r3
    mtspr   SPRN_BUCSR, r3  /* clear branch prediction */
    mtspr   SPRN_MAS8, r3   /* clear mmu assist 8 */
    isync   /* writing to MAS8 requires sync */
    msync
    tlbwe
    isync

/*    mtspr   CPU_PPC_E500_L2CSR0, r3 */ /* clear l2csr0 */
    isync

    /* zero timebase */
    li       r3,0
    mtspr    SPRN_TBWL,r3
    mtspr    SPRN_TBWU,r3
    mtspr    SPRN_TBWL,r3

    /* enable branch prediction */
    li      r3, BUCSR_INIT@l
    oris    r3,r3, BUCSR_INIT@h
    mtspr   SPRN_BUCSR, r3
    isync

    sync
    mfspr   r3, SPRN_HID0
    oris    r3, r3, HID0_EMCP@h
    ori     r3, r3, HID0_EMCP@l
    mtspr   SPRN_HID0, r3
    isync
    sync

/* eof */
