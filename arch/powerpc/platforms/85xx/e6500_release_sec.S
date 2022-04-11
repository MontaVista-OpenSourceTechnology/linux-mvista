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
#include "e6500_release_sec.h"
#include "cpu_ppc_e6500_mmu.h"

#define LOAD_REG_32(reg, data)  \
    lis     reg, (data)@h;      \
    ori     reg, reg, (data)@l;

#define LOAD_REG_ADDRESS(reg, data) LOAD_REG_32(reg,data)

#define WR_IVOR(nr, addr)       \
    li      r3, addr@l;         \
    mtspr   SPRN_IVOR##nr, r3;

#define WR_GIVOR(vector_number, vector_offset)    \
    li      r3,vector_offset@l;                   \
    mtspr   SPRN_GIVOR##vector_number,r3;

/* address relative to BOOT_PAGE */
#define GET_ADDR(x) ((x) - __e6500_secondary_start + BOOT_PAGE)

#define SHARED_L2_CLUSTER_OFFS 0xC20000
#define SHARED_L2_CLUSTER_SIZE 0x40000

#define L2CSR0_OFFSET        0x000
#define L2CSR1_OFFSET        0x004
#define L2CFG0_OFFSET        0x008

    .align 12
    .globl __e6500_secondary_start
__e6500_secondary_start:

    #include "init_ppc_e6500.S"

    /* ccsrbar tlb entry */
    LOAD_REG_IMMEDIATE(r3,CPU_PPC_E6500_MMU_MAS0_HELPER(1,2,0,0,0))
    LOAD_REG_IMMEDIATE(r4,CPU_PPC_E6500_MMU_MAS1_HELPER(CPU_PPC_E6500_MMU_MAS1_PROTECT,
        0,0,CPU_PPC_E6500_MMU_MAS1_TSIZE_16M))
    LOAD_REG_IMMEDIATE(r5,CPU_PPC_E6500_MMU_MAS2_HELPER(CCSRBAR,
        CPU_PPC_E6500_MMU_MAS2_I | CPU_PPC_E6500_MMU_MAS2_G))
    LOAD_REG_IMMEDIATE(r6,CPU_PPC_E6500_MMU_MAS3_HELPER(CCSRBAR,CPU_PPC_E6500_MMU_MAS3_SW |
        CPU_PPC_E6500_MMU_MAS3_SR | CPU_PPC_E6500_MMU_MAS3_UW | CPU_PPC_E6500_MMU_MAS3_UR))
    LOAD_REG_IMMEDIATE(r7,0x0f)
    bl      _ppc8500_add_tbe

    /* invalidate D caches */
    xor      r4,r4,r4
    LOAD_REG_IMMEDIATE(r5,L1CSR0_CFI | L1CSR0_CLFC)
    sync
    isync
    mtspr     SPRN_L1CSR0,r5   /* invalidate d-cache */
    isync

_dloop:
    mfspr    r4,SPRN_L1CSR0
    and.     r4,r4,r5
    bne      _dloop

    /* invalidate I caches */
    xor      r4,r4,r4
    LOAD_REG_IMMEDIATE(r5,L1CSR1_ICFI | L1CSR1_ICLFR)
    sync
    isync
    mtspr    SPRN_L1CSR1,r5   /* invalidate i-cache */
    isync
_iloop:
    mfspr    r4,SPRN_L1CSR1
    and.     r4,r4,r5
    bne      _iloop

    /*
        L2 is invalidate and enabled by core 0
        If L2 of this cluster is on L1 can be turn on
     */
    mfspr   r4,SPRN_PIR
    rlwinm  r4,r4,27,30,31   /* r4 = (r3>>5) & 0x3 */
    LOAD_REG_IMMEDIATE(r5,SHARED_L2_CLUSTER_SIZE)
    mullw   r4,r4,r5
    LOAD_REG_ADDRESS(r5,CCSRBAR + SHARED_L2_CLUSTER_OFFS)
    add     r5,r4,r5
    lwz     r5,L2CSR0_OFFSET(r5)
    LOAD_REG_IMMEDIATE(r4,L2CSR0_L2E)
    and.    r4,r4,r5
    beq     skip_cache_enable

    /* enable both */
    LOAD_REG_IMMEDIATE(r5,L1CSR0_DCE | L1CSR0_CPE)
    mfspr   r4,SPRN_L1CSR0
    or      r4,r4,r5
    mtspr   SPRN_L1CSR0,r4

    LOAD_REG_IMMEDIATE(r5,L1CSR1_ICE | L1CSR1_CPE)
    mfspr   r4,SPRN_L1CSR1
    or      r4,r4,r5
    mtspr   SPRN_L1CSR1,r4
skip_cache_enable:

    /* set L1 cache stashing id :
        L1: clustercoreID * 2 + 32
    */
    mfspr   r3,SPRN_PIR
    rlwinm  r4,r3,29,27,31  /* r4=(r3>>3) & 0x1F */
    slwi    r8,r4,2          /* r8=r4<<2 */
    addi    r8,r8,32
    mtspr   SPRN_L1CSR2,r8

    LOAD_REG_IMMEDIATE(r3,CPU_PPC_E6500_MMU_MAS0_HELPER(1,63,0,0,0))
    LOAD_REG_IMMEDIATE(r4,CPU_PPC_E6500_MMU_MAS1_HELPER(CPU_PPC_E6500_MMU_MAS1_PROTECT,0,
        CPU_PPC_E6500_MMU_MAS1_TS1,CPU_PPC_E6500_MMU_MAS1_TSIZE_4K))
    LOAD_REG_IMMEDIATE(r5,CPU_PPC_E6500_MMU_MAS2_HELPER(0,CPU_PPC_E6500_MMU_MAS2_M))
    LOAD_REG_IMMEDIATE(r6,CPU_PPC_E6500_MMU_MAS3_HELPER(0,CPU_PPC_E6500_MMU_MAS3_SX |
        CPU_PPC_E6500_MMU_MAS3_SW | CPU_PPC_E6500_MMU_MAS3_SR | CPU_PPC_E6500_MMU_MAS3_UX |
        CPU_PPC_E6500_MMU_MAS3_UW | CPU_PPC_E6500_MMU_MAS3_UR))

    LOAD_REG_IMMEDIATE(r7,0) /* BOOT_PAGE high = 0 */
    LOAD_REG_IMMEDIATE(r15,BOOT_PAGE) /* BOOT_PAGE low */
    LOAD_REG_IMMEDIATE(r16,0xfff)
    and     r5,r5,r16
    and     r6,r6,r16
    or      r5,r15,r5
    or      r6,r15,r6

    bl      _ppc8500_add_tbe

    /* switch to TS1  */
    mfmsr   r3
    oris    r3,r3,(MSR_IS | MSR_DS)@h
    ori     r3,r3,(MSR_IS | MSR_DS)@l
    mtspr   SPRN_SRR1,r3

    LOAD_REG_IMMEDIATE_SYM(r3,r14,GET_ADDR(jumpPoint))
    and     r3,r16,r3
    or      r3,r15,r3

    mtspr   SPRN_SRR0,r3
    rfi

    .align 6
jumpPoint:

    /* get pir */
    mfspr   r0,SPRN_PIR
    rlwinm  r4,r0,29,27,31   /* r4=(r0>>3) & 0x1F */

    slwi    r8,r4,6      /* r8=r4*32*2 (32=sizeof(spin structure),2=CES_T4240_THREAD_BY_CPU) */
    slwi    r4,r4,1      /* r4=r4*2=2*cpu_nr */

    LOAD_REG_IMMEDIATE_SYM(r3,r14,__e6500_spin_table)   /* we know our code is 4k aligned */
    bl       convertAddress

    /* set spin entries spin_table[cpu_nr * 32 * 2] */
    add      r10,r3,r8

    li       r3,0
    li       r8,1
    stw      r3,BOOT_ENTRY_ADDR_UPPER(r10)
    stw      r8,BOOT_ENTRY_ADDR_LOWER(r10)
    stw      r3,BOOT_ENTRY_R3_UPPER(r10)
    stw      r4,BOOT_ENTRY_R3_LOWER(r10)
    stw      r3,BOOT_ENTRY_RESV(r10)
    stw      r4,BOOT_ENTRY_PIR(r10)
    stw      r3,BOOT_ENTRY_R6_UPPER(r10)
    stw      r3,BOOT_ENTRY_R6_LOWER(r10)

    /* spin until new address */
_spin_loop:
    lwz      r4,BOOT_ENTRY_ADDR_LOWER(r10)
    andi.    r11,r4,1
    bne      _spin_loop

    /* set IVPR */
    lwz     r3,BOOT_ENTRY_ADDR_LOWER(r10)
    rlwinm  r3,r3,0,0,5
    mtspr   SPRN_IVPR,r3

    /* set IVOR */

    WR_IVOR( 0,0x020)
    WR_IVOR( 1,0x000)
    WR_IVOR( 2,0x060)
    WR_IVOR( 3,0x080)
    WR_IVOR( 4,0x0a0)
    WR_IVOR( 5,0x0c0)
    WR_IVOR( 6,0x0e0)
    WR_IVOR( 7,0x100)
    WR_IVOR( 8,0x120)
    WR_IVOR( 9,0x140)
    WR_IVOR(10,0x160)
    WR_IVOR(11,0x180)
    WR_IVOR(12,0x1A0)
    WR_IVOR(13,0x1C0)
    WR_IVOR(14,0x1E0)
    WR_IVOR(15,0x040)

    WR_IVOR(32,0x200)
    WR_IVOR(33,0x220)
    /* WR_IVOR(34,0x240)  make it crash!! */

    WR_IVOR(35,0x260)

    WR_IVOR(36,0x280)
    WR_IVOR(37,0x2a0)
    WR_IVOR(38,0x2c0)
    WR_IVOR(39,0x2e0)
    WR_IVOR(40,0x300)
    WR_IVOR(41,0x320)

    WR_GIVOR(2,0x060)
    WR_GIVOR(3,0x080)
    WR_GIVOR(4,0x0a0)
    WR_GIVOR(8,0x120)
    WR_GIVOR(13,0x1c0)
    WR_GIVOR(14,0x1e0)

    /* create mapping */

    lwz     r3,BOOT_ENTRY_ADDR_UPPER(r10)
    andi.   r7,r3,0x00ff                          /* r7 = r3 & 0x000000ff */

    lwz     r3,BOOT_ENTRY_ADDR_LOWER(r10)
    andis.  r3,r3,0xfc00                          /* r6 = r3 & 0xfc000000 (64MB mask) */
    mtspr   SPRN_IVPR,r3
    mr      r6,r3
    oris    r6,r6,CPU_PPC_E6500_MMU_MAS3_HELPER(0,CPU_PPC_E6500_MMU_MAS3_SX |
       CPU_PPC_E6500_MMU_MAS3_SW | CPU_PPC_E6500_MMU_MAS3_SR | CPU_PPC_E6500_MMU_MAS3_UX |
       CPU_PPC_E6500_MMU_MAS3_UR | CPU_PPC_E6500_MMU_MAS3_UW)@h
    ori     r6,r6,CPU_PPC_E6500_MMU_MAS3_HELPER(0,CPU_PPC_E6500_MMU_MAS3_SX |
       CPU_PPC_E6500_MMU_MAS3_SW | CPU_PPC_E6500_MMU_MAS3_SR | CPU_PPC_E6500_MMU_MAS3_UX |
       CPU_PPC_E6500_MMU_MAS3_UR | CPU_PPC_E6500_MMU_MAS3_UW)@l
    mr      r5,r3
    oris    r5,r5,CPU_PPC_E6500_MMU_MAS2_HELPER(0,CPU_PPC_E6500_MMU_MAS2_M)@h
    ori     r5,r5,CPU_PPC_E6500_MMU_MAS2_HELPER(0,CPU_PPC_E6500_MMU_MAS2_M)@l
    LOAD_REG_IMMEDIATE(r4,CPU_PPC_E6500_MMU_MAS1_HELPER(CPU_PPC_E6500_MMU_MAS1_PROTECT,0,0,
       CPU_PPC_E6500_MMU_MAS1_TSIZE_64M))
    LOAD_REG_IMMEDIATE(r3,CPU_PPC_E6500_MMU_MAS0_HELPER(1,0,0,0,0))
    bl  _ppc8500_add_tbe

    /* now r4 holds lower address */
    lwz     r3,BOOT_ENTRY_ADDR_LOWER(r10)
    sync
    isync
    mtspr   SPRN_SRR0,r3
    sync

    li      r3,0x2200
    mtspr   SPRN_SRR1,r3

    li      r3,3
    stw     r3,BOOT_ENTRY_ADDR_LOWER(r10)

    /* load up the pir */
    lwz     r0,BOOT_ENTRY_PIR(r10)
    mtspr   SPRN_PIR,r0
    mfspr   r0,SPRN_PIR
    stw     r0,BOOT_ENTRY_PIR(r10)

    /*
     * setup r3, r4, r5, r6, r7, r8, r9
     * r3 contains the value to put in the r3 register at secondary cpu
     * entry. The high 32-bits are ignored on 32-bit chip implementations.
     * 64-bit chip implementations however shall load all 64-bits
     */
    lwz     r3,BOOT_ENTRY_R3_LOWER(r10)
    li      r4,0
    li      r5,0
    lwz     r6,BOOT_ENTRY_R6_LOWER(r10)
    lis     r7,(64*1024*1024)@h
    li      r8,0
    li      r9,0

    rfi

    .align 6
_ppc8500_add_tbe:
    /* This fix a bug on freescale T4240 */
    rlwinm   r3,r3,0,0,31
    rlwinm   r4,r4,0,0,31
    rlwinm   r5,r5,0,0,31
    rlwinm   r6,r6,0,0,31
    rlwinm   r7,r7,0,0,31

    isync
    mtspr    SPRN_MAS0,r3
    mtspr    SPRN_MAS1,r4
    mtspr    SPRN_MAS2,r5
    mtspr    SPRN_MAS3,r6
    mtspr    SPRN_MAS7,r7
    isync
    msync
    tlbwe
    isync

    blr

/* We know that we have to run within 4k
 * r3 contains offset within the 4k
 */
convertAddress:
    mflr    r31
    LOAD_REG_IMMEDIATE(r30,BOOT_PAGE)
    and     r31,r31,r30           /* mask link register,r30 should contain something like 0x7ffff000 now */
    andi.   r3,r3,0x0fff          /* just to be sure,mask only the offset within the 4k page */
    or      r3,r31,r3             /* r30 | r3 should give something like 0x7ffff124 */
    blr

    . = 0xc00
    .globl __e6500_spin_table
__e6500_spin_table:
    .space E6500_CPU_NUM * BOOT_ENTRY_SIZE

    . = 0xffc
__secondary_reset_vector:
    b        __e6500_secondary_start
