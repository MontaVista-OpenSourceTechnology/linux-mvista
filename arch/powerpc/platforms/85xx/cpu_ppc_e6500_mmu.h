/*
 * e6500 MMU registers
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef CPU_PPC_E6500_MMU_H_
#define CPU_PPC_E6500_MMU_H_

#define CPU_PPC_E6500_MMU_TLBnCFG_ASSOC_MASK     (0xff000000)
#define CPU_PPC_E6500_MMU_TLBnCFG_ASSOC_SHFT     (24)
#define CPU_PPC_E6500_MMU_TLBnCFG_PT_MASK        (0x00040000)
#define CPU_PPC_E6500_MMU_TLBnCFG_PT_SHFT        (18)
#define CPU_PPC_E6500_MMU_TLBnCFG_IND_MASK       (0x00020000)
#define CPU_PPC_E6500_MMU_TLBnCFG_IND_SHFT       (17)
#define CPU_PPC_E6500_MMU_TLBnCFG_GTWE_MASK      (0x00010000)
#define CPU_PPC_E6500_MMU_TLBnCFG_GTWE_SHFT      (16)
#define CPU_PPC_E6500_MMU_TLBnCFG_IPROT_MASK     (0x00008000)
#define CPU_PPC_E6500_MMU_TLBnCFG_IPROT_SHFT     (15)
#define CPU_PPC_E6500_MMU_TLBnCFG_HES_MASK       (0x00002000)
#define CPU_PPC_E6500_MMU_TLBnCFG_HES_SHFT       (13)
#define CPU_PPC_E6500_MMU_TLBnCFG_NENTRY_MASK    (0x00000fff)
#define CPU_PPC_E6500_MMU_TLBnCFG_NENTRY_SHFT    (0)

#define CPU_PPC_E6500_MMU_MAS0_ATSEL_MSK         (0x80000000)
#define CPU_PPC_E6500_MMU_MAS0_ATSEL_SHFT        (31)
#define CPU_PPC_E6500_MMU_MAS0_TLB1SEL_MSK       (0x30000000)
#define CPU_PPC_E6500_MMU_MAS0_TLB1SEL_SHFT      (28)
#define CPU_PPC_E6500_MMU_MAS0_ESEL_MSK          (0x003f0000)
#define CPU_PPC_E6500_MMU_MAS0_ESEL_SHFT         (16)
#define CPU_PPC_E6500_MMU_MAS0_HES_MSK           (0x00004000)
#define CPU_PPC_E6500_MMU_MAS0_HES_SHFT          (14)
#define CPU_PPC_E6500_MMU_MAS0_NV_MSK            (0x00000003)
#define CPU_PPC_E6500_MMU_MAS0_NV_SHFT           (0)

/* just to make it phony */
#define CPU_PPC_E6500_MMU_MAS0_TLB0      (0)
#define CPU_PPC_E6500_MMU_MAS0_TLB1      (1)

#define CPU_PPC_E6500_MMU_MAS0_HELPER(tbsel, esel, nv, atsel, hes)	\
    ((((atsel)   << CPU_PPC_E6500_MMU_MAS0_ATSEL_SHFT)    & CPU_PPC_E6500_MMU_MAS0_ATSEL_MSK)    | \
     (((tbsel)   << CPU_PPC_E6500_MMU_MAS0_TLB1SEL_SHFT)  & CPU_PPC_E6500_MMU_MAS0_TLB1SEL_MSK)  | \
     (((esel)    << CPU_PPC_E6500_MMU_MAS0_ESEL_SHFT)     & CPU_PPC_E6500_MMU_MAS0_ESEL_MSK)     | \
     (((hes)     << CPU_PPC_E6500_MMU_MAS0_HES_SHFT)      & CPU_PPC_E6500_MMU_MAS0_HES_MSK)      | \
     (((nv)      << CPU_PPC_E6500_MMU_MAS0_NV_SHFT)       & CPU_PPC_E6500_MMU_MAS0_NV_MSK))


#define CPU_PPC_E6500_MMU_MAS1_VALID_SHFT        (31)
#define CPU_PPC_E6500_MMU_MAS1_PROTECT_SHFT      (30)
#define CPU_PPC_E6500_MMU_MAS1_TID_MSK           (0x1fff0000)
#define CPU_PPC_E6500_MMU_MAS1_TID_SHFT          (16)
#define CPU_PPC_E6500_MMU_MAS1_IND_SHFT          (13)
#define CPU_PPC_E6500_MMU_MAS1_TS_SHFT           (12)

#define CPU_PPC_E6500_MMU_MAS1_INVALID           (0)
#define CPU_PPC_E6500_MMU_MAS1_VALID             (1)
#define CPU_PPC_E6500_MMU_MAS1_NO_PROTECT        (0)
#define CPU_PPC_E6500_MMU_MAS1_PROTECT           (1)
#define CPU_PPC_E6500_MMU_MAS1_DIRECT		 (0)
#define CPU_PPC_E6500_MMU_MAS1_INDIRECT		 (1)
#define CPU_PPC_E6500_MMU_MAS1_TS0               (0)
#define CPU_PPC_E6500_MMU_MAS1_TS1               (1)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_MSK         (0x00000f80)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_SHFT        (7)

#define CPU_PPC_E6500_MMU_MAS1_TSIZE_4K          (0x02)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_8K          (0x03)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_16K         (0x04)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_32K         (0x05)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_64K         (0x06)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_128K        (0x07)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_256K        (0x08)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_512K        (0x09)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_1M          (0x0A)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_2M          (0x0B)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_4M          (0x0C)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_8M          (0x0D)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_16M         (0x0E)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_32M         (0x0F)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_64M         (0x10)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_128M        (0x11)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_256M        (0x12)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_512M        (0x13)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_1G          (0x14)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_2G          (0x15)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_4G          (0x16)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_8G          (0x17)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_16G         (0x18)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_32G         (0x19)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_64G         (0x1A)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_128G        (0x1B)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_256G        (0x1C)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_512G        (0x1D)
#define CPU_PPC_E6500_MMU_MAS1_TSIZE_1T          (0x1E)

#define CPU_PPC_E6500_MMU_MAS1_HELPER(prot, tid, ts, size)		\
    ((CPU_PPC_E6500_MMU_MAS1_VALID << CPU_PPC_E6500_MMU_MAS1_VALID_SHFT)                 | \
     (((prot) & 1) << CPU_PPC_E6500_MMU_MAS1_PROTECT_SHFT)                               | \
     (((tid) << CPU_PPC_E6500_MMU_MAS1_TID_SHFT) & CPU_PPC_E6500_MMU_MAS1_TID_MSK)       | \
     (((ts) & 1) << CPU_PPC_E6500_MMU_MAS1_TS_SHFT)                                      | \
     (((size) << CPU_PPC_E6500_MMU_MAS1_TSIZE_SHFT) & CPU_PPC_E6500_MMU_MAS1_TSIZE_MSK))

#define CPU_PPC_E6500_MMU_MAS1_HELPER1(valid, prot, tid, ts, size, ind)	\
   ((((valid) & 1) << CPU_PPC_E6500_MMU_MAS1_VALID_SHFT)                                 | \
     (((prot) & 1) << CPU_PPC_E6500_MMU_MAS1_PROTECT_SHFT)                               | \
     (((tid) << CPU_PPC_E6500_MMU_MAS1_TID_SHFT) & CPU_PPC_E6500_MMU_MAS1_TID_MSK)       | \
     (((ind)& 1) << CPU_PPC_E6500_MMU_MAS1_IND_SHFT)                                     | \
     (((ts) & 1) << CPU_PPC_E6500_MMU_MAS1_TS_SHFT)                                      | \
     (((size) << CPU_PPC_E6500_MMU_MAS1_TSIZE_SHFT) & CPU_PPC_E6500_MMU_MAS1_TSIZE_MSK))


#define CPU_PPC_E6500_MMU_MAS2_EPN_MSK           (0xfffff000)
#define CPU_PPC_E6500_MMU_MAS2_FLAGS_MASK        (0x0000007f)
#define CPU_PPC_E6500_MMU_MAS2_X0_SHFT           (6)
#define CPU_PPC_E6500_MMU_MAS2_X1_SHFT           (5)
#define CPU_PPC_E6500_MMU_MAS2_W_SHFT            (4)
#define CPU_PPC_E6500_MMU_MAS2_I_SHFT            (3)
#define CPU_PPC_E6500_MMU_MAS2_M_SHFT            (2)
#define CPU_PPC_E6500_MMU_MAS2_G_SHFT            (1)
#define CPU_PPC_E6500_MMU_MAS2_E_SHFT            (0)

#define CPU_PPC_E6500_MMU_MAS2_X0                ((1) << CPU_PPC_E6500_MMU_MAS2_X0_SHFT)
#define CPU_PPC_E6500_MMU_MAS2_X1                ((1) << CPU_PPC_E6500_MMU_MAS2_X1_SHFT)
#define CPU_PPC_E6500_MMU_MAS2_W                 ((1) << CPU_PPC_E6500_MMU_MAS2_W_SHFT)
#define CPU_PPC_E6500_MMU_MAS2_I                 ((1) << CPU_PPC_E6500_MMU_MAS2_I_SHFT)
#define CPU_PPC_E6500_MMU_MAS2_M                 ((1) << CPU_PPC_E6500_MMU_MAS2_M_SHFT)
#define CPU_PPC_E6500_MMU_MAS2_G                 ((1) << CPU_PPC_E6500_MMU_MAS2_G_SHFT)
#define CPU_PPC_E6500_MMU_MAS2_E                 ((1) << CPU_PPC_E6500_MMU_MAS2_E_SHFT)




#define CPU_PPC_E6500_MMU_MAS2_HELPER(vaddr, flags)  \
    (((vaddr) & CPU_PPC_E6500_MMU_MAS2_EPN_MSK)  | \
     ((flags) & CPU_PPC_E6500_MMU_MAS2_FLAGS_MASK))


#define CPU_PPC_E6500_MMU_MAS3_RPN_MSK           (0xfffff000)
#define CPU_PPC_E6500_MMU_MAS3_RPN_SHFT          (12)
#define CPU_PPC_E6500_MMU_MAS3_USR_MSK           (0x000003c0)
#define CPU_PPC_E6500_MMU_MAS3_USR_SHFT          (6)
#define CPU_PPC_E6500_MMU_MAS3_UX_SHFT           (5)
#define CPU_PPC_E6500_MMU_MAS3_SX_SHFT           (4)
#define CPU_PPC_E6500_MMU_MAS3_UW_SHFT           (3)
#define CPU_PPC_E6500_MMU_MAS3_SW_SHFT           (2)
#define CPU_PPC_E6500_MMU_MAS3_UR_SHFT           (1)
#define CPU_PPC_E6500_MMU_MAS3_SR_SHFT           (0)
#define CPU_PPC_E6500_MMU_MAS3_FLAGS_MASK        (0x0000003f)


#define CPU_PPC_E6500_MMU_MAS3_USR(x)           (((x) << CPU_PPC_E6500_MMU_MAS3_USR_SHFT) & \
						 CPU_PPC_E6500_MMU_MAS3_USR_MSK)
#define CPU_PPC_E6500_MMU_MAS3_UX                ((1) << CPU_PPC_E6500_MMU_MAS3_UX_SHFT)
#define CPU_PPC_E6500_MMU_MAS3_SX                ((1) << CPU_PPC_E6500_MMU_MAS3_SX_SHFT)
#define CPU_PPC_E6500_MMU_MAS3_UW                ((1) << CPU_PPC_E6500_MMU_MAS3_UW_SHFT)
#define CPU_PPC_E6500_MMU_MAS3_SW                ((1) << CPU_PPC_E6500_MMU_MAS3_SW_SHFT)
#define CPU_PPC_E6500_MMU_MAS3_UR                ((1) << CPU_PPC_E6500_MMU_MAS3_UR_SHFT)
#define CPU_PPC_E6500_MMU_MAS3_SR                ((1) << CPU_PPC_E6500_MMU_MAS3_SR_SHFT)



#define CPU_PPC_E6500_MMU_MAS3_HELPER(rpn, flags)          \
    (((rpn) & CPU_PPC_E6500_MMU_MAS3_RPN_MSK)            | \
     ((flags) & CPU_PPC_E6500_MMU_MAS3_FLAGS_MASK))

#define CPU_PPC_E6500_MMU_MAS5_SGS_SHFT          (31)
#define CPU_PPC_E6500_MMU_MAS5_SGS_MSK           (0x80000000)
#define CPU_PPC_E6500_MMU_MAS5_SLPID_SHFT        (5)
#define CPU_PPC_E6500_MMU_MAS5_SLPID_MSK         (0x0000003f)



#define CPU_PPC_E6500_MMU_MAS5_HELPER(sgs, slpid)           \
    ((((sgs) << CPU_PPC_E6500_MMU_MAS5_SGS_SHFT) & CPU_PPC_E6500_MMU_MAS5_SGS_MSK) | \
     (((slpid) << CPU_PPC_E6500_MMU_MAS5_SLPID_SHFT) & CPU_PPC_E6500_MMU_MAS5_SLPID_MSK))


#define CPU_PPC_E6500_MMU_MAS6_SPID_SHFT         (16)
#define CPU_PPC_E6500_MMU_MAS6_SPID_MSK          (0x3fff0000)
#define CPU_PPC_E6500_MMU_MAS6_ISIZE_SHFT        (7)
#define CPU_PPC_E6500_MMU_MAS6_ISIZE_MSK         (0x00000F80)
#define CPU_PPC_E6500_MMU_MAS6_SIND_SHFT         (1)
#define CPU_PPC_E6500_MMU_MAS6_SIND_MSK          (0x00000002)
#define CPU_PPC_E6500_MMU_MAS6_SAS_SHFT          (0)
#define CPU_PPC_E6500_MMU_MAS6_SAS_MSK           (0x00000001)


#define CPU_PPC_E6500_MMU_MAS6_HELPER(spid, sas, isize,sind)		\
    ((((spid)  << CPU_PPC_E6500_MMU_MAS6_SPID_SHFT)    & CPU_PPC_E6500_MMU_MAS6_SPID_MSK)    | \
     (((isize) << CPU_PPC_E6500_MMU_MAS6_ISIZE_SHFT)   & CPU_PPC_E6500_MMU_MAS6_ISIZE_MSK)   | \
     (((sind)  << CPU_PPC_E6500_MMU_MAS6_SIND_SHFT)    & CPU_PPC_E6500_MMU_MAS6_SIND_MSK)    | \
     (((sas)   << CPU_PPC_E6500_MMU_MAS6_SAS_SHFT)     & CPU_PPC_E6500_MMU_MAS6_SAS_MSK))



#define CPU_PPC_E6500_MMU_MAS7_RPN_SHFT          (32)
#define CPU_PPC_E6500_MMU_MAS7_RPN_MSK           (0x000000ff)

#define CPU_PPC_E6500_MMU_MAS7_HELPER(rpn_upper)              \
    (((rpn_upper) & CPU_PPC_E6500_MMU_MAS7_RPN_MSK))



#define CPU_PPC_E6500_MMU_MAS8_TGS_SHFT          (31)
#define CPU_PPC_E6500_MMU_MAS8_VF_SHFT           (30)
#define CPU_PPC_E6500_MMU_MAS8_TLPID_MSK         (0x0000003f)
#define CPU_PPC_E6500_MMU_MAS8_TLPID_SHFT        (0)

#define CPU_PPC_E6500_MMU_MAS8_TGS               (1 << CPU_PPC_E6500_MMU_MAS8_TGS_SHFT)
#define CPU_PPC_E6500_MMU_MAS8_VF                (1 << CPU_PPC_E6500_MMU_MAS8_VF_SHFT)

#endif /* CPU_PPC_E6500_MMU_H_ */
