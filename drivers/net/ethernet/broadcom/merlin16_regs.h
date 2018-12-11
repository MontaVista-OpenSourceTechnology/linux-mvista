/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MERLIN16_REGS_H
#define __MERLIN16_REGS_H

/***********************************************************************/
/*                            TSCE registers                           */
/***********************************************************************/

/* Definition of IEEE phyID2 Register (0x0002)	*/
#define CL22_B0_phyid2				0x0002
#define CL22_B0_phyid2_regid1			0

/* Definition of IEEE phyID3 Register (0x0003)	*/
#define CL22_B0_phyid3				0x0003
#define CL22_B0_phyid3_regid2			0

/* Definition of main control register (0x9000)	*/
#define MAIN0_SETUP				0x9000
#define MAIN0_SETUP_reserved			0
#define MAIN0_SETUP_stand_alone_mode		2
#define MAIN0_SETUP_single_port_mode		3
#define MAIN0_SETUP_port_mode_sel		4
#define MAIN0_SETUP_master_port_num		8
#define MAIN0_SETUP_pll_reset_en		10
#define MAIN0_SETUP_cl73_low_vco		11
#define MAIN0_SETUP_cl37_high_vco		12
#define MAIN0_SETUP_refclk_sel			13

/* Definition of LOOPBACK CONTROL register (0x9009) */
#define MAIN0_LOOPBACK_CONTROL			0x9009
#define MAIN0_LOOPBACK_CONTROL_local_pcs_loopback_enable	4
#define MAIN0_LOOPBACK_CONTROL_remote_pmd_loopback_enable	8
#define MAIN0_LOOPBACK_CONTROL_remote_pcs_loopback_enable	12

/* Definition of Serdes ID Register (0x900e) */
#define Main0_serdesID				0x900e
#define Main0_serdesID_model_number		0
#define Main0_serdesID_tech_proc		6
#define Main0_serdesID_bonding			9
#define Main0_serdesID_rev_number		11
#define Main0_serdesID_rev_letter		14

/* Definitions of PMD related registers in TSCE (0x9010) */
#define PMD_X1_CONTROL				0x9010
#define PMD_X1_CONTROL_core_dp_h_rstb		0
#define PMD_X1_CONTROL_por_h_rstb		1
#define PMD_X1_CONTROL_pram_ability		8

/* Definition of PMD lane reset controls (0xc010) */
#define PMD_X4_CONTROL				0xc010
#define PMD_X4_CONTROL_ln_rx_dp_h_rstb		0
#define PMD_X4_CONTROL_ln_rx_h_rstb		1
#define PMD_X4_CONTROL_ln_tx_h_pwrdn		2
#define PMD_X4_CONTROL_ln_rx_h_pwrdn		3
#define PMD_X4_CONTROL_tx_disable		8
#define PMD_X4_CONTROL_osr_mode			9
#define PMD_X4_CONTROL_rx_dme_en		13

/* Definition of PMD lane override (0xc014) */
#define PMD_X4_OVERRIDE				0xc014
#define PMD_X4_OVERRIDE_rx_lock_ovrd		0
#define PMD_X4_OVERRIDE_signal_detect_ovrd	1
#define PMD_X4_OVERRIDE_rx_clk_vld_ovrd		2
#define PMD_X4_OVERRIDE_lane_mode_oen		3
#define PMD_X4_OVERRIDE_rx_osr_mode_oen		4
#define PMD_X4_OVERRIDE_rx_dme_en_oen		5
#define PMD_X4_OVERRIDE_tx_disable_oen		6
#define PMD_X4_OVERRIDE_ln_rx_dp_h_rstb_oen	7
#define PMD_X4_OVERRIDE_tx_clk_vld_ovrd		8
#define PMD_X4_OVERRIDE_tx_osr_mode_oen		9
#define PMD_X4_OVERRIDE_ln_tx_dp_h_rstb_oen	10

/* Definition of SW speed change control (0xc050) */
#define SC_X4_CONTROL_CONTROL			0xc050
#define SC_X4_CONTROL_CONTROL_sw_speed		0
#define SC_X4_CONTROL_CONTROL_sw_speed_change	8

/* Definition of Misc register (0xc113)	*/
#define TX_X4_CONTROL0_MISC			0xc113
#define TX_X4_CONTROL0_MISC_enable_tx_lane	0
#define TX_X4_CONTROL0_MISC_rstb_tx_lane	1
#define TX_X4_CONTROL0_MISC_tx_fifo_watermark	2
#define TX_X4_CONTROL0_MISC_cl49_tx_rf_enable	6
#define TX_X4_CONTROL0_MISC_cl49_tx_lf_enable	7
#define TX_X4_CONTROL0_MISC_cl49_tx_li_enable	8
#define TX_X4_CONTROL0_MISC_fec_enable		10
#define TX_X4_CONTROL0_MISC_scr_mode		14

/* Definition of pma_control_0 register (0xc137) */
#define RX_X4_CONTROL0_PMA_CONTROL_0		0xc137
#define RX_X4_CONTROL0_PMA_CONTROL_0_rstb_lane		0
#define RX_X4_CONTROL0_PMA_CONTROL_0_rx_gbox_afrst_en	1
#define RX_X4_CONTROL0_PMA_CONTROL_0_os_mode		3

/* Definition of CL37 BASE PAGE ABILITIES (0xc181) */
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL37_BASE_ABILITIES	0xc181
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL37_BASE_ABILITIES_sgmii_speed			0
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL37_BASE_ABILITIES_sgmii_full_duplex		2
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL37_BASE_ABILITIES_cl37_full_duplex		4
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL37_BASE_ABILITIES_cl37_half_duplex		5
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL37_BASE_ABILITIES_cl37_pause			6
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL37_BASE_ABILITIES_cl37_next_page			8
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL37_BASE_ABILITIES_sgmii_master_mode		9
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL37_BASE_ABILITIES_an_pd_to_cl37_enable		10
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL37_BASE_ABILITIES_cl37_an_restart_reset_disable	11
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL37_BASE_ABILITIES_cl37_sw_restart_reset_disable	12

/* Definition of Cl37 OVER1G ABILITIES REG 1 register (0xc183) */
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1		0xc183
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_BAM_13GBASE_X4			0
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_BAM_15GBASE_X4			1
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_BAM_15p75GBASE_X2		2
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_BAM_16GBASE_X4			3
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_BAM_20GBASE_X4_CX4		4
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_BAM_20GBASE_X4			5
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_BAM_20GBASE_X2			6
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_BAM_20GBASE_X2_CX4		7
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_BAM_21GBASE_X4			8
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_BAM_25p455GBASE_X4		9
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_BAM_31p5GBASE_X4		10
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_BAM_32p7GBASE_X4		11
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_BAM_40GBASE_X4			12
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_CL72				13
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_FEC				14
#define AN_X4_ABILITIES_LOCAL_DEVICE_OVER1G_ABILITIES_1_HG2				15

/* Definition of CL73 BASE PAGE ABILITIES REG 1 register (0xc185) */
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL73_BASE_ABILITIES_1	0xc185
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL73_BASE_ABILITIES_1_base_selector		0
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL73_BASE_ABILITIES_1_transmit_nonce		5
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL73_BASE_ABILITIES_1_cl73_nonce_match_val		10
#define AN_X4_ABILITIES_LOCAL_DEVICE_CL73_BASE_ABILITIES_1_cl73_nonce_match_over	11


/***********************************************************************/
/*                          Merlin16 registers                         */
/***********************************************************************/

/* Definition of DSC uC Control (0xd00d) */
#define DSC_A_DSC_UC_CTRL			0xd00d
#define DSC_A_DSC_UC_CTRL_uc_dsc_gp_uc_req	0
#define DSC_A_DSC_UC_CTRL_uc_dsc_error_found	6
#define DSC_A_DSC_UC_CTRL_uc_dsc_ready_for_cmd	7
#define DSC_A_DSC_UC_CTRL_uc_dsc_supp_info	8

/* uC command sets */
#define CMD_NULL				0
#define CMD_UC_CTRL				1
#define CMD_HEYE_OFFSET				2
#define CMD_VEYE_OFFSET				3
#define CMD_UC_DBG				4
#define CMD_DIAG_EN				5
#define CMD_READ_UC_LANE_BYTE			6
#define CMD_WRITE_UC_LANE_BYTE			7
#define CMD_READ_UC_CORE_BYTE			8
#define CMD_WRITE_UC_CORE_BYTE			9
#define CMD_READ_UC_LANE_WORD			10
#define CMD_WRITE_UC_LANE_WORD			11
#define CMD_READ_UC_CORE_WORD			12
#define CMD_WRITE_UC_CORE_WORD			13
#define CMD_EVENT_LOG_CTRL			14
#define CMD_EVENT_LOG_READ			15
#define CMD_CAPTURE_BER_START			16
#define CMD_READ_DIAG_DATA_BYTE			17
#define CMD_READ_DIAG_DATA_WORD			18
#define CMD_CAPTURE_BER_END			19
#define CMD_CALC_CRC				20
#define CMD_FREEZE_STEADY_STATE			21
#define CMD_TDT_EN				22
#define CMD_UC_EMULATION			23
#define CMD_CHAR_ISI				24


/* Definition of DSC uC Control (0xd00e) */
#define DSC_A_DSC_SCRATCH			0xd00e
#define DSC_A_DSC_SCRATCH_uc_dsc_scratch	0

/* Definition of OSR_MODE_CONTROL (0xd080) */
#define CKRST_CTRL_OSR_MODE_CONTROL		0xd080
#define CKRST_CTRL_OSR_MODE_CONTROL_osr_mode_frc_val		0
#define CKRST_CTRL_OSR_MODE_CONTROL_osr_mode_frc		15

/* Definition of LANE_CLK_RESET_N_POWERDOWN_CONTROL (0xd081) */
#define RXTXCOM_CKRST_CTRL_LANE_CLK_RESET_N_POWERDOWN_CONTROL	0xd081
#define RXTXCOM_CKRST_CTRL_LANE_CLK_RESET_N_POWERDOWN_CONTROL_ln_dp_s_rstb		0

/* Definition of LANE_RESET_N_PWRDN_PIN_KILL_CONTROL (0xd083) */
#define RXTXCOM_CKRST_CTRL_LANE_RESET_N_PWRDN_PIN_KILL_CONTROL	0xd083
#define RXTXCOM_CKRST_CTRL_LANE_RESET_N_PWRDN_PIN_KILL_CONTROL_pmd_ln_h_rstb_pkill	0
#define RXTXCOM_CKRST_CTRL_LANE_RESET_N_PWRDN_PIN_KILL_CONTROL_pmd_ln_dp_h_rstb_pkill	1

/* Definition of TOP_USER_CONTROL_0 (0xd0f4) */
#define DIGCOM_TOP_USER_CONTROL_0				0xd0f4
#define DIGCOM_TOP_USER_CONTROL_0_heartbeat_count_1us		0
#define DIGCOM_TOP_USER_CONTROL_0_maskdata_bus_assign		10
#define DIGCOM_TOP_USER_CONTROL_0_core_dp_s_rstb		13
#define DIGCOM_TOP_USER_CONTROL_0_afe_s_pll_pwrdn		14
#define DIGCOM_TOP_USER_CONTROL_0_uc_active			15

/* Definition of REVID1 (0xd0fa) */
#define DIGCOM_REVID1				0xd0fa
#define DIGCOM_REVID1_revid_eee			0
#define DIGCOM_REVID1_revid_llp			1
#define DIGCOM_REVID1_revid_pir			2
#define DIGCOM_REVID1_revid_cl72		3
#define DIGCOM_REVID1_revid_micro		4
#define DIGCOM_REVID1_revid_mdio		5
#define DIGCOM_REVID1_revid_multiplicity	12

/* Definition of Clock control registers 0 (0xd200) */
#define MICRO_A_CLOCK_CONTROL0			0xd200
#define MICRO_A_CLOCK_CONTROL0_micro_master_clk_en		0
#define MICRO_A_CLOCK_CONTROL0_micro_core_clk_en		1

/* Definition of Reset control registers 0 (0xd201) */
#define MICRO_A_RESET_CONTROL0			0xd201
#define MICRO_A_RESET_CONTROL0_micro_master_rstb		0
#define MICRO_A_RESET_CONTROL0_micro_core_rstb			1
#define MICRO_A_RESET_CONTROL0_micro_pram_if_rstb		3

/* Definition of rmi to ahb control registers 0 (0xd202) */
#define MICRO_A_AHB_CONTROL0			0xd202
#define MICRO_A_AHB_CONTROL0_micro_ra_wrdatasize		0
#define MICRO_A_AHB_CONTROL0_micro_ra_rddatasize		4
#define MICRO_A_AHB_CONTROL0_micro_ra_init			8
#define MICRO_A_AHB_CONTROL0_micro_autoinc_wraddr_en		12
#define MICRO_A_AHB_CONTROL0_micro_autoinc_rdaddr_en		13

/* Definition of rmi to ahb status registers 0 (0xd203) */
#define MICRO_A_AHB_STATUS0			0xd203
#define MICRO_A_AHB_STATUS0_micro_ra_initdone	0

/* Definition of rmi to ahb write address LSW (bits 15:0) register (0xd204) */
#define MICRO_A_AHB_WRADDR_LSW			0xd204
#define MICRO_A_AHB_WRADDR_LSW_micro_ra_wraddr_lsw		0

/* Definition of rmi to ahb write address MSW (bits 31:16) register (0xd205) */
#define MICRO_A_AHB_WRADDR_MSW			0xd205
#define MICRO_A_AHB_WRADDR_MSW_micro_ra_wraddr_msw		0

/* Definition of rmi to ahb write data LSW (bits 15:0) register (0xd206) */
#define MICRO_A_AHB_WRDATA_LSW			0xd206
#define MICRO_A_AHB_WRDATA_LSW_micro_ra_wrdata_lsw		0

#endif /* __MERLIN16_REGS_H */
