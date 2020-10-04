# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the TI network device drivers.
#

obj-$(CONFIG_TI_CPSW) += cpsw-common.o
obj-$(CONFIG_TI_DAVINCI_EMAC) += cpsw-common.o

obj-$(CONFIG_TLAN) += tlan.o
obj-$(CONFIG_CPMAC) += cpmac.o
obj-$(CONFIG_TI_DAVINCI_EMAC) += davinci_emac.o
obj-$(CONFIG_TI_DAVINCI_MDIO) += davinci_mdio.o
obj-$(CONFIG_TI_DAVINCI_CPDMA) += davinci_cpdma.o
obj-$(CONFIG_TI_CPSW_PHY_SEL) += cpsw-phy-sel.o
obj-$(CONFIG_TI_CPSW_ALE) += cpsw_ale.o
obj-$(CONFIG_TI_CPTS_MOD) += cpts.o
obj-$(CONFIG_TI_CPSW) += ti_cpsw.o
ti_cpsw-y := cpsw.o

obj-$(CONFIG_TI_KEYSTONE_NETCP) += keystone_netcp.o
keystone_netcp-y := netcp_core.o
obj-$(CONFIG_TI_KEYSTONE_NETCP_ETHSS) += keystone_netcp_ethss.o
keystone_netcp_ethss-y := netcp_ethss.o netcp_sgmii.o netcp_xgbepcsr.o
obj-$(CONFIG_TI_KEYSTONE_NET) += keystone_net.o
obj-$(CONFIG_TI_KEYSTONE_PA) += keystone_pa.o keystone_pa2.o
obj-$(CONFIG_TI_KEYSTONE_QOS) += keystone_qos.o
obj-$(CONFIG_TI_KEYSTONE_NET_SA) += keystone_net_sa.o
keystone_net-y += cpsw_ale.o		\
		  keystone_ethss.o	\
		  keystone_sgmii.o	\
		  keystone_net_core.o	\
		  keystone_serdes.o	\
		  phy-keystone-serdes.o	\
		  keystone_ethss2.o
obj-$(CONFIG_TI_KEYSTONE_XGE) += keystone_xge.o
obj-$(CONFIG_TI_KEYSTONE_XGE_MDIO) += keystone_xgemdio.o
keystone_xge-y += keystone_xgess.o	\
		  keystone_xgepcsr.o
