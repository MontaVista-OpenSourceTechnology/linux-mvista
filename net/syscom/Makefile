#
# Makefile for the AGIB syscom network protocol driver.
#

ccflags-$(CONFIG_FSM4_AXM) += -DCONFIG_SYSCOM_REASSEMBLY_REPAIR=1

obj-$(CONFIG_AGIB_SYSCOM) = syscom.o
syscom-y = af.o dgram.o raw.o common.o route.o route-dev.o route-local.o route-gw.o gw.o service-socket.o notify.o trace.o delivery.o
