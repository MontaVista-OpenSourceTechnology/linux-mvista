
obj-y += timer.o edma.o cp_intc.o cpu_speed.o

# Common code for puma init
obj-y += core.o

# PT 1.5 common code
obj-$(CONFIG_PT_1_5) += mac_addr.o

# Board specific init code
obj-$(CONFIG_MACH_SCMB)     += board-scmb.o
