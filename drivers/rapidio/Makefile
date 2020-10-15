# SPDX-License-Identifier: GPL-2.0
#
# Makefile for RapidIO interconnect services
#
obj-$(CONFIG_RAPIDIO) += rapidio.o

rapidio-y := rio.o rio-access.o rio-driver.o rio-sysfs.o

ifeq ($(CONFIG_RAPIDIO_MEM_MAP),y)
rapidio-y += rio-mem-map.o
endif

obj-$(CONFIG_RAPIDIO_ENUM_BASIC) += rio-scan.o
obj-$(CONFIG_RAPIDIO_CHMAN)	+= rio_cm.o
obj-$(CONFIG_RAPIDIO_DEV)       += rio-dev.o

obj-$(CONFIG_RAPIDIO)		+= switches/
obj-$(CONFIG_RAPIDIO)		+= devices/

subdir-ccflags-$(CONFIG_RAPIDIO_DEBUG) := -DDEBUG
