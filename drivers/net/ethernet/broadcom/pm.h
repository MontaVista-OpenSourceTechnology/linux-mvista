/*
 * Copyright (C) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _PM_H
#define _PM_H


#define pmLoopbackMac	0
#define pmLoopbackPhy	1

struct iproc_pm_stats {
	u64 rx_frames;//XLMIB_GRxPkt
	u64 rx_frame_good;//XLMIB_GRxPOK
	u64 rx_bytes;//XLMIB_GRxByt
	u64 rx_frame_64;//XLMIB_GRx64
	u64 rx_frame_127;//XLMIB_GRx127
	u64 rx_frame_255;//XLMIB_GRx255
	u64 rx_frame_511;//XLMIB_GRx511
	u64 rx_frame_1023;//XLMIB_GRx1023
	u64 rx_frame_1518;//XLMIB_GRx1518
	u64 rx_frame_1522;//XLMIB_GRx1522
	u64 rx_frame_jumbo;//XLMIB_GRx2047 + XLMIB_GRx4095 + XLMIB_GRx9216+ XLMIB_GRx16383
	u64 rx_frame_unicast;//XLMIB_GRxUCA
	u64 rx_frame_multicast;//XLMIB_GRxMCA
	u64 rx_frame_broadcast;//XLMIB_GRxBCA
	u64 rx_frame_control;//XLMIB_GRxCF
	u64 rx_frame_pause;//XLMIB_GRxPF
	u64 rx_frame_jabber;//XLMIB_GRxJBR
	u64 rx_frame_fragment;//XLMIB_GRxFRG
	u64 rx_frame_vlan;//XLMIB_GRxVLN
	u64 rx_frame_dvlan;//XLMIB_GRxDVLN
	u64 rx_frame_fcs_error;//XLMIB_GRxFCS
	u64 rx_frame_unsupport;//XLMIB_GRxUO
	u64 rx_frame_wrong_sa;//XLMIB_GRxWSA
	u64 rx_frame_align_err;//XLMIB_GRxALN
	u64 rx_frame_length_err;//XLMIB_GRxFLR
	u64 rx_frame_oversize;//XLMIB_GRxOVR
	u64 rx_frame_mtu_err;//XLMIB_GRxMTUE
	u64 rx_frame_truncated_err;//XLMIB_GRxTRFU
	u64 rx_frame_undersize;//XLMIB_GRxUND
	u64 tx_frames;//XLMIB_GTxPkt
	u64 tx_frame_good;//XLMIB_GTxPOK
	u64 tx_bytes;//XLMIB_GTxBYT
	u64 tx_frame_64;//XLMIB_GTx64
	u64 tx_frame_127;//XLMIB_GTx127
	u64 tx_frame_255;//XLMIB_GTx255
	u64 tx_frame_511;//XLMIB_GTx511
	u64 tx_frame_1023;//XLMIB_GTx1023
	u64 tx_frame_1518;//XLMIB_GTx1518
	u64 tx_frame_1522;//XLMIB_GTx1522
	u64 tx_frame_jumbo;//XLMIB_GTx2047 + XLMIB_GTx4095 + XLMIB_GTx9216 + XLMIB_GTx16383
	u64 tx_frame_unicast;//XLMIB_GTxUCA
	u64 tx_frame_multicast;//XLMIB_GTxMCA
	u64 tx_frame_broadcast;//XLMIB_GTxBCA
	u64 tx_frame_control;//XLMIB_GTxCF
	u64 tx_frame_pause;//XLMIB_GTxPF
	u64 tx_frame_jabber;//XLMIB_GTxJBR
	u64 tx_frame_fragment;//XLMIB_GTxFRG
	u64 tx_frame_vlan;//XLMIB_GTxVLN
	u64 tx_frame_dvlan;//XLMIB_GTxDVLN
	u64 tx_frame_fcs_error;//XLMIB_GTxFCS
	u64 tx_frame_oversize;//XLMIB_GTxOVR
	u64 tx_frame_error;//XLMIB_GTxErr
	u64 tx_frame_fifo_underrun;//XLMIB_GTxUFL
	u64 tx_frame_collision;//XLMIB_GTxNCL
};

struct iproc_pm_ops {
	int (*port_enable)(int port, int enable);
	int (*port_speed)(int port, int speed);
	int (*port_loopback)(int port, int lb_type, int lb_en);
	int (*port_mac_addr)(int port, u8 *mac);
	int (*port_stats)(int port, struct iproc_pm_stats *stats);
	int (*port_stats_clear)(int port);
};

extern int pm4x10_pm_init(struct iproc_pm_ops *pm_ops, u8 lane_idx);
extern int pm4x10_pm_deinit(struct iproc_pm_ops *pm_ops);

extern int pm4x10_pm_xlport_port_config(int port, int enable);
extern int pm4x10_xlport_speed_set(int port, int speed);
extern int pm4x10_xlport_loopback_set(int port, int lb_type, int lb_en);
extern int pm4x10_xlport_mac_addr_set(int port, u8 *mac);
extern int pm4x10_xlport_stats_get(int port, struct iproc_pm_stats *stats);
extern int pm4x10_xlport_mib_reset(int port);

#endif /* _PM_H */
