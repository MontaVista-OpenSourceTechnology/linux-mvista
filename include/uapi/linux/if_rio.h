/*
 * rionet - definition of fake physical layer RIO address for AF_PACKET
 *
 * Author: Petr Malat
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef IF_RIO_H
#define IF_RIO_H

/** Fake physical layer RIO address */
struct riohdr {
	/** Destination of the packet (must be first) */
	__be16 destination;
	/** Packet source */
	__be16 source;
	/** Mailbox and letter */
	__u8 mbox_letter;
	__u8 padding[3];
} __attribute__(( __packed__));

#define riohdr_get_xmbox(hdr)  ((hdr)->mbox_letter >> 4)
#define riohdr_get_mbox(hdr)   (((hdr)->mbox_letter >> 2) & 0x3)
#define riohdr_get_letter(hdr) ((hdr)->mbox_letter & 0x3)

#define riohdr_set_xmbox(hdr, val)  (void)((hdr)->mbox_letter |= ((val) << 4))
#define riohdr_set_mbox(hdr, val)   (void)((hdr)->mbox_letter |= (((val) & 0x3) << 2))
#define riohdr_set_letter(hdr, val) (void)((hdr)->mbox_letter |= (val) & 0x3)

#endif // IF_RIO_H
