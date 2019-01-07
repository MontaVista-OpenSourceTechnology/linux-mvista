/*
 * Copyright (C) 2014 - 2019 MontaVista, Software, LLC.
 * Copyright (C) 2011 Ericsson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/memory.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/nvmem-consumer.h>

#include <mach/emac.h>
#include <mach/core.h>

/*-------------------------------------------------------------------------*/
/*-                  NETWORK MAC ADDR CONFIG (I2C EEPROM)                 -*/
/*-------------------------------------------------------------------------*/

/* EEPROM: Shelf Invetory - Device MAC */
/* Device MAC
 * MAC Address: Address = 0, Size = 18. "Device MAC address in ASCII format, e.g. 00:04:7A:1C:FA:30. Must be terminated by 0."
 * Reserved:    Address = 18, Size = 13. Reserved for future use.
 * Checksum:    Address = 31, Size = 1. Add all bytes from 0 - 30.
 */

typedef struct eeprom_device_mac_s {
	char mac_address_string[18]; /* in ASCII format */
	char reserved[13];
	unsigned char checksum;
} eeprom_device_mac_t;

static eeprom_device_mac_t eeprom_device_mac = {{0}, {0}, 0};

extern char *PUMA_mac_addr;

static unsigned char checksum_calculate(unsigned char *byte_data_array, int data_size)
{
	int byte_cnt = 0;
	unsigned char local_checksum = 0;

	/* Calculate the 1 byte checksum as the sum of all bytes wrapped in 1 byte */
	for (byte_cnt = 0; byte_cnt < data_size; byte_cnt++)
		local_checksum += byte_data_array[byte_cnt];
	return local_checksum;
}

void print_mac_address(char *mac_addr)
{
	int cnt;

	/* Convert the decimal MAC Address in the equivalent ASCII string */
	for (cnt = 0; cnt < ETH_ALEN; cnt++) {
		int mac_address_decimal_digit = mac_addr[cnt];
		char mac_address_ascii_digit[2] = {0};

		/* Convert the single MAC Address digit in its ASCII representation */
		sprintf(mac_address_ascii_digit, "%02X", mac_address_decimal_digit);
		memcpy(&mac_addr[3 * cnt], mac_address_ascii_digit, 2);

		/* Insert the ':' character at the end of every ASCII digit, excluding the last one */
		if (cnt != (ETH_ALEN - 1))
			mac_addr[(3 * cnt) + 2] = ':';
	}
}

void read_inventory_mac(struct nvmem_device *a, void *context)
{
	int read_bytes = 0;
	int write_bytes = 0;
	int write = 0;
	int cnt = 0;
	char mac_digit_string[3];
	unsigned char checksum_calculated = 0;
	char mac_address[ETH_ALEN] = {0};

	/* Read EEPROM Shelf Inventory - Device MAC component */
	read_bytes = nvmem_device_read(a, 0, sizeof(eeprom_device_mac_t), (char *)&eeprom_device_mac);

	/* Check if all bytes have been read correctly */
	if (read_bytes != sizeof(eeprom_device_mac_t)) {
		printk("PT_1.5: Read EEPROM Device MAC Failed!\n");
		write = 1;
		goto write_def_config;
	}

	/* Verify the checksum on all the bytes from 0 to 30 (excluding the checksum itself) */
	checksum_calculated = checksum_calculate((unsigned char *)&eeprom_device_mac, sizeof(eeprom_device_mac_t) - 1);
	if (checksum_calculated != eeprom_device_mac.checksum) {
		printk("PT_1.5: EEPROM Device MAC Checksum Failed! Read Checksum = 0x%02X, Calculated Checksum = 0x%02X\n", eeprom_device_mac.checksum, checksum_calculated);
		write = 1;
		goto write_def_config;
	}

	printk("PT_1.5: Read EEPROM Inventory Device MAC = %s\n", eeprom_device_mac.mac_address_string);

	/* Convert the ASCII MAC address in the corresponding numeric value */
	for (cnt = 0; cnt < ETH_ALEN; cnt++) {
		/* Clear the local string containig a single MAC address ASCII digit */
		memset(mac_digit_string, 0, sizeof(mac_digit_string));

		/* Copy the single MAC address digit without the ':' character */
		memcpy(mac_digit_string, &eeprom_device_mac.mac_address_string[3 * cnt], 2);

		/* Convert the hexadecimal ASCII digit in the equivalent decimal number */
		mac_address[cnt] = (char)simple_strtoul(mac_digit_string, NULL, 16);
	}

	if (!is_valid_ether_addr(mac_address)) {
		printk("PT_1.5: Retrieved INVALID MAC Address!\n");
		write = 1;
		goto write_def_config;
	}

write_def_config:
	if (write) {
		char random_mac_address_string[18] = {0};

		/* Clear the MAC Address variable */
		memset(mac_address, 0, ETH_ALEN);

		/* Generate a Randon MAC Address */
		random_ether_addr(mac_address);
		printk("PT_1.5: Writing Random MAC Address\n");

		print_mac_address(random_mac_address_string);
		printk("PT_1.5: EEPROM Random Device MAC = %s\n", random_mac_address_string);

		/* Fill the EEPROM Device MAC structure and then calculate its checksum */
		memcpy(eeprom_device_mac.mac_address_string, random_mac_address_string, sizeof(eeprom_device_mac.mac_address_string));
		memset(eeprom_device_mac.reserved, 0, sizeof(eeprom_device_mac.reserved));

		checksum_calculated = checksum_calculate((unsigned char *)&eeprom_device_mac, sizeof(eeprom_device_mac_t) - 1);
		eeprom_device_mac.checksum = checksum_calculated;

		/* Write the Device MAC structure in the Inventory EEPROM */
		write_bytes = nvmem_device_write(a, 0, sizeof(eeprom_device_mac_t), (char *)&eeprom_device_mac);
		mdelay(10);
	}

	memcpy(PUMA_mac_addr, mac_address, ETH_ALEN);
}
