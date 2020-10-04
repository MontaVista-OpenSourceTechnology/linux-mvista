#ifndef __FSMDDG_SFP_H__
#define __FSMDDG_SFP_H__
/*!
 *      FSM3.0 Project Copyright (C) Nokia 2014
 *
 * @file        sfp.h
 *
 * @subsystem   fsmddg
 *
 * @author      Mikko Mutanen, mikko.mutanen@nsn.com
 *
 * @brief       Header file for SFP subsystem
 *
 * @version     1.0
 *
 * @date        06.04.2010
 *
 * @see
 *
 * history      06.04.2010  mm          initial version\n
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/**
 * struct sfp_gpio - Structure containing gpio numbers of sfp device
 * if driver uses it's own gpio read and write functions
 * these gpios can have any values that only the driver knows.
 * @properties: flags to describe optional functionalities
 * @tx_fault:	gpio number of TX_FAULT
 * @rx_los:	gpio number of RX_LOS
 * @mod_abs:	gpio number of MOD_ABS
 * @rs0:	gpio number of RS0
 * @rs1:	gpio number of RS1
 * @tx_disable:	gpio number of TX_DISABLE
 */
struct sfp_gpio {
        u32 properties;
        int tx_fault;
        int rx_los;
        int mod_abs;
        int rs0;
        int rs1;
        int tx_disable;
};

/* sfp_gpio.properties definitions */

/* is tx_fault signal capable to be used as interrupt source */
#define TX_FAULT_INT_CAP 0x01
/* is RX_LOS signal capable to be used as interrupt source */
#define RX_LOS_INT_CAP   0x02
/* is MOD_ABS signal capable to be used as interrupt source */
#define MOD_ABS_INT_CAP  0x04
/* is RS0 implemented as electrical contact */
#define RS0_EXISTS       0x08
 /* is RS1 implemented as electrical contact */
#define RS1_EXISTS       0x10

/**
 * struct sfp_gpio_info - GPIO related information about SFP cage and it's
 * connections.
 * @index:	index of physical SFP cage
 * @gpio_data:  gpio specific data
 * @gpio_get:   function to get gpio pin status, leave NULL if linux GPIO
 *              subsystem is used.
 * @gpio_set:   function to set gpio pin status,leave NULL if linux GPIO
 *              subsystem is used
 * @data:	used as last parameter when calling gpio_set and gpio_get
 */
struct sfp_gpio_info {
	int index;
	struct sfp_gpio gpio_data;	
	int(*gpio_get)(unsigned gpio, void*);	
	void(*gpio_set)(unsigned gpio, int value, void*);
	void *data;
	int prev_state;
	int mod_abs_prev_state;
	};

/* SFP eeprom map types */
typedef enum{
	SFP_MAP_ID,		/* id map in 0x50 */
	SFP_MAP_DIAGNOSTICS,    /* diagnostics map in 0x51 */
} fsmddg_sfp_map_t;


/**
 * struct sfp_twsi_info - TWSI related information about SFP cage and it's
 * connections.
 * @index:	index of physical SFP cage
 * @type:	sfp eeprom map type
 * @client:     pointer to i2c_client, this is preferred way to communicate
 *              with SFP transceivers. If no real i2c driver exist for
 *              particular SFP cage, this may be left as NULL.
 * @i2c_read:   alternative read function, leave NULL if client available
 * @i2c_write:  alternative write function, leave NULL if client available
 * @properties: properties of alternative twsi functions
 * @data:	used as last parameter when calling i2_read and i2c_write
 */
struct sfp_twsi_info{
	int index;			
	fsmddg_sfp_map_t type;
	struct i2c_client* client;	

	int(*i2c_read)(struct i2c_client* client,
	               int reg_addr,
		       char*,
		       int count,
		       void*);

	int(*i2c_write)(struct i2c_client* client,
	                int reg_addr,
			char*,
			int count,
			void*);

	u32 properties;
	void *data;
};

/* sfp_twsi_info.properties definitions */
#define TWSI_SEQ_R_CAP 0x1	/* read function can do sequential read */
#define TWSI_SEQ_W_CAP 0x2	/* write function can do sequential write */

/* TWSI related functions */
int sfp_twsi_device_register(struct sfp_twsi_info *dev_info);
void sfp_twsi_device_unregister(struct sfp_twsi_info *dev_info);

/* GPIO related functions */
int sfp_gpio_device_register(struct sfp_gpio_info *dev_info);
void sfp_gpio_device_unregister(struct sfp_gpio_info *dev_info);

/* notifications */
void sfp_tx_fault_notify(struct sfp_gpio_info *dev_info);
void sfp_rx_los_notify(struct sfp_gpio_info *dev_info);
void sfp_mod_abs_notify(struct sfp_gpio_info *dev_info);

typedef enum {
	sfp_mod_abs,
	sfp_tx_fault,
	sfp_rx_los,
} sfp_sig_t;

int sfp_status_get(int sfp, sfp_sig_t );
#endif /*__FSMDDG_SFP_H__*/
