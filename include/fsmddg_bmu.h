/*!
 * \file fsmddg_bmu.h
 *
 * \brief interface for BMU FPGA to/from other kernel drivers
 *
 * \version 0.4
 *
 * \history
 *    30.09.2010 btz initial version of file 0.1\n
 *    05.10.2010 btz modification of interface to I2C\n
 *    02.05.2011 btz added enum identifiers 'BMU_FIRST' and 'BMU_LAST'\n
 *    04.04.2012 btz - adaptation to new LED states (for uREC FIFC)\n
 *                   - adaptation for expanded slot addressing for FIFC\n
 *
 * \section DESCRIPTION
 *    Specifies interfacees between BMU control driver and other (dependent) kernel
 *    drivers
 *    Currently this is used for GPIO and LED subsystems.
 *
 * \section AUTHORS
 *    Bernhard Bentz (btz), bernhard.bentz@nsn.com, Tel. +49 89 / 5159 - 38114\n
 *
 * \section COPYRIGHT
 *     FSM3.0 Project Copyright (C) Nokia 2014
 *
 * \endfile
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


#ifndef _fsmddg_bmu_h_
#define _fsmddg_bmu_h_


/* specification of LED brightness.
 * Note: These values are also used for SYS-FS. So always check against
 *       DDAL implementation (interface to SYS-FS part).
 *       However these values might be different from DDAL API definitions.
 */
typedef enum {
    LED_OFF,                /*  0 */
    LED_GREEN,              /*  1 */   /* for color providing LEDs */
    LED_RED,                /*  2 */   /* for color providing LEDs */
    LED_YELLOW,             /*  3 */   /* for color providing LEDs */
    LED_GREEN_BLINK,        /*  4 */   /* for color and blinking providing LEDs */
    LED_RED_BLINK,          /*  5 */   /* for color and blinking providing LEDs */
    LED_YELLOW_BLINK,       /*  6 */   /* for color and blinking providing LEDs */
    LED_GREEN_YELLOW_BLINK, /*  7 */   /* for dual color blinking providing LEDs */
    LED_RED_YELLOW_BLINK,   /*  8 */   /* for dual color blinking providing LEDs */
    LED_GREEN_RED_BLINK,    /*  9 */   /* for dual color blinking providing LEDs */
    LED_ON,                 /* 10 */   /* for non-color providing LEDs */
    LED_BLINK_1HZ,          /* 11 */   /* for non-color providing LEDs with dual blink rate */
    LED_BLINK_2HZ           /* 12 */   /* for non-color providing LEDs with dual blink rate */
} led_state_t;


/* specification of slot/location of (daughter) boards in the FSM3/FSM4 system */
/* do not use identifiers 'BMU_...' any more but use identifiers 'SLOT_...' */

#ifdef CONFIG_FSM4_ARM
typedef enum {
    SLOT_FIRST = 0,     /* 0 */
    SLOT_FSP1,          /* 1 */
    SLOT_FSP2,          /* 2 */
    SLOT_FSP3,          /* 3 */
    SLOT_FSP4,          /* 4 */
    SLOT_FSP5           /* 5 */
    SLOT_FSP6           /* 6 */
    SLOT_LAST = SLOT_FSP6 /* 6 */
} sm3_port_t;
#else
typedef enum {
    BMU_FIRST = 0,             /* 0 */
    SLOT_FIRST = BMU_FIRST,    /* 0 */
    BMU_FSP1 = BMU_FIRST,      /* 0 */
    SLOT_FSP1 = BMU_FSP1,      /* 0 */

    BMU_FSP2,                  /* 1 */
    SLOT_FSP2 = BMU_FSP2,      /* 1 */

    BMU_FSP3,                  /* 2 */
    SLOT_FSP3 = BMU_FSP3,      /* 2 */

    BMU_FTIF,                  /* 3 */
    BMU_LAST = BMU_FTIF,       /* 3 */
    SLOT_FTIF = BMU_FTIF,      /* 3 */
    SLOT_FTM = BMU_FTIF,       /* 3 */

    SLOT_FQD,                  /* 4 */

    SLOT_FQG,                  /* 5 */

    SLOT_FIFC,                 /* 6 */
    
    BMU_FSP4,                  /* 7 */
    SLOT_FSP4 = BMU_FSP4,      /* 7 */
    
    BMU_FSP5,                  /* 8 */
    SLOT_FSP5 = BMU_FSP5,      /* 8 */

    BMU_FSP6,                  /* 9 */
    SLOT_FSP6 = BMU_FSP6,      /* 9 */

    SLOT_LAST = SLOT_FSP6      /* 9 */
} sm3_port_t;
#endif

int gpio_twsi_device_register( /* function to register all GPIO pins of one BMU */
        int numGpio,           /* number of BMU GPIO pins to be serviced */
        const char *name[],    /* pointer to array of GPIO pin names */
        sm3_port_t port,       /* port/slot where board is plugged */
        int(*bmu_gpio_get)(int gpioIdx, uint8_t *val, void *data),
                               /* pointer to function to read the state of BMU GPIO pin */
        int(*bmu_gpio_set)(int gpioIdx, uint8_t val, void *data),
                               /* pointer to function to write the state of BMU GPIO pin */
        void *data             /* pointer to specific device data */
    );
void gpio_twsi_device_unregister(int gpioDevId);  /* function to unregister all GPIO pins of one BMU */



int led_twsi_device_register( /* function to register all BMU driven LEDs */
        int numLed,           /* number of BMU driven LEDs to be serviced */
        const char *name[],   /* pointer to array of BMU driven LED names */
        sm3_port_t port,      /* port/slot where board is plugged */
        int(*bmu_led_get)(int ledIdx, led_state_t *state, void *data),
                              /* pointer to function to read the state of a BMU driven LED */
        int(*bmu_led_set)(int ledIdx, led_state_t state, void *data),
                              /* pointer to function to write the state of a BMU driven LED */
        void *data            /* pointer to specific device data */
    );
void led_twsi_device_unregister(int ledDevId);   /* function to unregister all BMU driven LEDs */


#endif /* _fsmddg_bmu_h_ */

