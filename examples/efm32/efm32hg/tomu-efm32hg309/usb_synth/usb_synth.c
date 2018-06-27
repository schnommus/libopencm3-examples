/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2018 Seb Holzapfel <schnommus@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \addtogroup Examples
 *
 * This example implements a USB AUDIO device to demonstrate the use of the
 * USB device stack.
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>
#include <libopencm3/usb/dwc_otg_common.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Default AHB (core clock) frequency of Tomu board */
#define AHB_FREQUENCY		14000000

#define LED_GREEN_PORT		GPIOA
#define LED_GREEN_PIN		GPIO0
#define LED_RED_PORT		GPIOB
#define LED_RED_PIN		GPIO7

#define VENDOR_ID		0x1209	/* pid.code */
#define PRODUCT_ID		0x70b1	/* Assigned to Tomu project */
#define DEVICE_VER		0x0101	/* Program version */

usbd_device *g_usbd_dev = 0;

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,    /* was 0x0110 in Table B-1 example descriptor */
	.bDeviceClass = 0,   /* device defined at interface level */
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = VENDOR_ID,
	.idProduct = PRODUCT_ID,
	.bcdDevice = DEVICE_VER,
	.iManufacturer = 1,  /* index to string desc */
	.iProduct = 2,       /* index to string desc */
	.iSerialNumber = 3,  /* index to string desc */
	.bNumConfigurations = 1,
};

#include "mic_descriptors.h"
#include "midi_descriptors.h"

uint8_t mic_streaming_iface_cur_altsetting = 0;

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = mic_audio_control_iface,
}, {
	.num_altsetting = 2,
	.cur_altsetting = &mic_streaming_iface_cur_altsetting,
	.altsetting = mic_audio_streaming_iface,
}, {
	.num_altsetting = 1,
	.altsetting = midi_audio_control_iface,
}, {
	.num_altsetting = 1,
	.altsetting = midi_streaming_iface,
} };

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0, /* can be anything, it is updated automatically
                          when the usb code prepares the descriptor */
	.bNumInterfaces = 4, /* control/stream [audio] + control/stream [midi] */
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80, /* bus powered */
	.bMaxPower = 0x32,
	.interface = ifaces,
};

static char usb_serial_number[25]; /* 12 bytes of desig and a \0 */

static const char * usb_strings[] = {
	"libopencm3.org",
	"AUDIO demo",
	usb_serial_number
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[512];

#define DATA_SIZE 16

int16_t data[DATA_SIZE] = {0};

void init_data() {
    for(int i = 0; i != DATA_SIZE/2; ++i) {
        data[i*2] = i;
        data[i*2+1] = i;
    }
}

#define REBASE(x)        MMIO32((x) + (USB_OTG_FS_BASE))

void usbaudio_iso_stream_callback(usbd_device *usbd_dev, uint8_t ep)
{
    static int toggle = 0;

    gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN);

    if(toggle++ % 2 == 0) {
        REBASE(OTG_DIEPCTL(ep)) |= OTG_DIEPCTLX_SD0PID;
    } else {
        REBASE(OTG_DIEPCTL(ep)) |= OTG_DIEPCTLX_SD1PID;
    }

    usbd_ep_write_packet(usbd_dev, 0x82, data, DATA_SIZE*2);
}

/* SysEx identity message, preformatted with correct USB framing information */
const uint8_t sysex_identity[] = {
	0x04,	/* USB Framing (3 byte SysEx) */
	0xf0,	/* SysEx start */
	0x7e,	/* non-realtime */
	0x00,	/* Channel 0 */
	0x04,	/* USB Framing (3 byte SysEx) */
	0x7d,	/* Educational/prototype manufacturer ID */
	0x66,	/* Family code (byte 1) */
	0x66,	/* Family code (byte 2) */
	0x04,	/* USB Framing (3 byte SysEx) */
	0x51,	/* Model number (byte 1) */
	0x19,	/* Model number (byte 2) */
	0x00,	/* Version number (byte 1) */
	0x04,	/* USB Framing (3 byte SysEx) */
	0x00,	/* Version number (byte 2) */
	0x01,	/* Version number (byte 3) */
	0x00,	/* Version number (byte 4) */
	0x05,	/* USB Framing (1 byte SysEx) */
	0xf7,	/* SysEx end */
	0x00,	/* Padding */
	0x00,	/* Padding */
};

static void usbmidi_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

	/* This implementation treats any message from the host as a SysEx
	 * identity request. This works well enough providing the host
	 * packs the identify request in a single 8 byte USB message.
	 */
	if (len) {
		while (usbd_ep_write_packet(usbd_dev, 0x81, sysex_identity,
					    sizeof(sysex_identity)) == 0);
	}

	gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);
}

static void usbaudio_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_ISOCHRONOUS, DATA_SIZE*2, usbaudio_iso_stream_callback);

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64,
			usbmidi_data_rx_cb);

	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);

    /* XXX: This is necessary -> but why? */
    usbd_ep_write_packet(usbd_dev, 0x82, data, DATA_SIZE*2);
}


static void usbmidi_send_note(usbd_device *usbd_dev, int note_on)
{
	char buf[4] = { 0x08, /* USB framing: virtual cable 0, note on */
			0x80, /* MIDI command: note on, channel 1 */
			60,   /* Note 60 (middle C) */
			64,   /* "Normal" velocity */
	};

	buf[0] |= note_on;
	buf[1] |= note_on << 4;

	usbd_ep_write_packet(usbd_dev, 0x81, buf, sizeof(buf));
}

void usb_isr(void)
{
	usbd_poll(g_usbd_dev);
}

void hard_fault_handler(void)
{
	while (1);
}

int main(void)
{
	int i;

	/* Make sure the vector table is relocated correctly
	 * (after the Tomu bootloader) */
	SCB_VTOR = 0x4000;

	/* Disable the watchdog that the bootloader started. */
	WDOG_CTRL = 0;

	/* GPIO peripheral clock is necessary for us to set
	 * up the GPIO pins as outputs */
	cmu_periph_clock_enable(CMU_GPIO);

	/* Set up both LEDs as outputs */
	gpio_mode_setup(LED_RED_PORT, GPIO_MODE_WIRED_AND, LED_RED_PIN);
	gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_WIRED_AND, LED_GREEN_PIN);

    gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);

    init_data();

	/* Configure the USB core & stack */
	g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev, &config,
			       usb_strings, 3, usbd_control_buffer,
			       sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(g_usbd_dev, usbaudio_set_config);

	/* Enable USB IRQs */
	nvic_enable_irq(NVIC_USB_IRQ);

	while (1) {
		gpio_toggle(LED_RED_PORT, LED_RED_PIN);
        gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);

		for (i = 0; i != 200000; ++i)
			__asm__("nop");
		usbmidi_send_note(g_usbd_dev, 1);
	}
}
