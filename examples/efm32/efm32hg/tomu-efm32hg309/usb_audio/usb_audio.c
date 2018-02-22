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
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

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

static const struct {
	struct usb_audio_header_descriptor_head header_head;
	struct usb_audio_header_descriptor_body header_body;
	struct usb_audio_input_terminal_descriptor input_terminal_desc;
	struct usb_audio_feature_unit_descriptor_2ch feature_unit_desc;
	struct usb_audio_output_terminal_descriptor output_terminal_desc;
} __attribute__((packed)) audio_control_functional_descriptors = {
	.header_head = {
		.bLength = sizeof(struct usb_audio_header_descriptor_head) +
		           1 * sizeof(struct usb_audio_header_descriptor_body),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_HEADER,
		.bcdADC = 0x0100,
		.wTotalLength =
			   sizeof(struct usb_audio_header_descriptor_head) +
			   1 * sizeof(struct usb_audio_header_descriptor_body) +
			   sizeof(struct usb_audio_input_terminal_descriptor) +
			   sizeof(struct usb_audio_feature_unit_descriptor_2ch) +
			   sizeof(struct usb_audio_output_terminal_descriptor),
		.binCollection = 1,
	},
	.header_body = {
		.baInterfaceNr = 0x01,
	},
	.input_terminal_desc = {
		.bLength = sizeof(struct usb_audio_input_terminal_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_INPUT_TERMINAL,
		.bTerminalID = 1,
		.wTerminalType = 0x0710, /* XXX: Radio receiver */
		.bAssocTerminal = 0,
		.bNrChannels = 2,
		.wChannelConfig = 0x0003, /* XXX: Left & Right channels */
		.iChannelNames = 0,
		.iTerminal = 0,
	},
	.feature_unit_desc = {
		.head = {
			.bLength = sizeof(struct usb_audio_feature_unit_descriptor_2ch),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = USB_AUDIO_TYPE_FEATURE_UNIT,
			.bUnitID = 2,
			.bSourceID = 1, /* Input terminal 1 */
			.bControlSize = 2,
			.bmaControlMaster = 0x0001, /* XXX: 'Mute' is supported */
		},
		.channel_control = { {
			.bmaControl = 0x0000,
		}, {
			.bmaControl = 0x0000,
		} },
		.tail = {
			.iFeature = 0x00,
		}
	},
	.output_terminal_desc = {
		.bLength = sizeof(struct usb_audio_output_terminal_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_OUTPUT_TERMINAL,
		.bTerminalID = 3,
		.wTerminalType = 0x0101, /* XXX: USB Streaming */
		.bAssocTerminal = 0,
		.bSourceID = 0x02, /* Feature unit 2 */
		.iTerminal = 0,
	}
};

static const struct usb_interface_descriptor audio_control_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_CONTROL,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.extra = &audio_control_functional_descriptors,
	.extralen = sizeof(audio_control_functional_descriptors)
} };

static const struct {
	struct usb_audio_stream_interface_descriptor audio_cs_streaming_iface_desc;
	struct usb_audio_format_type1_descriptor_1freq audio_type1_format_desc;
} __attribute__((packed)) audio_streaming_functional_descriptors = {
	.audio_cs_streaming_iface_desc = {
		.bLength = sizeof(struct usb_audio_stream_interface_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = 1, /* XXX: AS_GENERAL */
		.bTerminalLink = 3, /* Terminal 3 */
		.bDelay = 0,
		.wFormatTag = 0x0001 /* XXX: PCM Format */,
	},
	.audio_type1_format_desc = {
		.head = {
			.bLength = sizeof(struct usb_audio_format_type1_descriptor_1freq),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = 2, /* XXX: FORMAT_TYPE */
			.bFormatType = 1,
			.bNrChannels = 2,
			.bSubFrameSize = 2,
			.bBitResolution = 16, /* Should be 10, but 16 is more reliable */
			.bSamFreqType = 1, /* 1 discrete sampling frequency */
		},
		.freqs = { {
			.tSamFreq = 48000,
		} },
	}
};

static const struct usb_audio_stream_audio_endpoint_descriptor audio_streaming_cs_ep_desc[] = { {
	.bLength = sizeof(struct usb_audio_stream_audio_endpoint_descriptor),
	.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
	.bDescriptorSubtype = 1, /* XXX: EP_GENERAL */
	.bmAttributes = 0,
	.bLockDelayUnits = 0x02, /* PCM samples */
	.wLockDelay = 0x0000,
} };

static const struct usb_endpoint_descriptor isochronous_ep[] = { {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82 /* XXX: EP3 In?? */,
	.bmAttributes = USB_ENDPOINT_ATTR_ASYNC | USB_ENDPOINT_ATTR_ISOCHRONOUS,
	.wMaxPacketSize = 64,
	.bInterval = 0x01, /* 1 millisecond */

	/* XXX: not using usb_audio_stream_endpoint_descriptor??
	 * (Why? These are USBv1.0 endpoint descriptors)*/

	.extra = &audio_streaming_cs_ep_desc[0],
	.extralen = sizeof(audio_streaming_cs_ep_desc[0])
} };

static const struct usb_interface_descriptor audio_streaming_iface[] = { {
	/* zerobw streaming interface (alt 0) */
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.extra = 0,
	.extralen = 0,
}, {
	/* Actual streaming interface (alt 1) */
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 1,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = isochronous_ep,

	.extra = &audio_streaming_functional_descriptors,
	.extralen = sizeof(audio_streaming_functional_descriptors)
} };

uint8_t streaming_iface_cur_altsetting = 0;

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = audio_control_iface,
}, {
	.num_altsetting = 2,
	.cur_altsetting = &streaming_iface_cur_altsetting,
	.altsetting = audio_streaming_iface,
} };

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0, /* can be anything, it is updated automatically
			      when the usb code prepares the descriptor */
	.bNumInterfaces = 2, /* control and streaming */
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
uint8_t usbd_control_buffer[128];

void usbaudio_iso_stream_callback(usbd_device *usbd_dev, uint8_t ep)
{
	while (1);
}

static void usbaudio_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_ISOCHRONOUS | USB_ENDPOINT_ATTR_ASYNC, 64, usbaudio_iso_stream_callback);
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

	/* Configure the USB core & stack */
	g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev, &config,
			       usb_strings, 3, usbd_control_buffer,
			       sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(g_usbd_dev, usbaudio_set_config);

	/* Enable USB IRQs */
	nvic_enable_irq(NVIC_USB_IRQ);

	while (1) {
		gpio_toggle(LED_RED_PORT, LED_RED_PIN);

		for (i = 0; i != 200000; ++i)
			__asm__("nop");
	}
}
