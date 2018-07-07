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

#include "midi_events.h"

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
uint8_t usbd_control_buffer[1024];

#define WAVEFORM_SAMPLES 256 /* 256 time units (stereo) */
#define ISO_COPY_BUF_SZ  32  /* bytes = 16 samples = 8 time units */
#define ISO_COPY_BUF_SAMPLES ISO_COPY_BUF_SZ/2

int16_t waveform_data[WAVEFORM_SAMPLES] = {0};

#define REBASE(x)        MMIO32((x) + (USB_OTG_FS_BASE))


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

#define MIDI_NOTE_MAX   127
#define MIDI_NOTE_MIN   0
#define MIDI_N_NOTES    128
#define MIDI_CHANNEL_MAX 15
#define MIDI_CHANNEL_MIN 0
#define MIDI_N_CHANNELS 16

#define MAX_VOICES 12

typedef struct {
    uint8_t channel;
    uint8_t key;
    uint8_t vel;

    uint8_t sustained;

    uint16_t cur_sample;
    uint16_t skip_factor;

    /* Add ADSR? */
} note_state_t;

/* Power of two so fixed-point divide optimizes to a shift */
#define FPM 128

#define NOTE_IGNORE_MAX 94
#define NOTE_IGNORE_MIN 21

note_state_t note_states[MAX_VOICES];

uint16_t note_skip_table[MIDI_N_NOTES];

void init_data() {
    /* init all note states */
    memset(note_states, 0, sizeof(note_states));

    /* Sine wave */
    for(int i = 0; i != WAVEFORM_SAMPLES; ++i) {
        waveform_data[i] = 15 * (float)sin(3.141 * 2.0 * (float)i / (float)WAVEFORM_SAMPLES);
    }

    /* Initialize the note skip table */

    /* Compute note frequency */
    /* Compute note period */
    /* Convert note period to per-sample master waveform skip */
    /* Fixed point instead of float skip proportions? */

    for(int n = 0; n != MIDI_N_NOTES; ++n) {
        uint16_t skip = 0;
        if(n > NOTE_IGNORE_MIN && n < NOTE_IGNORE_MAX) {
            float freq = 27.5 * 2.0 * pow(2.0, (n-21.0)/12.0);
            float period = 1.0 / freq;
            float waveform_period = ((float)WAVEFORM_SAMPLES / 8000.0);
            skip = (uint16_t)(FPM * waveform_period / period);
        }
        note_skip_table[n] = skip;
    }
}


void usbaudio_iso_stream_callback(usbd_device *usbd_dev, uint8_t ep)
{
    static int toggle = 0;

    if(toggle++ % 2 == 0) {
        REBASE(OTG_DIEPCTL(ep)) |= OTG_DIEPCTLX_SD0PID;
    } else {
        REBASE(OTG_DIEPCTL(ep)) |= OTG_DIEPCTLX_SD1PID;
    }

    uint16_t out_samples[ISO_COPY_BUF_SAMPLES];
    memset(out_samples, 0, ISO_COPY_BUF_SZ);

    for(int i = 0; i != MAX_VOICES; ++i) {
        if(note_states[i].key > 0) {
            /* For every (time) sample in the note... */
            for(int s = 0; s != ISO_COPY_BUF_SAMPLES/2; ++s) {
                note_states[i].cur_sample =
                    (note_states[i].cur_sample + note_states[i].skip_factor) % (WAVEFORM_SAMPLES*FPM);
                out_samples[s*2] += note_states[i].vel*waveform_data[note_states[i].cur_sample / FPM];
                out_samples[s*2+1] = out_samples[s*2];

            }

            /* Hack to slowly kill note volume */
            if(toggle % 10 == 0) {
                --note_states[i].vel;
                if(note_states[i].vel == 0) note_states[i].key = 0;
            }
        }
    }

    usbd_ep_write_packet(usbd_dev, 0x82, out_samples, ISO_COPY_BUF_SZ);
}

int evict = 0;

void note_on_event(uint8_t channel, uint8_t key, uint8_t vel) {

    /* Ignore drums and invalid notes */
    if(channel == 10 || key < NOTE_IGNORE_MIN || key > NOTE_IGNORE_MAX) {
        return;
    }

    /* Try and find an empty note slot */
    int target = 0;
    for(int i = 0; i != MAX_VOICES; ++i) {
        if(note_states[i].key == 0) {
            target = i;
        }
    }

    /* Otherwise just evict round-robin */
    if(target == 0)
        target = evict++ % MAX_VOICES;

    gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN);

    note_states[target].channel = channel;
    note_states[target].key = key;
    note_states[target].vel = vel;
    note_states[target].sustained = 0;
    note_states[target].cur_sample = 0;
    note_states[target].skip_factor = note_skip_table[key];
}

uint8_t sustain[MIDI_N_CHANNELS] = {0};

void note_off_event(uint8_t channel, uint8_t key, uint8_t vel) {

    gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);

    for(int i = 0; i != MAX_VOICES; ++i) {
        if(note_states[i].channel == channel &&
           note_states[i].key == key) {
            if(sustain[channel]) {
                note_states[i].sustained = 1;
            } else {
                note_states[i].key = 0;
            }
        }
    }
}

#define MIDI_CC_SUSTAIN 0x40 /* Sustain pedal */

void midi_cc_event(uint8_t channel, uint8_t cc_id, uint8_t value) {
    if(cc_id == MIDI_CC_SUSTAIN) {
        if(value >= 64) {
            /* Pedal DOWN */
            sustain[channel] = 1;
        } else {
            /* Pedal UP */
            sustain[channel] = 0;

            for(int i = 0; i != MAX_VOICES; ++i) {
                if(note_states[i].channel == channel) {
                    if(note_states[i].sustained) {
                        note_states[i].key = 0;
                    }
                }
            }
        }
    }
}

void decode_midi_event_packet(midi_usb_event_packet_t p) {

    uint8_t midi_channel = p.midi0 & 0xF;
    uint8_t midi_command = p.midi0 >> 4;

    if(midi_command != p.code_index_number) {
        return;
    }

    switch (midi_command) {
        case MIDI_NOTE_OFF: {
            note_off_event(midi_channel, p.midi1, p.midi2);
            break;
        }
        case MIDI_NOTE_ON: {
            note_on_event(midi_channel, p.midi1, p.midi2);
            break;
        }
        case MIDI_CONTINUOUS: {
            midi_cc_event(midi_channel, p.midi1, p.midi2);
        }
        default: {
            break;
        }
    }
}

static void usbmidi_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

    if(len % 4 == 0) {
        for(int i = 0; i != len; i += 4) {
            decode_midi_event_packet(*(midi_usb_event_packet_t*)(buf+i));
        }
    }

#if 0
	/* This implementation treats any message from the host as a SysEx
	 * identity request. This works well enough providing the host
	 * packs the identify request in a single 8 byte USB message.
	 */
	if (len) {
		while (usbd_ep_write_packet(usbd_dev, 0x81, sysex_identity,
					    sizeof(sysex_identity)) == 0);
	}
#endif

}

static void usbaudio_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_ISOCHRONOUS, ISO_COPY_BUF_SZ, usbaudio_iso_stream_callback);

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64,
			usbmidi_data_rx_cb);

	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);

    /* XXX: This is necessary -> but why? */
    usbd_ep_write_packet(usbd_dev, 0x82, waveform_data, ISO_COPY_BUF_SZ);
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
		//usbmidi_send_note(g_usbd_dev, 1);
	}
}
