/*
 * File: usb_synth.c
 * A basic USB synthesizer. The device will enumerate as a composite MIDI
 * device and microphone device. Sending MIDI notes to it will play tones.
 *
 * 'midi_descriptors.h' from the libopencm3 MIDI example, attribution:
 * Copyright (C) 2014 Daniel Thompson <daniel@redfelineninja.org.uk>
 *
 * All other files:
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

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>
#include <libopencm3/usb/dwc_otg_common.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "usb_descriptors.h"
#include "midi_events.h"

#include "tomu_pins.h"

/* Default AHB (core clock) frequency of Tomu board */
#define AHB_FREQUENCY        14000000

usbd_device *g_usbd_dev = 0;

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[1024];

#define ISO_COPY_BUF_SZ  32  /* bytes = 16 samples = 8 time units */
#define ISO_COPY_BUF_SAMPLES ISO_COPY_BUF_SZ/2

/* HACK: upstream libopencm3 currently does not handle isochronous endpoints
 * correctly. We must program the USB peripheral with an even/odd frame bit,
 * toggling it so that we respond to every iso IN request from the host.
 * If this toggling is not performed, we only get half the bandwidth. */
#define USB_REBASE(x) MMIO32((x) + (USB_OTG_FS_BASE))
#define USB_DIEPCTLX_SD1PID     (1 << 29) /* Odd frames */
#define USB_DIEPCTLX_SD0PID     (1 << 28) /* Even frames */
void toggle_isochronous_frame(uint8_t ep)
{
    static int toggle = 0;
    if (toggle++ % 2 == 0) {
        USB_REBASE(OTG_DIEPCTL(ep)) |= USB_DIEPCTLX_SD0PID;
    } else {
        USB_REBASE(OTG_DIEPCTL(ep)) |= USB_DIEPCTLX_SD1PID;
    }
}

void usbaudio_iso_stream_callback(usbd_device *usbd_dev, uint8_t ep)
{
    toggle_isochronous_frame(ep);

    uint16_t out_samples[ISO_COPY_BUF_SAMPLES];

    core_stream_callback(out_samples, ISO_COPY_BUF_SAMPLES);

    usbd_ep_write_packet(usbd_dev, 0x82, out_samples, ISO_COPY_BUF_SZ);
}

static void usbmidi_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

    /* Check if this USB packet contains one or more MIDI packets */
    if(len % 4 == 0) {
        for(int i = 0; i != len; i += 4) {
            decode_midi_event_packet(*(midi_usb_event_packet_t*)(buf+i));
        }
    }
}

static void usbaudio_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_ISOCHRONOUS,
                  ISO_COPY_BUF_SZ, usbaudio_iso_stream_callback);

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64,
                  usbmidi_data_rx_cb);

    usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);

    /* XXX: This is necessary -> but why? */
    uint8_t junk[ISO_COPY_BUF_SZ] = {0};
    usbd_ep_write_packet(usbd_dev, 0x82, junk, ISO_COPY_BUF_SZ);
}

void led_green_on() {
    gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN);
}

void led_green_off() {
    gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
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

    synth_core_init();

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
    }
}
