/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2014 Daniel Thompson <daniel@redfelineninja.org.uk>
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

#include <stdlib.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

uint8_t BufferIn(uint32_t byte);
uint8_t BufferOut(uint32_t *pByte);
void clock_setup(void);
void gpio_setup(void);
void key_scan(void);

/*
 * All references in this file come from Universal Serial Bus Device Class
 * Definition for MIDI Devices, release 1.0.
 */

/*
 * Table B-1: MIDI Adapter Device Descriptor
 */

 
static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,    // was 0x0110 in Table B-1 example descriptor */
	.bDeviceClass = 0,   // device defined at interface level */
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x6666,  // Prototype product vendor ID */
	.idProduct = 0x5119, // dd if=/dev/random bs=2 count=1 | hexdump */
	.bcdDevice = 0x0100,
	.iManufacturer = 1,  // index to string desc */
	.iProduct = 2,       // index to string desc */
	.iSerialNumber = 3,  // index to string desc */
	.bNumConfigurations = 1,
};

/*
 * Midi specific endpoint descriptors.
 */
static const struct usb_midi_endpoint_descriptor midi_bulk_endp[] = {{
	/* Table B-12: MIDI Adapter Class-specific Bulk OUT Endpoint
	 * Descriptor
	 */
	.head = {
		.bLength = sizeof(struct usb_midi_endpoint_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
		.bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
		.bNumEmbMIDIJack = 1,
	},
	.jack[0] = {
		.baAssocJackID = 0x01,
	},
}, {
	/* Table B-14: MIDI Adapter Class-specific Bulk IN Endpoint
	 * Descriptor
	 */
	.head = {
		.bLength = sizeof(struct usb_midi_endpoint_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
		.bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
		.bNumEmbMIDIJack = 1,
	},
	.jack[0] = {
		.baAssocJackID = 0x03,
	},
} };

/*
 * Standard endpoint descriptors
 */
static const struct usb_endpoint_descriptor bulk_endp[] = {{
	/* Table B-11: MIDI Adapter Standard Bulk OUT Endpoint Descriptor */
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 0x40,
	.bInterval = 0x00,

	.extra = &midi_bulk_endp[0],
	.extralen = sizeof(midi_bulk_endp[0])
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 0x40,
	.bInterval = 0x00,

	.extra = &midi_bulk_endp[1],
	.extralen = sizeof(midi_bulk_endp[1])
} };

/*
 * Table B-4: MIDI Adapter Class-specific AC Interface Descriptor
 */
static const struct {
	struct usb_audio_header_descriptor_head header_head;
	struct usb_audio_header_descriptor_body header_body;
} __attribute__((packed)) audio_control_functional_descriptors = {
	.header_head = {
		.bLength = sizeof(struct usb_audio_header_descriptor_head) +
		           1 * sizeof(struct usb_audio_header_descriptor_body),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_HEADER,
		.bcdADC = 0x0100,
		.wTotalLength =
			   sizeof(struct usb_audio_header_descriptor_head) +
			   1 * sizeof(struct usb_audio_header_descriptor_body),
		.binCollection = 1,
	},
	.header_body = {
		.baInterfaceNr = 0x01,
	},
};

/*
 * Table B-3: MIDI Adapter Standard AC Interface Descriptor
 */
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

/*
 * Class-specific MIDI streaming interface descriptor
 */
static const struct {
	struct usb_midi_header_descriptor header;
	struct usb_midi_in_jack_descriptor in_embedded;
	struct usb_midi_in_jack_descriptor in_external;
	struct usb_midi_out_jack_descriptor out_embedded;
	struct usb_midi_out_jack_descriptor out_external;
} __attribute__((packed)) midi_streaming_functional_descriptors = {
	/* Table B-6: Midi Adapter Class-specific MS Interface Descriptor */
	.header = {
		.bLength = sizeof(struct usb_midi_header_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MS_HEADER,
		.bcdMSC = 0x0100,
		.wTotalLength = sizeof(midi_streaming_functional_descriptors),
	},
	/* Table B-7: MIDI Adapter MIDI IN Jack Descriptor (Embedded) */
	.in_embedded = {
		.bLength = sizeof(struct usb_midi_in_jack_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
		.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
		.bJackID = 0x01,
		.iJack = 0x00,
	},
	/* Table B-8: MIDI Adapter MIDI IN Jack Descriptor (External) */
	.in_external = {
		.bLength = sizeof(struct usb_midi_in_jack_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
		.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
		.bJackID = 0x02,
		.iJack = 0x00,
	},
	/* Table B-9: MIDI Adapter MIDI OUT Jack Descriptor (Embedded) */
	.out_embedded = {
		.head = {
			.bLength = sizeof(struct usb_midi_out_jack_descriptor),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
			.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
			.bJackID = 0x03,
			.bNrInputPins = 1,
		},
		.source[0] = {
			.baSourceID = 0x02,
			.baSourcePin = 0x01,
		},
		.tail = {
			.iJack = 0x00,
		}
	},
	/* Table B-10: MIDI Adapter MIDI OUT Jack Descriptor (External) */
	.out_external = {
		.head = {
			.bLength = sizeof(struct usb_midi_out_jack_descriptor),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
			.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
			.bJackID = 0x04,
			.bNrInputPins = 1,
		},
		.source[0] = {
			.baSourceID = 0x01,
			.baSourcePin = 0x01,
		},
		.tail = {
			.iJack = 0x00,
		},
	},
};

/*
 * Table B-5: MIDI Adapter Standard MS Interface Descriptor
 */
static const struct usb_interface_descriptor midi_streaming_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_MIDISTREAMING,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = bulk_endp,

	.extra = &midi_streaming_functional_descriptors,
	.extralen = sizeof(midi_streaming_functional_descriptors)
} };

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = audio_control_iface,
}, {
	.num_altsetting = 1,
	.altsetting = midi_streaming_iface,
} };

/*
 * Table B-2: MIDI Adapter Configuration Descriptor
 */
static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0, /* can be anything, it is updated automatically
			      when the usb code prepares the descriptor */
	.bNumInterfaces = 2, /* control and data */
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80, /* bus powered */
	.bMaxPower = 0x32,

	.interface = ifaces,
};


static char usb_serial_number[25]; /* 12 bytes of desig and a \0 */

static const char * usb_strings[] = {
	"libopencm3.org",
	"MIDI demo",
	usb_serial_number
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

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

#define SUCCESS 1
#define FAIL 0
#define BUFFER_SIZE 32 
#define BUFFER_MASK (BUFFER_SIZE-1)

struct Buffer {
	uint32_t data[BUFFER_SIZE];
	uint8_t read;
	uint8_t write;
} buffer = { { }, 0, 0 };

uint8_t BufferIn(uint32_t byte) {
	uint8_t next = ((buffer.write + 1) & BUFFER_MASK);
	if (buffer.read == next)
		return FAIL;
	buffer.data[buffer.write] = byte;
	buffer.write = next;
	return SUCCESS;
}

uint8_t BufferOut(uint32_t *pByte) {
	if (buffer.read == buffer.write)
		return FAIL;
	*pByte = buffer.data[buffer.read];
	buffer.read = (buffer.read + 1) & BUFFER_MASK;
	return SUCCESS;
}

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

	gpio_toggle(GPIOC, GPIO12);
}

static void usbmidi_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64,
			usbmidi_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);
}

static void button_send_event(usbd_device *usbd_dev, int pressed, int note)
{
	char buf[4] = { 0x08, /* USB framing: virtual cable 0, note on */
			0x80, /* MIDI command: note on, channel 1 */
			note,   /* Note 60 (middle C) */
			64,   /* "Normal" velocity */
	};

	buf[0] |= pressed;
	buf[1] |= pressed << 4;

	while (usbd_ep_write_packet(usbd_dev, 0x81, buf, sizeof(buf)) == 0);
}

void clock_setup(void) 
{
	//STM32 set to 120MHz
	rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_120MHZ]);
}

//GPIO setup for output pins (d) and input pins (k)
void gpio_setup(void) 
{
	//Clock enable for GPIOA and GPIOB
	rcc_periph_clock_enable(RCC_GPIOA);
  	rcc_periph_clock_enable(RCC_GPIOC);
  	rcc_periph_clock_enable(RCC_GPIOD);
  	rcc_periph_clock_enable(RCC_GPIOE);

  	//rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_OTGFS);

	/* USB pins */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO9 | GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO9 | GPIO11 | GPIO12);

		/* Button pin */
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO0);

  	//Push-pull outputs with pulldown (d)
  	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN,
  			GPIO0 | GPIO1 | GPIO2 | GPIO3);

  	//Pulldown inputs (k)
  	gpio_mode_setup(GPIOE, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, 
  			GPIO4 | GPIO5 | GPIO6 | GPIO7 | GPIO8 | GPIO9 | GPIO10 | GPIO11);

  	//LEDs
  	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
  			GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

// scan key matrix
// send pressed key note to buffer
void key_scan(void) {

	gpio_set(GPIOE, GPIO0);

	if (!!gpio_get(GPIOE, GPIO4) == 1) BufferIn(41);	
	if (!!gpio_get(GPIOE, GPIO5) == 1) BufferIn(42);	
	if (!!gpio_get(GPIOE, GPIO6) == 1) BufferIn(43);	
	if (!!gpio_get(GPIOE, GPIO7) == 1) BufferIn(44);
	if (!!gpio_get(GPIOE, GPIO8) == 1) BufferIn(44);
	if (!!gpio_get(GPIOE, GPIO9) == 1) BufferIn(46);
	if (!!gpio_get(GPIOE, GPIO10) == 1) BufferIn(47);
	if (!!gpio_get(GPIOE, GPIO11) == 1) BufferIn(48);

	gpio_clear(GPIOE, GPIO0);

	gpio_set(GPIOE, GPIO1);

	if (!!gpio_get(GPIOE, GPIO4) == 1) BufferIn(49);	
	if (!!gpio_get(GPIOE, GPIO5) == 1) BufferIn(50);	
	if (!!gpio_get(GPIOE, GPIO6) == 1) BufferIn(51);	
	if (!!gpio_get(GPIOE, GPIO7) == 1) BufferIn(52);
	if (!!gpio_get(GPIOE, GPIO8) == 1) BufferIn(53);
	if (!!gpio_get(GPIOE, GPIO9) == 1) BufferIn(54);
	if (!!gpio_get(GPIOE, GPIO10) == 1) BufferIn(55);
	if (!!gpio_get(GPIOE, GPIO11) == 1) BufferIn(56);

	gpio_clear(GPIOE, GPIO1);

	gpio_set(GPIOE, GPIO2);

	if (!!gpio_get(GPIOE, GPIO4) == 1) BufferIn(57);	
	if (!!gpio_get(GPIOE, GPIO5) == 1) BufferIn(58);	
	if (!!gpio_get(GPIOE, GPIO6) == 1) BufferIn(58);	
	if (!!gpio_get(GPIOE, GPIO7) == 1) BufferIn(60);
	if (!!gpio_get(GPIOE, GPIO8) == 1) BufferIn(61);
	if (!!gpio_get(GPIOE, GPIO9) == 1) BufferIn(62);
	if (!!gpio_get(GPIOE, GPIO10) == 1) BufferIn(63);
	if (!!gpio_get(GPIOE, GPIO11) == 1) BufferIn(64);

	gpio_clear(GPIOE, GPIO2);

	gpio_set(GPIOE, GPIO3);

	if (!!gpio_get(GPIOE, GPIO4) == 1) BufferIn(65);	
	if (!!gpio_get(GPIOE, GPIO5) == 1) BufferIn(66);	
	if (!!gpio_get(GPIOE, GPIO6) == 1) BufferIn(67);	
	if (!!gpio_get(GPIOE, GPIO7) == 1) BufferIn(68);
	if (!!gpio_get(GPIOE, GPIO8) == 1) BufferIn(69);
	if (!!gpio_get(GPIOE, GPIO9) == 1) BufferIn(70);
	if (!!gpio_get(GPIOE, GPIO10) == 1) BufferIn(71);
	if (!!gpio_get(GPIOE, GPIO11) == 1) BufferIn(72);

	gpio_clear(GPIOE, GPIO3);
} 


int main(void)
{
	
	usbd_device *usbd_dev;
	uint32_t payload;

	clock_setup();
	gpio_setup();

	desig_get_unique_id_as_string(usb_serial_number, sizeof(usb_serial_number));

	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
			usb_strings, 3,
			usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev, usbmidi_set_config);

	while (1) {

		usbd_poll(usbd_dev);

		key_scan();

		if (BufferOut(&payload)) {
			gpio_set(GPIOD, GPIO15);

		} else gpio_clear(GPIOD, GPIO15);
		
		static uint32_t button_state = 0;

		uint32_t old_button_state = button_state;
		button_state = (button_state << 1) | (!!gpio_get(GPIOD, GPIO15) & 1);
		if ((0 == button_state) != (0 == old_button_state)) {
			button_send_event(usbd_dev, !!button_state, payload);
		}
	


	}
}
