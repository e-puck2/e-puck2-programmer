
#ifndef LIBOPENCM3_USB_AUDIO_CLASS_H
#define LIBOPENCM3_USB_AUDIO_CLASS_H

#include <libopencm3/usb/usbd.h>

/* Table A-1: Audio Interface Class Code */
#define USB_CLASS_AUDIO			0x01

/* Table A-2: Audio Interface Subclass Codes */
#define USB_AUDIO_SUBCLASS_UNDEFINED		0x00
#define USB_AUDIO_SUBCLASS_CONTROL		0x01
#define USB_AUDIO_SUBCLASS_AUDIOSTREAMING	0x02
#define USB_AUDIO_SUBCLASS_MIDISTREAMING	0x03

/* Table A-3 : Audio Interface Protocol Codes */
#define USB_AUDIO_PROTOCOL_CODE_UNDIFINED	0x00

/* Table A-4: Audio Class-specific Descriptor Types */
#define USB_AUDIO_DT_CS_UNDEFINED		0x20
#define USB_AUDIO_DT_CS_DEVICE			0x21
#define USB_AUDIO_DT_CS_CONFIGURATION		0x22
#define USB_AUDIO_DT_CS_STRING			0x23
#define USB_AUDIO_DT_CS_INTERFACE		0x24
#define USB_AUDIO_DT_CS_ENDPOINT		0x25

/* Table A-5: Audio Class-Specific AC Interface Descriptor Subtypes */
#define USB_AUDIO_TYPE_AC_DESCRIPTOR_UNDEFINED	0x00
#define USB_AUDIO_TYPE_HEADER			0x01
#define USB_AUDIO_TYPE_INPUT_TERMINAL		0x02
#define USB_AUDIO_TYPE_OUTPUT_TERMINAL		0x03
#define USB_AUDIO_TYPE_MIXER_UNIT		0x04
#define USB_AUDIO_TYPE_SELECTOR_UNIT		0x05
#define USB_AUDIO_TYPE_FEATURE_UNIT		0x06
#define USB_AUDIO_TYPE_PROCESSING_UNIT		0x07
#define USB_AUDIO_TYPE_EXTENSION_UNIT		0x08

#define USB_AUDIO_TYPE_FORMAT 0x02
#define USB_AUDIO_FORMAT_TYPE_I	0x01

/* Table 4-2: Class-Specific AC Interface Header Descriptor (head) */
struct usb_audio_header_descriptor_head {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint16_t bcdADC;
	uint16_t wTotalLength;
	uint8_t binCollection;
	/* ... */
} __attribute__((packed));

/* Table 4-2: Class-Specific AC Interface Header Descriptor (body) */
struct usb_audio_header_descriptor_body {
	/* ... */
	uint8_t baInterfaceNr;
} __attribute__((packed));

struct usb_audio_header_descriptor_in_terminal {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bTerminalID;
	uint16_t wTerminalType;
	uint8_t bAssocTerminal;
	uint8_t bNrChannels;
	uint16_t wChannelConfig;
	uint8_t iChannelNames;
	uint8_t iTerminal;
	/* .. */
} __attribute__((packed));

struct usb_audio_header_descriptor_out_terminal {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bTerminalID;
	uint16_t wTerminalType;
	uint8_t bAssocTerminal;
	uint8_t bSourceID;
	uint8_t iTerminal;
	/* .. */
} __attribute__((packed));

struct usb_audio_header_descriptor_microphone_interface {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bTerminalLink;
	uint8_t bDelay;
	uint16_t wFormatTag;
	/* .. */
} __attribute__((packed));

/** Table B-10: USB Microphone Type I Format Type Descriptor */
struct usb_audio_header_descriptor_microphone_format {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bFormatType;
	uint8_t bNrChannels;
	uint8_t bSubFrameSize;
	uint8_t bBitResolution;
	uint8_t bSamFreqType;
	uint8_t tSamFreqBit0;
	uint8_t tSamFreqBit1;
	uint8_t tSamFreqBit2;
	/* .. */
} __attribute__((packed));


struct usb_audio_specific_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bmAttributes;
	uint8_t bLockDelayUnits;
	uint16_t wLockDelay;
	/* .. */
} __attribute__((packed));

#endif

/**@}*/
