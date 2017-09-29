
#include "audio_class.h"

/** Interface Association Audio Class */
static const struct usb_iface_assoc_descriptor audio_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bFirstInterface = AUDIO_CONTROL_IFACE_NUM,
	.bInterfaceCount = 2,
	.bFunctionClass = USB_CLASS_AUDIO,
	.bFunctionSubClass = USB_AUDIO_SUBCLASS_CONTROL,
	.bFunctionProtocol = USB_AUDIO_PROTOCOL_CODE_UNDIFINED,
	.iFunction = 0x00,
};

/** Table B-4: USB Microphone Class-specific AC Interface Descriptor (control)*/
static const struct {
	struct usb_audio_header_descriptor_head header_head;
	struct usb_audio_header_descriptor_body header_body;
	struct usb_audio_header_descriptor_in_terminal header_in_terminal;
	struct usb_audio_header_descriptor_out_terminal header_out_terminal;
} __attribute__((packed)) audio_control_interface_descriptors = {
	.header_head = {
		.bLength = sizeof(struct usb_audio_header_descriptor_head) +
		           1 * sizeof(struct usb_audio_header_descriptor_body),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_HEADER,
		.bcdADC = 0x0100, //Revision of calss specification 1.0
		// Total number of bytes returned for the class-specific AudioControl interface descriptor.
	    // Includes the combined length of this descriptor header and all Unit and Terminal descriptors.    
		.wTotalLength = sizeof(struct usb_audio_header_descriptor_head) +
		           1 * sizeof(struct usb_audio_header_descriptor_body) +
		           1 * sizeof(struct usb_audio_header_descriptor_in_terminal) +
		           1 * sizeof(struct usb_audio_header_descriptor_out_terminal),
		.binCollection = 1,	//number of streaming interfaces
	},
	.header_body = {
		.baInterfaceNr = 0x01, //Audio steaming interface 1 belongs to this Audio control interface
	},
	/** Table B-5: USB Microphone Input Terminal Descriptor */
	.header_in_terminal = {
		.bLength = sizeof(struct usb_audio_header_descriptor_in_terminal),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_INPUT_TERMINAL,
		.bTerminalID = 0x01, //ID of this terminal
		.wTerminalType = 0x0201, //terminal is a microphone
		.bAssocTerminal = 0x00, //no association
		.bNrChannels = 0x01, //one microphone
		.wChannelConfig = 0x0000, //mono sets no position bits
		.iChannelNames = 0x00, //unused
		.iTerminal = 0x00, //unused
	},
	/** Table B-6: USB Microphone Output Terminal Descriptor */
	.header_out_terminal = {
		.bLength = sizeof(struct usb_audio_header_descriptor_out_terminal),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_OUTPUT_TERMINAL,
		.bTerminalID = 0x02, //ID of this terminal
		.wTerminalType = 0x0101, //USB streaming
		.bAssocTerminal = 0x00, //unused
		.bSourceID = 0x01, //From input terminal
		.iTerminal = 0x00, //unused
	}
};

/** Table B-3: USB Microphone Standard AC Interface Descriptor */
static const struct usb_interface_descriptor audio_control_iface[] = {{
	.bLength = 0x09,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = AUDIO_CONTROL_IFACE_NUM,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_CONTROL,
	.bInterfaceProtocol = USB_AUDIO_PROTOCOL_CODE_UNDIFINED, //unused
	.iInterface = 0, //unused

	.extra = &audio_control_interface_descriptors,
	.extralen = sizeof(audio_control_interface_descriptors)
}};

static const struct {
	struct usb_audio_specific_endpoint_descriptor extra_endpoint_streaming;
} __attribute__((packed)) audio_streaming_extra_endpoint_streaming = {
	/** Table B-12: USB Microphone Class-specific Isoc. Audio Data Endpoint Descriptor */
	.extra_endpoint_streaming = {
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
		.bDescriptorSubtype = USB_AUDIO_TYPE_HEADER,
		.bmAttributes = 0x00, //No sampling frequency control, no pitch control, no packet padding.
		.bLockDelayUnits = 0x00, //unused
		.wLockDelay = 0x0000, //unused
	}
};

/** Table B-11: USB Microphone Standard Endpoint Descriptor */
static const struct usb_endpoint_descriptor audio_streaming_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,// A valider devrait être 9 mais les deux champs sont unused 
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x85,
	.bmAttributes = USB_ENDPOINT_ATTR_ISOCHRONOUS,
	.wMaxPacketSize = 0x0010, //16 bytes per packet
	.bInterval = 0x01, //one paquet per frame

	.extra = &audio_streaming_extra_endpoint_streaming,
	.extralen = sizeof(audio_streaming_extra_endpoint_streaming),
}};

/** Class-Specific AC Interface Descriptor (Control) */
static const struct {
	struct usb_audio_header_descriptor_microphone_interface header_mic_iface;
	struct usb_audio_header_descriptor_microphone_format header_mic_format;
} __attribute__((packed)) audio_streaming_interface_descriptors = {
	/** Table B-9: USB Microphone Class-specific AS General Interface Descriptor */
	.header_mic_iface = {
		.bLength = sizeof(struct usb_audio_header_descriptor_microphone_interface),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_HEADER,
		.bTerminalLink = 0x02, //Unit ID of the Output Terminal
		.bDelay = 0x01, //interface delay
		.wFormatTag = 0x0001 // PCM format
	},
	/** Table B-10: USB Microphone Type I Format Type Descriptor */
	.header_mic_format = {
		.bLength = sizeof(struct usb_audio_header_descriptor_microphone_format),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_FORMAT,
		.bFormatType = USB_AUDIO_FORMAT_TYPE_I, 
		.bNrChannels = 0x01, //one channel
		.bSubFrameSize = 0x02, //two bytes per audio subframe
		.bBitResolution = 0x10, //16bits per sample
		.bSamFreqType = 0x01, //one frequency supported
		.tSamFreqBit0 = (44100 & 0xFF),	//44,1 kHz
		.tSamFreqBit1 = ((44100 >> 8 ) & 0xFF),
		.tSamFreqBit2 = ((44100 >> 16 ) & 0xFF),
	}
};

/** Table B-7: USB Microphone Standard AS Interface Descriptor */
static const struct usb_interface_descriptor audio_streaming_iface[] = {{
	//Alternate setting 0 with no bandwidth
	.bLength = 0x09,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = AUDIO_STREAM_IFACE_NUM,
	.bAlternateSetting = 0, //index of this alternate setting
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = USB_AUDIO_PROTOCOL_CODE_UNDIFINED, //unsused
	.iInterface = 0, //unsused
}, {
	//alternate setting 1 (operational)
	.bLength = 0x09,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = AUDIO_STREAM_IFACE_NUM,
	.bAlternateSetting = 1, //index of this alternate setting
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = USB_AUDIO_PROTOCOL_CODE_UNDIFINED, //unsused
	.iInterface = 0, //unsused

	.endpoint = audio_streaming_endp,

	.extra = &audio_streaming_interface_descriptors,
	.extralen = sizeof(audio_streaming_interface_descriptors)
}};