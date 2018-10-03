/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @modified by        Eliot Ferragni
 * @last modification  09.07.2018
 */

#include "hal.h"
#include "main.h"
#include "usbcfg.h"
#include "communications.h"

/*
 * Virtual serial ports over USB.
 */
SerialUSBDriver SDU1;
SerialUSBDriver SDU2;

#define USB_DEVICE_VID                  0x1D50  /* You MUST change this.*/
#define USB_DEVICE_PID                  0x6018  /* You MUST change this.*/

/*
 * Endpoints.
 */
#define USB_DATA_AVAILABLE_EP_A         1
#define USB_DATA_REQUEST_EP_A           1
#define USB_INTERRUPT_REQUEST_EP_A      2
#define USB_DATA_AVAILABLE_EP_B         3
#define USB_DATA_REQUEST_EP_B           3
#define USB_INTERRUPT_REQUEST_EP_B      4

#define USB_INTERRUPT_REQUEST_SIZE      0x10

/*
 * Interfaces
 */
#define USB_NUM_INTERFACES              4
#define USB_CDC_CIF_NUM0                0
#define USB_CDC_DIF_NUM0                1
#define USB_CDC_CIF_NUM1                2
#define USB_CDC_DIF_NUM1                3

#define USB_NUM_STRINGS                 6
#define USB_INDEX_STRING_SERIAL_A       4
#define USB_INDEX_STRING_SERIAL_B       5

typedef struct{
  uint8_t cdc_cif_num0_dtr;
  uint8_t cdc_cif_num0_rts;
  uint8_t cdc_cif_num1_dtr;
  uint8_t cdc_cif_num1_rts;
}control_line_states_t;

static control_line_states_t control_line_states = {
  .cdc_cif_num0_dtr = 0,
  .cdc_cif_num0_rts = 0,
  .cdc_cif_num1_dtr = 0,
  .cdc_cif_num1_rts = 0
};

/*
 * Current Line Coding.
 */
static cdc_linecoding_t linecoding = {
  {0x00, 0x96, 0x00, 0x00},             /* 38400.                           */
  LC_STOP_1, LC_PARITY_NONE, 8
};

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[] = {
  USB_DESC_DEVICE(
    0x0200,                                 /* bcdUSB (1.1).                */
    0xEF,                                   /* bDeviceClass (misc).         */
    0x02,                                   /* bDeviceSubClass (common).    */
    0x01,                                   /* bDeviceProtocol (IAD).       */
    USB_DATA_SIZE,                          /* bMaxPacketSize.              */
    USB_DEVICE_VID,                         /* idVendor.                    */
    USB_DEVICE_PID,                         /* idProduct.                   */
    0x0200,                                 /* bcdDevice.                   */
    1,                                      /* iManufacturer.               */
    2,                                      /* iProduct.                    */
    3,                                      /* iSerialNumber.               */
    1)                                      /* bNumConfigurations.          */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

#define CDC_IF_DESC_SET_SIZE                                                \
  (USB_DESC_INTERFACE_SIZE + 5 + 5 + 4 + 5 + USB_DESC_ENDPOINT_SIZE +       \
   USB_DESC_INTERFACE_SIZE + (USB_DESC_ENDPOINT_SIZE * 2))

#define CDC_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp, numInd)     \
  /* Interface Descriptor.*/                                                \
  USB_DESC_INTERFACE(                                                       \
    comIfNum,                               /* bInterfaceNumber.        */  \
    0x00,                                   /* bAlternateSetting.       */  \
    0x01,                                   /* bNumEndpoints.           */  \
    CDC_COMMUNICATION_INTERFACE_CLASS,      /* bInterfaceClass.         */  \
    CDC_ABSTRACT_CONTROL_MODEL,             /* bInterfaceSubClass.      */  \
    0x01,                                   /* bInterfaceProtocol (AT
                                               commands, CDC section
                                               4.4).                    */  \
    numInd),                                     /* iInterface.              */  \
  /* Header Functional Descriptor (CDC section 5.2.3).*/                    \
  USB_DESC_BYTE     (5),                    /* bLength.                 */  \
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType.         */  \
  USB_DESC_BYTE     (CDC_HEADER),           /* bDescriptorSubtype.      */  \
  USB_DESC_BCD      (0x0110),               /* bcdCDC.                  */  \
  /* Call Management Functional Descriptor.*/                               \
  USB_DESC_BYTE     (5),                    /* bFunctionLength.         */  \
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType.         */  \
  USB_DESC_BYTE     (CDC_CALL_MANAGEMENT),  /* bDescriptorSubtype.      */  \
  USB_DESC_BYTE     (0x00),    /*******/    /* bmCapabilities.          */  \
  USB_DESC_BYTE     (datIfNum),             /* bDataInterface.          */  \
  /* Abstract Control Management Functional Descriptor.*/                   \
  USB_DESC_BYTE     (4),                    /* bFunctionLength.         */  \
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType.         */  \
  USB_DESC_BYTE     (CDC_ABSTRACT_CONTROL_MANAGEMENT),                      \
  USB_DESC_BYTE     (0x02),                 /* bmCapabilities.          */  \
  /* Union Functional Descriptor.*/                                         \
  USB_DESC_BYTE     (5),                    /* bFunctionLength.         */  \
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType.         */  \
  USB_DESC_BYTE     (CDC_UNION),            /* bDescriptorSubtype.      */  \
  USB_DESC_BYTE     (comIfNum),             /* bMasterInterface.        */  \
  USB_DESC_BYTE     (datIfNum),             /* bSlaveInterface.         */  \
  /* Endpoint, Interrupt IN.*/                                              \
  USB_DESC_ENDPOINT (                                                       \
    comInEp,                                                                \
    USB_EP_MODE_TYPE_INTR,                  /* bmAttributes.            */  \
    USB_INTERRUPT_REQUEST_SIZE,             /* wMaxPacketSize.          */  \
    255),                                  /* bInterval.               */  \
                                                                            \
  /* CDC Data Interface Descriptor.*/                                       \
  USB_DESC_INTERFACE(                                                       \
    datIfNum,                               /* bInterfaceNumber.        */  \
    0x00,                                   /* bAlternateSetting.       */  \
    0x02,                                   /* bNumEndpoints.           */  \
    CDC_DATA_INTERFACE_CLASS,               /* bInterfaceClass.         */  \
    0x00,                                   /* bInterfaceSubClass (CDC
                                               section 4.6).            */  \
    0x00,                                   /* bInterfaceProtocol (CDC
                                               section 4.7).            */  \
    0x00),                                  /* iInterface.              */  \
  /* Endpoint, Bulk OUT.*/                                                  \
  USB_DESC_ENDPOINT(                                                        \
    datOutEp,                               /* bEndpointAddress.        */  \
    USB_EP_MODE_TYPE_BULK,                  /* bmAttributes.            */  \
    USB_DATA_SIZE,                          /* wMaxPacketSize.          */  \
    0x01),                                  /* bInterval.               */  \
  /* Endpoint, Bulk IN.*/                                                   \
  USB_DESC_ENDPOINT(                                                        \
    datInEp,                                /* bEndpointAddress.        */  \
    USB_EP_MODE_TYPE_BULK,                  /* bmAttributes.            */  \
    USB_DATA_SIZE,                          /* wMaxPacketSize.          */  \
    0x01)                                   /* bInterval.               */

#define IAD_CDC_IF_DESC_SET_SIZE                                            \
  (USB_DESC_INTERFACE_ASSOCIATION_SIZE + CDC_IF_DESC_SET_SIZE)

#define IAD_CDC_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp, numInd) \
  /* Interface Association Descriptor.*/                                    \
  USB_DESC_INTERFACE_ASSOCIATION(                                           \
    comIfNum,                               /* bFirstInterface.         */  \
    2,                                      /* bInterfaceCount.         */  \
    CDC_COMMUNICATION_INTERFACE_CLASS,      /* bFunctionClass.          */  \
    CDC_ABSTRACT_CONTROL_MODEL,             /* bFunctionSubClass.       */  \
    1,                                      /* bFunctionProcotol.       */  \
    0                                       /* iInterface.              */  \
  ),                                                                        \
  /* CDC Interface descriptor set */                                        \
  CDC_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp, numInd)

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(
    USB_DESC_CONFIGURATION_SIZE +
    (IAD_CDC_IF_DESC_SET_SIZE * 2),         /* wTotalLength.                */
    USB_NUM_INTERFACES,                     /* bNumInterfaces.              */
    0x01,                                   /* bConfigurationValue.         */
    0,                                      /* iConfiguration.              */
    0x80,                                   /* bmAttributes. */
    50                                      /* bMaxPower (100mA).           */
  ),
  IAD_CDC_IF_DESC_SET(
    USB_CDC_CIF_NUM0,
    USB_CDC_DIF_NUM0,
    USB_ENDPOINT_IN(USB_INTERRUPT_REQUEST_EP_A),
    USB_ENDPOINT_OUT(USB_DATA_AVAILABLE_EP_A),
    USB_ENDPOINT_IN(USB_DATA_REQUEST_EP_A),
    USB_INDEX_STRING_SERIAL_A
  ),
  IAD_CDC_IF_DESC_SET(
    USB_CDC_CIF_NUM1,
    USB_CDC_DIF_NUM1,
    USB_ENDPOINT_IN(USB_INTERRUPT_REQUEST_EP_B),
    USB_ENDPOINT_OUT(USB_DATA_AVAILABLE_EP_B),
    USB_ENDPOINT_IN(USB_DATA_REQUEST_EP_B),
    USB_INDEX_STRING_SERIAL_B
  ),
};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[] = {
  USB_DESC_BYTE(4),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t vcom_string1[] = {
  USB_DESC_BYTE(52),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'B', 0, 'l', 0, 'a', 0, 'c', 0, 'k', 0, ' ', 0, 'S', 0, 'p', 0,
  'h', 0, 'e', 0, 'r', 0, 'e', 0, ' ', 0, 'T', 0, 'e', 0, 'c', 0,
  'h', 0, 'n', 0, 'o', 0, 'l', 0, 'o', 0, 'g', 0, 'i', 0, 'e', 0,
  's', 0
};

/*
 * Device Description string.
 */
#define SIZE_VCOM_STRING2   (2 * sizeof(BOARD_IDENT) + 2)
static uint8_t vcom_string2[SIZE_VCOM_STRING2] = {
  USB_DESC_BYTE(SIZE_VCOM_STRING2),     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
};

static void constructVcomString2(void){

  //The string is composed of each letter followed by a 0.
  for(uint32_t i = 0 ; i < sizeof(BOARD_IDENT) ; i++){
    //we fill the string only from the 3rd byte
    vcom_string2[2 + 2 * i] = BOARD_IDENT[i]; 
    vcom_string2[3 + 2 * i] = 0;
  }
}

/*
 * Serial Number string.
 */
static const uint8_t vcom_string3[] = {
  USB_DESC_BYTE(12),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'E', 0, 'P', 0, 'U', 0, 'C', 0, 'K', 0
};

/*
 * Name Serial A.
 */
static const uint8_t vcom_string4[] = {
  USB_DESC_BYTE(38),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'e', 0, '-', 0, 'p', 0, 'u', 0, 'c', 0, 'k', 0, '2', 0, ' ', 0,
  'G', 0, 'D', 0, 'B', 0, ' ', 0, 'S', 0, 'e', 0, 'r', 0, 'v', 0,
  'e', 0, 'r', 0
};

/*
 * Name Serial B.
 */
static const uint8_t vcom_string5[] = {
  USB_DESC_BYTE(46),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'e', 0, '-', 0, 'p', 0, 'u', 0, 'c', 0, 'k', 0, '2', 0, ' ', 0,
  'S', 0, 'e', 0, 'r', 0, 'i', 0, 'a', 0, 'l', 0, ' ', 0, 'M', 0,
  'o', 0, 'n', 0, 'i', 0, 't', 0, 'o', 0, 'r', 0
};

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3},
  {sizeof vcom_string4, vcom_string4},
  {sizeof vcom_string5, vcom_string5}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {

  (void)usbp;
  (void)lang;
  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < USB_NUM_STRINGS)
      return &vcom_strings[dindex];
  }
  return NULL;
}

/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1instate;

/**
 * @brief   EP1 initialization structure (IN only).
 */
static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  sduInterruptTransmitted,
  NULL,
  USB_INTERRUPT_REQUEST_SIZE,
  0x0000,
  &ep1instate,
  NULL,
  1,
  NULL
};

/**
 * @brief   IN EP2 state.
 */
static USBInEndpointState ep2instate;

/**
 * @brief   OUT EP2 state.
 */
static USBOutEndpointState ep2outstate;

/**
 * @brief   EP2 initialization structure (both IN and OUT).
 */
static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  sduDataTransmitted,
  sduDataReceived,
  USB_DATA_SIZE,
  USB_DATA_SIZE,
  &ep2instate,
  &ep2outstate,
  1,
  NULL
};

/**
 * @brief   IN EP3 state.
 */
static USBInEndpointState ep3instate;

/**
 * @brief   EP3 initialization structure (IN only).
 */
static const USBEndpointConfig ep3config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  sduInterruptTransmitted,
  NULL,
  USB_INTERRUPT_REQUEST_SIZE,
  0x0000,
  &ep3instate,
  NULL,
  1,
  NULL
};

/**
 * @brief   IN EP4 state.
 */
static USBInEndpointState ep4instate;

/**
 * @brief   OUT EP4 state.
 */
static USBOutEndpointState ep4outstate;

/**
 * @brief   EP4 initialization structure (both IN and OUT).
 */
static const USBEndpointConfig ep4config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  sduDataTransmitted,
  sduDataReceived,
  USB_DATA_SIZE,
  USB_DATA_SIZE,
  &ep4instate,
  &ep4outstate,
  1,
  NULL
};

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {
  extern SerialUSBDriver SDU1;
  extern SerialUSBDriver SDU2;

  switch (event) {
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    chSysLockFromISR();

    if (usbp->state == USB_ACTIVE) {
      /* Enables the endpoints specified into the configuration.
         Note, this callback is invoked from an ISR so I-Class functions
         must be used.*/
      usbInitEndpointI(usbp, USB_INTERRUPT_REQUEST_EP_A, &ep1config);
      usbInitEndpointI(usbp, USB_DATA_REQUEST_EP_A, &ep2config);
      usbInitEndpointI(usbp, USB_INTERRUPT_REQUEST_EP_B, &ep3config);
      usbInitEndpointI(usbp, USB_DATA_REQUEST_EP_B, &ep4config);

      /* Resetting the state of the CDC subsystem.*/
      sduConfigureHookI(&SDU1);
      sduConfigureHookI(&SDU2);
    }
    else if (usbp->state == USB_SELECTED) {
      usbDisableEndpointsI(usbp);
    }

    chSysUnlockFromISR();
    return;
  case USB_EVENT_RESET:
    /* Falls into.*/
  case USB_EVENT_UNCONFIGURED:
    /* Falls into.*/
  case USB_EVENT_SUSPEND:
    chSysLockFromISR();

    /* Disconnection event on suspend.*/
    sduSuspendHookI(&SDU1);
    sduSuspendHookI(&SDU2);

    chSysUnlockFromISR();
    return;
  case USB_EVENT_WAKEUP:
    chSysLockFromISR();

    /* Disconnection event on suspend.*/
    sduWakeupHookI(&SDU1);
    sduWakeupHookI(&SDU2);

    chSysUnlockFromISR();
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}

/*
 * Handling messages not implemented in the default handler nor in the
 * SerialUSB handler.
 */
static bool requests_hook(USBDriver *usbp) {

  if (((usbp->setup[0] & USB_RTYPE_RECIPIENT_MASK) == USB_RTYPE_RECIPIENT_INTERFACE) &&
      (usbp->setup[1] == USB_REQ_SET_INTERFACE)) {
    usbSetupTransfer(usbp, NULL, 0, NULL);
    return true;
  }

  if ((usbp->setup[0] & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS) {
    switch (usbp->setup[1]) {
    case CDC_GET_LINE_CODING:
      usbSetupTransfer(usbp, (uint8_t *)&linecoding, sizeof(linecoding), NULL);
      return true;
    case CDC_SET_LINE_CODING:
      usbSetupTransfer(usbp, (uint8_t *)&linecoding, sizeof(linecoding), NULL);
      return true;
    case CDC_SET_CONTROL_LINE_STATE:
        switch(usbp->setup[4]){
          case USB_CDC_CIF_NUM0:
            control_line_states.cdc_cif_num0_dtr = (usbp->setup[2] & 1) ? TRUE : FALSE;
            control_line_states.cdc_cif_num0_rts = (usbp->setup[2] & 2) ? TRUE : FALSE;
            return TRUE;
          case USB_CDC_CIF_NUM1:
            control_line_states.cdc_cif_num1_dtr = (usbp->setup[2] & 1) ? TRUE : FALSE;
            control_line_states.cdc_cif_num1_rts = (usbp->setup[2] & 2) ? TRUE : FALSE;
            if(communicationGetActiveMode() == UART_ESP_PASSTHROUGH){
              // uint8_t enable = ( ? PAL_HIGH : PAL_LOW;
              // uint8_t gpio0 =  (control_line_states.cdc_cif_num1_rts || !control_line_states.cdc_cif_num1_dtr) ? PAL_HIGH : PAL_LOW;
              gpio_set_val(GPIOC, GPIOC_ESP32_EN, !control_line_states.cdc_cif_num1_rts || control_line_states.cdc_cif_num1_dtr);
              gpio_set_val(GPIOB, GPIOB_ESP_GPIO0, control_line_states.cdc_cif_num1_rts || !control_line_states.cdc_cif_num1_dtr);
              // palWriteLine(LINE_ESP32_EN, enable);
              // palWriteLine(LINE_ESP_GPIO0, gpio0);
            }
            return TRUE;
        }
    default:
      return false;
    }
  }
  //return sduRequestsHook(usbp);
  return false;
}

/*
 * Handles the USB driver global events.
 */
static void sof_handler(USBDriver *usbp) {

  (void)usbp;

  osalSysLockFromISR();
  sduSOFHookI(&SDU1);
  sduSOFHookI(&SDU2);
  osalSysUnlockFromISR();
}

/*
 * USB driver configuration.
 */
const USBConfig usbcfg = {
  usb_event,
  get_descriptor,
  requests_hook,
  sof_handler
};

/*
 * Serial over USB driver configuration 1.
 */
const SerialUSBConfig serusbcfg1 = {
  &USBD1,
  USB_DATA_REQUEST_EP_A,
  USB_DATA_AVAILABLE_EP_A,
  USB_INTERRUPT_REQUEST_EP_A
};

/*
 * Serial over USB driver configuration 2.
 */
const SerialUSBConfig serusbcfg2 = {
  &USBD1,
  USB_DATA_REQUEST_EP_B,
  USB_DATA_AVAILABLE_EP_B,
  USB_INTERRUPT_REQUEST_EP_B
};

void usbSerialStart(void){

  //creates the vcom string 2 dinnamicaly with BOARD_INDENT define in platform.h
  constructVcomString2();
  /*
   * Initializes two serial-over-USB CDC drivers.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg1);
  sduObjectInit(&SDU2);
  sduStart(&SDU2, &serusbcfg2);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg1.usbp);
  chThdSleepMilliseconds(100);
  usbStart(serusbcfg1.usbp, &usbcfg);
  usbConnectBus(serusbcfg1.usbp);
}

uint8_t isUSBConfigured(void){
  return (SDU1.config->usbp->state == USB_ACTIVE) ? 1 : 0;
}

uint8_t getControlLineState(interface_name_t interface, control_line_t rts_dtr){
  if(interface == GDB_INTERFACE){
    if(rts_dtr == CONTROL_LINE_RTS){
      return control_line_states.cdc_cif_num0_rts;
    }else if(rts_dtr == CONTROL_LINE_DTR){
      return control_line_states.cdc_cif_num0_dtr;
    }
  }else if(interface == SERIAL_INTERFACE){
    if(rts_dtr == CONTROL_LINE_RTS){
      return control_line_states.cdc_cif_num1_rts;
    }else if(rts_dtr == CONTROL_LINE_DTR){
      return control_line_states.cdc_cif_num1_dtr;
    }
  }
  return 0;
}
