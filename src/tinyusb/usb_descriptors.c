/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "tusb.h"
#include "class/audio/audio.h"

enum
{
    VENDOR_REQUEST_WEBUSB = 1,
    VENDOR_REQUEST_MICROSOFT = 2
};

// UAC2 Entity IDs
enum {
    UAC2_ENTITY_CLOCK = 1,
    UAC2_ENTITY_INPUT_TERMINAL = 2,
    UAC2_ENTITY_OUTPUT_TERMINAL = 3,
};

// Required by audio_device.c - length of audio function descriptors
// IAD (8) + AC Interface (9) + AC Header (9) + Clock (8) + Input Terminal (17) + Output Terminal (12)
// + AS Alt0 (9) + AS Alt1 (9) + AS General (16) + Format (6) + EP (7) + CS EP (8)
const uint16_t tud_audio_desc_lengths[] = { 8 + 9 + 9 + 8 + 17 + 12 + 9 + 9 + 16 + 6 + 7 + 8 };




/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]       MIDI | HID | MSC | CDC          [LSB]
 */
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                           _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4) )
extern uint32_t serialno;
//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0210, // 2.1 for webusb
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0xCafe,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}


//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum
{
  ITF_NUM_AUDIO_CONTROL = 0,
  ITF_NUM_AUDIO_STREAMING,
  ITF_NUM_MIDI,
  ITF_NUM_MIDI_STREAMING,
  ITF_NUM_VENDOR,
  ITF_NUM_TOTAL
};

// Endpoint numbers
#define EPNUM_AUDIO_IN    0x01
#define EPNUM_AUDIO_FB    0x04  // Feedback endpoint for clock sync
#define EPNUM_MIDI        0x02
#define EPNUM_VENDOR      0x03

// UAC2 descriptor lengths
#define UAC2_AC_DESC_LEN  (9 + 8 + 17 + 12)  // AC header + clock + input terminal + output terminal
#define UAC2_AS_DESC_LEN  (9 + 9 + 16 + 6 + 7 + 8)  // AS interface alt0 + alt1 + general + format + endpoint + cs_endpoint

// IAD (8) + AC Interface (9) + AC descriptors + AS descriptors
#define UAC2_TOTAL_LEN    (8 + 9 + UAC2_AC_DESC_LEN + UAC2_AS_DESC_LEN)

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + \
                           UAC2_TOTAL_LEN + \
                           TUD_MIDI_DESC_LEN + \
                           TUD_VENDOR_DESC_LEN)

uint8_t const desc_fs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 150),

  //------------- Interface Association Descriptor (IAD) for Audio -------------//
  // bLength, bDescriptorType, bFirstInterface, bInterfaceCount, bFunctionClass, bFunctionSubClass, bFunctionProtocol, iFunction
  8, TUSB_DESC_INTERFACE_ASSOCIATION, ITF_NUM_AUDIO_CONTROL, 2, TUSB_CLASS_AUDIO, AUDIO_FUNCTION_SUBCLASS_UNDEFINED, AUDIO_FUNC_PROTOCOL_CODE_V2, 0,

  //------------- Audio Control Interface -------------//
  // Standard AC Interface Descriptor (bInterfaceNumber, bAlternateSetting, bNumEndpoints, bInterfaceSubClass, bInterfaceProtocol, iInterface)
  9, TUSB_DESC_INTERFACE, ITF_NUM_AUDIO_CONTROL, 0, 0, TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_CONTROL, AUDIO_INT_PROTOCOL_CODE_V2, 0,

  // Class-Specific AC Interface Header Descriptor (UAC2)
  // bLength, bDescriptorType, bDescriptorSubtype, bcdADC, bCategory, wTotalLength, bmControls
  9, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_HEADER, U16_TO_U8S_LE(0x0200), AUDIO_FUNC_PRO_AUDIO, U16_TO_U8S_LE(UAC2_AC_DESC_LEN), 0,

  // Clock Source Descriptor (internal, fixed 48kHz - native Plinky rate)
  // bLength, bDescriptorType, bDescriptorSubtype, bClockID, bmAttributes, bmControls, bAssocTerminal, iClockSource
  8, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_CLOCK_SOURCE, UAC2_ENTITY_CLOCK, 0x01, 0x01, 0, 0,

  // Input Terminal Descriptor (from synth)
  // bLength, bDescriptorType, bDescriptorSubtype, bTerminalID, wTerminalType, bAssocTerminal, bCSourceID, bNrChannels, bmChannelConfig, iChannelNames, bmControls, iTerminal
  17, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_INPUT_TERMINAL, UAC2_ENTITY_INPUT_TERMINAL, U16_TO_U8S_LE(AUDIO_TERM_TYPE_IN_GENERIC_MIC), 0, UAC2_ENTITY_CLOCK, 2, U32_TO_U8S_LE(AUDIO_CHANNEL_CONFIG_FRONT_LEFT | AUDIO_CHANNEL_CONFIG_FRONT_RIGHT), 0, U16_TO_U8S_LE(0), 0,

  // Output Terminal Descriptor (to USB host)
  // bLength, bDescriptorType, bDescriptorSubtype, bTerminalID, wTerminalType, bAssocTerminal, bSourceID, bCSourceID, bmControls, iTerminal
  12, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL, UAC2_ENTITY_OUTPUT_TERMINAL, U16_TO_U8S_LE(AUDIO_TERM_TYPE_USB_STREAMING), 0, UAC2_ENTITY_INPUT_TERMINAL, UAC2_ENTITY_CLOCK, U16_TO_U8S_LE(0), 0,

  //------------- Audio Streaming Interface (Alt 0: zero-bandwidth) -------------//
  9, TUSB_DESC_INTERFACE, ITF_NUM_AUDIO_STREAMING, 0, 0, TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 0,

  //------------- Audio Streaming Interface (Alt 1: active) -------------//
  // bNumEndpoints = 1: data endpoint only (adaptive mode - syncs to USB timing)
  9, TUSB_DESC_INTERFACE, ITF_NUM_AUDIO_STREAMING, 1, 1, TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 0,

  // Class-Specific AS Interface Descriptor
  // bLength, bDescriptorType, bDescriptorSubtype, bTerminalLink, bmControls, bFormatType, bmFormats, bNrChannels, bmChannelConfig, iChannelNames
  16, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_AS_GENERAL, UAC2_ENTITY_OUTPUT_TERMINAL, 0, AUDIO_FORMAT_TYPE_I, U32_TO_U8S_LE(AUDIO_DATA_FORMAT_TYPE_I_PCM), 2, U32_TO_U8S_LE(AUDIO_CHANNEL_CONFIG_FRONT_LEFT | AUDIO_CHANNEL_CONFIG_FRONT_RIGHT), 0,

  // Type I Format Type Descriptor
  // bLength, bDescriptorType, bDescriptorSubtype, bFormatType, bSubslotSize, bBitResolution
  6, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_FORMAT_TYPE, AUDIO_FORMAT_TYPE_I, 2, 16,

  // Isochronous Audio Data Endpoint Descriptor (48kHz = 48 samples/frame * 2ch * 2bytes = 192, +headroom)
  // bLength, bDescriptorType, bEndpointAddress, bmAttributes, wMaxPacketSize, bInterval
  // bmAttributes: 0x09 = isochronous (0x01) + adaptive sync (0x08) - device adapts to USB frame timing
  7, TUSB_DESC_ENDPOINT, 0x80 | EPNUM_AUDIO_IN, TUSB_XFER_ISOCHRONOUS | 0x08, U16_TO_U8S_LE(196), 1,

  // Class-Specific AS Isochronous Audio Data Endpoint Descriptor
  // bLength, bDescriptorType, bDescriptorSubtype, bmAttributes, bmControls, bLockDelayUnits, wLockDelay
  8, TUSB_DESC_CS_ENDPOINT, AUDIO_CS_EP_SUBTYPE_GENERAL, 0, 0, 0, U16_TO_U8S_LE(0),

  //------------- MIDI -------------//
  // Interface number, string index, EP Out & EP In address, EP size
  TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 0, EPNUM_MIDI, 0x80 | EPNUM_MIDI, 64),

  //------------- Vendor (WebUSB) -------------//
  // Interface number, string index, EP Out & IN address, EP size
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, 4, EPNUM_VENDOR, 0x80 | EPNUM_VENDOR, 64)
};

#if TUD_OPT_HIGH_SPEED
uint8_t const desc_hs_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 150),

  // Interface number, string index, EP Out & EP In address, EP size
  TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 0, EPNUM_MIDI, 0x80 | EPNUM_MIDI, 512)
};
#endif

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // for multiple configurations

#if TUD_OPT_HIGH_SPEED
  // Although we are highspeed, host may be fullspeed.
  return (tud_speed_get() == TUSB_SPEED_HIGH) ?  desc_hs_configuration : desc_fs_configuration;
#else
  return desc_fs_configuration;
#endif
}

//--------------------------------------------------------------------+
// BOS Descriptor
//--------------------------------------------------------------------+

/* Microsoft OS 2.0 registry property descriptor
Per MS requirements https://msdn.microsoft.com/en-us/library/windows/hardware/hh450799(v=vs.85).aspx
device should create DeviceInterfaceGUIDs. It can be done by driver and
in case of real PnP solution device should expose MS "Microsoft OS 2.0
registry property descriptor". Such descriptor can insert any record
into Windows registry per device/configuration/interface. In our case it
will insert "DeviceInterfaceGUIDs" multistring property.
GUID is freshly generated and should be OK to use.
https://developers.google.com/web/fundamentals/native-hardware/build-for-webusb/
(Section Microsoft OS compatibility descriptors)
*/

#define BOS_TOTAL_LEN      (TUD_BOS_DESC_LEN + TUD_BOS_WEBUSB_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

#define MS_OS_20_DESC_LEN  0xB2

// BOS Descriptor is required for webUSB
uint8_t const desc_bos[] =
{
    // total length, number of device caps
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 2),

    // Vendor Code, iLandingPage
    TUD_BOS_WEBUSB_DESCRIPTOR(VENDOR_REQUEST_WEBUSB, 1),

    // Microsoft OS 2.0 descriptor
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT)
};

uint8_t const* tud_descriptor_bos_cb(void)
{
    return desc_bos;
}


uint8_t const desc_ms_os_20[] =
{
    // Set header: length, type, windows version, total length
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

    // Configuration subset header: length, type, configuration index, reserved, configuration total length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),

    // Function Subset header: length, type, first interface, reserved, subset length
    U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), ITF_NUM_VENDOR, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),

    // MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // sub-compatible

    // MS OS 2.0 Registry property descriptor: length, type
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08 - 0x08 - 0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A), // wPropertyDataType, wPropertyNameLength and PropertyName "DeviceInterfaceGUIDs\0" in UTF-16
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00,
    'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00,
    U16_TO_U8S_LE(0x0050), // wPropertyDataLength
      //bPropertyData: �{975F44D9-0D08-43FD-8B3E-127CA8AFFF9D}�.
    '{', 0x00, '9', 0x00, '7', 0x00, '5', 0x00, 'F', 0x00, '4', 0x00, '4', 0x00, 'D', 0x00, '9', 0x00, '-', 0x00,
    '0', 0x00, 'D', 0x00, '0', 0x00, '8', 0x00, '-', 0x00, '4', 0x00, '3', 0x00, 'F', 0x00, 'D', 0x00, '-', 0x00,
    '8', 0x00, 'B', 0x00, '3', 0x00, 'E', 0x00, '-', 0x00, '1', 0x00, '2', 0x00, '7', 0x00, 'C', 0x00, 'A', 0x00,
    '8', 0x00, 'A', 0x00, 'F', 0x00, 'F', 0x00, 'F', 0x00, '9', 0x00, 'D', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00
};

TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "Incorrect size");

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const* string_desc_arr [] =
{
  (const char[]) { 0x09, 0x04 }, // 0: is supported language is English (0x0409)
  "Plinky",                     // 1: Manufacturer
  "PlinkySynth",                   // 2: Product
  "",                      // 3: Serials, should use chip ID
  "TinyUSB WebUSB"               // 4: Vendor Interface
//   "TinyUSB CDC",                 // 5: CDC Interface // unused
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;

  uint8_t chr_count;

  if ( index == 0)
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }else
  {
    // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
    // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

    if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

    const char* str = string_desc_arr[index];

    // Cap at max char
    chr_count = strlen(str);
    if ( chr_count > 31 ) chr_count = 31;

    char serialbuf[10];
    if (index==3) {
    	chr_count = sprintf(serialbuf,"%08x", (unsigned int)serialno);
    	str=serialbuf;
    }
    // Convert ASCII string into UTF-16
    for(uint8_t i=0; i<chr_count; i++)
    {
      _desc_str[1+i] = str[i];
    }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

  return _desc_str;
}
