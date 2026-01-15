
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "tusb.h"
#include "main.h"
#include "class/audio/audio.h"

//--------------------------------------------------------------------+
// USB Audio Support - Polyphase FIR resampler (31.25kHz -> 48kHz)
// Precomputed coefficients = high quality + low CPU
//--------------------------------------------------------------------+
#if CFG_TUD_AUDIO

#include "class/audio/audio_device.h"

// Ring buffer for audio samples - in SRAM2 (separate 32KB bank)
#define USB_AUDIO_RING_SAMPLES 512
#define USB_AUDIO_RING_MASK (USB_AUDIO_RING_SAMPLES - 1)
#define USB_AUDIO_TARGET_FILL 192  // Target ~4ms latency at 48kHz
#define USB_AUDIO_SRAM2 __attribute__((section(".usb_audio_ram")))

static volatile int16_t usb_audio_ring_l[USB_AUDIO_RING_SAMPLES] USB_AUDIO_SRAM2;
static volatile int16_t usb_audio_ring_r[USB_AUDIO_RING_SAMPLES] USB_AUDIO_SRAM2;
static volatile uint32_t ring_write_pos USB_AUDIO_SRAM2;
static volatile uint32_t ring_read_pos USB_AUDIO_SRAM2;
static volatile uint8_t usb_audio_streaming USB_AUDIO_SRAM2;
static volatile uint8_t usb_audio_initialized USB_AUDIO_SRAM2;

// Initialize SRAM2 variables - MUST be called before any USB audio operations
void usb_audio_init(void) {
    ring_write_pos = 0;
    ring_read_pos = 0;
    usb_audio_streaming = 0;
    usb_audio_initialized = 1;
    for (int i = 0; i < USB_AUDIO_RING_SAMPLES; i++) {
        usb_audio_ring_l[i] = 0;
        usb_audio_ring_r[i] = 0;
    }
}

// Polyphase sinc interpolator: 31250->48000 Hz
// Windowed sinc (Kaiser beta=7), fc=0.32 (15.4kHz)
// Each phase = fractional delay 0/8, 1/8, ..., 7/8
// Q15 format, all phases sum to ~32767 (unity gain)
#define POLY_PHASES 8
#define POLY_TAPS 16

static const int16_t poly_coeffs[POLY_PHASES][POLY_TAPS] = {
    {      5,     34,   -247,    237,    969,  -2742,    765,  17364,  17364,    765,  -2742,    969,    237,   -247,     34,      5 },  // Phase 0: delay=0/8
    {      6,     18,   -249,    389,    708,  -3032,   2453,  18787,  15661,   -707,  -2321,   1155,     78,   -230,     47,      3 },  // Phase 1: delay=1/8
    {      8,      0,   -235,    526,    380,  -3158,   4319,  19886,  13732,  -1932,  -1801,   1261,    -77,   -200,     57,      1 },  // Phase 2: delay=2/8
    {      9,    -19,   -206,    639,      0,  -3094,   6320,  20623,  11637,  -2893,  -1218,   1286,   -220,   -160,     63,     -1 },  // Phase 3: delay=3/8
    {      9,    -37,   -162,    720,   -417,  -2817,   8402,  20973,   9438,  -3577,   -606,   1234,   -345,   -111,     66,     -3 },  // Phase 4: delay=4/8
    {      9,    -54,   -105,    762,   -849,  -2311,  10508,  20921,   7202,  -3986,      0,   1111,   -445,    -56,     64,     -5 },  // Phase 5: delay=5/8
    {      8,    -69,    -37,    758,  -1274,  -1566,  12580,  20465,   4994,  -4127,    570,    928,   -515,      0,     58,     -6 },  // Phase 6: delay=6/8
    {      7,    -80,     39,    704,  -1665,   -581,  14555,  19615,   2877,  -4019,   1075,    695,   -552,     55,     50,     -7 },  // Phase 7: delay=7/8
};

// Resampler state
static uint32_t resample_phase = 0;  // 24.8 fixed point

// Called from DMA callback
void usb_audio_write_samples(const int16_t* samples, uint32_t count) {
    uint32_t wp = ring_write_pos;
    for (uint32_t i = 0; i < count; i++) {
        usb_audio_ring_l[wp & USB_AUDIO_RING_MASK] = samples[i * 2];
        usb_audio_ring_r[wp & USB_AUDIO_RING_MASK] = samples[i * 2 + 1];
        wp++;
    }
    ring_write_pos = wp;
}

// Apply polyphase FIR filter (unrolled for 16 taps)
static inline int32_t poly_filter(const volatile int16_t* ring, uint32_t base_idx, int phase) {
    const int16_t* c = poly_coeffs[phase];
    uint32_t i = base_idx - 8;  // Start 8 samples before center
    int32_t sum =
        (int32_t)ring[(i  ) & USB_AUDIO_RING_MASK] * c[0] +
        (int32_t)ring[(i+1) & USB_AUDIO_RING_MASK] * c[1] +
        (int32_t)ring[(i+2) & USB_AUDIO_RING_MASK] * c[2] +
        (int32_t)ring[(i+3) & USB_AUDIO_RING_MASK] * c[3] +
        (int32_t)ring[(i+4) & USB_AUDIO_RING_MASK] * c[4] +
        (int32_t)ring[(i+5) & USB_AUDIO_RING_MASK] * c[5] +
        (int32_t)ring[(i+6) & USB_AUDIO_RING_MASK] * c[6] +
        (int32_t)ring[(i+7) & USB_AUDIO_RING_MASK] * c[7] +
        (int32_t)ring[(i+8) & USB_AUDIO_RING_MASK] * c[8] +
        (int32_t)ring[(i+9) & USB_AUDIO_RING_MASK] * c[9] +
        (int32_t)ring[(i+10) & USB_AUDIO_RING_MASK] * c[10] +
        (int32_t)ring[(i+11) & USB_AUDIO_RING_MASK] * c[11] +
        (int32_t)ring[(i+12) & USB_AUDIO_RING_MASK] * c[12] +
        (int32_t)ring[(i+13) & USB_AUDIO_RING_MASK] * c[13] +
        (int32_t)ring[(i+14) & USB_AUDIO_RING_MASK] * c[14] +
        (int32_t)ring[(i+15) & USB_AUDIO_RING_MASK] * c[15];
    return sum >> 15;
}

// TinyUSB audio callback - direct 48kHz passthrough (no resampling needed!)
bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting) {
    (void)rhport; (void)itf; (void)ep_in; (void)cur_alt_setting;

    int16_t tx_buf[48 * 2];

    // Direct copy - DSP and USB both at 48kHz
    for (int i = 0; i < 48; i++) {
        uint32_t idx = ring_read_pos & USB_AUDIO_RING_MASK;
        tx_buf[i * 2] = usb_audio_ring_l[idx];
        tx_buf[i * 2 + 1] = usb_audio_ring_r[idx];
        ring_read_pos++;
    }

    tud_audio_write((uint8_t*)tx_buf, sizeof(tx_buf));
    return true;
}

// Audio set interface callback
bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request) {
    (void)rhport; (void)p_request;
    // Reset ring buffer and resampler state
    ring_write_pos = 0;
    ring_read_pos = 0;
    resample_phase = 0;
    // Clear ring buffers
    for (int i = 0; i < USB_AUDIO_RING_SAMPLES; i++) {
        usb_audio_ring_l[i] = 0;
        usb_audio_ring_r[i] = 0;
    }
    return true;
}

// Audio control request callbacks (required stubs)
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const * p_request) {
    (void)rhport;

    // Handle clock source sample rate request
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    if (entityID == 1) {  // Clock Source
        if (ctrlSel == AUDIO_CS_CTRL_SAM_FREQ) {
            if (p_request->bRequest == AUDIO_CS_REQ_CUR) {
                // Return current sample rate: 48000 Hz (native Plinky rate)
                uint32_t freq = 48000;
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &freq, sizeof(freq));
            } else if (p_request->bRequest == AUDIO_CS_REQ_RANGE) {
                // Return supported range: only 48000 Hz
                static const struct {
                    uint16_t wNumSubRanges;
                    uint32_t dMIN;
                    uint32_t dMAX;
                    uint32_t dRES;
                } __attribute__((packed)) range = { 1, 48000, 48000, 0 };
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void*)&range, sizeof(range));
            }
        } else if (ctrlSel == AUDIO_CS_CTRL_CLK_VALID) {
            // Clock is always valid
            uint8_t valid = 1;
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &valid, sizeof(valid));
        }
    }

    return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff) {
    (void)rhport; (void)p_request; (void)pBuff;
    // We don't support changing sample rate
    return true;
}

#endif // CFG_TUD_AUDIO


enum
{
    VENDOR_REQUEST_WEBUSB = 1,
    VENDOR_REQUEST_MICROSOFT = 2
};

extern uint8_t const desc_ms_os_20[];

extern bool web_serial_connected;
bool web_serial_connected = false;


#define URL  "www.plinkysynth.com/webusb"

const tusb_desc_webusb_url_t desc_url =
{
  .bLength         = 3 + sizeof(URL) - 1,
  .bDescriptorType = 3, // WEBUSB URL type
  .bScheme         = 1, // 0: http, 1: https
  .url             = URL
};



void OTG_FS_IRQHandler(void)
{
  tud_int_handler(0);
}

//int HAL_GetTick(void);
#define millis HAL_GetTick
typedef unsigned char u8;
extern u8 led_ram[9][8];
void board_led_write(bool state) {
//	led_ram[0][0]=state?255:0;
	if (state)
		GPIOD->BSRR=1;
	else
		GPIOD->BRR=1;
}

/* This MIDI example send sequence of note (on/off) repeatedly. To test on PC, you need to install
 * synth software and midi connection management software. On
 * - Linux (Ubuntu): install qsynth, qjackctl. Then connect TinyUSB output port to FLUID Synth input port
 * - Windows: install MIDI-OX
 * - MacOS: SimpleSynth
 */

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
void led_blinking_task(void);
//void midi_task(void);

extern TIM_HandleTypeDef htim1;

static inline uint32_t mix(uint32_t a,uint32_t b,uint32_t c) \
{ \
  a -= b; a -= c; a ^= (c>>13); \
  b -= c; b -= a; b ^= (a<<8); \
  c -= a; c -= b; c ^= (b>>13); \
  a -= b; a -= c; a ^= (c>>12);  \
  b -= c; b -= a; b ^= (a<<16); \
  c -= a; c -= b; c ^= (b>>5); \
  a -= b; a -= c; a ^= (c>>3);  \
  b -= c; b -= a; b ^= (a<<10); \
  c -= a; c -= b; c ^= (b>>15); \
  return c;
}
uint32_t serialno;

/*------------- MAIN -------------*/
void midiinit(void)
{
#if 0

	// based on https://github.com/hathach/tinyusb/blob/fix-stm32l4/hw/bsp/stm32l476disco/stm32l476disco.c
	 /* Enable Power Clock*/
	  __HAL_RCC_PWR_CLK_ENABLE();

	  /* Enable USB power on Pwrctrl CR2 register */
	  HAL_PWREx_EnableVddUSB();

	  GPIO_InitTypeDef  GPIO_InitStruct;
	 // USB
	  /* Configure DM DP Pins */
	  GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /* Configure VBUS Pin */
	  GPIO_InitStruct.Pin = GPIO_PIN_11;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /* Enable USB FS Clock */
	  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

	  // L476Disco use general GPIO PC11 for VBUS sensing instead of dedicated PA9 as others
	  // Disable VBUS Sense and force device mode
	  USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBDEN;

	  USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_FHMOD;
	  USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;

	  USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
	  USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
#endif

	uint32_t 	uid0=HAL_GetUIDw0 ();
	uint32_t 	uid1=HAL_GetUIDw1 ();
	uint32_t 	uid2=HAL_GetUIDw2 ();
	serialno = mix(uid0,uid1,uid2);

#if CFG_TUD_AUDIO
  usb_audio_init();  // Initialize SRAM2 variables before USB starts
#endif
  tusb_init();
}


bool midi_receive(unsigned char packet[4]) {
	  return tud_midi_available() && tud_midi_packet_read(packet);
}
bool usb_midi_write(const uint8_t packet[4]) {
    return tud_midi_packet_write(packet);
}

/*
int miditest(void) {
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  GPIOD->MODER|=1;

  while (1)
  {
    tud_task(); // tinyusb device task
    led_blinking_task();
    midi_task();
  }
  return 0;
}*/


//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
  web_serial_connected = true;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
  web_serial_connected = false;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
  web_serial_connected = false;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
  web_serial_connected = true;
}

//--------------------------------------------------------------------+
// WebUSB use vendor class
//--------------------------------------------------------------------+

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request)
{
    // nothing to with DATA & ACK stage
    if (stage != CONTROL_STAGE_SETUP) return true;

    switch (request->bRequest)
    {
    case VENDOR_REQUEST_WEBUSB:
        // match vendor request in BOS descriptor
        // Get landing page url
        return tud_control_xfer(rhport, request, (void*)&desc_url, desc_url.bLength);

    case VENDOR_REQUEST_MICROSOFT:
        if (request->wIndex == 7)
        {
            // Get Microsoft OS 2.0 compatible descriptor
            uint16_t total_len;
            memcpy(&total_len, desc_ms_os_20 + 8, 2);

            return tud_control_xfer(rhport, request, (void*)desc_ms_os_20, total_len);
        }
        else
        {
            return false;
        }

    case 0x22:
        // Webserial simulate the CDC_REQUEST_SET_CONTROL_LINE_STATE (0x22) to
        // connect and disconnect.
        web_serial_connected = (request->wValue != 0);

        // Always lit LED if connected
        if (web_serial_connected)
        {
            board_led_write(true);
//            blink_interval_ms = BLINK_ALWAYS_ON;

            //tud_vendor_write_str("\r\nTinyUSB WebUSB device example\r\n");
        }
        else
        {
            blink_interval_ms = BLINK_MOUNTED;
        }

        // response with status OK
        return tud_control_status(rhport, request);

    default:
        // stall unknown request
        return false;
    }

    return true;
}

void PumpWebUSB(bool calling_from_audio_thread);


/*
//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void)
{
    if (tud_cdc_connected())
    {
        // connected and there are data available
        if (tud_cdc_available())
        {
            uint8_t buf[64];

            uint32_t count = tud_cdc_read(buf, sizeof(buf));

            // echo back to both web serial and cdc
            echo_all(buf, count);
        }
    }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf;

    // connected
    if (dtr && rts)
    {
        // print initial message when connected
        tud_cdc_write_str("\r\nTinyUSB WebUSB device example\r\n");
    }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
    (void)itf;
}
*/
/*
//--------------------------------------------------------------------+
// MIDI Task
//--------------------------------------------------------------------+

void DebugLog(const char *fmt, ...);

void midi_task(void)
{
  static uint32_t start_ms = 0;

  if (tud_midi_available()) {
	  unsigned char packet[4]={};
	  if (tud_midi_receive(packet)) {
		//  DebugLog("%02x %02x %02x %02x\r\n", packet[0],packet[1],packet[2],packet[3]);
	  }
  }

  // send note every 1000 ms
  if (millis() - start_ms < 286) return; // not enough time
  start_ms += 286;

  // Previous positions in the note sequence.
  int previous = note_pos - 1;

  // If we currently are at position 0, set the
  // previous position to the last note in the sequence.
  if (previous < 0) previous = sizeof(note_sequence) - 1;

  // Send Note On for current position at full velocity (127) on channel 1.
  tudi_midi_write24(0, 0x90, note_sequence[note_pos], 127);

  // Send Note Off for previous note.
  tudi_midi_write24(0, 0x80, note_sequence[previous], 0);

  // Increment position
  note_pos++;

  // If we are at the end of the sequence, start over.
  if (note_pos >= sizeof(note_sequence)) note_pos = 0;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
*/
