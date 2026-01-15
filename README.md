# USB Audio STM32

USB Audio Class 2.0 (UAC2) implementation for STM32L476 using TinyUSB 0.9.0.

## Overview

This implementation provides USB Audio output (device → host) at 48kHz/16-bit stereo using adaptive mode (no feedback endpoint required for TinyUSB 0.9.0).

## Key Features

- **Sample Rate:** 48kHz native (or 31.25kHz with polyphase resampler)
- **Bit Depth:** 16-bit stereo
- **Mode:** Adaptive (device adapts to USB frame timing)
- **Ring Buffer:** In SRAM2 (separate 32KB bank on STM32L476)
- **Clock:** ADC separated to PLLSAI1, SAI on PLLSAI2

## Files

```
src/
├── tinyusb/
│   ├── tusb_config.h      # USB Audio Class configuration
│   ├── usb_descriptors.c  # UAC2 descriptors (adaptive mode)
│   └── usbmidi.c          # Ring buffer, callbacks, resampler
└── STM32L476VGTX_FLASH.ld # Linker script with SRAM2 section
```

## Clock Configuration (main.c changes)

```c
// Separate ADC from audio PLL to allow PLLSAI2 changes for 48kHz
PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
PeriphClkInit.PLLSAI2.PLLSAI2N = 42;  // For 48kHz audio
PeriphClkInit.PLLSAI2.PLLSAI2ClockOut = RCC_PLLSAI2_SAI2CLK;  // Only SAI
```

## SRAM2 Usage

USB audio ring buffers are placed in SRAM2 to free main RAM:

```c
#define USB_AUDIO_SRAM2 __attribute__((section(".usb_audio_ram")))
static volatile int16_t usb_audio_ring_l[512] USB_AUDIO_SRAM2;
static volatile int16_t usb_audio_ring_r[512] USB_AUDIO_SRAM2;
```

Linker script section:
```ld
.usb_audio (NOLOAD) : {
    . = ALIGN(4);
    *(.usb_audio_ram)
    . = ALIGN(4);
} >RAM2
```

**Important:** SRAM2 is NOLOAD - must call `usb_audio_init()` before use.

## DMA Callback Integration

In your codec/SAI DMA callbacks:

```c
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef* hsai) {
    // Your audio processing...
    usb_audio_write_samples(audio_buffer, BLOCK_SAMPLES);
}
```

## Limitations (STM32L476)

- USB Full Speed only (12Mbps) - limits bandwidth
- Tight timing at 48kHz with BLOCK_SAMPLES=64
- May need BLOCK_SAMPLES=128 for stable touch/LED updates
- No feedback endpoint support in TinyUSB 0.9.0 for TX-only

## Future: STM32H7 / Daisy

For proper USB audio support, recommend STM32H750 (Daisy Seed):
- USB High Speed (480Mbps)
- Native 48/96kHz trivial
- Bidirectional audio
- 6x CPU headroom

## License

Based on work from Plinky Synth (open source).
