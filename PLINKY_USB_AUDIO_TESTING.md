# Plinky USB Audio Testing Report

**Testers:** George Redpath, Claude (Anthropic)
**Date:** January 2025
**Hardware:** Plinky V3 Black (STM32L476VGT6)
**Goal:** Add USB Audio Class 2.0 output for direct DAW integration (Ableton, etc.)

---

## Executive Summary

We attempted to add USB Audio output to Plinky using TinyUSB 0.9.0. **Partial success** - audio streams to DAW but native 48kHz has timing conflicts with touch/LED systems. The STM32L476's architecture creates fundamental constraints that make reliable 48kHz USB audio challenging without hardware changes.

---

## Test Configuration

| Component | Version/Setting |
|-----------|-----------------|
| TinyUSB | 0.9.0 (existing in Plinky) |
| USB Mode | Audio Class 2.0 (UAC2) |
| Target Sample Rate | 48kHz (DAW standard) |
| Bit Depth | 16-bit stereo |
| Sync Mode | Adaptive (no feedback endpoint) |

---

## Approach 1: Resampling (31.25kHz → 48kHz)

### Implementation
- Keep Plinky DSP at native 31.25kHz
- Resample to 48kHz in USB callback using polyphase FIR filter
- Ring buffer in main RAM

### Results
| Aspect | Result |
|--------|--------|
| Touch/LED | ✅ Working |
| Audio streams | ✅ Yes |
| Audio quality | ⚠️ Audible aliasing artifacts |
| CPU usage | ~15% overhead for resampler |

### Conclusion
Works but audio quality compromised. Polyphase FIR with 16 taps and 8 phases wasn't sufficient to eliminate aliasing. Would need longer filter (more CPU) or accept artifacts.

---

## Approach 2: Native 48kHz DSP

### Changes Required
```c
// main.c - Clock configuration
PeriphClkInit.PLLSAI2.PLLSAI2N = 42;  // Was 14 for 31.25kHz
hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;

// codec.h - WM8758 sample rate
wmcodec_write(ADDCTRL, ADDCTRL_SR_48kHz);

// arp.h - Timing constant
#define ACCURATE_FS 48000  // Was 31250
```

### Problem Discovered: Clock Tree Conflict

**ROOT CAUSE:** PLLSAI2 feeds BOTH SAI (audio) AND ADC (touch/CV inputs)

```
PLLSAI2 (N=14 for 31.25kHz, N=42 for 48kHz)
    ├── SAI2CLK → Audio codec ✓
    └── ADC2CLK → Touch sensing, CV inputs ✗ BREAKS AT 48kHz
```

Changing PLLSAI2N from 14→42 changes ADC clock timing, breaking touch sensing completely.

### Fix Attempted: Separate ADC to PLLSAI1

```c
// Move ADC to PLLSAI1 (already configured for USB at 48MHz)
PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
PeriphClkInit.PLLSAI2.PLLSAI2ClockOut = RCC_PLLSAI2_SAI2CLK;  // Only SAI
PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK | RCC_PLLSAI1_ADC1CLK;
```

### Results After Clock Separation

| Aspect | Result |
|--------|--------|
| Device boots | ✅ Yes |
| Audio @ 48kHz | ✅ Correct sample rate |
| Touch sensing | ❌ Not responding |
| LED matrix | ❌ Not updating |
| CV inputs | ❓ Not tested |

---

## Approach 3: Timing Analysis

### The Fundamental Problem

At 48kHz with BLOCK_SAMPLES=64:

| Parameter | 31.25kHz | 48kHz | Impact |
|-----------|----------|-------|--------|
| DMA callback rate | 976 Hz | 1500 Hz | +54% faster |
| Time between callbacks | 1.024 ms | 0.667 ms | -35% less time |
| TSC acquisition time | ~550 µs | ~550 µs | Unchanged |
| Available margin | ~470 µs | ~117 µs | **CRITICAL** |

### TSC (Touch Sensing Controller) Bottleneck

The TSC peripheral requires ~550µs to scan all 36 capacitive sensors. At 48kHz:
- Callback window: 667µs
- TSC needs: 550µs
- LED update needs: ~50µs
- **Margin: ~67µs** (too tight, causes overruns)

### Tests Performed

#### Test 3a: Alternate Touch/LED Updates
```c
if (half) {
    touch_update();  // Only on half=1
} else {
    led_update();    // Only on half=0
}
```
**Result:** LEDs work, touch responsive, but frame buffer synchronization broken - touch position doesn't match LED display correctly.

#### Test 3b: Increase BLOCK_SAMPLES to 128
```c
#define BLOCK_SAMPLES 128  // Was 64
```
**Result:** RAM overflow (fixed with SRAM2), but LEDs still not working. Callback rate reduced to 750Hz but still insufficient.

#### Test 3c: SRAM2 for USB Buffers
```c
#define USB_AUDIO_SRAM2 __attribute__((section(".usb_audio_ram")))
static volatile int16_t usb_audio_ring_l[512] USB_AUDIO_SRAM2;
```
**Result:** Freed ~2KB main RAM, enabled larger BLOCK_SAMPLES, but didn't solve timing issue.

---

## TinyUSB 0.9.0 Limitations

### No Feedback Endpoint for TX-Only

TinyUSB 0.9.0 doesn't support feedback endpoints for device→host only audio:
- `audiod_interface_t` has no `ep_fb` member
- Can't implement asynchronous mode properly
- Forced to use adaptive mode (device adapts to host timing)

### Adaptive Mode Implementation
```c
// usb_descriptors.c - Endpoint descriptor
// bmAttributes: 0x09 = isochronous (0x01) + adaptive sync (0x08)
7, TUSB_DESC_ENDPOINT, 0x80 | EPNUM_AUDIO_IN, 0x09, U16_TO_U8S_LE(196), 1,
```

This works but relies on host clock, which can cause drift over time.

---

## What Works

| Feature | Status | Notes |
|---------|--------|-------|
| USB Audio enumeration | ✅ | Device appears as "Plinky" audio input |
| Audio streaming @ 31.25kHz | ✅ | With resampler to 48kHz |
| Audio streaming @ 48kHz native | ⚠️ | Audio works but UI broken |
| Clock separation (ADC→PLLSAI1) | ✅ | Compiles and boots |
| SRAM2 for buffers | ✅ | Saves ~2KB main RAM |
| Ring buffer architecture | ✅ | Non-blocking DMA→USB transfer |

## What Doesn't Work

| Feature | Status | Root Cause |
|---------|--------|------------|
| Touch @ 48kHz | ❌ | TSC timing vs callback rate |
| LEDs @ 48kHz | ❌ | Frame sync with touch broken |
| Feedback endpoint | ❌ | TinyUSB 0.9.0 limitation |
| Native 48kHz + full UI | ❌ | Fundamental timing conflict |

---

## Technical Deep Dive: Why 48kHz Breaks UI

### uitick() Function Analysis

```c
void uitick(u32 *dst, const u32 *src, int half) {
    touch_update();   // ~550µs - TSC acquisition
    led_update();     // ~50µs - LED matrix refresh
    DoAudio(...);     // DSP processing
}
```

### Timing Budget

```
31.25kHz (working):
├── Callback period: 1024µs
├── touch_update(): 550µs
├── led_update(): 50µs
├── DoAudio(): 300µs
└── Margin: 124µs ✓

48kHz (broken):
├── Callback period: 667µs
├── touch_update(): 550µs
├── led_update(): 50µs
├── DoAudio(): 200µs (faster due to smaller effective block)
└── Margin: -133µs ✗ OVERRUN
```

### Frame Buffer Synchronization

Touch and LED share frame buffers (`fingers_ui_time`, `fingers_synth_time`). When separated:
- Touch writes to frame N
- LED reads from frame N-1 (stale)
- Position/pressure display doesn't match actual touch

This is why alternating touch/LED updates appeared to work but felt "wrong."

---

## Recommendations

### Option A: Accept 31.25kHz + Resampling
- Use polyphase resampler (already implemented)
- Accept minor aliasing artifacts
- Potentially improve with longer FIR filter (more CPU)
- **Effort: Low, Quality: Medium**

### Option B: Upgrade TinyUSB
- Update to TinyUSB 0.15+ with feedback endpoint support
- Implement proper asynchronous mode
- Still won't fix 48kHz timing issue
- **Effort: Medium, Quality: Medium**

### Option C: Decouple UI from Audio
- Move touch_update() to separate timer (TIM7?)
- Run LED matrix from hardware timer
- Major architectural change
- **Effort: High, Quality: High**

### Option D: Hardware Revision
- Use STM32H7 (6x faster, USB HS, more RAM)
- Or add dedicated USB audio chip (PCM2706, etc.)
- Trivially solves all timing issues
- **Effort: Very High (new PCB), Quality: Perfect**

---

## Files Modified

| File | Changes |
|------|---------|
| `sw/Core/Src/tinyusb/src/tusb_config.h` | USB Audio Class enable, buffer sizes |
| `sw/Core/Src/tinyusb/src/usb_descriptors.c` | UAC2 descriptors, adaptive mode endpoint |
| `sw/Core/Src/tinyusb/src/usbmidi.c` | Ring buffer, resampler, audio callbacks |
| `sw/Core/Src/main.c` | Clock config, ADC→PLLSAI1 separation |
| `sw/Core/Src/codec.h` | 48kHz codec setting, DMA hooks |
| `sw/Core/Src/arp.h` | ACCURATE_FS timing constant |
| `sw/Core/Src/plinky.c` | BLOCK_SAMPLES, uitick modifications |
| `sw/STM32L476VGTX_FLASH.ld` | SRAM2 section for USB buffers |

---

## Code Repository

USB Audio implementation preserved at:
**https://github.com/Ziforge/usb-audio-stm32** (public, CC BY-NC)

---

## Conclusion

USB Audio on Plinky is **technically possible but compromised** on the STM32L476. The chip wasn't designed for this use case - it's a low-power MCU being pushed beyond its comfortable limits.

The fundamental conflict is:
- **USB Audio wants 48kHz** (DAW standard)
- **Plinky's UI (TSC + LED) needs ~1ms callback windows**
- **48kHz with BLOCK_SAMPLES=64 gives only 0.67ms**
- **Something has to give**

For a "Plinky Pro" with proper USB Audio, an STM32H7 or dedicated USB audio chip would be the right solution.

---

*Report prepared for Alex Evans (mmalex) - Plinky creator*
