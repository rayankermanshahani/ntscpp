# NTSC-M notes
National Television System Committee, North American Standard

## Key Parameters
- resolution:
  - 525 lines total, ~480 visible (rows of pixels)
  - 45 lines are "metadata" used for sync signals, not picture data
  - 4:3 aspect ratio
- field/frame rate:
  - frame is divided into odd and even lines (odd comes first)
  - refresh rates: ~59.94 fields/s ~= 29.97 fps (frames/s)
- horizontal scan:
  - total length = ~63.55 µs = 1/15734 Hz (aka line rate)
  - horizontal blanking interval (~10.9 µs per line); contains sync pulse and color burst
    - front porch (~1.5 µs); ensures beam is fully off at blanking level (0 IRE) before the sync
    - horizontal sync pulse (~4.7 µs); broad negative (-40 IRE) pulse that signals EOL to tell receiver to retrace to next line
    - breezeway (~0.6 µs); brief interval between sync pulse + color burst to let signal return to blanking level (0 IRE)
    - color burst (~2.5 µs); short burst of color subcarrier sinusoid (8-10 cycles of 3.579545 MHz) that's ~40 IRE peak-to-peak that serves as a phase and amplitude reference for the receiver's color demodulator 
    - back porch (~1.6 µs); blanking period (0 IRE) after color burst to give receiver time to clamp its black level and retrace before active video starts
  - active video (~52.9 µs per line) 
    - modulates instantaneous amplitude of composite signal to carry picture luminance (Y)
- video bandwidth:
  - ~4.2 MHz luminance bandwidth (monochrome)
- color subcarrier:
  - ~3.579545 MHz used for chrominance modulation
  - chosen as 455/2 times the horizontal line rate 
- audio carrier:
  - +4.5 MHz offset from video carrier
  - frequency-modulated (FM) with the TV sound

## General notes
- interlaced scanning:
  - image is split into two fields; odd and even lines
  - each field has 262.5 lines; total of 525 lines
  - fields are scanned at ~59.94 fields per second --(/ 2 fields in the frame)--> 29.97 fps

- reference levels (IRE units):
  - NTSC signal levels are measured in IRE (Institute of Radio Engineers) units
  - 100 IRE = +714.3 mV <==> 1 IRE = +7.143 mV
  - analog waveform would have ~1 V peak-to-peak signal
  - +100 IRE = white level (brightest visible pixel) <==> +714.3 mV
  - +7.5 IRE = black level (darkest visible pixel) <==> +53.55 mV
  - 0 IRE = blanking level (no data being sent) <==> 0 mV
  - -40 IRE = sync tip (bottom of sync pulses, ie. "start of line" signal) <==> -285.6 mV
  - note: in NTSC-J (Japan) the black level is at 0 IRE (blanking level) instead of 7.5 IRE

## NTSC composite video signal structure (baseband)
- NTSC analog signal = luminance (brightness) & chrominance (color) & sync timing information

### line timing
- horizontal line composite signal:
  - total length = ~63.55 µs = 1/15734 Hz
  - horizontal blanking interval (~10.9 µs per line); contains sync pulse and color burst
    - front porch (~1.5 µs); ensures beam is fully off at blanking level (0 IRE) before the sync
    - horizontal sync pulse (~4.7 µs); broad negative (-40 IRE) pulse that signals EOL to tell receiver to retrace to next line
    - breezeway (~0.6 µs); brief interval between sync pulse + color burst to let signal return to blanking level (0 IRE)
    - color burst (~2.5 µs); short burst of color subcarrier sinusoid (8-10 cycles of 3.579545 MHz) that's ~40 IRE peak-to-peak that serves as a phase and amplitude reference for the receiver's color demodulator 
    - back porch (~1.6 µs); blanking period (0 IRE) after color burst to give receiver time to clamp its black level and retrace before active video starts
  - active video (~52.9 µs per line) 
    - modulates instantaneous amplitude of composite signal to carry picture luminance (Y)
- vertical blanking and vertical sync:
  - vertical blanking interval (VBI) occurs at end end of each field 
  - VBI signal informs electron beam to return from bottom to top (ie. start a new field)
  - vertical blanking spans the ~45 lines (= 525 total - 480 visible) that are not used for picture data
  - VBI pattern: 
    - 6 equalizing pulses before
      - ~2.3 µs each
      - come twice the normal line rate (every ~31.778 µs instead of 63.55 µs)
      - "keep-alive" signals to horizontal sync circuits
    - 3 vertical sync pulses
      - broad high level pulses
      - inverted from normal sync
      - tells receiver to start new field 
    - 6 equalizing pulses after

### YIQ color model
- luminance signal, luma (Y'):
  - represents monochromatic brightness of an image
  - weighted sum of linear RGB inputs after gamma correction
  - NTSC standard for luminance signal: Y' = 0.299*R' + 0.587*G' + 0.114*B'
  - Y' modulates the instantaneous amplitude of composite video signal (b/w black and white levels)
  - has bandwidth up to 4.2 MHz
- chrominance signal, chroma (C = I & Q):
  - represents color information
  - conveyed via two color-difference signals that modulate the 3.579545 subcarrier in a quadrature scheme (QAM)
  - R'-Y', red difference
  - B'-Y', blue difference
  - G'-Y' = -((R'-Y') + (B'-Y')), green difference can be inferred from red and blue differences
  - I (in-phase) and Q (quadrature) signals are linear combinations (rotated coordinate system) of R'-Y' and B'-Y'
  - I axis (~1.3 MHz bandwidth) corresponds to orange-cyan hues
  - Q axis (~0.4 MHz bandwidth) corresponds to purple-green hues
  - the I and Q signals aplitude-modulate two subcarrier waves of frequency ~3.579545 MHz
    - one cosine and the other sine
    - I and Q are 90 degrees out of phase
