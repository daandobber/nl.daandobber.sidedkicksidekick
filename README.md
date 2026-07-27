# Sidekick Sidekick

A multitrack performance looper for Tanmatsu and the Teenage Engineering
EP-136 K.O.-sidekick.

Sidekick Sidekick receives the EP-136's USB Audio Class 2.0 stream, monitors
the physical CH1, CH2 and AUX inputs through the Tanmatsu audio output, and
records four independent stereo loops.

## Controls

- `1`–`4`: select a track
- `F2`: record or overdub the selected track
- `F3`: play or pause
- `F4`: clear the selected track
- `F5`: select the next track
- `T`: tap tempo
- `M`: toggle metronome
- Arrow up/down: adjust BPM
- Arrow left/right: choose 1, 2, 4, 8 or 16 bars
- Volume buttons: adjust the selected track
- `F1`: return to the launcher

## Build

Use an ESP-IDF 5.5.1 environment:

```sh
idf.py -B build/tanmatsu \
  -DIDF_TARGET=esp32p4 \
  -DDEVICE=tanmatsu \
  -DSDKCONFIG_DEFAULTS="sdkconfigs/general;sdkconfigs/tanmatsu" \
  build
```

Connect the EP-136 to the Tanmatsu USB-A host port with a data-capable cable.
Enable USB audio output on the EP-136.
