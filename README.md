# Hallow

A candy dispenser that slaps back. An ESP32 watches the bowl through a load cell
(HX711); when one grab takes more than 10 g, it opens the lid, fires a
spring-loaded arm out at the hand, reels the arm back in, closes up and resets.

- `src/main.cpp`, `include/pins.h` — firmware (PlatformIO, `esp32dev`, Arduino).
- `web/index.html` — the controller page, one static file. It talks to the ESP32
  over Web Serial (Chrome or Edge), so the board plugs into the visitor's computer.
  Live at https://aditya-h.com/hallow/.

## Build and flash

```bash
pio run -t upload
pio device monitor   # 115200 baud
```

## Deploy the page

```bash
scp web/index.html root@157.230.55.192:/opt/hallow/web/
```

## The sequence

Five states, which the page's **Machine State** section draws and steps through:

1. **Closed & unspooled** — lid down, rack cocked by the pinion, tether slack.
2. **Triggered** — a grab took more than 10 g.
3. **Arm out** — lid lifted by the cam in 5° steps; the pinion (teeth on half its
   rim, always CCW) turns 90°, its gap releases the rack, the spring fires the
   rack and the arm; the rest of the turn re-cocks the rack.
4. **Reeled in** — the spool pulls the arm home by its tether.
5. **Lid closed** — then the spool lets the tether out, back to 1.

Each motor phase is a serial command (`step_fire`, `step_reel`, `step_close`,
`step_unspool`) and announces itself with `F,<phase>` as it starts and `F,done`
when it ends — whether the page asked for it, a real grab started it, or
`run_sequence` did. With the board connected the page animates from those
announcements, so it shows what the motors are actually doing. The full serial
protocol is at the top of `src/main.cpp`.
