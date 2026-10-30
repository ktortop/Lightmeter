# EVEI Light Meter – Arduino Mega 2560 shield (rev B)

Two-layer carrier shield that replaces the hand-wired proto shield. Generated from
`latest_version_of_lightmeter.ino` (pin map) and the Final Design Review slides (part list, wiring), and
checked against the team's current sketch, `Lightmeter_Week8.ino` in this repo (same pins; its `setLED()` sets the LED wiring).
No sketch was modified. Rev B needs a two-line firmware change for the GPS LED (see "Rev B changes"). Rev A is kept in `archive/revA/`.

Board: 90.7 x 102.1 mm with a 1.9 x 14 mm notch over the Mega's barrel jack, 2 layers, 1 oz copper, min track 0.3 mm (signals) / 0.6 mm (power), vias 0.8/0.4 mm (tented), copper at least 0.4 mm from every edge.
ERC: 0 violations. DRC: 0 errors, 0 warnings, 0 unconnected (schematic parity checked).

## Rev B changes

| Change | Detail |
|---|---|
| Second lux sensor port | J10, JST-XH 4 pin, same pinout as LUX1: GND · 5V · SDA · SCL. |
| I2C multiplexer | U3, PCA9548A (SOIC-24W, address 0x70). Needed because the TSL2591 has a fixed address (0x29), so two of them cannot share one bus. LUX2 is on mux channel 0 with 10k pull-ups (R11, R12). |
| Encoder port | J11, JST-XH 4 pin: GND · 5V · A · B. A goes to D2 (INT4), B to D3 (INT5), the two free hardware interrupts. |
| Encoder input conditioning | 10k pull-ups (R14, R15) so open-collector hall encoders work; 1k series + 1nF (R16/C10, R17/C11) filters cable noise and limits current if a pin is ever driven by mistake. |
| GPS status LED moved | D2/D3 to D4/D7, to free the interrupt pins for the encoder. |
| LEDs are common-cathode | Since Week 5 the sketch's `setLED()` drives a die by writing its pin HIGH (YELLOW = both HIGH, OFF = both LOW; the Week 5 comment says this fixed YELLOW and OFF being swapped on the real LEDs). So the LED connectors now give GND on pin 1 (common cathode), and each `*_red` pin drives the red die, each `*_green` pin the green die. All four states (green, red, yellow, off) show correctly with Week 8 as written. Rev A had 5V on pin 1 (common anode), which would have swapped YELLOW and OFF. |
| Barrel-jack notch | The Mega's DC jack (11 mm tall) reaches 0.4 mm under the rev A edge, higher than the 8.5 mm header gap. The bottom-left corner is notched to x = 12.9 mm, and the Mega hole beside the jack is left off (five Mega holes plus H1/H2 remain). |
| Regulator output cap | C2 is a 22 µF tantalum (EIA-3528-B). The AMS1117 needs some output-capacitor ESR to stay stable; a plain ceramic can make it oscillate. |
| SD chip-select pull-up | R18, 10k to 5V on the Mega side of the level shifter, so the card stays deselected while the Mega resets or is being programmed. |
| Nothing under the IMU | Track/via keepout under the BNO055. DC current in nearby traces bends the magnetometer and the heading. Nearest 5V trace is now 11 mm from the IMU centre. |
| Hand-soldering and fab | Thermal reliefs on all ground pads, vias tented, all silkscreen at least 0.15 mm, decoupling caps within 6 mm of their pins. |
| Board size | 13 mm taller (new top row). microSD moved up so its slot is still at the top edge. |

LUX1 stays on the main I2C bus. The mux powers up with every channel off, so LUX1 and the LCD work exactly as before.

### Firmware changes needed (sketch not edited)

1. GPS LED pins, in the constants block:
   ```cpp
   const int gps_red   = 4;   // was 2
   const int gps_green = 7;   // was 3
   ```
   **Make this change before plugging an encoder in.** The old sketch drives D2/D3 as outputs. The 1k series resistors limit the current, but the LED will not work until the pins are changed.

2. Reading the second lux sensor:
   ```cpp
   Adafruit_TSL2591 tsl2 = Adafruit_TSL2591(2592);
   void muxSelect(uint8_t ch) { Wire.beginTransmission(0x70); Wire.write(ch); Wire.endTransmission(); }
   // setup:  muxSelect(0x01); tsl2.begin(); tsl2.setGain(TSL2591_GAIN_LOW); muxSelect(0x00);
   // loop:   muxSelect(0x01); uint32_t lum2 = tsl2.getFullLuminosity(); muxSelect(0x00);
   ```
   Always write 0x00 afterwards. While channel 0 is open, both sensors answer at 0x29.

3. Encoder: D2 and D3 are hardware interrupts, so either `attachInterrupt(digitalPinToInterrupt(2), isrA, CHANGE)` or the PJRC `Encoder` library (`Encoder enc(2, 3);`) works at full speed.

## Folder contents

| Path | What |
|---|---|
| `kicad/` | KiCad 10 project: schematic, PCB, project-local symbol + footprint libs. Open `LightMeter_MegaShield.kicad_pro`. |
| `docs/schematic.pdf` | Schematic (A2, single sheet; rev B additions on the right). |
| `docs/render_top.png`, `render_bottom.png`, `render_3d.png` | 3D renders. |
| `docs/pcb_top.pdf`, `pcb_bottom.pdf` | Copper + silk plots. |
| `docs/BOM.csv` | Bill of materials (grouped). |
| `docs/LightMeter_MegaShield.step` | 3D model for the cart CAD. |
| `fab/LightMeter_MegaShield_gerbers.zip` | Gerbers + Excellon drills, ready for JLCPCB/PCBWay/OSH Park. `-pos.csv` is the SMD pick-and-place file. |
| `generator/` | Python scripts that generated everything (design.py holds the netlist and placement). Re-run if you change a pin. |

## How the Mega is "embedded"

The Arduino Mega 2560 plugs into the underside of this board through the standard Mega
stacking-header pattern (all 92 header and ICSP pads, checked pad by pad against KiCad's own
Arduino Mega template). Five of the Mega's six mounting holes are duplicated on the shield so the two
boards can be bolted together with M3 standoffs. The board is wider than the Mega on top (a
35 mm extension above the top header row) where all the cable connectors and the microSD live,
so no cable crosses the Mega. The left edge stops at x = 11 mm to clear the USB-B jack, and steps to 12.9 mm over the barrel jack.

Plain male headers (see "Assembly and soldering") or a Mega stacking header set both fit. The ICSP pads are on the
footprint but are left unconnected; do not fit a header there.

## What is on the board vs. on a cable

| Function | How | Mega pins (from the sketch) |
|---|---|---|
| Adafruit Ultimate GPS v3 (746) | Soldered onto the board on a 1x9 header (pin order PPS·VIN·GND·RX·TX·FIX·VBAT·EN·3.3V verified from Adafruit's board file) + two M2.5 holes | Serial1: D18→GPS RX, D19←GPS TX |
| microSD | Real socket on the board (Hirose DM3AT push-push), own AMS1117-3.3 regulator (22 µF tantalum output cap), 74LVC125A level shifter on MOSI/SCK/CS, 10k pull-ups on CS/DAT0/DAT1/DAT2 | HW SPI D50/51/52, CS D53 |
| Adafruit BNO055 (2472, non-STEMMA) | Soldered onto the board (1x6 + 1x4 headers, positions from Adafruit's board file) + four M2.5 holes. UART mode: PS1 tied to breakout 3Vo through solder jumper JP1 (bridged by default) | Serial2: D16→BNO SCL pin (UART RX), D17←BNO SDA pin (UART TX) |
| 20x4 LCD (I2C backpack) | J2, JST-XH 4 pin: GND · 5V · SDA · SCL | D20/D21 |
| TSL2591 lux sensor | J3, JST-XH 4 pin: GND · 5V · SDA · SCL (same pinout as LCD, cables interchangeable) | D20/D21 |
| Status LED 1 (GPS) | J4, JST-XH 3 pin: GND (common cathode) · RED · GREEN, 220 Ω series resistors on board | red D4 (`gps_red`), green D7 (`gps_green`) |
| Status LED 2 (LUX) | J5, same | red D5 (`lux_red`), green D6 (`lux_green`) |
| Status LED 3 (IMU) | J6, same | red D8 (`imu_red`), green D9 (`imu_green`) |
| Sleep button | J7, JST-XH 3 pin: GND · SW · 5V (5V for the arcade-button lamp, optional). 100 nF debounce cap on board, uses INPUT_PULLUP | D10 |
| Log-toggle button | J8, same | D11 |
| Force-log button | J9, same | D12 |

LED polarity matches `setLED()` in Week 5-8: output HIGH = die on, both HIGH = yellow, both LOW = off. Wire each LED's common cathode (usually the middle leg of a 3-leg bi-colour LED; check its datasheet) to pin 1, its red leg to pin 2 and its green leg to pin 3, as the silkscreen says (`GND R G`). Two separate red and green LEDs work too: both cathodes to pin 1. Common-anode LEDs will not work on this board.
Buttons go to GND (active LOW), matching `digitalRead(...) == LOW`.

### Cable pinouts (board side is always pin 1 = square pad = left end of the label)

| Port | Pin 1 | Pin 2 | Pin 3 | Pin 4 | Far end |
|---|---|---|---|---|---|
| LCD (J2) | GND | 5V | SDA | SCL | I2C backpack: GND, VCC, SDA, SCL (same order) |
| LUX1 (J3), LUX2 (J10) | GND | 5V | SDA | SCL | Adafruit TSL2591: GND, VIN, SDA, SCL (leave 3Vo and INT unconnected) |
| ENCODER (J11) | GND | 5V | A | B | encoder GND, VCC (5 V encoders only), channel A, channel B |
| GPS/LUX/IMU LED (J4-J6) | GND | red | green | | LED common cathode, red anode, green anode (no resistor needed, 220 Ω is on the board) |
| SLEEP/LOG/FORCE (J7-J9) | GND | SW | 5V | | switch between SW and GND; lamp (optional) between 5V and GND |

**Do not swap LED and button cables.** Both are 3-pin JST-XH, so either fits either port. A button in an LED port is harmless, but an LED in a button port puts 5 V straight onto its green leg with no resistor and burns it out. Label or colour-code the six 3-pin cables. The 4-pin ports are safe to mix up: GND and 5V are on the same pins everywhere.

## Assembly and soldering

Checked against the layout: all parts are on the top side, nothing sits under the GPS or IMU breakouts, no part is in
the path of the Mega header bodies, and the surface-mount parts are all soldered before anything tall goes on
(the closest, R14, is 0.8 mm from C5). Ground pads use thermal reliefs so they take solder with an ordinary iron.
No tracks run under any screw head, washer or standoff. The only pad gap under 0.4 mm is the microSD's unused
card-detect pad (pin 9), left unconnected so a bridge there does no harm.

**Tools:** temperature-controlled iron with a 1–1.6 mm chisel tip, 0.5 mm flux-core solder, a flux pen,
solder wick, fine tweezers, a magnifier or phone camera, a multimeter.

**Headers to buy:** plain male headers are recommended, since nothing stacks on top of this shield:
two 1x40 strips (cut into 1x10, 5 x 1x8) plus one 2x18 (or a 2x20 cut down). They hold the shield
2.5 mm higher than stacking headers do, which adds clearance over the Mega's USB jack, barrel jack and
USB-chip ICSP pins. A "Mega stacking header set" also fits; no part is in the way of either.

### Buy list (not on the BOM)

| Item | Qty | Notes |
|---|---|---|
| Male pin headers, 2.54 mm, 1x40 | 2 | cut into 1x10 + 5 x 1x8 for the Mega |
| Male pin header, 2.54 mm, 2x18 (or 2x20 cut down) | 1 | Mega double row |
| M2.5 nylon hex standoffs, 5 mm | 2 | GPS (the coin-cell holder needs the height) |
| M2.5 nylon screws, 6 mm | 4 | GPS (2 standoffs, both ends) |
| M2.5 nylon screws + nuts | 4 | IMU (optional, the headers hold it) |
| M3 nylon standoffs, 11 mm, + screws | up to 5 | Mega to shield (optional); 11 mm = 8.5 mm female header + 2.5 mm male header spacer |
| JST-XH housings XHP-4 / XHP-3 | 4 / 6 | cable ends |
| JST-XH crimp contacts SXH-001T-P0.6 | about 40 | needs a JST-type crimper (for example Engineer PA-09); pre-crimped XH leads avoid the tool |
| 5 mm bi-colour red/green LED, common cathode | 3 | or separate red + green LEDs |
| microSD card, 32 GB or less, FAT32 | 1 | the SD library only reads FAT16/FAT32 |
| CR1220 coin cell | 1 | optional, keeps the GPS almanac for faster fixes |

### Solder order (smallest and hardest first)

1. **microSD socket (J1).** Hardest joint: 8 pins at 1.1 mm pitch that stick out only 0.5 mm past the
   metal shell. Flux the pads, place the socket, tack one shell tab, check alignment, solder all four shell
   tabs, then touch each signal pin with the tip of the iron. Inspect for bridges and wick any away.
2. **SOIC chips, U1 (74LVC125A) and U3 (PCA9548A).** Match the dot on the chip to the pin-1 mark on the
   silkscreen. Tack one corner pin, check both rows are on their pads, solder the opposite corner, then
   the rest. Flux and wick fix bridges.
3. **Regulator U2 (AMS1117, SOT-223).** The large tab needs more heat: use a bigger tip or more dwell time.
4. **0805 resistors and capacitors.** Do one value at a time from the BOM: 220 Ω (R1–R6), 10 kΩ
   (R7–R15, R18), 1 kΩ (R16, R17), 100 nF (C3, C4, C6–C9), 1 nF (C10, C11), 10 µF (C1). Tin one pad,
   slide the part in with tweezers, then solder the other end.
5. **C2 tantalum.** Polarized: the stripe on the part goes to the side marked on the silkscreen (+3.3 V).
6. **Short test before anything else.** With the multimeter, check there is no short between 5V and GND,
   or between 3.3V and GND. It is much easier to fix now.
7. **C5 electrolytic** (100 µF/16 V, 6.3 mm or 5 mm diameter). Long leg (+) to the square pad, stripe (–) to the round pad.
8. **JST connectors J2–J11.** Push fully down; the outline on the silkscreen shows which way the housing faces.
9. **Mega headers.** Plug the headers into the Mega first, lay the shield on top, then solder from the top.
   This guarantees they line up with the Mega. Unplug the shield before the next steps.
10. **GPS breakout.** The CR1220 holder on the GPS's underside is taller than a header's 2.5 mm plastic spacer, so
    the GPS sits on 5 mm standoffs. Screw the two 5 mm nylon standoffs to the shield (holes at the far end from the
    header). Push the **short** pins of the 1x9 header up into the GPS from below (plastic spacer against the GPS),
    lower the long pins through the shield, screw the GPS to the standoffs, then solder on top of the GPS and under
    the shield. About 1.9 mm of pin comes through the shield. Antenna (ceramic patch) faces up.
11. **IMU breakout.** Nothing on its underside, so the headers' own 2.5 mm spacers set the height: short pins into
    the IMU (1x6 and 1x4), long pins through the shield, solder both ends. Nylon hardware only if you add screws
    (steel or brass near the magnetometer affects the heading). The +X arrow on the BNO055 must point along
    the cart's direction of travel when the board is mounted.
12. **Trim** the GPS and IMU header pins that stick out under the shield flush, so they cannot touch the Mega.
    Do **not** trim the Mega header pins: they are what plugs into the Mega.

JP1 needs no soldering: it ships bridged by a copper trace. Only cut it to move the IMU to I2C.

### First power-up

1. No cables plugged in. Connect the Mega to USB. Measure 5 V on any JST pin 2 (LCD/LUX) and 3.3 V on C4.
2. Feel U2, U1 and U3: none should be warm.
3. Change `gps_red = 4` and `gps_green = 7` in the sketch, upload, then plug in the LEDs, LCD, lux
   sensor and buttons. Insert the SD card last.
4. Expected at boot (Week 8): all three LEDs red; IMU turns green when the BNO055 answers, GPS turns yellow while it
   waits for a fix, LUX turns green when the TSL2591 answers.

### Other notes

- IMU orientation matters. Week 8's level check uses roll only (`eulY`, rotation about the BNO's Y axis), and the
  heading and forward acceleration (`linX`) assume the BNO's +X axis points along the cart's travel. Mount the board
  so the +X arrow printed on the BNO055 breakout points forward (the shield silkscreen says so). If the board has to be
  mounted 90° off, swap roll and pitch (and use `linY`) in the sketch instead.
- MISO comes back to D50 at 3.3 V without shifting, the same as the Adafruit SD breakout used today. The ATmega2560
  reads HIGH from 3.0 V, so the margin is small; real cards drive close to 3.3 V at this light load. If a card is
  flaky, try another brand before suspecting the board.
- Power budget from the Mega's 5 V rail: Mega about 80 mA, 20x4 LCD backlight about 100 mA, LEDs up to 80 mA (all
  yellow), SD card up to 100 mA while writing, GPS 30 mA, BNO055 12 mA, encoder and button lamps 20-60 mA:
  about 400-450 mA worst case. USB (500 mA fuse) or a 5 V supply into USB is fine. On the barrel jack use about 7 V:
  at 9 V the Mega's regulator dissipates about 1.8 W and will overheat.

## Design decisions / assumptions worth reviewing

- "Embedded Arduino" was implemented as a shield the Mega plugs into, not a bare ATmega2560 on
  the board. That keeps the USB bootloader, uses the Mega the team already has and is buildable
  by hand. A bare-chip version could follow once this one is proven.
- The IMU was placed on the board (not on a connector) because it must be rigidly fixed to the
  cart for the tilt check and it removes six jumper wires. If you want it remote, the header
  footprint accepts a cable instead.
- The Adafruit MicroSD breakout was replaced by an on-board socket and level shifter (its own
  circuit, copied). The Adafruit breakout is no longer needed.
- The cart-facing connectors are JST-XH because they lock and are cheap; the pin-order note is on
  the silkscreen next to every connector.
- Board outline, stacking-header positions and breakout footprints are from published board
  files, but nothing has been test-fitted physically. Print `docs/pcb_top.pdf` 1:1 and lay the
  Mega, the GPS and the IMU on it before ordering.

## Pre-order validation (final)

Automated: ERC 0, DRC 0 errors / 0 warnings, 0 unconnected, schematic-parity 0, netlist matches `generator/design.py` on all 42 nets.
`generator/audit.py` re-checks edge clearances, annular rings, decoupling distances, the IMU keepout and the LED mapping.

Against OSH Park's 2-layer rules: track 0.30 mm (min 0.152), clearance 0.20 mm (min 0.152), drill 0.40 mm (min 0.33),
annular ring 0.20 mm (min 0.178), copper to edge 0.40 mm (min 0.38). Gerber mask and drill files cross-checked against the board:
274 pad openings, 199 plated holes (147 pins + 52 vias), 13 unplated holes.
A second pass also ran DRC with every check enabled (only the expected Mega "no courtyard" and the 74LVC125 symbol's
DIP-only footprint filter came up), checked every net pin by pin against `design.py`, and checked the GPS, BNO055 and
Mega footprints against Adafruit's board files and KiCad's Arduino Mega template.

Check during the 1:1 paper test-fit, because only the physical parts can confirm them:
- the Mega's barrel jack clears the notch and the USB jack clears the left edge;
- the 6-pin ICSP header of the Mega's USB chip (near the USB jack) does not touch the underside (plain male headers give 2.5 mm more room; tape it if it touches);
- the GPS and IMU header pins are trimmed flush underneath so they cannot touch the Mega.

Not board issues, but decide before use:
- Power the Mega from USB (a USB power bank works) or about 7 V on the barrel jack; 9 V and above overheat its regulator at this load.
- Use nylon standoffs and screws near the IMU, not steel or brass.
- Long I2C cables to the LCD and lux sensors (over about 1 m) may need slower I2C (`Wire.setClock(50000)`).
- Arcade-button lamps are often rated 12 V; at 5 V they are dim but safe.
