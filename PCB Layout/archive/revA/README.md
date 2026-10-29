# EVEI Light Meter – Arduino Mega 2560 shield (rev A, preliminary)

Two-layer carrier shield that replaces the hand-wired proto shield. Generated from
`latest_version_of_lightmeter.ino` (pin map) and the Final Design Review slides (part list, wiring).
The sketch was not modified; the board is pin-compatible with it as-is.

Board: 90.7 x 88.1 mm, 2 layers, 1 oz copper, min track 0.3 mm (signals) / 0.6 mm (power), vias 0.8/0.4 mm.
ERC: 0 violations. DRC: 0 errors, 0 warnings, 0 unconnected (schematic parity checked).

## Folder contents

| Path | What |
|---|---|
| `kicad/` | KiCad 10 project: schematic, PCB, project-local symbol + footprint libs. Open `LightMeter_MegaShield.kicad_pro`. |
| `docs/schematic.pdf` | Schematic (A3, single sheet). |
| `docs/render_top.png`, `render_bottom.png`, `render_3d.png` | 3D renders. |
| `docs/pcb_top.pdf`, `pcb_bottom.pdf` | Copper + silk plots. |
| `docs/BOM.csv` | Bill of materials (grouped). |
| `docs/LightMeter_MegaShield.step` | 3D model for the cart CAD. |
| `fab/LightMeter_MegaShield_gerbers.zip` | Gerbers + Excellon drills, ready for JLCPCB/PCBWay/OSH Park. `-pos.csv` is the SMD pick-and-place file. |
| `generator/` | Python scripts that generated everything (design.py holds the netlist and placement). Re-run if you change a pin. |

## How the Mega is "embedded"

The Arduino Mega 2560 plugs into the underside of this board through the standard Mega
stacking-header pattern (all 98 pads incl. the 2x18 header, exact coordinates from the
Arduino Mega drawing). The four Mega mounting holes are duplicated on the shield so the two
boards can be bolted together with M3 standoffs. The board is wider than the Mega on top (a
35 mm extension above the top header row) where all the cable connectors and the microSD live,
so no cable crosses the Mega. The left edge stops at x = 11 mm to clear the USB-B and barrel jack.

Buy an "Arduino Mega stacking header set" (1x10, 5x 1x8, 1x 2x18). The ICSP pads are on the
footprint but are left unconnected; do not fit a header there.

## What is on the board vs. on a cable

| Function | How | Mega pins (from the sketch) |
|---|---|---|
| Adafruit Ultimate GPS v3 (746) | Soldered onto the board on a 1x9 header (pin order PPS·VIN·GND·RX·TX·FIX·VBAT·EN·3.3V verified from Adafruit's board file) + two M2.5 holes | Serial1: D18→GPS RX, D19←GPS TX |
| microSD | Real socket on the board (Hirose DM3AT push-push), own AMS1117-3.3 regulator, 74LVC125A level shifter on MOSI/SCK/CS, 10k pull-ups on CS/DAT0/DAT1/DAT2 | HW SPI D50/51/52, CS D53 |
| Adafruit BNO055 (2472, non-STEMMA) | Soldered onto the board (1x6 + 1x4 headers, positions from Adafruit's board file) + four M2.5 holes. UART mode: PS1 tied to breakout 3Vo through solder jumper JP1 (bridged by default) | Serial2: D16→BNO SCL pin (UART RX), D17←BNO SDA pin (UART TX) |
| 20x4 LCD (I2C backpack) | J2, JST-XH 4 pin: GND · 5V · SDA · SCL | D20/D21 |
| TSL2591 lux sensor | J3, JST-XH 4 pin: GND · 5V · SDA · SCL (same pinout as LCD, cables interchangeable) | D20/D21 |
| Status LED 1 (GPS) | J4, JST-XH 3 pin: 5V (common anode) · RED · GREEN, 220 Ω series resistors on board | D2 (red), D3 (green) |
| Status LED 2 (LUX) | J5, same | D5, D6 |
| Status LED 3 (IMU) | J6, same | D8, D9 |
| Sleep button | J7, JST-XH 3 pin: GND · SW · 5V (5V for the arcade-button lamp, optional). 100 nF debounce cap on board, uses INPUT_PULLUP | D10 |
| Log-toggle button | J8, same | D11 |
| Force-log button | J9, same | D12 |

LED polarity matches `setLED()`: outputs LOW = LED on, both LOW = yellow, both HIGH = off.
Buttons go to GND (active LOW), matching `digitalRead(...) == LOW`.

## Assembly notes

1. GPS breakout: the CR1220 holder is on its back, so mount it on 4–5 mm M2.5 standoffs and use
   a tall (or socket + pin) header. Nothing tall is placed under it. Antenna faces up; the board
   sits horizontal on the cart, so the patch antenna sees the sky.
2. IMU is rotated 90° on the board (long side vertical). Its axes are printed on the breakout; the
   sketch uses Euler heading/roll/pitch, so mount the whole shield level and re-check which BNO
   axis is "forward" when you first test.
3. JP1: leave bridged for UART mode (current firmware). Cut it and use SDA/SCL if you ever move
   the IMU back to I2C.
4. Solder order: SMD (U1, U2, 0805 R/C, JP1) → microSD socket → C5 → JST connectors → stacking
   headers → GPS/IMU breakouts last.
5. Power: everything runs from the Mega's 5V rail (~300 mA worst case incl. LEDs and lamps).
   Power the Mega from USB/5 V power bank or ≤ 9 V on the barrel jack to keep its regulator cool.
6. MISO comes back to D50 at 3.3 V logic without shifting, same as the Adafruit "5V-ready"
   breakout the team uses today. It has worked on the prototype; if SD reads ever get flaky, the
   spare 74LVC125 gate is on board but powered at 3.3 V, so the fix would be a 5V-supplied buffer.

## Design decisions / assumptions worth reviewing

- "Embedded Arduino" was implemented as a shield the Mega plugs into, not a bare ATmega2560 on
  the board. That keeps the USB bootloader, uses the Mega the team already has and is buildable
  by hand. A bare-chip version would be rev B once this one is proven.
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
