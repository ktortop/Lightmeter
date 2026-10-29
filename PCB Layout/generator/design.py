"""Single source of truth for the EVEI Light Meter shield: parts, nets, PCB placement."""

PROJECT = "LightMeter_MegaShield"
KLIB = "C:/Program Files/KiCad/10.0/share/kicad"

# ---------------------------------------------------------------- Mega pins
# pad names of the Arduino_Mega2560_Shield footprint (speeduino) -> function
MEGA_LEFT = [  # (pad, display name, type)
    ("5V1", "5V (pwr hdr)", "power_in"), ("5V3", "5V (2x18 hdr)", "power_in"), ("5V4", "5V (2x18 hdr)", "power_in"),
    ("3V3", "3V3", "power_out"), ("VIN", "VIN", "power_in"), ("IORF", "IOREF", "power_out"),
    ("GND1", "GND", "power_in"), ("GND2", "GND", "power_in"), ("GND3", "GND", "power_in"),
    ("GND5", "GND", "power_in"), ("GND6", "GND", "power_in"),
    ("RST1", "RESET", "input"), ("AREF", "AREF", "input"),
] + [("A%d" % i, "A%d" % i, "bidirectional") for i in range(16)]

_dnames = {0: "D0/RX0", 1: "D1/TX0", 14: "D14/TX3", 15: "D15/RX3", 16: "D16/TX2", 17: "D17/RX2",
           18: "D18/TX1", 19: "D19/RX1", 20: "D20/SDA", 21: "D21/SCL", 50: "D50/MISO", 51: "D51/MOSI",
           52: "D52/SCK", 53: "D53/SS"}
MEGA_RIGHT = [("D%d" % i, _dnames.get(i, "D%d" % i), "bidirectional") for i in range(54)] + [
    ("SCL", "SCL (hdr dup)", "bidirectional"), ("SDA", "SDA (hdr dup)", "bidirectional"),
    ("MISO", "ICSP MISO", "bidirectional"), ("MOSI", "ICSP MOSI", "bidirectional"), ("SCK", "ICSP SCK", "bidirectional"),
    ("RST2", "ICSP RESET", "input"), ("5V2", "ICSP 5V", "power_in"), ("GND4", "ICSP GND", "power_in"),
]

GPS_PINS = [("PPS", "PPS", "output"), ("VIN", "VIN", "power_in"), ("GND", "GND", "power_in"), ("RX", "RX", "input"),
            ("TX", "TX", "output"), ("FIX", "FIX", "output"), ("VBAT", "VBAT", "passive"), ("EN", "EN", "input"),
            ("3V3", "3.3V out", "power_out")]
IMU_PINS = [("VIN", "VIN", "power_in"), ("3VO", "3Vo out", "power_out"), ("GND", "GND", "power_in"),
            ("SDA", "SDA (UART TX)", "bidirectional"), ("SCL", "SCL (UART RX)", "bidirectional"), ("RST", "RST", "input"),
            ("PS0", "PS0", "input"), ("PS1", "PS1", "input"), ("INT", "INT", "output"), ("ADR", "ADR", "input")]

# ---------------------------------------------------------------- parts
# ref: (lib, symbol, value, footprint-lib:footprint, description)
PARTS = {
    "A1":  ("LightMeter", "Arduino_Mega2560_Shield", "Arduino Mega 2560 R3", "LightMeter:Arduino_Mega2560_Shield", "Arduino Mega 2560 (shield headers, board plugs on top)"),
    "GPS1": ("LightMeter", "Adafruit_UltimateGPS_v3", "Adafruit Ultimate GPS v3 (746)", "LightMeter:Adafruit_UltimateGPS_v3", "GPS breakout soldered to board via 1x9 header"),
    "IMU1": ("LightMeter", "Adafruit_BNO055_2472", "Adafruit BNO055 (2472)", "LightMeter:Adafruit_BNO055_2472", "IMU breakout soldered to board, UART mode"),
    "U1":  ("74xx", "74LVC125", "74LVC125A", "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm", "5V->3.3V level shifter for SD MOSI/SCK/CS"),
    "U2":  ("Regulator_Linear", "AMS1117-3.3", "AMS1117-3.3", "Package_TO_SOT_SMD:SOT-223-3_TabPin2", "3.3V LDO for microSD"),
    "J1":  ("Connector", "Micro_SD_Card_Det_Hirose_DM3AT", "microSD DM3AT-SF-PEJM5", "Connector_Card:microSD_HC_Hirose_DM3AT-SF-PEJM5", "microSD socket, push-push"),
    "J2":  ("Connector", "Conn_01x04_Pin", "LCD 20x4 I2C", "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical", "JST-XH 4p: GND 5V SDA SCL"),
    "J3":  ("Connector", "Conn_01x04_Pin", "LUX1 TSL2591", "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical", "JST-XH 4p: GND 5V SDA SCL"),
    "J4":  ("Connector", "Conn_01x03_Pin", "LED1 GPS", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", "JST-XH 3p: GND(common cathode) R G"),
    "J5":  ("Connector", "Conn_01x03_Pin", "LED2 LUX", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", "JST-XH 3p: GND(common cathode) R G"),
    "J6":  ("Connector", "Conn_01x03_Pin", "LED3 IMU", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", "JST-XH 3p: GND(common cathode) R G"),
    "J7":  ("Connector", "Conn_01x03_Pin", "BTN SLEEP", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", "JST-XH 3p: GND SW 5V(lamp)"),
    "J8":  ("Connector", "Conn_01x03_Pin", "BTN LOG TOGGLE", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", "JST-XH 3p: GND SW 5V(lamp)"),
    "J9":  ("Connector", "Conn_01x03_Pin", "BTN FORCE LOG", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", "JST-XH 3p: GND SW 5V(lamp)"),
    "R1":  ("Device", "R", "220", "Resistor_SMD:R_0805_2012Metric", "LED1 red series"),
    "R2":  ("Device", "R", "220", "Resistor_SMD:R_0805_2012Metric", "LED1 green series"),
    "R3":  ("Device", "R", "220", "Resistor_SMD:R_0805_2012Metric", "LED2 red series"),
    "R4":  ("Device", "R", "220", "Resistor_SMD:R_0805_2012Metric", "LED2 green series"),
    "R5":  ("Device", "R", "220", "Resistor_SMD:R_0805_2012Metric", "LED3 red series"),
    "R6":  ("Device", "R", "220", "Resistor_SMD:R_0805_2012Metric", "LED3 green series"),
    "R7":  ("Device", "R", "10k", "Resistor_SMD:R_0805_2012Metric", "SD CS pull-up"),
    "R8":  ("Device", "R", "10k", "Resistor_SMD:R_0805_2012Metric", "SD DAT0/MISO pull-up"),
    "R9":  ("Device", "R", "10k", "Resistor_SMD:R_0805_2012Metric", "SD DAT1 pull-up"),
    "R10": ("Device", "R", "10k", "Resistor_SMD:R_0805_2012Metric", "SD DAT2 pull-up"),
    "C1":  ("Device", "C", "10uF", "Capacitor_SMD:C_0805_2012Metric", "LDO input"),
    "C2":  ("Device", "C_Polarized", "22uF tantalum 10V", "Capacitor_Tantalum_SMD:CP_EIA-3528-21_Kemet-B", "LDO output; tantalum ESR keeps AMS1117 stable"),
    "C3":  ("Device", "C", "100nF", "Capacitor_SMD:C_0805_2012Metric", "U1 decoupling"),
    "C4":  ("Device", "C", "100nF", "Capacitor_SMD:C_0805_2012Metric", "SD VDD decoupling"),
    "C5":  ("Device", "C_Polarized", "100uF/16V", "Capacitor_THT:CP_Radial_D5.0mm_P2.00mm", "5V bulk"),
    "C6":  ("Device", "C", "100nF", "Capacitor_SMD:C_0805_2012Metric", "BTN sleep debounce"),
    "C7":  ("Device", "C", "100nF", "Capacitor_SMD:C_0805_2012Metric", "BTN log debounce"),
    "C8":  ("Device", "C", "100nF", "Capacitor_SMD:C_0805_2012Metric", "BTN force debounce"),
    "JP1": ("Jumper", "SolderJumper_2_Bridged", "PS1=3V3 (UART)", "Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm", "cut to leave IMU in I2C mode"),
    "J10": ("Connector", "Conn_01x04_Pin", "LUX2 TSL2591", "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical", "JST-XH 4p: GND 5V SDA SCL (behind I2C mux ch0)"),
    "J11": ("Connector", "Conn_01x04_Pin", "ENCODER", "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical", "JST-XH 4p: GND 5V A B, quadrature encoder"),
    "U3":  ("Interface_Expansion", "PCA9548ADW", "PCA9548AD (SOIC-24W)", "Package_SO:SOIC-24W_7.5x15.4mm_P1.27mm", "I2C mux, addr 0x70; ch0 = second TSL2591 (same 0x29 address)"),
    "R11": ("Device", "R", "10k", "Resistor_SMD:R_0805_2012Metric", "LUX2 SDA pull-up"),
    "R12": ("Device", "R", "10k", "Resistor_SMD:R_0805_2012Metric", "LUX2 SCL pull-up"),
    "R13": ("Device", "R", "10k", "Resistor_SMD:R_0805_2012Metric", "mux RESET pull-up"),
    "R14": ("Device", "R", "10k", "Resistor_SMD:R_0805_2012Metric", "encoder A pull-up (open-collector outputs)"),
    "R15": ("Device", "R", "10k", "Resistor_SMD:R_0805_2012Metric", "encoder B pull-up"),
    "R16": ("Device", "R", "1k", "Resistor_SMD:R_0805_2012Metric", "encoder A series (filter + pin protection)"),
    "R17": ("Device", "R", "1k", "Resistor_SMD:R_0805_2012Metric", "encoder B series"),
    "R18": ("Device", "R", "10k", "Resistor_SMD:R_0805_2012Metric", "SD CS pull-up, Mega side (card deselected during reset)"),
    "C9":  ("Device", "C", "100nF", "Capacitor_SMD:C_0805_2012Metric", "U3 decoupling"),
    "C10": ("Device", "C", "1nF", "Capacitor_SMD:C_0805_2012Metric", "encoder A filter, 1 us with R16"),
    "C11": ("Device", "C", "1nF", "Capacitor_SMD:C_0805_2012Metric", "encoder B filter"),
    "H1":  ("Mechanical", "MountingHole", "M3", "MountingHole:MountingHole_3.2mm_M3", ""),
    "H2":  ("Mechanical", "MountingHole", "M3", "MountingHole:MountingHole_3.2mm_M3", ""),
}

# ---------------------------------------------------------------- nets
# net name -> list of (ref, pin)
NETS = {
    "+5V": [("A1", "5V1"), ("A1", "5V3"), ("A1", "5V4"), ("U2", "3"), ("C1", "1"), ("C5", "1"),
            ("J2", "2"), ("J3", "2"), ("J7", "3"), ("J8", "3"), ("J9", "3"),
            ("GPS1", "VIN"), ("IMU1", "VIN"),
            ("J10", "2"), ("J11", "2"), ("U3", "24"), ("C9", "1"), ("R11", "1"), ("R12", "1"), ("R13", "1"), ("R14", "1"), ("R15", "1"), ("R18", "1")],
    "GND": [("A1", "GND1"), ("A1", "GND2"), ("A1", "GND3"), ("A1", "GND5"), ("A1", "GND6"),
            ("U1", "7"), ("U1", "1"), ("U1", "4"), ("U1", "10"), ("U1", "13"), ("U1", "12"),
            ("U2", "1"), ("C1", "2"), ("C2", "2"), ("C3", "2"), ("C4", "2"), ("C5", "2"), ("C6", "2"), ("C7", "2"), ("C8", "2"),
            ("J1", "6"), ("J1", "SH"), ("J1", "9"),
            ("J2", "1"), ("J3", "1"), ("J4", "1"), ("J5", "1"), ("J6", "1"), ("J7", "1"), ("J8", "1"), ("J9", "1"), ("GPS1", "GND"), ("IMU1", "GND"),
            ("J10", "1"), ("J11", "1"), ("U3", "12"), ("U3", "1"), ("U3", "2"), ("U3", "21"), ("C9", "2"), ("C10", "2"), ("C11", "2")],
    "+3V3": [("U2", "2"), ("C2", "1"), ("U1", "14"), ("C3", "1"), ("J1", "4"), ("C4", "1"),
             ("R7", "1"), ("R8", "1"), ("R9", "1"), ("R10", "1")],
    "SDA": [("A1", "D20"), ("J2", "3"), ("J3", "3"), ("U3", "23")],
    "SCL": [("A1", "D21"), ("J2", "4"), ("J3", "4"), ("U3", "22")],
    "GPS_TX": [("GPS1", "TX"), ("A1", "D19")],
    "GPS_RX": [("GPS1", "RX"), ("A1", "D18")],
    "IMU_TX": [("IMU1", "SDA"), ("A1", "D17")],
    "IMU_RX": [("IMU1", "SCL"), ("A1", "D16")],
    "IMU_3V3": [("IMU1", "3VO"), ("JP1", "1")],
    "IMU_PS1": [("IMU1", "PS1"), ("JP1", "2")],
    "SPI_MOSI": [("A1", "D51"), ("U1", "2")],
    "SPI_SCK": [("A1", "D52"), ("U1", "5")],
    "SD_CS_5V": [("A1", "D53"), ("U1", "9"), ("R18", "2")],
    "SD_DI": [("U1", "3"), ("J1", "3")],
    "SD_CLK": [("U1", "6"), ("J1", "5")],
    "SD_CS": [("U1", "8"), ("J1", "2"), ("R7", "2")],
    "SPI_MISO": [("J1", "7"), ("A1", "D50"), ("R8", "2")],
    "SD_DAT1": [("J1", "8"), ("R9", "2")],
    "SD_DAT2": [("J1", "1"), ("R10", "2")],
    "LED1_R": [("J4", "2"), ("R1", "1")], "LED1_R_MCU": [("R1", "2"), ("A1", "D4")],
    "LED1_G": [("J4", "3"), ("R2", "1")], "LED1_G_MCU": [("R2", "2"), ("A1", "D7")],
    "LED2_R": [("J5", "2"), ("R3", "1")], "LED2_R_MCU": [("R3", "2"), ("A1", "D5")],
    "LED2_G": [("J5", "3"), ("R4", "1")], "LED2_G_MCU": [("R4", "2"), ("A1", "D6")],
    "LED3_R": [("J6", "2"), ("R5", "1")], "LED3_R_MCU": [("R5", "2"), ("A1", "D8")],
    "LED3_G": [("J6", "3"), ("R6", "1")], "LED3_G_MCU": [("R6", "2"), ("A1", "D9")],
    "BTN_SLEEP": [("J7", "2"), ("C6", "1"), ("A1", "D10")],
    "BTN_LOG": [("J8", "2"), ("C7", "1"), ("A1", "D11")],
    "BTN_FORCE": [("J9", "2"), ("C8", "1"), ("A1", "D12")],
    # rev B: second lux sensor behind TCA9548A channel 0
    "MUX_RST": [("U3", "3"), ("R13", "2")],
    "LUX2_SDA": [("U3", "4"), ("J10", "3"), ("R11", "2")],
    "LUX2_SCL": [("U3", "5"), ("J10", "4"), ("R12", "2")],
    # rev B: quadrature encoder on external interrupts INT4 (D2) / INT5 (D3)
    "ENC_A": [("J11", "3"), ("R14", "2"), ("R16", "1")],
    "ENC_B": [("J11", "4"), ("R15", "2"), ("R17", "1")],
    "ENC_A_MCU": [("R16", "2"), ("C10", "1"), ("A1", "D2")],
    "ENC_B_MCU": [("R17", "2"), ("C11", "1"), ("A1", "D3")],
}
POWER_NETS = ("+5V", "GND", "+3V3")

# ---------------------------------------------------------------- PCB placement (mm, KiCad coords: +y down)
# Mega footprint at origin: its bottom edge y=0, top edge y=-53.34, x 0..101.6
BOARD = dict(x0=11.0, x1=101.6, y0=-102.0, y1=0.0, notch_x=12.9, notch_y=-14.0)  # notch clears the Mega barrel jack
PLACE = {  # ref: (x, y, rot)
    "A1": (0, 0, 0),
    "GPS1": (27.0, -27.5, 0),
    "IMU1": (51.5, -27.0, 90),
    "R1": (76.0, -9.0, 90), "R2": (76.0, -12.5, 90), "R3": (76.0, -16.0, 90),
    "R4": (76.0, -19.5, 90), "R5": (76.0, -23.0, 90), "R6": (76.0, -26.5, 90),
    "C6": (84.0, -9.0, 90), "C7": (84.0, -12.5, 90), "C8": (84.0, -16.0, 90),
    "JP1": (84.0, -22.0, 0),
    # extension strip (above the Mega's top header row)
    "J2": (16.5, -60.0, 0), "J3": (30.5, -60.0, 0), "J4": (44.5, -60.0, 0), "J5": (56.0, -60.0, 0), "J6": (67.5, -60.0, 0),
    "J7": (16.5, -76.0, 0), "J8": (28.5, -76.0, 0), "J9": (40.5, -76.0, 0),
    "J1": (89.0, -93.0, 180),
    "U1": (88.0, -71.5, 270),
    "C3": (95.0, -71.5, 90),
    "U2": (58.0, -80.0, 0), "C1": (52.0, -74.5, 0), "C2": (64.5, -80.0, 90),
    "C5": (48.0, -82.0, 0),
    "C4": (89.5, -80.5, 90),
    "R7": (87.0, -80.5, 90), "R8": (92.5, -80.5, 90), "R9": (95.0, -80.5, 90), "R10": (84.5, -80.5, 90),
    # extra mounting holes (Mega holes are part of the A1 footprint)
    "H1": (15.0, -98.5, 0), "H2": (98.0, -60.0, 0),
    # rev B top row: second lux port, encoder port, I2C mux
    "J10": (22.0, -94.0, 0), "J11": (36.0, -94.0, 0),
    "R14": (49.5, -90.0, 90), "R15": (49.5, -94.0, 90), "R16": (53.0, -90.0, 90), "R17": (53.0, -94.0, 90),
    "C10": (56.5, -90.0, 90), "C11": (56.5, -94.0, 90),
    "U3": (66.0, -95.0, 90),
    "R11": (61.0, -87.0, 0), "R12": (65.0, -87.0, 0), "R13": (69.0, -87.0, 0), "C9": (56.5, -98.0, 90), "R18": (85.5, -65.5, 90),
}
