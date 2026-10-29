"""Generate KiCad project: schematic (.kicad_sch), custom symbol lib, custom footprints, project files."""
import os, sys, uuid, json, shutil
import sexp
from sexp import Sym as S, findall, find
import design as D

OUT = sys.argv[1] if len(sys.argv) > 1 else "out"
os.makedirs(OUT, exist_ok=True)
os.makedirs(os.path.join(OUT, "LightMeter.pretty"), exist_ok=True)
SYMDIR = D.KLIB + "/symbols/"

def U(): return str(uuid.uuid4())
ROOT_UUID = U()

# ------------------------------------------------------------------ custom symbols
def rect_symbol(name, left, right, ref_prefix, value, footprint, desc, pin_len=5.08, width=30.48):
    """Generic box symbol. left/right: list of (number, name, type). Lib coords (y up)."""
    n = max(len(left), len(right))
    h = (n + 1) * 2.54
    top = h / 2
    body = [S("symbol"), name + "_0_1",
            [S("rectangle"), [S("start"), -width/2, top], [S("end"), width/2, -top],
             [S("stroke"), [S("width"), 0.254], [S("type"), S("default")]], [S("fill"), [S("type"), S("background")]]]]
    pins = [S("symbol"), name + "_1_1"]
    def mkpin(num, nm, typ, x, y, rot):
        return [S("pin"), S(typ), S("line"), [S("at"), x, y, rot], [S("length"), pin_len],
                [S("name"), nm, [S("effects"), [S("font"), [S("size"), 1.27, 1.27]]]],
                [S("number"), num, [S("effects"), [S("font"), [S("size"), 1.27, 1.27]]]]]
    for i, (num, nm, typ) in enumerate(left):
        y = top - 2.54 * (i + 1)
        pins.append(mkpin(num, nm, typ, -width/2 - pin_len, y, 0))
    for i, (num, nm, typ) in enumerate(right):
        y = top - 2.54 * (i + 1)
        pins.append(mkpin(num, nm, typ, width/2 + pin_len, y, 180))
    def prop(k, v, y, hide=False):
        e = [S("effects"), [S("font"), [S("size"), 1.27, 1.27]]]
        if hide: e.append([S("hide"), S("yes")])
        return [S("property"), k, v, [S("at"), 0, y, 0], e]
    sym = [S("symbol"), name, [S("pin_names"), [S("offset"), 1.016]], [S("exclude_from_sim"), S("no")],
           [S("in_bom"), S("yes")], [S("on_board"), S("yes")],
           prop("Reference", ref_prefix, top + 2.54), prop("Value", value, -top - 2.54),
           prop("Footprint", footprint, -top - 5.08, True), prop("Datasheet", "", 0, True),
           prop("Description", desc, 0, True), body, pins]
    return sym

CUSTOM = {
    "Arduino_Mega2560_Shield": rect_symbol("Arduino_Mega2560_Shield", D.MEGA_LEFT, D.MEGA_RIGHT, "A", "Arduino Mega 2560 R3",
                                           "LightMeter:Arduino_Mega2560_Shield", "Arduino Mega 2560 shield headers", width=40.64),
    "Adafruit_UltimateGPS_v3": rect_symbol("Adafruit_UltimateGPS_v3", D.GPS_PINS, [], "GPS", "Adafruit Ultimate GPS v3",
                                           "LightMeter:Adafruit_UltimateGPS_v3", "GPS breakout on 1x9 header", width=25.4),
    "Adafruit_BNO055_2472": rect_symbol("Adafruit_BNO055_2472", D.IMU_PINS, [], "IMU", "Adafruit BNO055 2472",
                                        "LightMeter:Adafruit_BNO055_2472", "BNO055 breakout (two headers)", width=25.4),
}
# custom symbol library file
lib = [S("kicad_symbol_lib"), [S("version"), 20231120], [S("generator"), "gen_sch"]] + list(CUSTOM.values())
open(os.path.join(OUT, "LightMeter.kicad_sym"), "w", encoding="utf-8").write(sexp.dump(lib) + "\n")

# ------------------------------------------------------------------ gather lib_symbols
import copy
lib_symbols = [S("lib_symbols")]
symdefs = {}   # "Lib:Name" -> symbol node (with sub-units)
def get_def(libname, name):
    key = libname + ":" + name
    if key in symdefs: return symdefs[key]
    if libname == "LightMeter":
        s = copy.deepcopy(CUSTOM[name])
    else:
        s = sexp.resolve_symbol(SYMDIR + libname + ".kicad_sym", name)
    s[1] = key
    symdefs[key] = s
    lib_symbols.append(s)
    return s
for ref, (libname, name, *_r) in D.PARTS.items(): get_def(libname, name)
for p in ("+5V", "GND", "+3V3", "PWR_FLAG"): get_def("power", p)

# ------------------------------------------------------------------ schematic layout
# ref -> (x, y) ; multi-unit U1 gets unit positions
POS = {
    "A1": (75, 165),
    "U2": (175, 40), "C1": (150, 48), "C2": (200, 48), "C5": (130, 48),
    "U1": {1: (175, 85), 2: (175, 105), 3: (175, 125), 4: (175, 145), 5: (225, 55)},
    "J1": (285, 100), "R7": (315, 60), "R8": (325, 60), "R9": (335, 60), "R10": (345, 60), "C3": (250, 48), "C4": (315, 130),
    "J2": (380, 45), "J3": (380, 65),
    "J4": (380, 95), "J5": (380, 115), "J6": (380, 135),
    "R1": (345, 100), "R2": (355, 100), "R3": (345, 120), "R4": (355, 120), "R5": (345, 140), "R6": (355, 140),
    "J7": (380, 170), "J8": (380, 190), "J9": (380, 210),
    "C6": (350, 175), "C7": (350, 195), "C8": (350, 215),
    "GPS1": (200, 190), "IMU1": (200, 240), "JP1": (250, 250),
    "H1": (300, 250), "H2": (315, 250),
    "U3": (480, 90), "C9": (445, 70), "R13": (445, 110), "R11": (515, 70), "R12": (525, 70), "J10": (560, 90),
    "R18": (445, 130), "R14": (465, 170), "R15": (475, 170), "R16": (490, 170), "R17": (500, 170), "C10": (515, 170), "C11": (525, 170), "J11": (560, 170),
}
PWRFLAG_POS = {"+5V": (120, 260), "GND": (140, 260)}

pin_to_net = {}
for net, pins in D.NETS.items():
    for rp in pins:
        assert rp not in pin_to_net, ("pin used twice", rp)
        pin_to_net[rp] = net

items = []          # schematic items (symbols, wires, labels, no_connects)
sym_uuids = {}      # ref -> uuid (unit 1) ; multiunit: ref -> {unit: uuid}
STUB = 5.08

def rot_pt(x, y, rot):
    if rot == 0: return x, y
    if rot == 90: return -y, x
    if rot == 180: return -x, -y
    if rot == 270: return y, -x

def snap(v): return round(round(v / 1.27) * 1.27, 4)

def place_symbol(ref, key, x, y, unit=1, rot=0, value=None, footprint="", show_value=True, desc=""):
    x, y = snap(x), snap(y)
    sd = symdefs[key]
    uid = U()
    pins = [p for p in sexp.symbol_pins(sd) if p["unit"] in (0, unit)]
    def prop(k, v, dx, dy, hide=False):
        e = [S("effects"), [S("font"), [S("size"), 1.27, 1.27]], [S("justify"), S("left")]]
        if hide: e.append([S("hide"), S("yes")])
        return [S("property"), k, v, [S("at"), x + dx, y + dy, 0], e]
    node = [S("symbol"), [S("lib_id"), key], [S("at"), x, y, rot], [S("unit"), unit],
            [S("exclude_from_sim"), S("no")], [S("in_bom"), S("no") if ref.startswith(("H", "JP")) else S("yes")], [S("on_board"), S("yes")], [S("dnp"), S("no")],
            [S("uuid"), uid],
            prop("Reference", ref, 2.54, -2.54 if unit == 1 else 0),
            prop("Value", value if value is not None else sd[1].split(":")[1], 2.54, 0, hide=not show_value),
            prop("Footprint", footprint, 0, 0, True), prop("Datasheet", "", 0, 0, True), prop("Description", desc, 0, 0, True)]
    for p in pins:
        node.append([S("pin"), p["number"], [S("uuid"), U()]])
    node.append([S("instances"), [S("project"), D.PROJECT, [S("path"), "/" + ROOT_UUID, [S("reference"), ref], [S("unit"), unit]]]])
    items.append(node)
    # pin connection points in schematic coords
    ends = {}
    for p in pins:
        px, py = rot_pt(p["x"], -p["y"], rot)   # lib y-up -> sch y-down
        # stub direction: away from body. lib rot: 0 body right, 180 body left, 90 body up, 270 body down
        r = (p["rot"] + rot) % 360
        dx, dy = {0: (-1, 0), 180: (1, 0), 90: (0, 1), 270: (0, -1)}[r]
        ends[p["number"]] = (round(x + px, 4), round(y + py, 4), dx, dy, p["type"])
    return uid, ends

def wire(x1, y1, x2, y2):
    items.append([S("wire"), [S("pts"), [S("xy"), x1, y1], [S("xy"), x2, y2]],
                  [S("stroke"), [S("width"), 0], [S("type"), S("default")]], [S("uuid"), U()]])

def glabel(net, x, y, dx, dy):
    ang = {(-1, 0): 180, (1, 0): 0, (0, 1): 270, (0, -1): 90}[(dx, dy)]
    just = S("right") if ang == 180 else S("left")
    items.append([S("global_label"), net, [S("shape"), S("passive")], [S("at"), x, y, ang], [S("fields_autoplaced"), S("yes")],
                  [S("effects"), [S("font"), [S("size"), 1.27, 1.27]], [S("justify"), just]], [S("uuid"), U()],
                  [S("property"), "Intersheetrefs", "${INTERSHEET_REFS}", [S("at"), x, y, 0],
                   [S("effects"), [S("font"), [S("size"), 1.27, 1.27]], [S("hide"), S("yes")]]]])

def power_symbol(net, x, y):
    key = "power:" + net
    uid, ends = place_symbol("#PWR" + str(len(items)), key, x, y, value=net, show_value=True)
    return uid

def noconnect(x, y):
    items.append([S("no_connect"), [S("at"), x, y], [S("uuid"), U()]])

def connect_pins(ref, ends):
    for num, (x, y, dx, dy, typ) in ends.items():
        net = pin_to_net.get((ref, num))
        if net is None:
            noconnect(x, y)
            continue
        ex, ey = x + dx * STUB, y + dy * STUB
        wire(x, y, ex, ey)
        if net in D.POWER_NETS:
            power_symbol(net, ex, ey)
        else:
            glabel(net, ex, ey, dx, dy)

for ref, (libname, name, value, footprint, desc) in D.PARTS.items():
    key = libname + ":" + name
    if isinstance(POS[ref], dict):
        sym_uuids[ref] = {}
        for unit, (x, y) in POS[ref].items():
            uid, ends = place_symbol(ref, key, x, y, unit=unit, value=value, footprint=footprint, desc=desc)
            sym_uuids[ref][unit] = uid
            connect_pins(ref, ends)
    else:
        x, y = POS[ref]
        rot = 0
        uid, ends = place_symbol(ref, key, x, y, rot=rot, value=value, footprint=footprint, desc=desc)
        sym_uuids[ref] = uid
        connect_pins(ref, ends)

# PWR_FLAGs on +5V and GND
for i, (net, (x, y)) in enumerate(PWRFLAG_POS.items()):
    x, y = snap(x), snap(y)
    place_symbol("#FLG%02d" % (i + 1), "power:PWR_FLAG", x, y, value="PWR_FLAG")
    wire(x, y, x, y + STUB)
    power_symbol(net, x, y + STUB)

# text notes
def text(s, x, y, size=2.0):
    items.append([S("text"), s, [S("exclude_from_sim"), S("no")], [S("at"), x, y, 0],
                  [S("effects"), [S("font"), [S("size"), size, size]], [S("justify"), S("left"), S("bottom")]], [S("uuid"), U()]])
text("EVEI LIGHT METER - Arduino Mega 2560 carrier shield", 20, 20, 3.5)
text("Pin map from latest_version_of_lightmeter.ino: SD CS=D53 (HW SPI 50/51/52), GPS=Serial1 (D18/D19), IMU=Serial2 (D16/D17), I2C=D20/D21,\n"
     "LEDs common-anode to 5V, active LOW. setLED(GREEN) pulls the *_red pin LOW, so *_red pins drive the GREEN die: GPS G=D4 R=D7, LUX G=D5 R=D6, IMU G=D8 R=D9, LUX D5/D6, IMU D8/D9; buttons to GND with INPUT_PULLUP: sleep D10, log toggle D11, force log D12.", 20, 28, 1.5)
text("microSD: on-board socket, 3.3V LDO, 74LVC125 shifts MOSI/SCK/CS 5V->3.3V; MISO (3.3V) goes straight to D50 (same as Adafruit 254 breakout).", 130, 20, 1.5)
text("IMU in UART mode: PS1 tied to breakout 3Vo via JP1. BNO SDA pin = UART TX -> Mega RX2 (D17); BNO SCL pin = UART RX -> Mega TX2 (D16).", 130, 165, 1.5)
text("REV B ADDITIONS", 430, 30, 3.0)
text("Second TSL2591 (J10): fixed I2C address 0x29 clashes with LUX1, so it sits behind PCA9548A (U3, addr 0x70) channel 0.\n"
     "Mux powers up with all channels off, so LUX1 on the main bus works with unchanged firmware. To read LUX2: write 0x01 to 0x70, read, write 0x00.", 430, 38, 1.5)
text("Encoder (J11): A -> D2 (INT4), B -> D3 (INT5). 10k pull-ups for open-collector encoders, 1k + 1nF filter and pin protection.\n"
     "GPS LED moved from D2/D3 to D4/D7 to free both interrupt pins: change gps_red = 4, gps_green = 7 in the sketch.", 430, 140, 1.5)
text("Connector pinouts (JST-XH): LCD/LUX = GND,5V,SDA,SCL   LEDx = 5V(common anode),RED,GREEN   BTNx = GND,SW,5V(lamp)", 300, 35, 1.5)

sch = [S("kicad_sch"), [S("version"), 20231120], [S("generator"), "gen_sch"], [S("generator_version"), "8.0"],
       [S("uuid"), ROOT_UUID], [S("paper"), "A2"],
       [S("title_block"), [S("title"), "EVEI Light Meter - Mega 2560 Shield"], [S("date"), "2026-10-29"], [S("rev"), "B"],
        [S("company"), "Purdue EVEI"]],
       lib_symbols] + items + [[S("sheet_instances"), [S("path"), "/", [S("page"), "1"]]]]
open(os.path.join(OUT, D.PROJECT + ".kicad_sch"), "w", encoding="utf-8").write(sexp.dump(sch) + "\n")

# ------------------------------------------------------------------ custom footprints
def fp_header(name, desc, pads, holes, outline, texts, ref_at, val_at):
    """pads: (num, x, y); holes: (x, y, drill); outline: (x0,y0,x1,y1) rect on silk+fab."""
    def prop(k, v, x, y, layer, hide=False):
        e = [S("effects"), [S("font"), [S("size"), 1, 1], [S("thickness"), 0.15]]]
        n = [S("property"), k, v, [S("at"), x, y, 0], [S("layer"), layer]]
        if hide: n.append([S("hide"), S("yes")])
        n += [[S("uuid"), U()], e]
        return n
    fp = [S("footprint"), name, [S("version"), 20240108], [S("generator"), "gen_sch"], [S("layer"), "F.Cu"],
          [S("descr"), desc], [S("tags"), "adafruit breakout"],
          prop("Reference", "REF**", ref_at[0], ref_at[1], "F.SilkS"), prop("Value", name, val_at[0], val_at[1], "F.Fab"),
          prop("Footprint", "", 0, 0, "F.Fab", True), prop("Datasheet", "", 0, 0, "F.Fab", True), prop("Description", "", 0, 0, "F.Fab", True),
          [S("attr"), S("through_hole")]]
    x0, y0, x1, y1 = outline
    for layer, w in (("F.SilkS", 0.15), ("F.Fab", 0.1)):
        fp.append([S("fp_rect"), [S("start"), x0, y0], [S("end"), x1, y1], [S("stroke"), [S("width"), w], [S("type"), S("default")]],
                   [S("fill"), S("none")], [S("layer"), layer], [S("uuid"), U()]])
    fp.append([S("fp_rect"), [S("start"), x0 - 0.5, y0 - 0.5], [S("end"), x1 + 0.5, y1 + 0.5], [S("stroke"), [S("width"), 0.05], [S("type"), S("default")]],
               [S("fill"), S("none")], [S("layer"), "F.CrtYd"], [S("uuid"), U()]])
    for t, x, y, sz in texts:
        fp.append([S("fp_text"), S("user"), t, [S("at"), x, y, 0], [S("layer"), "F.SilkS"], [S("uuid"), U()],
                   [S("effects"), [S("font"), [S("size"), sz, sz], [S("thickness"), 0.15]]]])
    for i, (num, x, y) in enumerate(pads):
        shape = S("rect") if i == 0 else S("circle")
        fp.append([S("pad"), num, S("thru_hole"), shape, [S("at"), x, y], [S("size"), 1.7, 1.7], [S("drill"), 1.0],
                   [S("layers"), "*.Cu", "*.Mask"], [S("remove_unused_layers"), S("no")], [S("uuid"), U()]])
    for x, y, d in holes:
        fp.append([S("pad"), "", S("np_thru_hole"), S("circle"), [S("at"), x, y], [S("size"), d, d], [S("drill"), d],
                   [S("layers"), "*.Cu", "*.Mask"], [S("uuid"), U()]])
    return fp

# GPS v3: board 25.4 x 34.29, header along bottom edge at y=2.032 (board coords, y up), pin1 PPS at x=22.86 ... 3V3 at 2.54
# footprint origin = board centre. KiCad y down => board y -> -(y - 17.145)
def gps_xy(bx, by): return round(bx - 12.7, 3), round(-(by - 17.145), 3)
gps_pads = []
for i, (num, *_r) in enumerate(D.GPS_PINS):
    gps_pads.append((num, *gps_xy(22.86 - 2.54 * i, 2.032)))
gps_holes = [(*gps_xy(2.54, 31.75), 2.7), (*gps_xy(22.86, 31.75), 2.7)]
gps_fp = fp_header("Adafruit_UltimateGPS_v3", "Adafruit Ultimate GPS Breakout v3 (PID 746) soldered on 1x9 0.1in header; M2.5 holes",
                   gps_pads, gps_holes, (-12.7, -17.145, 12.7, 17.145),
                   [("GPS v3  antenna UP", 0, -6, 1.0), ("PPS", 10.16, 12.6, 0.8), ("3V3", -10.16, 12.6, 0.8)], (0, -19), (0, 19))
open(os.path.join(OUT, "LightMeter.pretty", "Adafruit_UltimateGPS_v3.kicad_mod"), "w", encoding="utf-8").write(sexp.dump(gps_fp) + "\n")

# BNO055 2472: board 26.67 x 20.32; JP1 1x6 at y=1.27 x=6.985..19.685 (VIN,3VO,GND,SDA,SCL,RST); JP2 1x4 at y=19.05 x=9.525..17.145 (PS0,PS1,INT,ADR)
def imu_xy(bx, by): return round(bx - 13.335, 3), round(-(by - 10.16), 3)
imu_pads = []
for i, num in enumerate(["VIN", "3VO", "GND", "SDA", "SCL", "RST"]):
    imu_pads.append((num, *imu_xy(6.985 + 2.54 * i, 1.27)))
for i, num in enumerate(["PS0", "PS1", "INT", "ADR"]):
    imu_pads.append((num, *imu_xy(9.525 + 2.54 * i, 19.05)))
imu_holes = [(*imu_xy(x, y), 2.7) for x, y in ((2.54, 2.54), (2.54, 17.78), (24.13, 2.54), (24.13, 17.78))]
imu_fp = fp_header("Adafruit_BNO055_2472", "Adafruit BNO055 breakout (PID 2472, non-STEMMA) soldered on 1x6 + 1x4 headers; M2.5 holes",
                   imu_pads, imu_holes, (-13.335, -10.16, 13.335, 10.16),
                   [("BNO055", 0, 0, 1.0), ("VIN", -6.35, 6.4, 0.8), ("RST", 6.35, 6.4, 0.8), ("PS0", -3.81, -6.4, 0.8), ("ADR", 3.81, -6.4, 0.8)],
                   (0, -12), (0, 12))
open(os.path.join(OUT, "LightMeter.pretty", "Adafruit_BNO055_2472.kicad_mod"), "w", encoding="utf-8").write(sexp.dump(imu_fp) + "\n")

# Mega shield footprint (speeduino, legacy format) -> copy; build script re-layers its silk outline
shutil.copy("mega_shield_speeduino.kicad_mod", os.path.join(OUT, "LightMeter.pretty", "Arduino_Mega2560_Shield.kicad_mod"))

# ------------------------------------------------------------------ project + lib tables
open(os.path.join(OUT, "fp-lib-table"), "w").write('(fp_lib_table\n  (version 7)\n  (lib (name "LightMeter")(type "KiCad")(uri "${KIPRJMOD}/LightMeter.pretty")(options "")(descr "project footprints"))\n)\n')
open(os.path.join(OUT, "sym-lib-table"), "w").write('(sym_lib_table\n  (version 7)\n  (lib (name "LightMeter")(type "KiCad")(uri "${KIPRJMOD}/LightMeter.kicad_sym")(options "")(descr "project symbols"))\n)\n')

pro = {
    "board": {"design_settings": {
        "rules": {"min_clearance": 0.2, "min_track_width": 0.2, "min_via_diameter": 0.6, "min_via_annular_width": 0.15,
                  "min_hole_clearance": 0.25, "min_copper_edge_clearance": 0.4, "min_through_hole_diameter": 0.3, "min_silk_clearance": 0.0}},
        "layer_presets": [], "viewports": []},
    "meta": {"filename": D.PROJECT + ".kicad_pro", "version": 1},
    "net_settings": {
        "classes": [
            {"name": "Default", "clearance": 0.2, "track_width": 0.3, "via_diameter": 0.8, "via_drill": 0.4,
             "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25, "diff_pair_width": 0.2, "microvia_diameter": 0.3,
             "microvia_drill": 0.1, "wire_width": 6, "bus_width": 12, "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
             "schematic_color": "rgba(0, 0, 0, 0.000)", "priority": 2147483647},
            {"name": "Power", "clearance": 0.25, "track_width": 0.6, "via_diameter": 1.0, "via_drill": 0.5,
             "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25, "diff_pair_width": 0.2, "microvia_diameter": 0.3,
             "microvia_drill": 0.1, "wire_width": 6, "bus_width": 12, "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
             "schematic_color": "rgba(0, 0, 0, 0.000)", "priority": 0},
        ],
        "meta": {"version": 4},
        "netclass_assignments": {},
        "netclass_patterns": [{"netclass": "Power", "pattern": "+5V"}, {"netclass": "Power", "pattern": "GND"}, {"netclass": "Power", "pattern": "+3V3"}],
    },
    "schematic": {"legacy_lib_dir": "", "legacy_lib_list": []},
    "sheets": [[ROOT_UUID, "Root"]],
    "text_variables": {},
}
json.dump(pro, open(os.path.join(OUT, D.PROJECT + ".kicad_pro"), "w"), indent=2)
json.dump({"root_uuid": ROOT_UUID, "sym_uuids": sym_uuids}, open(os.path.join(OUT, "uuids.json"), "w"), indent=1)
print("wrote", OUT, "items:", len(items))
