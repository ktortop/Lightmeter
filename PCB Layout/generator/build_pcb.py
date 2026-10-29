"""Build the PCB from the exported netlist + design placement. Run with KiCad's python.exe.
usage: python build_pcb.py <projdir> [route]
"""
import os, sys, json, subprocess, shutil
import xml.etree.ElementTree as ET
import pcbnew
from pcbnew import VECTOR2I, VECTOR2I_MM, FromMM

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import design as D

OUT = os.path.abspath(sys.argv[1])
DO_ROUTE = len(sys.argv) > 2 and sys.argv[2] == "route"
PCB = os.path.join(OUT, D.PROJECT + ".kicad_pcb")
FPLIB = D.KLIB + "/footprints/"
LOCAL = os.path.join(OUT, "LightMeter.pretty")

# ------------------------------------------------------------- netlist
nl = ET.parse(os.path.join(OUT, "netlist.xml")).getroot()
comps = {}
for c in nl.find("components"):
    ref = c.get("ref")
    if ref.startswith("#"): continue
    fp = c.findtext("footprint")
    sp = c.find("sheetpath")
    tst = c.findtext("tstamps") or c.findtext("tstamp")
    desc = ""
    for f in c.iter("field"):
        if f.get("name") == "Description": desc = f.text or ""
    comps[ref] = dict(fp=fp, value=c.findtext("value"), path=(sp.get("tstamps") if sp is not None else "/") , tstamp=tst, desc=desc)
padnets = {}
for n in nl.find("nets"):
    name = n.get("name")
    for nd in n.findall("node"):
        padnets[(nd.get("ref"), nd.get("pin"))] = name

board = pcbnew.NewBoard(PCB)
ds = board.GetDesignSettings()
ds.SetCopperLayerCount(2)

# ------------------------------------------------------------- footprints
def load_fp(fpid):
    lib, name = fpid.split(":")
    d = LOCAL if lib == "LightMeter" else FPLIB + lib + ".pretty"
    fp = pcbnew.FootprintLoad(d, name)
    assert fp is not None, fpid
    fp.SetFPIDAsString(fpid)
    return fp

netinfo = {}
def net(name):
    if name not in netinfo:
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        netinfo[name] = ni
    return netinfo[name]

fps = {}
for ref, c in comps.items():
    fp = load_fp(c["fp"])
    fp.SetReference(ref)
    fp.SetValue(c["value"])
    fp.SetField("Description", c["desc"])
    fp.SetPath(pcbnew.KIID_PATH(c["path"].rstrip("/") + "/" + c["tstamp"]))
    board.Add(fp)
    for pad in fp.Pads():
        n = padnets.get((ref, pad.GetNumber()))
        if n and n.startswith("unconnected-"):
            n = n.replace("/", "{slash}")   # KiCad escapes "/" in auto-generated net names
        if n:
            pad.SetNet(net(n))
    x, y, rot = D.PLACE[ref]
    fp.SetPosition(VECTOR2I_MM(x, y))
    fp.SetOrientationDegrees(rot)
    fps[ref] = fp

# Mega footprint: move its full-board silk outline to F.Fab (board is narrower than the Mega) and tidy texts
mega = fps["A1"]
for it in mega.GraphicalItems():
    if it.GetLayer() == pcbnew.F_SilkS:
        it.SetLayer(pcbnew.F_Fab)
    elif it.GetLayer() == pcbnew.B_CrtYd:
        it.SetLayer(pcbnew.B_Fab)
mega.Reference().SetVisible(False)
mega.Value().SetVisible(False)
try:
    pcbnew.FootprintSave(LOCAL, mega)   # keep library copy in sync with the re-layered graphics
except Exception as e:
    print("FootprintSave:", e)
# hide/shrink noisy values on small parts
for ref, fp in fps.items():
    if ref.startswith(("R", "C", "H", "JP")):
        fp.Value().SetVisible(False)
    if ref.startswith("H"):
        fp.Reference().SetVisible(False)

# ------------------------------------------------------------- outline
def add_line(x1, y1, x2, y2, layer=pcbnew.Edge_Cuts, w=0.1):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(VECTOR2I_MM(x1, y1)); s.SetEnd(VECTOR2I_MM(x2, y2))
    s.SetLayer(layer); s.SetWidth(FromMM(w))
    board.Add(s)
    return s

B = D.BOARD
x0, x1, y0, y1 = B["x0"], B["x1"], B["y0"], B["y1"]
xn, yn = B.get("notch_x", x0), B.get("notch_y", y1)
import math as _m
def add_arc3(s, m, e):
    a = pcbnew.PCB_SHAPE(board); a.SetShape(pcbnew.SHAPE_T_ARC)
    a.SetArcGeometry(VECTOR2I_MM(*s), VECTOR2I_MM(*m), VECTOR2I_MM(*e))
    a.SetLayer(pcbnew.Edge_Cuts); a.SetWidth(FromMM(0.1)); board.Add(a)
# outline polygon with a radius per corner (0 = square inside corner of the notch)
OUTLINE = [((x0, y0), 2.0), ((x1, y0), 2.0), ((x1, y1), 2.0), ((xn, y1), 1.0), ((xn, yn), 0.0), ((x0, yn), 1.0)]
def _toward(ax, ay, bx, by, dd):
    L = _m.hypot(bx - ax, by - ay); return (ax + (bx - ax) / L * dd, ay + (by - ay) / L * dd)
_n = len(OUTLINE); _pts = []
for _i, ((cx, cy), rr) in enumerate(OUTLINE):
    (px, py), _ = OUTLINE[_i - 1]; (qx, qy), _ = OUTLINE[(_i + 1) % _n]
    if rr == 0:
        _pts.append(((cx, cy), (cx, cy), None)); continue
    _a = _toward(cx, cy, px, py, rr); _c = _toward(cx, cy, qx, qy, rr)
    _ctr = (_a[0] + _c[0] - cx, _a[1] + _c[1] - cy)
    _md = (cx - _ctr[0], cy - _ctr[1]); _L = _m.hypot(*_md)
    _pts.append((_a, _c, (_ctr[0] + _md[0] / _L * rr, _ctr[1] + _md[1] / _L * rr)))
for _i in range(_n):
    _a, _c, _mid = _pts[_i]
    if _mid is not None: add_arc3(_a, _mid, _c)
    _nx = _pts[(_i + 1) % _n][0]
    add_line(_c[0], _c[1], _nx[0], _nx[1])
# drop the Mega mounting hole beside the barrel jack: it would be cut by the notch edge
for _p in list(mega.Pads()):
    if (_p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH and abs(_p.GetPosition().x - FromMM(13.97)) < FromMM(0.2)
            and abs(_p.GetPosition().y - FromMM(-2.54)) < FromMM(0.2)):
        mega.Remove(_p)
try:
    pcbnew.FootprintSave(LOCAL, mega)
except Exception as _e:
    print("FootprintSave:", _e)
if "C2" in fps:
    fps["C2"].Reference().SetPosition(VECTOR2I_MM(D.PLACE["C2"][0] + 2.9, D.PLACE["C2"][1])); fps["C2"].Reference().SetTextAngleDegrees(90)

def board_poly(m):
    """board outline inset by m (rectilinear, notch-aware)"""
    return [(x0 + m, y0 + m), (x1 - m, y0 + m), (x1 - m, y1 - m), (xn + m, y1 - m), (xn + m, yn - m), (x0 + m, yn - m)]

# ------------------------------------------------------------- silkscreen text
def text(s, x, y, size=1.0, layer=pcbnew.F_SilkS, rot=0, bold=False):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(s); t.SetPosition(VECTOR2I_MM(x, y)); t.SetLayer(layer)
    t.SetTextSize(VECTOR2I(FromMM(size), FromMM(size))); t.SetTextThickness(FromMM(max(0.15, size * 0.15)))
    t.SetTextAngleDegrees(rot); t.SetBold(bold)
    board.Add(t)
    return t

text("EVEI LIGHT METER  -  MEGA 2560 SHIELD", 56, -53.2, 1.3, bold=True)
text("LCD G 5V SDA SCL", 20.3, -65.3, 0.8); text("LUX1 G 5V SDA SCL", 34.3, -65.3, 0.8)
text("GPS LED GND R G", 47.5, -65.3, 0.8); text("LUX LED GND R G", 59.0, -66.6, 0.8); text("IMU LED GND R G", 70.5, -65.3, 0.8)
text("SLEEP G SW 5V", 19.5, -70.5, 0.8); text("LOG G SW 5V", 31.0, -70.5, 0.8); text("FORCE G SW 5V", 43.0, -70.5, 0.8)
text("microSD", 89.0, -82.5, 0.9)
text("LUX2 G 5V SDA SCL", 25.75, -88.6, 0.8); text("ENCODER G 5V A B", 39.75, -88.6, 0.8)
text("ENC filter", 51.0, -99.6, 0.8)
# connector reference designators inside the housing outline
for ref, fp in fps.items():
    if ref.startswith("J") and ref != "J1":
        n = 4 if fp.GetFPIDAsString().find("1x04") >= 0 else 3
        fp.Reference().SetPosition(VECTOR2I_MM(D.PLACE[ref][0] + (n - 1) * 1.25, D.PLACE[ref][1] + 2.2))
        fp.Reference().SetTextSize(VECTOR2I(FromMM(0.8), FromMM(0.8))); fp.Reference().SetTextThickness(FromMM(0.15))
text("GPS v3 (antenna up)", 27.0, -47.0, 0.9); text("BNO055 IMU (UART)", 51.5, -43.5, 0.9)
fps["GPS1"].Reference().SetPosition(VECTOR2I_MM(27.0, -30.0)); fps["IMU1"].Reference().SetPosition(VECTOR2I_MM(51.5, -37.5))
text("JP1 PS1->3V3", 84.0, -25.5, 0.8)
text("LED R 220", 73.0, -18.0, 0.8, rot=90); text("BTN C 100n", 87.0, -12.5, 0.8, rot=90)
bt = text("EVEI Light Meter shield - 2026", 56, -44, 1.2, layer=pcbnew.B_SilkS); bt.SetMirrored(True)

# ------------------------------------------------------------- GND zones
def gnd_zone(layer):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer); z.SetNet(net("GND")); z.SetZoneName("GND_" + ("F" if layer == pcbnew.F_Cu else "B"))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetMinThickness(FromMM(0.25)); z.SetLocalClearance(FromMM(0.3))
    z.SetThermalReliefGap(FromMM(0.4)); z.SetThermalReliefSpokeWidth(FromMM(0.5))
    z.SetIsFilled(False)
    ol = z.Outline(); ol.NewOutline()
    m = 0.5
    for (px, py) in board_poly(m):
        ol.Append(int(FromMM(px)), int(FromMM(py)))
    board.Add(z)
    return z
# zones are added after routing so every GND pad gets a real track

# ------------------------------------------------------------- net classes (project file supplies them; make sure)
ns = ds.m_NetSettings
try:
    default = ns.GetDefaultNetclass()
    default.SetClearance(FromMM(0.2)); default.SetTrackWidth(FromMM(0.3)); default.SetViaDiameter(FromMM(0.8)); default.SetViaDrill(FromMM(0.4))
    nc = pcbnew.NETCLASS("Power")
    nc.SetClearance(FromMM(0.25)); nc.SetTrackWidth(FromMM(0.6)); nc.SetViaDiameter(FromMM(1.0)); nc.SetViaDrill(FromMM(0.5))
    ns.SetNetclass("Power", nc)
    for n in D.POWER_NETS:
        ns.SetNetclassPatternAssignment(n, "Power")
    ns.RecomputeEffectiveNetclasses()
    print("netclass Power:", ns.HasNetclass("Power"), "GND ->", ns.GetEffectiveNetClass("GND").GetName())
except Exception as e:
    print("netclass setup warning:", e)

# keep tracks and vias at least 0.6 mm from the board edge (fab copper-to-edge rule); pads and pours unaffected
def edge_keepout(xa, ya, xb, yb):
    k = pcbnew.ZONE(board)
    k.SetIsRuleArea(True)
    k.SetDoNotAllowTracks(True); k.SetDoNotAllowVias(True)
    k.SetDoNotAllowPads(False); k.SetDoNotAllowFootprints(False); k.SetDoNotAllowZoneFills(False)
    ls = pcbnew.LSET(); ls.AddLayer(pcbnew.F_Cu); ls.AddLayer(pcbnew.B_Cu); k.SetLayerSet(ls)
    o = k.Outline(); o.NewOutline()
    for px, py in ((xa, ya), (xb, ya), (xb, yb), (xa, yb)):
        o.Append(int(FromMM(px)), int(FromMM(py)))
    board.Add(k)
KO = 0.6
edge_keepout(x0, y0, x1, y0 + KO); edge_keepout(xn, y1 - KO, x1, y1)
edge_keepout(x0, y0, x0 + KO, yn); edge_keepout(x1 - KO, y0, x1, y1)
edge_keepout(xn, yn, xn + KO, y1); edge_keepout(x0, yn - KO, xn, yn)
# no tracks or vias under the BNO055: DC current near the magnetometer bends the compass heading
_ix, _iy = D.PLACE["IMU1"][0], D.PLACE["IMU1"][1]
edge_keepout(_ix - 6.5, _iy - 10.0, _ix + 6.5, _iy + 10.0)
# SD cluster under the socket: references go in a neat vertical row below the parts
for r in ("R10", "R7", "C4", "R8", "R9"):
    if r in fps:
        t = fps[r].Reference()
        t.SetPosition(VECTOR2I_MM(D.PLACE[r][0], D.PLACE[r][1] + 3.2)); t.SetTextAngleDegrees(90)
        t.SetTextSize(VECTOR2I(FromMM(0.8), FromMM(0.8))); t.SetTextThickness(FromMM(0.15))
if "U1" in fps:
    fps["U1"].Reference().SetPosition(VECTOR2I_MM(D.PLACE["U1"][0], D.PLACE["U1"][1])); fps["U1"].Reference().SetTextAngleDegrees(0)
if "U3" in fps:
    fps["U3"].Reference().SetPosition(VECTOR2I_MM(D.PLACE["U3"][0], D.PLACE["U3"][1]))
    fps["U3"].Reference().SetTextAngleDegrees(0)

try:
    ds.m_TentViasFront = True; ds.m_TentViasBack = True
except Exception as _e:
    print("tenting:", _e)
for _fp in board.GetFootprints():
    for _g in _fp.GraphicalItems():
        if _g.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS) and _g.GetClass() == "PCB_SHAPE" and _g.GetWidth() < FromMM(0.15):
            _g.SetWidth(FromMM(0.15))
    for _f in _fp.GetFields():
        if _f.GetTextThickness() < FromMM(0.15): _f.SetTextThickness(FromMM(0.15))
text("Align IMU +X axis with cart travel", D.PLACE["IMU1"][0], -12.2, 0.8)
board.BuildListOfNets()
pcbnew.SaveBoard(PCB, board)
print("saved", PCB, "footprints:", len(fps), "nets:", len(netinfo))

# ------------------------------------------------------------- autoroute
if DO_ROUTE:
    board = pcbnew.LoadBoard(PCB)
    dsn = os.path.join(OUT, "route.dsn"); ses = os.path.join(OUT, "route.ses")
    ok = pcbnew.ExportSpecctraDSN(board, dsn)
    print("DSN export:", ok)
    jar = os.path.join(os.path.dirname(os.path.abspath(__file__)), "freerouting.jar")
    cmd = ["java", "-jar", jar, "-de", dsn, "-do", ses, "-mp", "60", "-oit", "0.1", "-mt", "1"]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=1500)
    print("freerouting rc", r.returncode, (r.stdout or "")[-800:], (r.stderr or "")[-400:])
    ok = pcbnew.ImportSpecctraSES(board, ses)
    print("SES import:", ok)
    for layer in (pcbnew.B_Cu, pcbnew.F_Cu):
        z = gnd_zone(layer)
        z.SetNet(board.FindNet("GND"))
    # solid zone connection on through-hole GND pads (avoids starved thermals next to dense tracks)
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            pass  # thermal reliefs everywhere: much easier to hand-solder into two ground pours
    # remove dangling stubs left by the router
    conn = board.GetConnectivity()
    removed = 1
    while removed:
        removed = 0
        conn.Build(board)
        for t in list(board.GetTracks()):
            if t.GetClass() == "PCB_TRACK" and conn.TestTrackEndpointDangling(t, False):
                board.Remove(t); removed += 1
        print("removed dangling:", removed)
    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)
    tracks = [t for t in board.GetTracks()]
    print("tracks:", sum(1 for t in tracks if t.GetClass() == "PCB_TRACK"), "vias:", sum(1 for t in tracks if t.GetClass() == "PCB_VIA"))
    print("unrouted:", board.GetConnectivity().GetUnconnectedCount(True))
