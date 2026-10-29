"""Write a simple VRML model of an Arduino Mega 2560 (board + tall parts) and attach it to the A1 footprint.
usage: kicad python mega_model.py <projdir>
"""
import os, sys
import pcbnew

OUT = os.path.abspath(sys.argv[1])
D3 = os.path.join(OUT, "LightMeter.3dshapes")
os.makedirs(D3, exist_ok=True)
S = 1 / 2.54   # KiCad VRML: 1 unit = 2.54 mm

def box(x0, y0, z0, x1, y1, z1, rgb):
    v = [(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),(x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)]
    faces = [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
    pts = " ".join("%.4f %.4f %.4f" % (x*S, y*S, z*S) for x, y, z in v)
    idx = " ".join("%d %d %d %d -1" % f for f in faces)
    return ("Shape { appearance Appearance { material Material { diffuseColor %.2f %.2f %.2f } }\n"
            " geometry IndexedFaceSet { solid FALSE coord Coordinate { point [ %s ] } coordIndex [ %s ] } }\n" % (*rgb, pts, idx))

# model coords: x right, y up-the-board (= KiCad -y), z up. Origin = Mega lower-left corner, z=0 = Mega top surface
blue, black, silver, dark = (0.05, 0.35, 0.55), (0.1, 0.1, 0.1), (0.75, 0.75, 0.78), (0.2, 0.2, 0.22)
parts = [box(0, 0, -1.6, 101.6, 53.34, 0, blue)]          # PCB
parts.append(box(-6.35, 32.4, 0, 9.5, 43.8, 10.7, silver))  # USB-B
parts.append(box(-1.9, 3.2, 0, 11.4, 12.1, 11.0, black))    # barrel jack
parts.append(box(40, 18, 0, 54, 32, 1.2, dark))             # ATmega2560
parts.append(box(62.5, 24.5, 0, 67.5, 31.5, 6.0, black))    # ICSP
for x0, x1, y in ((17.5, 43.0, 50.8), (44.5, 64.7, 50.8), (67.4, 87.6, 50.8),       # top row headers
                  (26.8, 47.0, 2.54), (49.6, 69.8, 2.54), (72.4, 92.6, 2.54)):     # bottom row headers
    parts.append(box(x0, y - 1.27, 0, x1, y + 1.27, 8.5, black))
parts.append(box(92.7, 6.3, 0, 97.8, 52.1, 8.5, black))     # 2x18 header
parts.append(box(12, 44, 0, 18, 50, 3.5, silver))           # reset button
wrl = "#VRML V2.0 utf8\n" + "".join(parts)
path = os.path.join(D3, "Arduino_Mega2560_board.wrl")
open(path, "w").write(wrl)
print("wrote", path)

pcb = os.path.join(OUT, "LightMeter_MegaShield.kicad_pcb")
board = pcbnew.LoadBoard(pcb)
a1 = board.FindFootprintByReference("A1")
a1.Models().clear()
m = pcbnew.FP_3DMODEL()
m.m_Filename = "${KIPRJMOD}/LightMeter.3dshapes/Arduino_Mega2560_board.wrl"
m.m_Offset = pcbnew.VECTOR3D(0, 0, -10.1)   # 1.6 mm shield + 8.5 mm Mega female headers
a1.Models().push_back(m)
# bottom-silk note where the Mega sits
t = pcbnew.PCB_TEXT(board)
t.SetText("ARDUINO MEGA 2560 PLUGS IN HERE (stacking headers)")
t.SetPosition(pcbnew.VECTOR2I_MM(56, -47.2)); t.SetLayer(pcbnew.B_SilkS); t.SetMirrored(True)
t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(1.0), pcbnew.FromMM(1.0))); t.SetTextThickness(pcbnew.FromMM(0.15))
board.Add(t)
pcbnew.SaveBoard(pcb, board)
try:
    pcbnew.FootprintSave(os.path.join(OUT, "LightMeter.pretty"), a1)
except Exception as e:
    print("FootprintSave:", e)
print("model attached")
