"""Download datalog.csv from the Lightmeter's SD card over USB.

Needs tools/sddump uploaded to the board. Usage: python tools/sd_download.py [COM12] [out.csv]
"""
import sys, time, serial
port = sys.argv[1] if len(sys.argv) > 1 else "COM12"
out = sys.argv[2] if len(sys.argv) > 2 else "sd_datalog.csv"
s = serial.Serial(port, 115200, timeout=2)
s.setDTR(False); time.sleep(0.1); s.setDTR(True)
data, started, size, t0 = bytearray(), False, None, time.time()
while time.time() - t0 < 120:
    line = s.readline()
    if not line:
        continue
    if not started:
        txt = line.decode(errors="replace").strip()
        if txt.startswith("SDDUMP ERROR"):
            sys.exit(txt)
        if txt.startswith("SDDUMP BEGIN"):
            started, size = True, int(txt.split("size=")[1])
        continue
    if line.strip() == b"SDDUMP END":
        break
    data += line
s.close()
body = bytes(data)
if body.endswith(b"\r\n"):
    body = body[:-2]          # the newline printed before END
open(out, "wb").write(body)
print(f"expected {size} bytes, received {len(body)} bytes -> {out}")
