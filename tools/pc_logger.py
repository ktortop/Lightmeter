"""Save Lightmeter data to this PC over USB.

Usage:  python tools/pc_logger.py [COM8]
Every "CSV," line from the board is appended to logs/lightmeter_<start time>.csv.
All other lines (STATUS, setup messages, errors) are printed to the screen
and also saved to logs/lightmeter_<start time>.log. Press Ctrl+C to stop.
"""
import os, sys, time
from datetime import datetime
import serial
from serial.tools import list_ports

BAUD = 115200
HEADER = "date_yyyy-mm-dd,time_utc,fc,lat,lon,pc_time\n"

def find_port():
    for p in list_ports.comports():
        # 0x2341 / 0x2A03 = Arduino USB vendor IDs; also match by name
        if p.vid in (0x2341, 0x2A03) or "Arduino" in (p.description or "") or "Mega" in (p.description or ""):
            return p.device
    return None

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else (find_port() or "COM8")
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.abspath(os.path.join(out_dir, f"lightmeter_{stamp}.csv"))
    log_path = csv_path[:-4] + ".log"

    print(f"Opening {port} at {BAUD} baud. Close Arduino Serial Monitor first.")
    print(f"Data rows -> {csv_path}")
    rows = 0
    with serial.Serial(port, BAUD, timeout=1) as s, \
         open(csv_path, "w", newline="") as csv_f, open(log_path, "w") as log_f:
        csv_f.write(HEADER); csv_f.flush()
        while True:
            raw = s.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            now = datetime.now().strftime("%H:%M:%S")
            log_f.write(f"{now} {line}\n"); log_f.flush()
            if line.startswith("CSV,"):
                csv_f.write(line[4:] + f",{datetime.now().isoformat(timespec='seconds')}\n")
                csv_f.flush()
                rows += 1
                print(f"{now}  ROW {rows}: {line[4:]}")
            elif not line.startswith("CSVHDR"):
                print(f"{now}  {line}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
