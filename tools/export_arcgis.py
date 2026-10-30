"""Convert Lightmeter CSV logs into ArcGIS-ready map layers.

Usage:
    python tools/export_arcgis.py                      # newest CSV in logs/
    python tools/export_arcgis.py logs/a.csv logs/b.csv  # merge several runs
    python tools/export_arcgis.py --include-invalid    # keep rows with no light value

Writes to exports/<name>/:
    <name>_shapefile.zip  Point Shapefile in WGS84. Upload this zip directly to
                          ArcGIS Online, or unzip and add the .shp in ArcGIS Pro.
    <name>.geojson        Same points as GeoJSON (also accepted by ArcGIS).

Each point carries: fc (foot-candles), lux, date, time_utc, pc_time, run, valid.
"""
import csv, glob, json, math, os, sys, zipfile
import shapefile  # pyshp

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FC_TO_LUX = 10.76391
WGS84_PRJ = ('GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137.0,298.257223563]],'
             'PRIMEM["Greenwich",0.0],UNIT["Degree",0.0174532925199433]]')

def parse_fc(text):
    try:
        v = float(text)
    except ValueError:
        return None
    return v if math.isfinite(v) and v >= 0 else None

def load(paths, include_invalid):
    points, skipped = [], 0
    for path in paths:
        run = os.path.splitext(os.path.basename(path))[0]
        for r in csv.DictReader(open(path, newline="")):
            try:
                lat, lon = float(r["lat"]), float(r["lon"])
            except (KeyError, ValueError):
                skipped += 1; continue
            if not (-90 <= lat <= 90 and -180 <= lon <= 180) or (lat == 0 and lon == 0):
                skipped += 1; continue
            fc = parse_fc(r.get("fc", ""))
            if fc is None and not include_invalid:
                skipped += 1; continue
            points.append({
                "lat": lat, "lon": lon, "fc": fc,
                "lux": None if fc is None else round(fc * FC_TO_LUX, 2),
                "date": r.get("date_yyyy-mm-dd", ""), "time_utc": r.get("time_utc", ""),
                "pc_time": r.get("pc_time", ""), "run": run, "valid": 0 if fc is None else 1,
            })
    return points, skipped

def write_shapefile(points, base):
    w = shapefile.Writer(base, shapeType=shapefile.POINT)
    w.field("fc", "N", 12, 3)
    w.field("lux", "N", 12, 2)
    w.field("date", "C", 10)
    w.field("time_utc", "C", 8)
    w.field("pc_time", "C", 19)
    w.field("run", "C", 40)
    w.field("valid", "N", 1, 0)
    for p in points:
        w.point(p["lon"], p["lat"])
        w.record(p["fc"], p["lux"], p["date"], p["time_utc"], p["pc_time"][:19], p["run"][:40], p["valid"])
    w.close()
    open(base + ".prj", "w").write(WGS84_PRJ)
    open(base + ".cpg", "w").write("UTF-8")
    zpath = base + "_shapefile.zip"
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        for ext in (".shp", ".shx", ".dbf", ".prj", ".cpg"):
            z.write(base + ext, os.path.basename(base) + ext)
    for ext in (".shp", ".shx", ".dbf", ".prj", ".cpg"):
        os.remove(base + ext)
    return zpath

def write_geojson(points, path):
    feats = [{"type": "Feature",
              "geometry": {"type": "Point", "coordinates": [p["lon"], p["lat"]]},
              "properties": {k: p[k] for k in ("fc", "lux", "date", "time_utc", "pc_time", "run", "valid")}}
             for p in points]
    json.dump({"type": "FeatureCollection", "features": feats}, open(path, "w"), indent=1)

def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    include_invalid = "--include-invalid" in sys.argv
    paths = args or sorted(glob.glob(os.path.join(ROOT, "logs", "*.csv")), key=os.path.getmtime)[-1:]
    if not paths:
        sys.exit("No CSV logs found in logs/.")
    name = os.path.splitext(os.path.basename(paths[0]))[0] + ("_merged" if len(paths) > 1 else "")
    out_dir = os.path.join(ROOT, "exports", name)
    os.makedirs(out_dir, exist_ok=True)

    points, skipped = load(paths, include_invalid)
    if not points:
        sys.exit("No usable points (all rows invalid). Try --include-invalid.")
    zpath = write_shapefile(points, os.path.join(out_dir, name))
    gpath = os.path.join(out_dir, name + ".geojson")
    write_geojson(points, gpath)

    fcs = sorted(p["fc"] for p in points if p["fc"] is not None)
    print(f"Input: {', '.join(os.path.basename(p) for p in paths)}")
    print(f"Points written: {len(points)}   rows skipped: {skipped}")
    if fcs:
        print(f"fc range: {fcs[0]:.2f} to {fcs[-1]:.2f}   median {fcs[len(fcs)//2]:.2f}")
    print(f"Shapefile: {zpath}")
    print(f"GeoJSON:   {gpath}")

if __name__ == "__main__":
    main()
