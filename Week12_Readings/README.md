# Week 12 readings, Nov 12 2026

First walk recorded with every module working, including the SD card.
Recorded with `Lightmeter_Week12.ino`, read off the SD card's `datalog.csv` over USB
with `tools/sddump` + `tools/sd_download.py`.

- When: 9:54 to 9:56 PM local (02:54 to 02:56 UTC on Nov 13 in the GPS timestamps), after dark
- Where: Purdue campus walkways, about 120 m, roughly east to west then northwest
- Rows: 91, all with a valid light reading (0.19 to 1.95 fc, mostly 0.2 to 0.4)

| File | What it is |
|---|---|
| `week12_readings_nov12.csv` | The 91 readings: date, UTC time, foot-candles, lat, lon |
| `week12_readings_nov12_shapefile.zip` | ArcGIS point Shapefile (WGS84). Upload the zip to ArcGIS Online or add it in ArcGIS Pro |
| `week12_readings_nov12.geojson` | Same points as GeoJSON |
| `map_nov12.jpg` | The points in geojson.io, dark blue = dim, orange/yellow = brightest |

Hardware for this run: Adafruit MicroSD breakout board+ (product 254) powered from the Mega's 3V3 pin.
