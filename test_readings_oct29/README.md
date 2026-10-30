# Test readings, Oct 29 2026

Walk along Stadium Mall (Purdue), about 20:26 to 20:31 local time, after dark.
Recorded with `Lightmeter_Week8_NoSD.ino`, logging over USB with `tools/pc_logger.py`
(SD card module removed because it had failed and was shorting the 5V rail).

| File | What it is |
|---|---|
| `lightmeter_20261029_202630.csv` | Raw readings: date, UTC time, foot-candles, lat, lon, PC time |
| `lightmeter_20261029_202630.log` | Every line the board sent during the run |
| `lightmeter_20261029_202630_shapefile.zip` | ArcGIS point Shapefile (WGS84). Upload the zip to ArcGIS Online or add it in ArcGIS Pro |
| `lightmeter_20261029_202630.geojson` | Same points as GeoJSON |
| `map_stadium_mall.png` | The points on a map in geojson.io, dark blue = dim, yellow = brightest |

201 rows were recorded. 112 have `nan` for light, and the map files contain only the 89 valid ones.
The `nan` rows happen because the TSL2591 runs at fixed 1x gain: in the dark it reads 0 counts,
and the library's lux formula divides 0 by 0. Raising the gain (or auto-gain) is needed for night surveys.

Regenerate the map files from any log with `python tools/export_arcgis.py <csv>`.
