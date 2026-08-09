# Road curvature ahead from the OSM map

The `map_data` service answers one question, continuously: **how sharp is the road ahead, further than the
camera can see?** It is dragonpilot's `selfdrive/mapd` chain ported to C++ against the map this project
already ships. Nothing consumes its output yet — it publishes `map/local`, the bag records it, and checking
it against the road the car actually drove is the work that decides whether it should ever reach the
controller. That check has now been run: the geometry holds up, the peak curvature does not. See
[below](#the-test-that-actually-convicts-it).

    python3 bag_map_data.py adas_logs/<run> --vs-driven --compare-vision --plot out.png

## Why bother when the model already sees curvature

The controller's speed limit for corners comes from `utils/curvature_preview.h`: the vision path, out to
about 150 m. That is roughly 6 s at 25 m/s, and it ends where the camera's useful range ends — over a crest,
around a building, in rain, it ends sooner. A map knows the corner is there 2 km early and does not care
about weather.

It also knows things the camera cannot infer at all: that this is a slip road and not the main carriageway,
what the road is called, where the junctions are.

The catch is that a map is a drawing, and this one is drawn coarsely. That tension is what the logging is
for.

## What it does, step by step

| step | where | what |
|---|---|---|
| load | `mapmatch/road_map.{h,cpp}` | `Moscow.osm.admap`, one read, no parsing — **8.6 MB** resident, 6 ms |
| match | `road_route.cpp` `matchDirectedEdge` | pick the directed edge by distance **and** heading, one-ways applied |
| walk | `road_route.cpp` `buildRouteAhead` | grow 2 km forward: straightest continuation, same road name wins ties |
| resample | `resamplePolyline` | fixed 5 m steps along the route |
| curvature | `curvatureAlong` | heading change over a 25 m window, positive left |
| sections | `turnSections` | \|kappa\| >= 0.002 1/m, split by sign and by peak, `v = sqrt(2.6 / kappa)` |
| publish | `services/map_data_service.cpp` | `map/local` at 2 Hz, into the bag via the ZMQ bridge |

The bag cost is measured, not estimated (`tests/test_map_data_service.cpp` prints and pins it): **5.2 kB per
message** for the route, plus **15 kB** for the surrounding road graph, which rides along every 5 s rather
than every tick. About 45 MB/hour at the defaults — the same class as `vision/model_long` (45 MB/h on these
runs) and far under `localization/pose` (73 MB/h).

Thresholds are dragonpilot's — `_TURN_CURVATURE_THRESHOLD = 0.002`, `_MAX_LAT_ACC = 2.6`,
`_MIN_SPEED_SECTION_LENGTH = 100`, `_MAX_CURV_DEVIATION_FOR_SPLIT = 2`, `_MAX_CURV_SPLIT_ARC_ANGLE = 90` —
kept unchanged so the numbers are directly comparable with `liveMapData`. They are exposed in `config.json`
because the only way to know whether they suit *this* map is to vary them over a recorded run.

## Three places where this differs from dragonpilot

**No Overpass, no query thread, no cache.** dragonpilot fetches a 3 km circle over the network and rebuilds
its way collection when the car approaches the edge of it — `QUERY_RADIUS`, `MIN_DISTANCE_FOR_NEW_QUERY`, a
background thread and a lock. We already ship the whole of Moscow as a 4.9 MB file that loads in 6 ms
(`docs/MAPMATCH.md`), so the whole apparatus collapses into a lookup. Route build is **0.02 ms median,
0.04 ms p95** on a laptop; the service costs about a millisecond a second.

**Curvature comes from the heading profile, not from a spline.** dragonpilot inserts a node every 15 m
wherever OSM's are more than 50 m apart, then fits a spline through the result and differentiates it twice.
That works, but it invents the geometry between nodes. Here the heading of each map segment is sampled *at
the segment's midpoint* — on a curve, a chord is parallel to the tangent at its arc midpoint exactly — and
curvature is the slope of that profile over a 25 m window.

The difference is not cosmetic, because this map is coarse: median node spacing is 16 m but p90 is 67 m, and
a third of all segments are longer than the window. Measured on a 250 m arc — worst error in 1/R over the
middle of it, 25 m window, sampled every 5 m:

| OSM node spacing | from the map geometry (this) | from the resampled points |
|---|---|---|
| 2 m | 0.00 % | 0.8 % |
| 10 m | 0.01 % | 20 % |
| 40 m | 0.11 % | 100 % |
| 80 m | 0.43 % | 220 % |

Both tests are in `tests/test_road_route.cpp`, including the one that asserts the naive version *fails* — so
the structure (map geometry in, resampled grid only as query points) cannot be quietly simplified away.

**Position is anchored, not fused.** The map needs latitude and longitude, and only the raw
`sensors/gps/data` message still has them — by the time anything in C++ sees `sensors/gps/location`,
`TopicConvertService` has projected it into a frame whose origin is private to that projector. But GPS on
these runs arrives at 0.1–1 Hz and jumps, while `localization/pose` is 100 Hz. So each fix anchors the map
frame and pose deltas carry the position between fixes. The drift in between is **measured, not assumed**:
`pose_gps_gap_m` in every message is the error at the moment the next fix arrived.

## What it looks like on real runs

Nine runs from 2026-08-04 to 2026-08-08, replayed through the same C++ code:

* **96 % of samples matched**, match distance **median 3.3 m**, p95 6–16 m per run. On a map that draws a
  dual carriageway as one centreline, that is as good as the representation allows.
* Map **node spacing median 34 m**, p90 60–130 m. On the motorway stretches the map has a node every
  70–170 m, so `kappa` there is an average over hundreds of metres.
* Turn sections on 23–57 % of samples depending on the route; slowest section 14–41 km/h.

`maps/map_curvature_2026_08_07.png` and `maps/map_curvature_2026_08_08.png` show two of them: the road graph
in grey, the driven track in blue, every route the service built in orange, turn sections coloured by the
speed they imply.

### The test that actually convicts it

`--vs-driven` compares the heading change the map predicts over the next 200 m — the integral of the
curvature, which is what the geometry genuinely claims — against the heading change the car then made over
its next 200 m. It needs no second estimator to be right, which is why it, and not the vision correlation
below, is the number to trust.

| | result |
|---|---|
| predicted vs driven heading change | **median 1–2°**, p90 7–18° |
| route walked onto the wrong road | **0–13 samples per run, under 2 %** |
| peak \|kappa\| implying R < 100 m | 19–176 samples per run |
| …of those, where the car barely turned | **20–89 %** |

Read together those rows say something specific and useful: **the map's geometry is right, and the peak is
the wrong statistic to read off it.**

The walk almost never goes to the wrong road, and where it does the error is visible. The heading over the
next 200 m is right to a degree or two. But a fifth to nine tenths of the sharp curvature peaks sit at places
where the road does not actually turn — they are single nodes drawn a few metres off the centreline, and the
25 m window is short enough to read one as a 50 m-radius corner.

On run `2026_08_08_10_47_41` that is the whole story: 100 % matched, heading right to 1°, and yet 17 of the
19 sharp peaks are kinks. A planner reading peak curvature would have demanded a slowdown on 1.7 % of that
run, by up to 5 km/h, at a junction on Варшавское шоссе where the road is straight and one node is offset.

Two caveats on that table. An S-bend nets close to zero heading change while genuinely being curved, so the
"barely turned" count over-counts slightly. And run `2026_08_04_18_19_09` (67 samples, city) is the outlier
at 20° median error — short, slow, and worth looking at on its own rather than averaging away.

### Against the vision estimate

`--compare-vision` correlates the map's peak \|kappa\| within 200 m against the model's `kappa_ahead` at
**+0.11 to +0.44**, with the map's median 5–10× the model's.

Given the finding above, that low correlation is now mostly explained rather than mysterious: the map's peak
is contaminated by drawing kinks, and the two summarise different windows anyway (the map a peak over a fixed
200 m, the model a high percentile over `curv_preview_s` seconds of travel). The comparison as it stands
cannot separate a real disagreement from either effect.

### What this means for using it

Not "the map is unusable" — the geometry is good to a degree over 200 m, which is more than enough to know a
corner is coming. It means **the summary has to change before anything consumes it**: use the integrated
heading change over a window, or require a peak to accumulate into a real turn before it counts. Reading
`route_kappa.max()` and braking on it would brake for nothing several times an hour.

## What is not done

1. **Never run on the phone.** All timings above are from a laptop. The map format is phone-oriented and
   8.6 MB should fit, but the service has not been in the car once.
2. **A peak statistic that survives the kinks.** Measured above: a fifth to nine tenths of sharp curvature
   peaks are drawing artefacts. Until the summary is integrated rather than peak-based — and validated with
   `--vs-driven` on more runs — no map speed should reach `long_plan`.
3. **No speed limits.** dragonpilot also reads `maxspeed`, `highway=stop`, `give_way`, `traffic_calming` from
   the node tags. `ADASMAP1` carries none of them — the graph has geometry, names and one-way flags. Adding
   them is a change to `mapmatch/osm_graph.py` and the file format, not to this service.
4. **The route is a guess, not navigation.** "Keep going straight, prefer the same name" is wrong the moment
   the driver takes an exit — as it is in dragonpilot. The failure is visible in the log (the route diverges
   from the driven track) but nothing acts on that yet.
5. **Standstill.** Below `min_speed_mps` the last good heading is held, because yaw rate integrates noise and
   GPS course is meaningless when stopped. Whether that is enough at a long light is untested.

## Running it

**On** in the shipped `assets/config.json` (`nodes.map_data`) — it records, it does not act. The map itself
ships in the APK: `syncRoadMap` in
`app/build.gradle` copies `maps/Moscow.osm.admap` into assets the same way the ONNX models are packaged, and
it costs **3.4 MB** of APK (5.1 MB raw, 34 % after compression).

Unpacking it onto the device is gated on the same flag, so an install with `map_data` off pays the APK size
but not the 5 MB of internal storage. `AdasConfig.mapAsset()` reads `map.path` — the same key the C++ config
reads — Java unpacks that asset and `nativeStart` hands C++ the absolute path. One key, two readers, no
second place to keep in sync.

```bash
./scripts/build_project.sh
adb install -r app/build/outputs/apk/debug/app-debug.apk
# on a phone that has run the app before: "Reset params from assets" in the UI — see below
# drive, then:
python3 bag_map_data.py adas_logs/<run> --vs-driven --plot out.png
```

**The stale-config trap.** `ensureAssetCopied` does not overwrite an existing `filesDir/config.json`, and
`AdasConfig` prefers that file over the asset. So on a phone that has already run the app, a new APK does
*not* change any config value — including this flag. Nothing in the code reads the `version` key, so there
is no upgrade path but the UI's "Reset params from assets", which also discards any slider tuning stored
there. Both `AdasAppHandler` ("map_data node off — road map not unpacked") and `nativeStart`
(`map_data=0`) say so in logcat, so check there before concluding the service is broken.

A map the APK does not carry can still be tested without rebuilding — the service falls back to
`/sdcard/adas_maps/<name>`, so `adb push` of a different `.admap` overrides nothing but works.

Runs recorded *before* the service existed can still be analysed — the script falls back to replaying their
GPS and pose through the same C++ code (`--force-replay` forces this even when `map/local` is present).
Replay evaluates at each fix rather than on a timer, so it never exercises the dead reckoning between
fixes: it is the optimistic case for position.

## Files

| file | role |
|---|---|
| `cpp/include/mapmatch/road_route.h`, `cpp/src/mapmatch/road_route.cpp` | match, walk, resample, curvature, sections |
| `cpp/include/mapmatch/dir_edge.h` | directed-edge helpers, shared with `mapmatch/search.cpp` |
| `cpp/include/services/map_data_service.h`, `cpp/src/services/map_data_service.cpp` | the service: inputs, anchoring, publishing |
| `proto/map_data.proto` | `MapLocalState` — what lands in the bag |
| `cpp/tests/test_road_route.cpp`, `cpp/tests/test_map_data_service.cpp` | the maths, and the wiring |
| `scripts/bag_map_data.py` | run analysis, logged or replayed |
| `maps/Moscow.osm.admap` | 46 545 nodes, 57 677 edges — built by `scripts/mapmatch/osm_graph.py` |
