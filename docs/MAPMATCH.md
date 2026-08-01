# Global localization from track shape and OSM map

Task: find where the car drove **without GNSS** — odometry and map only. GNSS on runs is
unreliable: in the city it drifts tens of meters, and on one run it reports "total turn +147°"
where reality is +47° (extra accumulation from jitter while parked).

The idea is the same as a person with a map: "went straight two kilometers, then right, then
left" — and from such a chain the place is unique if there are enough turns. Implemented
entirely in C++ with Python bindings because the target is a phone.

    python3 -m mapmatch.locate adas_logs/<run> --verify-gnss

## Components

| layer | files | role |
|---|---|---|
| map | `mapmatch/osm_graph.py`, `road_map.{h,cpp}` | OSM PBF → compact `ADASMAP1` graph, read on phone |
| track | `track.{h,cpp}` | speed + yaw rate → path shape and maneuver chain |
| search | `search.{h,cpp}` | route candidates on graph, heading-profile comparison |
| fit | `fit.{h,cpp}` | elastic track fit onto route, deformation estimate |
| assembly | `mapmatch/locate.py` | search → fit selection → top-K with margin |
| plots | `mapmatch/plot_on_map.py` | GNSS, raw odometry, and fitted track on one map |

All in C++: `app/src/main/cpp/{include,src}/mapmatch`, about 1200 lines. Bindings — submodule
`pyadas.core.mapmatch`.

## Map

`mapmatch/fetch_map.py` pulls PBF (Moscow — 81 MB), `mapmatch/osm_graph.py` turns it into
`maps/Moscow.osm.admap` — **4.9 MB**: 46 545 nodes, 57 677 edges, 300 154 geometry points,
4335 names.

Format is phone-oriented: file read in one chunk, fixed-record tables and
CSR indices (node → edges, grid cell → edges), no parsing at runtime. Coordinates — centimeters from
anchor point in `int32`: enough for ±21 000 km at 1 cm resolution. Directed edge encoded as
`2·id + direction`, so a route is just a `uint32` array.

Download had to use `--chunked`: the mirror had a ~16 KB per-connection limit and normal download stalled.

## Track from a run

`mapmatch/track_from_bag.py` takes speed and yaw rate from the bag, `buildTrack` integrates them
into shape. Everything downstream is computed **by path length**, not time — 2 m steps so stops at
lights do not affect shape.

`segmentManeuvers` cuts the path into straights and turns by smoothed curvature (threshold — radius 80 m,
minimum turn 25°, gap merge up to 25 m) and yields a chain like
"straight 2.3 km → left 113° → straight 130 m → right 47° → …".

**On yaw rate.** Default source is ESP sensor, not phone IMU, and zero bias removal at stops is **off**.
This is measurement, not caution: parked ESP shows +0.145 °/s, but that bias is absent while moving — the block compensates zero itself. If parking bias is subtracted, total turn on run `2026_07_26_20_55_20` shifts from +49° to +9° vs GNSS reference +47°,
i.e. 40° error over 3 km. For phone IMU zeroing on stops helps (+6° → +4°), so the flag
remains but defaults off. IMU overall gave no clear win on two runs —
`analyzeYaw` prints comparison of both sources, including correlation and mutual scale.

## Finding location

Naive idea "match straight-right-left chain as a string" does not work. The same exit odometry sees as "+113°", GNSS as "+69° … +56°", and the map splits it into its own edges.
Matches are random — verified on run `2026_07_26_20_55_20`.

So **heading profile** θ(s) is compared: the route grows on the graph, and at each step
how well its direction matches track direction at the same path length is scored
(σ = 12° — both odometry error and OSM centerline drawn coarser than the real road).
This measure does not depend on edge subdivision.

Seeds are not all map edges (that cannot fit), only intersections where the **first track turn**
with the required angle can occur. Beam 400 branches, pruning by dominance with key
`(edge, heading, distance traveled)`. Separately checked that before the first turn the observed straight
actually accumulates — backward pass on the graph.

## Fitting onto route

Hard overlay of the track is impossible: odometry is accurate in shape but accumulates heading error. So a small **deformation** is also fit — and its magnitude separates the right place from a similar one:

| what is fit | tolerance |
|---|---|
| position and heading | free |
| speed scale (wheel radius) | 3 % |
| yaw-rate sensor scale | 5 % |
| per-turn angle correction | 6° |
| per-straight length correction | 8 % |
| heading drift, 300 m blocks | 0.6 °/100 m |

Minimized: track-point distance to centerlines (σ = 4 m — half lane plus map error) plus
deformation penalty. Gauss-Newton on Eigen with numerical Jacobian, Huber loss, residual cap
25 m, 30 iterations.

What had to be added for this to work:

* **Huber loss instead of hard residual cap** — the cap zeroed gradient, and a point landing
  in a courtyard or tunnel simply dropped out instead of pulling weaker;
* **heading-drift corrections in 300 m blocks** — without them the main odometry error (slow zero drift
  of the sensor) was not modeled at all: 0.1 °/s at 20 m/s is 10° over two kilometers;
* **σ annealing** (4 passes, starting at ×12) — in urban grid, points half a block apart
  stick to different streets and pull opposite ways, fit gets stuck;
* **Procrustes for initial position** — starting at route beginning gave a local minimum.

## Result

Run `2026_07_26_20_55_20`, 3 km, 4 turns, **without GNSS**:

* graph search — 40 candidates in **0.1 s** on a 57 677-edge map;
* coarse fit of 15 candidates + refine leaders — **0.4 s**;
* correct route (Donetskaya → Nizhnie Polya → Lyublinskaya) ranks **first**;
* reference check against recorded GNSS matches.

Plots: `maps/located.png` (found without GNSS), `maps/map_2026_07_26_20_55_20.png` (GNSS, raw
odometry and fitted on one map).

**But margin to second place is only 13 %** — by the script's own scale that is "ambiguous". So the
answer is correct but confidence is low: the second candidate is almost as good geometrically.

A run with one turn cannot be localized in principle, and the script says so directly.

## What remains

By descending importance.

1. **Understand the 13 % margin.** Suspicious that deformation of the correct route is 1.04 σ — that is
   a lot for the right place. First to check: does backward route extension truncate track start so the first long straight does not seat fully.
2. **No tests at all.** Need: `geo` reversibility (geo → local → geo), track segmentation on
   synthetic shape with known turns, search on a small hand-made map, fit on a known-correct route with deformation near zero check. Everything rests on two
   runs and eyeballing.
3. **Run statistics.** Two runs is not a sample. Need fraction of correct answers and margin
   distribution on at least ten, varying length and turn count. From that an honest threshold for
   "confident / ambiguous / cannot choose" instead of current 35 % and 10 % by eye.
4. **Phone measurement.** Map format is phone-ready (single-chunk read, CSR, int32), but on device **never run once**: no search time, memory, or whether Moscow fits the app budget. Numbers above are from a laptop.
5. **Localization as a service.** Currently an offline tool: script on a finished bag. For in-car use need incremental mode (track grows, candidates refine) and hypothesis hold between frames. None of that exists.
6. **Runs this was measured on were deleted from `adas_logs`** (`2026_07_26_*`). Numbers above cannot be reproduced — remeasure on current runs at first opportunity.

## My mistakes along the way, so they are not repeated

* "Total GNSS turn +147°" — counted from unfiltered fixes, including parking jitter. After filter +47°.
* "Track diagonally goes through empty space, map incomplete" — there is Nizhnie Polya street there,
  just without a name in my output.
* Started writing geometry in Python (`geo.py`) though target is phone; removed and moved to C++.
