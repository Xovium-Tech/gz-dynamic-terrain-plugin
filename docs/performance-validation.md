# Terrain optimization validation

The baseline is upstream `28202c64196ffd7b50475e4df20ddf45d44b11a8`.
Quality settings are unchanged. These changes reduce repeated decoding,
allocation, coordinate calculation, connection setup and synchronous upload
bursts; they do not lower terrain or image resolution.

## Deterministic regression checks

Build a repository checkout with `BUILD_TESTING=ON`. `DownloadCache_TEST`
starts a loopback-only HTTP/1.1 fixture and covers connection reuse across
batches, cache corruption, invalid HTTP-200 bodies, truncated responses,
permanent failures, long Retry-After deadlines and cancellation, response bounds, overlapping callers,
subscriber cancellation, cross-process flock, local sources, locked-cache
cancellation and shutdown during a stalled response. `DownloadCache_TLS_TEST`
uses an ephemeral local HTTPS certificate to check trust overrides, override
precedence, invalid bundles and hostname verification. Neither contacts a map
provider. The three `DownloadLocal_*_TEST` cases queue 64 local 1 MiB files and
check prompt release of completed payloads and tile-lock descriptors, accurate
byte accounting, and cancellation or shutdown between local tasks. Completed
local copies release their resources before the rest of a scheduling pass;
cancelled queued copies are never started. A 100-request serial fixture also rejects scheduler-induced idle gaps;
this check failed with the intermediate 25 ms polling delay (2.58 seconds) and
passed after immediate slot refilling (0.016 seconds). These are tiny local
responses used to isolate scheduling overhead. The runtime regression includes
100 immediate create/destroy cycles. The render test separately verifies that a
one-page upload limit spans three frames and only activates the complete result.
Tile-store regressions reject incomplete JPEG entropy even when OpenCV returns
partial pixels, repair a corrupt cached image whose size and timestamp were
preserved, isolate two custom datasets at the same XYZ coordinates, and ensure
an oversized DEM remains full resolution without exceeding the decoded-cache
budget.

`TerrainParityBenchmark` seeds exact offline imagery/elevation and alternates
between two neighboring centers. Compile the same
`tools/terrain_parity_benchmark.cc` against baseline and candidate core libraries
with the same compiler/OpenCV. It hashes all vertex coordinates, normals, UVs,
indices, bootstrap/refined RGB, progressive page ordering and encoded collision
heights, excluding resource names and timings. Compare identical loop counts.

One local Release comparison with GCC 13.3 and OpenCV 4.6, six iterations:

| Build | Exact output hash | First iteration | Mean remaining iterations |
| --- | --- | ---: | ---: |
| Baseline | `6f557636d9f709e3` | 566.5 ms | 495.3 ms |
| Candidate | `6f557636d9f709e3` | 580.4 ms | 184.6 ms |

The warm case improved about 63%; the first iteration, which includes initial
decode validation, was about 2.5% slower. These are decoded-cache measurements;
the benchmark does not flush the operating system's disk cache. This is a small deterministic workload, not a
general simulation FPS claim. Network fixture checks measure connection counts
and deterministic failure behavior rather than Internet-provider throughput.

A separate controlled renderer comparison ran the same 160×90 Ogre2 camera,
two 32-cell mountain pages and repeating route for 120 seconds per build. The
new failure-injection and upload-budget regressions were disabled equally for
this comparison because the baseline does not implement those behaviors.
They remain enabled in the normal regression and soak test.

| Build | Rendered frames | Frame p50 | Frame p95 | Frame p99 | Peak GPU memory |
| --- | ---: | ---: | ---: | ---: | ---: |
| Baseline | 61,196 | 2.1 ms | 2.5 ms | 2.8 ms | 76 MiB |
| Candidate | 62,295 | 2.0 ms | 2.5 ms | 2.8 ms | 76 MiB |

Frame timing includes terrain updates and camera rendering. Histogram buckets
are 0.1 ms wide. Both runs passed all pixel checks and retained six texture
entries; peak RSS was approximately 646 MB for both. This small scene showed
no material frame-time regression. It does not establish performance for a
large Gazebo world or measure the effect of upload budgeting on larger pages.

## Long-running checks

The camera renderer and streaming runtime are complementary component tests.
The former renders actual Ogre2 camera frames while recentering terrain and
checks every visible pixel for holes, generation retirement, failed staging
retention, texture counts, frame-time percentiles and RSS. The latter runs the
actual terrain download, cache, mesh, refinement and collision workers along a
moving geographic route against a local PNG server with recurring outages;
its sink acknowledges ready snapshots immediately. It records generations,
refinements, collision handoffs, RSS samples and shutdown latency.

```bash
build-jetty/Ogre2TextureLifecycle_TEST --seconds 3600 --report /tmp/render-soak.json
python3 tools/streaming_soak.py --binary build-core/TerrainStreamingSoak \
  --seconds 3600 --report /tmp/streaming-soak.json
```

Use a working display or Xvfb for Ogre2, and permit loopback sockets for the
network fixture. Keep logs with the JSON reports to inspect memory trends.
These checks do not substitute for an aircraft flight, engine-wide physics
validation, a particular provider's availability, or Isaac/Unreal renderer
tests. Long runs are only reported as passed when their processes complete.

## Real physics contact and replacement

The separate `tools/physics_smoke.py` fixture loads the actual terrain plugin
inside a Gazebo server with the default DART physics engine. It seeds offline
Terrarium tiles at two locations with 10 m and 20 m elevations, drops a dynamic
1 m box under gravity, and checks four contact stages across three terrain
replacements. A missing destination dataset exercises failed replacement while
the original ground must continue supporting the body for two wall-clock
seconds. Restoring the tiles must allow replacement and renewed contact.
Each run has a unique local transport partition. Body velocity commands are
removed after one step for consistent Harmonic and Jetty initialization.

```bash
python3 tools/physics_smoke.py --build /tmp/build-jetty \
  --report /tmp/physics-jetty.json
# Optional original libraries with the same Gazebo ABI:
python3 tools/physics_smoke.py --build /tmp/build-jetty \
  --libraries /tmp/baseline-build --report /tmp/physics-baseline.json
```

The helper compiles only the fixture against an existing Makefiles build with
`BUILD_TESTING=ON`; it does not relink or replace loaded plugin libraries.
The baseline, optimized Jetty, and optimized Harmonic checks all passed. Their
four measured contact heights matched at the recorded precision: 10.5717192691,
19.7673462446, 10.5717192691, and 19.7673462446 m. Analytical terrain elevation
plus half the box height is 10.5000000002 and 19.7944627307 m; the test allows
0.15 m for the small heightfield/solver deviation also present in the baseline.
The existing nearly-flat heightfield encoding uses a minimum 0.10 m vertical
span; these checks do not isolate which engine conversion causes the deviation.
This establishes actual contact and replacement for these fixtures; it does
not validate every vehicle collision shape, physics engine, or flight scenario.

The system-plugin regression also submits rapid cached replacements and pauses
simulation while replacement work completes. The original implementation could
overwrite its single pending retirement before the 0.10 s overlap expired,
leaving older collision models alive indefinitely. The corrected system keeps
that overlap and waits for the retired entity to disappear before consuming the
next coalesced result. The regression checks at most two terrain collision models
throughout, including paused updates, and one after the final retirement.
## Imagery continuity during recentering

Replacement generations prepare each outer quadtree page at its configured final
imagery LOD before activation. Failed downloads leave the previous complete view
active. Initial startup retains progressive per-page bootstrap publication.
Same-key resident imagery and pinned decoded images survive generation changes;
late refinements are applied to staging instead of briefly sharpening terrain
that is about to be replaced. Geometry, image composition and target LODs are
unchanged.

`TerrainRuntime_TEST` exercises an uncached replacement during an outage and
checks exact final geometry and pixels against the original builder.
`Ogre2TextureLifecycle_TEST` checks sharp pixels across generation changes and
late refinements, plus front/down camera parity during turning and recentering.
`GuiTerrain_TEST` verifies the optional preview retains detail without leaking
images between datasets. These are targeted regressions, not a new long soak.

