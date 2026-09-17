# Terrain engine and simulator adapters

One `gz-dynamic-terrain-core` library contains the terrain engine. Each build links
that engine to the Gazebo Sim adapter for Harmonic or Jetty. Binaries must be
rebuilt for the selected simulator release.

```text
cmake/GazeboBackend.cmake
include/dynamic_terrain/
  core/                       # neutral types, builders, runtime, small interfaces
  adapters/
    gzsim/                    # renderer, native geography, compatibility, GUI
src/
  core/                       # one copy of every terrain algorithm
  adapters/
    SdfConfig.cc              # compiled for the selected SDFormat family
    CollisionSdf.cc
    gzsim/                    # one system implementation for Harmonic and Jetty
      gui/
examples/{harmonic,jetty}/
tests/{core,gzsim}/
.github/workflows/build.yml
```

## Original source classification

| Original source | Responsibility | Destination |
| --- | --- | --- |
| `TerrainTypes.hh/.cc` | A: tile math, providers, configuration defaults, normalization, logging | `core/TerrainTypes`, `core/TerrainConfig` |
| `TerrainTypes.hh/.cc` | B: entity configuration registry and native geographic conversion | `adapters/gzsim/ModelConfigRegistry`, `GzGeographicTransform` |
| `TerrainTypes.cc` | E: SDF configuration parsing | `adapters/SdfConfig` |
| `TileStore.hh/.cc` | A: HTTP downloads, disk and DEM caches, interpolation | `core/TileStore` |
| `PersistentTerrain.hh/.cc` | A: mesh/image generation, page cache, progressive imagery and LOD | `core/TerrainBuilder`, `TerrainData` |
| `PersistentTerrain.hh/.cc` | C: render callbacks, frustum residency, mesh/material/texture lifetime | `adapters/gzsim/GzTerrainRenderer` |
| `CollisionTerrain.hh/.cc` | A: collision sampling, bounds, heightmap encoding | `core/CollisionTerrain` |
| `CollisionTerrain.cc` | E: collision SDF serialization | `adapters/CollisionSdf` |
| `DynamicTerrainSystem.cc` | A: request generations, streaming workers, retry and refinement scheduling | `core/TerrainRuntime` |
| `DynamicTerrainSystem.cc` | B/E: plugin registration, world/model lookup, pose, collision entities, startup ground | `adapters/gzsim/DynamicTerrainSystem` |
| `Ogre2ResourceCleanup.hh/.cc` | C: Ogre2 GPU resource cleanup | `adapters/gzsim/Ogre2ResourceCleanup` |
| `GuiTerrain.hh/.cc/.proto` | D: preview serialization and Gazebo transport | `adapters/gzsim/gui/GuiTerrain` |
| `DynamicTerrainGui.cc/.qml/.qrc` | D: Qt GUI plugin | `adapters/gzsim/gui/DynamicTerrainGui` |

The following files define the core and adapter boundaries:

- `GeographicTransform.hh`, `TerrainRenderSink.hh`, `TerrainData.hh` and
  `TerrainConfig.hh` define simulator-independent input/output types.
- `TerrainRuntime.hh/.cc` holds the streaming workers extracted from the system
  plugin. `TerrainBuilder.hh/.cc` holds the extracted builder.
- `GzMathCompat.hh` and `GzGeographicTransform.hh` contain the geography
  compatibility boundary.
- `cmake/GazeboBackend.cmake`, `.github/workflows/build.yml`, the two distro
  example directories, and these architecture/validation documents cover build
  selection and operation.
- Tests under `tests/core` and `tests/gzsim` cover core caches and runtime,
  adapter geography, plugin loading, rendering, preview transport,
  SDF serialization and the core dependency boundary.

## Boundaries and ownership

`GeographicTransform` translates plain latitude/longitude/elevation and `Vec3`
values. The Gazebo Sim implementation retains the original `LOCAL2` conversion,
including heading and elevation reference. The core does not implement a second
approximation of Gazebo's spherical-coordinate math.

`TerrainRuntime` owns the original visual, refinement and collision worker queues.
Simulation callbacks submit current positions and consume completed collision
patches. Simulator entities, physics objects, SDF parsing and lifecycle callbacks
remain in adapters. Destroying the runtime stops and joins its workers before
rendering and preview sinks are destroyed.

`TerrainRenderSink` is the small rendering boundary: immutable terrain snapshots,
texture updates and activation status. A snapshot carries generation, bounds,
page mesh arrays (positions, normals, UVs and triangle indices), and RGB image
bytes. The runtime copies the snapshot's page list before handing it off;
immutable mesh/image storage can be shared. Each renderer owns its native GPU
resources and performs uploads on its render event thread.

The Gazebo Sim renderer retains staging/active/retired generations, warmup,
frustum eviction, texture reference counts and deferred Ogre2 cleanup. GUI
preview remains an optional Qt plugin; its server-side transport service has no
Qt dependency. `BUILD_GUI=OFF` omits the Qt plugin.

The SDF adapters are compiled against the selected distribution's SDFormat
release; neither belongs to the shared terrain library. Collision output
serializes the core's sampled 16-bit heightmap for Gazebo Sim.

## Harmonic and Jetty compatibility

Harmonic retains Math7's `LOCAL2` coordinate convention. Jetty's Math9 uses
`LOCAL` for that corrected convention and a typed coordinate-vector transform
API. `GzMathCompat.hh` translates this one difference, leaving the terrain
engine and Gazebo Sim system source shared. Qt5/Qt6 selection and versioned
Harmonic versus unversioned Jetty package discovery live in CMake.

The source layout deliberately avoids empty placeholders and duplicated terrain
builders. See the README for build, example and validation commands.
