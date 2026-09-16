# Terrain engine and simulator adapters

One `gz-dynamic-terrain-core` library contains the terrain engine. Each build links
that engine to either the Gazebo Sim adapter (Harmonic or Jetty) or the Gazebo
Classic 11 adapter. Binaries must be rebuilt for the selected simulator release.

```text
cmake/GazeboBackend.cmake
include/dynamic_terrain/
  core/                       # neutral types, builders, runtime, small interfaces
  adapters/
    gzsim/                    # renderer, native geography, compatibility, GUI
    classic/                  # collision, renderer, native geography, transport
src/
  core/                       # one copy of every terrain algorithm
  adapters/
    SdfConfig.cc              # compiled for the selected SDFormat family
    CollisionSdf.cc
    gzsim/                    # one system implementation for Harmonic and Jetty
      gui/
    classic/                  # WorldPlugin, VisualPlugin, Ogre1, transport codec
examples/{harmonic,jetty,classic}/
tests/{core,gzsim,classic}/
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

New files provide the boundaries and the additional backend:

- `GeographicTransform.hh`, `TerrainRenderSink.hh`, `TerrainData.hh` and
  `TerrainConfig.hh` define simulator-independent input/output types.
- `TerrainRuntime.hh/.cc` holds the streaming workers extracted from the system
  plugin. `TerrainBuilder.hh/.cc` holds the extracted builder.
- `GzMathCompat.hh` and `GzGeographicTransform.hh` contain the modern geography
  compatibility boundary.
- `ClassicDynamicTerrainPlugin.cc`, `ClassicCollisionAdapter.hh/.cc`,
  `ClassicGeographicTransform.hh`, `ClassicTerrainRenderer.hh/.cc` and
  `ClassicTerrainVisualPlugin.cc` implement the Classic backend.
- `ClassicTerrain.proto`, `ClassicTerrainCodec.hh/.cc` and
  `ClassicTerrainTransport.hh/.cc` implement Classic's page delivery protocol.
- `cmake/GazeboBackend.cmake`, `.github/workflows/build.yml`, the three distro
  example directories, and these architecture/validation documents cover build
  selection and operation.
- Existing deterministic and GUI tests move under `tests/core` and
  `tests/gzsim`. New tests cover core cache/runtime/collision-mesh behavior,
  adapter geography and plugin loading, Classic physics/rendering/transport,
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

The two SDF adapters are compiled against the selected backend's SDFormat release;
neither belongs to the shared terrain library. Collision output accepts an SDF
version so Classic can use its supported schema without altering patch generation.

## Classic lifecycle

The Classic world plugin reads `Model::WorldPose()` and uses the same
`TerrainRuntime`. Classic's image-heightmap loader cannot preserve the core's
16-bit PNG samples, so a shared core helper converts that grid into triangle
data. The Classic adapter serializes it as a native mesh collision, retaining
the sample precision and patch pose. Modern Gazebo keeps its existing heightmap
path. Collision insertion is asynchronous: the adapter waits for the new mesh
shape before starting the old patch's retirement delay. The temporary startup
ground does not depend on a client or render sensor.

Classic's mesh manager retains loaded CPU meshes for the process lifetime.
The collision adapter therefore uses three reusable mesh slots. A slot becomes
available only after its previous model is absent following a physics step;
the adapter then replaces its vertex/index data through the native public API.
When all slots are occupied, the newest pending patch waits for retirement.
This bounds mesh-cache growth during streaming while keeping the old collision
live until replacement is ready.

Classic client and sensor scenes belong to rendering processes/threads. The
world plugin therefore inserts an origin-fixed visual anchor with a
`GZ_REGISTER_VISUAL_PLUGIN` plugin. Each scene requests terrain through Classic
transport. Lightweight manifests identify the current generation and page
textures; individual requests carry one mesh/image page at a time. Missing pages
are retried, new geometry is activated only after complete assembly, and texture
refinements reuse the existing mesh. The wire format preserves doubles and RGB
bytes. Even a maximum 4096-pixel page stays below a 64 MiB message; the server
does not serialize an entire terrain texture set into a single transport message.

The Ogre1 renderer uploads on pre-render callbacks, warms up replacement slots,
evicts off-screen pages, and releases old resources on render callbacks. It has
its own material/texture lifetime management and never calls Ogre2 cleanup.
Visual ownership is weak to avoid a visual/plugin reference cycle. A shared
Classic adapter library ensures the transport schema is registered once when
world and visual plugins load in the same server process.

The split follows Classic's [plugin lifecycle](https://classic.gazebosim.org/tutorials?tut=plugins_hello_world)
and [scene implementation](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/rendering/Scene.cc).

## Modern compatibility

Harmonic retains Math7's `LOCAL2` coordinate convention. Jetty's Math9 uses
`LOCAL` for that corrected convention and a typed coordinate-vector transform
API. `GzMathCompat.hh` translates this one difference, leaving the terrain
engine and Gazebo Sim system source shared. Qt5/Qt6 selection and versioned
Harmonic versus unversioned Jetty package discovery live in CMake.

The source layout deliberately avoids empty placeholders and duplicated terrain
builders. See the README for build, example and validation commands.
