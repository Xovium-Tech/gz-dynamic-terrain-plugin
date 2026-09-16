# Validation and test coverage

These are local checks for v0.2.0 and the v0.2.1 source-only build workflow,
performed on September 16, 2026. They distinguish
compilation from short component/runtime tests; they are not a claim of complete
simulator compatibility or long-running flight validation.

## Build environments

- Core: Ubuntu 24.04, GCC 13.3, C++17, Curl and OpenCV 4.9; no Gazebo CMake discovery.
- Harmonic: official Harmonic development/runtime packages extracted into an
  isolated prefix under `/tmp`; gz-sim8 8.15.0, gz-rendering8 8.2.3, Math7 and Qt5.
- Jetty: installed Ubuntu 24.04 packages; gz-sim10 10.5.0,
  gz-rendering10 10.0.2, Math9 and Qt6. A fresh headless v0.2.0 build was also
  checked with Ubuntu's system OpenCV 4.6 instead of the custom OpenCV 4.9.
- Classic: separate Ubuntu 20.04 root filesystem under `/tmp`, executed with
  PRoot, using GCC 9, Gazebo Classic 11.15.1, SDFormat 9.10, Ogre 1.9 and
  OpenCV 4.2. No Classic packages were installed into the host's Jetty environment.

Modern `BUILD_GUI=OFF` configurations were checked for absence of Qt and gz-gui
package discovery. The core library's dynamic dependency table contains
OpenCV/Curl/C++ runtime libraries, with no Gazebo, Ogre, Qt or SDFormat links.
Unsupported distro values and attempts to change the distro in an existing build
directory were confirmed to fail with explicit configuration errors.

Installation checks produced the three modern libraries (including optional
GUI) and all four Classic libraries. The adapter/plugin libraries use `$ORIGIN`
runtime paths to find their sibling libraries. The modern server library has
no direct Qt or gz-gui dependency.

## Results

| Build | Local result |
| --- | --- |
| Core only | 8/8 tests passed; no simulator package discovery |
| Harmonic, GUI off | System/core compiled; no Qt or gz-gui discovery |
| Harmonic, GUI on | 13/13 tests passed, including Ogre2 and Qt5 plugin runtime tests |
| Jetty, GUI off, system OpenCV 4.6 | 11/11 tests passed; no Qt or gz-gui discovery; installed libraries resolve system OpenCV `.406` |
| Jetty, GUI on | 13/13 tests passed, including Ogre2 and Qt6 plugin runtime tests |
| Classic, GUI off, render tests on | 13/13 tests passed, including ODE contact, collision-cache reuse and Ogre1 visual-plugin runtime tests |

## Scope of the tests

The simulator-independent suite verifies deterministic tile/bounds conversion,
provider URLs/cache paths, Terrarium and Terrain-RGB decoding, DEM/cache
retention, terrain mesh topology/normals/UVs, collision patch alignment and
heightmap encoding, LOD selection and image-resolution caps, progressive page
ordering, bounded page caches, and runtime worker handoff. The runtime test
builds collision before activating any renderer and checks that refinement does
not mutate a published snapshot. `CollisionMesh_TEST` verifies the neutral mesh
conversion's 16-bit sample precision, north/south orientation and triangle winding.

`CollisionSdf_TEST` checks adapter serialization independently of Gazebo.
`CheckCoreBoundary` enforces the dependency boundary. Example XML files parse,
and filename tests cover the preserved modern aliases plus Classic's world
plugin and tracked-model configuration.

Modern geographic tests preserve the original 56 round trips over multiple
origins/headings and distances up to 100 km. The system smoke test loads the
actual system library through Gazebo's plugin loader, instantiates both
`custom::` aliases, configures a model, creates collision entities and retires
startup ground. It uses a real ECM but does not run a physics engine.

The Ogre2 regression exercises resource cleanup, 20 terrain generations,
camera-based eviction/reload and mountain-scene pixel continuity. The GUI test
loads the actual preview plugin and checks transport, rendering events and
cleanup. Engines are selected from the imported CMake target to avoid loading a
different distro's similarly named renderer during isolated-prefix testing.

Classic's Ogre1 test renders five terrain generations, reads camera pixels,
applies texture refinement and verifies texture/material counts return to the
baseline after destruction. It then loads the actual visual plugin, receives an
already-published generation through manifest/page requests, renders a later
texture update, and unloads the plugin with resource counts back at baseline.
This runs in the isolated Classic environment with Xvfb and software OpenGL.

Classic's world smoke test loads the actual world plugin, builds terrain from
offline fixtures, waits for startup ground to retire, and drops a sphere onto
the resulting mesh using ODE. Moving the tracked aircraft triggers a replacement
patch; the test observes overlapping old/new patches before the old one is
removed, then shuts the world down. It runs without an X display, independently
of a rendering client.

`ClassicCollisionPool_TEST` cycles through seven different sampled terrain
heights and checks the sphere's resting height after each replacement. It then
fills all three mesh slots, submits two waiting updates and verifies that the
newest update eventually produces the expected contact height. The native mesh
cache and generated mesh files remain limited to three slots.

The Classic render/transport regression also delays refined imagery until the
bootstrap generation is visible, checking that ongoing refinement cannot block
initial display. A maximum configured page (128 cells per side and a 4096-pixel
RGB texture) serialized to 51,594,393 bytes (49.2042 MiB). Sender and receiver
enforce 51 MiB per page and 4 MiB per manifest.

All terrain test fixtures use synthetic local data; no provider credentials or
online imagery downloads are needed.

The build helper passed shell syntax and mocked command checks for all three
distros, optional modern GUI builds, default settings and invalid input. The
fresh Jetty plugin-loading test confirmed both startup/configuration messages
report `v0.2.0`. Its installed libraries have no missing dependencies or custom
OpenCV/CUDA paths; this was a local validation build, not a published archive.

For v0.2.1, a fresh copy of the source without `.git` or generated files was
configured for Jetty with system OpenCV 4.6 and built using
`JOBS=2 ./build.sh jetty`. It produced only the core and system runtime libraries, with no GUI or
test targets. Dependencies resolved without missing or custom OpenCV/CUDA
libraries, and compiled version strings reported 0.2.1. The helper also passed
15 mocked configurations and three invalid-input checks, including explicit
test enablement. The simulator algorithms and CI matrix were unchanged.

## Reproducing the checks

Use the isolated distro environments described above, then configure each build
as shown in the README. To include graphics checks, configure with
`-DBUILD_RENDER_TESTS=ON`; use `-DBUILD_GUI=ON` for the modern GUI checks.

```bash
cmake -S . -B build-core -DBUILD_SIMULATOR=OFF -DBUILD_GUI=OFF
cmake --build build-core --parallel 2
ctest --test-dir build-core --output-on-failure

cmake -S . -B build-harmonic -DGZ_DISTRO=harmonic -DBUILD_GUI=ON -DBUILD_RENDER_TESTS=ON
cmake --build build-harmonic --parallel 2
ctest --test-dir build-harmonic --output-on-failure

cmake -S . -B build-jetty -DGZ_DISTRO=jetty -DBUILD_GUI=ON -DBUILD_RENDER_TESTS=ON
cmake --build build-jetty --parallel 2
ctest --test-dir build-jetty --output-on-failure

# Inside a separate Classic 11 / Ubuntu 20.04 environment:
cmake -S . -B build-classic -DGZ_DISTRO=classic -DBUILD_GUI=OFF -DBUILD_RENDER_TESTS=ON
cmake --build build-classic --parallel 2
(cd build-classic && xvfb-run -a env LIBGL_ALWAYS_SOFTWARE=1 ctest --output-on-failure)
```

The modern graphics commands above use an available display. The CI workflow
uses Xvfb and Mesa software rendering. With an extracted Harmonic prefix, also
set its `LD_LIBRARY_PATH`, `GZ_RENDERING_RESOURCE_PATH`, `QT_PLUGIN_PATH` and
`QML2_IMPORT_PATH` so resources and plugins come from Harmonic. These extra
paths are unnecessary in an ordinary single-distro installation.

## Limits

The CI workflow has been added but has not been executed on GitHub by this task.
The local tests do not cover PX4 flight, production imagery/elevation services,
long-duration streaming, every physics backend, or every camera type. See
[releasing.md](releasing.md) for source-only publication. Users compile locally;
the local validation libraries are not uploaded as release assets.
