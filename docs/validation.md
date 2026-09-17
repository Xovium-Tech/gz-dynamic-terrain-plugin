# Validation and test coverage

The supported simulator targets are Gazebo Harmonic and Jetty on Ubuntu 24.04.
The [CI workflow](../.github/workflows/build.yml) checks the simulator-independent
core and each target with the optional GUI disabled and enabled. Build results
for a particular commit are available on the
[Actions page](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/actions).

## Build boundaries

`BUILD_SIMULATOR=OFF` builds the core without Gazebo package discovery. Its
dependencies are C++17, Threads, Curl and OpenCV; it does not link Gazebo, Ogre,
Qt or SDFormat.

`BUILD_GUI=OFF` omits Qt and gz-gui discovery from simulator builds. Those builds
produce the core and system libraries; enabling the GUI adds the preview
library. Keep libraries from the same build together so their `$ORIGIN` runtime
paths resolve sibling dependencies.

Each simulator needs a separate build directory. Unsupported distribution
values and attempts to change the distribution in an existing build directory
are rejected during configuration.

## Scope of the tests

The simulator-independent suite verifies deterministic tile/bounds conversion,
provider URLs and cache paths, Terrarium and Terrain-RGB decoding, elevation
sampling, terrain mesh topology/normals/UVs, collision patch alignment and
heightmap encoding, LOD selection and image-resolution caps, progressive page
ordering, bounded page caches, and runtime worker handoff. The runtime test
builds collision before activating any renderer and checks that refinement does
not mutate a published snapshot.

`CollisionSdf_TEST` checks collision serialization independently of Gazebo.
`CheckCoreBoundary` enforces the dependency boundary. Example XML and filename
checks cover the supported system and model configuration aliases.

Geographic tests exercise round trips over multiple origins and headings, at
distances up to 100 km. The system smoke test loads the actual system library
through Gazebo's plugin loader, instantiates both `custom::` aliases, configures
a model, creates collision entities and retires startup ground. It uses a real
ECM but does not run a physics engine.

The Ogre2 regression exercises resource cleanup, repeated terrain generations,
camera-based eviction/reload and mountain-scene pixel continuity. The GUI test
loads the preview plugin and checks transport, rendering events and cleanup.
Engines are selected from the imported CMake target to avoid loading another
distribution's similarly named renderer during isolated-prefix testing.

Terrain test fixtures use synthetic local data; no provider credentials or
online imagery downloads are needed.

## Reproducing the checks

Install the selected simulator's dependencies as described in the
[README](../README.md). These commands explicitly enable tests; `build.sh`
disables them by default.

```bash
cmake -S . -B build-core -DBUILD_SIMULATOR=OFF -DBUILD_GUI=OFF -DBUILD_TESTING=ON
cmake --build build-core --parallel 2
ctest --test-dir build-core --output-on-failure

cmake -S . -B build-harmonic -DGZ_DISTRO=harmonic -DBUILD_GUI=ON -DBUILD_TESTING=ON -DBUILD_RENDER_TESTS=ON
cmake --build build-harmonic --parallel 2
xvfb-run -a env LIBGL_ALWAYS_SOFTWARE=1 ctest --test-dir build-harmonic --output-on-failure

cmake -S . -B build-jetty -DGZ_DISTRO=jetty -DBUILD_GUI=ON -DBUILD_TESTING=ON -DBUILD_RENDER_TESTS=ON
cmake --build build-jetty --parallel 2
xvfb-run -a env LIBGL_ALWAYS_SOFTWARE=1 ctest --test-dir build-jetty --output-on-failure
```

Graphics checks need Xvfb and Mesa software rendering for the commands above,
or an available display when running `ctest` directly. Test `BUILD_GUI=OFF` in a
separate build directory as well to check the server-only dependency boundary.

With an extracted Harmonic prefix, set its `LD_LIBRARY_PATH`,
`GZ_RENDERING_RESOURCE_PATH`, `QT_PLUGIN_PATH` and `QML2_IMPORT_PATH` so resources
and plugins come from Harmonic. These extra paths are unnecessary in an ordinary
single-distribution installation.

## Limits

Compilation and short component/runtime tests do not establish full simulator
compatibility. The suite does not cover PX4 flight, production imagery/elevation
services, long-duration streaming, every physics backend, or every camera type.
See [releasing.md](releasing.md) for source-only publication. Users compile
locally; validation libraries are not uploaded as release assets.
