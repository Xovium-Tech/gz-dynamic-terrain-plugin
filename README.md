# Dynamic Terrain Plugin

[![CI](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/actions/workflows/build.yml)
[![Latest release](https://img.shields.io/github/v/release/Xovium-Tech/gz-dynamic-terrain-plugin)](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/releases/latest)

Stream map imagery and elevation terrain around a moving model in **Gazebo
Harmonic or Jetty**. One shared terrain engine handles tile downloads,
caching, mesh generation, collision terrain and progressive texture detail.

[![Watch the terrain demo](https://xovium.tech/media/dynamic-terrain-demo-140cfb8db1ca/preview.gif)](https://xovium.tech/videos/dynamic-terrain/)

**Source-only distribution:** download the release ZIP and compile it on the
computer running Gazebo. The archive contains source, headers, CMake files,
`build.sh`, this README, and LICENSE. It excludes docs, images, examples, tests,
development tools, CI files, and prebuilt libraries. GitHub's automatic source
ZIP/tar.gz downloads use the same exclusions.

## Supported Gazebo versions

| Build target | Simulator | Documented build environment |
| --- | --- | --- |
| `harmonic` | Gazebo Harmonic / gz-sim8 | Ubuntu 24.04 |
| `jetty` | Gazebo Jetty / gz-sim10 | Ubuntu 24.04 |

Choose the target matching your installed simulator. Each target uses a separate
build directory; compiled libraries are specific to that Gazebo version.

Gazebo Classic dynamic terrain support has been removed. Existing Classic / Ubuntu
20.04 installations should use their static ground or `uneven_ground` model and
remove the dynamic terrain world plugin. This source no longer builds Classic
plugin libraries.

## 1. Install dependencies

Install the chosen simulator using its official instructions:
[Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/) or
[Jetty](https://gazebosim.org/docs/jetty/install_ubuntu/).

Install the common build dependencies:

```bash
sudo apt update
sudo apt install build-essential git cmake pkg-config \
  libcurl4-openssl-dev libopencv-dev libprotobuf-dev protobuf-compiler
```

Then install the development packages for **your chosen target only**, with the
Gazebo package repository configured:

```bash
# Harmonic / Ubuntu 24.04
sudo apt install libgz-sim8-dev libgz-plugin2-dev libgz-rendering8-ogre2-dev

# Jetty / Ubuntu 24.04
sudo apt install libgz-sim10-dev libgz-plugin4-dev libgz-rendering10-ogre2-dev
```

The default build does not discover Qt or gz-gui. Gazebo's distribution packages
may still install GUI dependencies through the package manager.

## 2. Get the source and build

```bash
git clone --depth 1 https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin.git
cd gz-dynamic-terrain-plugin
```

Alternatively, download `gz-dynamic-terrain-plugin-v0.2.1.zip` from the
[latest release](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/releases/latest),
extract it and open a terminal in the extracted directory.

Run **one** command for your simulator:

```bash
JOBS=2 ./build.sh harmonic
JOBS=2 ./build.sh jetty
```

The helper builds only the plugin libraries by default, in Release mode, with
tests and the optional GUI preview disabled. Increase `JOBS` if you have
enough RAM for more parallel compilation.

Equivalent CMake commands, using Jetty as an example:

```bash
cmake -S . -B build-jetty -DGZ_DISTRO=jetty \
  -DBUILD_GUI=OFF -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build-jetty --parallel 2
```

Replace `jetty` in both places with `harmonic` for a Harmonic build.

## 3. Load the plugin

The build helper prints the required search path. Set it in the terminal where
you launch Gazebo or PX4:

```bash
# Harmonic; replace build-harmonic with build-jetty for Jetty.
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build-harmonic${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
```

Keep `libgz-dynamic-terrain-core.so` and `libgz-dynamic-terrain-system.so`
from the same build together. Internet access is needed for terrain tiles not
already cached.

Add these systems inside your world's `<world>` element, retaining the world's
other systems, including Physics. Keep an existing Sensors entry rather than
adding a duplicate:

```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
<plugin filename="libgz-dynamic-terrain-system.so"
        name="custom::DynamicTerrainSystem"/>
```

For PX4, put these entries inside `<plugins>` in
`src/modules/simulation/gz_bridge/server.config`, adding `entity_name="*"`
and `entity_type="world"` to each plugin. Load each system once, with the terrain
system after Sensors. Terrain rendering needs an active camera sensor and a
usable Ogre2 renderer.

Set a geographic origin inside `<world>`, replacing these coordinates and
elevation with your starting location:

```xml
<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>37.4319</latitude_deg>
  <longitude_deg>-122.1697</longitude_deg>
  <elevation>30</elevation>
  <heading_deg>0</heading_deg>
</spherical_coordinates>
```

Add this configuration directly inside the `<model>` that terrain should follow:

```xml
<plugin filename="libgz-dynamic-terrain-system.so"
        name="custom::DynamicTerrainConfig">
  <imagery_provider>google_satellite</imagery_provider>
  <elevation_provider>terrarium</elevation_provider>
  <visual_radius_m>7500</visual_radius_m>
  <visual_page_texture_max_size>2048</visual_page_texture_max_size>
  <enable_collision>true</enable_collision>
  <align_origin_to_ground>true</align_origin_to_ground>
  <cache_dir>~/.cache/gz_dynamic_terrain</cache_dir>
  <diagnostics>true</diagnostics>
</plugin>
```

Omitting `camera_names` considers all cameras in the server scene. To select
specific cameras, add `<camera_names>camera_front,camera_down</camera_names>`
using only sensor names that exist in your model. The renderer waits for all
listed cameras before evicting off-screen pages.

By default, terrain at the geographic origin is aligned to local Z = 0. Set
`align_origin_to_ground` to `false` to retain source elevation relative to the
world reference. Remove or reposition any ground plane that overlaps the
terrain. For lower VRAM use, reduce the visual radius or texture size to `1024`.
The disk cache has no automatic size limit.

Start your configured world with `gz sim -r /path/to/your_world.sdf`, or launch
your existing PX4 target from the same terminal. The first load downloads tiles;
view the terrain through a camera sensor and check `[DynamicTerrain]` messages
for download or setup errors. Provider settings and cache details are covered in
the [configuration guide](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/blob/main/docs/configuration.md).

## Optional GUI and tests

For the GUI preview, install `libgz-gui8-dev`, `qtbase5-dev` and
`qtdeclarative5-dev` for Harmonic, or `libgz-gui10-dev`, `qt6-base-dev` and
`qt6-declarative-dev` for Jetty. Then build with `BUILD_GUI=ON`:

```bash
JOBS=2 BUILD_GUI=ON ./build.sh jetty
export GZ_GUI_PLUGIN_PATH="$PWD/build-jetty${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"
```

Tests and development fixtures are available in a Git clone, not in release
archives. Contributors can enable tests with `BUILD_TESTING=ON`. For
simulator-independent tests:

```bash
cmake -S . -B build-core -DBUILD_SIMULATOR=OFF -DBUILD_GUI=OFF -DBUILD_TESTING=ON
cmake --build build-core --parallel 2
ctest --test-dir build-core --output-on-failure
```

## Documentation

Detailed documentation is available online and is excluded from release archives.

- [Configuration, providers and cache settings](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/blob/main/docs/configuration.md)
- [World/model setup and PX4 integration](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/blob/main/docs/setup.md)
- [Architecture](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/blob/main/docs/architecture.md)
- [Validation and test coverage](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/blob/main/docs/validation.md)
- [Source-only release procedure](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/blob/main/docs/releasing.md)

## License

BSD 3-Clause. Copyright (c) 2026 Xovium Tech. See [LICENSE](LICENSE).
