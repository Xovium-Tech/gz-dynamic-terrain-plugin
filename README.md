# Dynamic Terrain Plugin

[![CI](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/actions/workflows/build.yml)
[![Latest release](https://img.shields.io/github/v/release/Xovium-Tech/gz-dynamic-terrain-plugin)](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/releases/latest)

Stream map imagery and elevation terrain around a moving model in **Gazebo
Harmonic or Jetty**. One shared terrain engine handles tile downloads,
caching, mesh generation, collision terrain and progressive texture detail.

[![Watch the terrain demo](https://xovium.tech/media/dynamic-terrain-demo-140cfb8db1ca/preview.gif)](https://xovium.tech/videos/dynamic-terrain/)

**Source-only distribution:** download or clone the source and compile it on
the computer running Gazebo. Releases use GitHub's automatic source ZIP/tar.gz
archives. No precompiled plugin libraries or separate distro packages are
published.

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

Alternatively, download **Source code (zip)** or **Source code (tar.gz)** from a
[release](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/releases),
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

Configure the simulator using the matching example and the
[simulation setup guide](docs/setup.md):

The [Harmonic](examples/harmonic/plugin_snippet.sdf) and
[Jetty](examples/jetty/plugin_snippet.sdf) examples use the world system
`custom::DynamicTerrainSystem` plus model configuration
`custom::DynamicTerrainConfig`. Terrain rendering needs the Ogre2 Sensors
system and an active camera sensor.

Keep the core and system libraries from each build together. Internet access
is needed for terrain tiles not already cached.

## Optional GUI and tests

For the GUI preview, install `libgz-gui8-dev`, `qtbase5-dev` and
`qtdeclarative5-dev` for Harmonic, or `libgz-gui10-dev`, `qt6-base-dev` and
`qt6-declarative-dev` for Jetty. Then build with `BUILD_GUI=ON`:

```bash
JOBS=2 BUILD_GUI=ON ./build.sh jetty
export GZ_GUI_PLUGIN_PATH="$PWD/build-jetty${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"
```

Contributors can enable tests with `BUILD_TESTING=ON`. For simulator-independent
tests:

```bash
cmake -S . -B build-core -DBUILD_SIMULATOR=OFF -DBUILD_GUI=OFF -DBUILD_TESTING=ON
cmake --build build-core --parallel 2
ctest --test-dir build-core --output-on-failure
```

## Documentation

- [Configuration, providers and cache settings](docs/configuration.md)
- [World/model setup and PX4 integration](docs/setup.md)
- [Architecture](docs/architecture.md)
- [Validation and test coverage](docs/validation.md)
- [Source-only release procedure](docs/releasing.md)

## License

BSD 3-Clause. Copyright (c) 2026 Alex Chazov. See [LICENSE](LICENSE).
