Minimal source release for Gazebo Harmonic and Jetty.

Download and extract `gz-dynamic-terrain-plugin-v0.2.1.zip`, install the dependencies
listed in `README.md`, and build the target matching your simulator:

```bash
JOBS=2 ./build.sh harmonic
# Or:
JOBS=2 ./build.sh jetty
```

The archive contains source, headers, required source resources, CMake files,
`build.sh`, README, and the BSD 3-Clause LICENSE. No docs directory, images,
examples, tests, development tools, CI files, or prebuilt binaries are included.
GitHub's automatic source ZIP/tar.gz downloads use the same exclusions. The README
includes the world and model setup needed to use the plugin without extra files.

The default build compiles the core and system libraries, with the optional GUI
preview disabled. Set `BUILD_GUI=ON` to build it after installing its dependencies.
Tests remain available in a Git clone.

Since v0.2.0, Gazebo Classic support has been removed from the source. This release
supports Harmonic and Jetty only. Existing Classic installations should remove the
dynamic terrain world plugin and restore their static ground or `uneven_ground`
model. The license now uses Copyright (c) 2026 Xovium Tech, matching the other plugins.

Libraries built for different Gazebo releases are not interchangeable. See the
[validation guide](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/blob/main/docs/validation.md)
for runtime test scope and limitations.
