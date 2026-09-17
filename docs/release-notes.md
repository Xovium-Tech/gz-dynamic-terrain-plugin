Source-only release for Gazebo Harmonic and Gazebo Jetty.

Download **Source code (zip)** or **Source code (tar.gz)** below, install the
dependencies listed in README.md and compile the target matching your simulator:

```bash
JOBS=2 ./build.sh harmonic
# or: JOBS=2 ./build.sh jetty
```

The default build compiles plugin libraries only. Tests and the optional
GUI preview are disabled; contributors can enable them explicitly.

Gazebo Classic dynamic terrain support has been removed, including its adapters,
examples and CI job. Classic / Ubuntu 20.04 installations should remove the
dynamic terrain world plugin and restore their static ground or `uneven_ground`
model. This release does not build or provide Classic plugin libraries.

See docs/validation.md for runtime test scope and limitations.

No precompiled libraries are attached. Compile locally against your installed
Gazebo version; libraries built for different Gazebo releases are not
interchangeable.
