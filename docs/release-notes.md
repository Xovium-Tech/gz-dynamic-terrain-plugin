Source-only release for Gazebo Harmonic, Gazebo Jetty and Gazebo Classic 11.

Download **Source code (zip)** or **Source code (tar.gz)** below, install the
dependencies listed in README.md and compile the target matching your simulator:

```bash
JOBS=2 ./build.sh harmonic
# or: JOBS=2 ./build.sh jetty
# or: JOBS=2 ./build.sh classic
```

The default build compiles plugin libraries only. Tests and the optional modern
GUI preview are disabled; contributors can enable them explicitly.

This release simplifies installation and release documentation. The shared
terrain engine and separate simulator adapters are unchanged. See
docs/validation.md for existing runtime test scope and limitations.

No precompiled libraries are attached. Compile locally against your installed
Gazebo version; libraries built for different Gazebo releases are not
interchangeable.
