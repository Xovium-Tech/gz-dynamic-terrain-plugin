Adds Gazebo Jetty and Gazebo Classic 11 support while preserving Gazebo Harmonic
support.

## Highlights

- One shared terrain engine for downloading, caching, elevation processing,
  visual mesh generation, collision generation and terrain streaming.
- One Gazebo Sim adapter shared by Harmonic and Jetty.
- Separate Classic world, collision, transport and Ogre1 rendering adapters.
- Preserved `custom::DynamicTerrainSystem` and `custom::DynamicTerrainConfig`
  aliases and the existing modern system/core library filenames.
- Explicit distro selection and optional modern GUI builds.
- Simulator-independent core tests and isolated CI coverage for all three targets.
- Distro-aware build helper and matching configuration examples.

## Validation

Local checks passed for the core, Harmonic system/Ogre2/Qt5 GUI, Jetty
system/Ogre2/Qt6 GUI, and Classic world/collision/transport/Ogre1 rendering.
Tests cover plugin loading, collision replacement, cache reuse, rendering
lifecycle and shutdown. See `docs/validation.md` for exact scope and results.

Long-duration PX4 flight and production imagery/elevation services are outside
this validation. Modern system smoke tests check entity creation rather than
physics-engine contact; Classic contact tests use ODE.

## Binary targets

- Harmonic: Ubuntu 24.04 amd64.
- Jetty: Ubuntu 24.04 amd64.
- Classic 11: Ubuntu 20.04 amd64.

Primary packages use `BUILD_GUI=OFF` and require the matching Gazebo/runtime
dependencies. The modern `DynamicTerrainGui` preview remains available to build
from source. Harmonic, Jetty and Classic binaries are not interchangeable.
