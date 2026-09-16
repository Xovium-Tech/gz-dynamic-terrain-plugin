#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISTRO="${1:-harmonic}"
BUILD_GUI="${BUILD_GUI:-OFF}"
BUILD_TESTING="${BUILD_TESTING:-OFF}"

if (( $# > 1 )); then
    echo "Usage: $0 [harmonic|jetty|classic]" >&2
    exit 2
fi
case "$DISTRO" in
    harmonic|jetty) ;;
    classic) BUILD_GUI=OFF ;;
    *)
        echo "Usage: $0 [harmonic|jetty|classic]" >&2
        exit 2
        ;;
esac

JOBS="${JOBS:-$(nproc)}"
if [[ ! "$JOBS" =~ ^[1-9][0-9]*$ ]]; then
    echo "JOBS must be a positive integer." >&2
    exit 2
fi
case "${BUILD_GUI^^}" in
    ON|TRUE|YES|Y|1) BUILD_GUI=ON ;;
    OFF|FALSE|NO|N|0) BUILD_GUI=OFF ;;
    *)
        echo "BUILD_GUI must be ON or OFF." >&2
        exit 2
        ;;
esac
case "${BUILD_TESTING^^}" in
    ON|TRUE|YES|Y|1) BUILD_TESTING=ON ;;
    OFF|FALSE|NO|N|0) BUILD_TESTING=OFF ;;
    *)
        echo "BUILD_TESTING must be ON or OFF." >&2
        exit 2
        ;;
esac

BUILD_DIR="$ROOT/build-$DISTRO"
cmake -S "$ROOT" -B "$BUILD_DIR" \
    -DGZ_DISTRO="$DISTRO" \
    -DBUILD_SIMULATOR=ON \
    -DBUILD_GUI="$BUILD_GUI" \
    -DBUILD_TESTING="$BUILD_TESTING" \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --parallel "$JOBS"

printf '\nBuild complete: %s\n\nPlugin search paths:\n' "$BUILD_DIR"
if [[ "$DISTRO" == classic ]]; then
    printf 'export GAZEBO_PLUGIN_PATH=%q"${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"\n' "$BUILD_DIR"
else
    printf 'export GZ_SIM_SYSTEM_PLUGIN_PATH=%q"${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"\n' "$BUILD_DIR"
    if [[ "$BUILD_GUI" == ON ]]; then
        printf 'export GZ_GUI_PLUGIN_PATH=%q"${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"\n' "$BUILD_DIR"
    fi
fi

printf '\nLibraries (keep these together):\n%s/libgz-dynamic-terrain-core.so\n' "$BUILD_DIR"
if [[ "$DISTRO" == classic ]]; then
    printf '%s/%s\n' \
        "$BUILD_DIR" libdynamic-terrain-classic-adapter.so \
        "$BUILD_DIR" libgazebo-classic-dynamic-terrain.so \
        "$BUILD_DIR" libgazebo-classic-dynamic-terrain-visual.so
else
    printf '%s/libgz-dynamic-terrain-system.so\n' "$BUILD_DIR"
    if [[ "$BUILD_GUI" == ON ]]; then
        printf '%s/libDynamicTerrainGui.so\n' "$BUILD_DIR"
    fi
fi
