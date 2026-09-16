# Releasing v0.2.0

One source tag covers Harmonic, Jetty and Classic 11. Each simulator needs a
separate binary package built against its own dependencies. Primary binary
packages use `BUILD_GUI=OFF`; optional modern GUI packages should have distinct
`-gui` asset names.

## Commit and push

Use the existing `github-xovium` SSH alias for this repository:

```bash
git remote set-url origin git@github-xovium:Xovium-Tech/gz-dynamic-terrain-plugin.git
ssh -T git@github-xovium
git remote -v
git config --local --get user.name
git config --local --get user.email
git fetch origin --tags
git tag --list v0.2.0
```

GitHub's successful SSH greeting identifies the account and normally exits with
status 1 because GitHub does not provide shell access. Confirm the commit email
is verified on the intended GitHub account. The tag query must be empty before
creating a new `v0.2.0` tag.

Review the complete source migration, including new files and deleted old paths:

```bash
git status --short
git diff --check
git add -A
git diff --cached --stat
git diff --cached --check
git commit -m "Add Harmonic, Jetty and Classic 11 support"
git pull --rebase origin main
git push origin main
```

Resolve any rebase conflicts before pushing. If branch protection requires a
pull request, push a feature branch and merge through the repository's normal
review process instead.

## Wait for CI, then tag

On the repository's [Actions page](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/actions),
wait for **Build and smoke test** to pass for all six configurations:

- Core only.
- Harmonic with GUI OFF and ON.
- Jetty with GUI OFF and ON.
- Classic 11.

Then update the local branch and confirm the resulting commit is the one that
passed CI. If it changed, wait for that commit's checks before tagging.

```bash
git switch main
git pull --ff-only origin main
git log -1 --format='%H %s'
git tag -a v0.2.0 -m "v0.2.0: Gazebo Harmonic, Jetty and Classic 11"
git push origin v0.2.0
git show v0.2.0 --no-patch
```

## Build release archives

Run the following recipe independently in each clean environment, with the
matching Gazebo and system development packages installed as described in the
README. Use a fresh checkout of the published tag and a fresh build directory.
Do not package the earlier local validation builds: they use custom OpenCV 4.9
or a temporary Harmonic SDK prefix. The CI workflow currently builds and tests;
it does not upload archives or publish releases.

| `release_distro` | `release_platform` | Environment |
| --- | --- | --- |
| `harmonic` | `ubuntu24.04-amd64` | Ubuntu 24.04 + Harmonic |
| `jetty` | `ubuntu24.04-amd64` | Ubuntu 24.04 + Jetty |
| `classic` | `ubuntu20.04-amd64` | Ubuntu 20.04 + Classic 11 |

Select the two variables from the table for each environment:

```bash
git clone --branch v0.2.0 --depth 1 \
  https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin.git
cd gz-dynamic-terrain-plugin

release_distro=harmonic
release_platform=ubuntu24.04-amd64
release_build="build-release-$release_distro"
release_package="gz-dynamic-terrain-v0.2.0-$release_distro-$release_platform"
release_stage=$(mktemp -d)

cmake -S . -B "$release_build" \
  -DGZ_DISTRO="$release_distro" \
  -DBUILD_GUI=OFF \
  -DBUILD_TESTING=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_LIBDIR=lib \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
cmake --build "$release_build" --parallel 2
cmake --install "$release_build" --prefix "$release_stage/$release_package"

cp LICENSE README.md "$release_stage/$release_package/"
cp -R docs "$release_stage/$release_package/"
mkdir -p "$release_stage/$release_package/examples"
cp -R "examples/$release_distro" "$release_stage/$release_package/examples/"

for release_library in "$release_stage/$release_package/lib/"*.so; do
  ldd "$release_library"
  readelf -d "$release_library" | grep -E 'NEEDED|RPATH|RUNPATH'
done
```

Before packaging, confirm there are no `not found` dependencies, custom OpenCV
paths or temporary SDK paths. OpenCV must resolve from the normal system
directories. Modern dependencies must match the selected Gazebo family. These
archives require the matching Gazebo/runtime packages; they are not standalone
bundles of Gazebo or OpenCV.

Harmonic and Jetty each install the matching core and system libraries. Classic
must install all four:

```text
lib/libgz-dynamic-terrain-core.so
lib/libdynamic-terrain-classic-adapter.so
lib/libgazebo-classic-dynamic-terrain.so
lib/libgazebo-classic-dynamic-terrain-visual.so
```

After checking linkage, create the archive:

```bash
mkdir -p build-release-assets
tar -C "$release_stage" \
  -czf "build-release-assets/$release_package.tar.gz" "$release_package"
tar -tzf "build-release-assets/$release_package.tar.gz"
```

Collect the three archives into one `build-release-assets` directory, then run:

```bash
cd build-release-assets
sha256sum gz-dynamic-terrain-v0.2.0-*.tar.gz > SHA256SUMS
sha256sum -c SHA256SUMS
```

## Publish the GitHub release

Open [Create a new release](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/releases/new),
select the existing `v0.2.0` tag, and use the title
**v0.2.0 — Gazebo Harmonic, Jetty and Classic 11**.

Use [release-notes-v0.2.0.md](release-notes-v0.2.0.md) as the release body. Attach:

```text
gz-dynamic-terrain-v0.2.0-harmonic-ubuntu24.04-amd64.tar.gz
gz-dynamic-terrain-v0.2.0-jetty-ubuntu24.04-amd64.tar.gz
gz-dynamic-terrain-v0.2.0-classic-ubuntu20.04-amd64.tar.gz
SHA256SUMS
```

Review the files, notes and selected tag, then publish. The same release can be
published with source archives only if prebuilt packages are not ready; in that
case describe it as a source release and omit claims that binary assets are
attached.
