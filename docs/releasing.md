# Minimal source releases

Publish one source ZIP per version for Harmonic and Jetty, matching the camera
and GPS plugins. Users compile the selected backend on their own computer using
`build.sh` and the self-contained [README](../README.md).

The ZIP contains `CMakeLists.txt`, `LICENSE`, `README.md`, `build.sh`, `cmake/`,
`include/`, and `src/`. Keep the GUI QML, QRC, and protobuf files: they are source
resources, and the protobuf code is required even when the GUI is disabled.

`.gitattributes` excludes docs, images, examples, tests, development tools, CI,
and Git metadata files from `git archive` and GitHub's automatic source downloads.
These files remain in the Git repository for development. Do not include build
outputs, cached terrain, or precompiled libraries in a release.

## Validate and commit

Update the project version, HTTP User-Agent, and both startup/configuration log
versions together. Use a new version tag; retain existing release tags and assets.

Review `git status`, run `git diff --check`, and commit the intended changes.
Export that commit to a temporary directory and verify both supported builds:

```bash
release_dir=$(mktemp -d)
git archive HEAD | tar -x -C "$release_dir"
JOBS=2 "$release_dir/build.sh" harmonic
JOBS=2 "$release_dir/build.sh" jetty
```

Verify that the export has no `docs`, `images`, `examples`, `tests`, `tools`, or
`.github` directories. Confirm both the core and system libraries were built.
The optional GUI can also be built with `BUILD_GUI=ON` and the listed dependencies.

Push the commit, then wait for **Build and smoke test** on the
[Actions page](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/actions).
It checks the core and Harmonic/Jetty with the GUI both OFF and ON, including
building the exported source.

## Package and publish

After CI passes, create the new tag at that verified commit. For v0.2.1:

```bash
git tag -a v0.2.1 -m "v0.2.1: minimal source release for Harmonic and Jetty"
git push origin v0.2.1
git archive --format=zip --prefix=gz-dynamic-terrain-plugin-v0.2.1/ \
  --output=/tmp/gz-dynamic-terrain-plugin-v0.2.1.zip v0.2.1
```

Inspect the ZIP file list and compare its `LICENSE` with the repository's license.
Publish a release titled **v0.2.1**, attach that one source ZIP, use
[release-notes.md](release-notes.md) for the description, and mark it latest.
Do not attach binaries, examples, images, tests, or separate documentation.

GitHub also supplies automatic ZIP and tar.gz downloads with the same exclusions.
Verify the uploaded asset and automatic source archive after publication.
