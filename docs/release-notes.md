Stable terrain streaming for Gazebo Harmonic and Jetty.

- Reuse persistent download connections, retry transient failures, validate cached images and bound disposable caches and queued work.
- Stream completed startup pages independently. Keep active terrain and sharp imagery through recentering, delayed refinements and failed replacements.
- Add optional stitched outer coverage for high-altitude camera views without changing existing nearby mesh, imagery or collision settings.
- Retain the optional GUI preview and existing plugin names and SDF configuration.

Extract `gz-dynamic-terrain-plugin-v0.3.0.zip`, install the dependencies in README.md, then run `JOBS=2 ./build.sh harmonic` or `JOBS=2 ./build.sh jetty`. Use `BUILD_GUI=ON` for the optional GUI plugin. The release contains buildable source only; tests, examples, media and binaries remain outside the ZIP.
