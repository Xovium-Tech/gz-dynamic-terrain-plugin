# Simulation setup

Build the plugin for your simulator and set its plugin search path as described
in the [README](../README.md). The modern setup below applies to Harmonic and
Jetty; [Classic setup](#classic-setup) uses a world plugin.

## Server setup

For PX4, open `<PX4_PATH>/src/modules/simulation/gz_bridge/server.config`. Add the terrain system inside `<plugins>`, after the existing Ogre2 Sensors system:

```xml
<plugin entity_name="*" entity_type="world"
        filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>

<plugin entity_name="*" entity_type="world"
        filename="libgz-dynamic-terrain-system.so"
        name="custom::DynamicTerrainSystem"/>
```

Keep the other PX4 systems and load each system only once. The [example server configuration](../examples/server.config) shows the placement; do not replace your configuration if it contains other plugins you need.

For a standalone Gazebo world, add the same two plugin entries directly inside `<world>`, without the `entity_name` and `entity_type` attributes. Keep the world's other systems, including Physics.

## World setup

Set the geographic origin inside your world's `<world>` element:

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

Replace the latitude, longitude, and elevation with your starting location. The plugin uses Gazebo's geographic transform, including the world heading, to position terrain.

By default, `align_origin_to_ground` shifts the downloaded elevation so the ground at the origin is at local Z = 0. Set it to `false` to keep the source elevation relative to the world's elevation reference. Remove or reposition any existing ground plane that would overlap the generated terrain.

## Model setup

Copy the block from [examples/plugin_snippet.sdf](../examples/plugin_snippet.sdf) into the `<model>` that terrain should follow, not into a link or camera sensor. For PX4's Cessna, this is usually `<PX4_PATH>/Tools/simulation/gz/models/rc_cessna/model.sdf`.

Set `camera_names` to the camera sensor names in your model:

```xml
<camera_names>camera_front,camera_down</camera_names>
```

Only list cameras that exist. The renderer waits until all listed cameras are available before evicting off-screen pages. Omit this setting to consider all cameras in the server scene.

The example uses a 7.5 km terrain radius and concentrates detailed imagery around the terrain patch below the aircraft. Despite its name, `bottom_camera_only` selects a ground-distance region, not the exact footprint or direction of a camera.

For video streaming, the separate [GstPlaneCameraSystem plugin](https://github.com/Xovium-Tech/px4-gazebo-gstreamer-camera-plugin) can be used with the same cameras. It is not required by the terrain plugin.

## Start the simulation

After configuring the server, world, and model, start your PX4 target:

```bash
cd /path/to/PX4-Autopilot
make px4_sitl gz_rc_cessna
```

Or start your configured standalone world:

```bash
gz sim -r /path/to/your_world.sdf
```

The first load takes longer while imagery and elevation tiles download. View the terrain through a camera sensor. With `diagnostics` enabled, messages prefixed with `[DynamicTerrain]` report downloads, terrain updates, and resource usage.

## Classic setup

Load the world plugin under `<world>`, with `<tracked_model>` naming the model to
follow. Put the same terrain configuration element names used above directly
inside this world plugin. The complete
[Classic example](../examples/classic/plugin_snippet.world) includes a geographic
origin and camera model:

```xml
<plugin name="dynamic_terrain" filename="libgazebo-classic-dynamic-terrain.so">
  <tracked_model>vehicle</tracked_model>
  <imagery_provider>google_satellite</imagery_provider>
  <elevation_provider>terrarium</elevation_provider>
</plugin>
```

The world plugin inserts a visual anchor which loads the Ogre1 visual plugin in
rendering processes. Classic transport passes the same core-generated terrain
mesh/image data to the renderer. Headless collision generation is independent
of the GUI. Run `gazebo examples/classic/plugin_snippet.world`, or `gzserver`
for a server-only run, after setting `GAZEBO_PLUGIN_PATH`.

Classic uses triangle-mesh collisions generated from the same 16-bit heightmap
samples; its native image-heightmap loader does not support that precision.
`heightmap_size` controls the collision grid in both adapters. Large grids cost
more mesh memory and insertion time in Classic than the modern heightmap path.

## Troubleshooting

- **Plugin not found:** check that `GZ_SIM_SYSTEM_PLUGIN_PATH` points to the build directory in the terminal that launches Gazebo or PX4. Keep both terrain libraries together.
- **No terrain:** check the world coordinates, the model configuration block, the Ogre2 Sensors system, and that a camera is active. Look for tile download errors in the server output.
- **Tiny or displaced Classic terrain:** rebuild the Classic plugin with the metre-scale fix. Older renderers inherited the `0.001` scale of the internal visual anchor, shrinking both the terrain and its distance from the world origin by 1,000. The updated renderer keeps page geometry in world metres; enlarging the world or `uneven_ground` is unnecessary.
- **Terrain missing at altitude:** check the camera's far clipping distance as well as the visual radius. Increasing either can increase rendering work.
- **Off-screen memory does not drop:** check camera names first. Cache reuse and allocator behaviour can keep process RSS above the amount of live terrain data, so inspect resource diagnostics as well as system memory.

