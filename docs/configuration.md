# Terrain configuration and providers

These options apply to all three backends. Modern Gazebo reads them from the
model configuration plugin; Classic reads them from the world plugin. See
[simulation setup](setup.md) for the corresponding SDF placement.

## Configuration

In modern Gazebo, these settings go inside the model's `custom::DynamicTerrainConfig` block. Classic uses the same settings inside its world plugin. Values below are defaults; the examples adjust a few of them for aircraft use.

| Parameter | Default | Purpose |
| --- | --- | --- |
| `visual_radius_m` | `7500` | Radius of the visual terrain area, in metres. |
| `visual_geometry_zoom` | `14` | Tile zoom used to divide the terrain mesh into pages. |
| `visual_elevation_zoom` | `13` | Elevation source zoom for the visual mesh. |
| `visual_mesh_cells_per_tile` | `64` | Mesh subdivisions per tile edge. |
| `visual_page_texture_max_size` | `2048` | Maximum texture edge length, in pixels. |
| `visual_page_cache_mb` | `128` | Refined-image cache budget in RAM, in MiB. |
| `decoded_dem_cache_mb` | `256` | Decoded elevation cache budget in RAM, in MiB. |
| `visual_frustum_eviction` | `true` | Release rendering resources for off-screen pages. |
| `visual_offscreen_frames` | `30` | Off-screen grace period, in render frames. |
| `download_concurrency` | `4` | Maximum concurrent tile downloads. |
| `download_per_host` | `1` | Maximum concurrent downloads to one host. |
| `enable_collision` | `true` | Generate the moving collision heightmap. |
| `align_origin_to_ground` | `true` | Align terrain at the origin to local Z = 0. |
| `cache_dir` | `~/.cache/gz_dynamic_terrain` | Location of cached downloads and generated terrain files. |

For lower VRAM use, start by reducing `visual_page_texture_max_size` to `1024` or reducing the visual radius. Lowering texture resolution also limits the imagery detail available on each page. A shorter off-screen grace period frees resources sooner but may cause more uploads when the camera turns.

The two RAM cache budgets are not a limit on total Gazebo memory. Active terrain, pending updates, cameras, physics, and the renderer need additional memory. The disk cache is separate and has no automatic size limit.

Collision resolution changes with the model's local Z coordinate when `dynamic_zoom` is enabled; it is not based on height above the terrain. For fixed resolution, set `dynamic_zoom` to `false` and choose `static_zoom`.

## Imagery and elevation providers

Imagery supplies the ground texture; elevation supplies its shape. They are configured independently, so you can combine, for example, Mapbox satellite imagery with Terrarium elevation.

Put the settings below inside the aircraft's `custom::DynamicTerrainConfig` plugin block, directly under `<model>`. Replace existing provider settings rather than adding duplicate tags. These are model settings, not entries for the world-level `server.config` plugin.

### Built-in imagery

| `imagery_provider` | Ground texture |
| --- | --- |
| `google_satellite` | Satellite imagery; the default. |
| `google_street` | Road map. |
| `google_terrain` | Terrain-style map, not elevation data. |
| `google_hybrid` | Satellite imagery with roads and labels. |
| `google_labels` | Labels only; not a complete background map. |
| `bing_road` | Road map. |
| `bing_satellite` | Satellite imagery. |
| `bing_hybrid` | Satellite imagery with roads and labels. |

For example, to use Bing satellite imagery with the default elevation source:

```xml
<imagery_provider>bing_satellite</imagery_provider>
<elevation_provider>terrarium</elevation_provider>
<elevation_max_zoom>15</elevation_max_zoom>
```

These names select URL templates included in the plugin; they do not guarantee service availability or permission to download tiles. The plugin does not composite separate imagery layers, so use `google_hybrid` instead of `google_labels` if you want labels over satellite imagery.

An unknown imagery name without an `imagery_url` falls back to Google satellite. In particular, setting `imagery_provider` to `mapbox` or `custom` alone does not configure another service.

### Elevation sources

| `elevation_provider` | Behaviour |
| --- | --- |
| `terrarium` | Decode Terrarium RGB tiles. Defaults to the public `elevation-tiles-prod` S3 endpoint, with no token in the URL. |
| `mapbox` | Decode Mapbox Terrain-RGB heights. Use the explicit URL in the Mapbox example below. |
| `flat` or `none` | Skip elevation downloads and use a flat surface at the world's elevation reference, before `z_offset_m`. Imagery still downloads. |

`elevation_max_zoom` limits source elevation requests and defaults to `15`. `visual_elevation_zoom` selects the source detail for the visual mesh and defaults to `13`. Raising imagery zoom does not add elevation detail.

The elevation provider also selects the pixel decoder. Use `terrarium` for Terrarium-encoded data and `mapbox` for Terrain-RGB, even when hosting those tiles yourself. A generic `custom` elevation name is not format detection: all non-flat providers other than `terrarium` currently use the Terrain-RGB decoder.

### Mapbox satellite imagery and elevation

Create an access token in your Mapbox account with access to the requested tilesets. Replace both `YOUR_MAPBOX_ACCESS_TOKEN` values below; the imagery and elevation token settings are separate, even when they contain the same token.

This is a complete model-side configuration block:

```xml
<plugin filename="libgz-dynamic-terrain-system.so"
        name="custom::DynamicTerrainConfig">
  <imagery_provider>mapbox_satellite</imagery_provider>
  <imagery_url>https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}.jpg90?access_token={token}</imagery_url>
  <imagery_extension>jpg</imagery_extension>
  <imagery_token>YOUR_MAPBOX_ACCESS_TOKEN</imagery_token>

  <elevation_provider>mapbox</elevation_provider>
  <elevation_url>https://api.mapbox.com/v4/mapbox.terrain-rgb/{z}/{x}/{y}.pngraw?access_token={token}</elevation_url>
  <elevation_token>YOUR_MAPBOX_ACCESS_TOKEN</elevation_token>
  <elevation_max_zoom>15</elevation_max_zoom>
  <visual_elevation_zoom>13</visual_elevation_zoom>

  <cache_dir>~/.cache/gz_dynamic_terrain_mapbox_rgb</cache_dir>
  <diagnostics>true</diagnostics>
</plugin>
```

Here, `mapbox_satellite` is a custom cache name, not a built-in preset; `imagery_url` selects the actual service. The imagery URL uses Mapbox's [Raster Tiles API](https://docs.mapbox.com/api/maps/raster-tiles/). The elevation URL uses its documented [Terrain-RGB endpoint](https://docs.mapbox.com/data/tilesets/guides/access-elevation-data/); keep `.pngraw` so elevation values are preserved.

Do not omit `elevation_url` in this example. The plugin's current Mapbox fallback points at Terrain-DEM, which Mapbox documents as [available only through its SDKs, not the Raster Tiles API](https://docs.mapbox.com/data/tilesets/reference/mapbox-terrain-dem-v1/). Gazebo is not a Mapbox SDK client.

The current cache stores non-Terrarium elevation tiles with a `.webp` filename even when the response contains PNG data. OpenCV reads the image content; the cache suffix does not change the elevation encoding.

For Mapbox imagery with Terrarium elevation instead, keep the imagery settings, replace the elevation settings with `<elevation_provider>terrarium</elevation_provider>`, and remove `elevation_url` and `elevation_token`.

### Custom and local tile servers

Use an HTTP(S) URL that returns raster tiles on the Web Mercator XYZ grid. A URL override takes precedence over a built-in provider's URL. For example:

```xml
<imagery_provider>local_orthophoto</imagery_provider>
<imagery_url>http://127.0.0.1:8080/imagery/{z}/{x}/{y}.jpg</imagery_url>
<imagery_extension>jpg</imagery_extension>

<elevation_provider>terrarium</elevation_provider>
<elevation_url>http://127.0.0.1:8080/elevation/{z}/{x}/{y}.png</elevation_url>
<elevation_max_zoom>14</elevation_max_zoom>
<cache_dir>~/.cache/gz_dynamic_terrain_local</cache_dir>
```

The server must actually provide those paths. For locally hosted Terrain-RGB elevation, change `elevation_provider` to `mapbox` and keep an explicit URL. Elevation tiles must preserve the encoded RGB values; do not JPEG-compress them or substitute a coloured hillshade.

Use ordinary 256 × 256 imagery tiles as a starting point. Vector tiles (`.pbf`), TileJSON, `mapbox://` identifiers, and raw GeoTIFF or MBTiles datasets are not direct inputs. Serve or convert them to compatible raster tiles first. There is no automatic TMS Y-axis inversion or alpha-overlay compositing.

### URL placeholders and tokens

Both `imagery_url` and `elevation_url` support:

| Placeholder | Replacement |
| --- | --- |
| `{x}`, `{y}`, `{z}` | Tile column, row, and zoom. |
| `{q}` | Bing-style quadkey, for servers that use quadkeys instead of XYZ paths. |
| `{s}` | Numeric server shard from `0` to `3`. |
| `{s4}` | Numeric server shard from `1` to `4`. |
| `{token}` | `imagery_token` or `elevation_token`, depending on the URL. |

Only these placeholders are expanded. Tokens are substituted literally, not URL-encoded; environment variables such as `$MAPBOX_TOKEN` in SDF are not expanded by the plugin. Supplying a token does not add authentication unless the URL contains `{token}`. Custom HTTP authorization headers are not configurable.

In XML, write `&amp;` between query parameters, for example:

```xml
<imagery_url>https://your-tile-server.example/{z}/{x}/{y}.png?key={token}&amp;style=satellite</imagery_url>
<imagery_extension>png</imagery_extension>
<imagery_token>YOUR_PROVIDER_TOKEN</imagery_token>
```

### Cache and provider errors

Restart the simulation after changing provider settings. Cache entries are keyed by provider name and tile coordinates, not by the complete URL or token. When changing an endpoint or elevation dataset under the same provider name, choose a new `cache_dir` to avoid reusing old tiles.

For failed downloads, enable `diagnostics` and check the provider response. Mapbox documents `401` for missing or invalid tokens, `403` for access issues (including some URL-restricted tokens), `404` for missing tiles or tilesets, and `429` for rate limiting. See its [API error reference](https://docs.mapbox.com/api/maps/raster-tiles/). If requests are throttled, reduce `download_concurrency` and `download_per_host`.

Mapbox Terrain-RGB can return a non-image response for [tiles entirely over ocean](https://docs.mapbox.com/data/tilesets/guides/access-elevation-data/). This plugin does not translate that response into sea-level terrain, so those tiles can fail to load.

Keep tokens out of published SDF files and shared logs. Check your provider's access, caching, attribution, and usage requirements before downloading an area. The project's BSD license covers the plugin, not third-party map data.
