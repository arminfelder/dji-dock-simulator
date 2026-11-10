# dock-sim

A software DJI Dock. It speaks the device side of the DJI Cloud API over MQTT so
a DJI Cloud API platform can be exercised without a dock, a drone or a remote
controller.

## What it emulates

| Slice | What the simulator does |
|---|---|
| Connectivity and presence | Connects to EMQX and sends the `config` request. Like the real dock, it checks the `app_license` in the reply against DJI's public key and stays offline if the check fails. The key is not in this repository: set `device.license_key` to its PEM text. Without it, every license passes. Then it publishes `update_topo` with the drone as a sub-device, then streams dock and drone `osd` and `state`. Dock and drone appear online in the web UI. |
| Mission lifecycle | Handles `flighttask_prepare` / `execute` / `pause` / `recovery` / `stop` / `undo` / `progress_get`. Downloads the KMZ, parses the WPML waylines, flies the route by interpolation and reports `flighttask_progress`. |
| Media pipeline | Requests `storage_config_get`, renders one JPEG per photo action from aerial imagery (see [Simulated photos](#simulated-photos)), uploads it to the returned S3 or MinIO bucket with the returned STS credentials, then sends `file_upload_callback` with the pose of the shot. |
| Livestream | Answers `live_start_push`, `live_stop_push` and `live_lens_change`. Streams the rendered camera view of the drone or the dock (see [Live feed](#live-feed)) towards the RTMP, RTSP or WHIP URL the platform gives. |
| Dock commands | `return_home`, `property/set`, `supplement_light_*`, `alarm_state_switch`, `battery_store_mode_switch`, `battery_maintenance_switch`, `sdr_workmode_switch`, `sim_slot_switch`. The OSD shows the battery mode, the battery maintenance state, the air conditioner and the link mode. |
| Remote debug | `debug_mode_open` and `debug_mode_close` set dock mode 2. In mode 2, `cover_*`, `putter_*` (dock1 only), `charge_*`, `drone_open`/`close`, `drone_format`, `device_format` and `air_conditioner_mode_switch` change the dock state. The jobs reply `in_progress`, then send progress events. `device_reboot` makes the dock silent for `maintenance.reboot_duration`, then the dock comes back online. `esim_activate`, `esim_operator_switch` and `rtk_calibration` send progress events and change nothing. |
| Firmware | `ota_create` sets dock mode 3, sends `ota_progress` and then reports the new `firmware_version`. The simulator does not download the firmware file. |
| Device logs | `fileupload_list` lists three boots per device. `fileupload_start` uploads a ZIP of synthetic text logs to the given bucket and sends `fileupload_progress`. `fileupload_update` with `cancel` stops it. |
| Custom flight areas and offline map | `flight_areas_update` (and one sync after going online) gets the file list, downloads each file, checks its SHA256 and reads the GeoJSON circles and polygons. In flight, the dock sends `flight_areas_drone_location`. A checked takeoff or fly-to into a no-fly zone is refused. `offline_map_update` downloads and checks the map files. |
| HMS | Sends the alarms of `maintenance.hms` as `hms` events. The list is empty by default. |
| Other services | `unlock_license_*` answer an empty license list. `flighttask_create` and conditional tasks with `flighttask_ready` work. A failed KMZ download asks `flighttask_resource_get` for a new URL. PSDK speaker, widget and custom data services answer 314000, because the aircraft has no PSDK payload. |
| Manual flight | `flight_authority_grab`, `payload_authority_grab`, `takeoff_to_point`, `fly_to_point`, `fly_to_point_update`, `fly_to_point_stop`. Flies the one-key takeoff and fly-to legs and reports `takeoff_to_point_progress` and `fly_to_point_progress`. `return_home` and `return_home_cancel` fly the aircraft home or stop it in a hover, with `return_home_info`. At 30 % battery or less, the aircraft returns home by itself. `poi_mode_enter`, `poi_circle_speed_set` and `poi_mode_exit` fly an orbit around a point with `poi_status_notify`. `in_flight_wayline_deliver`, `_stop`, `_recover` and `_cancel` fly a KMZ route from the hover with `in_flight_wayline_progress`, and the aircraft stays in the air after it. |
| Camera | `camera_mode_switch`, `camera_photo_take` (with `camera_photo_take_progress` and an upload of the photo), `camera_photo_stop`, `camera_recording_start` / `_stop`, `camera_screen_split`, `photo_storage_set`, `video_storage_set`, exposure, focus and IR metering services. The aircraft osd `cameras[]` shows the state. A photo in flight uploads after the landing with the media of the flight. |
| Live flight control (DRC) | `drc_mode_enter` opens the second broker session; a repeated enter keeps it. `drone_control` sticks move the aircraft in manual mode. `drone_emergency_stop`, `drc_emergency_landing` and `drc_force_landing` stop or land the aircraft. Each command gets a `drc/up` reply with its `seq`. The dock pushes `osd_info_push`, `hsi_info_push`, `delay_info_push`, `drc_drone_state_push`, `drc_batteries_info_push`, `drc_camera_state_push` and `drc_camera_osd_info_push`, and leaves DRC after 60 s without `heart_beat`. |

It also models a battery: the pack drains in flight and recharges in the dock.

## What it does not emulate

- DRC light, speaker, PSDK and AI commands: the dock answers result 0 and
  changes nothing (except `drc_night_lights_state_set` and
  `drc_stealth_state_set`, which it stores). The DRC camera setters store their
  value; night mode, denoise and photo format change nothing.
- Video files: a recording changes the camera state only. The dock uploads no
  video. Panorama and timed shots take one photo.
- IR temperatures and the laser range finder: the temperatures are fixed, and
  the range target is where the camera axis meets the plane of the dock
- Obstacle sensing and wind: `hsi_info_push` always shows a clear area, wind is 0
- A landing away from the dock (emergency or forced landing) also ends with the
  aircraft in the dock. There is no field recovery.
- A remote controller, multi-dock return and the 20 m minimum fly-to height
- HMS alarms that come from the simulated state: the dock sends only the
  configured list
- The firmware file: `ota_create` does not download or install it, and the dock
  does not restart after the update
- A real restart: `device_reboot` stops all messages but keeps the MQTT session
- Real device logs: the uploaded logs are synthetic text
- Flight area physics: the aircraft does not stop at a zone edge, wayline
  missions are not checked, and there are no altitude limits or DJI GEO zones.
  The offline map is stored by name only.
- Remote unlocking licenses, PSDK and ESDK payloads, eSIM and 4G dongles, live
  quality levels and `live_camera_change`
- The DJI thing model in full — the OSD carries the fields the platform reads,
  not every property in the specification

## Protocol source

Every topic, method and field comes from the DJI Cloud API documentation, kept
offline in `reference/Cloud-API-Doc` (the `dji-sdk/Cloud-API-Doc` repository).
Dock 2 is the default model: `domain 3, type 2, sub_type 0`, paired with a
Matrice 3D (`domain 0, type 91, sub_type 0`).

## Build and test

STE note: the procedures in this document use ASD-STE100 Simplified Technical English.
"Build", "compile", "configure" and "preset" are technical verbs and nouns of
CMake; they are not in the STE dictionary.

dock-sim is written in C++26: a static library and a command line program.

| Part | Location |
|---|---|
| Library `docksim` (static) | `lib/` (public headers in `lib/include/docksim`) |
| Command line program `dock-sim` | `apps/dock-sim/` |
| Tests (Catch2) | `test/` |
| CMake modules and dependencies (CPM) | `cmake/` |

The release binary links all third-party libraries statically. It needs only
the glibc of the build image. The Vulkan loader for ray tracing is opened at run
time, so the binary also starts on a computer without a GPU.

The license is GPL-3.0-or-later. See `LICENSE`, `COPYING` and `THIRD_PARTY.md`.
The live video uses FFmpeg (LGPL) and openh264 compiled from source. That
openh264 has no H.264 patent license, so the binary is for internal use only.

### Compile and test

1. Make sure that GCC 15 or Clang 21, CMake 3.30 or later, Ninja and nasm are installed.
2. Configure, compile and test the debug version:

   ```bash
   cmake --workflow --preset gcc-debug
   ```

3. Make sure that the last test line shows `100% tests passed`.
4. To make the release binary, do this command:

   ```bash
   cmake --workflow --preset gcc-release-static
   ```

5. Make sure that the binary needs only glibc:

   ```bash
   readelf -d out/cmake/gcc-release-static/apps/dock-sim/dock-sim | grep NEEDED
   ```

6. Make sure that the output shows only `libc.so.6`, `libm.so.6` and `ld-linux-x86-64.so.2`.

NOTE: The first configure downloads the dependencies into `out/cpm`. This
needs access to github.com. The ray tracing tests use Mesa lavapipe if the
computer has no GPU with ray queries. The other presets are `clang-debug` and
`asan-ubsan`.

### Do the integration test

The integration test does not use the cluster or credentials. It starts an EMQX
broker in Docker and a scripted cloud. The dock connects, flies one mission,
uploads the media and opens a DRC session.

1. Make sure that Docker is installed and the `emqx/emqx:5.8.8` image is available.
2. Do the test:

   ```bash
   ctest --test-dir out/cmake/gcc-debug -L integration
   ```

3. Make sure that the output shows `100% tests passed`.

NOTE: If Docker is not available, the integration tests show as skipped. To
use a broker that runs already, set `DOCKSIM_TEST_BROKER=host:port`.

### Make the Docker image

1. Make the image:

   ```bash
   docker build -t dock-sim .
   ```

2. To make the image with Vulkan and the NVIDIA runtime settings, do this command:

   ```bash
   docker build --target runtime-gpu -t dock-sim:gpu .
   ```

NOTE: The image runs `dock-sim --config /config/config.yaml`. The command line
options are `--login`, `--dock-sn`, `--drone-sn`, `--log-level` and `--log-wire`.

## How a dock is provisioned

The DJI sample chain does not apply to this deployment. The backend answers
`POST /manage/api/v1/login` with 406, so the DJI-style login is unreachable. The
supported chain is:

| Step | Call | Gives |
|---|---|---|
| 1 | `POST /server/auth` with `{email, password, source:"web"}` | the session token for `x-auth-token` |
| 2 | `GET /server/org/current` with that header | `mqttUsername`, `mqttPassword`, `mqttAddr`, `organizationId`, `bindingCode` |
| 3 | MQTT CONNECT with those credentials | the broker session |
| 4 | `config` request, repeated every 10 s until a reply comes | `app_id`, `app_key`, `app_license` (the backend `cloud-api.app.*` values); the license must verify against `device.license_key`, or the dock disconnects |
| 5 | `airport_bind_status`, then `airport_organization_get`, then `airport_organization_bind` | the dock and the drone joined to the organization |
| 6 | `update_topo` and `osd` | the devices online in the UI |

`--login` does steps 1, 2 and 5 for you.

The MQTT credentials belong to the user and do not change between logins.

The backend serves `/manage/**`, `/org/**` and `/auth/**` under the `/server`
path, as `https://<host>/server/...`.

## Run against a backend

1. Copy the example configuration.

   ```bash
   cp config.example.yaml config.yaml
   ```

2. In `config.yaml`, set `broker.host` and `http.base_url` to your deployment.
   If the broker certificate comes from a private CA, set `broker.ca_file` to
   that CA certificate.

3. Set the identity. Two methods are possible:

   - Put `broker.username` and `broker.password` in the file.
   - Put a platform account in `http.username` and `http.password`, then start
     the simulator with `--login`.

4. Start the simulator.

   ```bash
   dock-sim -c config.yaml --login
   ```

   To use the Docker image, do this command:

   ```bash
   docker run --rm -v "$PWD/config.yaml:/config/config.yaml:ro" dock-sim --config /config/config.yaml --login
   ```

5. Open the web UI. The dock and the drone must show as online.

**Caution:** The simulator publishes to the same broker as a real dock. Give it
a serial number that no real device uses.

## Configuration

`config.example.yaml` lists every key with its default. Environment variables
override the file:

`DOCKSIM_BROKER_HOST`, `DOCKSIM_BROKER_PORT`, `DOCKSIM_BROKER_CA`,
`DOCKSIM_MQTT_USERNAME`, `DOCKSIM_MQTT_PASSWORD`, `DOCKSIM_BASE_URL`,
`DOCKSIM_HTTP_USERNAME`, `DOCKSIM_HTTP_PASSWORD`, `DOCKSIM_DOCK_SN`,
`DOCKSIM_DRONE_SN`, `DOCKSIM_API_TOKEN`, `DOCKSIM_LOG_LEVEL`.

The `maintenance` section sets the times of remote debug, firmware update and
log upload, and the flight area and HMS behavior:

| Key | Default | Meaning |
|---|---|---|
| `job_duration` | 4.0 | Seconds from the reply of a remote-debug job to its end |
| `reboot_duration` | 60.0 | Seconds that `device_reboot` keeps the dock offline |
| `ota_duration` | 30.0 | Seconds from `ota_create` to the new firmware version |
| `log_step_duration` | 1.0 | Seconds of each step (pull, compress, upload) of each log file |
| `sync_flight_areas_on_start` | true | Get the custom flight areas once after going online |
| `flight_area_report_interval` | 5.0 | Seconds between two `flight_areas_drone_location` events |
| `hms_interval` | 30.0 | Seconds between two `hms` events; 0 sends the list once |
| `hms` | `[]` | Alarms: `code`, `level` (0 to 2), `module` (0 to 3), `device` (dock or drone), `in_the_sky`, `imminent`, `component_index`, `sensor_index` |

## Instructor console

dock-sim serves a REST API, a WebSocket and an Angular web UI on
`http://127.0.0.1:8080/` (section `api` of the config). The web UI shows the
dock, the aircraft, the mission and the MQTT traffic live, and lets the
instructor change the simulation while a trainee works in the platform web UI.

The API is built with Drogon (`DOCKSIM_WITH_API`, default ON). The web UI is a
separate npm project in `web/` (Angular 21, Leaflet).

| Method | Path | Body or result |
|---|---|---|
| GET | `/api/state` | Snapshot of dock, aircraft, mission, faults |
| WS | `/api/ws` | `{"type":"state"}` twice a second, `{"type":"wire"}` with new MQTT messages |
| PUT | `/api/dock/position` | `{latitude, longitude[, altitude]}`, only with the aircraft in the dock |
| PUT | `/api/drone/position` | `{latitude, longitude[, height]}`, only in flight |
| PUT | `/api/battery` | `{percent}` |
| PUT, DELETE | `/api/weather` | `{wind_speed, wind_direction, rainfall, temperature, humidity}`, any subset; DELETE goes back to random calm weather |
| PUT | `/api/gnss/dock`, `/api/gnss/drone` | `{gps_number, rtk_number, quality, fix}` |
| PUT | `/api/clock` | `{time_scale}`, 0.1 to 100 |
| PUT | `/api/faults` | `{emergency_stop, cover_fault}` |
| POST, DELETE | `/api/hms`, `/api/hms/{code}` | Raise or clear an HMS alarm |
| GET | `/api/wire?since=<seq>` | The last 500 MQTT messages |
| GET | `/api/camera/{drone-wide,drone-zoom,drone-ir,dock}.jpg?w=640` | One rendered frame |

Every change answers with the new snapshot. Bad values give 400, a change that
the state does not allow gives 409 (for example, a dock move while the aircraft
flies). With `api.token` set, each `/api` request needs
`Authorization: Bearer <token>` or `?token=<token>`.

Effects in the simulation:

- Wind at or above `weather.max_wind_takeoff` refuses `flighttask_execute` and
  `takeoff_to_point` with 514135. Rainfall at or above
  `weather.max_rainfall_takeoff` gives 514134. The emergency stop gives 514121,
  the cover fault 514107, and a battery below 30 % gives 514103.
- A headwind slows the aircraft, and wind increases the battery drain
  (1.5 times at 12 m/s). With fewer than 6 GPS satellites, the aircraft is in
  ATTI mode and drifts with the wind.
- These limits and models come from DJI specification values. They are not
  read from the dock firmware.

To build and use the web UI (Simplified Technical English, ASD-STE100):

1. Install Node.js 22.12 or later.
2. In `web/`, run `npm install` and then `npm run build`.
3. Start dock-sim. It finds `web/dist/dock-sim/browser` when it runs from the
   build tree.
4. For UI development, run `npm start` in `web/`. The development server
   sends `/api` to port 8080.

`cmake -DDOCKSIM_BUILD_WEB=ON` runs the npm build as part of the CMake build.
The Docker image contains the web UI and listens on all interfaces. Set
`DOCKSIM_API_TOKEN` when you run it.

## Simulated photos

At each `takePhoto` or `orientedShoot` waypoint the mission records the drone's
position, height, heading, gimbal pitch and the time. After landing,
the renderer (`lib/src/render/`) draws what the wide camera sees from that pose:

- The ground texture is the basemap.at orthophoto (0.15 m to 0.3 m, Austria
  only), at up to three zoom levels so near ground is sharp and far ground is
  cheap. Set `media.imagery_url` to any XYZ layer outside Austria.
- The terrain comes from AWS Terrarium elevation tiles (global). Rays are
  ray-marched against it, so oblique shots show hills and mountains occluding
  the horizon, then the sky.
- The time of day sets the sun position (almanac formula), which drives the
  exposure, white balance, sky colour and sensor noise. Shadows are the ones
  baked into the orthophoto. `media.lighting_time` overrides the time
  (`"19:30"` for today at 19:30, or a full ISO time); see config.example.yaml.
- Buildings come from OpenStreetMap: each footprint is extruded to a flat roof
  of `height`, else `building:levels` × 3 m + 1 m, else 8 m. Roofs keep the
  orthophoto texture; walls get a plaster colour per building and a window grid
  (3 m floors). Footprints are fetched from Overpass per 0.01° cell and cached
  in `media.cache_dir/osm-buildings`; the live feed fetches in the background
  and shows buildings once they arrive.
- Terrain and buildings cast sun shadows for the time of day, and slopes and
  walls are lit by their angle to the sun.
- Each JPEG carries EXIF GPS and DJI `drone-dji:` XMP tags (gimbal and flight
  yaw/pitch, absolute and relative altitude).

Tiles are cached under `media.cache_dir`. If a render fails (no network, no
coverage) the photo falls back to the synthetic test pattern and the log shows
a warning. Set `media.renderer: synthetic` to always use the test pattern.

Render one photo without a flight:

```bash
dock-sim render --lat 47.263 --lon 11.401 --alt 80 --heading 0 --pitch -20 \
    --time 2026-10-14T16:45:00Z -o dusk.jpg
```

Measure the frame rate at 1280x720 and 1920x1080:

```bash
dock-sim bench --device auto
```

Imagery: Datenquelle: basemap.at (CC BY 4.0). Elevation: Mapzen Terrain Tiles
on AWS (see their attribution list). Buildings: © OpenStreetMap contributors (ODbL).

## Live feed

With `live.source: render` the livestream shows the same rendered view as the
photos, live:

- **Drone camera** (`video_id` starting with the drone SN): follows the drone's
  position, height, heading and gimbal pitch while it flies. In the dock it
  shows a dark frame with "AIRCRAFT IN DOCK".
- **Lens switch**: `live_lens_change` switches between `wide`/`normal` (84°),
  `zoom` (15°) and `ir` (61°, a white-hot look that is not physical).
- **Dock camera**: a fixed view across the landing site from 2 m. It renders
  again every 30 s, so the light follows the time of day.

A render thread draws frames as fast as the CPU allows. The push sends the
latest frame at `live.fps` and repeats it while the next one renders. At
1280x720 with 8 workers the render runs at about 8–10 fps, and at 960x540
at about 12–16 fps. The overlay shows the `video_id`, the scene time and the
pose. The stream is H.264 (openh264) through FFmpeg for RTMP and RTSP, and
through libdatachannel for WHIP. `live.encoder: nvenc` moves the encode to
an NVIDIA GPU. `live.source: pattern` sends colour bars.

### Renderers

`media.render_device` selects the renderer:

| Value | Renderer |
|---|---|
| `cpu` | Heightfield ray march on all CPU cores (`lib/src/render/cpu/`). Terrain and buildings come from a raster grid. |
| `rt` | Hardware ray tracing through Vulkan ray queries (`lib/src/render/rt/`): NVIDIA RTX, AMD RDNA2 or newer, Intel Arc. |
| `auto` | `rt` on a GPU with ray queries, else `cpu`. |

The old value `gpu` (the OpenCL renderer of the Python version) now selects the
CPU. `media.gpu_device` selects a Vulkan device by part of its name, for example
`Arc` or `Radeon`.

The ray tracer uses triangle meshes for terrain tiles and OSM buildings, so
walls, building edges and roofs (gabled, hipped, pyramidal, skillion, dome) are
exact. Window recesses are traced in the shader: the view ray goes on into the
opening, and the sun shadow of the reveal follows the time of day. The shader is
`lib/resources/shaders/render.slang`; the build compiles it to SPIR-V with the
Slang compiler.

1. Make sure that the Vulkan driver of the GPU is installed.
2. Run `vulkaninfo --summary` and make sure that the GPU is in the list.
3. In `config.yaml`, set `media.render_device` to `rt` or `auto`.

Near the camera (`media.building_scan_radius`, 1500 m) the ray tracer measures
buildings whose height and roof OSM only guesses (no height tag, no roof:shape,
no building parts) with the BEV laser scan of Austria (surface model minus
terrain model, 1 m, CC BY 4.0, "Datenquelle: BEV"): the best fitting standard roof
(flat, gabled, hipped, pyramidal) with measured eaves and ridge. Tagged buildings,
and roofs that no standard shape fits, keep their OSM geometry.
In the same radius, the scan also gives trees: where it is more than 3 m above
the terrain outside every OSM building, the ray tracer adds a canopy mesh on a 2 m
grid with the orthophoto colour. Smooth patches (roofs that OSM does not have) and
strips narrower than 6 m are left out.
The blocks are read by HTTP range requests and cached in `cache_dir/als`. Outside
Austria, and with `media.building_scan: false`, the OSM roofs stay and there are no trees.

NOTE: The tests run the ray tracer on Mesa lavapipe, a Vulkan driver for the CPU
(package `mesa-vulkan-drivers`). The tests set `VK_ICD_FILENAMES` to lavapipe
when the file is present. Set `DOCKSIM_TEST_GPU=1` to test on the real GPU.

## Known platform constraints

- The backend answers **423 LOCKED** to almost every HTTP request until the dock
  licence is activated. MQTT is not affected, but the UI is.
- Unauthenticated HTTP requests get **406**, never 401. A 406 from `/server/...`
  means no token, not a wrong path.
- Point clouds and terrain have no public API. They enter through the frontend.

## Layout

```
apps/dock-sim/     command line: run one dock, render, bench
lib/include/docksim/  public API: config, run_dock, provision, render_photo
lib/src/
  config/       YAML loader, DOCKSIM_* environment, validation
  protocol/     wire format: envelope, topics, codes, osd/state payloads
  transport/    MQTT session (Paho), request/reply correlation
  device/       dock and aircraft state, physics, geodesy, gimbal maths, models
  services/     service registry and the service groups
  wayline/      KMZ/WPML reader and the flight engine
  flight/       manual flight: one-key takeoff, fly-to, sticks, return to home
  media/        photo files with EXIF/XMP, S3 upload, upload pipeline
  live/         rendered feed, FFmpeg push, WHIP
  cloud/        app license check, backend login
  sim/          clocks, coroutine tasks, per-dock context, camera views, DockSim
  raster/       Web Mercator, local frames, BEV laser scan reader
  osm/          Overpass client, building cell cache, footprints, roofs, facades, grid
  render/       renderer: tiles, terrain, imagery, shading, materials
    cpu/        heightfield march and shade
    rt/         Vulkan ray tracer: meshes, scene, frame driver
lib/resources/  embedded files: shader, wall materials, font, OSM extras
test/
  golden/       wire snapshots: the contract with the backend
  render/       pixel snapshots of the CPU and ray tracing renderers, offline scene
  unit/         unit tests
  integration/  real EMQX in Docker plus a scripted cloud
cmake/          dependencies (CPM), FFmpeg build, shader build, warnings
tools/
  fetch_reference.sh  download the DJI Cloud API documentation
docs/
  protocol-quirks.md  every wire rule the code depends on, with evidence
```
