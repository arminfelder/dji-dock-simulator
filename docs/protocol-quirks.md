# Protocol quirks

Wire behaviour that dock-sim depends on and that the DJI documentation does not
tell you. Each entry was learned from a real Dock 2, the cloud backend, or a test
run. A refactor that "cleans up" one of these breaks the dock in the web UI,
usually without an error on the dock side.

Format: **rule** · source · guarding test (`test/golden/`).

"Unverified on a real dock" marks an entry that has only DJI doc or backend evidence.

## Session start

- **The dock asks for its app license before it goes online.** `requests/config`
  `{config_type: "json", config_scope: "product"}`, repeated every 10 s until a
  reply comes. The reply needs `app_id`, `app_key`, `app_license` flat in `data`.
  The license is RSA-SHA256 over `app_id + app_key` with DJI's 1024-bit key.
  Missing or bad → disconnect, no `update_topo`. · real Dock 2 · `connect`
- **`storage_config_get` is sent about 3 s after `update_topo`.** The backend sets
  `connect_storage = storage_update > login_time` (strictly greater). Same
  millisecond reads as false, and the launch dialog warns. · backend ·
  `connect`
- **The real dock subscribes `state_reply` and `network/probe`.** Probe answer
  format is unknown; the simulator only logs it. · EMQX log of a real Dock 3 ·
  `connect` (subscriptions)

## State and OSD

- **Every `state` message carries top-level `gateway`.** The backend looks up the
  device SDK by it; null → NPE → message dropped. · backend · `connect`
- **One property group per `state` message.** The backend picks the first enum
  group that shares any key, and `FIRMWARE_VERSION` comes first, so a bundled
  message loses everything else. · backend · `connect`
- **The firmware group carries `compatible_status` as a Boolean.** Unboxed with no
  null check; missing → NPE, firmware never stored. `true` = compatible.
  `firmware_upgrade_status` is a Boolean too. · backend · `connect`
- **Dock OSD is two messages.** `osd_msg_half` has `mode_code` and the dock
  state; `osd_msg_next` has `job_number`, power and maintenance. The backend
  ignores a dock OSD with `job_number` for mode tracking, so one combined message
  means the dock OSD is never stored and media parsing later throws
  `NoSuchElementException`. · real Dock 2, backend · `osd_tick`
- **OSD is QoS 0, everything else QoS 1.** The Dock 2 topic config agrees:
  `thing/product/uav/osd` qos 0, the other topics qos 1. · EMQX log of a real
  Dock 3, real Dock 2 · `osd_tick`, `connect`
- **The dock OSD of Dock 2 has a third message.** The dock sends
  `osd_msg_half` (`mode_code`, cover, `air_conditioner`, `battery_store_mode`,
  `sub_device`) and `osd_msg_next` (`job_number`, maintenance, backup battery,
  `drone_battery_maintenance_info`). It sends `drc_state`, `wireless_link`,
  `media_file_detail`, `flighttask_step_code` and `flighttask_prepare_capacity`
  in a message of its own. The simulator sends the same three messages (half,
  mission, next). The backend needs the third one: the backend drops a dock OSD with
  `job_number`, stores one with `mode_code` but puts the stored `drc_state` back
  into it, and takes `drc_state` only from a message with neither key. · real
  Dock 2, backend · `osd_tick`
- **`thermal_supported_palette_styles` is in the aircraft OSD `cameras[]`, not
  in `drc_camera_osd_info_push`.** · real Dock 2 · `osd_tick`
- **`live_capacity.device_list` must not be empty**, and each camera needs a
  `camera_index` of `type-subtype-position` with position in {0, 1, 2, 7}. Dock
  camera is `165-0-7`. Video types only `zoom|wide|thermal|normal|ir`. Empty list
  → UI "No available cameras". · backend · `connect`

## Services

- **`live_start_push` output is a JSON string, not an object.** The backend types
  it `ServicesReplyData<String>`; an object throws and the UI shows "Error
  occurred". The simulator echoes the push URL. · backend · `live_start_push`
- **A repeated `flighttask_prepare` for the same `flight_id` answers 0.** The
  backend sometimes sends prepare twice ~250 ms apart; an error fails the task. ·
  backend log · `mission`
- **`flighttask_progress` goes on `events` with `need_reply: 1`.** · `mission`
- **A method that the dock does not route gets no reply.** The dock forwards
  only the `services` methods of its topic config and drops every other method
  without an error reply. The simulator has the Dock 1 and Dock 2 lists in
  `protocol/routing.cpp` and drops the other methods, also on `drc/down`. Dock 3
  passes every method (not known). On Dock 2 this drops `flighttask_create`,
  `poi_*`, `putter_*`, `esim_activate`, `esim_operator_switch`,
  `rtk_calibration` and `flight_areas_delete`. The eSIM methods have a
  `services_reply` topic but no `services` topic: a gap in the DJI config, or
  the cloud cannot use them. · real Dock 1 and Dock 2 · `services`
  (`no_such_method`, POI), `poi_wayline_test`
- **Every routed service gets exactly one `services_reply`**, also when the
  handler fails (514171 for bad parameters, 314000 otherwise). A routed method
  with no handler in the simulator answers 314001. Without a reply the backend
  waits for its timeout. · `test_regressions::test_failing_handler_still_replies`
- **`putter_open`/`putter_close` exist only on Dock 1.** · real Dock 2 topic
  config · `services`
- **`flighttask_create` exists only on Dock 1.** The Dock 2 topic config does not
  route it. · real Dock 1 and Dock 2 · `maintenance_test`
- **POI exists on Dock 1, not on Dock 2.** The Dock 2 config has no `poi_*`
  topic and no `poi_status_notify`. Dock 1 routes the three services and
  `events_reply/poi_status_notify`. The simulator keeps POI on Dock 1 and on
  Dock 3; on Dock 3 it is unverified on a real dock. · real Dock 1 and Dock 2 ·
  `poi_wayline_test`
- **The in-flight wayline methods and the `drc_*` camera commands exist on
  Dock 2, not on Dock 1.** Dock 1 routes only `drone_control`,
  `drone_emergency_stop` and `heart_beat` on `drc/down`. · real Dock 1 ·
  `manual_flight_test`

## Media

- **`file_upload_callback` needs top-level `result: 0` and `progress: 100`.**
  The backend unboxes `result` with no null check. · backend · `mission`
- **`payload_model_key` is `1-{type}-{subtype}`** (payload domain 1), not the
  camera index. The backend matches it in `DeviceEnum.find` and throws if absent.
  · backend · `mission`
- **The metadata key is `created_time`**, not DJI's documented `create_time`. The
  real dock sends `created_time` as well. · real Dock 2 · `mission`
- **Uploaded JPEGs need EXIF FileSize/ImageWidth/ImageHeight**, or the backend
  rejects them as "not a drone media". · backend

## DRC

- **`drc_mode_enter` carries the broker nested in `data.mqtt_broker`**
  (`address, username, password, client_id, enable_tls`). The backend sends a `wss://`
  address with `enable_tls: false`; the scheme decides TLS and transport. ·
  real Dock 1 and Dock 2 · `drc`
- **Subscribe the exact `thing/product/{sn}/drc/down`.** The DRC JWT denies a
  wildcard and EMQX then drops the session. The real dock subscribes the same
  topic. The per-method topics `thing/product/gnd/drc/down/<method>` of the
  topic config are names inside the dock: the dock adds `/<method>` from the
  payload. · EMQX log, real Dock 2 · `drc`
- **`drc_status_notify` is flat `{result, drc_state}` on `events`.**
  `drc_state` 2 = connected, 0 = not; `result` 514303 on connection loss. ·
  real Dock 1 and Dock 2 · `drc`
- **`heart_beat` on `drc/down` goes back on `drc/up` with no change, QoS 0.**
  The dock removes and adds only its internal `transfer_info`. There is no
  `ack_timestamp`. · real Dock 1 and Dock 2 · `drc`, `manual_flight_test`

- **`drc_mode_enter` to the session that is up answers 0 and keeps the session.**
  The backend sends `drc_mode_enter` on every dock OSD with `sub_device` while
  the stored `drc_state` is 0. Each call carries a new JWT as password (seen on
  a test deployment, about once a second while `drc_state` sat next to
  `mode_code`). A new password alone keeps the session; a reconnect each time
  drops the browser's live control. A new broker, port or client id opens a new
  session. · backend, test deployment · `manual_flight_test` (idempotent)
- **The dock OSD carries `drc_state` and `sub_device`.** The backend reads
  `drc_state` from the stored dock OSD; without it the backend sees "not in DRC"
  and sends `flight_authority_grab` plus `drc_mode_enter` before every command.
  `sub_device` (`device_sn`, `device_model_key`, `device_online_status`,
  `device_paired`) starts the automatic DRC enter and is in `osd_msg_half`.
  `drc_state` must be in the mission message: next to `mode_code`, the backend
  never stores it. · backend, `OsdDock`, `DockSubDevice` · `osd_tick`
- **A drc/down command with a top-level `seq` gets a drc/up reply with the same
  `seq`, the same `method` and `data.result`.** The backend sends a UUID `seq`
  next to `data` and waits 3 s for it; no reply → HTTP 417 "No message reply
  received". This covers `drone_emergency_stop`, `drc_emergency_landing`,
  `drc_force_landing` and every other `RemoteControlMethodEnum` method. The
  real dock copies the request `seq` as it is, if there is one. The reply has no
  `timestamp`, and goes out with QoS 1. · backend, real Dock 2 · `drc`
- **`drone_control` comes from the browser, not the backend, every 50 ms**, with
  `seq` inside `data`. `x` is forward, `y` right, `h` up (m/s), `w` yaw rate
  clockwise (°/s); limits ±17, +5/−4, ±90. The UI sends zero sticks all the time
  while its panel is open, so only non-zero sticks in a wrong mode give
  `joystick_invalid_notify` (flat `{reason}`, int). The reply is
  `data {result, output {seq}}` (`output.seq` int). · web UI,
  `DroneControlRequest`, real Dock 2, DJI doc 110.drc.md "DRC-flight control" ·
  `drc`, `manual_flight_test`
- **drc/up pushes are `{method, data, seq, timestamp}`, QoS 1.** `seq` is a
  number from one counter for every push; `timestamp` is a number in ms.
  `osd_info_push` at `osd_frequency`, the others (`hsi_info_push`,
  `delay_info_push`, `drc_drone_state_push`, `drc_batteries_info_push`) at
  `hsi_frequency`. · real Dock 2; the frequencies are DJI doc 110.drc.md ·
  `drc`, `manual_flight_test`
- **`hsi_info_push` uses `around_distances`, plural.** The DJI table agrees.
  The DJI example and the backend `HsiInfoPush` say `around_distance`, so the
  backend loses the 360 distances of a real dock (a supplier defect). The
  `*_enable` and `*_work` keys are bool, `up_distance` and `down_distance`
  int. · real Dock 2 · `drc`
- **`osd_info_push` has no `attitude_pitch` or `attitude_roll`.** It has
  `ultrasonic_height` (int, -1 if not known). `wind_speed` and
  `wind_direction` are int, the other values double. · real Dock 2 · `drc`
- **`delay_info_push` has `sdr_cmd_delay` and `liveview_delay_list`.** The
  real dock also reads a `stick_delay`; the simulator does not send it. · real
  Dock 2 · `drc`
- **`drc_drone_state_push` has more keys than the DJI table:**
  `current_commander_flight_mode`, `is_near_height_limit`,
  `is_near_distance_limit`, `assistant_video {switch, mode}`, `landing_type`,
  `landing_protection_type`. Every value is an int, also `stealth_state` and
  `night_lights_state`. The backend types the last two Boolean; Jackson
  accepts 0 and 1. · real Dock 2 · `drc`
- **`drc_batteries_info_push` adds `bat_remaining_capacity`,
  `remain_job_time`, `need_gohome_time` and the two warning thresholds** to the
  battery of the aircraft OSD. · real Dock 2 · `drc`
- **The dock leaves DRC after 60 s without `heart_beat`** and sends
  `drc_status_notify` with `drc_state` 0. · DJI doc 110.drc.md "DRC-heart beat"
  · `manual_flight_test`

## Manual flight

- **`takeoff_to_point_progress` and `fly_to_point_progress` are flat in `data`**
  (`status, result, flight_id | fly_to_id, way_point_index, remaining_distance,
  remaining_time, planned_path_points`). Takeoff also has `track_id`. Takeoff
  order: `task_ready`, `wayline_progress` at 1 Hz, `wayline_ok` or
  `wayline_cancel`, `task_finish`. The backend closes the takeoff job on
  `task_finish`. · real Dock 2, DJI doc 110.drc.md, backend · `manual_flight`
- **`remaining_time` is a double**, as the DJI table says. The backend
  `TakeoffToPointProgress` types it `Integer`; Jackson cuts the fraction. · real
  Dock 2, `TakeoffToPointProgress` · `manual_flight`
- **Heights in `target_height`, `points[].height` and `planned_path_points` are
  ellipsoid heights.** The backend adds the dock OSD `height` to a relative
  height before it sends. · backend · `manual_flight`
- **`takeoff_to_point` needs dock mode idle; `fly_to_point` needs drone mode 3
  or 16.** The backend checks both before it sends. The simulator refuses with
  514116 and 319022 in other states. · backend · `manual_flight_test`
  (refusals)
- **The backend skips `flight_authority_grab` while the aircraft state shows
  `control_source` "A".** After a grab the simulator sends the control-source
  group again on `thing/product/{drone}/state` with the dock as `gateway`. ·
  the backend · `services`
- **A manual RTH sends `return_home_info`** with the current point and home at the
  RTH height in `planned_path_points`, `last_point_type`, `home_dock_sn` and
  `multi_dock_home_info [{sn, plan_status, home_distance,
  estimated_battery_consumption}]`. It has no `flight_id`, which the backend
  `ReturnHomeInfo` reads. · real Dock 2 · `manual_flight`

- **`poi_status_notify` is flat in `data`** (`status, reason, circle_radius,
  circle_speed, max_circle_speed`). `status` is a FlighttaskStatusEnum string,
  `reason` an int. The simulator sends `in_progress` every 5 s and after a
  speed change, `ok` after `poi_mode_exit`, and `failed` with reason 6 when
  RTH, fly-to, emergency stop or low battery ends the orbit (a guess). On the
  ground it refuses with 314000 and sends `failed` with reason 5. The
  backend has no caller for POI, and its notify handler only logs. ·
  `PoiStatusNotify`, `PoiStatusReasonEnum`, `EventsMethodEnum`, unverified on a
  real dock (Dock 2 has no POI) · `poi_wayline_test`, `poi_wayline` (Dock 3)
- **The POI `height` is an ellipsoid height**, as in the other manual-flight
  commands. Not confirmed: no backend code sends `poi_mode_enter`. ·
  `PoiModeEnterRequest` · `poi_wayline`
- **`in_flight_wayline_progress` is flat in `data`, and `status` is an int**
  (1 uploading, 2 uploaded, 3 progress, 4 paused, 5 cancel, 6 ok, 7 failed,
  8 timeout). `result` must be present: the backend unboxes it with no null
  check. The backend refuses a new deliver until a progress with an end status
  (5 to 8) clears its stored route, so every route ends with one. The real dock
  keys are `in_flight_wayline_id, progress {percent}, result, status,
  way_point_index`, all int but the id. · real Dock 2,
  `InFlightWaylineStatusEnum` (`@JsonValue` Integer), `InFlightWaylineProgress`,
  the backend · `poi_wayline`
- **The in-flight wayline methods are `in_flight_wayline_deliver`,
  `in_flight_wayline_stop` (pause), `in_flight_wayline_recover` (resume) and
  `in_flight_wayline_cancel` (no data).** Deliver carries
  `in_flight_wayline_id`, `file {url, fingerprint}` and `rth_altitude`; stop and
  recover carry only `in_flight_wayline_id`. The DJI Dock 2 documents do not
  have these methods. · `WaylineMethodEnum`, `InFlightWaylineExecuteRequest`,
  `InFlightWaylinePauseResumeRequest` · `services`

## Camera

- **The backend checks `cameras[]` of the aircraft osd before it sends a camera
  command.** `camera_photo_take` needs `photo_state` 0 and `remain_photo_num`
  above 0. `camera_mode_switch` needs a new mode and both states idle.
  `camera_recording_start` needs `camera_mode` 1, `recording_state` 0 and
  `remain_record_duration` above 0. `camera_recording_stop` needs
  `recording_state` 1. An osd camera that does not change makes the commands
  fail in the web UI before they reach the dock. · backend · `osd_tick`,
  `camera_test`
- **`camera_photo_take_progress` has `result` next to `output {ext
  {camera_mode}, progress {current_step, percent}, status}`.** The backend
  reads it as `EventsDataRequest`, unlike most other manual-flight events. ·
  real Dock 2, DJI doc 110.drc.md, `EventsMethodEnum` · `services`
- **`drc_camera_osd_info_push` needs `wide_lense` and `zoom_lense`, with
  `wide_dewarping_state`.** During a mission the backend reads the stored push
  to set exposure, focus and dewarping, and unboxes `wide_dewarping_state` with
  no null check. It compares `wide_iso`, `zoom_shutter_speed` and the other
  values with what the mission asks for, so the DRC setters must store them.
  The lens values are int, `zoom_factor` double. The real dock also sends
  `*_fov_info`; the simulator does not. `ir_lense` has no
  `thermal_supported_palette_styles`, and `ir_metering_area` has no
  `temperature`. · backend, real Dock 2 · `drc`
- **`video_resolution` and `linkage_zoom_state` in `drc_camera_state_push` are
  int.** `video_resolution` is 0 or 1, -1 if not known. The backend
  `VideoResolutionEnum` reads a string; Jackson changes 0 and 1 to "0" and
  "1", and -1 throws. `payload_index` is a string. · real Dock 2 · `drc`,
  `camera_test`
- **Exposure, focus and IR values are the int codes of the cloud enums**
  (`ExposureModeEnum` 1–4, `ExposureValueEnum` 16 = 0 EV, `CameraIsoEnum` 0 =
  auto, `CameraShutterSpeedEnum` 60 = auto, `FocusModeEnum` 0–2,
  `MeteringModeEnum` 0–2). The aircraft osd shows them as `wide_*`, `zoom_*` and
  `ir_metering_*`. · `OsdCamera`, `WideLenseData`, `ZoomLenseData` · `osd_tick`

## Maintenance

- **A remote-debug job replies first, then sends events with its own method.**
  The reply is `{result: 0, output: {status: "in_progress"}}`; the dock uses
  `{result, output: {status}}` for these replies, and `{result}` only for
  the other replies. The events carry `{result, output: {status, progress:
  {percent, current_step, step_key, step_result, total_steps}}}`, with
  `percent` = `current_step` × 100 / `total_steps`, and end with `ok` and 100.
  The effect occurs at the end, after `maintenance.job_duration`. The status
  string of the first reply is not confirmed. · real Dock 2, DJI doc 70.cmd.md
  · `services`, `maintenance_test`
- **The events of a job use the bid of its services message.** The backend
  keeps progress per bid. The same rule applies to `ota_progress` and
  `fileupload_progress`. · backend · `maintenance_test`
- **The step keys of each job.** Every job starts with `get_bid`,
  `check_work_mode`, `check_task_state` (`cover_force_close` has no
  `check_task_state` first). Then: `cover_open` `check_scram_state,
  open_cover`; `cover_close` `check_scram_state, turn_on_drone,
  drone_paddle_forward, drone_paddle_reverse, drone_paddle_stop, close_cover`;
  `charge_open` `check_scram_state, start_charge`; `charge_close`
  `stop_charge`; `drone_open` `check_scram_state, open_drone`; `drone_close`
  `check_scram_state, close_drone`; `device_reboot` `upgrading_prevent_reboot`
  after `get_bid`, then `write_reboot_param_file`. The order inside
  `cover_close` and `cover_force_close` is a guess. Without a step key, the
  dock sends `percent` only; the simulator does that for `drone_format` and
  `device_format` (unverified on a real dock). Putter steps of Dock 1 are the
  backend enum values. · real Dock 2 · `services`, `maintenance_test`
- **`step_key` must be a value of `RemoteDebugStepKeyEnum`.** The enum throws
  `CloudSDKException` for an unknown value, and the backend drops the event.
  The enum has no `close_drone`, so the backend drops every `drone_close`
  progress of a real dock (a supplier defect). · `RemoteDebugStepKeyEnum`, real
  Dock 2 · `services`
- **Which services need remote debug mode is a choice of the simulator.** The
  DJI doc gives no rule per method, and the real dock check is not confirmed
  (every job has a `check_work_mode` step). The simulator follows the cmd/job
  table of 70.remote-debug.md: cover, putter, charge, aircraft power, both
  formats, `device_reboot` and `air_conditioner_mode_switch` answer 514153
  outside mode 2. The light, alarm, battery modes, `sdr_workmode_switch`, the
  SIM and eSIM services and `rtk_calibration` do not need it. · DJI doc
  70.remote-debug.md, 71.error-code.md · `services`, `maintenance_test`
- **Remote debug mode blocks flights.** `debug_mode_open` answers 514148 while
  the dock works or the aircraft is out, 514147 during an update and 514150
  during a restart. In mode 2, `takeoff_to_point` and `flighttask_execute`
  answer 514146. A second job while one runs answers 514116. · DJI doc
  71.error-code.md · `maintenance_test`
- **`device_reboot` makes the dock silent; it does not close the MQTT session.**
  After the `ok` event the dock closes DRC (`drc_status_notify`) and live video
  and sends `update_topo` with no sub-devices. For `maintenance.reboot_duration`
  it sends no osd, state or topology and gives no `services_reply`, so the cloud
  times out. Then it sends `update_topo` online and every state group, and the
  mode is idle. A real dock loses the connection; that is not simulated. ·
  choice, DJI doc 70.cmd.md · `services`, `maintenance_test`
- **`ota_progress` has `current_step` and no `step_key`.** The DJI table names
  `step_key` (`download_firmware`, `upgrade_firmware`), but the real dock
  uses `output.progress {percent, current_step}`. The backend `OtaProgressData`
  reads `current_step` as `OtaProgressStepEnum` (1 download, 2 update); the
  real dock values of `current_step` are not confirmed. The firmware state
  group with the new version goes out before the `ok` event, because the
  backend compares the stored version at the end. The simulator does not
  download the file and does not check `md5`. · real Dock 2, `OtaProgressData`,
  the backend · `maintenance_test`
- **`fileupload_start` gives the full object key.** The backend adds the STS
  prefix and `.zip`. The simulator uploads one ZIP per file, with one synthetic
  text log per boot index; `fingerprint` is the MD5 of the ZIP.
  `fileupload_progress` has `need_reply: 0`. `start_time` and `end_time` of
  `fileupload_list` are seconds, as the DJI table says; the DJI example shows
  milliseconds. The file entries (`module, size, key, device_sn, progress
  {current_step, total_step, progress, upload_rate, status, result,
  finish_time}`) are in `output.ext.files`. · real Dock 2, backend, DJI doc
  90.log.md · `maintenance_test`
- **`flight_areas_sync_progress` and `offline_map_sync_progress` have flat
  data:** `{status, reason, file: {name, checksum}}`. The backend type is not
  `EventsDataRequest`. Reasons: 1 unusable reply, 3 download failed or wrong
  size, 8 the GeoJSON did not load, 12 SHA256 mismatch. `OfflineMapSyncReasonEnum`
  has no checksum reason, so a bad map checksum is 3. The reason values are
  unverified on a real dock. · real Dock 2, `FlightAreasSyncProgress`,
  `FlightAreaSyncReasonEnum`, `OfflineMapSyncReasonEnum` · `services`,
  `maintenance_test`
- **The area type is `geofence_type` next to `geometry`,** not in
  `properties`. `properties` has `subType: "Circle"` with `radius` for a Point,
  and `enable`. · `FlightAreaFeature`, `FeatureProperty` · `maintenance_test`
- **`flight_areas_drone_location` is flat `{drone_locations: [{area_id,
  area_distance, is_in_area}]}` with `need_reply: 0`.** It goes out every
  `maintenance.flight_area_report_interval` while the aircraft flies.
  `area_distance` (double) is metres to the edge, `is_in_area` a bool. ·
  real Dock 2, DJI doc 130.custom-flight-area.md, `FlightAreasDroneLocation` ·
  `maintenance_test`
- **No-fly zones refuse targets only with `flight_safety_advance_check`.** A
  `takeoff_to_point` with the flag and a target in an enabled no-fly zone
  answers 336513; `fly_to_point` after such a takeoff does the same. With DRC
  up, entry into a no-fly zone or exit from a geofence sends
  `joystick_invalid_notify` reason 3. Not simulated: braking at the edge,
  checks of wayline missions, altitude limits and DJI GEO zones. · DJI doc
  71.error-code.md, `JoystickInvalidReasonEnum` · `maintenance_test`
- **`hms` is flat `{list: [...]}` with `need_reply: 0`.** `device_type` is
  `domain-type-subtype`; `DeviceEnum.find` throws for an unknown value.
  Unverified on a real dock. · `Hms`, `DeviceHms`, `DeviceEnum` ·
  `maintenance_test`
- **A conditional task (`task_type` 2) sends `flighttask_ready {flight_ids}`
  after the prepare reply.** The simulator does not check the ready conditions.
  On Dock 1, `flighttask_create` is the same as `flighttask_prepare`. The
  ready data is `{flight_ids}`. · real Dock 2, `FlighttaskReady`,
  `TaskTypeEnum` · `maintenance_test`
- **A failed KMZ download asks `flighttask_resource_get {flight_id}` once.**
  The reply has `output.file.url`. A second failure answers 514002. ·
  the backend · `maintenance_test`
- **`unlock_license_list` answers `{consistence: true, device_model_domain,
  licenses: []}`.** Switch and update answer 0 and store nothing. ·
  `UnlockLicenseListResponse`, DJI doc 170.flysafe.md · `services`
- **PSDK and ESDK services answer 314000.** The simulated aircraft has no PSDK
  speaker, widget or ESDK device. `live_set_quality` and `live_camera_change`
  stay stubs: the streamers have no quality levels. · choice ·
  `maintenance_test` (registry)

## Result codes

The DJI table is `reference/Cloud-API-Doc/docs/en/71.error-code.md` (fetch with
`tools/fetch_reference.sh`). Several codes in the current code do not match it:
the flight engine returns 514001–514005, which are not in the table; DRC returns
514100 ("Dock error") and 514101 ("driving rods failed") for broker problems;
routed services with no handler get 314001 ("Failed to distribute task"); the
real dock does not use 314001 for this, because it drops the methods that it
does not route. These codes are kept until
the web UI reaction to each is checked, because the golden suite pins them.
The manual flight uses table codes: 514116 (dock busy) for `takeoff_to_point`,
319022 for `fly_to_point`, 316024 for `return_home` before liftoff, 319043 for
`return_home_cancel` with no RTH or landing to cancel.
