export interface Gnss {
  fix: number;
  quality: number;
  gps_number: number;
  rtk_number: number;
}

export interface Weather {
  noise: boolean;
  wind_speed: number;
  wind_direction: number;
  rainfall: number;
  temperature: number;
  humidity: number;
}

export interface HmsAlarm {
  code: string;
  level: number;
  module: number;
  device: string;
  in_the_sky: boolean;
  imminent: boolean;
  source: 'config' | 'runtime';
}

export interface Waypoint {
  latitude: number;
  longitude: number;
  height: number;
}

export interface SimState {
  time: number;
  time_scale: number;
  time_scale_adjustable: boolean;
  dock: {
    sn: string;
    model: string;
    online: boolean;
    mode_code: number;
    cover_state: number;
    putter_state: number;
    drone_in_dock: boolean;
    charge_state: number;
    charge_percent: number;
    flighttask_step_code: number;
    drc_state: number;
    latitude: number;
    longitude: number;
    altitude: number;
    gnss: Gnss;
    weather: Weather;
    weather_limits: { max_wind_takeoff: number; max_rainfall_takeoff: number };
    launch_refusal: number | null;
  };
  drone: {
    sn: string;
    model: string;
    mode_code: number;
    latitude: number;
    longitude: number;
    height: number;
    attitude_head: number;
    course: number;
    horizontal_speed: number;
    vertical_speed: number;
    battery: number;
    home_distance: number;
    gimbal_pitch: number;
    camera_heading: number;
    zoom_factor: number;
    rth_altitude: number;
    total_flight_time: number;
    total_flight_distance: number;
    camera_mode: number;
    recording: boolean;
    gnss: Gnss;
  };
  mission: {
    flying: boolean;
    flight_id?: string;
    status?: string;
    percent?: number;
    waypoint?: number;
    waypoints?: Waypoint[];
  };
  manual: { flying: boolean; phase: number };
  faults: { emergency_stop: boolean; cover_fault: boolean; hms: HmsAlarm[] };
}

export interface WireEntry {
  seq: number;
  time: number;
  direction: 'in' | 'out';
  topic: string;
  method: string;
  body: string;
}
