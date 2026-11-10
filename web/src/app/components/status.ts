import { Component, computed, inject } from '@angular/core';
import { DecimalPipe } from '@angular/common';
import { LiveService } from '../services/live.service';
import { COVER_STATE, DOCK_MODE, DRONE_MODE, FLIGHTTASK_STEP, MANUAL_PHASE, RAINFALL, REFUSAL, label } from '../services/labels';

@Component({
  selector: 'ds-status',
  imports: [DecimalPipe],
  template: `
    @if (live.state(); as s) {
      <section>
        <h3>Dock <small>{{ s.dock.sn }}</small></h3>
        <dl>
          <dt>Mode</dt><dd>{{ name(DOCK_MODE, s.dock.mode_code) }} @if (!s.dock.online) { <b class="bad">offline</b> }</dd>
          <dt>Cover</dt><dd>{{ name(COVER_STATE, s.dock.cover_state) }}</dd>
          <dt>Aircraft</dt><dd>{{ s.dock.drone_in_dock ? 'in dock' : 'out' }}</dd>
          <dt>Charging</dt><dd>{{ s.dock.charge_state ? 'yes' : 'no' }} · {{ s.dock.charge_percent }} %</dd>
          <dt>Task step</dt><dd>{{ name(FLIGHTTASK_STEP, s.dock.flighttask_step_code) }}</dd>
          <dt>Weather</dt>
          <dd>
            @if (s.dock.weather.noise) { random calm }
            @else {
              {{ s.dock.weather.wind_speed | number: '1.1-1' }} m/s from {{ s.dock.weather.wind_direction | number: '1.0-0' }}° ·
              {{ RAINFALL[s.dock.weather.rainfall] }} rain
            }
          </dd>
          <dt>GNSS</dt><dd>{{ s.dock.gnss.gps_number }} GPS · {{ s.dock.gnss.rtk_number }} RTK</dd>
          <dt>Launch</dt>
          <dd>
            @if (s.dock.launch_refusal; as code) { <b class="bad">refused {{ code }}: {{ name(REFUSAL, code) }}</b> }
            @else { <span class="good">allowed</span> }
          </dd>
        </dl>
      </section>
      <section>
        <h3>Aircraft <small>{{ s.drone.sn }}</small></h3>
        <div class="battery" [class.low]="s.drone.battery < 30">
          <div class="fill" [style.width.%]="s.drone.battery"></div>
          <span>{{ s.drone.battery | number: '1.0-1' }} %</span>
        </div>
        <dl>
          <dt>Mode</dt><dd>{{ name(DRONE_MODE, s.drone.mode_code) }}</dd>
          <dt>Height</dt><dd>{{ s.drone.height | number: '1.1-1' }} m · {{ s.drone.vertical_speed | number: '1.1-1' }} m/s</dd>
          <dt>Speed</dt><dd>{{ s.drone.horizontal_speed | number: '1.1-1' }} m/s · course {{ s.drone.course | number: '1.0-0' }}°</dd>
          <dt>Heading</dt><dd>{{ s.drone.attitude_head | number: '1.0-0' }}° · gimbal {{ s.drone.gimbal_pitch | number: '1.0-0' }}°</dd>
          <dt>Home</dt><dd>{{ s.drone.home_distance | number: '1.0-0' }} m</dd>
          <dt>GNSS</dt>
          <dd [class.bad]="s.drone.gnss.gps_number < 6">{{ s.drone.gnss.gps_number }} GPS · {{ s.drone.gnss.rtk_number }} RTK
            @if (s.drone.gnss.gps_number < 6) { (ATTI) }</dd>
          <dt>Camera</dt><dd>zoom {{ s.drone.zoom_factor | number: '1.0-1' }}× @if (s.drone.recording) { <b class="bad">● REC</b> }</dd>
          <dt>Flight</dt><dd>{{ s.drone.total_flight_time | number: '1.0-0' }} s · {{ s.drone.total_flight_distance | number: '1.0-0' }} m</dd>
        </dl>
      </section>
      <section>
        <h3>Task</h3>
        <dl>
          @if (s.mission.flight_id) {
            <dt>Mission</dt><dd>{{ s.mission.flight_id }}</dd>
            <dt>Status</dt><dd>{{ s.mission.status }} · {{ s.mission.percent }} % · WP {{ s.mission.waypoint }}</dd>
          } @else {
            <dt>Mission</dt><dd>none</dd>
          }
          <dt>Manual</dt><dd>{{ s.manual.flying ? MANUAL_PHASE[s.manual.phase] : 'idle' }}</dd>
        </dl>
        <div class="bar"><div [style.width.%]="progress()"></div></div>
      </section>
    } @else {
      <p class="muted">Waiting for the simulator…</p>
    }
  `,
  styles: `
    section { margin-bottom: 14px; }
    h3 small { color: var(--muted); font-weight: normal; margin-left: 6px; }
    .battery { position: relative; height: 18px; background: var(--panel2); border-radius: 4px; overflow: hidden; margin: 4px 0 8px; }
    .battery .fill { position: absolute; inset: 0 auto 0 0; background: var(--good); }
    .battery.low .fill { background: var(--bad); }
    .battery span { position: relative; padding-left: 6px; font-weight: 600; color: #000; }
    .bar { height: 6px; background: var(--panel2); border-radius: 3px; }
    .bar div { height: 100%; background: var(--accent); border-radius: 3px; }
  `,
})
export class Status {
  readonly live = inject(LiveService);
  readonly DOCK_MODE = DOCK_MODE;
  readonly DRONE_MODE = DRONE_MODE;
  readonly FLIGHTTASK_STEP = FLIGHTTASK_STEP;
  readonly COVER_STATE = COVER_STATE;
  readonly MANUAL_PHASE = MANUAL_PHASE;
  readonly RAINFALL = RAINFALL;
  readonly REFUSAL = REFUSAL;
  readonly name = label;
  readonly progress = computed(() => this.live.state()?.mission.percent ?? 0);
}
