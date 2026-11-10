import { Component, effect, inject, signal, untracked } from '@angular/core';
import { DecimalPipe } from '@angular/common';
import { ApiService } from '../services/api.service';
import { LiveService } from '../services/live.service';
import { RAINFALL } from '../services/labels';

type Tab = 'weather' | 'power' | 'faults' | 'position';

function num(event: Event): number {
  return Number((event.target as HTMLInputElement).value);
}

function text(event: Event): string {
  return (event.target as HTMLInputElement).value;
}

@Component({
  selector: 'ds-controls',
  imports: [DecimalPipe],
  template: `
    <nav class="tabs">
      @for (t of tabs; track t.id) {
        <button [class.active]="tab() === t.id" (click)="tab.set(t.id)">{{ t.label }}</button>
      }
    </nav>

    @switch (tab()) {
      @case ('weather') {
        <label>Wind speed <b>{{ wind() | number: '1.1-1' }} m/s</b>
          <input type="range" min="0" max="25" step="0.5" [value]="wind()" (input)="wind.set(num($event))" />
        </label>
        <label>Wind from <b>{{ direction() }}°</b>
          <div class="dial-row">
            <input type="range" min="0" max="355" step="5" [value]="direction()" (input)="direction.set(num($event))" />
            <svg viewBox="-20 -20 40 40" class="dial">
              <circle r="18" />
              <text y="-10" text-anchor="middle">N</text>
              <line y1="-14" y2="10" [attr.transform]="'rotate(' + (direction() + 180) + ')'" />
              <polygon points="-4,6 4,6 0,14" [attr.transform]="'rotate(' + (direction() + 180) + ')'" />
            </svg>
          </div>
        </label>
        <label>Rainfall
          <select [value]="rain()" (change)="rain.set(num($event))">
            @for (r of RAINFALL; track $index) { <option [value]="$index" [selected]="$index === rain()">{{ r }}</option> }
          </select>
        </label>
        <label>Temperature <b>{{ temperature() }} °C</b>
          <input type="range" min="-20" max="45" step="1" [value]="temperature()" (input)="temperature.set(num($event))" />
        </label>
        <label>Humidity <b>{{ humidity() }} %</b>
          <input type="range" min="0" max="100" step="1" [value]="humidity()" (input)="humidity.set(num($event))" />
        </label>
        @if (live.state(); as s) {
          <p class="muted">Launch limit: wind ≥ {{ s.dock.weather_limits.max_wind_takeoff }} m/s or
            {{ RAINFALL[s.dock.weather_limits.max_rainfall_takeoff] }} rain. Without GPS fix the aircraft drifts with the wind.</p>
        }
        <div class="row">
          <button class="primary" (click)="applyWeather()">Apply weather</button>
          <button (click)="api.resetWeather()">Random calm</button>
        </div>
      }

      @case ('power') {
        <label>Battery <b>{{ battery() }} %</b>
          <input type="range" min="0" max="100" step="1" [value]="battery()" (input)="battery.set(num($event))" />
        </label>
        <div class="row">
          <button class="primary" (click)="api.setBattery(battery())">Set battery</button>
          <button (click)="api.setBattery(25)">Drop to 25 % (RTH)</button>
        </div>
        <h4>Aircraft GNSS</h4>
        <label>GPS satellites <b>{{ droneGps() }}</b>
          <input type="range" min="0" max="30" step="1" [value]="droneGps()" (input)="droneGps.set(num($event))" />
        </label>
        <label>RTK satellites <b>{{ droneRtk() }}</b>
          <input type="range" min="0" max="30" step="1" [value]="droneRtk()" (input)="droneRtk.set(num($event))" />
        </label>
        <div class="row">
          <button class="primary" (click)="applyGnss()">Set GNSS</button>
          <button (click)="api.setGnss('drone', { gps_number: 0, rtk_number: 0, quality: 0, fix: 0 })">GPS loss</button>
          <button (click)="api.setGnss('drone', { gps_number: 19, rtk_number: 24, quality: 5, fix: 2 })">Restore</button>
        </div>
        @if (live.state(); as s) {
          <h4>Time</h4>
          <label>Time scale <b>{{ scale() }}×</b>
            <input type="range" min="0.5" max="20" step="0.5" [value]="scale()" (input)="scale.set(num($event))"
                   [disabled]="!s.time_scale_adjustable" />
          </label>
          <div class="row">
            <button class="primary" [disabled]="!s.time_scale_adjustable" (click)="api.setTimeScale(scale())">Set time scale</button>
            <button [disabled]="!s.time_scale_adjustable" (click)="scale.set(1); api.setTimeScale(1)">Real time</button>
          </div>
        }
      }

      @case ('faults') {
        @if (live.state(); as s) {
          <div class="row">
            <button [class.danger]="s.faults.emergency_stop" (click)="api.setFaults({ emergency_stop: !s.faults.emergency_stop })">
              Emergency stop: {{ s.faults.emergency_stop ? 'PRESSED' : 'released' }}</button>
            <button [class.danger]="s.faults.cover_fault" (click)="api.setFaults({ cover_fault: !s.faults.cover_fault })">
              Cover fault: {{ s.faults.cover_fault ? 'ON' : 'off' }}</button>
          </div>
          <h4>HMS alarms</h4>
          <table>
            @for (a of s.faults.hms; track a.code) {
              <tr>
                <td><code>{{ a.code }}</code></td><td>{{ a.device }}</td><td>level {{ a.level }}</td>
                <td>@if (a.source === 'runtime') { <button (click)="api.clearHms(a.code)">clear</button> } @else { config }</td>
              </tr>
            } @empty { <tr><td class="muted">none</td></tr> }
          </table>
          <div class="hms-form">
            <input placeholder="0x16100083" [value]="hmsCode()" (input)="hmsCode.set(text($event))" />
            <select (change)="hmsLevel.set(num($event))">
              <option value="0">notice</option><option value="1">reminder</option><option value="2" selected>warning</option>
            </select>
            <select (change)="hmsDevice.set(text($event))">
              <option value="dock">dock</option><option value="drone">drone</option>
            </select>
            <button class="primary" [disabled]="!hmsCode()" (click)="raise()">Raise</button>
          </div>
        }
      }

      @case ('position') {
        @if (live.state(); as s) {
          <p class="muted">Pick on the map, or type coordinates.
            <button (click)="fillPosition()">Use current</button></p>
          <h4>Dock</h4>
          <div class="coords">
            <input type="number" step="0.000001" [value]="dock()[0]" (change)="dock.set([num($event), dock()[1], dock()[2]])" />
            <input type="number" step="0.000001" [value]="dock()[1]" (change)="dock.set([dock()[0], num($event), dock()[2]])" />
            <input type="number" step="0.1" [value]="dock()[2]" title="Altitude above the ellipsoid, m"
                   (change)="dock.set([dock()[0], dock()[1], num($event)])" />
            <button class="primary" [disabled]="!s.dock.drone_in_dock"
                    (click)="api.moveDock(dock()[0], dock()[1], dock()[2])">Move dock</button>
          </div>
          <h4>Aircraft</h4>
          <div class="coords">
            <input type="number" step="0.000001" [value]="drone()[0]" (change)="drone.set([num($event), drone()[1], drone()[2]])" />
            <input type="number" step="0.000001" [value]="drone()[1]" (change)="drone.set([drone()[0], num($event), drone()[2]])" />
            <input type="number" step="1" [value]="drone()[2]" title="Height above the dock, m"
                   (change)="drone.set([drone()[0], drone()[1], num($event)])" />
            <button class="primary" [disabled]="s.dock.drone_in_dock"
                    (click)="api.teleportDrone(drone()[0], drone()[1], drone()[2])">Teleport</button>
          </div>
        }
      }
    }

    @if (api.error()) { <p class="error">{{ api.error() }}</p> }
  `,
  styles: `
    .tabs { display: flex; gap: 4px; margin-bottom: 12px; }
    label { display: block; margin: 10px 0; }
    label b { float: right; }
    input[type='range'], select { width: 100%; }
    .row { display: flex; gap: 6px; flex-wrap: wrap; margin: 10px 0; }
    .dial-row { display: flex; gap: 10px; align-items: center; }
    .dial { width: 56px; height: 56px; flex: none; }
    .dial circle { fill: var(--panel2); stroke: var(--muted); }
    .dial line { stroke: var(--accent); stroke-width: 2; }
    .dial polygon { fill: var(--accent); }
    .dial text { fill: var(--muted); font-size: 7px; }
    .coords { display: grid; grid-template-columns: 1fr 1fr 70px auto; gap: 4px; }
    .hms-form { display: grid; grid-template-columns: 1fr auto auto auto; gap: 4px; margin-top: 8px; }
    table { width: 100%; border-collapse: collapse; }
    td { padding: 2px 4px; border-bottom: 1px solid var(--line); }
    .error { color: var(--bad); }
  `,
})
export class Controls {
  readonly api = inject(ApiService);
  readonly live = inject(LiveService);
  readonly RAINFALL = RAINFALL;
  readonly num = num;
  readonly text = text;
  readonly tabs: { id: Tab; label: string }[] = [
    { id: 'weather', label: 'Weather' },
    { id: 'power', label: 'Battery · GPS · Time' },
    { id: 'faults', label: 'Faults' },
    { id: 'position', label: 'Position' },
  ];
  readonly tab = signal<Tab>('weather');

  readonly wind = signal(0);
  readonly direction = signal(270);
  readonly rain = signal(0);
  readonly temperature = signal(15);
  readonly humidity = signal(50);
  readonly battery = signal(100);
  readonly droneGps = signal(19);
  readonly droneRtk = signal(24);
  readonly scale = signal(1);
  readonly hmsCode = signal('');
  readonly hmsLevel = signal(2);
  readonly hmsDevice = signal('dock');
  readonly dock = signal<[number, number, number]>([0, 0, 0]);
  readonly drone = signal<[number, number, number]>([0, 0, 50]);

  private loaded = false;

  constructor() {
    // Take the current values once, so that the sliders start where the simulator is.
    effect(() => {
      const s = this.live.state();
      if (!s || this.loaded) return;
      this.loaded = true;
      untracked(() => {
        const w = s.dock.weather;
        if (!w.noise) {
          this.wind.set(w.wind_speed);
          this.direction.set(w.wind_direction);
          this.rain.set(w.rainfall);
          this.temperature.set(w.temperature);
          this.humidity.set(w.humidity);
        }
        this.battery.set(Math.round(s.drone.battery));
        this.droneGps.set(s.drone.gnss.gps_number);
        this.droneRtk.set(s.drone.gnss.rtk_number);
        this.scale.set(s.time_scale);
        this.fillPosition();
      });
    });
  }

  fillPosition(): void {
    const s = this.live.state();
    if (!s) return;
    this.dock.set([s.dock.latitude, s.dock.longitude, s.dock.altitude]);
    this.drone.set([s.drone.latitude, s.drone.longitude, Math.round(s.drone.height)]);
  }

  applyWeather(): void {
    this.api.setWeather({
      wind_speed: this.wind(),
      wind_direction: this.direction(),
      rainfall: this.rain(),
      temperature: this.temperature(),
      humidity: this.humidity(),
    });
  }

  applyGnss(): void {
    const gps = this.droneGps();
    this.api.setGnss('drone', {
      gps_number: gps,
      rtk_number: this.droneRtk(),
      quality: gps >= 12 ? 5 : gps >= 6 ? 3 : 0,
      fix: this.droneRtk() > 0 ? 2 : gps >= 6 ? 1 : 0,
    });
  }

  raise(): void {
    this.api.raiseHms({
      code: this.hmsCode(),
      level: this.hmsLevel(),
      module: 3,
      device: this.hmsDevice(),
      in_the_sky: this.live.airborne(),
      imminent: false,
    });
  }
}
