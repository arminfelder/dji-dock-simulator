import { Component, inject, signal } from '@angular/core';
import { DatePipe } from '@angular/common';
import { Camera } from './components/camera';
import { Controls } from './components/controls';
import { MapView } from './components/map';
import { Status } from './components/status';
import { WireLog } from './components/wire-log';
import { LiveService } from './services/live.service';
import { getToken, setToken } from './services/token';

@Component({
  selector: 'ds-root',
  imports: [DatePipe, Camera, Controls, MapView, Status, WireLog],
  template: `
    <header>
      <span class="dot" [class.on]="live.connected()" [title]="live.connected() ? 'connected' : 'reconnecting'"></span>
      <b>dock-sim</b> <span class="muted">instructor console</span>
      @if (live.state(); as s) {
        <span class="muted">{{ s.dock.model }} · {{ s.drone.model }}</span>
        <span>{{ s.time | date: 'HH:mm:ss' }}</span>
        @if (s.time_scale !== 1) { <span class="badge">{{ s.time_scale }}× time</span> }
      }
      <span class="spacer"></span>
      <button (click)="settings.set(!settings())">Settings</button>
    </header>
    @if (settings()) {
      <div class="settings">
        <label>API token <input type="password" [value]="token()" (change)="saveToken($event)" /></label>
        <label>Map tiles <input [value]="tiles()" (change)="saveTiles($event)" /></label>
        <span class="muted">Tile changes apply after a reload.</span>
      </div>
    }
    <main>
      <div class="map"><ds-map /></div>
      <aside class="status"><ds-status /></aside>
      <aside class="controls">
        <ds-controls />
        <h4>Camera</h4>
        <ds-camera />
      </aside>
      <section class="wire"><ds-wire-log /></section>
    </main>
  `,
  styles: `
    :host { display: flex; flex-direction: column; height: 100vh; }
    header { display: flex; align-items: center; gap: 12px; padding: 6px 12px; background: var(--panel); border-bottom: 1px solid var(--line); }
    .spacer { flex: 1; }
    .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--bad); }
    .dot.on { background: var(--good); }
    .badge { background: var(--accent); color: #000; padding: 0 6px; border-radius: 4px; }
    .settings { display: flex; gap: 16px; padding: 6px 12px; background: var(--panel2); align-items: center; }
    main { flex: 1; min-height: 0; display: grid; grid-template-columns: 1fr 300px 360px; grid-template-rows: 1fr 240px; }
    .map { grid-row: 1; grid-column: 1; min-height: 0; }
    .status, .controls { overflow: auto; padding: 10px 12px; background: var(--panel); border-left: 1px solid var(--line); }
    .status { grid-row: 1; grid-column: 2; }
    .controls { grid-row: 1 / span 2; grid-column: 3; }
    .wire { grid-row: 2; grid-column: 1 / span 2; min-height: 0; border-top: 1px solid var(--line); background: var(--panel); }
  `,
})
export class App {
  readonly live = inject(LiveService);
  readonly settings = signal(false);
  readonly token = signal(getToken());
  readonly tiles = signal(localStorage.getItem('dock-sim-tiles') ?? '');

  saveToken(event: Event): void {
    const value = (event.target as HTMLInputElement).value.trim();
    setToken(value);
    this.token.set(value);
    this.live.reconnect();
  }

  saveTiles(event: Event): void {
    const value = (event.target as HTMLInputElement).value.trim();
    if (value) localStorage.setItem('dock-sim-tiles', value);
    else localStorage.removeItem('dock-sim-tiles');
    this.tiles.set(value);
  }
}
