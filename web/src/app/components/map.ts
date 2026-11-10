import { Component, ElementRef, OnDestroy, afterNextRender, effect, inject, signal, viewChild } from '@angular/core';
import * as L from 'leaflet';
import { ApiService } from '../services/api.service';
import { LiveService } from '../services/live.service';

type ClickMode = 'none' | 'dock' | 'drone';

const TILE_KEY = 'dock-sim-tiles';
const DEFAULT_TILES = 'https://tile.openstreetmap.org/{z}/{x}/{y}.png';

@Component({
  selector: 'ds-map',
  template: `
    <div class="map" #host [class.picking]="mode() !== 'none'"></div>
    <div class="map-tools">
      <button [class.active]="mode() === 'dock'" [disabled]="live.airborne()"
              title="Only while the aircraft is in the dock" (click)="toggle('dock')">Move dock here</button>
      <button [class.active]="mode() === 'drone'" [disabled]="!live.airborne()"
              title="Only while the aircraft flies" (click)="toggle('drone')">Teleport drone here</button>
      <button (click)="follow.set(!follow())" [class.active]="follow()">Follow</button>
      @if (mode() !== 'none') { <span class="hint">Click the map</span> }
    </div>
  `,
  styles: `
    :host { position: relative; display: block; height: 100%; }
    .map { position: absolute; inset: 0; }
    .map.picking { cursor: crosshair; }
    .map-tools { position: absolute; top: 8px; right: 8px; z-index: 1000; display: flex; gap: 6px; align-items: center; }
    .hint { background: var(--accent); color: #000; padding: 2px 8px; border-radius: 4px; }
  `,
})
export class MapView implements OnDestroy {
  readonly live = inject(LiveService);
  private readonly api = inject(ApiService);
  private readonly host = viewChild.required<ElementRef<HTMLDivElement>>('host');

  readonly mode = signal<ClickMode>('none');
  readonly follow = signal(true);

  private map?: L.Map;
  private dock?: L.Marker;
  private drone?: L.Marker;
  private trackLine?: L.Polyline;
  private route?: L.Polyline;
  private centred = false;

  constructor() {
    afterNextRender(() => this.init());
    effect(() => this.draw());
  }

  ngOnDestroy(): void {
    this.map?.remove();
  }

  toggle(mode: ClickMode): void {
    this.mode.set(this.mode() === mode ? 'none' : mode);
  }

  private init(): void {
    this.map = L.map(this.host().nativeElement, { zoomControl: true }).setView([47.263, 11.401], 16);
    L.tileLayer(localStorage.getItem(TILE_KEY) || DEFAULT_TILES, {
      maxZoom: 20,
      maxNativeZoom: 19,
      attribution: '© OpenStreetMap contributors',
    }).addTo(this.map);
    this.trackLine = L.polyline([], { color: '#ffb300', weight: 3 }).addTo(this.map);
    this.route = L.polyline([], { color: '#4fc3f7', weight: 2, dashArray: '6 6' }).addTo(this.map);
    this.dock = L.marker([0, 0], {
      icon: L.divIcon({ className: 'dock-icon', html: '<div>DOCK</div>', iconSize: [44, 20] }),
      zIndexOffset: 0,
    }).addTo(this.map);
    this.drone = L.marker([0, 0], {
      icon: L.divIcon({ className: 'drone-icon', html: '<div class="arrow">▲</div>', iconSize: [24, 24] }),
      zIndexOffset: 1000,
    }).addTo(this.map);
    this.map.on('click', (e: L.LeafletMouseEvent) => this.pick(e.latlng));
    this.map.on('dragstart', () => this.follow.set(false));
    this.draw();
  }

  private pick(at: L.LatLng): void {
    const mode = this.mode();
    if (mode === 'dock') this.api.moveDock(at.lat, at.lng);
    if (mode === 'drone') this.api.teleportDrone(at.lat, at.lng);
    this.mode.set('none');
  }

  private draw(): void {
    const s = this.live.state();
    const track = this.live.track();
    if (!s || !this.map || !this.dock || !this.drone) return;
    this.dock.setLatLng([s.dock.latitude, s.dock.longitude]);
    this.drone.setLatLng([s.drone.latitude, s.drone.longitude]);
    this.drone.setOpacity(s.dock.drone_in_dock ? 0.35 : 1);
    const arrow = this.drone.getElement()?.querySelector<HTMLElement>('.arrow');
    if (arrow) arrow.style.transform = `rotate(${s.drone.attitude_head}deg)`;
    this.trackLine?.setLatLngs(track);
    this.route?.setLatLngs((s.mission.waypoints ?? []).map((w) => [w.latitude, w.longitude] as [number, number]));
    if (!this.centred) {
      this.map.setView([s.dock.latitude, s.dock.longitude], 17);
      this.centred = true;
    } else if (this.follow()) {
      const target = s.dock.drone_in_dock ? L.latLng(s.dock.latitude, s.dock.longitude) : L.latLng(s.drone.latitude, s.drone.longitude);
      if (!this.map.getBounds().pad(-0.2).contains(target)) this.map.panTo(target);
    }
  }
}
