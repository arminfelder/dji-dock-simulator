import { Injectable, computed, signal } from '@angular/core';
import { SimState, WireEntry } from './model';
import { getToken } from './token';

const MAX_WIRE = 1000;
const MAX_TRACK = 2000;

// WebSocket /api/ws: state twice a second, and the MQTT messages of the dock.
@Injectable({ providedIn: 'root' })
export class LiveService {
  readonly state = signal<SimState | null>(null);
  readonly connected = signal(false);
  readonly wire = signal<WireEntry[]>([]);
  readonly wirePaused = signal(false);
  // Positions of the aircraft since its last takeoff.
  readonly track = signal<[number, number][]>([]);
  readonly airborne = computed(() => {
    const s = this.state();
    return !!s && !s.dock.drone_in_dock;
  });

  private socket?: WebSocket;
  private retry?: ReturnType<typeof setTimeout>;

  constructor() {
    this.open();
  }

  reconnect(): void {
    this.socket?.close();
  }

  apply(state: SimState): void {
    this.state.set(state);
    if (state.dock.drone_in_dock) {
      if (this.track().length) this.track.set([]);
      return;
    }
    const point: [number, number] = [state.drone.latitude, state.drone.longitude];
    const track = this.track();
    const last = track[track.length - 1];
    if (!last || last[0] !== point[0] || last[1] !== point[1]) {
      this.track.set([...track.slice(-MAX_TRACK + 1), point]);
    }
  }

  clearWire(): void {
    this.wire.set([]);
  }

  private open(): void {
    const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
    const token = getToken();
    const query = token ? `?token=${encodeURIComponent(token)}` : '';
    const ws = new WebSocket(`${scheme}://${location.host}/api/ws${query}`);
    this.socket = ws;
    ws.onopen = () => this.connected.set(true);
    ws.onclose = () => {
      this.connected.set(false);
      clearTimeout(this.retry);
      this.retry = setTimeout(() => this.open(), 2000);
    };
    ws.onmessage = (event) => {
      const message = JSON.parse(event.data as string);
      if (message.type === 'state') this.apply(message.state as SimState);
      else if (message.type === 'wire' && !this.wirePaused()) {
        const current = this.wire();
        const last = current.length ? current[current.length - 1].seq : 0;
        const entries = (message.entries as WireEntry[]).filter((e) => e.seq > last);
        if (entries.length) this.wire.set([...current, ...entries].slice(-MAX_WIRE));
      }
    };
  }
}
