import { Component, computed, inject, signal } from '@angular/core';
import { DatePipe } from '@angular/common';
import { LiveService } from '../services/live.service';
import { WireEntry } from '../services/model';

// The method filter hides the two osd streams by default; they come every second.
const NOISY = /\/(osd|state)$/;

@Component({
  selector: 'ds-wire-log',
  imports: [DatePipe],
  template: `
    <div class="bar">
      <b>MQTT</b>
      <input placeholder="filter topic / method / body" [value]="filter()" (input)="filter.set(value($event))" />
      <label><input type="checkbox" [checked]="showOsd()" (change)="showOsd.set(!showOsd())" /> osd/state</label>
      <button (click)="live.wirePaused.set(!live.wirePaused())">{{ live.wirePaused() ? 'Resume' : 'Pause' }}</button>
      <button (click)="live.clearWire()">Clear</button>
      <span class="muted">{{ shown().length }} / {{ live.wire().length }}</span>
    </div>
    <div class="list">
      @for (e of shown(); track e.seq) {
        <div class="entry" [class.in]="e.direction === 'in'" (click)="toggle(e)">
          <span class="time">{{ e.time | date: 'HH:mm:ss.SSS' }}</span>
          <span class="dir">{{ e.direction === 'in' ? '←' : '→' }}</span>
          <span class="method">{{ e.method || '—' }}</span>
          <span class="topic">{{ e.topic }}</span>
          @if (open() === e.seq) { <pre>{{ pretty(e.body) }}</pre> }
        </div>
      }
    </div>
  `,
  styles: `
    :host { display: flex; flex-direction: column; height: 100%; }
    .bar { display: flex; gap: 8px; align-items: center; padding: 4px 8px; border-bottom: 1px solid var(--line); }
    .bar input[type='text'], .bar input:not([type]) { flex: 1; }
    .list { overflow: auto; flex: 1; font-family: var(--mono); font-size: 12px; }
    .entry { padding: 1px 8px; cursor: pointer; border-bottom: 1px solid var(--line); }
    .entry.in { background: rgba(79, 195, 247, 0.06); }
    .time { color: var(--muted); margin-right: 8px; }
    .dir { margin-right: 6px; }
    .method { display: inline-block; min-width: 200px; color: var(--accent); }
    .topic { color: var(--muted); }
    pre { white-space: pre-wrap; margin: 4px 0 6px 24px; color: var(--fg); }
  `,
})
export class WireLog {
  readonly live = inject(LiveService);
  readonly filter = signal('');
  readonly showOsd = signal(false);
  readonly open = signal<number | null>(null);

  readonly shown = computed(() => {
    const f = this.filter().toLowerCase();
    const osd = this.showOsd();
    return this.live
      .wire()
      .filter((e) => osd || !NOISY.test(e.topic))
      .filter((e) => !f || e.topic.toLowerCase().includes(f) || e.method.toLowerCase().includes(f) || e.body.toLowerCase().includes(f))
      .slice(-300)
      .reverse();
  });

  value(event: Event): string {
    return (event.target as HTMLInputElement).value;
  }

  toggle(e: WireEntry): void {
    this.open.set(this.open() === e.seq ? null : e.seq);
  }

  pretty(body: string): string {
    try {
      return JSON.stringify(JSON.parse(body), null, 2);
    } catch {
      return body;
    }
  }
}
