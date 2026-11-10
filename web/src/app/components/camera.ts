import { Component, OnDestroy, OnInit, signal } from '@angular/core';
import { getToken } from '../services/token';

const VIEWS = [
  { id: 'drone-wide', label: 'Wide' },
  { id: 'drone-zoom', label: 'Zoom' },
  { id: 'drone-ir', label: 'IR' },
  { id: 'dock', label: 'Dock' },
];

// Polls one rendered frame per second. The next frame loads only after the last one arrived.
@Component({
  selector: 'ds-camera',
  template: `
    <div class="bar">
      @for (v of views; track v.id) {
        <button [class.active]="view() === v.id" (click)="view.set(v.id)">{{ v.label }}</button>
      }
      <button (click)="running.set(!running())">{{ running() ? 'Stop' : 'Start' }}</button>
    </div>
    @if (src()) {
      <img [src]="src()" (load)="loaded()" (error)="loaded()" alt="camera view" />
    } @else {
      <p class="muted">Camera preview is off.</p>
    }
  `,
  styles: `
    .bar { display: flex; gap: 4px; margin-bottom: 6px; }
    img { width: 100%; aspect-ratio: 16 / 9; background: #000; border-radius: 4px; display: block; }
  `,
})
export class Camera implements OnInit, OnDestroy {
  readonly views = VIEWS;
  readonly view = signal('drone-wide');
  readonly running = signal(false);
  readonly src = signal('');
  private timer?: ReturnType<typeof setTimeout>;
  private waiting = false;

  ngOnInit(): void {
    this.tick();
  }

  ngOnDestroy(): void {
    clearTimeout(this.timer);
  }

  loaded(): void {
    this.waiting = false;
  }

  private tick(): void {
    if (!this.running()) {
      if (this.src()) this.src.set('');
    } else if (!this.waiting) {
      this.waiting = true;
      const token = getToken();
      const auth = token ? `&token=${encodeURIComponent(token)}` : '';
      this.src.set(`/api/camera/${this.view()}.jpg?w=640&t=${Date.now()}${auth}`);
    }
    this.timer = setTimeout(() => this.tick(), 1000);
  }
}
