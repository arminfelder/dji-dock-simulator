import { HttpClient, HttpErrorResponse } from '@angular/common/http';
import { Injectable, inject, signal } from '@angular/core';
import { Observable } from 'rxjs';
import { SimState } from './model';
import { LiveService } from './live.service';

// REST calls. Every change answers with the new snapshot, which goes to LiveService.
@Injectable({ providedIn: 'root' })
export class ApiService {
  private readonly http = inject(HttpClient);
  private readonly live = inject(LiveService);

  readonly error = signal<string>('');
  readonly busy = signal(false);

  private send(call: Observable<SimState>): void {
    this.busy.set(true);
    call.subscribe({
      next: (state) => {
        this.error.set('');
        this.live.apply(state);
        this.busy.set(false);
      },
      error: (e: HttpErrorResponse) => {
        const message = (e.error && typeof e.error === 'object' && 'error' in e.error) ? e.error.error : e.message;
        this.error.set(`${e.status || ''} ${message}`.trim());
        this.busy.set(false);
      },
    });
  }

  moveDock(latitude: number, longitude: number, altitude?: number): void {
    this.send(this.http.put<SimState>('/api/dock/position', { latitude, longitude, altitude }));
  }

  teleportDrone(latitude: number, longitude: number, height?: number): void {
    this.send(this.http.put<SimState>('/api/drone/position', { latitude, longitude, height }));
  }

  setBattery(percent: number): void {
    this.send(this.http.put<SimState>('/api/battery', { percent }));
  }

  setWeather(weather: Partial<{ wind_speed: number; wind_direction: number; rainfall: number; temperature: number; humidity: number }>): void {
    this.send(this.http.put<SimState>('/api/weather', weather));
  }

  resetWeather(): void {
    this.send(this.http.delete<SimState>('/api/weather'));
  }

  setGnss(target: 'dock' | 'drone', gnss: Partial<{ gps_number: number; rtk_number: number; quality: number; fix: number }>): void {
    this.send(this.http.put<SimState>(`/api/gnss/${target}`, gnss));
  }

  setTimeScale(time_scale: number): void {
    this.send(this.http.put<SimState>('/api/clock', { time_scale }));
  }

  setFaults(faults: Partial<{ emergency_stop: boolean; cover_fault: boolean }>): void {
    this.send(this.http.put<SimState>('/api/faults', faults));
  }

  raiseHms(alarm: { code: string; level: number; module: number; device: string; in_the_sky: boolean; imminent: boolean }): void {
    this.send(this.http.post<SimState>('/api/hms', alarm));
  }

  clearHms(code: string): void {
    this.send(this.http.delete<SimState>(`/api/hms/${encodeURIComponent(code)}`));
  }
}
