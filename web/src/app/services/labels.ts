// Names of the DJI Cloud API codes that the snapshot carries as numbers.

export const DOCK_MODE: Record<number, string> = {
  0: 'Idle',
  1: 'On-site debug',
  2: 'Remote debug',
  3: 'Upgrading',
  4: 'Working',
  5: 'To be calibrated',
};

export const DRONE_MODE: Record<number, string> = {
  0: 'Standby',
  1: 'Prepare',
  2: 'Ready',
  3: 'Manual',
  4: 'Auto takeoff',
  5: 'Wayline',
  6: 'Panorama',
  7: 'Active track',
  8: 'ADS-B avoidance',
  9: 'Return to home',
  10: 'Auto landing',
  11: 'Forced landing',
  12: 'Three-prop landing',
  13: 'Upgrading',
  14: 'Disconnected',
  15: 'APAS',
  16: 'Virtual stick',
  17: 'Live flight control',
};

export const FLIGHTTASK_STEP: Record<number, string> = {
  0: 'Initial',
  6: 'Dock working',
  7: 'Cover open',
  9: 'Wait for flight system',
  17: 'Download KMZ',
  18: 'Upload KMZ',
  23: 'Set home',
  24: 'Wayline running',
  27: 'Landing',
  28: 'Cover close',
  29: 'Dock idle',
  33: 'Media count',
  35: 'Notify result',
  255: 'No task',
  65534: 'None',
};

export const MANUAL_PHASE = ['Idle', 'Prepare', 'Takeoff', 'Hover', 'Fly to', 'POI', 'Wayline', 'RTH', 'Landing'];

export const COVER_STATE: Record<number, string> = { 0: 'Closed', 1: 'Open', 2: 'Half open', 3: 'Error' };

export const RAINFALL = ['None', 'Light', 'Moderate', 'Heavy'];

export const REFUSAL: Record<number, string> = {
  514103: 'Battery below 30 %',
  514107: 'Cover does not open',
  514121: 'Emergency stop pressed',
  514134: 'Heavy rainfall',
  514135: 'Wind speed too high',
};

export function label(table: Record<number, string> | string[], code: number | null | undefined): string {
  if (code === null || code === undefined) return '—';
  const name = (table as Record<number, string>)[code];
  return name ? name : `#${code}`;
}
