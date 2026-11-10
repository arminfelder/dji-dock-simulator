import { HttpInterceptorFn } from '@angular/common/http';

const KEY = 'dock-sim-token';

export function getToken(): string {
  return localStorage.getItem(KEY) ?? '';
}

export function setToken(token: string): void {
  if (token) localStorage.setItem(KEY, token);
  else localStorage.removeItem(KEY);
}

export const tokenInterceptor: HttpInterceptorFn = (req, next) => {
  const token = getToken();
  if (!token || !req.url.startsWith('/api')) return next(req);
  return next(req.clone({ setHeaders: { Authorization: `Bearer ${token}` } }));
};
