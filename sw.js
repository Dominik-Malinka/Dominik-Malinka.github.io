// Service worker: zapisuje stronę w pamięci telefonu, żeby działała bez internetu.
// v1.2: najpierw sieć (najnowsza wersja), pamięć telefonu tylko bez internetu.
// Po każdej zmianie plików strony zwiększ numer wersji (np. hamulec-v5).
const CACHE = 'hamulec-v4';
const FILES = ['./', './index.html', './manifest.webmanifest', './icons/icon-192.png', './icons/icon-512.png'];
const NET_TIMEOUT_MS = 4000;   // przy słabym zasięgu po tym czasie strona bierze kopię z pamięci

self.addEventListener('install', e => {
  // każdy plik osobno: brak jednego pliku nie blokuje zapisu pozostałych
  e.waitUntil(caches.open(CACHE).then(c => Promise.all(FILES.map(f =>
    fetch(f, { cache: 'no-store' }).then(r => r.ok && c.put(f, r)).catch(() => {})))));
  self.skipWaiting();
});

self.addEventListener('activate', e => {
  e.waitUntil(caches.keys()
    .then(keys => Promise.all(keys.filter(k => k !== CACHE).map(k => caches.delete(k))))
    .then(() => self.clients.claim()));
});

self.addEventListener('fetch', e => {
  if (e.request.method !== 'GET' || new URL(e.request.url).origin !== location.origin) return;
  e.respondWith((async () => {
    const cache = await caches.open(CACHE);
    try {
      // cache: 'no-store' omija pamięć przeglądarki, więc zawsze przychodzi plik z serwera
      const r = await Promise.race([
        fetch(e.request, { cache: 'no-store' }),
        new Promise((_, rej) => setTimeout(() => rej(new Error('timeout')), NET_TIMEOUT_MS))
      ]);
      if (r.ok) cache.put(e.request, r.clone());
      return r;
    } catch {
      return (await cache.match(e.request, { ignoreSearch: true })) ||
             (e.request.mode === 'navigate' ? await cache.match('./index.html') : undefined) ||
             Response.error();
    }
  })());
});
