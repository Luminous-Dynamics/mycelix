// Mycelix Pulse — Service Worker v3
// Static-shell caching only. Security-sensitive/API responses are never intercepted.

const CACHE_NAME = 'mycelix-pulse-v3-security';
const STATIC_ASSETS = [
  '/',
  '/index.html',
  '/manifest.json',
  '/style/main.css',
];

function isCacheableStaticResponse(response) {
  if (!response || !response.ok || response.type !== 'basic') return false;
  const policy = response.headers.get('Cache-Control') || '';
  return !/(?:no-store|private)/i.test(policy);
}

async function cacheStatic(request, response) {
  if (!isCacheableStaticResponse(response)) return;
  const cache = await caches.open(CACHE_NAME);
  await cache.put(request, response.clone());
}

// Install: cache only the public application shell.
self.addEventListener('install', (event) => {
  event.waitUntil(caches.open(CACHE_NAME).then((cache) => cache.addAll(STATIC_ASSETS)));
  self.skipWaiting();
});

// Activate: purge every prior Pulse cache. Older workers could cache API responses,
// so keeping any prior cache would preserve credential material after an upgrade.
self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(keys.filter((key) => key !== CACHE_NAME).map((key) => caches.delete(key)))
    )
  );
  self.clients.claim();
});

self.addEventListener('fetch', (event) => {
  const request = event.request;
  const url = new URL(request.url);

  // Never let the service worker observe, cache, replay, or provide an offline
  // fallback for authentication, signing, or any future API endpoint.
  if (url.origin === self.location.origin && url.pathname.startsWith('/api/')) return;

  // Cache Storage is only used for idempotent same-origin public assets.
  if (request.method !== 'GET' || url.origin !== self.location.origin) return;

  const isHashedAsset = /\.(?:wasm|js|css)$/.test(url.pathname);
  const isMediaAsset = /\.(?:png|jpg|jpeg|svg|gif|woff2?|ttf|ico)$/.test(url.pathname);

  if (isHashedAsset || isMediaAsset) {
    event.respondWith(
      caches.match(request).then((cached) => {
        if (cached) return cached;
        return fetch(request).then((response) => {
          event.waitUntil(cacheStatic(request, response));
          return response;
        });
      })
    );
    return;
  }

  // Navigation is network-first with a static shell fallback. Do not cache
  // arbitrary navigation responses, which may vary by session or headers.
  if (request.mode === 'navigate') {
    event.respondWith(
      fetch(request, { cache: 'no-store' }).catch(() => caches.match('/index.html'))
    );
  }
});

// Background sync — ask the page to flush its IndexedDB/local queue. The
// service worker itself deliberately stores no mail or credential payloads.
self.addEventListener('sync', (event) => {
  if (event.tag === 'mycelix-offline-sync') {
    event.waitUntil(processOfflineQueue());
  }
});

async function processOfflineQueue() {
  const clients = await self.clients.matchAll();
  for (const client of clients) {
    client.postMessage({ type: 'flush-offline-queue' });
  }
}

self.addEventListener('push', (event) => {
  const data = event.data ? event.data.json() : { title: 'Mycelix Pulse', body: 'New activity' };
  event.waitUntil(
    self.registration.showNotification(data.title || 'Mycelix Pulse', {
      body: data.body || 'You have new messages',
      icon: '/icons/icon-192.png',
      badge: '/icons/icon-72.png',
      tag: data.tag || 'mycelix-notification',
      data: { url: data.url || '/' },
    })
  );
});

self.addEventListener('notificationclick', (event) => {
  event.notification.close();
  const url = event.notification.data?.url || '/';
  event.waitUntil(
    self.clients.matchAll({ type: 'window' }).then((clients) => {
      for (const client of clients) {
        if (client.url.includes(self.location.origin) && 'focus' in client) {
          return client.focus();
        }
      }
      return self.clients.openWindow(url);
    })
  );
});

self.addEventListener('periodicsync', (event) => {
  if (event.tag === 'mycelix-check-mail') {
    event.waitUntil(checkForNewMail());
  }
});

async function checkForNewMail() {
  const clients = await self.clients.matchAll();
  for (const client of clients) {
    client.postMessage({ type: 'check-new-mail' });
  }
}
