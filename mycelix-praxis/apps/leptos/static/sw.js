// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// Mycelix Praxis — Shadow Node Service Worker (PWA Offline Resilience)

const CACHE_NAME = 'praxis-shell-v2';
const SHELL_ASSETS = [
    '/',
    '/index.html',
    '/manifest.json',
    '/content-manifest.json',
    '/icon-192.png',
    '/icon-512.png'
];

self.addEventListener('install', (event) => {
    event.waitUntil(
        caches.open(CACHE_NAME)
            .then((cache) => cache.addAll(SHELL_ASSETS))
            .then(() => self.skipWaiting())
    );
});

self.addEventListener('fetch', (event) => {
    const request = event.request;
    if (request.method !== 'GET' || new URL(request.url).origin !== self.location.origin) {
        return;
    }

    event.respondWith(
        caches.match(request).then(async (cached) => {
            if (cached) return cached;

            try {
                const response = await fetch(request);
                if (response.ok) {
                    const cache = await caches.open(CACHE_NAME);
                    await cache.put(request, response.clone());
                }
                return response;
            } catch (_) {
                if (request.mode === 'navigate') {
                    return (await caches.match('/index.html')) || (await caches.match('/'));
                }
                return Response.error();
            }
        })
    );
});

self.addEventListener('activate', (event) => {
    event.waitUntil(
        caches.keys()
            .then((names) => Promise.all(
                names.filter((name) => name !== CACHE_NAME).map((name) => caches.delete(name))
            ))
            .then(() => self.clients.claim())
    );
});
