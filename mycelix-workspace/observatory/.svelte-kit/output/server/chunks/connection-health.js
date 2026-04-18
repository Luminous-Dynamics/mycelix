// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { w as writable, d as derived } from "./index.js";
import { c as conductorStatus } from "./conductor.js";
const queueCount = writable(0);
const DB_NAME = "mycelix-offline-queue";
const STORE_NAME = "submissions";
const DB_VERSION = 1;
let dbPromise = null;
function getDb() {
  if (dbPromise) return dbPromise;
  dbPromise = new Promise((resolve, reject) => {
    if (typeof indexedDB === "undefined") {
      reject(new Error("IndexedDB not available"));
      return;
    }
    const request = indexedDB.open(DB_NAME, DB_VERSION);
    request.onupgradeneeded = () => {
      const db = request.result;
      if (!db.objectStoreNames.contains(STORE_NAME)) {
        db.createObjectStore(STORE_NAME, { keyPath: "id" });
      }
    };
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error);
  });
  return dbPromise;
}
function tx(mode) {
  return getDb().then((db) => {
    const transaction = db.transaction(STORE_NAME, mode);
    const store = transaction.objectStore(STORE_NAME);
    const done = new Promise((resolve, reject) => {
      transaction.oncomplete = () => resolve();
      transaction.onerror = () => reject(transaction.error);
      transaction.onabort = () => reject(transaction.error);
    });
    return { store, done };
  });
}
function idbGetAll() {
  return tx("readonly").then(({ store, done }) => {
    const req = store.getAll();
    return done.then(() => req.result ?? []);
  });
}
async function getQueue() {
  try {
    const all = await idbGetAll();
    return all.sort((a, b) => a.created_at - b.created_at);
  } catch {
    return [];
  }
}
const _health = writable({
  internet: true,
  conductor: "disconnected",
  lastSuccessfulCall: 0,
  reconnectAttempts: 0,
  meshBridgeDetected: false
});
({ subscribe: _health.subscribe });
const connectionQuality = derived(_health, ($h) => {
  if (!$h.internet) return "offline";
  if ($h.conductor === "connected") {
    const staleMs = $h.lastSuccessfulCall > 0 ? Date.now() - $h.lastSuccessfulCall : 0;
    if (staleMs > 0 && staleMs > 12e4) return "degraded";
    return "excellent";
  }
  if ($h.conductor === "demo" || $h.conductor === "connecting") return "degraded";
  return "offline";
});
const connectionLabel = derived(
  [_health, connectionQuality],
  ([$h, $q]) => {
    switch ($q) {
      case "excellent":
        return $h.meshBridgeDetected ? "Connected + Mesh" : "Connected";
      case "good":
        return "Connected";
      case "degraded":
        if ($h.conductor === "demo") return "Demo Mode";
        if ($h.conductor === "connecting") return `Connecting (attempt ${$h.reconnectAttempts})`;
        return "Degraded";
      case "offline":
        return "Offline";
    }
  }
);
const qualityColor = derived(connectionQuality, ($q) => {
  switch ($q) {
    case "excellent":
      return "bg-green-500";
    case "good":
      return "bg-green-400";
    case "degraded":
      return "bg-yellow-500";
    case "offline":
      return "bg-red-500";
  }
});
function setConductorStatus(status) {
  _health.update((h) => ({ ...h, conductor: status }));
}
function setReconnectAttempts(n) {
  _health.update((h) => ({ ...h, reconnectAttempts: n }));
}
conductorStatus.subscribe((status) => {
  setConductorStatus(status);
  if (status === "connected") {
    setReconnectAttempts(0);
  }
});
export {
  connectionQuality as a,
  qualityColor as b,
  connectionLabel as c,
  getQueue as g,
  queueCount as q
};
