// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { c as create_ssr_component, a as subscribe, o as onDestroy } from "../../../chunks/ssr.js";
import { w as writable } from "../../../chunks/index.js";
import { c as createFreshness, e as getChannels, l as getMessages } from "../../../chunks/freshness.js";
import "../../../chunks/conductor.js";
const STORAGE_KEY = "mycelix-push-enabled";
function readStored() {
  if (typeof window === "undefined") return false;
  return localStorage.getItem(STORAGE_KEY) === "true";
}
const pushEnabled = writable(readStored());
pushEnabled.subscribe((v) => {
  if (typeof window !== "undefined") {
    localStorage.setItem(STORAGE_KEY, String(v));
  }
});
function isSupported() {
  return typeof window !== "undefined" && "Notification" in window;
}
function isEnabled() {
  return isSupported() && Notification.permission === "granted";
}
function showNotification(title, body, options) {
  if (!isEnabled()) return null;
  return new Notification(title, {
    body,
    icon: "/icon-192.png",
    badge: "/favicon.png",
    ...options
  });
}
function notifyEmergency(message) {
  if (message.priority !== "Flash" && message.priority !== "Immediate") {
    return null;
  }
  const title = message.priority === "Flash" ? "[FLASH] Emergency" : "[IMMEDIATE] Alert";
  const notification = showNotification(title, message.content, {
    tag: "emergency-" + Date.now(),
    requireInteraction: true
  });
  if ("vibrate" in navigator) {
    navigator.vibrate(message.priority === "Flash" ? [200, 100, 200, 100, 400] : [200, 100, 200]);
  }
  return notification;
}
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let $selectedChannel, $$unsubscribe_selectedChannel;
  let $$unsubscribe_pushEnabled;
  let $$unsubscribe_channels;
  let $$unsubscribe_messages;
  $$unsubscribe_pushEnabled = subscribe(pushEnabled, (value) => value);
  const channels = writable([]);
  $$unsubscribe_channels = subscribe(channels, (value) => value);
  const messages = writable([]);
  $$unsubscribe_messages = subscribe(messages, (value) => value);
  const selectedChannel = writable(null);
  $$unsubscribe_selectedChannel = subscribe(selectedChannel, (value) => $selectedChannel = value);
  const seenMessageIds = /* @__PURE__ */ new Set();
  async function fetchData() {
    const ch = await getChannels();
    channels.set(ch);
    const sel = $selectedChannel;
    if (sel) {
      const msgs = await getMessages(sel.id);
      for (const msg of msgs) {
        if (!seenMessageIds.has(msg.id)) {
          seenMessageIds.add(msg.id);
          if (msg.priority === "Flash" || msg.priority === "Immediate") {
            notifyEmergency(msg);
          }
        }
      }
      messages.set(msgs);
    } else if (ch.length > 0) {
      selectChannel(ch[0]);
    }
  }
  const freshness = createFreshness(fetchData, 3e4);
  const { stopPolling } = freshness;
  onDestroy(() => stopPolling());
  async function selectChannel(ch) {
    selectedChannel.set(ch);
    const msgs = await getMessages(ch.id);
    for (const msg of msgs) {
      seenMessageIds.add(msg.id);
    }
    messages.set(msgs);
  }
  $$unsubscribe_selectedChannel();
  $$unsubscribe_pushEnabled();
  $$unsubscribe_channels();
  $$unsubscribe_messages();
  return `${$$result.head += `<!-- HEAD_svelte-g84czu_START -->${$$result.title = `<title>Emergency Comms | Mycelix Observatory</title>`, ""}<!-- HEAD_svelte-g84czu_END -->`, ""} ${`<div class="text-white p-8 text-center text-gray-400" data-svelte-h="svelte-1gthlnx">Loading emergency channels...</div>`}`;
});
export {
  Page as default
};
