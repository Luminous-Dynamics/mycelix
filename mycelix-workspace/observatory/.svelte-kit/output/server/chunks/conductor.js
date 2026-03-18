import "@holochain/client";
import { w as writable } from "./index.js";
let nextId = 0;
function createToastStore() {
  const { subscribe, update } = writable([]);
  function add(message, type = "success", durationMs = 4e3) {
    const id = nextId++;
    update((toasts2) => [...toasts2, { id, message, type }]);
    setTimeout(() => dismiss(id), durationMs);
  }
  function dismiss(id) {
    update((toasts2) => toasts2.filter((t) => t.id !== id));
  }
  return {
    subscribe,
    success: (msg) => add(msg, "success"),
    error: (msg) => add(msg, "error", 6e3),
    info: (msg) => add(msg, "info"),
    dismiss
  };
}
const toasts = createToastStore();
const conductorStatus = writable("disconnected");
const conductorStatus$ = { subscribe: conductorStatus.subscribe };
export {
  conductorStatus$ as a,
  conductorStatus as c,
  toasts as t
};
