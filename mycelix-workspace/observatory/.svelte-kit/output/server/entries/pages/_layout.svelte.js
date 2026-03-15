import { c as create_ssr_component, a as subscribe, e as escape } from "../../chunks/ssr.js";
import "../../chunks/stores.js";
import { c as conductorStatus } from "../../chunks/conductor.js";
const Layout = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let statusColor;
  let $conductorStatus, $$unsubscribe_conductorStatus;
  $$unsubscribe_conductorStatus = subscribe(conductorStatus, (value) => $conductorStatus = value);
  statusColor = {
    connecting: "bg-yellow-500",
    connected: "bg-green-500",
    disconnected: "bg-red-500",
    demo: "bg-yellow-500"
  }[$conductorStatus] ?? "bg-gray-500";
  $$unsubscribe_conductorStatus();
  return ` ${$conductorStatus !== "connected" ? `<div class="${"fixed bottom-0 left-0 right-0 z-50 px-4 py-2 text-sm flex items-center justify-center gap-2 " + escape(
    $conductorStatus === "demo" ? "bg-yellow-900/90 text-yellow-200" : "",
    true
  ) + " " + escape(
    $conductorStatus === "connecting" ? "bg-yellow-900/90 text-yellow-200" : "",
    true
  ) + " " + escape(
    $conductorStatus === "disconnected" ? "bg-red-900/90 text-red-200" : "",
    true
  )}"><span class="relative flex h-2.5 w-2.5">${$conductorStatus === "connecting" ? `<span class="animate-ping absolute inline-flex h-full w-full rounded-full bg-yellow-400 opacity-75"></span>` : ``} <span class="${"relative inline-flex rounded-full h-2.5 w-2.5 " + escape(statusColor, true)}"></span></span> <span>${$conductorStatus === "demo" ? `Demo Mode — showing simulated data (no Holochain conductor detected, retrying...)` : `${$conductorStatus === "connecting" ? `Connecting to Holochain conductor...` : `${$conductorStatus === "disconnected" ? `Disconnected from conductor` : ``}`}`}</span></div>` : ``}  <nav class="bg-gray-900 border-b border-gray-800 px-4 py-2" data-svelte-h="svelte-1duazax"><div class="container mx-auto flex items-center gap-1 overflow-x-auto text-sm"><a href="/" class="px-3 py-1.5 rounded text-gray-300 hover:bg-gray-800 hover:text-white transition-colors whitespace-nowrap">Dashboard</a> <span class="text-gray-700">|</span> <a href="/tend" class="px-3 py-1.5 rounded text-gray-300 hover:bg-gray-800 hover:text-white transition-colors whitespace-nowrap">TEND</a> <a href="/food" class="px-3 py-1.5 rounded text-gray-300 hover:bg-gray-800 hover:text-white transition-colors whitespace-nowrap">Food</a> <a href="/mutual-aid" class="px-3 py-1.5 rounded text-gray-300 hover:bg-gray-800 hover:text-white transition-colors whitespace-nowrap">Mutual Aid</a> <a href="/emergency" class="px-3 py-1.5 rounded text-gray-300 hover:bg-gray-800 hover:text-white transition-colors whitespace-nowrap">Emergency</a> <a href="/value-anchor" class="px-3 py-1.5 rounded text-gray-300 hover:bg-gray-800 hover:text-white transition-colors whitespace-nowrap">Value Anchor</a> <span class="text-gray-700">|</span> <a href="/governance" class="px-3 py-1.5 rounded text-gray-300 hover:bg-gray-800 hover:text-white transition-colors whitespace-nowrap">Governance</a> <a href="/network" class="px-3 py-1.5 rounded text-gray-300 hover:bg-gray-800 hover:text-white transition-colors whitespace-nowrap">Network</a> <a href="/analytics" class="px-3 py-1.5 rounded text-gray-300 hover:bg-gray-800 hover:text-white transition-colors whitespace-nowrap">Analytics</a> <a href="/attribution" class="px-3 py-1.5 rounded text-gray-300 hover:bg-gray-800 hover:text-white transition-colors whitespace-nowrap">Attribution</a></div></nav> ${slots.default ? slots.default({}) : ``}`;
});
export {
  Layout as default
};
