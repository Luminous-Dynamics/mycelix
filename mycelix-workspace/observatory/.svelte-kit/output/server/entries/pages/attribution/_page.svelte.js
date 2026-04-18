// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { c as create_ssr_component, o as onDestroy, b as escape } from "../../../chunks/ssr.js";
import "../../../chunks/conductor.js";
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  onDestroy(() => {
  });
  return `<div class="min-h-screen bg-gray-900 text-white"> <header class="bg-gray-800 border-b border-gray-700 px-6 py-4"><div class="flex items-center justify-between"><div class="flex items-center gap-3" data-svelte-h="svelte-3uw7s1"><h1 class="text-xl font-bold">Attribution Registry</h1> <span class="text-sm text-gray-400">Open-Source Stewardship Dashboard</span></div> <div class="flex items-center gap-2"><span class="${"inline-block w-2 h-2 rounded-full " + escape("bg-yellow-500", true)}"></span> <span class="text-sm text-gray-400">${escape("Demo")} Mode</span></div></div></header> <main class="p-6 max-w-7xl mx-auto">${`<div class="flex items-center justify-center h-64" data-svelte-h="svelte-17f3dz8"><div class="text-gray-400">Loading attribution data...</div></div>`}</main></div>`;
});
export {
  Page as default
};
