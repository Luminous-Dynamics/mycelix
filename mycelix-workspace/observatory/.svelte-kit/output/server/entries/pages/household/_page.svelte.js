// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { c as create_ssr_component } from "../../../chunks/ssr.js";
import "../../../chunks/conductor.js";
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  return `<div class="min-h-screen bg-gray-950 text-gray-100 p-6"><div class="container mx-auto max-w-6xl"><h1 class="text-2xl font-bold mb-1" data-svelte-h="svelte-yli229">Household Emergency</h1> <p class="text-gray-400 mb-6" data-svelte-h="svelte-1vsh4em">Emergency plans, shared resources, and family coordination.</p> ${`<div class="text-gray-400" data-svelte-h="svelte-33zwro">Loading household data...</div>`}</div></div>`;
});
export {
  Page as default
};
