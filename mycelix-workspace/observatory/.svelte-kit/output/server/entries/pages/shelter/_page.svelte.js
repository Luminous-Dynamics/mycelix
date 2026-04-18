// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { c as create_ssr_component } from "../../../chunks/ssr.js";
import "../../../chunks/conductor.js";
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let units = [];
  let selectedUnitId = null;
  units.find((u) => u.unit_number === selectedUnitId) ?? null;
  return `<div class="min-h-screen bg-gray-950 text-gray-100 p-6"><div class="container mx-auto max-w-6xl"><h1 class="text-2xl font-bold mb-1" data-svelte-h="svelte-s1rzm5">Emergency Shelter</h1> <p class="text-gray-400 mb-6" data-svelte-h="svelte-11kxdmr">Available housing units for emergency placement. Filter by size, accessibility, and location.</p> ${`<div class="text-gray-400" data-svelte-h="svelte-1yb66ro">Loading shelter data...</div>`}</div></div>  ${``}`;
});
export {
  Page as default
};
