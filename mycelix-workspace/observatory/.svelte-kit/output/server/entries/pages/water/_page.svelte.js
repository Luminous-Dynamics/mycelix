// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { c as create_ssr_component, o as onDestroy, v as validate_component } from "../../../chunks/ssr.js";
import { c as createFreshness, h as getAllWaterSystems, i as getActiveWaterAlerts } from "../../../chunks/freshness.js";
import "../../../chunks/conductor.js";
import { F as FreshnessBar } from "../../../chunks/FreshnessBar.js";
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let systems = [];
  let alerts = [];
  async function fetchData() {
    [systems, alerts] = await Promise.all([getAllWaterSystems(), getActiveWaterAlerts()]);
  }
  const freshness = createFreshness(fetchData, 6e4);
  const { lastUpdated, loadError, refreshing, stopPolling, refresh } = freshness;
  onDestroy(() => stopPolling());
  return `<div class="min-h-screen bg-gray-950 text-gray-100 p-6"><div class="container mx-auto max-w-6xl"><h1 class="text-2xl font-bold mb-1" data-svelte-h="svelte-3gx7ts">Water Safety</h1> <p class="text-gray-400 mb-4" data-svelte-h="svelte-17a8iy4">Community water harvesting systems, storage levels, and quality monitoring.</p> ${validate_component(FreshnessBar, "FreshnessBar").$$render(
    $$result,
    {
      lastUpdated,
      loadError,
      refreshing,
      refresh
    },
    {},
    {}
  )} ${`<div class="text-gray-400" data-svelte-h="svelte-16ds3ie">Loading water data...</div>`}</div></div>`;
});
export {
  Page as default
};
