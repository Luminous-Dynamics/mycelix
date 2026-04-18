// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { c as create_ssr_component, a as subscribe, o as onDestroy } from "../../../chunks/ssr.js";
import { w as writable } from "../../../chunks/index.js";
import { c as createFreshness, f as getAllPlots, m as getCommunityInputs, n as getNutrientSummary } from "../../../chunks/freshness.js";
import "../../../chunks/conductor.js";
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let $$unsubscribe_plots;
  let $$unsubscribe_harvests;
  let $$unsubscribe_nutrientSummary;
  let $$unsubscribe_resourceInputs;
  const plots = writable([]);
  $$unsubscribe_plots = subscribe(plots, (value) => value);
  const harvests = writable([]);
  $$unsubscribe_harvests = subscribe(harvests, (value) => value);
  const resourceInputs = writable([]);
  $$unsubscribe_resourceInputs = subscribe(resourceInputs, (value) => value);
  const nutrientSummary = writable(null);
  $$unsubscribe_nutrientSummary = subscribe(nutrientSummary, (value) => value);
  async function fetchData() {
    const [p, ri, ns] = await Promise.all([getAllPlots(), getCommunityInputs(), getNutrientSummary()]);
    plots.set(p);
    resourceInputs.set(ri);
    nutrientSummary.set(ns);
  }
  const freshness = createFreshness(fetchData, 12e4);
  const { stopPolling } = freshness;
  onDestroy(() => stopPolling());
  $$unsubscribe_plots();
  $$unsubscribe_harvests();
  $$unsubscribe_nutrientSummary();
  $$unsubscribe_resourceInputs();
  return `${$$result.head += `<!-- HEAD_svelte-1dz8gax_START -->${$$result.title = `<title>Food Production | Mycelix Observatory</title>`, ""}<!-- HEAD_svelte-1dz8gax_END -->`, ""} ${`<div class="text-white p-8 text-center text-gray-400" data-svelte-h="svelte-1v65znv">Loading food production data...</div>`}`;
});
export {
  Page as default
};
