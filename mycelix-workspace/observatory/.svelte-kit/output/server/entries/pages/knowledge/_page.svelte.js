// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { c as create_ssr_component, b as escape, d as add_attribute, e as each } from "../../../chunks/ssr.js";
import "../../../chunks/conductor.js";
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let searchTag = "";
  let activeTag = "";
  const RESILIENCE_TAGS = [
    "food-production",
    "water-safety",
    "first-aid",
    "energy",
    "emergency",
    "maintenance",
    "permaculture",
    "cost-saving",
    "communications",
    "health",
    "baking",
    "infrastructure"
  ];
  return `<div class="min-h-screen bg-gray-950 text-gray-100 p-6"><div class="container mx-auto max-w-6xl"><h1 class="text-2xl font-bold mb-1" data-svelte-h="svelte-32pg9y">Community Knowledge</h1> <p class="text-gray-400 mb-6" data-svelte-h="svelte-gvfov5">Shared expertise, practical know-how, and fact-checked claims for community resilience.</p> ${``}  ${``}  <div class="mb-4"><button class="${"px-4 py-2 rounded-lg text-white transition-colors " + escape(
    "bg-purple-700 hover:bg-purple-600",
    true
  )}">${escape("+ Share Knowledge")}</button></div>  ${``}  <div class="mb-4"><form class="flex gap-2"><input type="text" placeholder="Search by topic (e.g., water-safety, first-aid)..." aria-label="Search knowledge by topic" class="flex-1 bg-gray-900 border border-gray-700 rounded-lg px-4 py-2 text-white placeholder-gray-500 focus:border-purple-500 focus:outline-none"${add_attribute("value", searchTag, 0)}> <button type="submit" class="px-4 py-2 bg-purple-700 hover:bg-purple-600 rounded-lg text-white transition-colors" data-svelte-h="svelte-dp1hmy">Search</button></form></div>  <div class="flex flex-wrap gap-2 mb-6">${each(RESILIENCE_TAGS, (tag) => {
    return `<button class="${"px-3 py-1 rounded-full text-sm transition-colors " + escape(
      activeTag === tag ? "bg-purple-700 text-white" : "bg-gray-800 text-gray-400 hover:bg-gray-700 hover:text-gray-200",
      true
    )}">${escape(tag)} </button>`;
  })}</div>  ${`<div class="text-gray-400" data-svelte-h="svelte-1bl81df">Searching...</div>`}</div></div>`;
});
export {
  Page as default
};
