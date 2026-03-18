import { c as create_ssr_component, o as onDestroy, v as validate_component, d as add_attribute, b as escape } from "../../../chunks/ssr.js";
import { c as createFreshness, t as getAllInventoryItems, j as getLowStockItems } from "../../../chunks/freshness.js";
import "../../../chunks/conductor.js";
import { F as FreshnessBar } from "../../../chunks/FreshnessBar.js";
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let items = [];
  let lowStock = [];
  async function fetchData() {
    [items, lowStock] = await Promise.all([getAllInventoryItems(), getLowStockItems()]);
  }
  const freshness = createFreshness(fetchData, 12e4);
  const { lastUpdated, loadError, refreshing, stopPolling, refresh } = freshness;
  onDestroy(() => stopPolling());
  [...new Set(items.map((i) => i.category))].sort();
  new Set(lowStock.map((ls) => ls.item.id));
  return `<div class="min-h-screen bg-gray-950 text-gray-100 p-6"><div class="container mx-auto max-w-6xl">${validate_component(FreshnessBar, "FreshnessBar").$$render(
    $$result,
    {
      lastUpdated,
      loadError,
      refreshing,
      refresh
    },
    {},
    {}
  )} <div class="flex items-center justify-between mb-1"><h1 class="text-2xl font-bold" data-svelte-h="svelte-1c991d4">Community Supplies</h1> <button class="px-4 py-2 rounded-lg text-sm font-semibold transition-colors bg-amber-700 hover:bg-amber-600 text-white"${add_attribute(
    "aria-label",
    "Add new inventory item",
    0
  )}>${escape("+ Add Item")}</button></div> <p class="text-gray-400 mb-6" data-svelte-h="svelte-1nbx1be">Emergency inventory tracking — food, water, medical, fuel, and shelter supplies.</p> ${`<div class="text-gray-400" data-svelte-h="svelte-6jrvn2">Loading supply data...</div>`}</div></div>`;
});
export {
  Page as default
};
