import { c as create_ssr_component, a as subscribe, e as escape, b as each } from "../../../chunks/ssr.js";
import { w as writable } from "../../../chunks/index.js";
import "../../../chunks/conductor.js";
function totalArea(plots2) {
  return plots2.reduce((sum, p) => sum + p.area_sqm, 0);
}
function totalHarvest(harvests2) {
  return harvests2.reduce((sum, h) => sum + h.quantity_kg, 0);
}
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let $plots, $$unsubscribe_plots;
  let $harvests, $$unsubscribe_harvests;
  const plots = writable([]);
  $$unsubscribe_plots = subscribe(plots, (value) => $plots = value);
  const harvests = writable([]);
  $$unsubscribe_harvests = subscribe(harvests, (value) => $harvests = value);
  $$unsubscribe_plots();
  $$unsubscribe_harvests();
  return `${$$result.head += `<!-- HEAD_svelte-1dz8gax_START -->${$$result.title = `<title>Food Production | Mycelix Observatory</title>`, ""}<!-- HEAD_svelte-1dz8gax_END -->`, ""} <div class="text-white"><header class="bg-gray-800/50 border-b border-gray-700 px-4 py-2"><div class="container mx-auto flex justify-between items-center"><div class="flex items-center gap-2" data-svelte-h="svelte-xj9bl4"><span class="text-xl">🌱</span> <div><h1 class="text-lg font-bold">Food Production</h1> <p class="text-xs text-gray-400">Community growing, harvest tracking</p></div></div> <div class="flex items-center gap-4"><div class="text-right"><p class="text-xs text-gray-400" data-svelte-h="svelte-1c7ihpo">Total Growing Area</p> <p class="text-lg font-bold text-green-400">${escape(totalArea($plots))} m²</p></div></div></div></header> <main class="container mx-auto p-6"> <div class="grid grid-cols-2 md:grid-cols-4 gap-4 mb-8"><div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-11ugyva">Active Plots</h3> <p class="text-2xl font-bold mt-1 text-green-400">${escape($plots.length)}</p></div> <div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-1i3p7ub">Total Area</h3> <p class="text-2xl font-bold mt-1">${escape(totalArea($plots))} m²</p></div> <div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-zhlkk6">Harvests Logged</h3> <p class="text-2xl font-bold mt-1 text-yellow-400">${escape($harvests.length)}</p></div> <div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-1r9218n">Total Yield</h3> <p class="text-2xl font-bold mt-1 text-orange-400">${escape(totalHarvest($harvests).toFixed(1))} kg</p></div></div>  <div class="flex gap-3 mb-6"><button class="px-4 py-2 bg-green-600 hover:bg-green-700 rounded text-sm font-medium transition-colors" data-svelte-h="svelte-jxo8ra">+ Register Plot</button> <button class="px-4 py-2 bg-yellow-600 hover:bg-yellow-700 rounded text-sm font-medium transition-colors" ${$plots.length === 0 ? "disabled" : ""}>+ Record Harvest</button></div>  ${``}  ${``}  <div class="bg-gray-800 rounded-lg border border-gray-700"><div class="p-4 border-b border-gray-700" data-svelte-h="svelte-1y9rqaa"><h2 class="text-lg font-semibold flex items-center gap-2"><span>🌿</span> Community Plots</h2></div> <div class="p-4 space-y-3">${$plots.length ? each($plots, (plot) => {
    return `<div class="p-4 bg-gray-700/50 rounded-lg"><div class="flex justify-between items-start"><div><p class="font-medium">${escape(plot.name)}</p> <p class="text-xs text-gray-400 mt-1">${escape(plot.location)}</p></div> <div class="text-right"><span class="text-sm font-bold text-green-400">${escape(plot.area_sqm)} m²</span> <p class="text-xs text-gray-400 mt-1">${escape(plot.plot_type)}</p> </div></div> <div class="flex justify-between text-xs text-gray-500 mt-2"><span>Owner: ${escape(plot.owner_did)}</span> <span>Since ${escape(new Date(plot.created_at).toLocaleDateString())}</span></div> </div>`;
  }) : `<p class="text-gray-500 text-center py-8" data-svelte-h="svelte-1e0u5tn">No plots registered yet. Be the first!</p>`}</div></div> <footer class="mt-8 text-center text-gray-500 text-sm" data-svelte-h="svelte-1ao9xde"><p>Food Production Tracker · Mycelix Commons</p></footer></main></div>`;
});
export {
  Page as default
};
