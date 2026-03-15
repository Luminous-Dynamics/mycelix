import { c as create_ssr_component, a as subscribe, d as add_attribute, e as escape, b as each } from "../../../chunks/ssr.js";
import { w as writable } from "../../../chunks/index.js";
import "../../../chunks/conductor.js";
import { h as hasBasket, a as purchasingPower } from "../../../chunks/value-basket.js";
function tierColor(tier) {
  switch (tier) {
    case "Normal":
      return "text-green-400";
    case "Elevated":
      return "text-yellow-400";
    case "High":
      return "text-orange-400";
    case "Emergency":
      return "text-red-400";
    default:
      return "text-gray-400";
  }
}
function tierBg(tier) {
  switch (tier) {
    case "Normal":
      return "bg-green-900/30 border-green-700";
    case "Elevated":
      return "bg-yellow-900/30 border-yellow-700";
    case "High":
      return "bg-orange-900/30 border-orange-700";
    case "Emergency":
      return "bg-red-900/30 border-red-700";
    default:
      return "bg-gray-900/30 border-gray-700";
  }
}
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let $oracle, $$unsubscribe_oracle;
  let $balance, $$unsubscribe_balance;
  let $hasBasket, $$unsubscribe_hasBasket;
  let $lastExchange, $$unsubscribe_lastExchange;
  let $listings, $$unsubscribe_listings;
  let $requests, $$unsubscribe_requests;
  $$unsubscribe_hasBasket = subscribe(hasBasket, (value) => $hasBasket = value);
  const balance = writable(null);
  $$unsubscribe_balance = subscribe(balance, (value) => $balance = value);
  const oracle = writable({
    vitality: 72,
    tier: "Normal",
    updated_at: Date.now()
  });
  $$unsubscribe_oracle = subscribe(oracle, (value) => $oracle = value);
  const listings = writable([]);
  $$unsubscribe_listings = subscribe(listings, (value) => $listings = value);
  const requests = writable([]);
  $$unsubscribe_requests = subscribe(requests, (value) => $requests = value);
  const lastExchange = writable(null);
  $$unsubscribe_lastExchange = subscribe(lastExchange, (value) => $lastExchange = value);
  let receiverDid = "";
  let hours = 1;
  let serviceDesc = "";
  const categories = [
    "General",
    "Maintenance",
    "Education",
    "Childcare",
    "Transport",
    "Food",
    "Care",
    "Construction",
    "Tech"
  ];
  $$unsubscribe_oracle();
  $$unsubscribe_balance();
  $$unsubscribe_hasBasket();
  $$unsubscribe_lastExchange();
  $$unsubscribe_listings();
  $$unsubscribe_requests();
  return `${$$result.head += `<!-- HEAD_svelte-1jgxnnz_START -->${$$result.title = `<title>TEND | Mycelix Observatory</title>`, ""}<!-- HEAD_svelte-1jgxnnz_END -->`, ""} <div class="text-white"> <header class="bg-gray-800/50 border-b border-gray-700 px-4 py-2"><div class="container mx-auto flex justify-between items-center"><div class="flex items-center gap-2" data-svelte-h="svelte-17evu3m"><span class="text-xl">🤝</span> <div><h1 class="text-lg font-bold">TEND — Time Exchange</h1> <p class="text-xs text-gray-400">All hours are equal. 1 TEND = 1 hour of labor.</p></div></div> ${$oracle ? `<div class="flex items-center gap-3"><div${add_attribute("class", `px-3 py-1 rounded border text-sm font-medium ${tierBg($oracle.tier)}`, 0)}><span${add_attribute("class", tierColor($oracle.tier), 0)}>Oracle: ${escape($oracle.tier)}</span></div> <div class="text-right"><p class="text-xs text-gray-400" data-svelte-h="svelte-b7re9v">Limit</p> <p class="text-sm font-bold">${$oracle.tier === "Normal" ? `±40` : `${$oracle.tier === "Elevated" ? `±60` : `${$oracle.tier === "High" ? `±80` : `±120`}`}`} TEND</p></div></div>` : ``}</div></header> <main class="container mx-auto p-6"> <div class="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-8"> <div class="bg-gray-800 rounded-lg border border-gray-700 p-6"><h2 class="text-gray-400 text-xs uppercase mb-4" data-svelte-h="svelte-16brzr9">Your Balance</h2> ${$balance ? `<p class="${[
    "text-4xl font-bold",
    ($balance.balance > 0 ? "text-green-400" : "") + " " + ($balance.balance < 0 ? "text-red-400" : "")
  ].join(" ").trim()}">${escape($balance.balance > 0 ? "+" : "")}${escape($balance.balance)} TEND</p> <div class="mt-4 grid grid-cols-3 gap-2 text-center"><div><p class="text-lg font-semibold text-green-400">${escape($balance.total_earned)}</p> <p class="text-xs text-gray-400" data-svelte-h="svelte-7trxox">Earned</p></div> <div><p class="text-lg font-semibold text-orange-400">${escape($balance.total_spent)}</p> <p class="text-xs text-gray-400" data-svelte-h="svelte-a4yvm4">Spent</p></div> <div><p class="text-lg font-semibold">${escape($balance.exchange_count)}</p> <p class="text-xs text-gray-400" data-svelte-h="svelte-xlfw2q">Exchanges</p></div></div> ${$hasBasket && $balance ? `<div class="mt-4 pt-3 border-t border-gray-700"><p class="text-xs text-gray-400 mb-2" data-svelte-h="svelte-1u4p8qh">Purchasing Power</p> <div class="space-y-1">${each(purchasingPower($balance.balance).slice(0, 4), (item) => {
    return `<div class="flex justify-between text-xs"><span class="text-gray-300">${escape(item.name)}</span> <span class="text-green-400 font-medium">${escape(item.quantity)} ${escape(item.unit)}</span> </div>`;
  })}</div> <a href="/value-anchor" class="block mt-2 text-xs text-blue-400 hover:text-blue-300" data-svelte-h="svelte-hgof1q">Edit basket</a></div>` : ``}` : `<p class="text-gray-500" data-svelte-h="svelte-1m0j2om">Loading...</p>`}</div>  <div class="bg-gray-800 rounded-lg border border-gray-700 p-6 lg:col-span-2"><h2 class="text-gray-400 text-xs uppercase mb-4" data-svelte-h="svelte-1adlwty">Record Exchange</h2> ${$lastExchange ? `<div class="mb-4 p-3 bg-green-900/30 border border-green-700 rounded-lg text-sm">Exchange recorded: ${escape($lastExchange.hours)}h to ${escape($lastExchange.receiver_did)} — ${escape($lastExchange.service_description)}</div>` : ``} <form class="grid grid-cols-1 md:grid-cols-2 gap-4"><div><label for="receiver" class="text-xs text-gray-400" data-svelte-h="svelte-l2in5j">Receiver DID</label> <input id="receiver" placeholder="member.did" class="w-full mt-1 bg-gray-700 border border-gray-600 rounded px-3 py-2 text-sm focus:outline-none focus:border-blue-500"${add_attribute("value", receiverDid, 0)}></div> <div><label for="hours" class="text-xs text-gray-400" data-svelte-h="svelte-1rgnpkw">Hours</label> <input id="hours" type="number" min="0.25" step="0.25" max="12" class="w-full mt-1 bg-gray-700 border border-gray-600 rounded px-3 py-2 text-sm focus:outline-none focus:border-blue-500"${add_attribute("value", hours, 0)}></div> <div><label for="desc" class="text-xs text-gray-400" data-svelte-h="svelte-1o1xoq8">Service Description</label> <input id="desc" placeholder="What was done?" class="w-full mt-1 bg-gray-700 border border-gray-600 rounded px-3 py-2 text-sm focus:outline-none focus:border-blue-500"${add_attribute("value", serviceDesc, 0)}></div> <div><label for="cat" class="text-xs text-gray-400" data-svelte-h="svelte-q99rna">Category</label> <select id="cat" class="w-full mt-1 bg-gray-700 border border-gray-600 rounded px-3 py-2 text-sm focus:outline-none focus:border-blue-500">${each(categories, (cat) => {
    return `<option${add_attribute("value", cat, 0)}>${escape(cat)}</option>`;
  })}</select></div> <div class="md:col-span-2"><button type="submit" ${"disabled"} class="w-full bg-blue-600 hover:bg-blue-700 disabled:bg-gray-600 disabled:cursor-not-allowed rounded px-4 py-2 text-sm font-medium transition-colors">${escape("Record Exchange")}</button></div></form></div></div>  <div class="bg-gray-800 rounded-lg border border-gray-700"><div class="p-4 border-b border-gray-700 flex items-center gap-4"><h2 class="text-lg font-semibold" data-svelte-h="svelte-1og555j">Service Marketplace</h2> <div class="flex gap-1 ml-auto"><button class="${"px-3 py-1 rounded text-sm " + escape(
    "bg-blue-600",
    true
  ) + " transition-colors"}">Listings (${escape($listings.length)})</button> <button class="${"px-3 py-1 rounded text-sm " + escape(
    "bg-gray-700 hover:bg-gray-600",
    true
  ) + " transition-colors"}">Requests (${escape($requests.length)})</button></div></div> <div class="p-4 space-y-3">${`${$listings.length ? each($listings, (listing) => {
    return `<div class="p-4 bg-gray-700/50 rounded-lg hover:bg-gray-700/70 transition-colors"><div class="flex justify-between items-start"><div><p class="font-medium">${escape(listing.title)}</p> <p class="text-xs text-gray-400 mt-1">${escape(listing.description)}</p></div> <div class="text-right"><span class="text-sm font-bold text-green-400">${escape(listing.hours_estimate)}h</span> <p class="text-xs text-gray-400 mt-1">${escape(listing.category)}</p> </div></div> <div class="flex justify-between text-xs text-gray-500 mt-2"><span>by ${escape(listing.provider_did)}</span> <span>${escape(new Date(listing.created_at).toLocaleDateString())}</span></div> </div>`;
  }) : `<p class="text-gray-500 text-center py-8" data-svelte-h="svelte-1mfc0sx">No listings yet</p>`}`}</div></div>  <div class="mt-6 grid grid-cols-1 md:grid-cols-3 gap-6" data-svelte-h="svelte-1lya21q"><div class="bg-gray-800 rounded-lg border border-gray-700 p-4"><h3 class="text-sm font-semibold mb-3 text-gray-300">Mutual Credit</h3> <p class="text-xs text-gray-400">Zero-sum: every credit = equal debit elsewhere. No money creation, no inflation. Community credit always sums to zero.</p></div> <div class="bg-gray-800 rounded-lg border border-gray-700 p-4"><h3 class="text-sm font-semibold mb-3 text-gray-300">Radical Equality</h3> <p class="text-xs text-gray-400">1 TEND = 1 hour, regardless of service. A doctor&#39;s hour equals a gardener&#39;s hour. Dignity in all labor.</p></div> <div class="bg-gray-800 rounded-lg border border-gray-700 p-4"><h3 class="text-sm font-semibold mb-3 text-gray-300">Oracle Tiers</h3> <p class="text-xs text-gray-400">Credit limits adjust with community stress. Normal ±40, Elevated ±60, High ±80, Emergency ±120 TEND.</p></div></div> <footer class="mt-8 text-center text-gray-500 text-sm" data-svelte-h="svelte-14daw5c"><p>TEND v1.0 · Commons Charter Article II, Section 2 · All Hours Are Equal</p></footer></main></div>`;
});
export {
  Page as default
};
