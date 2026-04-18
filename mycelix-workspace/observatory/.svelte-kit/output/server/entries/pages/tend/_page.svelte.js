// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { c as create_ssr_component, a as subscribe, o as onDestroy } from "../../../chunks/ssr.js";
import { w as writable } from "../../../chunks/index.js";
import { c as createFreshness, o as getBalance, k as getOracleState, g as getDaoListings, a as getDaoRequests } from "../../../chunks/freshness.js";
import { h as hasBasket } from "../../../chunks/value-basket.js";
import "../../../chunks/conductor.js";
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let $requests, $$unsubscribe_requests;
  let $listings, $$unsubscribe_listings;
  let $$unsubscribe_oracle;
  let $$unsubscribe_balance;
  let $$unsubscribe_hasBasket;
  let $$unsubscribe_lastExchange;
  $$unsubscribe_hasBasket = subscribe(hasBasket, (value) => value);
  const balance = writable(null);
  $$unsubscribe_balance = subscribe(balance, (value) => value);
  const oracle = writable({
    vitality: 72,
    tier: "Normal",
    updated_at: Date.now()
  });
  $$unsubscribe_oracle = subscribe(oracle, (value) => value);
  const listings = writable([]);
  $$unsubscribe_listings = subscribe(listings, (value) => $listings = value);
  const requests = writable([]);
  $$unsubscribe_requests = subscribe(requests, (value) => $requests = value);
  const lastExchange = writable(null);
  $$unsubscribe_lastExchange = subscribe(lastExchange, (value) => value);
  let searchQuery = "";
  function matchesSearch(...fields) {
    if (!searchQuery.trim()) return true;
    const q = searchQuery.toLowerCase();
    return fields.some((f) => f && f.toLowerCase().includes(q));
  }
  function matchesCategory(category) {
    return true;
  }
  async function fetchData() {
    const [bal, ora, lst, req] = await Promise.all([getBalance("self.did"), getOracleState(), getDaoListings(), getDaoRequests()]);
    balance.set(bal);
    oracle.set(ora);
    listings.set(lst);
    requests.set(req);
  }
  const freshness = createFreshness(fetchData, 12e4);
  const { stopPolling } = freshness;
  onDestroy(() => stopPolling());
  $listings.filter((l) => matchesSearch(l.title, l.description) && matchesCategory(l.category));
  $requests.filter((r) => matchesSearch(r.title, r.description) && matchesCategory(r.category));
  $$unsubscribe_requests();
  $$unsubscribe_listings();
  $$unsubscribe_oracle();
  $$unsubscribe_balance();
  $$unsubscribe_hasBasket();
  $$unsubscribe_lastExchange();
  return `${$$result.head += `<!-- HEAD_svelte-1jgxnnz_START -->${$$result.title = `<title>TEND | Mycelix Observatory</title>`, ""}<!-- HEAD_svelte-1jgxnnz_END -->`, ""} ${`<div class="text-white p-8 text-center text-gray-400" data-svelte-h="svelte-16x6nn1">Loading TEND data...</div>`}`;
});
export {
  Page as default
};
