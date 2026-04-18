// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { c as create_ssr_component, a as subscribe, b as escape, e as each, d as add_attribute } from "../../../chunks/ssr.js";
import { v as volatilityData, e as enrichedItems, h as hasBasket, p as purchasingPowerIndex, b as basketHistory, o as oracleLoading, a as basketIndexData, C as CANONICAL_ITEMS, R as RESILIENCE_BASKET_NAME } from "../../../chunks/value-basket.js";
import { a as conductorStatus$ } from "../../../chunks/conductor.js";
import { a as getCommunityConfig } from "../../../chunks/community.js";
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let consensusCount;
  let personalCount;
  let displayItems;
  let volatilityColor;
  let $volatilityData, $$unsubscribe_volatilityData;
  let $enrichedItems, $$unsubscribe_enrichedItems;
  let $conductorStatus$, $$unsubscribe_conductorStatus$;
  let $hasBasket, $$unsubscribe_hasBasket;
  let $purchasingPowerIndex, $$unsubscribe_purchasingPowerIndex;
  let $$unsubscribe_basketHistory;
  let $oracleLoading, $$unsubscribe_oracleLoading;
  let $basketIndexData, $$unsubscribe_basketIndexData;
  $$unsubscribe_volatilityData = subscribe(volatilityData, (value) => $volatilityData = value);
  $$unsubscribe_enrichedItems = subscribe(enrichedItems, (value) => $enrichedItems = value);
  $$unsubscribe_conductorStatus$ = subscribe(conductorStatus$, (value) => $conductorStatus$ = value);
  $$unsubscribe_hasBasket = subscribe(hasBasket, (value) => $hasBasket = value);
  $$unsubscribe_purchasingPowerIndex = subscribe(purchasingPowerIndex, (value) => $purchasingPowerIndex = value);
  $$unsubscribe_basketHistory = subscribe(basketHistory, (value) => value);
  $$unsubscribe_oracleLoading = subscribe(oracleLoading, (value) => $oracleLoading = value);
  $$unsubscribe_basketIndexData = subscribe(basketIndexData, (value) => $basketIndexData = value);
  CANONICAL_ITEMS[0].key;
  let reportPrice = CANONICAL_ITEMS[0].default_price;
  let reportEvidence = "";
  let reportStatus = "idle";
  let taxYear = /* @__PURE__ */ (/* @__PURE__ */ new Date()).getFullYear().toString();
  consensusCount = $enrichedItems.filter((i) => i.source === "consensus").length;
  personalCount = $enrichedItems.filter((i) => i.source === "personal").length;
  displayItems = $enrichedItems;
  volatilityColor = $volatilityData ? $volatilityData.recommended_tier === "Emergency" ? "text-red-400" : $volatilityData.recommended_tier === "High" ? "text-orange-400" : $volatilityData.recommended_tier === "Elevated" ? "text-yellow-400" : "text-green-400" : "text-gray-400";
  $$unsubscribe_volatilityData();
  $$unsubscribe_enrichedItems();
  $$unsubscribe_conductorStatus$();
  $$unsubscribe_hasBasket();
  $$unsubscribe_purchasingPowerIndex();
  $$unsubscribe_basketHistory();
  $$unsubscribe_oracleLoading();
  $$unsubscribe_basketIndexData();
  return `${$$result.head += `<!-- HEAD_svelte-8mhws2_START -->${$$result.title = `<title>Value Anchor | Mycelix Observatory</title>`, ""}<!-- HEAD_svelte-8mhws2_END -->`, ""} <div class="text-white"><header class="bg-gray-800/50 border-b border-gray-700 px-4 py-2"><div class="container mx-auto flex justify-between items-center"><div class="flex items-center gap-2"><span class="text-xl" aria-hidden="true" data-svelte-h="svelte-8ir44h">⚖</span> <div><h1 class="text-lg font-bold" data-svelte-h="svelte-ddds6d">Value Anchor</h1> <p class="text-xs text-gray-400">${escape(RESILIENCE_BASKET_NAME)}</p></div></div> <div class="flex flex-col sm:flex-row items-start sm:items-center gap-2 sm:gap-4"> <div class="text-xs">${$conductorStatus$ === "connected" ? `<span class="text-green-400" data-svelte-h="svelte-1op8tqr">DHT Live</span>` : `<span class="text-yellow-400" data-svelte-h="svelte-1efz3i0">Demo Mode</span>`}</div>  ${$volatilityData ? `<div class="text-right"><p class="text-xs text-gray-400" data-svelte-h="svelte-86kal">Volatility</p> <p class="${"text-sm font-bold " + escape(volatilityColor, true)}">${escape($volatilityData.recommended_tier)}
              (${escape(($volatilityData.weekly_change * 100).toFixed(1))}%)</p></div>` : ``}  ${$hasBasket ? `<div class="text-right"><p class="text-xs text-gray-400" data-svelte-h="svelte-s76w1u">Basket Index</p> <p class="text-lg font-bold text-green-400">${escape($purchasingPowerIndex.toFixed(3))} TEND</p></div>` : ``}</div></div></header> <main class="container mx-auto p-6"> ${``} <div class="grid grid-cols-1 lg:grid-cols-3 gap-6"> <div class="bg-gray-800 rounded-lg border border-gray-700 lg:col-span-2"><div class="p-4 border-b border-gray-700 flex justify-between items-center"><div class="flex items-center gap-4"><h2 class="text-lg font-semibold" data-svelte-h="svelte-knb8nu">Community Prices</h2> <div class="flex gap-2 text-xs"><span class="text-green-400">${escape(consensusCount)} consensus</span> <span class="text-gray-500" data-svelte-h="svelte-1rwugb2">|</span> <span class="text-yellow-400">${escape(personalCount)} personal</span></div></div> <div class="flex gap-2"><button ${$oracleLoading ? "disabled" : ""} class="text-xs px-2 py-1 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-600 rounded transition-colors">${escape($oracleLoading ? "Refreshing..." : "Refresh DHT")}</button> <button class="text-xs px-2 py-1 bg-gray-700 hover:bg-gray-600 rounded transition-colors">${escape("Consensus Only")}</button> <button class="text-xs px-2 py-1 bg-gray-700 hover:bg-gray-600 rounded transition-colors" data-svelte-h="svelte-1edsvdg">Export</button> <button class="text-xs px-2 py-1 bg-gray-700 hover:bg-gray-600 rounded transition-colors" data-svelte-h="svelte-e5gdnq">Import</button></div></div> ${``}  <div class="p-4 space-y-2">${displayItems.length ? each(displayItems, (item) => {
    return `<div class="p-3 bg-gray-700/50 rounded-lg flex flex-col sm:flex-row sm:justify-between sm:items-center gap-2"><div class="flex items-center gap-3"> <div class="${"w-2 h-2 rounded-full " + escape(
      item.source === "consensus" ? "bg-green-400" : "bg-yellow-400",
      true
    )}"${add_attribute(
      "title",
      item.source === "consensus" ? "DHT Consensus" : "Personal Estimate",
      0
    )} aria-hidden="true"></div> <div><p class="font-medium text-sm">${escape(item.name)}</p> <p class="text-xs text-gray-400">${escape(item.unit)}</p> </div></div> <div class="flex items-center gap-4">${item.source === "consensus" ? `<div class="text-right"><span class="text-sm font-bold text-green-400">${escape(item.consensus_price?.toFixed(2))} TEND</span> <p class="text-xs text-gray-500">${escape(item.reporter_count)} reporters | SD ${escape(item.std_dev.toFixed(3))} |
                      <span${add_attribute(
      "class",
      item.signal_integrity > 0.8 ? "text-green-400" : item.signal_integrity > 0.5 ? "text-yellow-400" : "text-red-400",
      0
    )}>SI ${escape((item.signal_integrity * 100).toFixed(0))}%
                      </span></p> </div>` : `<span class="text-sm font-bold text-yellow-400">${escape(item.price_tend.toFixed(2))} TEND</span>`} <button class="text-xs text-blue-400 hover:text-blue-300" aria-label="${"Report price for " + escape(item.name, true)}">report</button></div> </div>`;
  }) : `<div class="text-center py-8" data-svelte-h="svelte-1yla079"><p class="text-gray-500 mb-2">No price data yet</p> <p class="text-xs text-gray-500">Be the first to report a price below</p> </div>`}</div></div>  <div class="space-y-6"> <div class="bg-gray-800 rounded-lg border border-green-700/50 p-4"><h3 class="text-sm font-semibold text-green-300 mb-3" data-svelte-h="svelte-8wuilp">Signal a Local Price</h3> <p class="text-xs text-gray-400 mb-3" data-svelte-h="svelte-c7bnc">Report what you paid today. Your signal joins the community consensus.</p> <form class="space-y-3"><div><label for="report-item" class="text-xs text-gray-400" data-svelte-h="svelte-qfj7px">Item</label> <select id="report-item" class="w-full mt-1 bg-gray-700 border border-gray-600 rounded px-3 py-2 text-sm focus:outline-none focus:border-green-500">${each(CANONICAL_ITEMS, (item) => {
    return `<option${add_attribute("value", item.key, 0)}>${escape(item.name)}</option>`;
  })}</select></div> <div><label for="report-price" class="text-xs text-gray-400" data-svelte-h="svelte-3gqtmv">Price (TEND)</label> <input id="report-price" type="number" min="0.01" step="0.01" class="w-full mt-1 bg-gray-700 border border-gray-600 rounded px-3 py-2 text-sm focus:outline-none focus:border-green-500"${add_attribute("value", reportPrice, 0)}> <p class="text-xs text-gray-500 mt-1" data-svelte-h="svelte-1ky5brn">1 TEND = 1 hour of labor</p></div> <div><label for="report-evidence" class="text-xs text-gray-400" data-svelte-h="svelte-146uu8i">Evidence (optional)</label> <input id="report-evidence" type="text" placeholder="e.g. Local shop, 2026-03-14" maxlength="512" class="w-full mt-1 bg-gray-700 border border-gray-600 rounded px-3 py-2 text-sm focus:outline-none focus:border-green-500"${add_attribute("value", reportEvidence, 0)}></div> <button type="submit" ${reportPrice <= 0 || reportStatus === "submitting" ? "disabled" : ""} class="w-full bg-green-600 hover:bg-green-700 disabled:bg-gray-600 disabled:cursor-not-allowed rounded px-3 py-2 text-sm font-medium transition-colors">${`${`${`Broadcast Price Signal`}`}`}</button></form></div>  ${$basketIndexData ? `<div class="bg-gray-800 rounded-lg border border-gray-700 p-4"><h3 class="text-sm font-semibold text-gray-300 mb-3" data-svelte-h="svelte-1amullx">Basket Breakdown</h3> <div class="space-y-2">${each($basketIndexData.item_prices, (ip) => {
    return `<div class="flex justify-between text-xs"><span class="text-gray-400">${escape(ip.item)}</span> <div class="text-right"><span class="text-white">${escape(ip.price.toFixed(2))}</span> <span class="text-gray-500">x ${escape((ip.weight * 100).toFixed(0))}%</span> <span class="text-green-400 ml-1">= ${escape(ip.weighted_price.toFixed(3))}</span></div> </div>`;
  })} <div class="border-t border-gray-700 pt-2 flex justify-between text-sm font-bold"><span data-svelte-h="svelte-lyxz78">Total Index</span> <span class="text-green-400">${escape($basketIndexData.index.toFixed(3))} TEND</span></div></div></div>` : ``}  ${$volatilityData ? `<div class="bg-gray-800 rounded-lg border border-gray-700 p-4"><h3 class="text-sm font-semibold text-gray-300 mb-2" data-svelte-h="svelte-17mst57">Economic Pulse</h3> <div class="space-y-2 text-xs"><div class="flex justify-between"><span class="text-gray-400" data-svelte-h="svelte-189tkqu">Weekly Change</span> <span class="${escape(volatilityColor, true) + " font-bold"}">${escape(($volatilityData.weekly_change * 100).toFixed(1))}%</span></div> <div class="flex justify-between"><span class="text-gray-400" data-svelte-h="svelte-g71q89">Tier</span> <span${add_attribute("class", volatilityColor, 0)}>${escape($volatilityData.recommended_tier)}</span></div> <div class="flex justify-between"><span class="text-gray-400" data-svelte-h="svelte-gl9t1f">TEND Limit Escalated</span> <span${add_attribute(
    "class",
    $volatilityData.escalated ? "text-red-400" : "text-gray-500",
    0
  )}>${escape($volatilityData.escalated ? "Yes" : "No")}</span></div> ${$volatilityData.recommended_tier !== "Normal" ? `<p class="text-yellow-400 mt-2">TEND credit limits adjusted to ${escape($volatilityData.recommended_tier)} tier.</p>` : ``}</div></div>` : ``}  <div class="bg-gray-800 rounded-lg border border-gray-700 p-4"><h3 class="text-sm font-semibold text-gray-300 mb-2" data-svelte-h="svelte-1t0mcrd">Tax Record Export</h3> <p class="text-xs text-gray-400 mb-3">Download TEND exchange history valued in ${escape(getCommunityConfig().currency_code)}
            for ${escape(getCommunityConfig().tax_authority)} (${escape(getCommunityConfig().tax_form_name)}) reporting.</p> <div class="flex gap-2 mb-3"><div class="flex-1"><label for="tax-year" class="text-xs text-gray-400" data-svelte-h="svelte-1bvp8fh">Tax Year</label> <input id="tax-year" type="number" min="2020" max="2030" class="w-full mt-1 bg-gray-700 border border-gray-600 rounded px-3 py-2 text-sm focus:outline-none focus:border-blue-500"${add_attribute("value", taxYear, 0)}></div></div> <button class="w-full bg-gray-600 hover:bg-gray-500 rounded px-3 py-2 text-sm font-medium transition-colors" data-svelte-h="svelte-13lm8ng">Download CSV</button> <p class="text-xs text-gray-500 mt-2">Valued at ${escape(getCommunityConfig().currency_symbol)}${escape(getCommunityConfig().labor_hour_value)}/hour
            (${escape(getCommunityConfig().labor_hour_source)})</p></div>  <div class="bg-gray-800/50 rounded-lg border border-gray-700/50 p-4" data-svelte-h="svelte-1dk5wy4"><h3 class="text-sm font-semibold text-gray-400 mb-2">How Consensus Works</h3> <ul class="text-xs text-gray-500 space-y-1"><li>Community members report local prices</li> <li>Top/bottom 10% are trimmed (anti-manipulation)</li> <li>Accuracy-weighted median = consensus price</li> <li>Reporters weighted by historical accuracy (EMA)</li> <li>SI% = Signal Integrity (avg reporter accuracy)</li> <li>Min 2 reporters required per item</li> <li>7-day rolling window</li> <li>20%+ weekly swing = TEND limit escalation</li></ul></div></div></div> <footer class="mt-8 text-center text-gray-500 text-sm" data-svelte-h="svelte-o5m8m0"><p>Value Anchor · Community-driven price consensus on Holochain DHT</p></footer></main></div>`;
});
export {
  Page as default
};
