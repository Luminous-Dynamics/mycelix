// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { d as derived, w as writable } from "./index.js";
import "./conductor.js";
import { b as getCanonicalItems, c as getBasketWeights, d as getBasketName } from "./community.js";
const CANONICAL_ITEMS = getCanonicalItems();
getBasketWeights();
const RESILIENCE_BASKET_NAME = getBasketName();
function loadBasket() {
  return /* @__PURE__ */ new Map();
}
function saveBasket(basket) {
  return;
}
function loadHistory() {
  return [];
}
function saveHistory(history) {
  return;
}
const _basket = writable(loadBasket());
const _history = writable(loadHistory());
const _consensus = writable(/* @__PURE__ */ new Map());
const _basketIndex = writable(null);
const _volatility = writable(null);
const _oracleLoading = writable(false);
_basket.subscribe((b) => saveBasket());
_history.subscribe((h) => saveHistory());
derived(
  _basket,
  ($b) => Array.from($b.entries()).map(([key, item]) => ({ ...item, key })).sort((a, b) => a.name.localeCompare(b.name))
);
const enrichedItems = derived(
  [_basket, _consensus],
  ([$basket, $consensus]) => {
    const result = [];
    const allKeys = /* @__PURE__ */ new Set();
    CANONICAL_ITEMS.forEach((c) => allKeys.add(c.key));
    for (const key of $basket.keys()) allKeys.add(key);
    for (const key of $consensus.keys()) allKeys.add(key);
    for (const key of allKeys) {
      const canonical = CANONICAL_ITEMS.find((c) => c.key === key);
      const personal = $basket.get(key);
      const consensus = $consensus.get(key);
      const name = personal?.name ?? canonical?.name ?? key;
      const unit = personal?.unit ?? canonical?.unit ?? "unit";
      const personalPrice = personal?.price_tend ?? canonical?.default_price ?? 0;
      result.push({
        key,
        name,
        unit,
        price_tend: consensus?.median_price ?? personalPrice,
        updated_at: personal?.updated_at ?? Date.now(),
        consensus_price: consensus?.median_price ?? null,
        reporter_count: consensus?.reporter_count ?? 0,
        std_dev: consensus?.std_dev ?? 0,
        source: consensus ? "consensus" : "personal",
        signal_integrity: consensus?.signal_integrity ?? 1
      });
    }
    return result.sort((a, b) => a.name.localeCompare(b.name));
  }
);
const purchasingPowerIndex = derived(
  [_basketIndex, _basket],
  ([$idx, $basket]) => {
    if ($idx && $idx.index > 0) return $idx.index;
    if ($basket.size === 0) return 0;
    let total = 0;
    for (const item of $basket.values()) total += item.price_tend;
    return total / $basket.size;
  }
);
const basketHistory = derived(_history, ($h) => $h);
const hasBasket = derived(
  [_basket, _consensus],
  ([$b, $c]) => $b.size > 0 || $c.size > 0
);
const volatilityData = derived(_volatility, ($v) => $v);
const oracleLoading = derived(_oracleLoading, ($l) => $l);
const basketIndexData = derived(_basketIndex, ($b) => $b);
export {
  CANONICAL_ITEMS as C,
  RESILIENCE_BASKET_NAME as R,
  basketIndexData as a,
  basketHistory as b,
  enrichedItems as e,
  hasBasket as h,
  oracleLoading as o,
  purchasingPowerIndex as p,
  volatilityData as v
};
