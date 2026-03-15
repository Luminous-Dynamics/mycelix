import { d as derived, w as writable } from "./index.js";
import "./conductor.js";
const community_name = "Roodepoort Resilience Cooperative";
const basket_name = "Roodepoort Resilience Basket";
const dao_did = "roodepoort-resilience";
const currency_code = "ZAR";
const currency_symbol = "R";
const labor_hour_value = 27.58;
const labor_hour_source = "South African National Minimum Wage 2026";
const tax_year_start_month = 3;
const tax_form_name = "IT12";
const tax_authority = "SARS";
const basket_items = [
  {
    key: "bread_750g",
    name: "Bread (750g)",
    unit: "loaf",
    default_price: 0.15,
    weight: 0.12
  },
  {
    key: "eggs_6",
    name: "Eggs (6-pack)",
    unit: "pack",
    default_price: 0.25,
    weight: 0.1
  },
  {
    key: "diesel_1l",
    name: "Diesel (1L)",
    unit: "litre",
    default_price: 0.5,
    weight: 0.12
  },
  {
    key: "milk_1l",
    name: "Milk (1L)",
    unit: "litre",
    default_price: 0.1,
    weight: 0.08
  },
  {
    key: "mealie_meal_2.5kg",
    name: "Maize meal (2.5kg)",
    unit: "bag",
    default_price: 0.2,
    weight: 0.15
  },
  {
    key: "solar_kwh",
    name: "Solar electricity (1 kWh)",
    unit: "kWh",
    default_price: 0.4,
    weight: 0.1
  },
  {
    key: "taxi_trip",
    name: "Taxi trip (local)",
    unit: "trip",
    default_price: 0.3,
    weight: 0.08
  },
  {
    key: "chicken_whole",
    name: "Whole chicken",
    unit: "chicken",
    default_price: 0.75,
    weight: 0.1
  },
  {
    key: "cooking_oil_750ml",
    name: "Cooking oil (750ml)",
    unit: "bottle",
    default_price: 0.2,
    weight: 0.08
  },
  {
    key: "sugar_2.5kg",
    name: "Sugar (2.5kg)",
    unit: "bag",
    default_price: 0.15,
    weight: 0.07
  }
];
const defaultConfig = {
  community_name,
  basket_name,
  dao_did,
  currency_code,
  currency_symbol,
  labor_hour_value,
  labor_hour_source,
  tax_year_start_month,
  tax_form_name,
  tax_authority,
  basket_items
};
let _config = defaultConfig;
function getCommunityConfig() {
  return _config;
}
function getCanonicalItems() {
  return _config.basket_items.map(({ key, name, unit, default_price }) => ({
    key,
    name,
    unit,
    default_price
  }));
}
function getBasketWeights() {
  return _config.basket_items.map(({ key, weight }) => ({
    item: key,
    weight
  }));
}
function getBasketName() {
  return _config.basket_name;
}
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
function purchasingPower(tendBalance) {
  const result = [];
  let enriched = [];
  enrichedItems.subscribe(($e) => {
    enriched = $e;
  })();
  for (const item of enriched) {
    const price = item.consensus_price ?? item.price_tend;
    if (price > 0) {
      const qty = tendBalance / price;
      result.push({
        name: item.name,
        quantity: qty >= 10 ? Math.round(qty).toString() : qty.toFixed(1),
        unit: item.unit
      });
    }
  }
  return result.sort((a, b) => parseFloat(b.quantity) - parseFloat(a.quantity));
}
export {
  CANONICAL_ITEMS as C,
  RESILIENCE_BASKET_NAME as R,
  purchasingPower as a,
  basketHistory as b,
  basketIndexData as c,
  enrichedItems as e,
  getCommunityConfig as g,
  hasBasket as h,
  oracleLoading as o,
  purchasingPowerIndex as p,
  volatilityData as v
};
