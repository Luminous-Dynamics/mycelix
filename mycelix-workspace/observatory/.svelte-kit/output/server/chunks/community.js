// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
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
function getDefaultDao() {
  return _config.dao_did;
}
export {
  getCommunityConfig as a,
  getCanonicalItems as b,
  getBasketWeights as c,
  getBasketName as d,
  getDefaultDao as g
};
