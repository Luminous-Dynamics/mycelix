// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { c as create_ssr_component, a as subscribe, e as each, b as escape, d as add_attribute, v as validate_component } from "../../chunks/ssr.js";
import { p as page } from "../../chunks/stores.js";
import "../../chunks/stores2.js";
import { t as toasts, c as conductorStatus } from "../../chunks/conductor.js";
import { q as queueCount, c as connectionLabel, a as connectionQuality, b as qualityColor } from "../../chunks/connection-health.js";
import "../../chunks/value-basket.js";
import { d as derived, w as writable } from "../../chunks/index.js";
const css = {
  code: "@keyframes svelte-1hhxpg-slideIn{from{transform:translateX(100%);opacity:0}to{transform:translateX(0);opacity:1}}.animate-slide-in{animation:svelte-1hhxpg-slideIn 0.2s ease-out}",
  map: '{"version":3,"file":"Toast.svelte","sources":["Toast.svelte"],"sourcesContent":["<script lang=\\"ts\\">import { toasts } from \\"$lib/toast\\";\\nconst typeStyles = {\\n  success: \\"bg-green-800 border-green-600 text-green-100\\",\\n  error: \\"bg-red-800 border-red-600 text-red-100\\",\\n  info: \\"bg-blue-800 border-blue-600 text-blue-100\\"\\n};\\n<\/script>\\n\\n{#if $toasts.length > 0}\\n  <div class=\\"fixed top-4 right-4 z-[60] space-y-2 max-w-sm\\" aria-live=\\"polite\\" aria-label=\\"Notifications\\">\\n    {#each $toasts as toast (toast.id)}\\n      <div\\n        class=\\"px-4 py-3 rounded-lg border shadow-lg text-sm flex items-start gap-2 animate-slide-in {typeStyles[toast.type] ?? typeStyles.info}\\"\\n        role=\\"status\\"\\n      >\\n        <span class=\\"flex-1\\">{toast.message}</span>\\n        <button\\n          on:click={() => toasts.dismiss(toast.id)}\\n          class=\\"text-current opacity-60 hover:opacity-100 ml-2 shrink-0\\"\\n          aria-label=\\"Dismiss notification\\"\\n        >\\n          &#x2715;\\n        </button>\\n      </div>\\n    {/each}\\n  </div>\\n{/if}\\n\\n<style>\\n  @keyframes slideIn {\\n    from { transform: translateX(100%); opacity: 0; }\\n    to { transform: translateX(0); opacity: 1; }\\n  }\\n  :global(.animate-slide-in) {\\n    animation: slideIn 0.2s ease-out;\\n  }\\n</style>\\n"],"names":[],"mappings":"AA6BE,WAAW,qBAAQ,CACjB,IAAK,CAAE,SAAS,CAAE,WAAW,IAAI,CAAC,CAAE,OAAO,CAAE,CAAG,CAChD,EAAG,CAAE,SAAS,CAAE,WAAW,CAAC,CAAC,CAAE,OAAO,CAAE,CAAG,CAC7C,CACQ,iBAAmB,CACzB,SAAS,CAAE,qBAAO,CAAC,IAAI,CAAC,QAC1B"}'
};
const Toast = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let $toasts, $$unsubscribe_toasts;
  $$unsubscribe_toasts = subscribe(toasts, (value) => $toasts = value);
  const typeStyles = {
    success: "bg-green-800 border-green-600 text-green-100",
    error: "bg-red-800 border-red-600 text-red-100",
    info: "bg-blue-800 border-blue-600 text-blue-100"
  };
  $$result.css.add(css);
  $$unsubscribe_toasts();
  return `${$toasts.length > 0 ? `<div class="fixed top-4 right-4 z-[60] space-y-2 max-w-sm" aria-live="polite" aria-label="Notifications">${each($toasts, (toast) => {
    return `<div class="${"px-4 py-3 rounded-lg border shadow-lg text-sm flex items-start gap-2 animate-slide-in " + escape(typeStyles[toast.type] ?? typeStyles.info, true)}" role="status"><span class="flex-1">${escape(toast.message)}</span> <button class="text-current opacity-60 hover:opacity-100 ml-2 shrink-0" aria-label="Dismiss notification" data-svelte-h="svelte-wxdua1">✕</button> </div>`;
  })}</div>` : ``}`;
});
const translations = {
  en: {
    // Navigation
    "nav.dashboard": "Dashboard",
    "nav.resilience": "Resilience",
    "nav.tend": "TEND",
    "nav.food": "Food",
    "nav.mutual_aid": "Mutual Aid",
    "nav.emergency": "Emergency",
    "nav.value_anchor": "Value Anchor",
    "nav.water": "Water",
    "nav.household": "Household",
    "nav.knowledge": "Knowledge",
    "nav.care_circles": "Care Circles",
    "nav.shelter": "Shelter",
    "nav.supplies": "Supplies",
    "nav.operator": "Operator",
    "nav.governance": "Governance",
    "nav.network": "Network",
    "nav.analytics": "Analytics",
    "nav.attribution": "Attribution",
    // Common actions
    "action.save": "Save",
    "action.cancel": "Cancel",
    "action.submit": "Submit",
    "action.delete": "Delete",
    "action.export": "Export",
    "action.refresh": "Refresh",
    "action.search": "Search",
    "action.filter": "Filter",
    "action.back": "Back",
    "action.next": "Next",
    "action.join": "Join",
    "action.sync": "Sync",
    // Status
    "status.loading": "Loading...",
    "status.offline": "Offline",
    "status.connected": "Connected",
    "status.disconnected": "Disconnected",
    "status.demo_mode": "Demo Mode",
    "status.syncing": "Syncing...",
    // Domains
    "tend.balance": "TEND Balance",
    "tend.record_exchange": "Record Exchange",
    "tend.marketplace": "Marketplace",
    "food.plots": "Food Plots",
    "food.harvests": "Harvests",
    "emergency.channels": "Channels",
    "emergency.send_message": "Send Message",
    "water.systems": "Water Systems",
    "water.alerts": "Water Alerts",
    "supplies.inventory": "Inventory",
    "supplies.low_stock": "Low Stock",
    // Welcome / Onboarding
    "welcome.title": "Welcome to Mycelix Resilience Kit",
    "welcome.subtitle": "Community-powered tools for mutual aid, food security, and emergency coordination",
    "welcome.step1": "What is this?",
    "welcome.step2": "Your Community",
    "welcome.step3": "Get Started",
    "welcome.dont_show": "Don't show this again",
    "welcome.enter": "Enter the Kit",
    // Errors
    "error.load_failed": "Failed to load data",
    "error.save_failed": "Failed to save",
    "error.connection_failed": "Connection failed"
  },
  // Placeholder stubs — fill in for v0.2
  af: {},
  zu: {},
  st: {}
};
const locale = writable("en");
derived(
  locale,
  ($locale) => (key, params) => {
    let str = translations[$locale]?.[key] ?? translations.en[key] ?? key;
    if (params) {
      for (const [k, v] of Object.entries(params)) {
        str = str.replace(`{${k}}`, v);
      }
    }
    return str;
  }
);
const LocaleSelector = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let $locale, $$unsubscribe_locale;
  $$unsubscribe_locale = subscribe(locale, (value) => $locale = value);
  const locales = [
    { code: "en", label: "English" },
    { code: "af", label: "Afrikaans" },
    { code: "zu", label: "isiZulu" },
    { code: "st", label: "Sesotho" }
  ];
  $$unsubscribe_locale();
  return `<select${add_attribute("value", $locale, 0)} aria-label="Select language" class="bg-gray-800 border border-gray-700 rounded px-2 py-1 text-xs text-gray-300 focus:outline-none focus:border-indigo-500">${each(locales, (loc) => {
    return `<option${add_attribute("value", loc.code, 0)} ${loc.code !== "en" ? "disabled" : ""}>${escape(loc.label)}${escape(loc.code !== "en" ? " (v0.2)" : "")}</option>`;
  })}</select>`;
});
const Layout = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let statusColor;
  let currentPath;
  let $page, $$unsubscribe_page;
  let $conductorStatus, $$unsubscribe_conductorStatus;
  let $queueCount, $$unsubscribe_queueCount;
  let $connectionLabel, $$unsubscribe_connectionLabel;
  let $connectionQuality, $$unsubscribe_connectionQuality;
  let $qualityColor, $$unsubscribe_qualityColor;
  $$unsubscribe_page = subscribe(page, (value) => $page = value);
  $$unsubscribe_conductorStatus = subscribe(conductorStatus, (value) => $conductorStatus = value);
  $$unsubscribe_queueCount = subscribe(queueCount, (value) => $queueCount = value);
  $$unsubscribe_connectionLabel = subscribe(connectionLabel, (value) => $connectionLabel = value);
  $$unsubscribe_connectionQuality = subscribe(connectionQuality, (value) => $connectionQuality = value);
  $$unsubscribe_qualityColor = subscribe(qualityColor, (value) => $qualityColor = value);
  let mobileMenuOpen = false;
  function isActive(href) {
    if (href === "/") return currentPath === "/";
    return currentPath === href || currentPath.startsWith(href + "/");
  }
  function linkClass(href) {
    const base = "px-3 py-1.5 rounded transition-colors whitespace-nowrap";
    return isActive(href) ? `${base} bg-gray-700 text-white font-medium` : `${base} text-gray-300 hover:bg-gray-800 hover:text-white`;
  }
  const navGroups = [
    {
      label: "Overview",
      items: [
        { href: "/", label: "Dashboard" },
        { href: "/resilience", label: "Resilience" }
      ]
    },
    {
      label: "Resilience Kit",
      items: [
        { href: "/tend", label: "TEND" },
        { href: "/food", label: "Food" },
        { href: "/mutual-aid", label: "Mutual Aid" },
        { href: "/emergency", label: "Emergency" },
        {
          href: "/value-anchor",
          label: "Value Anchor"
        },
        { href: "/water", label: "Water" },
        { href: "/household", label: "Household" },
        { href: "/knowledge", label: "Knowledge" },
        {
          href: "/care-circles",
          label: "Care Circles"
        },
        { href: "/shelter", label: "Shelter" },
        { href: "/supplies", label: "Supplies" }
      ]
    },
    {
      label: "Observatory",
      items: [
        { href: "/admin", label: "Operator" },
        { href: "/governance", label: "Governance" },
        { href: "/network", label: "Network" },
        { href: "/analytics", label: "Analytics" },
        {
          href: "/attribution",
          label: "Attribution"
        }
      ]
    }
  ];
  statusColor = {
    connecting: "bg-yellow-500",
    connected: "bg-green-500",
    disconnected: "bg-red-500",
    demo: "bg-yellow-500"
  }[$conductorStatus] ?? "bg-gray-500";
  currentPath = $page.url.pathname;
  {
    if (currentPath) mobileMenuOpen = false;
  }
  $$unsubscribe_page();
  $$unsubscribe_conductorStatus();
  $$unsubscribe_queueCount();
  $$unsubscribe_connectionLabel();
  $$unsubscribe_connectionQuality();
  $$unsubscribe_qualityColor();
  return ` ${`${$conductorStatus !== "connected" ? `<div class="${"fixed bottom-0 left-0 right-0 z-50 px-4 py-2 text-sm flex items-center justify-center gap-2 " + escape(
    $conductorStatus === "demo" ? "bg-yellow-900/90 text-yellow-200" : "",
    true
  ) + " " + escape(
    $conductorStatus === "connecting" ? "bg-yellow-900/90 text-yellow-200" : "",
    true
  ) + " " + escape(
    $conductorStatus === "disconnected" ? "bg-red-900/90 text-red-200" : "",
    true
  )}"><span class="relative flex h-2.5 w-2.5">${$conductorStatus === "connecting" ? `<span class="animate-ping absolute inline-flex h-full w-full rounded-full bg-yellow-400 opacity-75"></span>` : ``} <span class="${"relative inline-flex rounded-full h-2.5 w-2.5 " + escape(statusColor, true)}"></span></span> <span>${$conductorStatus === "demo" ? `Demo Mode — showing simulated data (no Holochain conductor detected, retrying...)` : `${$conductorStatus === "connecting" ? `Connecting to Holochain conductor...` : `${$conductorStatus === "disconnected" ? `Disconnected from conductor` : ``}`}`}</span> ${$queueCount > 0 ? `<span class="ml-2 inline-flex items-center px-2 py-0.5 rounded-full text-xs font-medium bg-amber-600 text-amber-50">${escape($queueCount)} queued</span> <button ${""} class="ml-1 inline-flex items-center px-2 py-0.5 rounded-full text-xs font-medium bg-amber-500 hover:bg-amber-400 text-amber-950 transition-colors disabled:opacity-50 disabled:cursor-not-allowed">${escape("Sync")}</button>` : ``}</div>` : ``}`}  <nav class="bg-gray-900 border-b border-gray-800" aria-label="Main navigation"><div class="container mx-auto px-4 py-2"> <div class="hidden md:flex items-center gap-1 text-sm overflow-x-auto">${each(navGroups, (group, i) => {
    return `${i > 0 ? `<span class="text-gray-700 mx-1" aria-hidden="true" data-svelte-h="svelte-1qclpqg">|</span>` : ``} ${each(group.items, (item) => {
      return `<a${add_attribute("href", item.href, 0)}${add_attribute("class", linkClass(item.href), 0)}>${escape(item.label)}</a>`;
    })}`;
  })} <span class="ml-auto" aria-hidden="true"></span> <span class="flex items-center gap-1.5 text-xs text-gray-400 pl-2 border-l border-gray-700"${add_attribute("title", $connectionLabel, 0)}><span class="relative flex h-2 w-2">${$connectionQuality === "degraded" ? `<span class="${"animate-ping absolute inline-flex h-full w-full rounded-full " + escape($qualityColor, true) + " opacity-75"}"></span>` : ``} <span class="${"relative inline-flex rounded-full h-2 w-2 " + escape($qualityColor, true)}"></span></span> ${escape($connectionLabel)}</span> <span class="pl-2 border-l border-gray-700">${validate_component(LocaleSelector, "LocaleSelector").$$render($$result, {}, {}, {})}</span></div>  <div class="md:hidden flex items-center justify-between"><a href="/" class="text-white font-semibold text-sm" data-svelte-h="svelte-rjz6bz">Mycelix</a> <button class="p-2 rounded text-gray-300 hover:bg-gray-800 hover:text-white"${add_attribute(
    "aria-label",
    mobileMenuOpen ? "Close navigation menu" : "Open navigation menu",
    0
  )}${add_attribute("aria-expanded", mobileMenuOpen, 0)}>${mobileMenuOpen ? `<svg class="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24" aria-hidden="true"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M6 18L18 6M6 6l12 12"></path></svg>` : `<svg class="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24" aria-hidden="true"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M4 6h16M4 12h16M4 18h16"></path></svg>`}</button></div> ${mobileMenuOpen ? `<div class="md:hidden mt-2 pb-2 space-y-3">${each(navGroups, (group) => {
    return `<div><p class="text-xs text-gray-500 uppercase tracking-wider px-3 mb-1">${escape(group.label)}</p> <div class="grid grid-cols-2 gap-1">${each(group.items, (item) => {
      return `<a${add_attribute("href", item.href, 0)} class="${escape(linkClass(item.href), true) + " text-sm block"}">${escape(item.label)}</a>`;
    })}</div> </div>`;
  })}</div>` : ``}</div></nav> ${validate_component(Toast, "Toast").$$render($$result, {}, {}, {})} ${slots.default ? slots.default({}) : ``}`;
});
export {
  Layout as default
};
