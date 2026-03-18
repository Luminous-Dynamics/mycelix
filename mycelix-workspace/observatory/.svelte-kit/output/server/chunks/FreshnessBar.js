import { c as create_ssr_component, a as subscribe, o as onDestroy, b as escape } from "./ssr.js";
import { u as timeAgo } from "./freshness.js";
const FreshnessBar = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let $lastUpdated, $$unsubscribe_lastUpdated;
  let $refreshing, $$unsubscribe_refreshing;
  let $loadError, $$unsubscribe_loadError;
  let { lastUpdated } = $$props;
  $$unsubscribe_lastUpdated = subscribe(lastUpdated, (value) => $lastUpdated = value);
  let { loadError } = $$props;
  $$unsubscribe_loadError = subscribe(loadError, (value) => $loadError = value);
  let { refreshing } = $$props;
  $$unsubscribe_refreshing = subscribe(refreshing, (value) => $refreshing = value);
  let { refresh } = $$props;
  let displayTime = "never";
  let tickInterval = null;
  onDestroy(() => {
    if (tickInterval) clearInterval(tickInterval);
  });
  if ($$props.lastUpdated === void 0 && $$bindings.lastUpdated && lastUpdated !== void 0) $$bindings.lastUpdated(lastUpdated);
  if ($$props.loadError === void 0 && $$bindings.loadError && loadError !== void 0) $$bindings.loadError(loadError);
  if ($$props.refreshing === void 0 && $$bindings.refreshing && refreshing !== void 0) $$bindings.refreshing(refreshing);
  if ($$props.refresh === void 0 && $$bindings.refresh && refresh !== void 0) $$bindings.refresh(refresh);
  {
    if ($lastUpdated) {
      displayTime = timeAgo($lastUpdated);
      if (!tickInterval) {
        tickInterval = setInterval(
          () => {
            displayTime = timeAgo($lastUpdated);
          },
          1e4
        );
      }
    }
  }
  $$unsubscribe_lastUpdated();
  $$unsubscribe_refreshing();
  $$unsubscribe_loadError();
  return `<div class="max-w-6xl mx-auto mb-4 flex items-center gap-3 text-xs text-gray-500" role="status" aria-live="polite">${$refreshing ? `<span class="inline-flex items-center gap-1.5" data-svelte-h="svelte-1xr1dw5"><span class="w-2 h-2 rounded-full bg-blue-400 animate-pulse" aria-hidden="true"></span>
      Refreshing...</span>` : `${$loadError ? `<span class="inline-flex items-center gap-1.5 text-red-400"><span class="w-2 h-2 rounded-full bg-red-400" aria-hidden="true"></span> ${escape($loadError)}</span> <button class="text-gray-400 hover:text-white underline" aria-label="Retry data refresh" data-svelte-h="svelte-1ja3kdm">Retry</button>` : `<span class="inline-flex items-center gap-1.5"><span class="w-2 h-2 rounded-full bg-green-400" aria-hidden="true"></span>
      Updated ${escape(displayTime)}</span>`}`} <button ${$refreshing ? "disabled" : ""} class="ml-auto text-gray-500 hover:text-white transition-colors disabled:opacity-50" aria-label="Refresh data now" title="Refresh now"><svg class="${"w-3.5 h-3.5 " + escape($refreshing ? "animate-spin" : "", true)}" fill="none" stroke="currentColor" viewBox="0 0 24 24" aria-hidden="true"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15"></path></svg></button></div>`;
});
export {
  FreshnessBar as F
};
