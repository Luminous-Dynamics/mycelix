import { c as create_ssr_component, a as subscribe, o as onDestroy } from "../../../chunks/ssr.js";
import { w as writable } from "../../../chunks/index.js";
import { c as createFreshness, b as getServiceOffers, d as getServiceRequests } from "../../../chunks/freshness.js";
import "../../../chunks/conductor.js";
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let $requests, $$unsubscribe_requests;
  let $offers, $$unsubscribe_offers;
  const offers = writable([]);
  $$unsubscribe_offers = subscribe(offers, (value) => $offers = value);
  const requests = writable([]);
  $$unsubscribe_requests = subscribe(requests, (value) => $requests = value);
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
    const [o, r] = await Promise.all([getServiceOffers(), getServiceRequests()]);
    offers.set(o);
    requests.set(r);
  }
  const freshness = createFreshness(fetchData, 12e4);
  const { stopPolling } = freshness;
  onDestroy(() => stopPolling());
  $offers.filter((o) => matchesSearch(o.title, o.description) && matchesCategory(o.category));
  $requests.filter((r) => matchesSearch(r.title, r.description) && matchesCategory(r.category));
  $$unsubscribe_requests();
  $$unsubscribe_offers();
  return `${$$result.head += `<!-- HEAD_svelte-1839n2y_START -->${$$result.title = `<title>Mutual Aid | Mycelix Observatory</title>`, ""}<!-- HEAD_svelte-1839n2y_END -->`, ""} ${`<div class="text-white p-8 text-center text-gray-400" data-svelte-h="svelte-1ucku6e">Loading mutual aid data...</div>`}`;
});
export {
  Page as default
};
