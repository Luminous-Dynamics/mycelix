import { c as create_ssr_component, a as subscribe, o as onDestroy } from "../../../chunks/ssr.js";
import { c as createFreshness, g as getDaoListings, a as getDaoRequests, b as getServiceOffers, d as getServiceRequests, e as getChannels, f as getAllPlots, h as getAllWaterSystems, i as getActiveWaterAlerts, j as getLowStockItems, k as getOracleState } from "../../../chunks/freshness.js";
import { a as connectionQuality, b as qualityColor, c as connectionLabel, q as queueCount, g as getQueue } from "../../../chunks/connection-health.js";
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let $$unsubscribe_connectionQuality;
  let $$unsubscribe_qualityColor;
  let $$unsubscribe_connectionLabel;
  let $$unsubscribe_queueCount;
  $$unsubscribe_connectionQuality = subscribe(connectionQuality, (value) => value);
  $$unsubscribe_qualityColor = subscribe(qualityColor, (value) => value);
  $$unsubscribe_connectionLabel = subscribe(connectionLabel, (value) => value);
  $$unsubscribe_queueCount = subscribe(queueCount, (value) => value);
  let tendListings = 0;
  let aidOffers = 0;
  let aidRequests = 0;
  let waterAlerts = 0;
  let activityLog = [];
  function log(domain, message) {
    activityLog = [{ time: Date.now(), domain, message }, ...activityLog].slice(0, 50);
  }
  async function fetchData() {
    const results = await Promise.allSettled([
      getDaoListings(),
      getDaoRequests(),
      getServiceOffers(),
      getServiceRequests(),
      getChannels(),
      getAllPlots(),
      getAllWaterSystems(),
      getActiveWaterAlerts(),
      getLowStockItems(),
      getOracleState()
    ]);
    const prev = {
      tendListings,
      aidOffers,
      aidRequests,
      waterAlerts
    };
    if (results[0].status === "fulfilled") tendListings = results[0].value.length;
    if (results[1].status === "fulfilled") results[1].value.length;
    if (results[2].status === "fulfilled") aidOffers = results[2].value.length;
    if (results[3].status === "fulfilled") aidRequests = results[3].value.length;
    if (results[4].status === "fulfilled") results[4].value.length;
    if (results[5].status === "fulfilled") results[5].value.length;
    if (results[6].status === "fulfilled") results[6].value.length;
    if (results[7].status === "fulfilled") {
      const newAlerts = results[7].value.length;
      if (newAlerts > prev.waterAlerts && prev.waterAlerts > 0) {
        log("water", `${newAlerts - prev.waterAlerts} new water alert(s)`);
      }
      waterAlerts = newAlerts;
    }
    if (results[8].status === "fulfilled") results[8].value;
    if (results[9].status === "fulfilled") results[9].value;
    await getQueue();
    if (tendListings !== prev.tendListings) log("tend", `TEND listings: ${prev.tendListings} → ${tendListings}`);
    if (aidOffers !== prev.aidOffers) log("mutual-aid", `Aid offers: ${prev.aidOffers} → ${aidOffers}`);
    if (aidRequests !== prev.aidRequests) log("mutual-aid", `Aid requests: ${prev.aidRequests} → ${aidRequests}`);
  }
  const freshness = createFreshness(fetchData, 3e4);
  const { stopPolling } = freshness;
  onDestroy(() => stopPolling());
  $$unsubscribe_connectionQuality();
  $$unsubscribe_qualityColor();
  $$unsubscribe_connectionLabel();
  $$unsubscribe_queueCount();
  return `${$$result.head += `<!-- HEAD_svelte-wkwarw_START -->${$$result.title = `<title>Operator Dashboard | Mycelix Observatory</title>`, ""}<!-- HEAD_svelte-wkwarw_END -->`, ""} ${`<div class="text-white p-8 text-center text-gray-400" data-svelte-h="svelte-1dasfze">Loading operator dashboard...</div>`}`;
});
export {
  Page as default
};
