import { c as create_ssr_component, d as add_attribute, e as each, b as escape } from "../../../chunks/ssr.js";
import "@sveltejs/kit/internal";
import "../../../chunks/exports.js";
import "../../../chunks/utils.js";
import "@sveltejs/kit/internal/server";
import "../../../chunks/state.svelte.js";
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let step = 1;
  const features = [
    {
      icon: "&#9201;",
      title: "TEND",
      description: "Time-based mutual credit — 1 hour = 1 TEND, all labor equal",
      color: "border-green-500 text-green-400"
    },
    {
      icon: "&#9752;",
      title: "Food",
      description: "Track community gardens, harvests, and soil health",
      color: "border-emerald-500 text-emerald-400"
    },
    {
      icon: "&#9888;",
      title: "Emergency",
      description: "Priority messaging for crisis coordination",
      color: "border-red-500 text-red-400"
    },
    {
      icon: "&#9829;",
      title: "Mutual Aid",
      description: "Post and find help — neighbor to neighbor",
      color: "border-blue-500 text-blue-400"
    }
  ];
  return `${$$result.head += `<!-- HEAD_svelte-yp7zb1_START -->${$$result.title = `<title>Welcome — Mycelix Resilience Kit</title>`, ""}<!-- HEAD_svelte-yp7zb1_END -->`, ""} <div class="min-h-screen bg-gray-950 text-gray-100 flex items-center justify-center p-4 sm:p-6"><div class="max-w-2xl w-full"> <div class="flex items-center justify-center gap-2 mb-8" role="progressbar" aria-label="Onboarding progress"${add_attribute("aria-valuenow", step, 0)}${add_attribute("aria-valuemin", 1, 0)}${add_attribute("aria-valuemax", 3, 0)}>${each([1, 2, 3], (s) => {
    return `<div class="${"h-2 rounded-full transition-all duration-300 " + escape(
      s === step ? "w-10 bg-indigo-500" : s < step ? "w-6 bg-indigo-700" : "w-6 bg-gray-700",
      true
    )}"></div>`;
  })}</div>  ${`<div class="text-center mb-10" data-svelte-h="svelte-1vha14s"><h1 class="text-3xl sm:text-4xl font-bold text-white mb-3">Welcome to Mycelix Resilience Kit</h1> <p class="text-lg text-gray-400">Community-powered tools for mutual aid, food security, and emergency coordination</p></div> <div class="grid grid-cols-1 sm:grid-cols-2 gap-4 mb-10">${each(features, (card) => {
    return `<div class="${"border " + escape(card.color, true) + " bg-gray-900 rounded-lg p-5 transition-colors"}"><div class="text-2xl mb-2"><!-- HTML_TAG_START -->${card.icon}<!-- HTML_TAG_END --></div> <h3 class="text-lg font-semibold text-white mb-1">${escape(card.title)}</h3> <p class="text-sm text-gray-400">${escape(card.description)}</p> </div>`;
  })}</div> <div class="flex justify-center"><button aria-label="Go to next step" class="px-8 py-3 bg-indigo-600 hover:bg-indigo-500 text-white font-medium rounded-lg transition-colors" data-svelte-h="svelte-jybzqw">Next →</button></div> `}  <p class="text-center text-xs text-gray-600 mt-8">Step ${escape(step)} of 3</p></div></div>`;
});
export {
  Page as default
};
