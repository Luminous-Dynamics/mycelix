import { c as create_ssr_component, a as subscribe, e as escape, b as each, d as add_attribute } from "../../../chunks/ssr.js";
import { w as writable } from "../../../chunks/index.js";
import "../../../chunks/conductor.js";
function priorityColor(p) {
  switch (p) {
    case "Flash":
      return "bg-red-600 text-white";
    case "Immediate":
      return "bg-orange-500/20 text-orange-400 border border-orange-500/50";
    case "Priority":
      return "bg-yellow-500/20 text-yellow-400 border border-yellow-500/50";
    case "Routine":
      return "bg-gray-600 text-gray-300";
    default:
      return "bg-gray-600 text-gray-300";
  }
}
function priorityBorder(p) {
  switch (p) {
    case "Flash":
      return "border-l-red-500";
    case "Immediate":
      return "border-l-orange-400";
    case "Priority":
      return "border-l-yellow-400";
    default:
      return "border-l-gray-600";
  }
}
function formatTime(ts) {
  return new Date(ts).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
}
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let activeChannel;
  let $selectedChannel, $$unsubscribe_selectedChannel;
  let $channels, $$unsubscribe_channels;
  let $messages, $$unsubscribe_messages;
  const channels = writable([]);
  $$unsubscribe_channels = subscribe(channels, (value) => $channels = value);
  const messages = writable([]);
  $$unsubscribe_messages = subscribe(messages, (value) => $messages = value);
  const selectedChannel = writable(null);
  $$unsubscribe_selectedChannel = subscribe(selectedChannel, (value) => $selectedChannel = value);
  let msgContent = "";
  const priorities = ["Flash", "Immediate", "Priority", "Routine"];
  activeChannel = $selectedChannel;
  $$unsubscribe_selectedChannel();
  $$unsubscribe_channels();
  $$unsubscribe_messages();
  return `${$$result.head += `<!-- HEAD_svelte-g84czu_START -->${$$result.title = `<title>Emergency Comms | Mycelix Observatory</title>`, ""}<!-- HEAD_svelte-g84czu_END -->`, ""} <div class="text-white"><header class="bg-gray-800/50 border-b border-gray-700 px-4 py-2"><div class="container mx-auto flex justify-between items-center"><div class="flex items-center gap-2" data-svelte-h="svelte-13uy3fo"><span class="text-xl">🚨</span> <div><h1 class="text-lg font-bold">Emergency Communications</h1> <p class="text-xs text-gray-400">Priority messaging for community resilience</p></div></div> <div class="flex items-center gap-3"><div class="text-right"><p class="text-xs text-gray-400" data-svelte-h="svelte-1k8k2ko">Channels</p> <p class="text-lg font-bold">${escape($channels.length)}</p></div></div></div></header> <main class="container mx-auto p-6"><div class="grid grid-cols-1 lg:grid-cols-4 gap-6" style="min-height: 60vh;"> <div class="bg-gray-800 rounded-lg border border-gray-700"><div class="p-4 border-b border-gray-700 flex justify-between items-center"><h2 class="text-sm font-semibold" data-svelte-h="svelte-jx1m12">Channels</h2> <button class="text-xs px-2 py-1 bg-gray-700 hover:bg-gray-600 rounded transition-colors" data-svelte-h="svelte-pokttj">+ New</button></div> ${``} <div class="p-2 space-y-1 max-h-96 overflow-y-auto">${$channels.length ? each($channels, (ch) => {
    return `<button class="${"w-full text-left p-3 rounded-lg transition-colors " + escape(
      activeChannel?.id === ch.id ? "bg-blue-600/30 border border-blue-500/50" : "bg-gray-700/50 hover:bg-gray-700",
      true
    )}"><p class="font-medium text-sm">${escape(ch.name)}</p> <p class="text-xs text-gray-400 mt-0.5">${escape(ch.description)}</p> <p class="text-xs text-gray-500 mt-1">${escape(ch.member_count)} members</p> </button>`;
  }) : `<p class="text-gray-500 text-center py-4 text-sm" data-svelte-h="svelte-1sjv8lu">No channels</p>`}</div></div>  <div class="bg-gray-800 rounded-lg border border-gray-700 lg:col-span-3 flex flex-col">${activeChannel ? `<div class="p-4 border-b border-gray-700"><h2 class="text-lg font-semibold">${escape(activeChannel.name)}</h2> <p class="text-xs text-gray-400">${escape(activeChannel.description)}</p></div>  <div class="flex-1 p-4 space-y-3 overflow-y-auto max-h-96">${$messages.length ? each($messages, (msg) => {
    return `<div class="${"p-3 bg-gray-700/50 rounded-lg border-l-4 " + escape(priorityBorder(msg.priority), true)}"><div class="flex justify-between items-start"><div class="flex items-center gap-2"><span class="font-medium text-sm">${escape(msg.sender_did)}</span> <span${add_attribute("class", `text-xs px-2 py-0.5 rounded ${priorityColor(msg.priority)}`, 0)}>${escape(msg.priority)} </span></div> <div class="flex items-center gap-2 text-xs text-gray-400"><span>${escape(formatTime(msg.sent_at))}</span> ${msg.synced ? `<span class="text-green-400" title="Synced" data-svelte-h="svelte-19cffw3">✓</span>` : `<span class="text-yellow-400 animate-pulse" title="Pending sync" data-svelte-h="svelte-1awr0zo">●</span>`} </div></div> <p class="text-sm mt-2">${escape(msg.content)}</p> </div>`;
  }) : `<p class="text-gray-500 text-center py-8" data-svelte-h="svelte-g4aan2">No messages in this channel</p>`}</div>  <div class="p-4 border-t border-gray-700"><form class="flex gap-2"><select class="bg-gray-700 border border-gray-600 rounded px-2 py-2 text-sm focus:outline-none focus:border-blue-500 w-32">${each(priorities, (p) => {
    return `<option${add_attribute("value", p, 0)}>${escape(p)}</option>`;
  })}</select> <input placeholder="Type message..." class="flex-1 bg-gray-700 border border-gray-600 rounded px-3 py-2 text-sm focus:outline-none focus:border-blue-500"${add_attribute("value", msgContent, 0)}> <button type="submit" ${!msgContent.trim() ? "disabled" : ""} class="bg-blue-600 hover:bg-blue-700 disabled:bg-gray-600 disabled:cursor-not-allowed rounded px-4 py-2 text-sm font-medium transition-colors">Send</button></form></div>` : `<div class="flex-1 flex items-center justify-center text-gray-500" data-svelte-h="svelte-u6e40r"><div class="text-center"><p class="text-4xl mb-2">📡</p> <p>Select a channel to view messages</p></div></div>`}</div></div>  <div class="mt-6 grid grid-cols-2 md:grid-cols-4 gap-4" data-svelte-h="svelte-7ewa7n"><div class="bg-red-900/30 border border-red-700 rounded-lg p-3"><h3 class="text-sm font-bold text-red-400">FLASH</h3> <p class="text-xs text-gray-400 mt-1">Life-threatening. Immediate action required by all members.</p></div> <div class="bg-orange-900/30 border border-orange-700 rounded-lg p-3"><h3 class="text-sm font-bold text-orange-400">IMMEDIATE</h3> <p class="text-xs text-gray-400 mt-1">Urgent situation. Response needed within minutes.</p></div> <div class="bg-yellow-900/30 border border-yellow-700 rounded-lg p-3"><h3 class="text-sm font-bold text-yellow-400">PRIORITY</h3> <p class="text-xs text-gray-400 mt-1">Important update. Read when able, respond within hours.</p></div> <div class="bg-gray-800 border border-gray-700 rounded-lg p-3"><h3 class="text-sm font-bold text-gray-300">ROUTINE</h3> <p class="text-xs text-gray-400 mt-1">General information. No time pressure.</p></div></div> <footer class="mt-8 text-center text-gray-500 text-sm" data-svelte-h="svelte-100dp65"><p>Emergency Communications · Mycelix Civic</p></footer></main></div>`;
});
export {
  Page as default
};
