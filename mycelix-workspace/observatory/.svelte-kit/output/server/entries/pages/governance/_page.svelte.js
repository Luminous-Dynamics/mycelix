import { c as create_ssr_component, a as subscribe, o as onDestroy, b as escape, e as each, d as add_attribute } from "../../../chunks/ssr.js";
import { w as writable } from "../../../chunks/index.js";
function getStatusBadge(status) {
  switch (status) {
    case "active":
      return "bg-blue-500/20 text-blue-400 border-blue-500/50";
    case "passed":
    case "executed":
      return "bg-green-500/20 text-green-400 border-green-500/50";
    case "rejected":
      return "bg-red-500/20 text-red-400 border-red-500/50";
    case "pending":
      return "bg-yellow-500/20 text-yellow-400 border-yellow-500/50";
    default:
      return "bg-gray-500/20 text-gray-400 border-gray-500/50";
  }
}
function getVoteColor(vote) {
  switch (vote) {
    case "for":
      return "text-green-400";
    case "against":
      return "text-red-400";
    case "abstain":
      return "text-yellow-400";
    default:
      return "text-gray-400";
  }
}
function daysRemaining(ts) {
  const days = Math.ceil((ts - Date.now()) / 864e5);
  if (days < 0) return "Ended";
  if (days === 0) return "Ends today";
  return `${days} days left`;
}
function getVotePercentage(proposal) {
  const total = proposal.votesFor + proposal.votesAgainst + proposal.abstentions;
  if (total === 0) return 0;
  return proposal.votesFor / total * 100;
}
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let $stats, $$unsubscribe_stats;
  let $proposals, $$unsubscribe_proposals;
  let $recentVotes, $$unsubscribe_recentVotes;
  let $councils, $$unsubscribe_councils;
  const stats = writable({
    totalProposals: 156,
    activeProposals: 8,
    passedProposals: 112,
    totalVoters: 2341,
    participationRate: 0.67,
    avgTurnout: 0.54
  });
  $$unsubscribe_stats = subscribe(stats, (value) => $stats = value);
  const proposals = writable([
    {
      id: "prop-001",
      title: "Increase Byzantine Tolerance Threshold",
      description: "Proposal to research advanced BFT mechanisms targeting 40% threshold (current validated max: 34%)",
      proposer: "alice.dao",
      status: "active",
      category: "Protocol",
      votesFor: 1234,
      votesAgainst: 456,
      abstentions: 89,
      quorum: 1500,
      quorumReached: true,
      createdAt: Date.now() - 864e5 * 5,
      endsAt: Date.now() + 864e5 * 2,
      epistemicWeight: 0.85
    },
    {
      id: "prop-002",
      title: "Community Treasury Allocation Q1 2026",
      description: "Allocate 50,000 SAP to development initiatives",
      proposer: "treasury.council",
      status: "active",
      category: "Treasury",
      votesFor: 892,
      votesAgainst: 234,
      abstentions: 156,
      quorum: 1200,
      quorumReached: true,
      createdAt: Date.now() - 864e5 * 3,
      endsAt: Date.now() + 864e5 * 4,
      epistemicWeight: 0.78
    },
    {
      id: "prop-003",
      title: "Add New Supply Chain Verification Zome",
      description: "Integrate advanced provenance tracking for material passports",
      proposer: "dev.collective",
      status: "pending",
      category: "Technical",
      votesFor: 0,
      votesAgainst: 0,
      abstentions: 0,
      quorum: 1e3,
      quorumReached: false,
      createdAt: Date.now() - 864e5 * 1,
      endsAt: Date.now() + 864e5 * 6,
      epistemicWeight: 0.92
    },
    {
      id: "prop-004",
      title: "Marketplace Fee Reduction",
      description: "Reduce transaction fees from 2% to 1.5%",
      proposer: "merchant.guild",
      status: "passed",
      category: "Economic",
      votesFor: 1567,
      votesAgainst: 423,
      abstentions: 210,
      quorum: 1500,
      quorumReached: true,
      createdAt: Date.now() - 864e5 * 14,
      endsAt: Date.now() - 864e5 * 7,
      epistemicWeight: 0.72
    }
  ]);
  $$unsubscribe_proposals = subscribe(proposals, (value) => $proposals = value);
  const councils = writable([
    {
      id: "council-001",
      name: "Technical Council",
      members: 12,
      activeProposals: 3,
      domain: "Protocol & Code",
      trustThreshold: 0.85
    },
    {
      id: "council-002",
      name: "Treasury Council",
      members: 7,
      activeProposals: 2,
      domain: "Finance & Allocation",
      trustThreshold: 0.9
    },
    {
      id: "council-003",
      name: "Community Council",
      members: 15,
      activeProposals: 1,
      domain: "Governance & Policy",
      trustThreshold: 0.75
    },
    {
      id: "council-004",
      name: "Ethics Council",
      members: 5,
      activeProposals: 2,
      domain: "Disputes & Appeals",
      trustThreshold: 0.95
    }
  ]);
  $$unsubscribe_councils = subscribe(councils, (value) => $councils = value);
  const recentVotes = writable([
    {
      id: "vote-001",
      proposalTitle: "Byzantine Tolerance",
      voter: "alice.dao",
      vote: "for",
      weight: 1.2,
      timestamp: Date.now() - 6e4
    },
    {
      id: "vote-002",
      proposalTitle: "Treasury Allocation",
      voter: "bob.coop",
      vote: "for",
      weight: 0.8,
      timestamp: Date.now() - 18e4
    },
    {
      id: "vote-003",
      proposalTitle: "Byzantine Tolerance",
      voter: "carol.node",
      vote: "against",
      weight: 1.5,
      timestamp: Date.now() - 3e5
    },
    {
      id: "vote-004",
      proposalTitle: "Treasury Allocation",
      voter: "dave.maker",
      vote: "abstain",
      weight: 0.9,
      timestamp: Date.now() - 42e4
    }
  ]);
  $$unsubscribe_recentVotes = subscribe(recentVotes, (value) => $recentVotes = value);
  let currentTime = /* @__PURE__ */ (/* @__PURE__ */ new Date()).toLocaleTimeString();
  onDestroy(() => {
  });
  $$unsubscribe_stats();
  $$unsubscribe_proposals();
  $$unsubscribe_recentVotes();
  $$unsubscribe_councils();
  return `${$$result.head += `<!-- HEAD_svelte-1ny2wf8_START -->${$$result.title = `<title>Governance | Mycelix Observatory</title>`, ""}<!-- HEAD_svelte-1ny2wf8_END -->`, ""} <div class="text-white"> <header class="bg-gray-800/50 border-b border-gray-700 px-4 py-2"><div class="container mx-auto flex justify-between items-center"><div class="flex items-center gap-2" data-svelte-h="svelte-18hws56"><span class="text-xl">🏛️</span> <div><h1 class="text-lg font-bold">Governance Commons</h1> <p class="text-xs text-gray-400">Democratic Decision-Making</p></div></div> <div class="flex items-center gap-4"><div class="text-right"><p class="text-xs text-gray-400" data-svelte-h="svelte-15b5eff">Participation Rate</p> <p class="text-lg font-bold text-green-400">${escape(($stats.participationRate * 100).toFixed(0))}%</p></div> <span class="text-gray-400 font-mono text-sm">${escape(currentTime)}</span></div></div></header> <main class="container mx-auto p-6"> <div class="grid grid-cols-2 md:grid-cols-6 gap-4 mb-8"><div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-1gdz5qx">Total Proposals</h3> <p class="text-2xl font-bold mt-1">${escape($stats.totalProposals)}</p></div> <div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-1q8v4r">Active Votes</h3> <p class="text-2xl font-bold mt-1 text-blue-400">${escape($stats.activeProposals)}</p></div> <div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-i7jeaw">Passed</h3> <p class="text-2xl font-bold mt-1 text-green-400">${escape($stats.passedProposals)}</p></div> <div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-yytpt5">Total Voters</h3> <p class="text-2xl font-bold mt-1">${escape($stats.totalVoters.toLocaleString())}</p></div> <div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-1bx150l">Participation</h3> <p class="text-2xl font-bold mt-1 text-purple-400">${escape(($stats.participationRate * 100).toFixed(0))}%</p></div> <div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-1nbzu7t">Avg Turnout</h3> <p class="text-2xl font-bold mt-1 text-cyan-400">${escape(($stats.avgTurnout * 100).toFixed(0))}%</p></div></div>  <div class="grid grid-cols-1 lg:grid-cols-3 gap-6"> <div class="bg-gray-800 rounded-lg border border-gray-700 lg:col-span-2"><div class="p-4 border-b border-gray-700 flex justify-between items-center"><h2 class="text-lg font-semibold flex items-center gap-2" data-svelte-h="svelte-jtsn38"><span>📜</span> Proposals</h2> <span class="text-sm text-gray-400">${escape($proposals.filter((p) => p.status === "active").length)} active</span></div> <div class="p-4 space-y-4">${each($proposals, (proposal) => {
    return `<div class="p-4 bg-gray-700/50 rounded-lg hover:bg-gray-700/70 transition-colors"><div class="flex justify-between items-start"><div><div class="flex items-center gap-2"><p class="font-medium">${escape(proposal.title)}</p> <span${add_attribute("class", `text-xs px-2 py-0.5 rounded border ${getStatusBadge(proposal.status)}`, 0)}>${escape(proposal.status)}</span> <span class="text-xs px-2 py-0.5 rounded bg-gray-600 text-gray-300">${escape(proposal.category)} </span></div> <p class="text-xs text-gray-400 mt-1">${escape(proposal.description)}</p></div> <div class="text-right text-sm"><p class="text-gray-400">${escape(daysRemaining(proposal.endsAt))}</p> </div></div>  <div class="mt-4"><div class="flex justify-between text-xs mb-1"><span class="text-green-400">For: ${escape(proposal.votesFor)}</span> <span class="text-red-400">Against: ${escape(proposal.votesAgainst)}</span> <span class="text-yellow-400">Abstain: ${escape(proposal.abstentions)}</span></div> <div class="w-full bg-gray-600 rounded-full h-2 flex overflow-hidden"><div class="bg-green-500 h-2" style="${"width: " + escape(getVotePercentage(proposal), true) + "%"}"></div> <div class="bg-red-500 h-2" style="${"width: " + escape(proposal.votesAgainst / (proposal.votesFor + proposal.votesAgainst + proposal.abstentions || 1) * 100, true) + "%"}"></div></div> <div class="flex justify-between text-xs mt-1"><span class="text-gray-400">Quorum: ${escape(proposal.quorumReached ? "✓" : `${proposal.votesFor + proposal.votesAgainst + proposal.abstentions}/${proposal.quorum}`)}</span> <span class="text-purple-400">Epistemic Weight: ${escape((proposal.epistemicWeight * 100).toFixed(0))}%</span> </div></div> </div>`;
  })}</div></div>  <div class="space-y-6"> <div class="bg-gray-800 rounded-lg border border-gray-700"><div class="p-4 border-b border-gray-700" data-svelte-h="svelte-1nbm3hz"><h2 class="text-lg font-semibold flex items-center gap-2"><span>🗳️</span> Recent Votes</h2></div> <div class="p-4 space-y-2 max-h-64 overflow-y-auto">${each($recentVotes, (vote) => {
    return `<div class="p-2 bg-gray-700/50 rounded-lg text-sm"><div class="flex justify-between items-center"><span class="font-medium">${escape(vote.voter)}</span> <span${add_attribute("class", `font-bold ${getVoteColor(vote.vote)}`, 0)}>${escape(vote.vote.toUpperCase())}</span></div> <div class="flex justify-between text-xs text-gray-400 mt-1"><span>${escape(vote.proposalTitle)}</span> <span>Weight: ${escape(vote.weight)}x</span></div> </div>`;
  })}</div></div>  <div class="bg-gray-800 rounded-lg border border-gray-700"><div class="p-4 border-b border-gray-700" data-svelte-h="svelte-lbq2dm"><h2 class="text-lg font-semibold flex items-center gap-2"><span>👥</span> Councils</h2></div> <div class="p-4 space-y-2">${each($councils, (council) => {
    return `<div class="p-3 bg-gray-700/50 rounded-lg"><div class="flex justify-between items-start"><div><p class="font-medium text-sm">${escape(council.name)}</p> <p class="text-xs text-gray-400">${escape(council.domain)}</p></div> <span class="text-xs text-blue-400">${escape(council.activeProposals)} active</span></div> <div class="flex justify-between text-xs mt-2"><span class="text-gray-400">${escape(council.members)} members</span> <span class="text-green-400">Min trust: ${escape((council.trustThreshold * 100).toFixed(0))}%</span></div> </div>`;
  })}</div></div></div></div>  <div class="mt-6 grid grid-cols-1 md:grid-cols-3 gap-6" data-svelte-h="svelte-j46tt7"><div class="bg-gray-800 rounded-lg border border-gray-700 p-4"><h3 class="text-lg font-semibold mb-3 flex items-center gap-2"><span>⚖️</span> Voting Power</h3> <div class="bg-gray-900 rounded p-3 font-mono text-xs"><p class="text-green-400">VP = SAP × Patience × Trust</p></div> <div class="mt-3 space-y-2 text-sm"><div class="flex justify-between"><span class="text-gray-400">Base</span> <span>SAP Balance</span></div> <div class="flex justify-between"><span class="text-gray-400">Patience Mult</span> <span>1.0 - 3.0x</span></div> <div class="flex justify-between"><span class="text-gray-400">Trust Mult</span> <span>MATL Score</span></div></div></div> <div class="bg-gray-800 rounded-lg border border-gray-700 p-4"><h3 class="text-lg font-semibold mb-3 flex items-center gap-2"><span>🎯</span> Quorum Rules</h3> <div class="space-y-2 text-sm"><div class="flex justify-between"><span class="text-gray-400">Protocol Changes</span> <span class="text-yellow-400">67%</span></div> <div class="flex justify-between"><span class="text-gray-400">Treasury</span> <span class="text-yellow-400">60%</span></div> <div class="flex justify-between"><span class="text-gray-400">Community</span> <span class="text-green-400">50%</span></div> <div class="flex justify-between"><span class="text-gray-400">Emergency</span> <span class="text-red-400">75%</span></div></div></div> <div class="bg-gray-800 rounded-lg border border-gray-700 p-4"><h3 class="text-lg font-semibold mb-3 flex items-center gap-2"><span>🛡️</span> Constitution</h3> <div class="space-y-2 text-sm"><div class="flex justify-between"><span class="text-gray-400">Version</span> <span>v1.2.0</span></div> <div class="flex justify-between"><span class="text-gray-400">Amendments</span> <span>12</span></div> <div class="flex justify-between"><span class="text-gray-400">Last Updated</span> <span>2026-01-15</span></div> <div class="flex justify-between"><span class="text-gray-400">Signatories</span> <span class="text-green-400">847</span></div></div></div></div>  <footer class="mt-8 text-center text-gray-500 text-sm" data-svelte-h="svelte-1m6w9al"><p>Mycelix Governance v0.1.0 • 7 Zomes • HDK 0.6.0</p> <p class="mt-1">Democratic Coordination for the Civilizational OS</p></footer></main></div>`;
});
export {
  Page as default
};
