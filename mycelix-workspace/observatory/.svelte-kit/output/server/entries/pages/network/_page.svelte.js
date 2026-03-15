import { c as create_ssr_component, a as subscribe, o as onDestroy, e as escape, b as each, n as null_to_empty } from "../../../chunks/ssr.js";
import { w as writable } from "../../../chunks/index.js";
const css = {
  code: "@keyframes svelte-15hu0lc-pulse{0%,100%{opacity:1}50%{opacity:0.5}}.animate-pulse.svelte-15hu0lc{animation:svelte-15hu0lc-pulse 2s cubic-bezier(0.4, 0, 0.6, 1) infinite}",
  map: `{"version":3,"file":"+page.svelte","sources":["+page.svelte"],"sourcesContent":["<script lang=\\"ts\\">import { onMount, onDestroy } from \\"svelte\\";\\nimport { writable } from \\"svelte/store\\";\\nconst stats = writable({\\n  totalNodes: 156,\\n  onlineNodes: 142,\\n  totalPeers: 1234,\\n  avgLatency: 45,\\n  messagesPerSecond: 2456,\\n  byzantineDetections: 3\\n});\\nconst nodes = writable([\\n  { id: \\"node-001\\", name: \\"portland-hub\\", location: \\"Portland, OR\\", status: \\"online\\", happs: [\\"core\\", \\"marketplace\\", \\"mail\\"], peers: 24, uptime: 0.999, lastSeen: Date.now(), trustScore: 0.96 },\\n  { id: \\"node-002\\", name: \\"berlin-gateway\\", location: \\"Berlin, DE\\", status: \\"online\\", happs: [\\"core\\", \\"governance\\", \\"identity\\"], peers: 31, uptime: 0.998, lastSeen: Date.now(), trustScore: 0.94 },\\n  { id: \\"node-003\\", name: \\"tokyo-relay\\", location: \\"Tokyo, JP\\", status: \\"syncing\\", happs: [\\"core\\", \\"edunet\\"], peers: 18, uptime: 0.987, lastSeen: Date.now() - 3e4, trustScore: 0.91 },\\n  { id: \\"node-004\\", name: \\"austin-maker\\", location: \\"Austin, TX\\", status: \\"online\\", happs: [\\"core\\", \\"fabrication\\", \\"supplychain\\"], peers: 22, uptime: 0.995, lastSeen: Date.now(), trustScore: 0.93 },\\n  { id: \\"node-005\\", name: \\"london-finance\\", location: \\"London, UK\\", status: \\"degraded\\", happs: [\\"core\\", \\"finance\\"], peers: 15, uptime: 0.956, lastSeen: Date.now() - 12e4, trustScore: 0.88 },\\n  { id: \\"node-006\\", name: \\"sydney-bridge\\", location: \\"Sydney, AU\\", status: \\"online\\", happs: [\\"core\\", \\"marketplace\\", \\"supplychain\\"], peers: 12, uptime: 0.992, lastSeen: Date.now(), trustScore: 0.9 }\\n]);\\nconst happHealth = writable([\\n  { name: \\"Core\\", nodes: 142, status: \\"healthy\\", latency: 23, throughput: 1456 },\\n  { name: \\"Marketplace\\", nodes: 89, status: \\"healthy\\", latency: 34, throughput: 678 },\\n  { name: \\"Identity\\", nodes: 76, status: \\"healthy\\", latency: 28, throughput: 345 },\\n  { name: \\"Mail\\", nodes: 65, status: \\"healthy\\", latency: 41, throughput: 234 },\\n  { name: \\"Governance\\", nodes: 54, status: \\"degraded\\", latency: 67, throughput: 123 },\\n  { name: \\"EduNet\\", nodes: 45, status: \\"healthy\\", latency: 52, throughput: 189 },\\n  { name: \\"Fabrication\\", nodes: 34, status: \\"healthy\\", latency: 38, throughput: 145 },\\n  { name: \\"Supply Chain\\", nodes: 28, status: \\"healthy\\", latency: 45, throughput: 98 }\\n]);\\nconst recentConnections = writable([\\n  { source: \\"portland-hub\\", target: \\"berlin-gateway\\", latency: 142, status: \\"stable\\" },\\n  { source: \\"berlin-gateway\\", target: \\"tokyo-relay\\", latency: 198, status: \\"stable\\" },\\n  { source: \\"austin-maker\\", target: \\"portland-hub\\", latency: 45, status: \\"stable\\" },\\n  { source: \\"london-finance\\", target: \\"berlin-gateway\\", latency: 89, status: \\"unstable\\" },\\n  { source: \\"sydney-bridge\\", target: \\"tokyo-relay\\", latency: 156, status: \\"new\\" }\\n]);\\nlet currentTime = (/* @__PURE__ */ new Date()).toLocaleTimeString();\\nlet interval;\\nonMount(() => {\\n  interval = setInterval(() => {\\n    currentTime = (/* @__PURE__ */ new Date()).toLocaleTimeString();\\n    simulateNetwork();\\n  }, 2e3);\\n});\\nonDestroy(() => {\\n  if (interval) clearInterval(interval);\\n});\\nfunction simulateNetwork() {\\n  stats.update((s) => ({\\n    ...s,\\n    messagesPerSecond: s.messagesPerSecond + Math.floor(Math.random() * 100) - 50,\\n    avgLatency: Math.max(20, Math.min(80, s.avgLatency + Math.floor(Math.random() * 10) - 5))\\n  }));\\n  nodes.update((n) => n.map((node) => ({\\n    ...node,\\n    lastSeen: node.status === \\"online\\" ? Date.now() : node.lastSeen,\\n    peers: node.peers + Math.floor(Math.random() * 3) - 1\\n  })));\\n}\\nfunction getStatusColor(status) {\\n  switch (status) {\\n    case \\"online\\":\\n    case \\"healthy\\":\\n    case \\"stable\\":\\n      return \\"bg-green-500\\";\\n    case \\"syncing\\":\\n    case \\"new\\":\\n      return \\"bg-blue-500\\";\\n    case \\"degraded\\":\\n    case \\"unstable\\":\\n      return \\"bg-yellow-500\\";\\n    case \\"offline\\":\\n      return \\"bg-red-500\\";\\n    default:\\n      return \\"bg-gray-500\\";\\n  }\\n}\\nfunction getStatusBadge(status) {\\n  switch (status) {\\n    case \\"online\\":\\n    case \\"healthy\\":\\n    case \\"stable\\":\\n      return \\"bg-green-500/20 text-green-400 border-green-500/50\\";\\n    case \\"syncing\\":\\n    case \\"new\\":\\n      return \\"bg-blue-500/20 text-blue-400 border-blue-500/50\\";\\n    case \\"degraded\\":\\n    case \\"unstable\\":\\n      return \\"bg-yellow-500/20 text-yellow-400 border-yellow-500/50\\";\\n    case \\"offline\\":\\n      return \\"bg-red-500/20 text-red-400 border-red-500/50\\";\\n    default:\\n      return \\"bg-gray-500/20 text-gray-400 border-gray-500/50\\";\\n  }\\n}\\nfunction formatUptime(uptime) {\\n  return (uptime * 100).toFixed(2) + \\"%\\";\\n}\\nfunction formatLatency(ms) {\\n  return ms + \\"ms\\";\\n}\\nfunction formatThroughput(ops) {\\n  if (ops >= 1e3) return (ops / 1e3).toFixed(1) + \\"K/s\\";\\n  return ops + \\"/s\\";\\n}\\nfunction timeSince(ts) {\\n  const seconds = Math.floor((Date.now() - ts) / 1e3);\\n  if (seconds < 60) return \\"just now\\";\\n  if (seconds < 3600) return Math.floor(seconds / 60) + \\"m ago\\";\\n  return Math.floor(seconds / 3600) + \\"h ago\\";\\n}\\n<\/script>\\n\\n<svelte:head>\\n  <title>Network | Mycelix Observatory</title>\\n</svelte:head>\\n\\n<div class=\\"text-white\\">\\n  <!-- Page Header -->\\n  <header class=\\"bg-gray-800/50 border-b border-gray-700 px-4 py-2\\">\\n    <div class=\\"container mx-auto flex justify-between items-center\\">\\n      <div class=\\"flex items-center gap-2\\">\\n        <span class=\\"text-xl\\">🌐</span>\\n        <div>\\n          <h1 class=\\"text-lg font-bold\\">Network Health</h1>\\n          <p class=\\"text-xs text-gray-400\\">Node Connectivity & DHT Status</p>\\n        </div>\\n      </div>\\n      <div class=\\"flex items-center gap-4\\">\\n        <div class=\\"text-right\\">\\n          <p class=\\"text-xs text-gray-400\\">Avg Latency</p>\\n          <p class=\\"text-lg font-bold text-green-400\\">{$stats.avgLatency}ms</p>\\n        </div>\\n        <span class=\\"text-gray-400 font-mono text-sm\\">{currentTime}</span>\\n      </div>\\n    </div>\\n  </header>\\n\\n  <main class=\\"container mx-auto p-6\\">\\n    <!-- Stats Grid -->\\n    <div class=\\"grid grid-cols-2 md:grid-cols-6 gap-4 mb-8\\">\\n      <div class=\\"bg-gray-800 rounded-lg p-4 border border-gray-700\\">\\n        <h3 class=\\"text-gray-400 text-xs uppercase\\">Total Nodes</h3>\\n        <p class=\\"text-2xl font-bold mt-1\\">{$stats.totalNodes}</p>\\n      </div>\\n      <div class=\\"bg-gray-800 rounded-lg p-4 border border-gray-700\\">\\n        <h3 class=\\"text-gray-400 text-xs uppercase\\">Online</h3>\\n        <p class=\\"text-2xl font-bold mt-1 text-green-400\\">{$stats.onlineNodes}</p>\\n      </div>\\n      <div class=\\"bg-gray-800 rounded-lg p-4 border border-gray-700\\">\\n        <h3 class=\\"text-gray-400 text-xs uppercase\\">Peer Connections</h3>\\n        <p class=\\"text-2xl font-bold mt-1\\">{$stats.totalPeers.toLocaleString()}</p>\\n      </div>\\n      <div class=\\"bg-gray-800 rounded-lg p-4 border border-gray-700\\">\\n        <h3 class=\\"text-gray-400 text-xs uppercase\\">Avg Latency</h3>\\n        <p class=\\"text-2xl font-bold mt-1 text-blue-400\\">{$stats.avgLatency}ms</p>\\n      </div>\\n      <div class=\\"bg-gray-800 rounded-lg p-4 border border-gray-700\\">\\n        <h3 class=\\"text-gray-400 text-xs uppercase\\">Messages/sec</h3>\\n        <p class=\\"text-2xl font-bold mt-1 text-purple-400\\">{formatThroughput($stats.messagesPerSecond)}</p>\\n      </div>\\n      <div class=\\"bg-gray-800 rounded-lg p-4 border border-gray-700\\">\\n        <h3 class=\\"text-gray-400 text-xs uppercase\\">Byzantine Alerts</h3>\\n        <p class=\\"text-2xl font-bold mt-1 {$stats.byzantineDetections > 0 ? 'text-yellow-400' : 'text-green-400'}\\">\\n          {$stats.byzantineDetections}\\n        </p>\\n      </div>\\n    </div>\\n\\n    <!-- Main Grid -->\\n    <div class=\\"grid grid-cols-1 lg:grid-cols-3 gap-6\\">\\n      <!-- Node List -->\\n      <div class=\\"bg-gray-800 rounded-lg border border-gray-700 lg:col-span-2\\">\\n        <div class=\\"p-4 border-b border-gray-700 flex justify-between items-center\\">\\n          <h2 class=\\"text-lg font-semibold flex items-center gap-2\\">\\n            <span>💻</span> Active Nodes\\n          </h2>\\n          <span class=\\"text-sm text-gray-400\\">{$nodes.filter(n => n.status === 'online').length}/{$nodes.length} online</span>\\n        </div>\\n        <div class=\\"p-4 space-y-3\\">\\n          {#each $nodes as node}\\n            <div class=\\"p-4 bg-gray-700/50 rounded-lg hover:bg-gray-700/70 transition-colors\\">\\n              <div class=\\"flex justify-between items-start\\">\\n                <div>\\n                  <div class=\\"flex items-center gap-2\\">\\n                    <span class={\`w-2 h-2 rounded-full \${getStatusColor(node.status)} \${node.status === 'online' ? 'animate-pulse' : ''}\`}></span>\\n                    <p class=\\"font-medium\\">{node.name}</p>\\n                    <span class={\`text-xs px-2 py-0.5 rounded border \${getStatusBadge(node.status)}\`}>\\n                      {node.status}\\n                    </span>\\n                  </div>\\n                  <p class=\\"text-xs text-gray-400 mt-1\\">{node.location}</p>\\n                </div>\\n                <div class=\\"text-right\\">\\n                  <p class=\\"text-sm text-green-400\\">{(node.trustScore * 100).toFixed(0)}% trust</p>\\n                  <p class=\\"text-xs text-gray-400\\">{timeSince(node.lastSeen)}</p>\\n                </div>\\n              </div>\\n              <div class=\\"mt-3 grid grid-cols-4 gap-4 text-xs\\">\\n                <div>\\n                  <p class=\\"text-gray-400\\">Peers</p>\\n                  <p class=\\"font-medium\\">{node.peers}</p>\\n                </div>\\n                <div>\\n                  <p class=\\"text-gray-400\\">Uptime</p>\\n                  <p class=\\"font-medium text-green-400\\">{formatUptime(node.uptime)}</p>\\n                </div>\\n                <div>\\n                  <p class=\\"text-gray-400\\">hApps</p>\\n                  <p class=\\"font-medium\\">{node.happs.length}</p>\\n                </div>\\n                <div>\\n                  <p class=\\"text-gray-400\\">Running</p>\\n                  <p class=\\"font-medium text-blue-400\\">{node.happs.slice(0, 2).join(', ')}{node.happs.length > 2 ? '...' : ''}</p>\\n                </div>\\n              </div>\\n            </div>\\n          {/each}\\n        </div>\\n      </div>\\n\\n      <!-- hApp Health & Connections -->\\n      <div class=\\"space-y-6\\">\\n        <!-- hApp Health -->\\n        <div class=\\"bg-gray-800 rounded-lg border border-gray-700\\">\\n          <div class=\\"p-4 border-b border-gray-700\\">\\n            <h2 class=\\"text-lg font-semibold flex items-center gap-2\\">\\n              <span>📦</span> hApp Health\\n            </h2>\\n          </div>\\n          <div class=\\"p-4 space-y-2 max-h-64 overflow-y-auto\\">\\n            {#each $happHealth as happ}\\n              <div class=\\"p-2 bg-gray-700/50 rounded-lg\\">\\n                <div class=\\"flex justify-between items-center\\">\\n                  <div class=\\"flex items-center gap-2\\">\\n                    <span class={\`w-2 h-2 rounded-full \${getStatusColor(happ.status)}\`}></span>\\n                    <span class=\\"text-sm font-medium\\">{happ.name}</span>\\n                  </div>\\n                  <span class=\\"text-xs text-gray-400\\">{happ.nodes} nodes</span>\\n                </div>\\n                <div class=\\"flex justify-between text-xs mt-1\\">\\n                  <span class=\\"text-gray-400\\">{formatLatency(happ.latency)}</span>\\n                  <span class=\\"text-blue-400\\">{formatThroughput(happ.throughput)}</span>\\n                </div>\\n              </div>\\n            {/each}\\n          </div>\\n        </div>\\n\\n        <!-- Recent Connections -->\\n        <div class=\\"bg-gray-800 rounded-lg border border-gray-700\\">\\n          <div class=\\"p-4 border-b border-gray-700\\">\\n            <h2 class=\\"text-lg font-semibold flex items-center gap-2\\">\\n              <span>🔗</span> Recent Connections\\n            </h2>\\n          </div>\\n          <div class=\\"p-4 space-y-2\\">\\n            {#each $recentConnections as conn}\\n              <div class=\\"p-2 bg-gray-700/50 rounded-lg text-sm\\">\\n                <div class=\\"flex items-center gap-2\\">\\n                  <span class=\\"text-gray-400\\">{conn.source}</span>\\n                  <span class=\\"text-gray-500\\">→</span>\\n                  <span class=\\"text-gray-400\\">{conn.target}</span>\\n                  <span class={\`ml-auto text-xs px-1.5 py-0.5 rounded \${getStatusBadge(conn.status)}\`}>\\n                    {conn.status}\\n                  </span>\\n                </div>\\n                <p class=\\"text-xs text-gray-500 mt-1\\">{formatLatency(conn.latency)}</p>\\n              </div>\\n            {/each}\\n          </div>\\n        </div>\\n      </div>\\n    </div>\\n\\n    <!-- Network Topology Info -->\\n    <div class=\\"mt-6 grid grid-cols-1 md:grid-cols-3 gap-6\\">\\n      <div class=\\"bg-gray-800 rounded-lg border border-gray-700 p-4\\">\\n        <h3 class=\\"text-lg font-semibold mb-3 flex items-center gap-2\\">\\n          <span>🌍</span> Geographic Distribution\\n        </h3>\\n        <div class=\\"space-y-2 text-sm\\">\\n          <div class=\\"flex justify-between\\">\\n            <span class=\\"text-gray-400\\">North America</span>\\n            <span>42 nodes</span>\\n          </div>\\n          <div class=\\"flex justify-between\\">\\n            <span class=\\"text-gray-400\\">Europe</span>\\n            <span>38 nodes</span>\\n          </div>\\n          <div class=\\"flex justify-between\\">\\n            <span class=\\"text-gray-400\\">Asia Pacific</span>\\n            <span>31 nodes</span>\\n          </div>\\n          <div class=\\"flex justify-between\\">\\n            <span class=\\"text-gray-400\\">Other</span>\\n            <span>31 nodes</span>\\n          </div>\\n        </div>\\n      </div>\\n\\n      <div class=\\"bg-gray-800 rounded-lg border border-gray-700 p-4\\">\\n        <h3 class=\\"text-lg font-semibold mb-3 flex items-center gap-2\\">\\n          <span>⚡</span> DHT Performance\\n        </h3>\\n        <div class=\\"space-y-2 text-sm\\">\\n          <div class=\\"flex justify-between\\">\\n            <span class=\\"text-gray-400\\">Gossip Interval</span>\\n            <span class=\\"text-green-400\\">2s</span>\\n          </div>\\n          <div class=\\"flex justify-between\\">\\n            <span class=\\"text-gray-400\\">Validation Rate</span>\\n            <span class=\\"text-green-400\\">98.7%</span>\\n          </div>\\n          <div class=\\"flex justify-between\\">\\n            <span class=\\"text-gray-400\\">Sync Lag</span>\\n            <span class=\\"text-blue-400\\">&lt;500ms</span>\\n          </div>\\n          <div class=\\"flex justify-between\\">\\n            <span class=\\"text-gray-400\\">Partition Events</span>\\n            <span class=\\"text-green-400\\">0</span>\\n          </div>\\n        </div>\\n      </div>\\n\\n      <div class=\\"bg-gray-800 rounded-lg border border-gray-700 p-4\\">\\n        <h3 class=\\"text-lg font-semibold mb-3 flex items-center gap-2\\">\\n          <span>🛡️</span> Byzantine Status\\n        </h3>\\n        <div class=\\"space-y-2 text-sm\\">\\n          <div class=\\"flex justify-between\\">\\n            <span class=\\"text-gray-400\\">BFT Threshold</span>\\n            <span class=\\"text-green-400\\">34%</span>\\n          </div>\\n          <div class=\\"flex justify-between\\">\\n            <span class=\\"text-gray-400\\">Current Tolerance</span>\\n            <span class=\\"text-green-400\\">36.2%</span>\\n          </div>\\n          <div class=\\"flex justify-between\\">\\n            <span class=\\"text-gray-400\\">Detected Bad Actors</span>\\n            <span class=\\"text-yellow-400\\">3</span>\\n          </div>\\n          <div class=\\"flex justify-between\\">\\n            <span class=\\"text-gray-400\\">Cartel Detection</span>\\n            <span class=\\"text-green-400\\">Active</span>\\n          </div>\\n        </div>\\n      </div>\\n    </div>\\n\\n    <!-- Footer -->\\n    <footer class=\\"mt-8 text-center text-gray-500 text-sm\\">\\n      <p>Mycelix Network Monitor v0.1.0 • Holochain 0.6.0</p>\\n      <p class=\\"mt-1\\">Distributed Infrastructure for the Civilizational OS</p>\\n    </footer>\\n  </main>\\n</div>\\n\\n<style>\\n  @keyframes pulse {\\n    0%, 100% { opacity: 1; }\\n    50% { opacity: 0.5; }\\n  }\\n  .animate-pulse {\\n    animation: pulse 2s cubic-bezier(0.4, 0, 0.6, 1) infinite;\\n  }\\n</style>\\n"],"names":[],"mappings":"AAsWE,WAAW,oBAAM,CACf,EAAE,CAAE,IAAK,CAAE,OAAO,CAAE,CAAG,CACvB,GAAI,CAAE,OAAO,CAAE,GAAK,CACtB,CACA,6BAAe,CACb,SAAS,CAAE,oBAAK,CAAC,EAAE,CAAC,aAAa,GAAG,CAAC,CAAC,CAAC,CAAC,CAAC,GAAG,CAAC,CAAC,CAAC,CAAC,CAAC,QACnD"}`
};
function getStatusColor(status) {
  switch (status) {
    case "online":
    case "healthy":
    case "stable":
      return "bg-green-500";
    case "syncing":
    case "new":
      return "bg-blue-500";
    case "degraded":
    case "unstable":
      return "bg-yellow-500";
    case "offline":
      return "bg-red-500";
    default:
      return "bg-gray-500";
  }
}
function getStatusBadge(status) {
  switch (status) {
    case "online":
    case "healthy":
    case "stable":
      return "bg-green-500/20 text-green-400 border-green-500/50";
    case "syncing":
    case "new":
      return "bg-blue-500/20 text-blue-400 border-blue-500/50";
    case "degraded":
    case "unstable":
      return "bg-yellow-500/20 text-yellow-400 border-yellow-500/50";
    case "offline":
      return "bg-red-500/20 text-red-400 border-red-500/50";
    default:
      return "bg-gray-500/20 text-gray-400 border-gray-500/50";
  }
}
function formatUptime(uptime) {
  return (uptime * 100).toFixed(2) + "%";
}
function formatLatency(ms) {
  return ms + "ms";
}
function formatThroughput(ops) {
  if (ops >= 1e3) return (ops / 1e3).toFixed(1) + "K/s";
  return ops + "/s";
}
function timeSince(ts) {
  const seconds = Math.floor((Date.now() - ts) / 1e3);
  if (seconds < 60) return "just now";
  if (seconds < 3600) return Math.floor(seconds / 60) + "m ago";
  return Math.floor(seconds / 3600) + "h ago";
}
const Page = create_ssr_component(($$result, $$props, $$bindings, slots) => {
  let $stats, $$unsubscribe_stats;
  let $nodes, $$unsubscribe_nodes;
  let $happHealth, $$unsubscribe_happHealth;
  let $recentConnections, $$unsubscribe_recentConnections;
  const stats = writable({
    totalNodes: 156,
    onlineNodes: 142,
    totalPeers: 1234,
    avgLatency: 45,
    messagesPerSecond: 2456,
    byzantineDetections: 3
  });
  $$unsubscribe_stats = subscribe(stats, (value) => $stats = value);
  const nodes = writable([
    {
      id: "node-001",
      name: "portland-hub",
      location: "Portland, OR",
      status: "online",
      happs: ["core", "marketplace", "mail"],
      peers: 24,
      uptime: 0.999,
      lastSeen: Date.now(),
      trustScore: 0.96
    },
    {
      id: "node-002",
      name: "berlin-gateway",
      location: "Berlin, DE",
      status: "online",
      happs: ["core", "governance", "identity"],
      peers: 31,
      uptime: 0.998,
      lastSeen: Date.now(),
      trustScore: 0.94
    },
    {
      id: "node-003",
      name: "tokyo-relay",
      location: "Tokyo, JP",
      status: "syncing",
      happs: ["core", "edunet"],
      peers: 18,
      uptime: 0.987,
      lastSeen: Date.now() - 3e4,
      trustScore: 0.91
    },
    {
      id: "node-004",
      name: "austin-maker",
      location: "Austin, TX",
      status: "online",
      happs: ["core", "fabrication", "supplychain"],
      peers: 22,
      uptime: 0.995,
      lastSeen: Date.now(),
      trustScore: 0.93
    },
    {
      id: "node-005",
      name: "london-finance",
      location: "London, UK",
      status: "degraded",
      happs: ["core", "finance"],
      peers: 15,
      uptime: 0.956,
      lastSeen: Date.now() - 12e4,
      trustScore: 0.88
    },
    {
      id: "node-006",
      name: "sydney-bridge",
      location: "Sydney, AU",
      status: "online",
      happs: ["core", "marketplace", "supplychain"],
      peers: 12,
      uptime: 0.992,
      lastSeen: Date.now(),
      trustScore: 0.9
    }
  ]);
  $$unsubscribe_nodes = subscribe(nodes, (value) => $nodes = value);
  const happHealth = writable([
    {
      name: "Core",
      nodes: 142,
      status: "healthy",
      latency: 23,
      throughput: 1456
    },
    {
      name: "Marketplace",
      nodes: 89,
      status: "healthy",
      latency: 34,
      throughput: 678
    },
    {
      name: "Identity",
      nodes: 76,
      status: "healthy",
      latency: 28,
      throughput: 345
    },
    {
      name: "Mail",
      nodes: 65,
      status: "healthy",
      latency: 41,
      throughput: 234
    },
    {
      name: "Governance",
      nodes: 54,
      status: "degraded",
      latency: 67,
      throughput: 123
    },
    {
      name: "EduNet",
      nodes: 45,
      status: "healthy",
      latency: 52,
      throughput: 189
    },
    {
      name: "Fabrication",
      nodes: 34,
      status: "healthy",
      latency: 38,
      throughput: 145
    },
    {
      name: "Supply Chain",
      nodes: 28,
      status: "healthy",
      latency: 45,
      throughput: 98
    }
  ]);
  $$unsubscribe_happHealth = subscribe(happHealth, (value) => $happHealth = value);
  const recentConnections = writable([
    {
      source: "portland-hub",
      target: "berlin-gateway",
      latency: 142,
      status: "stable"
    },
    {
      source: "berlin-gateway",
      target: "tokyo-relay",
      latency: 198,
      status: "stable"
    },
    {
      source: "austin-maker",
      target: "portland-hub",
      latency: 45,
      status: "stable"
    },
    {
      source: "london-finance",
      target: "berlin-gateway",
      latency: 89,
      status: "unstable"
    },
    {
      source: "sydney-bridge",
      target: "tokyo-relay",
      latency: 156,
      status: "new"
    }
  ]);
  $$unsubscribe_recentConnections = subscribe(recentConnections, (value) => $recentConnections = value);
  let currentTime = /* @__PURE__ */ (/* @__PURE__ */ new Date()).toLocaleTimeString();
  onDestroy(() => {
  });
  $$result.css.add(css);
  $$unsubscribe_stats();
  $$unsubscribe_nodes();
  $$unsubscribe_happHealth();
  $$unsubscribe_recentConnections();
  return `${$$result.head += `<!-- HEAD_svelte-149kig4_START -->${$$result.title = `<title>Network | Mycelix Observatory</title>`, ""}<!-- HEAD_svelte-149kig4_END -->`, ""} <div class="text-white"> <header class="bg-gray-800/50 border-b border-gray-700 px-4 py-2"><div class="container mx-auto flex justify-between items-center"><div class="flex items-center gap-2" data-svelte-h="svelte-1sp6jci"><span class="text-xl">🌐</span> <div><h1 class="text-lg font-bold">Network Health</h1> <p class="text-xs text-gray-400">Node Connectivity &amp; DHT Status</p></div></div> <div class="flex items-center gap-4"><div class="text-right"><p class="text-xs text-gray-400" data-svelte-h="svelte-uzy8tu">Avg Latency</p> <p class="text-lg font-bold text-green-400">${escape($stats.avgLatency)}ms</p></div> <span class="text-gray-400 font-mono text-sm">${escape(currentTime)}</span></div></div></header> <main class="container mx-auto p-6"> <div class="grid grid-cols-2 md:grid-cols-6 gap-4 mb-8"><div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-1g67h07">Total Nodes</h3> <p class="text-2xl font-bold mt-1">${escape($stats.totalNodes)}</p></div> <div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-1ntruyj">Online</h3> <p class="text-2xl font-bold mt-1 text-green-400">${escape($stats.onlineNodes)}</p></div> <div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-5hke5p">Peer Connections</h3> <p class="text-2xl font-bold mt-1">${escape($stats.totalPeers.toLocaleString())}</p></div> <div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-8vsex8">Avg Latency</h3> <p class="text-2xl font-bold mt-1 text-blue-400">${escape($stats.avgLatency)}ms</p></div> <div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-1bf6j36">Messages/sec</h3> <p class="text-2xl font-bold mt-1 text-purple-400">${escape(formatThroughput($stats.messagesPerSecond))}</p></div> <div class="bg-gray-800 rounded-lg p-4 border border-gray-700"><h3 class="text-gray-400 text-xs uppercase" data-svelte-h="svelte-ljg4ab">Byzantine Alerts</h3> <p class="${"text-2xl font-bold mt-1 " + escape(
    $stats.byzantineDetections > 0 ? "text-yellow-400" : "text-green-400",
    true
  )}">${escape($stats.byzantineDetections)}</p></div></div>  <div class="grid grid-cols-1 lg:grid-cols-3 gap-6"> <div class="bg-gray-800 rounded-lg border border-gray-700 lg:col-span-2"><div class="p-4 border-b border-gray-700 flex justify-between items-center"><h2 class="text-lg font-semibold flex items-center gap-2" data-svelte-h="svelte-1sqgvfp"><span>💻</span> Active Nodes</h2> <span class="text-sm text-gray-400">${escape($nodes.filter((n) => n.status === "online").length)}/${escape($nodes.length)} online</span></div> <div class="p-4 space-y-3">${each($nodes, (node) => {
    return `<div class="p-4 bg-gray-700/50 rounded-lg hover:bg-gray-700/70 transition-colors"><div class="flex justify-between items-start"><div><div class="flex items-center gap-2"><span class="${escape(null_to_empty(`w-2 h-2 rounded-full ${getStatusColor(node.status)} ${node.status === "online" ? "animate-pulse" : ""}`), true) + " svelte-15hu0lc"}"></span> <p class="font-medium">${escape(node.name)}</p> <span class="${escape(null_to_empty(`text-xs px-2 py-0.5 rounded border ${getStatusBadge(node.status)}`), true) + " svelte-15hu0lc"}">${escape(node.status)} </span></div> <p class="text-xs text-gray-400 mt-1">${escape(node.location)}</p></div> <div class="text-right"><p class="text-sm text-green-400">${escape((node.trustScore * 100).toFixed(0))}% trust</p> <p class="text-xs text-gray-400">${escape(timeSince(node.lastSeen))}</p> </div></div> <div class="mt-3 grid grid-cols-4 gap-4 text-xs"><div><p class="text-gray-400" data-svelte-h="svelte-ifj8gw">Peers</p> <p class="font-medium">${escape(node.peers)}</p></div> <div><p class="text-gray-400" data-svelte-h="svelte-gjrb2x">Uptime</p> <p class="font-medium text-green-400">${escape(formatUptime(node.uptime))}</p></div> <div><p class="text-gray-400" data-svelte-h="svelte-1s1g8q9">hApps</p> <p class="font-medium">${escape(node.happs.length)}</p></div> <div><p class="text-gray-400" data-svelte-h="svelte-qjnbxq">Running</p> <p class="font-medium text-blue-400">${escape(node.happs.slice(0, 2).join(", "))}${escape(node.happs.length > 2 ? "..." : "")}</p> </div></div> </div>`;
  })}</div></div>  <div class="space-y-6"> <div class="bg-gray-800 rounded-lg border border-gray-700"><div class="p-4 border-b border-gray-700" data-svelte-h="svelte-nvkvi"><h2 class="text-lg font-semibold flex items-center gap-2"><span>📦</span> hApp Health</h2></div> <div class="p-4 space-y-2 max-h-64 overflow-y-auto">${each($happHealth, (happ) => {
    return `<div class="p-2 bg-gray-700/50 rounded-lg"><div class="flex justify-between items-center"><div class="flex items-center gap-2"><span class="${escape(null_to_empty(`w-2 h-2 rounded-full ${getStatusColor(happ.status)}`), true) + " svelte-15hu0lc"}"></span> <span class="text-sm font-medium">${escape(happ.name)}</span></div> <span class="text-xs text-gray-400">${escape(happ.nodes)} nodes</span></div> <div class="flex justify-between text-xs mt-1"><span class="text-gray-400">${escape(formatLatency(happ.latency))}</span> <span class="text-blue-400">${escape(formatThroughput(happ.throughput))}</span></div> </div>`;
  })}</div></div>  <div class="bg-gray-800 rounded-lg border border-gray-700"><div class="p-4 border-b border-gray-700" data-svelte-h="svelte-6lbn4e"><h2 class="text-lg font-semibold flex items-center gap-2"><span>🔗</span> Recent Connections</h2></div> <div class="p-4 space-y-2">${each($recentConnections, (conn) => {
    return `<div class="p-2 bg-gray-700/50 rounded-lg text-sm"><div class="flex items-center gap-2"><span class="text-gray-400">${escape(conn.source)}</span> <span class="text-gray-500" data-svelte-h="svelte-1jbvjas">→</span> <span class="text-gray-400">${escape(conn.target)}</span> <span class="${escape(null_to_empty(`ml-auto text-xs px-1.5 py-0.5 rounded ${getStatusBadge(conn.status)}`), true) + " svelte-15hu0lc"}">${escape(conn.status)} </span></div> <p class="text-xs text-gray-500 mt-1">${escape(formatLatency(conn.latency))}</p> </div>`;
  })}</div></div></div></div>  <div class="mt-6 grid grid-cols-1 md:grid-cols-3 gap-6" data-svelte-h="svelte-svae7x"><div class="bg-gray-800 rounded-lg border border-gray-700 p-4"><h3 class="text-lg font-semibold mb-3 flex items-center gap-2"><span>🌍</span> Geographic Distribution</h3> <div class="space-y-2 text-sm"><div class="flex justify-between"><span class="text-gray-400">North America</span> <span>42 nodes</span></div> <div class="flex justify-between"><span class="text-gray-400">Europe</span> <span>38 nodes</span></div> <div class="flex justify-between"><span class="text-gray-400">Asia Pacific</span> <span>31 nodes</span></div> <div class="flex justify-between"><span class="text-gray-400">Other</span> <span>31 nodes</span></div></div></div> <div class="bg-gray-800 rounded-lg border border-gray-700 p-4"><h3 class="text-lg font-semibold mb-3 flex items-center gap-2"><span>⚡</span> DHT Performance</h3> <div class="space-y-2 text-sm"><div class="flex justify-between"><span class="text-gray-400">Gossip Interval</span> <span class="text-green-400">2s</span></div> <div class="flex justify-between"><span class="text-gray-400">Validation Rate</span> <span class="text-green-400">98.7%</span></div> <div class="flex justify-between"><span class="text-gray-400">Sync Lag</span> <span class="text-blue-400">&lt;500ms</span></div> <div class="flex justify-between"><span class="text-gray-400">Partition Events</span> <span class="text-green-400">0</span></div></div></div> <div class="bg-gray-800 rounded-lg border border-gray-700 p-4"><h3 class="text-lg font-semibold mb-3 flex items-center gap-2"><span>🛡️</span> Byzantine Status</h3> <div class="space-y-2 text-sm"><div class="flex justify-between"><span class="text-gray-400">BFT Threshold</span> <span class="text-green-400">34%</span></div> <div class="flex justify-between"><span class="text-gray-400">Current Tolerance</span> <span class="text-green-400">36.2%</span></div> <div class="flex justify-between"><span class="text-gray-400">Detected Bad Actors</span> <span class="text-yellow-400">3</span></div> <div class="flex justify-between"><span class="text-gray-400">Cartel Detection</span> <span class="text-green-400">Active</span></div></div></div></div>  <footer class="mt-8 text-center text-gray-500 text-sm" data-svelte-h="svelte-9d83ds"><p>Mycelix Network Monitor v0.1.0 • Holochain 0.6.0</p> <p class="mt-1">Distributed Infrastructure for the Civilizational OS</p></footer></main> </div>`;
});
export {
  Page as default
};
