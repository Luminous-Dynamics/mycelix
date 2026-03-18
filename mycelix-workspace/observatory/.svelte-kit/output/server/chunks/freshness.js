import "./conductor.js";
import { g as getDefaultDao } from "./community.js";
import { w as writable } from "./index.js";
const DEFAULT_DAO = getDefaultDao();
async function getBalance(memberDid, daoDid = DEFAULT_DAO) {
  return mockBalance(memberDid, daoDid);
}
async function getDaoListings(daoDid = DEFAULT_DAO) {
  return mockListings();
}
async function getDaoRequests(daoDid = DEFAULT_DAO) {
  return mockRequests();
}
async function getOracleState() {
  return { vitality: 72, tier: "Normal", updated_at: Date.now() };
}
async function getAllPlots(daoDid = DEFAULT_DAO) {
  return mockPlots();
}
async function getCommunityInputs(limit = 50) {
  return mockResourceInputs();
}
async function getNutrientSummary() {
  return mockNutrientSummary();
}
async function getServiceOffers(daoDid = DEFAULT_DAO) {
  return mockOffers();
}
async function getServiceRequests(daoDid = DEFAULT_DAO) {
  return mockAidRequests();
}
async function getChannels() {
  return mockChannels();
}
async function getMessages(channelId) {
  return mockMessages(channelId);
}
async function getAllCareCircles() {
  return mockCareCircles();
}
async function getAvailableUnits() {
  return mockHousingUnits();
}
async function getAllInventoryItems() {
  return mockInventoryItems();
}
async function getLowStockItems() {
  return mockLowStockItems();
}
async function getAllWaterSystems() {
  return mockWaterSystems();
}
async function getActiveWaterAlerts() {
  return mockWaterAlerts();
}
async function getMyHearths() {
  return mockHearths();
}
async function getGraphStats() {
  return mockGraphStats();
}
function mockBalance(memberDid, daoDid) {
  return {
    member_did: memberDid,
    dao_did: daoDid,
    balance: 24,
    total_earned: 87,
    total_spent: 63,
    exchange_count: 34
  };
}
function mockListings() {
  return [
    { id: "ls-001", provider_did: "thandi.did", title: "Plumbing repair", description: "Basic plumbing fixes, pipe replacement", category: "Maintenance", hours_estimate: 2, dao_did: DEFAULT_DAO, created_at: Date.now() - 864e5, active: true },
    { id: "ls-002", provider_did: "sipho.did", title: "Vegetable gardening lessons", description: "Teaching companion planting and permaculture basics", category: "Education", hours_estimate: 1.5, dao_did: DEFAULT_DAO, created_at: Date.now() - 1728e5, active: true },
    { id: "ls-003", provider_did: "fatima.did", title: "Child minding (weekdays)", description: "After-school care for primary school children", category: "Childcare", hours_estimate: 3, dao_did: DEFAULT_DAO, created_at: Date.now() - 2592e5, active: true },
    { id: "ls-004", provider_did: "james.did", title: "Vehicle maintenance", description: "Oil changes, brake pads, basic diagnostics", category: "Transport", hours_estimate: 2, dao_did: DEFAULT_DAO, created_at: Date.now() - 3456e5, active: true },
    { id: "ls-005", provider_did: "noma.did", title: "Bread baking (bulk)", description: "Fresh bread, 10 loaves per batch", category: "Food", hours_estimate: 4, dao_did: DEFAULT_DAO, created_at: Date.now() - 432e6, active: true }
  ];
}
function mockRequests() {
  return [
    { id: "rq-001", requester_did: "lerato.did", title: "Roof leak repair", description: "Corrugated iron roof leaking in two places", category: "Maintenance", hours_budget: 3, urgency: "High", dao_did: DEFAULT_DAO, created_at: Date.now() - 432e5, open: true },
    { id: "rq-002", requester_did: "mandla.did", title: "Maths tutoring (Grade 10)", description: "Need help with algebra and trigonometry", category: "Education", hours_budget: 2, urgency: "Medium", dao_did: DEFAULT_DAO, created_at: Date.now() - 864e5, open: true },
    { id: "rq-003", requester_did: "busi.did", title: "Transport to clinic", description: "Weekly transport to Leratong Hospital for treatment", category: "Transport", hours_budget: 1, urgency: "High", dao_did: DEFAULT_DAO, created_at: Date.now() - 1296e5, open: true }
  ];
}
function mockPlots() {
  return [
    { id: "plot-001", owner_did: "sipho.did", name: "Sipho's backyard", location: "Florida, Roodepoort", area_sqm: 45, plot_type: "Raised beds", created_at: Date.now() - 2592e6 },
    { id: "plot-002", owner_did: "community.did", name: "Ontdekkers Park community garden", location: "Ontdekkers Rd", area_sqm: 200, plot_type: "Open field", created_at: Date.now() - 5184e6 },
    { id: "plot-003", owner_did: "fatima.did", name: "Fatima's tunnel", location: "Horison, Roodepoort", area_sqm: 30, plot_type: "Greenhouse tunnel", created_at: Date.now() - 1296e6 }
  ];
}
const NUTRIENT_LOOKUP = {
  KitchenWaste: { nitrogen_pct: 1.5, phosphorus_pct: 0.5, potassium_pct: 1 },
  GreenWaste: { nitrogen_pct: 2, phosphorus_pct: 0.3, potassium_pct: 1.5 },
  Biochar: { nitrogen_pct: 0.5, phosphorus_pct: 0.2, potassium_pct: 1 },
  Vermicompost: { nitrogen_pct: 2.5, phosphorus_pct: 1.5, potassium_pct: 1.5 }
};
function mockResourceInputs() {
  return [
    { id: "ri-001", plot_id: "plot-001", input_type: "KitchenWaste", quantity_kg: 12.5, nutrient_estimate: NUTRIENT_LOOKUP.KitchenWaste, contributor_did: "sipho.did", contributed_at: Date.now() - 6048e5, notes: "Weekly collection" },
    { id: "ri-002", plot_id: "plot-002", input_type: "Vermicompost", quantity_kg: 30, nutrient_estimate: NUTRIENT_LOOKUP.Vermicompost, contributor_did: "fatima.did", contributed_at: Date.now() - 432e6, notes: "Worm farm harvest" },
    { id: "ri-003", plot_id: null, input_type: "Biochar", quantity_kg: 8, nutrient_estimate: NUTRIENT_LOOKUP.Biochar, contributor_did: "community.did", contributed_at: Date.now() - 1728e5, notes: "Pyrolysis kiln run" },
    { id: "ri-004", plot_id: "plot-001", input_type: "GreenWaste", quantity_kg: 20, nutrient_estimate: NUTRIENT_LOOKUP.GreenWaste, contributor_did: "sipho.did", contributed_at: Date.now() - 864e5, notes: "Garden clippings" }
  ];
}
function mockNutrientSummary() {
  const inputs = mockResourceInputs();
  const byType = /* @__PURE__ */ new Map();
  let n = 0, p = 0, k = 0;
  for (const ri of inputs) {
    byType.set(ri.input_type, (byType.get(ri.input_type) ?? 0) + ri.quantity_kg);
    n += ri.quantity_kg * ri.nutrient_estimate.nitrogen_pct / 100;
    p += ri.quantity_kg * ri.nutrient_estimate.phosphorus_pct / 100;
    k += ri.quantity_kg * ri.nutrient_estimate.potassium_pct / 100;
  }
  return {
    total_kg_by_type: [...byType.entries()].sort((a, b) => b[1] - a[1]),
    total_nitrogen_kg: n,
    total_phosphorus_kg: p,
    total_potassium_kg: k,
    total_contributions: inputs.length
  };
}
function mockOffers() {
  return [
    { id: "ao-001", provider_did: "thandi.did", title: "Elder home visit", description: "Weekly check-in on elderly neighbors, medication reminders", category: "Care", hours_available: 2, recurring: true, created_at: Date.now() - 6048e5 },
    { id: "ao-002", provider_did: "james.did", title: "Lift share (Roodepoort-JHB CBD)", description: "Daily commute, can take 3 passengers", category: "Transport", hours_available: 1, recurring: true, created_at: Date.now() - 432e6 },
    { id: "ao-003", provider_did: "noma.did", title: "Bulk cooking for families in need", description: "Prepare nutritious meals for 10+ people", category: "Food", hours_available: 4, recurring: false, created_at: Date.now() - 2592e5 }
  ];
}
function mockAidRequests() {
  return [
    { id: "ar-001", requester_did: "grace.did", title: "Help with load-shedding prep", description: "Need help installing a gas stove safely", category: "Maintenance", urgency: "medium", hours_needed: 2, created_at: Date.now() - 1728e5, fulfilled: false },
    { id: "ar-002", requester_did: "peter.did", title: "Food parcel delivery", description: "Elderly neighbor needs groceries delivered weekly", category: "Food", urgency: "high", hours_needed: 1, created_at: Date.now() - 864e5, fulfilled: false }
  ];
}
function mockChannels() {
  return [
    { id: "ch-001", name: "Roodepoort General", description: "Community-wide emergency channel", created_by: "admin.did", created_at: Date.now() - 2592e6, member_count: 156 },
    { id: "ch-002", name: "Load Shedding Alerts", description: "Real-time Eskom schedule updates", created_by: "admin.did", created_at: Date.now() - 2592e6, member_count: 89 },
    { id: "ch-003", name: "Water Supply", description: "Rand Water and Joburg Water alerts", created_by: "admin.did", created_at: Date.now() - 1296e6, member_count: 67 }
  ];
}
function mockMessages(channelId) {
  if (channelId === "ch-002") {
    return [
      { id: "msg-001", sender_did: "eskom.did", channel_id: channelId, content: "Stage 4 load shedding from 16:00-20:30. Roodepoort affected areas: Florida, Horison, Constantia Kloof.", priority: "Immediate", sent_at: Date.now() - 36e5, synced: true },
      { id: "msg-002", sender_did: "admin.did", channel_id: channelId, content: "Community kitchen at Ontdekkers Park open during outage. Hot meals available.", priority: "Priority", sent_at: Date.now() - 18e5, synced: true }
    ];
  }
  return [
    { id: "msg-010", sender_did: "admin.did", channel_id: channelId, content: "Water pressure low in Florida area. Fill containers as precaution.", priority: "Priority", sent_at: Date.now() - 72e5, synced: true },
    { id: "msg-011", sender_did: "thandi.did", channel_id: channelId, content: "Confirmed: pressure restored in Horison. Florida still affected.", priority: "Routine", sent_at: Date.now() - 36e5, synced: true }
  ];
}
function mockCareCircles() {
  return [
    { id: "cc-001", name: "Sector 7 Neighbourhood Watch", description: "Community safety and mutual support for Sector 7 residents", circle_type: "Neighbourhood", location: "Sector 7, Roodepoort", max_members: 50, member_count: 34, created_at: Date.now() - 7776e6 },
    { id: "cc-002", name: "Florida Lake Gardeners", description: "Shared gardening knowledge and seed exchange around Florida Lake", circle_type: "Neighbourhood", location: "Florida, Roodepoort", max_members: 30, member_count: 18, created_at: Date.now() - 5184e6 },
    { id: "cc-003", name: "Roodepoort First Responders", description: "Workplace first-aid trained volunteers for emergency response", circle_type: "Workplace", location: "Roodepoort CBD", max_members: 20, member_count: 12, created_at: Date.now() - 2592e6 },
    { id: "cc-004", name: "St. Mark's Care Network", description: "Faith-based elder care and food distribution", circle_type: "Faith", location: "Horison, Roodepoort", max_members: 40, member_count: 27, created_at: Date.now() - 10368e6 }
  ];
}
function mockHousingUnits() {
  return [
    { id: "hu-001", building_id: "bld-001", unit_number: "A1", unit_type: "Studio", bedrooms: 0, bathrooms: 1, square_meters: 28, floor: 0, status: "Available", accessibility_features: ["WheelchairAccessible", "GrabBars"] },
    { id: "hu-002", building_id: "bld-001", unit_number: "B3", unit_type: "OneBedroom", bedrooms: 1, bathrooms: 1, square_meters: 42, floor: 1, status: "Available", accessibility_features: [] },
    { id: "hu-003", building_id: "bld-002", unit_number: "C2", unit_type: "TwoBedroom", bedrooms: 2, bathrooms: 1, square_meters: 58, floor: 0, status: "Available", accessibility_features: ["WheelchairAccessible", "WideDoorways"] },
    { id: "hu-004", building_id: "bld-002", unit_number: "D1", unit_type: "ThreeBedroom", bedrooms: 3, bathrooms: 2, square_meters: 76, floor: 1, status: "Available", accessibility_features: ["GrabBars"] }
  ];
}
function mockInventoryItems() {
  return [
    { id: "inv-001", name: "Rice 5kg bags", description: "Long-grain white rice, sealed bags", category: "Food", sku: "FD-RICE-5KG", unit: "bags", reorder_point: 20, reorder_quantity: 50 },
    { id: "inv-002", name: "Water purification tablets", description: "Chlorine-based water treatment, 50 per box", category: "Water", sku: "WT-PURIFY-50", unit: "boxes", reorder_point: 15, reorder_quantity: 40 },
    { id: "inv-003", name: "First aid kits", description: "Standard community first aid kit with bandages, antiseptic, gloves", category: "Medical", sku: "MD-FAID-STD", unit: "kits", reorder_point: 10, reorder_quantity: 25 },
    { id: "inv-004", name: "Diesel 20L", description: "Diesel fuel for generator backup", category: "Fuel", sku: "FL-DIESEL-20L", unit: "jerricans", reorder_point: 8, reorder_quantity: 20 },
    { id: "inv-005", name: "Soap bars", description: "Antibacterial soap, individually wrapped", category: "Hygiene", sku: "HY-SOAP-BAR", unit: "bars", reorder_point: 50, reorder_quantity: 200 },
    { id: "inv-006", name: "Emergency blankets", description: "Mylar thermal blankets, single-use", category: "Shelter", sku: "SH-BLANKET-EM", unit: "blankets", reorder_point: 30, reorder_quantity: 100 }
  ];
}
function mockLowStockItems() {
  const items = mockInventoryItems();
  return [
    { item: items[3], total_stock: 5 },
    // Diesel — below reorder_point of 8
    { item: items[2], total_stock: 7 }
    // First aid kits — below reorder_point of 10
  ];
}
function mockWaterSystems() {
  return [
    { id: "ws-001", name: "Sector 7 Community Roof Harvest", system_type: "RoofRainwater", capacity_liters: 1e4, catchment_area_sqm: 120, efficiency_percent: 85, owner_did: "community.did", location_lat: -26.1496, location_lon: 27.8625, installed_at: Date.now() - 15552e6 },
    { id: "ws-002", name: "Florida Lake Ground Catchment", system_type: "GroundCatchment", capacity_liters: 25e3, catchment_area_sqm: 500, efficiency_percent: 60, owner_did: "community.did", location_lat: -26.17, location_lon: 27.91, installed_at: Date.now() - 31104e6 },
    { id: "ws-003", name: "Hilltop Fog Collector", system_type: "FogCollection", capacity_liters: 2e3, catchment_area_sqm: 40, efficiency_percent: 30, owner_did: "sipho.did", location_lat: -26.135, location_lon: 27.845, installed_at: Date.now() - 7776e6 }
  ];
}
function mockWaterAlerts() {
  return [
    { id: "wa-001", source_id: "ws-002", contaminant: "E. coli", measured_value: 12, threshold_value: 1, severity: "Warning", reported_at: Date.now() - 864e5, reported_by: "thandi.did" }
  ];
}
function mockHearths() {
  return [
    { id: "hth-001", name: "Stoltz Family", member_count: 4, created_at: Date.now() - 31104e6 },
    { id: "hth-002", name: "Sector 7 Neighbours", member_count: 8, created_at: Date.now() - 15552e6 }
  ];
}
function mockGraphStats() {
  return {
    total_claims: 47,
    total_relationships: 83,
    total_ontologies: 5,
    total_concepts: 31
  };
}
function createFreshness(fetchFn, intervalMs) {
  const lastUpdated = writable(0);
  const loadError = writable("");
  const refreshing = writable(false);
  let timer = null;
  async function refresh() {
    refreshing.set(true);
    try {
      await fetchFn();
      lastUpdated.set(Date.now());
      loadError.set("");
    } catch (e) {
      loadError.set(e instanceof Error ? e.message : "Failed to load data");
    } finally {
      refreshing.set(false);
    }
  }
  function startPolling() {
    if (intervalMs > 0 && !timer) {
      timer = setInterval(refresh, intervalMs);
    }
  }
  function stopPolling() {
    if (timer) {
      clearInterval(timer);
      timer = null;
    }
  }
  return { lastUpdated, loadError, refreshing, startPolling, stopPolling, refresh };
}
function timeAgo(ts) {
  if (ts === 0) return "never";
  const seconds = Math.floor((Date.now() - ts) / 1e3);
  if (seconds < 10) return "just now";
  if (seconds < 60) return `${seconds}s ago`;
  const minutes = Math.floor(seconds / 60);
  if (minutes < 60) return `${minutes}m ago`;
  const hours = Math.floor(minutes / 60);
  return `${hours}h ago`;
}
export {
  getDaoRequests as a,
  getServiceOffers as b,
  createFreshness as c,
  getServiceRequests as d,
  getChannels as e,
  getAllPlots as f,
  getDaoListings as g,
  getAllWaterSystems as h,
  getActiveWaterAlerts as i,
  getLowStockItems as j,
  getOracleState as k,
  getMessages as l,
  getCommunityInputs as m,
  getNutrientSummary as n,
  getBalance as o,
  getMyHearths as p,
  getGraphStats as q,
  getAllCareCircles as r,
  getAvailableUnits as s,
  getAllInventoryItems as t,
  timeAgo as u
};
