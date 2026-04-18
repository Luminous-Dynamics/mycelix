// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { w as writable, d as derived } from "./index.js";
import { c as conductorStatus } from "./conductor.js";
const ECOSYSTEM_HAPPS = [
  // === CORE INFRASTRUCTURE ===
  {
    name: "Mycelix-Core",
    version: "0.6.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 6
    // FL coordinator/integrity, civitas reputation, causal contribution
  },
  {
    name: "Mycelix-Mail",
    version: "0.2.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 4
    // mail_messages, trust_filter, contacts, profiles
  },
  // === EXISTING DOMAIN HAPPS ===
  {
    name: "Mycelix-Marketplace",
    version: "0.4.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 12
    // listings, reputation, transactions, arbitration, messaging, notifications
  },
  {
    name: "Mycelix-EduNet",
    version: "0.3.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 18
    // learning, fl, credential, dao, pods, knowledge, srs, gamification, adaptive, integration
  },
  {
    name: "Mycelix-SupplyChain",
    version: "0.1.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 10
    // procurement, inventory, logistics, payments, trust
  },
  // === GOVERNANCE PILLAR (New Civilizational hApps) ===
  {
    name: "Mycelix-Identity",
    version: "0.1.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 4
    // did_registry, credential_schema, revocation, recovery
  },
  {
    name: "Mycelix-Governance",
    version: "0.1.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 4
    // proposals, voting, execution, constitution
  },
  {
    name: "Mycelix-Justice",
    version: "0.1.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 4
    // complaints, evidence, arbitration, enforcement
  },
  // === ECONOMIC PILLAR (New Civilizational hApps) ===
  {
    name: "Mycelix-Finance",
    version: "0.1.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 4
    // credit_scoring, lending, payments, treasury
  },
  {
    name: "Mycelix-Property",
    version: "0.1.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 4
    // registry, transfer, disputes, commons
  },
  {
    name: "Mycelix-Energy",
    version: "0.1.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 4
    // projects, investments, regenerative, grid
  },
  // === KNOWLEDGE PILLAR (New Civilizational hApps) ===
  {
    name: "Mycelix-Media",
    version: "0.1.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 4
    // publication, attribution, factcheck, curation
  },
  {
    name: "Mycelix-Knowledge",
    version: "0.1.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 4
    // claims, graph, query, inference
  },
  // === MANUFACTURING PILLAR (New Civilizational hApps) ===
  {
    name: "Mycelix-Fabrication",
    version: "0.1.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 14
    // designs, printers, prints, materials, verification, bridge, symthaea (integrity + coordinator each)
  },
  // === COMMUNITY PILLAR (New Civilizational hApps) ===
  {
    name: "Mycelix-Care",
    version: "0.1.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 12
    // caregiving, scheduling, needs, resources, matching, coordination (integrity + coordinator each)
  },
  {
    name: "Mycelix-Emergency",
    version: "0.1.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 14
    // alerts, response, resources, coordination, communication, mapping, recovery (integrity + coordinator each)
  },
  {
    name: "Mycelix-Water",
    version: "0.1.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 12
    // sources, quality, distribution, conservation, rights, monitoring (integrity + coordinator each)
  },
  {
    name: "Mycelix-Housing",
    version: "0.1.0",
    status: "healthy",
    hdkVersion: "0.6.0",
    nodeCount: 0,
    lastSeen: Date.now(),
    zomeCount: 14
    // listings, applications, leases, maintenance, community, cooperatives, commons (integrity + coordinator each)
  }
];
function defaultMetrics() {
  return {
    totalTransactions: 0,
    avgLatencyMs: 0,
    errorRate: 0,
    trustScoreAvg: 0.85,
    byzantineEvents: 0
  };
}
class EcosystemService {
  status;
  alerts = [];
  listeners = /* @__PURE__ */ new Set();
  mfaMetrics;
  mfaListeners = /* @__PURE__ */ new Set();
  constructor() {
    this.status = {
      healthy: true,
      totalNodes: 0,
      byzantineTolerance: 34,
      // 34% BFT threshold (validated maximum)
      happs: ECOSYSTEM_HAPPS.map((h) => ({ ...h, metrics: defaultMetrics() })),
      crossHappQueries: 0,
      bridgeMessageCount: 0,
      lastAuditTime: Date.now()
    };
    this.mfaMetrics = this.defaultMfaMetrics();
  }
  defaultMfaMetrics() {
    return {
      totalMfaEnabled: 0,
      totalDids: 0,
      mfaAdoptionRate: 0,
      assuranceLevelDistribution: {
        anonymous: 0,
        basic: 0,
        verified: 0,
        highlyAssured: 0,
        constitutionallyCritical: 0
      },
      factorTypeDistribution: {
        primaryKeyPair: 0,
        hardwareKey: 0,
        biometric: 0,
        socialRecovery: 0,
        gitcoinPassport: 0,
        verifiableCredential: 0,
        reputationAttestation: 0,
        securityQuestions: 0,
        recoveryPhrase: 0
      },
      flEligibleCount: 0,
      flEligibilityRate: 0,
      avgFactorsPerIdentity: 0,
      avgCategoriesPerIdentity: 0,
      avgAssuranceScore: 0,
      staleFactorCount: 0,
      factorsNeedingReverification: 0,
      recentEnrollments: 0,
      recentRevocations: 0,
      recentVerifications: 0
    };
  }
  /**
   * Get current ecosystem status
   */
  getStatus() {
    return { ...this.status };
  }
  /**
   * Get all active alerts
   */
  getAlerts() {
    return [...this.alerts];
  }
  /**
   * Subscribe to status updates
   */
  subscribe(callback) {
    this.listeners.add(callback);
    return () => this.listeners.delete(callback);
  }
  /**
   * Update hApp status (called when connecting to conductors)
   */
  updateHappStatus(name, update) {
    const happ = this.status.happs.find((h) => h.name === name);
    if (happ) {
      Object.assign(happ, update);
      happ.lastSeen = Date.now();
      this.recalculateEcosystemHealth();
      this.notifyListeners();
    }
  }
  /**
   * Record a cross-hApp bridge query
   */
  recordBridgeQuery() {
    this.status.crossHappQueries++;
    this.status.bridgeMessageCount++;
    this.notifyListeners();
  }
  /**
   * Add a Byzantine detection alert
   */
  addAlert(severity, message, source) {
    const alert = {
      id: `alert-${Date.now()}-${Math.random().toString(36).slice(2)}`,
      severity,
      message,
      source,
      timestamp: Date.now(),
      resolved: false
    };
    this.alerts.unshift(alert);
    if (this.alerts.length > 100) {
      this.alerts = this.alerts.slice(0, 100);
    }
    const happ = this.status.happs.find((h) => h.name === source);
    if (happ) {
      happ.metrics.byzantineEvents++;
    }
    this.notifyListeners();
  }
  /**
   * Resolve an alert
   */
  resolveAlert(alertId) {
    const alert = this.alerts.find((a) => a.id === alertId);
    if (alert) {
      alert.resolved = true;
      this.notifyListeners();
    }
  }
  /**
   * Simulate real-time metrics update (for demo/development)
   */
  simulateMetricsUpdate() {
    this.status.happs.forEach((happ) => {
      happ.metrics.totalTransactions += Math.floor(Math.random() * 10);
      happ.metrics.avgLatencyMs = 50 + Math.random() * 100;
      happ.metrics.trustScoreAvg = 0.8 + Math.random() * 0.15;
      happ.nodeCount = Math.floor(5 + Math.random() * 20);
    });
    this.status.totalNodes = this.status.happs.reduce((sum, h) => sum + h.nodeCount, 0);
    this.notifyListeners();
    this.simulateMfaMetrics();
  }
  // ============================================
  // MFA METRICS METHODS
  // ============================================
  /**
   * Get current MFA metrics
   */
  getMfaMetrics() {
    return { ...this.mfaMetrics };
  }
  /**
   * Subscribe to MFA metrics updates
   */
  subscribeMfa(callback) {
    this.mfaListeners.add(callback);
    return () => this.mfaListeners.delete(callback);
  }
  /**
   * Update MFA metrics from conductor data
   */
  updateMfaMetrics(metrics) {
    Object.assign(this.mfaMetrics, metrics);
    this.notifyMfaListeners();
  }
  /**
   * Record MFA enrollment event
   */
  recordMfaEnrollment() {
    this.mfaMetrics.recentEnrollments++;
    this.mfaMetrics.totalMfaEnabled++;
    this.recalculateMfaRates();
    this.notifyMfaListeners();
  }
  /**
   * Record MFA revocation event
   */
  recordMfaRevocation() {
    this.mfaMetrics.recentRevocations++;
    this.notifyMfaListeners();
  }
  /**
   * Record MFA verification event
   */
  recordMfaVerification() {
    this.mfaMetrics.recentVerifications++;
    if (this.mfaMetrics.factorsNeedingReverification > 0) {
      this.mfaMetrics.factorsNeedingReverification--;
    }
    this.notifyMfaListeners();
  }
  /**
   * Update assurance level distribution
   */
  updateAssuranceDistribution(level, delta) {
    const key = level.toLowerCase();
    if (key in this.mfaMetrics.assuranceLevelDistribution) {
      this.mfaMetrics.assuranceLevelDistribution[key] += delta;
    }
    this.recalculateMfaRates();
    this.notifyMfaListeners();
  }
  /**
   * Simulate MFA metrics for demo mode
   */
  simulateMfaMetrics() {
    const totalDids = 100 + Math.floor(Math.random() * 50);
    const mfaEnabled = Math.floor(totalDids * (0.65 + Math.random() * 0.2));
    this.mfaMetrics.totalDids = totalDids;
    this.mfaMetrics.totalMfaEnabled = mfaEnabled;
    this.mfaMetrics.mfaAdoptionRate = mfaEnabled / totalDids * 100;
    this.mfaMetrics.assuranceLevelDistribution = {
      anonymous: Math.floor(mfaEnabled * 0.05),
      basic: Math.floor(mfaEnabled * 0.25),
      verified: Math.floor(mfaEnabled * 0.4),
      highlyAssured: Math.floor(mfaEnabled * 0.22),
      constitutionallyCritical: Math.floor(mfaEnabled * 0.08)
    };
    this.mfaMetrics.factorTypeDistribution = {
      primaryKeyPair: mfaEnabled,
      // Everyone has this
      hardwareKey: Math.floor(mfaEnabled * 0.15),
      biometric: Math.floor(mfaEnabled * 0.25),
      socialRecovery: Math.floor(mfaEnabled * 0.3),
      gitcoinPassport: Math.floor(mfaEnabled * 0.2),
      verifiableCredential: Math.floor(mfaEnabled * 0.35),
      reputationAttestation: Math.floor(mfaEnabled * 0.45),
      securityQuestions: Math.floor(mfaEnabled * 0.4),
      recoveryPhrase: Math.floor(mfaEnabled * 0.55)
    };
    const flEligible = Math.floor(mfaEnabled * 0.35);
    this.mfaMetrics.flEligibleCount = flEligible;
    this.mfaMetrics.flEligibilityRate = flEligible / mfaEnabled * 100;
    this.mfaMetrics.avgFactorsPerIdentity = 2.3 + Math.random() * 0.5;
    this.mfaMetrics.avgCategoriesPerIdentity = 1.8 + Math.random() * 0.4;
    this.mfaMetrics.avgAssuranceScore = 0.45 + Math.random() * 0.15;
    this.mfaMetrics.staleFactorCount = Math.floor(mfaEnabled * 0.12);
    this.mfaMetrics.factorsNeedingReverification = Math.floor(mfaEnabled * 0.08);
    this.mfaMetrics.recentEnrollments = Math.floor(Math.random() * 5);
    this.mfaMetrics.recentRevocations = Math.floor(Math.random() * 2);
    this.mfaMetrics.recentVerifications = Math.floor(Math.random() * 15);
    this.notifyMfaListeners();
  }
  recalculateMfaRates() {
    if (this.mfaMetrics.totalDids > 0) {
      this.mfaMetrics.mfaAdoptionRate = this.mfaMetrics.totalMfaEnabled / this.mfaMetrics.totalDids * 100;
    }
    if (this.mfaMetrics.totalMfaEnabled > 0) {
      this.mfaMetrics.flEligibilityRate = this.mfaMetrics.flEligibleCount / this.mfaMetrics.totalMfaEnabled * 100;
    }
  }
  notifyMfaListeners() {
    const metrics = this.getMfaMetrics();
    this.mfaListeners.forEach((callback) => callback(metrics));
  }
  recalculateEcosystemHealth() {
    const healthyCount = this.status.happs.filter((h) => h.status === "healthy").length;
    const totalCount = this.status.happs.length;
    this.status.healthy = healthyCount / totalCount >= 0.6;
  }
  notifyListeners() {
    const status = this.getStatus();
    this.listeners.forEach((callback) => callback(status));
  }
}
let ecosystemService$1 = null;
function getEcosystemService() {
  if (!ecosystemService$1) {
    ecosystemService$1 = new EcosystemService();
  }
  return ecosystemService$1;
}
const ecosystemService = getEcosystemService();
const ecosystemStatus = writable(ecosystemService.getStatus());
const byzantineAlerts = writable(ecosystemService.getAlerts());
const isLiveMode = derived(conductorStatus, ($status) => $status === "connected");
export {
  byzantineAlerts as b,
  ecosystemStatus as e,
  isLiveMode as i
};
