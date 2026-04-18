// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Health Monitoring Service for Mycelix Mail
 *
 * Monitors:
 * - Holochain conductor connectivity
 * - Zome health status
 * - Network peer reachability
 * - Sync state health
 * - Performance metrics
 */

import type { AppClient } from '@holochain/client';
import type { MycelixMailClient } from '../index';

// ==================== TYPES ====================

export interface HealthStatus {
  overall: 'healthy' | 'degraded' | 'unhealthy';
  timestamp: Date;
  components: ComponentHealth[];
  metrics: HealthMetrics;
  alerts: HealthAlert[];
}

export interface ComponentHealth {
  name: string;
  status: 'healthy' | 'degraded' | 'unhealthy' | 'unknown';
  latency_ms: number | null;
  last_check: Date;
  error: string | null;
  details: Record<string, unknown>;
}

export interface HealthMetrics {
  uptime_ms: number;
  total_requests: number;
  failed_requests: number;
  avg_latency_ms: number;
  p95_latency_ms: number;
  p99_latency_ms: number;
  cache_hit_rate: number;
  sync_lag_ms: number;
  pending_operations: number;
  memory_usage_mb: number;
}

export interface HealthAlert {
  level: 'info' | 'warning' | 'error' | 'critical';
  component: string;
  message: string;
  timestamp: Date;
  resolved: boolean;
}

export interface HealthCheckResult {
  name: string;
  passed: boolean;
  duration_ms: number;
  error?: string;
}

export type HealthEventHandler = (status: HealthStatus) => void;

// ==================== HEALTH SERVICE ====================

export class HealthService {
  private startTime: Date;
  private requestCount = 0;
  private failedRequests = 0;
  private latencies: number[] = [];
  private maxLatencyHistory = 1000;
  private alerts: HealthAlert[] = [];
  private handlers: Set<HealthEventHandler> = new Set();
  private checkInterval: NodeJS.Timeout | null = null;
  private lastStatus: HealthStatus | null = null;

  constructor(
    private client: MycelixMailClient,
    private checkIntervalMs: number = 30000
  ) {
    this.startTime = new Date();
  }

  // ==================== LIFECYCLE ====================

  /**
   * Start health monitoring
   */
  start(): void {
    if (this.checkInterval) return;

    this.checkInterval = setInterval(() => {
      this.performHealthCheck();
    }, this.checkIntervalMs);

    // Initial check
    this.performHealthCheck();
  }

  /**
   * Stop health monitoring
   */
  stop(): void {
    if (this.checkInterval) {
      clearInterval(this.checkInterval);
      this.checkInterval = null;
    }
  }

  /**
   * Subscribe to health updates
   */
  subscribe(handler: HealthEventHandler): () => void {
    this.handlers.add(handler);
    return () => this.handlers.delete(handler);
  }

  // ==================== HEALTH CHECKS ====================

  /**
   * Perform comprehensive health check
   */
  async performHealthCheck(): Promise<HealthStatus> {
    const components: ComponentHealth[] = [];
    const checks: Promise<ComponentHealth>[] = [];

    // Check each component
    checks.push(this.checkConductor());
    checks.push(this.checkMessagesZome());
    checks.push(this.checkTrustZome());
    checks.push(this.checkSyncZome());
    checks.push(this.checkNetwork());

    const results = await Promise.all(checks);
    components.push(...results);

    // Calculate overall status
    const overall = this.calculateOverallStatus(components);

    // Build metrics
    const metrics = this.getMetrics();

    // Check for new alerts
    this.checkForAlerts(components, metrics);

    const status: HealthStatus = {
      overall,
      timestamp: new Date(),
      components,
      metrics,
      alerts: this.alerts.filter(a => !a.resolved),
    };

    this.lastStatus = status;

    // Notify handlers
    this.handlers.forEach(handler => handler(status));

    return status;
  }

  /**
   * Check Holochain conductor
   */
  private async checkConductor(): Promise<ComponentHealth> {
    const start = Date.now();
    try {
      // Simple ping by getting app info
      await (this.client as any).client.appInfo();

      return {
        name: 'conductor',
        status: 'healthy',
        latency_ms: Date.now() - start,
        last_check: new Date(),
        error: null,
        details: {},
      };
    } catch (error) {
      return {
        name: 'conductor',
        status: 'unhealthy',
        latency_ms: Date.now() - start,
        last_check: new Date(),
        error: (error as Error).message,
        details: {},
      };
    }
  }

  /**
   * Check messages zome
   */
  private async checkMessagesZome(): Promise<ComponentHealth> {
    const start = Date.now();
    try {
      await this.client.messages.getStats();

      return {
        name: 'messages',
        status: 'healthy',
        latency_ms: Date.now() - start,
        last_check: new Date(),
        error: null,
        details: {},
      };
    } catch (error) {
      return {
        name: 'messages',
        status: 'unhealthy',
        latency_ms: Date.now() - start,
        last_check: new Date(),
        error: (error as Error).message,
        details: {},
      };
    }
  }

  /**
   * Check trust zome
   */
  private async checkTrustZome(): Promise<ComponentHealth> {
    const start = Date.now();
    try {
      await this.client.trust.getMyStake();

      return {
        name: 'trust',
        status: 'healthy',
        latency_ms: Date.now() - start,
        last_check: new Date(),
        error: null,
        details: {},
      };
    } catch (error) {
      return {
        name: 'trust',
        status: 'degraded', // Trust is not critical
        latency_ms: Date.now() - start,
        last_check: new Date(),
        error: (error as Error).message,
        details: {},
      };
    }
  }

  /**
   * Check sync zome
   */
  private async checkSyncZome(): Promise<ComponentHealth> {
    const start = Date.now();
    try {
      const state = await this.client.sync.getSyncState();

      const details: Record<string, unknown> = {};
      if (state) {
        details.is_online = state.is_online;
        details.pending_ops = state.pending_ops;
      }

      const status = state?.is_online ? 'healthy' : 'degraded';

      return {
        name: 'sync',
        status,
        latency_ms: Date.now() - start,
        last_check: new Date(),
        error: null,
        details,
      };
    } catch (error) {
      return {
        name: 'sync',
        status: 'unhealthy',
        latency_ms: Date.now() - start,
        last_check: new Date(),
        error: (error as Error).message,
        details: {},
      };
    }
  }

  /**
   * Check network peers
   */
  private async checkNetwork(): Promise<ComponentHealth> {
    const start = Date.now();
    try {
      const peers = await this.client.sync.getSyncPeers();
      const reachable = peers.filter(p => p.is_reachable).length;

      const status = reachable > 0 ? 'healthy' : 'degraded';

      return {
        name: 'network',
        status,
        latency_ms: Date.now() - start,
        last_check: new Date(),
        error: null,
        details: {
          total_peers: peers.length,
          reachable_peers: reachable,
        },
      };
    } catch (error) {
      return {
        name: 'network',
        status: 'unknown',
        latency_ms: Date.now() - start,
        last_check: new Date(),
        error: (error as Error).message,
        details: {},
      };
    }
  }

  // ==================== METRICS ====================

  /**
   * Record a request
   */
  recordRequest(latency_ms: number, success: boolean): void {
    this.requestCount++;
    if (!success) this.failedRequests++;

    this.latencies.push(latency_ms);
    if (this.latencies.length > this.maxLatencyHistory) {
      this.latencies.shift();
    }
  }

  /**
   * Get current metrics
   */
  getMetrics(): HealthMetrics {
    const sortedLatencies = [...this.latencies].sort((a, b) => a - b);
    const len = sortedLatencies.length;

    return {
      uptime_ms: Date.now() - this.startTime.getTime(),
      total_requests: this.requestCount,
      failed_requests: this.failedRequests,
      avg_latency_ms: len > 0
        ? sortedLatencies.reduce((a, b) => a + b, 0) / len
        : 0,
      p95_latency_ms: len > 0
        ? sortedLatencies[Math.floor(len * 0.95)] ?? 0
        : 0,
      p99_latency_ms: len > 0
        ? sortedLatencies[Math.floor(len * 0.99)] ?? 0
        : 0,
      cache_hit_rate: 0, // Would integrate with CacheService
      sync_lag_ms: 0, // Would calculate from sync state
      pending_operations: 0,
      memory_usage_mb: this.getMemoryUsage(),
    };
  }

  private getMemoryUsage(): number {
    if (typeof performance !== 'undefined' && 'memory' in performance) {
      return (performance as any).memory.usedJSHeapSize / 1024 / 1024;
    }
    return 0;
  }

  // ==================== ALERTS ====================

  private checkForAlerts(
    components: ComponentHealth[],
    metrics: HealthMetrics
  ): void {
    // Check component health
    for (const component of components) {
      if (component.status === 'unhealthy') {
        this.addAlert({
          level: 'error',
          component: component.name,
          message: `${component.name} is unhealthy: ${component.error}`,
          timestamp: new Date(),
          resolved: false,
        });
      } else {
        // Resolve existing alerts
        this.resolveAlerts(component.name);
      }
    }

    // Check latency
    if (metrics.p95_latency_ms > 5000) {
      this.addAlert({
        level: 'warning',
        component: 'performance',
        message: `High latency detected: P95 = ${metrics.p95_latency_ms}ms`,
        timestamp: new Date(),
        resolved: false,
      });
    }

    // Check error rate
    const errorRate = metrics.total_requests > 0
      ? metrics.failed_requests / metrics.total_requests
      : 0;

    if (errorRate > 0.1) {
      this.addAlert({
        level: 'warning',
        component: 'reliability',
        message: `High error rate: ${(errorRate * 100).toFixed(1)}%`,
        timestamp: new Date(),
        resolved: false,
      });
    }
  }

  private addAlert(alert: HealthAlert): void {
    // Don't duplicate alerts
    const existing = this.alerts.find(
      a => a.component === alert.component && !a.resolved
    );
    if (!existing) {
      this.alerts.push(alert);
    }
  }

  private resolveAlerts(component: string): void {
    for (const alert of this.alerts) {
      if (alert.component === component && !alert.resolved) {
        alert.resolved = true;
      }
    }
  }

  // ==================== UTILITIES ====================

  private calculateOverallStatus(
    components: ComponentHealth[]
  ): 'healthy' | 'degraded' | 'unhealthy' {
    const unhealthy = components.filter(c => c.status === 'unhealthy');
    const degraded = components.filter(c => c.status === 'degraded');

    if (unhealthy.length > 0) return 'unhealthy';
    if (degraded.length > 0) return 'degraded';
    return 'healthy';
  }

  /**
   * Get last health status
   */
  getLastStatus(): HealthStatus | null {
    return this.lastStatus;
  }

  /**
   * Get all alerts
   */
  getAlerts(): HealthAlert[] {
    return [...this.alerts];
  }

  /**
   * Clear resolved alerts
   */
  clearResolvedAlerts(): void {
    this.alerts = this.alerts.filter(a => !a.resolved);
  }

  /**
   * Run specific health check
   */
  async runCheck(name: string): Promise<HealthCheckResult> {
    const start = Date.now();

    try {
      switch (name) {
        case 'conductor':
          await this.checkConductor();
          break;
        case 'messages':
          await this.checkMessagesZome();
          break;
        case 'trust':
          await this.checkTrustZome();
          break;
        case 'sync':
          await this.checkSyncZome();
          break;
        case 'network':
          await this.checkNetwork();
          break;
        default:
          throw new Error(`Unknown check: ${name}`);
      }

      return {
        name,
        passed: true,
        duration_ms: Date.now() - start,
      };
    } catch (error) {
      return {
        name,
        passed: false,
        duration_ms: Date.now() - start,
        error: (error as Error).message,
      };
    }
  }
}

export default HealthService;
