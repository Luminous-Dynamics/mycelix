// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Metrics and Telemetry Service for Mycelix Mail
 *
 * Comprehensive monitoring:
 * - Performance metrics (latency, throughput)
 * - Usage analytics (emails sent/received)
 * - Trust network statistics
 * - Sync health metrics
 * - Error tracking
 * - Custom counters and gauges
 */

// ==================== TYPES ====================

export type MetricType = 'counter' | 'gauge' | 'histogram' | 'timer';

export interface Metric {
  name: string;
  type: MetricType;
  value: number;
  labels?: Record<string, string>;
  timestamp: number;
}

export interface HistogramBucket {
  le: number; // less than or equal
  count: number;
}

export interface HistogramMetric extends Metric {
  type: 'histogram';
  buckets: HistogramBucket[];
  sum: number;
  count: number;
}

export interface TimerMetric extends Metric {
  type: 'timer';
  min: number;
  max: number;
  mean: number;
  p50: number;
  p95: number;
  p99: number;
  count: number;
}

export interface MetricsSnapshot {
  timestamp: number;
  uptime: number;
  metrics: Record<string, Metric>;
  histograms: Record<string, HistogramMetric>;
  timers: Record<string, TimerMetric>;
}

export interface MetricsConfig {
  /** Enable metrics collection */
  enabled?: boolean;
  /** Flush interval (ms) */
  flushInterval?: number;
  /** Max metrics in memory */
  maxMetrics?: number;
  /** Default histogram buckets */
  defaultBuckets?: number[];
  /** Timer percentiles */
  timerPercentiles?: number[];
  /** Metric prefix */
  prefix?: string;
  /** Export endpoint */
  exportEndpoint?: string;
  /** Export format */
  exportFormat?: 'json' | 'prometheus';
}

export type MetricHandler = (metrics: MetricsSnapshot) => void;

// ==================== INTERNAL STRUCTURES ====================

interface CounterData {
  value: number;
  labels?: Record<string, string>;
}

interface GaugeData {
  value: number;
  labels?: Record<string, string>;
}

interface HistogramData {
  values: number[];
  buckets: number[];
  labels?: Record<string, string>;
}

interface TimerData {
  values: number[];
  labels?: Record<string, string>;
}

// ==================== METRICS SERVICE ====================

export class MetricsService {
  private counters: Map<string, CounterData> = new Map();
  private gauges: Map<string, GaugeData> = new Map();
  private histograms: Map<string, HistogramData> = new Map();
  private timers: Map<string, TimerData> = new Map();
  private activeTimers: Map<string, number> = new Map();
  private handlers: Set<MetricHandler> = new Set();
  private flushInterval: NodeJS.Timeout | null = null;
  private startTime: number;
  private config: MetricsConfig;

  private readonly DEFAULT_BUCKETS = [5, 10, 25, 50, 100, 250, 500, 1000, 2500, 5000, 10000];
  private readonly DEFAULT_PERCENTILES = [0.5, 0.95, 0.99];

  constructor(config: Partial<MetricsConfig> = {}) {
    this.startTime = Date.now();
    this.config = {
      enabled: true,
      flushInterval: 60000,
      maxMetrics: 10000,
      defaultBuckets: this.DEFAULT_BUCKETS,
      timerPercentiles: this.DEFAULT_PERCENTILES,
      prefix: 'mycelix_',
      exportFormat: 'json',
      ...config,
    };

    if (this.config.enabled && this.config.flushInterval) {
      this.startFlushInterval();
    }

    // Register built-in metrics
    this.registerBuiltinMetrics();
  }

  // ==================== COUNTERS ====================

  /**
   * Increment a counter
   */
  increment(name: string, value = 1, labels?: Record<string, string>): void {
    if (!this.config.enabled) return;

    const key = this.makeKey(name, labels);
    const current = this.counters.get(key);

    if (current) {
      current.value += value;
    } else {
      this.counters.set(key, { value, labels });
    }
  }

  /**
   * Get counter value
   */
  getCounter(name: string, labels?: Record<string, string>): number {
    const key = this.makeKey(name, labels);
    return this.counters.get(key)?.value ?? 0;
  }

  // ==================== GAUGES ====================

  /**
   * Set a gauge value
   */
  setGauge(name: string, value: number, labels?: Record<string, string>): void {
    if (!this.config.enabled) return;

    const key = this.makeKey(name, labels);
    this.gauges.set(key, { value, labels });
  }

  /**
   * Increment a gauge
   */
  incrementGauge(name: string, value = 1, labels?: Record<string, string>): void {
    if (!this.config.enabled) return;

    const key = this.makeKey(name, labels);
    const current = this.gauges.get(key);

    if (current) {
      current.value += value;
    } else {
      this.gauges.set(key, { value, labels });
    }
  }

  /**
   * Decrement a gauge
   */
  decrementGauge(name: string, value = 1, labels?: Record<string, string>): void {
    this.incrementGauge(name, -value, labels);
  }

  /**
   * Get gauge value
   */
  getGauge(name: string, labels?: Record<string, string>): number {
    const key = this.makeKey(name, labels);
    return this.gauges.get(key)?.value ?? 0;
  }

  // ==================== HISTOGRAMS ====================

  /**
   * Record a histogram value
   */
  recordHistogram(
    name: string,
    value: number,
    labels?: Record<string, string>,
    buckets?: number[]
  ): void {
    if (!this.config.enabled) return;

    const key = this.makeKey(name, labels);
    let histogram = this.histograms.get(key);

    if (!histogram) {
      histogram = {
        values: [],
        buckets: buckets ?? this.config.defaultBuckets!,
        labels,
      };
      this.histograms.set(key, histogram);
    }

    histogram.values.push(value);

    // Limit stored values
    if (histogram.values.length > this.config.maxMetrics!) {
      histogram.values = histogram.values.slice(-this.config.maxMetrics!);
    }
  }

  /**
   * Get histogram stats
   */
  getHistogram(name: string, labels?: Record<string, string>): HistogramMetric | null {
    const key = this.makeKey(name, labels);
    const histogram = this.histograms.get(key);

    if (!histogram || histogram.values.length === 0) {
      return null;
    }

    const bucketCounts = histogram.buckets.map((le) => ({
      le,
      count: histogram.values.filter((v) => v <= le).length,
    }));

    return {
      name: this.prefixName(name),
      type: 'histogram',
      value: histogram.values.length,
      labels: histogram.labels,
      timestamp: Date.now(),
      buckets: bucketCounts,
      sum: histogram.values.reduce((a, b) => a + b, 0),
      count: histogram.values.length,
    };
  }

  // ==================== TIMERS ====================

  /**
   * Start a timer
   */
  startTimer(name: string, labels?: Record<string, string>): string {
    const timerId = `${name}_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    this.activeTimers.set(timerId, Date.now());
    return timerId;
  }

  /**
   * Stop a timer and record the duration
   */
  stopTimer(timerId: string, labels?: Record<string, string>): number {
    const startTime = this.activeTimers.get(timerId);
    if (!startTime) {
      console.warn(`Timer not found: ${timerId}`);
      return 0;
    }

    const duration = Date.now() - startTime;
    this.activeTimers.delete(timerId);

    // Extract name from timerId
    const name = timerId.split('_').slice(0, -2).join('_');
    this.recordTimer(name, duration, labels);

    return duration;
  }

  /**
   * Record a timer value directly
   */
  recordTimer(name: string, duration: number, labels?: Record<string, string>): void {
    if (!this.config.enabled) return;

    const key = this.makeKey(name, labels);
    let timer = this.timers.get(key);

    if (!timer) {
      timer = { values: [], labels };
      this.timers.set(key, timer);
    }

    timer.values.push(duration);

    // Limit stored values
    if (timer.values.length > this.config.maxMetrics!) {
      timer.values = timer.values.slice(-this.config.maxMetrics!);
    }
  }

  /**
   * Time a function execution
   */
  async time<T>(
    name: string,
    fn: () => T | Promise<T>,
    labels?: Record<string, string>
  ): Promise<T> {
    const timerId = this.startTimer(name, labels);
    try {
      return await fn();
    } finally {
      this.stopTimer(timerId, labels);
    }
  }

  /**
   * Get timer stats
   */
  getTimer(name: string, labels?: Record<string, string>): TimerMetric | null {
    const key = this.makeKey(name, labels);
    const timer = this.timers.get(key);

    if (!timer || timer.values.length === 0) {
      return null;
    }

    const sorted = [...timer.values].sort((a, b) => a - b);
    const sum = sorted.reduce((a, b) => a + b, 0);
    const count = sorted.length;

    return {
      name: this.prefixName(name),
      type: 'timer',
      value: sum / count,
      labels: timer.labels,
      timestamp: Date.now(),
      min: sorted[0],
      max: sorted[count - 1],
      mean: sum / count,
      p50: this.percentile(sorted, 0.5),
      p95: this.percentile(sorted, 0.95),
      p99: this.percentile(sorted, 0.99),
      count,
    };
  }

  // ==================== BUILT-IN METRICS ====================

  /**
   * Register built-in metrics
   */
  private registerBuiltinMetrics(): void {
    // Will be populated as operations occur
    this.counters.set('emails_sent_total', { value: 0 });
    this.counters.set('emails_received_total', { value: 0 });
    this.counters.set('emails_read_total', { value: 0 });
    this.counters.set('emails_deleted_total', { value: 0 });
    this.counters.set('sync_operations_total', { value: 0 });
    this.counters.set('trust_attestations_total', { value: 0 });
    this.counters.set('errors_total', { value: 0 });

    this.gauges.set('inbox_size', { value: 0 });
    this.gauges.set('pending_sync_operations', { value: 0 });
    this.gauges.set('active_connections', { value: 0 });
    this.gauges.set('cache_size', { value: 0 });
    this.gauges.set('cache_hit_rate', { value: 0 });
  }

  /**
   * Record email metrics
   */
  recordEmailSent(): void {
    this.increment('emails_sent_total');
    this.recordTimer('email_send_duration', 0); // Will be updated
  }

  recordEmailReceived(isSpam = false): void {
    this.increment('emails_received_total');
    if (isSpam) {
      this.increment('spam_detected_total');
    }
  }

  recordEmailRead(): void {
    this.increment('emails_read_total');
  }

  recordEmailDeleted(): void {
    this.increment('emails_deleted_total');
  }

  /**
   * Record sync metrics
   */
  recordSync(operationsCount: number, duration: number): void {
    this.increment('sync_operations_total', operationsCount);
    this.recordTimer('sync_duration', duration);
    this.recordHistogram('sync_operations_per_batch', operationsCount);
  }

  /**
   * Record trust metrics
   */
  recordTrustAttestation(category: string): void {
    this.increment('trust_attestations_total', 1, { category });
  }

  /**
   * Record error
   */
  recordError(type: string, message?: string): void {
    this.increment('errors_total', 1, { type });
  }

  // ==================== SNAPSHOT & EXPORT ====================

  /**
   * Get metrics snapshot
   */
  getSnapshot(): MetricsSnapshot {
    const now = Date.now();
    const metrics: Record<string, Metric> = {};
    const histogramMetrics: Record<string, HistogramMetric> = {};
    const timerMetrics: Record<string, TimerMetric> = {};

    // Counters
    for (const [key, data] of this.counters) {
      const name = this.extractName(key);
      metrics[key] = {
        name: this.prefixName(name),
        type: 'counter',
        value: data.value,
        labels: data.labels,
        timestamp: now,
      };
    }

    // Gauges
    for (const [key, data] of this.gauges) {
      const name = this.extractName(key);
      metrics[key] = {
        name: this.prefixName(name),
        type: 'gauge',
        value: data.value,
        labels: data.labels,
        timestamp: now,
      };
    }

    // Histograms
    for (const [key, _data] of this.histograms) {
      const name = this.extractName(key);
      const histogram = this.getHistogram(name);
      if (histogram) {
        histogramMetrics[key] = histogram;
      }
    }

    // Timers
    for (const [key, _data] of this.timers) {
      const name = this.extractName(key);
      const timer = this.getTimer(name);
      if (timer) {
        timerMetrics[key] = timer;
      }
    }

    return {
      timestamp: now,
      uptime: now - this.startTime,
      metrics,
      histograms: histogramMetrics,
      timers: timerMetrics,
    };
  }

  /**
   * Export metrics in specified format
   */
  export(format?: 'json' | 'prometheus'): string {
    const fmt = format ?? this.config.exportFormat!;
    const snapshot = this.getSnapshot();

    if (fmt === 'prometheus') {
      return this.toPrometheus(snapshot);
    }

    return JSON.stringify(snapshot, null, 2);
  }

  /**
   * Convert to Prometheus format
   */
  private toPrometheus(snapshot: MetricsSnapshot): string {
    const lines: string[] = [];

    // Counters and Gauges
    for (const [_key, metric] of Object.entries(snapshot.metrics)) {
      const labels = this.formatLabels(metric.labels);
      lines.push(`# TYPE ${metric.name} ${metric.type}`);
      lines.push(`${metric.name}${labels} ${metric.value}`);
    }

    // Histograms
    for (const [_key, histogram] of Object.entries(snapshot.histograms)) {
      const labels = this.formatLabels(histogram.labels);
      lines.push(`# TYPE ${histogram.name} histogram`);

      for (const bucket of histogram.buckets) {
        const bucketLabel = labels
          ? `${labels.slice(0, -1)},le="${bucket.le}"}`
          : `{le="${bucket.le}"}`;
        lines.push(`${histogram.name}_bucket${bucketLabel} ${bucket.count}`);
      }

      lines.push(`${histogram.name}_sum${labels} ${histogram.sum}`);
      lines.push(`${histogram.name}_count${labels} ${histogram.count}`);
    }

    // Timers (as summary)
    for (const [_key, timer] of Object.entries(snapshot.timers)) {
      const labels = this.formatLabels(timer.labels);
      lines.push(`# TYPE ${timer.name}_seconds summary`);

      const quantileLabels = (q: number) =>
        labels
          ? `${labels.slice(0, -1)},quantile="${q}"}`
          : `{quantile="${q}"}`;

      lines.push(`${timer.name}_seconds${quantileLabels(0.5)} ${timer.p50 / 1000}`);
      lines.push(`${timer.name}_seconds${quantileLabels(0.95)} ${timer.p95 / 1000}`);
      lines.push(`${timer.name}_seconds${quantileLabels(0.99)} ${timer.p99 / 1000}`);
      lines.push(`${timer.name}_seconds_sum${labels} ${timer.mean * timer.count / 1000}`);
      lines.push(`${timer.name}_seconds_count${labels} ${timer.count}`);
    }

    return lines.join('\n');
  }

  // ==================== FLUSH & HANDLERS ====================

  /**
   * Start periodic flush
   */
  private startFlushInterval(): void {
    this.flushInterval = setInterval(() => {
      this.flush();
    }, this.config.flushInterval!);
  }

  /**
   * Flush metrics to handlers
   */
  flush(): void {
    const snapshot = this.getSnapshot();

    for (const handler of this.handlers) {
      try {
        handler(snapshot);
      } catch (e) {
        console.error('Metrics handler error:', e);
      }
    }

    // Export if endpoint configured
    if (this.config.exportEndpoint) {
      this.sendToEndpoint(snapshot);
    }
  }

  /**
   * Send metrics to export endpoint
   */
  private async sendToEndpoint(snapshot: MetricsSnapshot): Promise<void> {
    if (!this.config.exportEndpoint) return;

    try {
      const body = this.config.exportFormat === 'prometheus'
        ? this.toPrometheus(snapshot)
        : JSON.stringify(snapshot);

      const contentType = this.config.exportFormat === 'prometheus'
        ? 'text/plain'
        : 'application/json';

      await fetch(this.config.exportEndpoint, {
        method: 'POST',
        headers: { 'Content-Type': contentType },
        body,
      });
    } catch (e) {
      console.error('Failed to export metrics:', e);
    }
  }

  /**
   * Subscribe to metrics flush
   */
  subscribe(handler: MetricHandler): () => void {
    this.handlers.add(handler);
    return () => this.handlers.delete(handler);
  }

  // ==================== UTILITIES ====================

  private makeKey(name: string, labels?: Record<string, string>): string {
    if (!labels || Object.keys(labels).length === 0) {
      return name;
    }

    const labelStr = Object.entries(labels)
      .sort(([a], [b]) => a.localeCompare(b))
      .map(([k, v]) => `${k}="${v}"`)
      .join(',');

    return `${name}{${labelStr}}`;
  }

  private extractName(key: string): string {
    return key.split('{')[0];
  }

  private prefixName(name: string): string {
    if (this.config.prefix && !name.startsWith(this.config.prefix)) {
      return this.config.prefix + name;
    }
    return name;
  }

  private formatLabels(labels?: Record<string, string>): string {
    if (!labels || Object.keys(labels).length === 0) {
      return '';
    }

    const labelStr = Object.entries(labels)
      .map(([k, v]) => `${k}="${v}"`)
      .join(',');

    return `{${labelStr}}`;
  }

  private percentile(sorted: number[], p: number): number {
    if (sorted.length === 0) return 0;
    const index = Math.ceil(p * sorted.length) - 1;
    return sorted[Math.max(0, index)];
  }

  /**
   * Reset all metrics
   */
  reset(): void {
    this.counters.clear();
    this.gauges.clear();
    this.histograms.clear();
    this.timers.clear();
    this.activeTimers.clear();
    this.registerBuiltinMetrics();
  }

  /**
   * Stop the service
   */
  stop(): void {
    if (this.flushInterval) {
      clearInterval(this.flushInterval);
      this.flushInterval = null;
    }
  }
}

/**
 * Create metrics service
 */
export function createMetricsService(config?: Partial<MetricsConfig>): MetricsService {
  return new MetricsService(config);
}

/**
 * Global metrics instance
 */
let globalMetrics: MetricsService | null = null;

export function getMetrics(config?: Partial<MetricsConfig>): MetricsService {
  if (!globalMetrics) {
    globalMetrics = new MetricsService(config);
  }
  return globalMetrics;
}

export default MetricsService;
