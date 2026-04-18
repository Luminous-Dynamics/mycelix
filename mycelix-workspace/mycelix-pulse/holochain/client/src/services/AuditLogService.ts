// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Audit Log Service for Mycelix Mail
 *
 * Comprehensive activity tracking:
 * - All user actions logged
 * - Security events
 * - Compliance reporting
 * - Tamper-evident storage
 * - Export for auditors
 */

import type { ActionHash, AgentPubKey } from '@holochain/client';
import type { MycelixMailClient } from '../index';

// ==================== TYPES ====================

export type AuditCategory =
  | 'authentication'
  | 'email'
  | 'contact'
  | 'trust'
  | 'sync'
  | 'settings'
  | 'security'
  | 'admin'
  | 'system';

export type AuditAction =
  // Authentication
  | 'login'
  | 'logout'
  | 'session_start'
  | 'session_end'
  | 'key_rotation'
  // Email actions
  | 'email_send'
  | 'email_receive'
  | 'email_read'
  | 'email_delete'
  | 'email_archive'
  | 'email_forward'
  | 'email_reply'
  | 'email_export'
  | 'email_import'
  | 'draft_create'
  | 'draft_update'
  | 'draft_delete'
  | 'attachment_download'
  | 'attachment_upload'
  // Contact actions
  | 'contact_create'
  | 'contact_update'
  | 'contact_delete'
  | 'contact_block'
  | 'contact_unblock'
  | 'contact_import'
  | 'contact_export'
  // Trust actions
  | 'trust_attest'
  | 'trust_revoke'
  | 'trust_query'
  | 'byzantine_flag'
  // Sync actions
  | 'sync_start'
  | 'sync_complete'
  | 'sync_conflict'
  | 'sync_error'
  // Settings
  | 'settings_change'
  | 'filter_create'
  | 'filter_update'
  | 'filter_delete'
  // Security
  | 'permission_grant'
  | 'permission_revoke'
  | 'suspicious_activity'
  | 'rate_limit_exceeded'
  | 'encryption_key_access'
  // System
  | 'error'
  | 'warning'
  | 'migration';

export type AuditSeverity = 'debug' | 'info' | 'warning' | 'error' | 'critical';

export interface AuditEntry {
  id: string;
  hash?: ActionHash;
  timestamp: number;
  category: AuditCategory;
  action: AuditAction;
  severity: AuditSeverity;
  actor: AuditActor;
  target?: AuditTarget;
  details: AuditDetails;
  metadata: AuditMetadata;
  signature?: Uint8Array;
}

export interface AuditActor {
  agentPubKey?: AgentPubKey;
  email?: string;
  ipAddress?: string;
  userAgent?: string;
  sessionId?: string;
}

export interface AuditTarget {
  type: 'email' | 'contact' | 'thread' | 'folder' | 'setting' | 'agent' | 'system';
  id?: string;
  hash?: ActionHash;
  description?: string;
}

export interface AuditDetails {
  message: string;
  before?: Record<string, unknown>;
  after?: Record<string, unknown>;
  error?: string;
  stackTrace?: string;
  custom?: Record<string, unknown>;
}

export interface AuditMetadata {
  source: 'client' | 'server' | 'system' | 'external';
  version: string;
  correlationId?: string;
  parentId?: string;
  tags?: string[];
  retention?: number; // days
}

export interface AuditFilter {
  startDate?: number;
  endDate?: number;
  categories?: AuditCategory[];
  actions?: AuditAction[];
  severities?: AuditSeverity[];
  actorEmail?: string;
  actorAgent?: AgentPubKey;
  targetType?: string;
  targetId?: string;
  searchQuery?: string;
  tags?: string[];
}

export interface AuditReport {
  generatedAt: number;
  filter: AuditFilter;
  summary: AuditSummary;
  entries: AuditEntry[];
  hash: string; // For integrity verification
}

export interface AuditSummary {
  totalEntries: number;
  byCategory: Record<AuditCategory, number>;
  byAction: Record<string, number>;
  bySeverity: Record<AuditSeverity, number>;
  uniqueActors: number;
  timeRange: { start: number; end: number };
}

export interface AuditConfig {
  enabled: boolean;
  minSeverity: AuditSeverity;
  maxEntriesInMemory: number;
  flushInterval: number;
  retentionDays: number;
  signEntries: boolean;
  includeStackTraces: boolean;
  excludeActions?: AuditAction[];
}

export type AuditHandler = (entry: AuditEntry) => void;

// ==================== AUDIT LOG SERVICE ====================

export class AuditLogService {
  private entries: AuditEntry[] = [];
  private handlers: Set<AuditHandler> = new Set();
  private flushInterval: NodeJS.Timeout | null = null;
  private config: AuditConfig;
  private sessionId: string;
  private correlationStack: string[] = [];

  private readonly VERSION = '1.0.0';
  private readonly SEVERITY_LEVELS: Record<AuditSeverity, number> = {
    debug: 0,
    info: 1,
    warning: 2,
    error: 3,
    critical: 4,
  };

  constructor(
    private client?: MycelixMailClient,
    config: Partial<AuditConfig> = {}
  ) {
    this.sessionId = this.generateId();
    this.config = {
      enabled: true,
      minSeverity: 'info',
      maxEntriesInMemory: 10000,
      flushInterval: 60000,
      retentionDays: 365,
      signEntries: true,
      includeStackTraces: false,
      ...config,
    };

    if (this.config.enabled) {
      this.startFlushInterval();
      this.logSystemStart();
    }
  }

  // ==================== LOGGING ====================

  /**
   * Log an audit entry
   */
  log(
    category: AuditCategory,
    action: AuditAction,
    details: Partial<AuditDetails>,
    options: {
      severity?: AuditSeverity;
      target?: AuditTarget;
      tags?: string[];
    } = {}
  ): string {
    if (!this.config.enabled) return '';
    if (this.config.excludeActions?.includes(action)) return '';

    const severity = options.severity ?? this.getDefaultSeverity(action);
    if (this.SEVERITY_LEVELS[severity] < this.SEVERITY_LEVELS[this.config.minSeverity]) {
      return '';
    }

    const entry: AuditEntry = {
      id: this.generateId(),
      timestamp: Date.now(),
      category,
      action,
      severity,
      actor: this.getCurrentActor(),
      target: options.target,
      details: {
        message: details.message ?? this.getDefaultMessage(action),
        before: details.before,
        after: details.after,
        error: details.error,
        stackTrace: this.config.includeStackTraces ? details.stackTrace : undefined,
        custom: details.custom,
      },
      metadata: {
        source: 'client',
        version: this.VERSION,
        correlationId: this.correlationStack[this.correlationStack.length - 1],
        tags: options.tags,
        retention: this.config.retentionDays,
      },
    };

    // Sign entry if enabled
    if (this.config.signEntries) {
      entry.signature = this.signEntry(entry);
    }

    this.entries.push(entry);
    this.notifyHandlers(entry);

    // Trim if too many entries
    if (this.entries.length > this.config.maxEntriesInMemory) {
      this.entries = this.entries.slice(-this.config.maxEntriesInMemory);
    }

    return entry.id;
  }

  // ==================== CONVENIENCE METHODS ====================

  /**
   * Log email action
   */
  logEmail(
    action: AuditAction,
    emailHash: ActionHash,
    details?: Partial<AuditDetails>
  ): string {
    return this.log('email', action, details ?? {}, {
      target: { type: 'email', hash: emailHash },
    });
  }

  /**
   * Log contact action
   */
  logContact(
    action: AuditAction,
    contactId: string,
    details?: Partial<AuditDetails>
  ): string {
    return this.log('contact', action, details ?? {}, {
      target: { type: 'contact', id: contactId },
    });
  }

  /**
   * Log trust action
   */
  logTrust(
    action: AuditAction,
    agent: AgentPubKey,
    details?: Partial<AuditDetails>
  ): string {
    return this.log('trust', action, details ?? {}, {
      target: { type: 'agent', hash: agent as ActionHash },
    });
  }

  /**
   * Log security event
   */
  logSecurity(
    action: AuditAction,
    details: Partial<AuditDetails>,
    severity: AuditSeverity = 'warning'
  ): string {
    return this.log('security', action, details, { severity });
  }

  /**
   * Log error
   */
  logError(
    error: Error,
    context?: Record<string, unknown>
  ): string {
    return this.log('system', 'error', {
      message: error.message,
      error: error.name,
      stackTrace: error.stack,
      custom: context,
    }, {
      severity: 'error',
    });
  }

  /**
   * Log settings change
   */
  logSettingsChange(
    settingKey: string,
    before: unknown,
    after: unknown
  ): string {
    return this.log('settings', 'settings_change', {
      message: `Setting "${settingKey}" changed`,
      before: { [settingKey]: before },
      after: { [settingKey]: after },
    });
  }

  // ==================== CORRELATION ====================

  /**
   * Start a correlated operation
   */
  startCorrelation(name?: string): string {
    const correlationId = name ?? this.generateId();
    this.correlationStack.push(correlationId);
    return correlationId;
  }

  /**
   * End correlated operation
   */
  endCorrelation(): void {
    this.correlationStack.pop();
  }

  /**
   * Run function with correlation
   */
  async withCorrelation<T>(
    name: string,
    fn: () => T | Promise<T>
  ): Promise<T> {
    this.startCorrelation(name);
    try {
      return await fn();
    } finally {
      this.endCorrelation();
    }
  }

  // ==================== QUERYING ====================

  /**
   * Get entries by filter
   */
  getEntries(filter: AuditFilter = {}, limit = 100, offset = 0): AuditEntry[] {
    let results = this.entries;

    if (filter.startDate) {
      results = results.filter((e) => e.timestamp >= filter.startDate!);
    }

    if (filter.endDate) {
      results = results.filter((e) => e.timestamp <= filter.endDate!);
    }

    if (filter.categories?.length) {
      results = results.filter((e) => filter.categories!.includes(e.category));
    }

    if (filter.actions?.length) {
      results = results.filter((e) => filter.actions!.includes(e.action));
    }

    if (filter.severities?.length) {
      results = results.filter((e) => filter.severities!.includes(e.severity));
    }

    if (filter.actorEmail) {
      results = results.filter((e) => e.actor.email === filter.actorEmail);
    }

    if (filter.targetType) {
      results = results.filter((e) => e.target?.type === filter.targetType);
    }

    if (filter.targetId) {
      results = results.filter((e) => e.target?.id === filter.targetId);
    }

    if (filter.searchQuery) {
      const query = filter.searchQuery.toLowerCase();
      results = results.filter(
        (e) =>
          e.details.message.toLowerCase().includes(query) ||
          e.action.toLowerCase().includes(query)
      );
    }

    if (filter.tags?.length) {
      results = results.filter((e) =>
        e.metadata.tags?.some((t) => filter.tags!.includes(t))
      );
    }

    // Sort by timestamp descending
    results = results.sort((a, b) => b.timestamp - a.timestamp);

    return results.slice(offset, offset + limit);
  }

  /**
   * Get entry by ID
   */
  getEntry(id: string): AuditEntry | null {
    return this.entries.find((e) => e.id === id) ?? null;
  }

  /**
   * Get entries by correlation ID
   */
  getCorrelatedEntries(correlationId: string): AuditEntry[] {
    return this.entries.filter((e) => e.metadata.correlationId === correlationId);
  }

  // ==================== REPORTING ====================

  /**
   * Generate audit report
   */
  generateReport(filter: AuditFilter = {}): AuditReport {
    const entries = this.getEntries(filter, this.entries.length);
    const summary = this.calculateSummary(entries);

    const report: AuditReport = {
      generatedAt: Date.now(),
      filter,
      summary,
      entries,
      hash: '', // Will be calculated
    };

    // Calculate integrity hash
    report.hash = this.hashReport(report);

    return report;
  }

  /**
   * Calculate summary statistics
   */
  private calculateSummary(entries: AuditEntry[]): AuditSummary {
    const byCategory: Record<string, number> = {};
    const byAction: Record<string, number> = {};
    const bySeverity: Record<string, number> = {};
    const actors = new Set<string>();

    let minTime = Infinity;
    let maxTime = 0;

    for (const entry of entries) {
      byCategory[entry.category] = (byCategory[entry.category] ?? 0) + 1;
      byAction[entry.action] = (byAction[entry.action] ?? 0) + 1;
      bySeverity[entry.severity] = (bySeverity[entry.severity] ?? 0) + 1;

      if (entry.actor.email) actors.add(entry.actor.email);
      if (entry.actor.agentPubKey) actors.add(String(entry.actor.agentPubKey));

      minTime = Math.min(minTime, entry.timestamp);
      maxTime = Math.max(maxTime, entry.timestamp);
    }

    return {
      totalEntries: entries.length,
      byCategory: byCategory as Record<AuditCategory, number>,
      byAction,
      bySeverity: bySeverity as Record<AuditSeverity, number>,
      uniqueActors: actors.size,
      timeRange: {
        start: minTime === Infinity ? 0 : minTime,
        end: maxTime,
      },
    };
  }

  /**
   * Export audit log to JSON
   */
  exportToJson(filter?: AuditFilter): string {
    const report = this.generateReport(filter);
    return JSON.stringify(report, null, 2);
  }

  /**
   * Export audit log to CSV
   */
  exportToCsv(filter?: AuditFilter): string {
    const entries = this.getEntries(filter, this.entries.length);

    const headers = [
      'timestamp',
      'category',
      'action',
      'severity',
      'actor_email',
      'target_type',
      'target_id',
      'message',
    ];

    const rows = entries.map((e) => [
      new Date(e.timestamp).toISOString(),
      e.category,
      e.action,
      e.severity,
      e.actor.email ?? '',
      e.target?.type ?? '',
      e.target?.id ?? '',
      `"${e.details.message.replace(/"/g, '""')}"`,
    ]);

    return [headers.join(','), ...rows.map((r) => r.join(','))].join('\n');
  }

  // ==================== PERSISTENCE ====================

  /**
   * Flush entries to Holochain
   */
  async flush(): Promise<void> {
    if (!this.client || this.entries.length === 0) return;

    const toFlush = this.entries.filter((e) => !e.hash);

    for (const entry of toFlush) {
      try {
        entry.hash = await this.client.callZome('audit', 'create_entry', entry);
      } catch (error) {
        console.error('Failed to flush audit entry:', error);
      }
    }
  }

  /**
   * Load entries from Holochain
   */
  async loadFromHolochain(filter?: AuditFilter): Promise<void> {
    if (!this.client) return;

    try {
      const entries: AuditEntry[] = await this.client.callZome(
        'audit',
        'get_entries',
        filter
      );

      for (const entry of entries) {
        if (!this.entries.find((e) => e.id === entry.id)) {
          this.entries.push(entry);
        }
      }

      // Sort by timestamp
      this.entries.sort((a, b) => a.timestamp - b.timestamp);
    } catch (error) {
      console.error('Failed to load audit entries:', error);
    }
  }

  // ==================== HANDLERS ====================

  /**
   * Subscribe to audit events
   */
  subscribe(handler: AuditHandler): () => void {
    this.handlers.add(handler);
    return () => this.handlers.delete(handler);
  }

  /**
   * Notify handlers
   */
  private notifyHandlers(entry: AuditEntry): void {
    for (const handler of this.handlers) {
      try {
        handler(entry);
      } catch (e) {
        console.error('Audit handler error:', e);
      }
    }
  }

  // ==================== INTERNAL ====================

  /**
   * Get current actor info
   */
  private getCurrentActor(): AuditActor {
    return {
      agentPubKey: this.client?.myAgentPubKey,
      sessionId: this.sessionId,
      userAgent: typeof navigator !== 'undefined' ? navigator.userAgent : undefined,
    };
  }

  /**
   * Get default severity for action
   */
  private getDefaultSeverity(action: AuditAction): AuditSeverity {
    const severityMap: Partial<Record<AuditAction, AuditSeverity>> = {
      // Critical
      email_delete: 'warning',
      contact_delete: 'warning',
      trust_revoke: 'warning',
      permission_revoke: 'warning',
      suspicious_activity: 'critical',
      rate_limit_exceeded: 'warning',
      byzantine_flag: 'critical',
      // Errors
      error: 'error',
      sync_error: 'error',
      // Info
      login: 'info',
      logout: 'info',
      email_send: 'info',
      email_read: 'debug',
    };

    return severityMap[action] ?? 'info';
  }

  /**
   * Get default message for action
   */
  private getDefaultMessage(action: AuditAction): string {
    const messages: Partial<Record<AuditAction, string>> = {
      login: 'User logged in',
      logout: 'User logged out',
      session_start: 'Session started',
      session_end: 'Session ended',
      email_send: 'Email sent',
      email_receive: 'Email received',
      email_read: 'Email marked as read',
      email_delete: 'Email deleted',
      email_archive: 'Email archived',
      contact_create: 'Contact created',
      contact_update: 'Contact updated',
      contact_delete: 'Contact deleted',
      trust_attest: 'Trust attestation created',
      trust_revoke: 'Trust attestation revoked',
      sync_start: 'Sync started',
      sync_complete: 'Sync completed',
      sync_error: 'Sync error occurred',
      settings_change: 'Settings changed',
      suspicious_activity: 'Suspicious activity detected',
    };

    return messages[action] ?? `Action: ${action}`;
  }

  /**
   * Sign an entry (simplified)
   */
  private signEntry(entry: AuditEntry): Uint8Array {
    // In production, use proper cryptographic signing
    const data = JSON.stringify({
      id: entry.id,
      timestamp: entry.timestamp,
      category: entry.category,
      action: entry.action,
      details: entry.details.message,
    });

    // Simple hash for demo (use real signature in production)
    let hash = 0;
    for (let i = 0; i < data.length; i++) {
      const char = data.charCodeAt(i);
      hash = ((hash << 5) - hash) + char;
      hash = hash & hash;
    }

    return new Uint8Array([hash & 0xff, (hash >> 8) & 0xff, (hash >> 16) & 0xff, (hash >> 24) & 0xff]);
  }

  /**
   * Hash a report for integrity
   */
  private hashReport(report: AuditReport): string {
    const data = JSON.stringify({
      generatedAt: report.generatedAt,
      entryCount: report.entries.length,
      firstEntryId: report.entries[0]?.id,
      lastEntryId: report.entries[report.entries.length - 1]?.id,
    });

    // Simple hash for demo
    let hash = 0;
    for (let i = 0; i < data.length; i++) {
      const char = data.charCodeAt(i);
      hash = ((hash << 5) - hash) + char;
      hash = hash & hash;
    }

    return Math.abs(hash).toString(16);
  }

  /**
   * Log system start
   */
  private logSystemStart(): void {
    this.log('system', 'session_start', {
      message: 'Audit service started',
      custom: {
        version: this.VERSION,
        config: {
          minSeverity: this.config.minSeverity,
          retentionDays: this.config.retentionDays,
        },
      },
    });
  }

  /**
   * Start periodic flush
   */
  private startFlushInterval(): void {
    this.flushInterval = setInterval(() => {
      this.flush();
    }, this.config.flushInterval);
  }

  /**
   * Stop the service
   */
  async stop(): Promise<void> {
    if (this.flushInterval) {
      clearInterval(this.flushInterval);
      this.flushInterval = null;
    }

    this.log('system', 'session_end', {
      message: 'Audit service stopped',
    });

    await this.flush();
  }

  // ==================== UTILITIES ====================

  private generateId(): string {
    return `audit_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * Get entry count
   */
  getEntryCount(): number {
    return this.entries.length;
  }

  /**
   * Clear entries (for testing)
   */
  clear(): void {
    this.entries = [];
  }
}

/**
 * Create audit log service
 */
export function createAuditLogService(
  client?: MycelixMailClient,
  config?: Partial<AuditConfig>
): AuditLogService {
  return new AuditLogService(client, config);
}

/**
 * Global audit instance
 */
let globalAudit: AuditLogService | null = null;

export function getAuditLog(
  client?: MycelixMailClient,
  config?: Partial<AuditConfig>
): AuditLogService {
  if (!globalAudit) {
    globalAudit = new AuditLogService(client, config);
  }
  return globalAudit;
}

export default AuditLogService;
