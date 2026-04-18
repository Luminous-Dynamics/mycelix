// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Notification Queue Service for Mycelix Mail
 *
 * Priority-based notification system with:
 * - Priority queuing (urgent, high, normal, low)
 * - Deduplication
 * - Batching similar notifications
 * - Persistence (survives page refresh)
 * - Delivery confirmation
 * - Rate limiting
 * - Quiet hours support
 */

import type { ActionHash, AgentPubKey } from '@holochain/client';

// ==================== TYPES ====================

export type NotificationPriority = 'urgent' | 'high' | 'normal' | 'low';
export type NotificationType =
  | 'email_received'
  | 'email_sent'
  | 'read_receipt'
  | 'delivery_failed'
  | 'trust_change'
  | 'trust_warning'
  | 'sync_complete'
  | 'sync_conflict'
  | 'system'
  | 'mention'
  | 'reminder';

export interface QueuedNotification {
  id: string;
  type: NotificationType;
  priority: NotificationPriority;
  title: string;
  body: string;
  icon?: string;
  image?: string;
  data?: NotificationData;
  actions?: NotificationAction[];
  timestamp: number;
  expiresAt?: number;
  groupKey?: string;
  dedupeKey?: string;
  delivered: boolean;
  clicked: boolean;
  dismissed: boolean;
}

export interface NotificationData {
  emailHash?: ActionHash;
  sender?: AgentPubKey;
  threadId?: string;
  folderHash?: ActionHash;
  url?: string;
  [key: string]: unknown;
}

export interface NotificationAction {
  action: string;
  title: string;
  icon?: string;
}

export interface NotificationConfig {
  /** Enable browser notifications */
  browserNotifications?: boolean;
  /** Enable sound */
  sound?: boolean;
  /** Sound URL */
  soundUrl?: string;
  /** Max notifications per minute */
  rateLimit?: number;
  /** Quiet hours start (0-23) */
  quietHoursStart?: number;
  /** Quiet hours end (0-23) */
  quietHoursEnd?: number;
  /** Notification lifetime (ms) */
  defaultTTL?: number;
  /** Enable batching similar notifications */
  batchSimilar?: boolean;
  /** Batch window (ms) */
  batchWindow?: number;
  /** Max batch size before forced delivery */
  maxBatchSize?: number;
  /** Enable persistence */
  persist?: boolean;
}

export interface NotificationStats {
  total: number;
  delivered: number;
  clicked: number;
  dismissed: number;
  pending: number;
  byType: Record<NotificationType, number>;
  byPriority: Record<NotificationPriority, number>;
}

export type NotificationHandler = (notification: QueuedNotification) => void;
export type ActionHandler = (action: string, notification: QueuedNotification) => void;

// ==================== NOTIFICATION QUEUE SERVICE ====================

export class NotificationQueueService {
  private queue: Map<string, QueuedNotification> = new Map();
  private batchBuffer: Map<string, QueuedNotification[]> = new Map();
  private handlers: Set<NotificationHandler> = new Set();
  private actionHandlers: Map<string, ActionHandler> = new Map();
  private deliveryHistory: number[] = [];
  private config: NotificationConfig;
  private processInterval: NodeJS.Timeout | null = null;
  private batchTimeouts: Map<string, NodeJS.Timeout> = new Map();

  private readonly STORAGE_KEY = 'mycelix_notifications';
  private readonly PRIORITY_ORDER: NotificationPriority[] = ['urgent', 'high', 'normal', 'low'];

  constructor(config: Partial<NotificationConfig> = {}) {
    this.config = {
      browserNotifications: true,
      sound: true,
      soundUrl: '/notification.mp3',
      rateLimit: 10,
      quietHoursStart: undefined,
      quietHoursEnd: undefined,
      defaultTTL: 24 * 60 * 60 * 1000, // 24 hours
      batchSimilar: true,
      batchWindow: 5000,
      maxBatchSize: 5,
      persist: true,
      ...config,
    };

    this.loadFromStorage();
    this.startProcessing();
    this.requestBrowserPermission();
  }

  // ==================== QUEUE MANAGEMENT ====================

  /**
   * Add notification to queue
   */
  enqueue(notification: Omit<QueuedNotification, 'id' | 'timestamp' | 'delivered' | 'clicked' | 'dismissed'>): string {
    const id = this.generateId();
    const now = Date.now();

    const queued: QueuedNotification = {
      ...notification,
      id,
      timestamp: now,
      expiresAt: notification.expiresAt ?? now + this.config.defaultTTL!,
      delivered: false,
      clicked: false,
      dismissed: false,
    };

    // Check for deduplication
    if (queued.dedupeKey) {
      const existing = this.findByDedupeKey(queued.dedupeKey);
      if (existing) {
        // Update existing notification
        existing.timestamp = now;
        existing.title = queued.title;
        existing.body = queued.body;
        this.saveToStorage();
        return existing.id;
      }
    }

    // Handle batching
    if (this.config.batchSimilar && queued.groupKey) {
      return this.addToBatch(queued);
    }

    this.queue.set(id, queued);
    this.saveToStorage();
    this.processQueue();

    return id;
  }

  /**
   * Add to batch buffer for grouped delivery
   */
  private addToBatch(notification: QueuedNotification): string {
    const groupKey = notification.groupKey!;

    if (!this.batchBuffer.has(groupKey)) {
      this.batchBuffer.set(groupKey, []);

      // Set timeout to flush batch
      const timeout = setTimeout(() => {
        this.flushBatch(groupKey);
      }, this.config.batchWindow!);
      this.batchTimeouts.set(groupKey, timeout);
    }

    const batch = this.batchBuffer.get(groupKey)!;
    batch.push(notification);

    // Force flush if batch is too large
    if (batch.length >= this.config.maxBatchSize!) {
      this.flushBatch(groupKey);
    }

    return notification.id;
  }

  /**
   * Flush batched notifications
   */
  private flushBatch(groupKey: string): void {
    const batch = this.batchBuffer.get(groupKey);
    if (!batch || batch.length === 0) return;

    // Clear timeout
    const timeout = this.batchTimeouts.get(groupKey);
    if (timeout) {
      clearTimeout(timeout);
      this.batchTimeouts.delete(groupKey);
    }

    if (batch.length === 1) {
      // Single notification - enqueue normally
      this.queue.set(batch[0].id, batch[0]);
    } else {
      // Create batched notification
      const first = batch[0];
      const batchedNotification: QueuedNotification = {
        ...first,
        id: this.generateId(),
        title: this.getBatchTitle(batch),
        body: this.getBatchBody(batch),
        data: {
          ...first.data,
          batchedIds: batch.map((n) => n.id),
          batchCount: batch.length,
        },
      };
      this.queue.set(batchedNotification.id, batchedNotification);
    }

    this.batchBuffer.delete(groupKey);
    this.saveToStorage();
    this.processQueue();
  }

  /**
   * Generate batch title
   */
  private getBatchTitle(batch: QueuedNotification[]): string {
    const type = batch[0].type;
    const count = batch.length;

    switch (type) {
      case 'email_received':
        return `${count} new emails`;
      case 'read_receipt':
        return `${count} emails read`;
      case 'trust_change':
        return `${count} trust updates`;
      default:
        return `${count} notifications`;
    }
  }

  /**
   * Generate batch body
   */
  private getBatchBody(batch: QueuedNotification[]): string {
    const previews = batch
      .slice(0, 3)
      .map((n) => n.title)
      .join(', ');

    if (batch.length > 3) {
      return `${previews} and ${batch.length - 3} more`;
    }
    return previews;
  }

  /**
   * Remove notification from queue
   */
  dequeue(id: string): boolean {
    const deleted = this.queue.delete(id);
    if (deleted) {
      this.saveToStorage();
    }
    return deleted;
  }

  /**
   * Get notification by ID
   */
  get(id: string): QueuedNotification | undefined {
    return this.queue.get(id);
  }

  /**
   * Get all notifications
   */
  getAll(): QueuedNotification[] {
    return Array.from(this.queue.values()).sort(
      (a, b) => this.comparePriority(a, b)
    );
  }

  /**
   * Get pending notifications
   */
  getPending(): QueuedNotification[] {
    return this.getAll().filter((n) => !n.delivered && !n.dismissed);
  }

  /**
   * Get unread notifications
   */
  getUnread(): QueuedNotification[] {
    return this.getAll().filter((n) => n.delivered && !n.clicked && !n.dismissed);
  }

  /**
   * Find by dedupe key
   */
  private findByDedupeKey(key: string): QueuedNotification | undefined {
    for (const notification of this.queue.values()) {
      if (notification.dedupeKey === key) {
        return notification;
      }
    }
    return undefined;
  }

  // ==================== PROCESSING ====================

  /**
   * Start queue processing
   */
  private startProcessing(): void {
    if (this.processInterval) return;

    this.processInterval = setInterval(() => {
      this.cleanExpired();
    }, 60000); // Cleanup every minute
  }

  /**
   * Stop queue processing
   */
  stop(): void {
    if (this.processInterval) {
      clearInterval(this.processInterval);
      this.processInterval = null;
    }

    // Clear batch timeouts
    for (const timeout of this.batchTimeouts.values()) {
      clearTimeout(timeout);
    }
    this.batchTimeouts.clear();
  }

  /**
   * Process queue and deliver notifications
   */
  private async processQueue(): Promise<void> {
    const pending = this.getPending();
    if (pending.length === 0) return;

    // Check quiet hours
    if (this.isQuietHours()) {
      // Only deliver urgent notifications during quiet hours
      const urgent = pending.filter((n) => n.priority === 'urgent');
      for (const notification of urgent) {
        await this.deliver(notification);
      }
      return;
    }

    // Check rate limit
    const now = Date.now();
    this.deliveryHistory = this.deliveryHistory.filter(
      (ts) => now - ts < 60000
    );

    const remaining = this.config.rateLimit! - this.deliveryHistory.length;
    if (remaining <= 0) return;

    // Deliver in priority order
    const toDeliver = pending.slice(0, remaining);
    for (const notification of toDeliver) {
      await this.deliver(notification);
    }
  }

  /**
   * Deliver a notification
   */
  private async deliver(notification: QueuedNotification): Promise<void> {
    // Update delivery status
    notification.delivered = true;
    this.deliveryHistory.push(Date.now());
    this.saveToStorage();

    // Notify handlers
    for (const handler of this.handlers) {
      try {
        handler(notification);
      } catch (e) {
        console.error('Notification handler error:', e);
      }
    }

    // Browser notification
    if (this.config.browserNotifications) {
      await this.showBrowserNotification(notification);
    }

    // Play sound
    if (this.config.sound && notification.priority !== 'low') {
      this.playSound();
    }
  }

  /**
   * Show browser notification
   */
  private async showBrowserNotification(notification: QueuedNotification): Promise<void> {
    if (!('Notification' in window) || Notification.permission !== 'granted') {
      return;
    }

    try {
      const options: NotificationOptions = {
        body: notification.body,
        icon: notification.icon ?? '/icon.png',
        image: notification.image,
        tag: notification.groupKey ?? notification.id,
        renotify: true,
        requireInteraction: notification.priority === 'urgent',
        data: { id: notification.id, ...notification.data },
        actions: notification.actions?.slice(0, 2),
      };

      const browserNotification = new Notification(notification.title, options);

      browserNotification.onclick = () => {
        this.markClicked(notification.id);
        window.focus();
      };

      browserNotification.onclose = () => {
        this.markDismissed(notification.id);
      };
    } catch (e) {
      console.error('Browser notification error:', e);
    }
  }

  /**
   * Request browser notification permission
   */
  private async requestBrowserPermission(): Promise<void> {
    if (!('Notification' in window)) return;

    if (Notification.permission === 'default') {
      try {
        await Notification.requestPermission();
      } catch {
        // Ignore permission errors
      }
    }
  }

  /**
   * Play notification sound
   */
  private playSound(): void {
    if (!this.config.soundUrl) return;

    try {
      const audio = new Audio(this.config.soundUrl);
      audio.volume = 0.5;
      audio.play().catch(() => {
        // Audio play failed (autoplay blocked)
      });
    } catch {
      // Audio not supported
    }
  }

  /**
   * Check if currently in quiet hours
   */
  private isQuietHours(): boolean {
    if (
      this.config.quietHoursStart === undefined ||
      this.config.quietHoursEnd === undefined
    ) {
      return false;
    }

    const hour = new Date().getHours();
    const start = this.config.quietHoursStart;
    const end = this.config.quietHoursEnd;

    if (start <= end) {
      return hour >= start && hour < end;
    } else {
      // Spans midnight (e.g., 22:00 to 07:00)
      return hour >= start || hour < end;
    }
  }

  /**
   * Clean expired notifications
   */
  private cleanExpired(): void {
    const now = Date.now();
    let changed = false;

    for (const [id, notification] of this.queue) {
      if (notification.expiresAt && notification.expiresAt < now) {
        this.queue.delete(id);
        changed = true;
      }
    }

    if (changed) {
      this.saveToStorage();
    }
  }

  // ==================== STATUS UPDATES ====================

  /**
   * Mark notification as clicked
   */
  markClicked(id: string): void {
    const notification = this.queue.get(id);
    if (notification) {
      notification.clicked = true;
      this.saveToStorage();
    }
  }

  /**
   * Mark notification as dismissed
   */
  markDismissed(id: string): void {
    const notification = this.queue.get(id);
    if (notification) {
      notification.dismissed = true;
      this.saveToStorage();
    }
  }

  /**
   * Mark all as read
   */
  markAllRead(): void {
    for (const notification of this.queue.values()) {
      notification.clicked = true;
    }
    this.saveToStorage();
  }

  /**
   * Dismiss all
   */
  dismissAll(): void {
    for (const notification of this.queue.values()) {
      notification.dismissed = true;
    }
    this.saveToStorage();
  }

  /**
   * Clear all notifications
   */
  clear(): void {
    this.queue.clear();
    this.saveToStorage();
  }

  // ==================== HANDLERS ====================

  /**
   * Subscribe to notifications
   */
  subscribe(handler: NotificationHandler): () => void {
    this.handlers.add(handler);
    return () => this.handlers.delete(handler);
  }

  /**
   * Register action handler
   */
  onAction(action: string, handler: ActionHandler): () => void {
    this.actionHandlers.set(action, handler);
    return () => this.actionHandlers.delete(action);
  }

  /**
   * Handle notification action
   */
  handleAction(action: string, notificationId: string): void {
    const notification = this.queue.get(notificationId);
    if (!notification) return;

    const handler = this.actionHandlers.get(action);
    if (handler) {
      handler(action, notification);
    }

    this.markClicked(notificationId);
  }

  // ==================== PERSISTENCE ====================

  /**
   * Save queue to storage
   */
  private saveToStorage(): void {
    if (!this.config.persist) return;

    try {
      const data = Array.from(this.queue.values());
      localStorage.setItem(this.STORAGE_KEY, JSON.stringify(data));
    } catch {
      // Storage full or unavailable
    }
  }

  /**
   * Load queue from storage
   */
  private loadFromStorage(): void {
    if (!this.config.persist) return;

    try {
      const data = localStorage.getItem(this.STORAGE_KEY);
      if (data) {
        const notifications: QueuedNotification[] = JSON.parse(data);
        for (const notification of notifications) {
          this.queue.set(notification.id, notification);
        }
      }
    } catch {
      // Storage unavailable or corrupted
    }
  }

  // ==================== STATISTICS ====================

  /**
   * Get notification statistics
   */
  getStats(): NotificationStats {
    const all = Array.from(this.queue.values());

    const byType: Record<NotificationType, number> = {
      email_received: 0,
      email_sent: 0,
      read_receipt: 0,
      delivery_failed: 0,
      trust_change: 0,
      trust_warning: 0,
      sync_complete: 0,
      sync_conflict: 0,
      system: 0,
      mention: 0,
      reminder: 0,
    };

    const byPriority: Record<NotificationPriority, number> = {
      urgent: 0,
      high: 0,
      normal: 0,
      low: 0,
    };

    for (const n of all) {
      byType[n.type]++;
      byPriority[n.priority]++;
    }

    return {
      total: all.length,
      delivered: all.filter((n) => n.delivered).length,
      clicked: all.filter((n) => n.clicked).length,
      dismissed: all.filter((n) => n.dismissed).length,
      pending: all.filter((n) => !n.delivered && !n.dismissed).length,
      byType,
      byPriority,
    };
  }

  // ==================== CONFIGURATION ====================

  /**
   * Update configuration
   */
  configure(config: Partial<NotificationConfig>): void {
    this.config = { ...this.config, ...config };
  }

  /**
   * Get current configuration
   */
  getConfig(): NotificationConfig {
    return { ...this.config };
  }

  /**
   * Set quiet hours
   */
  setQuietHours(start: number | undefined, end: number | undefined): void {
    this.config.quietHoursStart = start;
    this.config.quietHoursEnd = end;
  }

  // ==================== UTILITIES ====================

  private generateId(): string {
    return `notif_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  private comparePriority(a: QueuedNotification, b: QueuedNotification): number {
    const aPriority = this.PRIORITY_ORDER.indexOf(a.priority);
    const bPriority = this.PRIORITY_ORDER.indexOf(b.priority);

    if (aPriority !== bPriority) {
      return aPriority - bPriority;
    }

    // Same priority - sort by timestamp (newest first)
    return b.timestamp - a.timestamp;
  }
}

// ==================== CONVENIENCE FUNCTIONS ====================

let defaultQueue: NotificationQueueService | null = null;

/**
 * Get or create default notification queue
 */
export function getNotificationQueue(config?: Partial<NotificationConfig>): NotificationQueueService {
  if (!defaultQueue) {
    defaultQueue = new NotificationQueueService(config);
  }
  return defaultQueue;
}

/**
 * Create notification for new email
 */
export function notifyNewEmail(
  queue: NotificationQueueService,
  sender: string,
  subject: string,
  emailHash: ActionHash
): string {
  return queue.enqueue({
    type: 'email_received',
    priority: 'normal',
    title: `New email from ${sender}`,
    body: subject,
    groupKey: 'new_emails',
    dedupeKey: `email_${emailHash}`,
    data: { emailHash, sender },
    actions: [
      { action: 'read', title: 'Read' },
      { action: 'archive', title: 'Archive' },
    ],
  });
}

/**
 * Create notification for read receipt
 */
export function notifyReadReceipt(
  queue: NotificationQueueService,
  recipient: string,
  subject: string,
  emailHash: ActionHash
): string {
  return queue.enqueue({
    type: 'read_receipt',
    priority: 'low',
    title: `${recipient} read your email`,
    body: subject,
    groupKey: 'read_receipts',
    data: { emailHash },
  });
}

/**
 * Create notification for delivery failure
 */
export function notifyDeliveryFailed(
  queue: NotificationQueueService,
  recipient: string,
  subject: string,
  reason: string,
  emailHash: ActionHash
): string {
  return queue.enqueue({
    type: 'delivery_failed',
    priority: 'high',
    title: `Delivery failed to ${recipient}`,
    body: `${subject}\nReason: ${reason}`,
    dedupeKey: `failed_${emailHash}`,
    data: { emailHash, reason },
    actions: [{ action: 'retry', title: 'Retry' }],
  });
}

/**
 * Create notification for trust warning
 */
export function notifyTrustWarning(
  queue: NotificationQueueService,
  agent: AgentPubKey,
  warningType: string,
  details: string
): string {
  return queue.enqueue({
    type: 'trust_warning',
    priority: 'urgent',
    title: 'Trust Warning',
    body: `${warningType}: ${details}`,
    dedupeKey: `trust_warning_${agent}_${warningType}`,
    data: { agent, warningType },
  });
}

/**
 * Create notification for sync conflict
 */
export function notifySyncConflict(
  queue: NotificationQueueService,
  conflictType: string,
  details: string
): string {
  return queue.enqueue({
    type: 'sync_conflict',
    priority: 'high',
    title: 'Sync Conflict Detected',
    body: `${conflictType}: ${details}`,
    groupKey: 'sync_conflicts',
    data: { conflictType },
    actions: [{ action: 'resolve', title: 'Resolve' }],
  });
}

export default NotificationQueueService;
