// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Email Scheduler Service for Mycelix Mail
 *
 * Schedule emails for later:
 * - One-time scheduled sends
 * - Recurring emails
 * - Timezone-aware scheduling
 * - Smart send time optimization
 * - Snooze/remind later
 */

import type { ActionHash } from '@holochain/client';
import type { MycelixMailClient } from '../index';

// ==================== TYPES ====================

export interface ScheduledEmail {
  id: string;
  hash?: ActionHash;
  draftHash: ActionHash;
  scheduledFor: number;
  timezone: string;
  recurrence?: RecurrenceRule;
  status: ScheduleStatus;
  retries: number;
  lastError?: string;
  createdAt: number;
  updatedAt: number;
  metadata: ScheduleMetadata;
}

export type ScheduleStatus =
  | 'pending'
  | 'processing'
  | 'sent'
  | 'failed'
  | 'cancelled'
  | 'paused';

export interface RecurrenceRule {
  frequency: 'daily' | 'weekly' | 'monthly' | 'yearly';
  interval: number;
  daysOfWeek?: number[]; // 0=Sunday, 6=Saturday
  dayOfMonth?: number;
  monthOfYear?: number;
  endDate?: number;
  maxOccurrences?: number;
  occurrenceCount: number;
}

export interface ScheduleMetadata {
  subject?: string;
  recipientCount?: number;
  smartSendOptimized?: boolean;
  originalScheduledFor?: number;
}

export interface SnoozeOptions {
  duration?: number; // ms
  until?: number; // timestamp
  nextMorning?: boolean;
  nextWeek?: boolean;
}

export interface SmartSendConfig {
  enabled: boolean;
  workHoursStart: number; // 0-23
  workHoursEnd: number;
  workDays: number[]; // 0=Sunday
  timezone: string;
  recipientAnalysis: boolean;
}

export interface SchedulerConfig {
  checkInterval?: number;
  maxRetries?: number;
  retryDelay?: number;
  defaultTimezone?: string;
  smartSend?: SmartSendConfig;
}

export interface SchedulerStats {
  pending: number;
  sent: number;
  failed: number;
  cancelled: number;
  recurringActive: number;
  nextScheduled?: ScheduledEmail;
}

export type ScheduleHandler = (scheduled: ScheduledEmail, result: 'sent' | 'failed') => void;

// ==================== SCHEDULER SERVICE ====================

export class SchedulerService {
  private schedules: Map<string, ScheduledEmail> = new Map();
  private handlers: Set<ScheduleHandler> = new Set();
  private checkInterval: NodeJS.Timeout | null = null;
  private config: SchedulerConfig;

  private readonly DEFAULT_CONFIG: SchedulerConfig = {
    checkInterval: 60000, // 1 minute
    maxRetries: 3,
    retryDelay: 5 * 60 * 1000, // 5 minutes
    defaultTimezone: Intl.DateTimeFormat().resolvedOptions().timeZone,
    smartSend: {
      enabled: false,
      workHoursStart: 9,
      workHoursEnd: 17,
      workDays: [1, 2, 3, 4, 5], // Mon-Fri
      timezone: Intl.DateTimeFormat().resolvedOptions().timeZone,
      recipientAnalysis: false,
    },
  };

  constructor(
    private client?: MycelixMailClient,
    config: Partial<SchedulerConfig> = {}
  ) {
    this.config = { ...this.DEFAULT_CONFIG, ...config };
    this.startChecking();
  }

  // ==================== SCHEDULING ====================

  /**
   * Schedule an email for later
   */
  async scheduleEmail(
    draftHash: ActionHash,
    scheduledFor: Date | number,
    options: {
      timezone?: string;
      recurrence?: Omit<RecurrenceRule, 'occurrenceCount'>;
      useSmartSend?: boolean;
      metadata?: Partial<ScheduleMetadata>;
    } = {}
  ): Promise<ScheduledEmail> {
    const id = this.generateId();
    const now = Date.now();
    let targetTime = typeof scheduledFor === 'number' ? scheduledFor : scheduledFor.getTime();

    // Apply smart send optimization
    if (options.useSmartSend && this.config.smartSend?.enabled) {
      targetTime = this.optimizeSendTime(targetTime);
    }

    const scheduled: ScheduledEmail = {
      id,
      draftHash,
      scheduledFor: targetTime,
      timezone: options.timezone ?? this.config.defaultTimezone!,
      recurrence: options.recurrence
        ? { ...options.recurrence, occurrenceCount: 0 }
        : undefined,
      status: 'pending',
      retries: 0,
      createdAt: now,
      updatedAt: now,
      metadata: {
        ...options.metadata,
        originalScheduledFor: targetTime,
        smartSendOptimized: options.useSmartSend && this.config.smartSend?.enabled,
      },
    };

    // Store in Holochain
    if (this.client) {
      scheduled.hash = await this.client.callZome('scheduler', 'schedule_email', scheduled);
    }

    this.schedules.set(id, scheduled);
    return scheduled;
  }

  /**
   * Schedule email for smart send time
   */
  async scheduleSmartSend(
    draftHash: ActionHash,
    metadata?: Partial<ScheduleMetadata>
  ): Promise<ScheduledEmail> {
    const optimalTime = this.calculateOptimalSendTime();
    return this.scheduleEmail(draftHash, optimalTime, {
      useSmartSend: true,
      metadata,
    });
  }

  /**
   * Reschedule an email
   */
  async reschedule(
    scheduleId: string,
    newTime: Date | number
  ): Promise<ScheduledEmail | null> {
    const scheduled = this.schedules.get(scheduleId);
    if (!scheduled || scheduled.status === 'sent') {
      return null;
    }

    scheduled.scheduledFor = typeof newTime === 'number' ? newTime : newTime.getTime();
    scheduled.status = 'pending';
    scheduled.updatedAt = Date.now();

    if (this.client && scheduled.hash) {
      await this.client.callZome('scheduler', 'update_schedule', scheduled);
    }

    return scheduled;
  }

  /**
   * Cancel a scheduled email
   */
  async cancelSchedule(scheduleId: string): Promise<boolean> {
    const scheduled = this.schedules.get(scheduleId);
    if (!scheduled || scheduled.status === 'sent') {
      return false;
    }

    scheduled.status = 'cancelled';
    scheduled.updatedAt = Date.now();

    if (this.client && scheduled.hash) {
      await this.client.callZome('scheduler', 'cancel_schedule', scheduled.hash);
    }

    return true;
  }

  /**
   * Pause a recurring schedule
   */
  async pauseSchedule(scheduleId: string): Promise<boolean> {
    const scheduled = this.schedules.get(scheduleId);
    if (!scheduled || !scheduled.recurrence || scheduled.status !== 'pending') {
      return false;
    }

    scheduled.status = 'paused';
    scheduled.updatedAt = Date.now();

    if (this.client && scheduled.hash) {
      await this.client.callZome('scheduler', 'update_schedule', scheduled);
    }

    return true;
  }

  /**
   * Resume a paused schedule
   */
  async resumeSchedule(scheduleId: string): Promise<boolean> {
    const scheduled = this.schedules.get(scheduleId);
    if (!scheduled || scheduled.status !== 'paused') {
      return false;
    }

    scheduled.status = 'pending';
    scheduled.updatedAt = Date.now();

    // Calculate next occurrence
    if (scheduled.recurrence) {
      scheduled.scheduledFor = this.calculateNextOccurrence(
        Date.now(),
        scheduled.recurrence
      );
    }

    if (this.client && scheduled.hash) {
      await this.client.callZome('scheduler', 'update_schedule', scheduled);
    }

    return true;
  }

  // ==================== SNOOZE ====================

  /**
   * Snooze an email (remind later)
   */
  async snoozeEmail(
    emailHash: ActionHash,
    options: SnoozeOptions
  ): Promise<ScheduledEmail> {
    let snoozeUntil: number;

    if (options.until) {
      snoozeUntil = options.until;
    } else if (options.duration) {
      snoozeUntil = Date.now() + options.duration;
    } else if (options.nextMorning) {
      snoozeUntil = this.getNextMorning();
    } else if (options.nextWeek) {
      snoozeUntil = this.getNextWeekday(1); // Monday
    } else {
      // Default: 3 hours
      snoozeUntil = Date.now() + 3 * 60 * 60 * 1000;
    }

    return this.scheduleEmail(emailHash, snoozeUntil, {
      metadata: { subject: 'Snoozed reminder' },
    });
  }

  /**
   * Get next morning (9 AM)
   */
  private getNextMorning(): number {
    const now = new Date();
    const tomorrow = new Date(now);
    tomorrow.setDate(tomorrow.getDate() + 1);
    tomorrow.setHours(9, 0, 0, 0);
    return tomorrow.getTime();
  }

  /**
   * Get next specified weekday
   */
  private getNextWeekday(targetDay: number): number {
    const now = new Date();
    const current = now.getDay();
    const daysUntil = (targetDay - current + 7) % 7 || 7;

    const target = new Date(now);
    target.setDate(target.getDate() + daysUntil);
    target.setHours(9, 0, 0, 0);

    return target.getTime();
  }

  // ==================== RECURRENCE ====================

  /**
   * Calculate next occurrence for recurring schedule
   */
  private calculateNextOccurrence(
    fromDate: number,
    rule: RecurrenceRule
  ): number {
    const date = new Date(fromDate);

    switch (rule.frequency) {
      case 'daily':
        date.setDate(date.getDate() + rule.interval);
        break;

      case 'weekly':
        if (rule.daysOfWeek?.length) {
          // Find next matching day
          let found = false;
          for (let i = 1; i <= 7; i++) {
            const checkDate = new Date(date);
            checkDate.setDate(checkDate.getDate() + i);
            if (rule.daysOfWeek.includes(checkDate.getDay())) {
              date.setTime(checkDate.getTime());
              found = true;
              break;
            }
          }
          if (!found) {
            date.setDate(date.getDate() + 7 * rule.interval);
          }
        } else {
          date.setDate(date.getDate() + 7 * rule.interval);
        }
        break;

      case 'monthly':
        date.setMonth(date.getMonth() + rule.interval);
        if (rule.dayOfMonth) {
          date.setDate(Math.min(rule.dayOfMonth, this.getDaysInMonth(date)));
        }
        break;

      case 'yearly':
        date.setFullYear(date.getFullYear() + rule.interval);
        if (rule.monthOfYear !== undefined) {
          date.setMonth(rule.monthOfYear);
        }
        if (rule.dayOfMonth) {
          date.setDate(Math.min(rule.dayOfMonth, this.getDaysInMonth(date)));
        }
        break;
    }

    return date.getTime();
  }

  /**
   * Get days in month
   */
  private getDaysInMonth(date: Date): number {
    return new Date(date.getFullYear(), date.getMonth() + 1, 0).getDate();
  }

  /**
   * Check if recurrence should continue
   */
  private shouldContinueRecurrence(scheduled: ScheduledEmail): boolean {
    if (!scheduled.recurrence) return false;

    const rule = scheduled.recurrence;

    // Check end date
    if (rule.endDate && Date.now() > rule.endDate) {
      return false;
    }

    // Check max occurrences
    if (rule.maxOccurrences && rule.occurrenceCount >= rule.maxOccurrences) {
      return false;
    }

    return true;
  }

  // ==================== SMART SEND ====================

  /**
   * Calculate optimal send time
   */
  private calculateOptimalSendTime(): number {
    const config = this.config.smartSend!;
    const now = new Date();

    // Check if current time is within work hours
    if (this.isWithinWorkHours(now)) {
      // Send immediately
      return now.getTime();
    }

    // Find next work hours
    return this.getNextWorkHoursStart();
  }

  /**
   * Optimize send time based on configuration
   */
  private optimizeSendTime(originalTime: number): number {
    const config = this.config.smartSend!;
    const date = new Date(originalTime);

    // If already within work hours on a work day, keep it
    if (this.isWithinWorkHours(date) && this.isWorkDay(date)) {
      return originalTime;
    }

    // Move to next work hours
    while (!this.isWithinWorkHours(date) || !this.isWorkDay(date)) {
      if (!this.isWorkDay(date)) {
        // Move to next work day
        date.setDate(date.getDate() + 1);
        date.setHours(config.workHoursStart, 0, 0, 0);
      } else if (date.getHours() < config.workHoursStart) {
        // Move to start of work hours
        date.setHours(config.workHoursStart, 0, 0, 0);
      } else {
        // Move to next day's work hours
        date.setDate(date.getDate() + 1);
        date.setHours(config.workHoursStart, 0, 0, 0);
      }
    }

    return date.getTime();
  }

  /**
   * Check if time is within work hours
   */
  private isWithinWorkHours(date: Date): boolean {
    const config = this.config.smartSend!;
    const hour = date.getHours();
    return hour >= config.workHoursStart && hour < config.workHoursEnd;
  }

  /**
   * Check if date is a work day
   */
  private isWorkDay(date: Date): boolean {
    const config = this.config.smartSend!;
    return config.workDays.includes(date.getDay());
  }

  /**
   * Get next work hours start time
   */
  private getNextWorkHoursStart(): number {
    const config = this.config.smartSend!;
    const now = new Date();

    // Start from tomorrow
    const date = new Date(now);
    date.setDate(date.getDate() + 1);
    date.setHours(config.workHoursStart, 0, 0, 0);

    // Find next work day
    while (!config.workDays.includes(date.getDay())) {
      date.setDate(date.getDate() + 1);
    }

    return date.getTime();
  }

  // ==================== PROCESSING ====================

  /**
   * Start checking for due schedules
   */
  private startChecking(): void {
    if (this.checkInterval) return;

    this.checkInterval = setInterval(() => {
      this.processDueSchedules();
    }, this.config.checkInterval!);

    // Initial check
    this.processDueSchedules();
  }

  /**
   * Stop checking
   */
  stop(): void {
    if (this.checkInterval) {
      clearInterval(this.checkInterval);
      this.checkInterval = null;
    }
  }

  /**
   * Process due schedules
   */
  private async processDueSchedules(): Promise<void> {
    const now = Date.now();

    for (const scheduled of this.schedules.values()) {
      if (scheduled.status !== 'pending') continue;
      if (scheduled.scheduledFor > now) continue;

      await this.processSchedule(scheduled);
    }
  }

  /**
   * Process a single schedule
   */
  private async processSchedule(scheduled: ScheduledEmail): Promise<void> {
    scheduled.status = 'processing';
    scheduled.updatedAt = Date.now();

    try {
      // Send the email
      if (this.client) {
        await this.client.messages.sendDraft(scheduled.draftHash);
      }

      // Handle recurrence
      if (this.shouldContinueRecurrence(scheduled)) {
        scheduled.recurrence!.occurrenceCount++;
        scheduled.scheduledFor = this.calculateNextOccurrence(
          Date.now(),
          scheduled.recurrence!
        );
        scheduled.status = 'pending';
      } else {
        scheduled.status = 'sent';
      }

      this.notifyHandlers(scheduled, 'sent');

    } catch (error) {
      scheduled.retries++;
      scheduled.lastError = String(error);

      if (scheduled.retries >= this.config.maxRetries!) {
        scheduled.status = 'failed';
        this.notifyHandlers(scheduled, 'failed');
      } else {
        // Schedule retry
        scheduled.scheduledFor = Date.now() + this.config.retryDelay!;
        scheduled.status = 'pending';
      }
    }

    scheduled.updatedAt = Date.now();

    // Update in Holochain
    if (this.client && scheduled.hash) {
      await this.client.callZome('scheduler', 'update_schedule', scheduled);
    }
  }

  /**
   * Notify handlers of schedule result
   */
  private notifyHandlers(scheduled: ScheduledEmail, result: 'sent' | 'failed'): void {
    for (const handler of this.handlers) {
      try {
        handler(scheduled, result);
      } catch (e) {
        console.error('Schedule handler error:', e);
      }
    }
  }

  // ==================== QUERIES ====================

  /**
   * Get scheduled email by ID
   */
  getSchedule(id: string): ScheduledEmail | null {
    return this.schedules.get(id) ?? null;
  }

  /**
   * Get all scheduled emails
   */
  getAllSchedules(): ScheduledEmail[] {
    return Array.from(this.schedules.values());
  }

  /**
   * Get pending schedules
   */
  getPending(): ScheduledEmail[] {
    return this.getAllSchedules()
      .filter((s) => s.status === 'pending')
      .sort((a, b) => a.scheduledFor - b.scheduledFor);
  }

  /**
   * Get schedules due soon
   */
  getDueSoon(withinMs = 60 * 60 * 1000): ScheduledEmail[] {
    const threshold = Date.now() + withinMs;
    return this.getPending().filter((s) => s.scheduledFor <= threshold);
  }

  /**
   * Get recurring schedules
   */
  getRecurring(): ScheduledEmail[] {
    return this.getAllSchedules().filter((s) => s.recurrence);
  }

  // ==================== HANDLERS ====================

  /**
   * Subscribe to schedule events
   */
  onScheduleComplete(handler: ScheduleHandler): () => void {
    this.handlers.add(handler);
    return () => this.handlers.delete(handler);
  }

  // ==================== STATISTICS ====================

  /**
   * Get scheduler statistics
   */
  getStats(): SchedulerStats {
    const all = this.getAllSchedules();
    const pending = this.getPending();

    return {
      pending: pending.length,
      sent: all.filter((s) => s.status === 'sent').length,
      failed: all.filter((s) => s.status === 'failed').length,
      cancelled: all.filter((s) => s.status === 'cancelled').length,
      recurringActive: all.filter((s) => s.recurrence && s.status === 'pending').length,
      nextScheduled: pending[0],
    };
  }

  // ==================== CONFIGURATION ====================

  /**
   * Update smart send configuration
   */
  configureSmartSend(config: Partial<SmartSendConfig>): void {
    this.config.smartSend = { ...this.config.smartSend!, ...config };
  }

  /**
   * Get current configuration
   */
  getConfig(): SchedulerConfig {
    return { ...this.config };
  }

  // ==================== UTILITIES ====================

  private generateId(): string {
    return `sched_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * Load schedules from Holochain
   */
  async loadFromHolochain(): Promise<void> {
    if (!this.client) return;

    try {
      const schedules: ScheduledEmail[] = await this.client.callZome(
        'scheduler',
        'get_all_schedules',
        null
      );

      for (const schedule of schedules) {
        this.schedules.set(schedule.id, schedule);
      }
    } catch (e) {
      console.error('Failed to load schedules:', e);
    }
  }
}

/**
 * Create scheduler service
 */
export function createSchedulerService(
  client?: MycelixMailClient,
  config?: Partial<SchedulerConfig>
): SchedulerService {
  return new SchedulerService(client, config);
}

export default SchedulerService;
