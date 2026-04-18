// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Service exports for Mycelix Mail Holochain Client
 */

export { CryptoService, cryptoService } from './CryptoService';
export type {
  CryptoSuite,
  SignatureAlgorithm,
  KeyPair,
  EncryptedData,
  SignedData,
  DecryptResult,
  CryptoConfig,
} from './CryptoService';

export { CacheService, cacheService } from './CacheService';
export type {
  CacheConfig,
  CacheEntry,
  CacheStats,
} from './CacheService';

export { HealthService } from './HealthService';
export type {
  HealthStatus,
  ComponentHealth,
  HealthMetrics,
  HealthAlert,
  HealthCheckResult,
  HealthEventHandler,
} from './HealthService';

export { BatchService, createBatchService } from './BatchService';
export type {
  BatchOptions,
  BatchProgress,
  BatchResult,
  BatchError,
  BatchOperation,
} from './BatchService';

export { SpamFilterService, createSpamFilter } from './SpamFilterService';
export type {
  SpamVerdict,
  SpamCategory,
  SpamAction,
  SpamFeatures,
  SpamRule,
  SpamFilterConfig,
} from './SpamFilterService';

export { ImportExportService, createImportExportService } from './ImportExportService';
export type {
  ImportOptions,
  ExportOptions,
  ImportProgress,
  ExportProgress,
  ImportResult,
  ExportResult,
  ImportError,
  ParsedEmail,
  ParsedAttachment,
} from './ImportExportService';

export {
  NotificationQueueService,
  getNotificationQueue,
  notifyNewEmail,
  notifyReadReceipt,
  notifyDeliveryFailed,
  notifyTrustWarning,
  notifySyncConflict,
} from './NotificationQueueService';
export type {
  NotificationPriority,
  NotificationType,
  QueuedNotification,
  NotificationData,
  NotificationAction,
  NotificationConfig,
  NotificationStats,
  NotificationHandler,
  ActionHandler,
} from './NotificationQueueService';

export { OfflineQueueService, createOfflineQueue } from './OfflineQueueService';
export type {
  OperationType,
  QueuedOperation,
  OperationStatus,
  OperationPayload,
  OfflineQueueConfig,
  QueueStats,
  OperationHandler,
  StatusChangeHandler,
} from './OfflineQueueService';

export {
  EmailStateMachine,
  createEmailStateMachine,
  getEmailStateMachine,
} from './EmailStateMachine';
export type {
  EmailState,
  EmailEvent,
  StateTransition,
  EmailContext,
  StateChangeEvent,
  UndoAction,
  StateChangeHandler,
} from './EmailStateMachine';

export { MetricsService, createMetricsService, getMetrics } from './MetricsService';
export type {
  MetricType,
  Metric,
  HistogramBucket,
  HistogramMetric,
  TimerMetric,
  MetricsSnapshot,
  MetricsConfig,
  MetricHandler,
} from './MetricsService';

export { KeyExchangeService, createKeyExchangeService } from './KeyExchangeService';
export type {
  KeyPair,
  IdentityKeyPair,
  SignedPreKey,
  OneTimePreKey,
  PreKeyBundle,
  SessionState,
  EncryptedMessage,
  MessageHeader,
  KeyExchangeConfig,
  KeyExchangeStats,
} from './KeyExchangeService';

export { ContactService, createContactService } from './ContactService';
export type {
  Contact,
  ContactEmail,
  ContactPhone,
  ContactAddress,
  ContactMetadata,
  ContactGroup,
  ContactSuggestion,
  SuggestionReason,
  ContactFilter,
  ContactStats,
} from './ContactService';

export { SchedulerService, createSchedulerService } from './SchedulerService';
export type {
  ScheduledEmail,
  ScheduleStatus,
  RecurrenceRule,
  ScheduleMetadata,
  SnoozeOptions,
  SmartSendConfig,
  SchedulerConfig,
  SchedulerStats,
  ScheduleHandler,
} from './SchedulerService';

export { ThreadService, createThreadService } from './ThreadService';
export type {
  EmailThread,
  ThreadParticipant,
  ThreadMessage,
  ThreadFilter,
  ThreadSortOptions,
  ThreadStats,
} from './ThreadService';

export { AuditLogService, createAuditLogService, getAuditLog } from './AuditLogService';
export type {
  AuditCategory,
  AuditAction,
  AuditSeverity,
  AuditEntry,
  AuditActor,
  AuditTarget,
  AuditDetails,
  AuditMetadata,
  AuditFilter,
  AuditReport,
  AuditSummary,
  AuditConfig,
  AuditHandler,
} from './AuditLogService';
