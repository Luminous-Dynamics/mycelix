// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Offline Queue Service for Mycelix Mail
 *
 * Handles operations when offline:
 * - Queue operations during network outage
 * - Persist to IndexedDB
 * - Auto-sync when back online
 * - Conflict resolution
 * - Retry with exponential backoff
 */

import type { ActionHash, AgentPubKey } from '@holochain/client';
import type { MycelixMailClient } from '../index';

// ==================== TYPES ====================

export type OperationType =
  | 'send_email'
  | 'mark_read'
  | 'mark_unread'
  | 'archive'
  | 'delete'
  | 'move_to_folder'
  | 'add_label'
  | 'remove_label'
  | 'star'
  | 'unstar'
  | 'create_draft'
  | 'update_draft'
  | 'create_folder'
  | 'delete_folder'
  | 'create_attestation'
  | 'revoke_attestation';

export interface QueuedOperation {
  id: string;
  type: OperationType;
  payload: OperationPayload;
  timestamp: number;
  retries: number;
  maxRetries: number;
  lastError?: string;
  status: OperationStatus;
  priority: number;
  dependsOn?: string[];
  createdAt: number;
  updatedAt: number;
}

export type OperationStatus = 'pending' | 'processing' | 'completed' | 'failed' | 'cancelled';

export interface OperationPayload {
  emailHash?: ActionHash;
  email?: {
    to: string[];
    cc?: string[];
    bcc?: string[];
    subject: string;
    body: string;
    html_body?: string;
    attachments?: unknown[];
  };
  folderHash?: ActionHash;
  label?: string;
  draftId?: string;
  draftContent?: unknown;
  attestation?: {
    subject: AgentPubKey;
    category: string;
    level: string;
    confidence: number;
  };
  [key: string]: unknown;
}

export interface OfflineQueueConfig {
  /** Maximum operations in queue */
  maxQueueSize?: number;
  /** Default max retries */
  defaultMaxRetries?: number;
  /** Base delay for exponential backoff (ms) */
  baseRetryDelay?: number;
  /** Max retry delay (ms) */
  maxRetryDelay?: number;
  /** Auto-sync when online */
  autoSync?: boolean;
  /** Sync interval when online (ms) */
  syncInterval?: number;
  /** Enable persistence */
  persist?: boolean;
}

export interface QueueStats {
  total: number;
  pending: number;
  processing: number;
  completed: number;
  failed: number;
  cancelled: number;
  oldestPending?: number;
  averageRetries: number;
}

export type OperationHandler = (
  operation: QueuedOperation
) => Promise<void>;

export type StatusChangeHandler = (
  operation: QueuedOperation,
  oldStatus: OperationStatus
) => void;

// ==================== OFFLINE QUEUE SERVICE ====================

export class OfflineQueueService {
  private queue: Map<string, QueuedOperation> = new Map();
  private handlers: Map<OperationType, OperationHandler> = new Map();
  private statusHandlers: Set<StatusChangeHandler> = new Set();
  private config: OfflineQueueConfig;
  private processing = false;
  private syncInterval: NodeJS.Timeout | null = null;
  private isOnline = navigator.onLine;
  private db: IDBDatabase | null = null;

  private readonly DB_NAME = 'mycelix_offline_queue';
  private readonly STORE_NAME = 'operations';

  constructor(
    private client: MycelixMailClient,
    config: Partial<OfflineQueueConfig> = {}
  ) {
    this.config = {
      maxQueueSize: 1000,
      defaultMaxRetries: 5,
      baseRetryDelay: 1000,
      maxRetryDelay: 60000,
      autoSync: true,
      syncInterval: 30000,
      persist: true,
      ...config,
    };

    this.registerDefaultHandlers();
    this.setupNetworkListeners();
    this.initializeDB();
  }

  // ==================== INITIALIZATION ====================

  /**
   * Initialize IndexedDB for persistence
   */
  private async initializeDB(): Promise<void> {
    if (!this.config.persist) return;

    return new Promise((resolve, reject) => {
      const request = indexedDB.open(this.DB_NAME, 1);

      request.onerror = () => reject(request.error);

      request.onsuccess = () => {
        this.db = request.result;
        this.loadFromDB().then(resolve).catch(reject);
      };

      request.onupgradeneeded = (event) => {
        const db = (event.target as IDBOpenDBRequest).result;
        if (!db.objectStoreNames.contains(this.STORE_NAME)) {
          const store = db.createObjectStore(this.STORE_NAME, { keyPath: 'id' });
          store.createIndex('status', 'status', { unique: false });
          store.createIndex('type', 'type', { unique: false });
          store.createIndex('timestamp', 'timestamp', { unique: false });
        }
      };
    });
  }

  /**
   * Load operations from IndexedDB
   */
  private async loadFromDB(): Promise<void> {
    if (!this.db) return;

    return new Promise((resolve, reject) => {
      const transaction = this.db!.transaction(this.STORE_NAME, 'readonly');
      const store = transaction.objectStore(this.STORE_NAME);
      const request = store.getAll();

      request.onerror = () => reject(request.error);
      request.onsuccess = () => {
        const operations: QueuedOperation[] = request.result;
        for (const op of operations) {
          // Resume pending operations
          if (op.status === 'processing') {
            op.status = 'pending';
          }
          this.queue.set(op.id, op);
        }
        resolve();
      };
    });
  }

  /**
   * Save operation to IndexedDB
   */
  private async saveToDB(operation: QueuedOperation): Promise<void> {
    if (!this.db) return;

    return new Promise((resolve, reject) => {
      const transaction = this.db!.transaction(this.STORE_NAME, 'readwrite');
      const store = transaction.objectStore(this.STORE_NAME);
      const request = store.put(operation);

      request.onerror = () => reject(request.error);
      request.onsuccess = () => resolve();
    });
  }

  /**
   * Delete operation from IndexedDB
   */
  private async deleteFromDB(id: string): Promise<void> {
    if (!this.db) return;

    return new Promise((resolve, reject) => {
      const transaction = this.db!.transaction(this.STORE_NAME, 'readwrite');
      const store = transaction.objectStore(this.STORE_NAME);
      const request = store.delete(id);

      request.onerror = () => reject(request.error);
      request.onsuccess = () => resolve();
    });
  }

  /**
   * Setup network event listeners
   */
  private setupNetworkListeners(): void {
    window.addEventListener('online', () => {
      this.isOnline = true;
      if (this.config.autoSync) {
        this.processQueue();
        this.startSyncInterval();
      }
    });

    window.addEventListener('offline', () => {
      this.isOnline = false;
      this.stopSyncInterval();
    });

    // Start sync if already online
    if (this.isOnline && this.config.autoSync) {
      this.startSyncInterval();
    }
  }

  /**
   * Start periodic sync
   */
  private startSyncInterval(): void {
    if (this.syncInterval) return;

    this.syncInterval = setInterval(() => {
      if (this.isOnline && !this.processing) {
        this.processQueue();
      }
    }, this.config.syncInterval!);
  }

  /**
   * Stop periodic sync
   */
  private stopSyncInterval(): void {
    if (this.syncInterval) {
      clearInterval(this.syncInterval);
      this.syncInterval = null;
    }
  }

  // ==================== QUEUE MANAGEMENT ====================

  /**
   * Add operation to queue
   */
  async enqueue(
    type: OperationType,
    payload: OperationPayload,
    options: {
      priority?: number;
      maxRetries?: number;
      dependsOn?: string[];
    } = {}
  ): Promise<string> {
    if (this.queue.size >= this.config.maxQueueSize!) {
      // Remove oldest completed/failed operations
      this.cleanup();

      if (this.queue.size >= this.config.maxQueueSize!) {
        throw new Error('Queue is full');
      }
    }

    const now = Date.now();
    const operation: QueuedOperation = {
      id: this.generateId(),
      type,
      payload,
      timestamp: now,
      retries: 0,
      maxRetries: options.maxRetries ?? this.config.defaultMaxRetries!,
      status: 'pending',
      priority: options.priority ?? 5,
      dependsOn: options.dependsOn,
      createdAt: now,
      updatedAt: now,
    };

    this.queue.set(operation.id, operation);
    await this.saveToDB(operation);

    // Process immediately if online
    if (this.isOnline && !this.processing) {
      this.processQueue();
    }

    return operation.id;
  }

  /**
   * Get operation by ID
   */
  get(id: string): QueuedOperation | undefined {
    return this.queue.get(id);
  }

  /**
   * Get all operations
   */
  getAll(): QueuedOperation[] {
    return Array.from(this.queue.values());
  }

  /**
   * Get pending operations
   */
  getPending(): QueuedOperation[] {
    return this.getAll()
      .filter((op) => op.status === 'pending')
      .sort((a, b) => {
        // Sort by priority (lower is higher priority), then timestamp
        if (a.priority !== b.priority) {
          return a.priority - b.priority;
        }
        return a.timestamp - b.timestamp;
      });
  }

  /**
   * Cancel operation
   */
  async cancel(id: string): Promise<boolean> {
    const operation = this.queue.get(id);
    if (!operation || operation.status === 'processing') {
      return false;
    }

    const oldStatus = operation.status;
    operation.status = 'cancelled';
    operation.updatedAt = Date.now();

    await this.saveToDB(operation);
    this.notifyStatusChange(operation, oldStatus);

    return true;
  }

  /**
   * Retry failed operation
   */
  async retry(id: string): Promise<boolean> {
    const operation = this.queue.get(id);
    if (!operation || operation.status !== 'failed') {
      return false;
    }

    const oldStatus = operation.status;
    operation.status = 'pending';
    operation.retries = 0;
    operation.lastError = undefined;
    operation.updatedAt = Date.now();

    await this.saveToDB(operation);
    this.notifyStatusChange(operation, oldStatus);

    if (this.isOnline && !this.processing) {
      this.processQueue();
    }

    return true;
  }

  /**
   * Remove completed operations
   */
  async cleanup(maxAge?: number): Promise<number> {
    const now = Date.now();
    const threshold = maxAge ?? 24 * 60 * 60 * 1000; // 24 hours
    let removed = 0;

    for (const [id, operation] of this.queue) {
      if (
        (operation.status === 'completed' || operation.status === 'cancelled') &&
        now - operation.updatedAt > threshold
      ) {
        this.queue.delete(id);
        await this.deleteFromDB(id);
        removed++;
      }
    }

    return removed;
  }

  // ==================== PROCESSING ====================

  /**
   * Process pending operations
   */
  async processQueue(): Promise<void> {
    if (this.processing || !this.isOnline) return;

    this.processing = true;

    try {
      const pending = this.getPending();

      for (const operation of pending) {
        // Check dependencies
        if (operation.dependsOn?.length) {
          const unmetDeps = operation.dependsOn.filter((depId) => {
            const dep = this.queue.get(depId);
            return dep && dep.status !== 'completed';
          });

          if (unmetDeps.length > 0) {
            continue; // Skip, dependencies not met
          }
        }

        await this.processOperation(operation);

        // Small delay between operations
        await this.delay(100);
      }
    } finally {
      this.processing = false;
    }
  }

  /**
   * Process single operation
   */
  private async processOperation(operation: QueuedOperation): Promise<void> {
    const handler = this.handlers.get(operation.type);
    if (!handler) {
      console.warn(`No handler for operation type: ${operation.type}`);
      return;
    }

    const oldStatus = operation.status;
    operation.status = 'processing';
    operation.updatedAt = Date.now();
    await this.saveToDB(operation);
    this.notifyStatusChange(operation, oldStatus);

    try {
      await handler(operation);

      operation.status = 'completed';
      operation.updatedAt = Date.now();
      await this.saveToDB(operation);
      this.notifyStatusChange(operation, 'processing');

    } catch (error) {
      operation.retries++;
      operation.lastError = String(error);
      operation.updatedAt = Date.now();

      if (operation.retries >= operation.maxRetries) {
        operation.status = 'failed';
      } else {
        operation.status = 'pending';
        // Schedule retry with exponential backoff
        const delay = this.calculateBackoff(operation.retries);
        setTimeout(() => {
          if (this.isOnline && !this.processing) {
            this.processQueue();
          }
        }, delay);
      }

      await this.saveToDB(operation);
      this.notifyStatusChange(operation, 'processing');
    }
  }

  /**
   * Calculate exponential backoff delay
   */
  private calculateBackoff(retries: number): number {
    const delay = this.config.baseRetryDelay! * Math.pow(2, retries);
    const jitter = Math.random() * 0.3 * delay;
    return Math.min(delay + jitter, this.config.maxRetryDelay!);
  }

  // ==================== HANDLERS ====================

  /**
   * Register operation handler
   */
  registerHandler(type: OperationType, handler: OperationHandler): void {
    this.handlers.set(type, handler);
  }

  /**
   * Subscribe to status changes
   */
  onStatusChange(handler: StatusChangeHandler): () => void {
    this.statusHandlers.add(handler);
    return () => this.statusHandlers.delete(handler);
  }

  /**
   * Notify status change
   */
  private notifyStatusChange(
    operation: QueuedOperation,
    oldStatus: OperationStatus
  ): void {
    for (const handler of this.statusHandlers) {
      try {
        handler(operation, oldStatus);
      } catch (e) {
        console.error('Status change handler error:', e);
      }
    }
  }

  /**
   * Register default handlers for all operation types
   */
  private registerDefaultHandlers(): void {
    // Email operations
    this.registerHandler('send_email', async (op) => {
      if (!op.payload.email) throw new Error('Email payload required');
      await this.client.messages.sendEmail(op.payload.email as never);
    });

    this.registerHandler('mark_read', async (op) => {
      if (!op.payload.emailHash) throw new Error('Email hash required');
      await this.client.messages.markAsRead(op.payload.emailHash);
    });

    this.registerHandler('mark_unread', async (op) => {
      if (!op.payload.emailHash) throw new Error('Email hash required');
      await this.client.messages.markAsUnread(op.payload.emailHash);
    });

    this.registerHandler('archive', async (op) => {
      if (!op.payload.emailHash) throw new Error('Email hash required');
      await this.client.messages.archiveEmail(op.payload.emailHash);
    });

    this.registerHandler('delete', async (op) => {
      if (!op.payload.emailHash) throw new Error('Email hash required');
      await this.client.messages.deleteEmail(op.payload.emailHash);
    });

    this.registerHandler('move_to_folder', async (op) => {
      if (!op.payload.emailHash || !op.payload.folderHash) {
        throw new Error('Email hash and folder hash required');
      }
      await this.client.messages.moveToFolder(
        op.payload.emailHash,
        op.payload.folderHash
      );
    });

    this.registerHandler('add_label', async (op) => {
      if (!op.payload.emailHash || !op.payload.label) {
        throw new Error('Email hash and label required');
      }
      await this.client.messages.addLabel(op.payload.emailHash, op.payload.label);
    });

    this.registerHandler('remove_label', async (op) => {
      if (!op.payload.emailHash || !op.payload.label) {
        throw new Error('Email hash and label required');
      }
      await this.client.messages.removeLabel(op.payload.emailHash, op.payload.label);
    });

    this.registerHandler('star', async (op) => {
      if (!op.payload.emailHash) throw new Error('Email hash required');
      await this.client.messages.starEmail(op.payload.emailHash);
    });

    this.registerHandler('unstar', async (op) => {
      if (!op.payload.emailHash) throw new Error('Email hash required');
      await this.client.messages.unstarEmail(op.payload.emailHash);
    });

    // Draft operations
    this.registerHandler('create_draft', async (op) => {
      if (!op.payload.draftContent) throw new Error('Draft content required');
      await this.client.messages.createDraft(op.payload.draftContent as never);
    });

    this.registerHandler('update_draft', async (op) => {
      if (!op.payload.draftId || !op.payload.draftContent) {
        throw new Error('Draft ID and content required');
      }
      await this.client.messages.updateDraft(
        op.payload.draftId as never,
        op.payload.draftContent as never
      );
    });

    // Folder operations
    this.registerHandler('create_folder', async (op) => {
      if (!op.payload.folderName) throw new Error('Folder name required');
      await this.client.messages.createFolder(op.payload.folderName as string);
    });

    this.registerHandler('delete_folder', async (op) => {
      if (!op.payload.folderHash) throw new Error('Folder hash required');
      await this.client.messages.deleteFolder(op.payload.folderHash);
    });

    // Trust operations
    this.registerHandler('create_attestation', async (op) => {
      if (!op.payload.attestation) throw new Error('Attestation payload required');
      await this.client.trust.createAttestation(op.payload.attestation as never);
    });

    this.registerHandler('revoke_attestation', async (op) => {
      if (!op.payload.attestationHash) throw new Error('Attestation hash required');
      await this.client.trust.revokeAttestation(op.payload.attestationHash as never);
    });
  }

  // ==================== STATISTICS ====================

  /**
   * Get queue statistics
   */
  getStats(): QueueStats {
    const all = this.getAll();
    const pending = all.filter((op) => op.status === 'pending');

    return {
      total: all.length,
      pending: pending.length,
      processing: all.filter((op) => op.status === 'processing').length,
      completed: all.filter((op) => op.status === 'completed').length,
      failed: all.filter((op) => op.status === 'failed').length,
      cancelled: all.filter((op) => op.status === 'cancelled').length,
      oldestPending: pending.length > 0
        ? Math.min(...pending.map((op) => op.timestamp))
        : undefined,
      averageRetries: all.length > 0
        ? all.reduce((sum, op) => sum + op.retries, 0) / all.length
        : 0,
    };
  }

  /**
   * Check if online
   */
  getOnlineStatus(): boolean {
    return this.isOnline;
  }

  // ==================== UTILITIES ====================

  private generateId(): string {
    return `op_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  private delay(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  /**
   * Cleanup and close
   */
  destroy(): void {
    this.stopSyncInterval();
    if (this.db) {
      this.db.close();
      this.db = null;
    }
  }
}

/**
 * Create offline queue service for a client
 */
export function createOfflineQueue(
  client: MycelixMailClient,
  config?: Partial<OfflineQueueConfig>
): OfflineQueueService {
  return new OfflineQueueService(client, config);
}

export default OfflineQueueService;
