// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Offline-First Queue - Never Lose a Transaction
 *
 * Queues transactions when offline and syncs automatically when
 * connection is restored. Provides visual feedback for pending operations
 * and handles conflicts gracefully.
 *
 * @example
 * ```typescript
 * const queue = new OfflineQueue(storage, executor);
 *
 * // Queue a transaction (works offline!)
 * const pendingTx = await queue.enqueue({
 *   type: 'send',
 *   to: '@alice',
 *   amount: 100,
 *   currency: 'MYC',
 * });
 *
 * // Check pending count
 * console.log(`${queue.pendingCount} transactions waiting to sync`);
 *
 * // Auto-syncs when online, or manual sync:
 * await queue.sync();
 * ```
 */

import { BehaviorSubject, type Subscription } from '../reactive/index.js';

// =============================================================================
// Types
// =============================================================================

/** Unique queue item identifier */
export type QueueItemId = string & { readonly __brand: 'QueueItemId' };

/** Types of operations that can be queued */
export type QueuedOperationType =
  | 'send' // Send currency
  | 'request' // Request payment
  | 'memo' // Add memo to transaction
  | 'contact_add' // Add contact
  | 'contact_update' // Update contact
  | 'reputation_record'; // Record reputation interaction

/** Status of a queued item */
export type QueueItemStatus =
  | 'pending' // Waiting to be synced
  | 'syncing' // Currently being processed
  | 'completed' // Successfully synced
  | 'failed' // Failed after all retries
  | 'cancelled'; // Manually cancelled

/** A queued operation */
export interface QueuedOperation<T = unknown> {
  /** Unique ID */
  id: QueueItemId;
  /** Operation type */
  type: QueuedOperationType;
  /** Operation payload */
  payload: T;
  /** When queued */
  createdAt: number;
  /** When last attempted */
  lastAttemptAt?: number;
  /** Number of sync attempts */
  attempts: number;
  /** Maximum retry attempts */
  maxAttempts: number;
  /** Current status */
  status: QueueItemStatus;
  /** Error message if failed */
  error?: string;
  /** Priority (higher = process first) */
  priority: number;
  /** Dependencies (must complete before this) */
  dependsOn?: QueueItemId[];
  /** Idempotency key for deduplication */
  idempotencyKey?: string;
}

/** Result of atomically claiming a key in the durable queue store. */
export interface QueueStorageIdempotencyClaim {
  kind: 'inserted' | 'existing';
  item: QueuedOperation;
}

/** Send operation payload */
export interface SendPayload {
  to: string;
  amount: number;
  currency: string;
  memo?: string;
  /** Same stable key is reused by every executor retry. Optional only for legacy persisted items. */
  idempotencyKey?: string;
}

/** Request payment payload */
export interface RequestPayload {
  from: string;
  amount: number;
  currency: string;
  memo?: string;
  expiresAt?: number;
}

/** Memo payload */
export interface MemoPayload {
  transactionId: string;
  memo: string;
}

/** Queue state (reactive) */
export interface QueueState {
  /** All queued items */
  items: QueuedOperation[];
  /** Pending items count */
  pendingCount: number;
  /** Currently syncing */
  isSyncing: boolean;
  /** Online status */
  isOnline: boolean;
  /** Last sync timestamp */
  lastSyncAt?: number;
  /** Last error */
  lastError?: string;
}

/** Queue configuration */
export interface QueueConfig {
  /** Maximum retry attempts per operation */
  maxAttempts?: number;
  /** Base delay between retries (ms) */
  retryDelayMs?: number;
  /** Maximum delay between retries (ms) */
  maxRetryDelayMs?: number;
  /** Auto-sync when online? */
  autoSync?: boolean;
  /** Sync interval when online (ms) */
  syncIntervalMs?: number;
  /** Keep completed items for (ms) */
  completedRetentionMs?: number;
}

/** Storage interface for queue persistence */
export interface QueueStorage {
  getAll(): Promise<QueuedOperation[]>;
  save(item: QueuedOperation): Promise<void>;
  /**
   * Atomically claim a previously unused idempotency key, or return its existing
   * record. Implementations must serialize this read/check/write across callers
   * sharing the same durable store (including separate browser tabs).
   */
  claimIdempotencyKey(item: QueuedOperation): Promise<QueueStorageIdempotencyClaim>;
  delete(id: QueueItemId): Promise<void>;
  clear(): Promise<void>;
}

/** Executor interface for processing operations */
export interface QueueExecutor {
  execute(operation: QueuedOperation): Promise<ExecutionResult>;
  canExecute(operation: QueuedOperation): boolean;
}

/** Result of executing an operation */
export interface ExecutionResult {
  success: boolean;
  result?: unknown;
  error?: string;
  /** Should retry on failure? */
  retryable?: boolean;
}

/** Sync event */
export interface SyncEvent {
  type: 'sync_started' | 'sync_completed' | 'sync_failed' | 'item_synced' | 'item_failed';
  timestamp: number;
  itemId?: QueueItemId;
  itemsProcessed?: number;
  itemsFailed?: number;
  error?: string;
}

// =============================================================================
// In-Memory Storage (Default)
// =============================================================================

class InMemoryQueueStorage implements QueueStorage {
  private items: Map<QueueItemId, QueuedOperation> = new Map();

  async getAll(): Promise<QueuedOperation[]> {
    return Array.from(this.items.values());
  }

  async claimIdempotencyKey(item: QueuedOperation): Promise<QueueStorageIdempotencyClaim> {
    const key = item.idempotencyKey;
    if (!key) throw new Error('Cannot claim an empty idempotency key');

    const matches = Array.from(this.items.values()).filter(
      (stored) => stored.idempotencyKey === key
    );
    if (matches.length > 1) {
      throw new Error('Ambiguous queue state: multiple records already use this idempotency key');
    }
    if (matches.length === 1) return { kind: 'existing', item: matches[0] };

    // No await occurs between lookup and insertion, so this default store's claim
    // is atomic within the JavaScript agent.
    this.items.set(item.id, item);
    return { kind: 'inserted', item };
  }

  async save(item: QueuedOperation): Promise<void> {
    this.items.set(item.id, item);
  }

  async delete(id: QueueItemId): Promise<void> {
    this.items.delete(id);
  }

  async clear(): Promise<void> {
    this.items.clear();
  }
}

// =============================================================================
// IndexedDB Storage (Persistent)
// =============================================================================

/**
 * IndexedDB-based storage for offline persistence
 */
export class IndexedDBQueueStorage implements QueueStorage {
  private dbName: string;
  private storeName = 'queue';
  private db: IDBDatabase | null = null;

  constructor(dbName: string = 'mycelix-offline-queue') {
    this.dbName = dbName;
  }

  private async getDB(): Promise<IDBDatabase> {
    if (this.db) return this.db;

    return new Promise((resolve, reject) => {
      const request = indexedDB.open(this.dbName, 2);

      request.onerror = () => reject(request.error);
      request.onsuccess = () => {
        const db = request.result;
        this.db = db;
        // Let a newer tab upgrade the database instead of leaving it blocked.
        db.onversionchange = () => {
          db.close();
          if (this.db === db) this.db = null;
        };
        resolve(db);
      };

      request.onupgradeneeded = (event) => {
        const openRequest = event.target as IDBOpenDBRequest;
        const db = openRequest.result;
        let store: IDBObjectStore;
        if (!db.objectStoreNames.contains(this.storeName)) {
          store = db.createObjectStore(this.storeName, { keyPath: 'id' });
          store.createIndex('status', 'status', { unique: false });
          store.createIndex('createdAt', 'createdAt', { unique: false });
          store.createIndex('priority', 'priority', { unique: false });
        } else {
          store = openRequest.transaction!.objectStore(this.storeName);
        }
        // Non-unique so legacy duplicate data does not make the schema upgrade
        // fail. claimIdempotencyKey detects duplicates and quarantines ambiguity.
        if (!store.indexNames.contains('idempotencyKey')) {
          store.createIndex('idempotencyKey', 'idempotencyKey', { unique: false });
        }
      };
    });
  }

  async getAll(): Promise<QueuedOperation[]> {
    const db = await this.getDB();
    return new Promise((resolve, reject) => {
      const tx = db.transaction(this.storeName, 'readonly');
      const store = tx.objectStore(this.storeName);
      const request = store.getAll();

      request.onerror = () => reject(request.error);
      request.onsuccess = () => resolve(request.result);
    });
  }

  async claimIdempotencyKey(item: QueuedOperation): Promise<QueueStorageIdempotencyClaim> {
    const key = item.idempotencyKey;
    if (!key) throw new Error('Cannot claim an empty idempotency key');

    const db = await this.getDB();
    return new Promise((resolve, reject) => {
      // A read/write transaction on the same object store serializes this
      // lookup/check/insert across storage instances and browser tabs.
      const tx = db.transaction(this.storeName, 'readwrite');
      const store = tx.objectStore(this.storeName);
      const lookup = store.index('idempotencyKey').getAll(key);
      let outcome: QueueStorageIdempotencyClaim | undefined;
      let failure: Error | undefined;

      lookup.onerror = () => {
        failure = lookup.error ?? new Error('Unable to read queue idempotency index');
      };
      lookup.onsuccess = () => {
        const matches = lookup.result as QueuedOperation[];
        if (matches.length > 1) {
          failure = new Error(
            'Ambiguous queue state: multiple records already use this idempotency key'
          );
          return;
        }
        if (matches.length === 1) {
          outcome = { kind: 'existing', item: matches[0] };
          return;
        }

        const insert = store.add(item);
        insert.onsuccess = () => {
          outcome = { kind: 'inserted', item };
        };
        insert.onerror = () => {
          failure = insert.error ?? new Error('Unable to persist queue idempotency claim');
        };
      };

      tx.oncomplete = () => {
        if (failure) {
          reject(failure);
        } else if (outcome) {
          resolve(outcome);
        } else {
          reject(new Error('IndexedDB idempotency transaction completed without a result'));
        }
      };
      tx.onabort = () => {
        reject(failure ?? tx.error ?? new Error('IndexedDB idempotency transaction aborted'));
      };
      tx.onerror = () => {
        failure ??= tx.error ?? new Error('IndexedDB idempotency transaction failed');
      };
    });
  }

  async save(item: QueuedOperation): Promise<void> {
    const db = await this.getDB();
    return new Promise((resolve, reject) => {
      const tx = db.transaction(this.storeName, 'readwrite');
      const store = tx.objectStore(this.storeName);
      store.put(item);

      tx.oncomplete = () => resolve();
      tx.onabort = () => reject(tx.error ?? new Error('Queue save transaction aborted'));
      tx.onerror = () => reject(tx.error ?? new Error('Queue save transaction failed'));
    });
  }

  async delete(id: QueueItemId): Promise<void> {
    const db = await this.getDB();
    return new Promise((resolve, reject) => {
      const tx = db.transaction(this.storeName, 'readwrite');
      const store = tx.objectStore(this.storeName);
      const request = store.delete(id);

      request.onerror = () => reject(request.error);
      request.onsuccess = () => resolve();
    });
  }

  async clear(): Promise<void> {
    const db = await this.getDB();
    return new Promise((resolve, reject) => {
      const tx = db.transaction(this.storeName, 'readwrite');
      const store = tx.objectStore(this.storeName);
      const request = store.clear();

      request.onerror = () => reject(request.error);
      request.onsuccess = () => resolve();
    });
  }

  close(): void {
    if (this.db) {
      this.db.close();
      this.db = null;
    }
  }
}

// =============================================================================
// Offline Queue
// =============================================================================

const DEFAULT_CONFIG: Required<QueueConfig> = {
  maxAttempts: 5,
  retryDelayMs: 1000,
  maxRetryDelayMs: 60000,
  autoSync: true,
  syncIntervalMs: 30000,
  completedRetentionMs: 24 * 60 * 60 * 1000, // 24 hours
};

/**
 * Offline-first queue for transactions and operations
 */
export class OfflineQueue {
  private storage: QueueStorage;
  private executor: QueueExecutor;
  private config: Required<QueueConfig>;
  private _state$: BehaviorSubject<QueueState>;
  /** Serializes enqueue lookup + persistence so concurrent identical keys cannot both win locally. */
  private enqueueTail: Promise<void> = Promise.resolve();
  private syncInterval?: ReturnType<typeof setInterval>;
  private cleanupInterval?: ReturnType<typeof setInterval>;
  private onlineListener?: () => void;
  private offlineListener?: () => void;
  private syncEventListeners: Array<(event: SyncEvent) => void> = [];

  constructor(
    storage?: QueueStorage,
    executor?: QueueExecutor,
    config?: QueueConfig
  ) {
    this.storage = storage ?? new InMemoryQueueStorage();
    this.executor = executor ?? createNoOpExecutor();
    this.config = { ...DEFAULT_CONFIG, ...config };

    this._state$ = new BehaviorSubject<QueueState>({
      items: [],
      pendingCount: 0,
      isSyncing: false,
      isOnline: typeof navigator !== 'undefined' ? navigator.onLine : true,
    });
  }

  // ===========================================================================
  // Reactive State
  // ===========================================================================

  /** Observable state */
  get state$(): BehaviorSubject<QueueState> {
    return this._state$;
  }

  /** Current state */
  get state(): QueueState {
    return this._state$.value;
  }

  /** Number of pending items */
  get pendingCount(): number {
    return this._state$.value.pendingCount;
  }

  /** Whether queue is syncing */
  get isSyncing(): boolean {
    return this._state$.value.isSyncing;
  }

  /** Whether device is online */
  get isOnline(): boolean {
    return this._state$.value.isOnline;
  }

  /** Subscribe to state changes */
  subscribe(observer: (state: QueueState) => void): Subscription {
    return this._state$.subscribe(observer);
  }

  /** Subscribe to sync events */
  onSyncEvent(listener: (event: SyncEvent) => void): () => void {
    this.syncEventListeners.push(listener);
    return () => {
      const index = this.syncEventListeners.indexOf(listener);
      if (index !== -1) this.syncEventListeners.splice(index, 1);
    };
  }

  // ===========================================================================
  // Executor Management
  // ===========================================================================

  /**
   * Set or replace the queue executor.
   * Useful when the executor depends on components created after the queue.
   *
   * @example
   * ```typescript
   * // Create queue first
   * const queue = createOfflineQueue();
   *
   * // Create wallet/provider that needs the queue
   * const provider = createFinanceProvider({ queue });
   *
   * // Now wire the executor back to the queue
   * queue.setExecutor(provider.createQueueExecutor());
   * ```
   */
  setExecutor(executor: QueueExecutor): void {
    this.executor = executor;
  }

  /**
   * Get the current executor (for testing/debugging)
   */
  getExecutor(): QueueExecutor {
    return this.executor;
  }

  // ===========================================================================
  // Lifecycle
  // ===========================================================================

  /**
   * Initialize the queue
   */
  async initialize(): Promise<void> {
    // Load persisted items
    const items = await this.storage.getAll();
    this.updateState({
      items,
      pendingCount: items.filter((i) => i.status === 'pending' || i.status === 'syncing').length,
    });

    // Setup online/offline listeners
    if (typeof window !== 'undefined') {
      this.onlineListener = () => {
        this.updateState({ isOnline: true });
        if (this.config.autoSync) {
          void this.sync();
        }
      };
      this.offlineListener = () => {
        this.updateState({ isOnline: false });
      };

      window.addEventListener('online', this.onlineListener);
      window.addEventListener('offline', this.offlineListener);
    }

    // Setup auto-sync interval
    if (this.config.autoSync && this.config.syncIntervalMs > 0) {
      this.syncInterval = setInterval(() => {
        if (this._state$.value.isOnline && !this._state$.value.isSyncing) {
          void this.sync();
        }
      }, this.config.syncIntervalMs);
    }

    // Setup cleanup interval (every hour)
    this.cleanupInterval = setInterval(() => {
      void this.cleanup();
    }, 60 * 60 * 1000);

    // Initial sync if online
    if (this._state$.value.isOnline && this.config.autoSync) {
      void this.sync();
    }
  }

  /**
   * Cleanup and release resources
   */
  destroy(): void {
    if (this.syncInterval) {
      clearInterval(this.syncInterval);
    }
    if (this.cleanupInterval) {
      clearInterval(this.cleanupInterval);
    }
    if (typeof window !== 'undefined') {
      if (this.onlineListener) {
        window.removeEventListener('online', this.onlineListener);
      }
      if (this.offlineListener) {
        window.removeEventListener('offline', this.offlineListener);
      }
    }
    this.syncEventListeners = [];
  }

  // ===========================================================================
  // Queue Operations
  // ===========================================================================

  /**
   * Enqueue an operation
   */
  enqueue<T>(
    type: QueuedOperationType,
    payload: T,
    options?: {
      priority?: number;
      maxAttempts?: number;
      dependsOn?: QueueItemId[];
      idempotencyKey?: string;
    }
  ): Promise<QueuedOperation<T>> {
    // Deduplication must cover the whole lookup -> durable write -> state update.
    // Without this local mutex, simultaneous calls can both observe a missing key
    // and persist two queue entries before either updates reactive state.
    const scheduled = this.enqueueTail.then(() => this.enqueueUnlocked(type, payload, options));
    this.enqueueTail = scheduled.then(
      () => undefined,
      () => undefined
    );
    return scheduled;
  }

  private async enqueueUnlocked<T>(
    type: QueuedOperationType,
    payload: T,
    options?: {
      priority?: number;
      maxAttempts?: number;
      dependsOn?: QueueItemId[];
      idempotencyKey?: string;
    }
  ): Promise<QueuedOperation<T>> {
    if (options?.idempotencyKey === '') {
      throw new Error('Idempotency key must not be empty');
    }

    let operationPayload = payload;
    let operationIdempotencyKey = options?.idempotencyKey;
    if (type === 'send') {
      const isObjectPayload =
        operationPayload !== null &&
        typeof operationPayload === 'object' &&
        !Array.isArray(operationPayload);

      if (isObjectPayload) {
        const payloadKey = (operationPayload as { idempotencyKey?: unknown }).idempotencyKey;
        if (payloadKey !== undefined && (typeof payloadKey !== 'string' || payloadKey.length === 0)) {
          throw new Error('Send payload idempotency key must be a non-empty string');
        }
        if (
          operationIdempotencyKey !== undefined &&
          payloadKey !== undefined &&
          payloadKey !== operationIdempotencyKey
        ) {
          throw new Error('Send payload idempotency key does not match queue idempotency key');
        }
        if (operationIdempotencyKey === undefined && typeof payloadKey === 'string') {
          operationIdempotencyKey = payloadKey;
        }
      }

      // Every new send has a queue identity, and the persisted payload and
      // metadata carry exactly the same value for every retry path.
      operationIdempotencyKey ??= `send:${generateQueueItemId()}`;
      if (isObjectPayload) {
        operationPayload = {
          ...(operationPayload as Record<string, unknown>),
          idempotencyKey: operationIdempotencyKey,
        } as T;
      }
    }

    const item: QueuedOperation<T> = {
      id: generateQueueItemId(),
      type,
      payload: operationPayload,
      createdAt: Date.now(),
      attempts: 0,
      maxAttempts: options?.maxAttempts ?? this.config.maxAttempts,
      status: 'pending',
      priority: options?.priority ?? 0,
      dependsOn: options?.dependsOn,
      idempotencyKey: operationIdempotencyKey,
    };

    let persistedItem = item as QueuedOperation;
    if (persistedItem.idempotencyKey) {
      // The storage transaction, not this instance's cache, is the authority.
      // That is what makes key claims safe across separate tabs/instances.
      const claim = await this.storage.claimIdempotencyKey(persistedItem);
      persistedItem = claim.item;

      if (claim.kind === 'existing') {
        const samePayload =
          persistedItem.type === type &&
          JSON.stringify(persistedItem.payload) === JSON.stringify(item.payload);
        if (!samePayload) {
          throw new Error('Idempotency key reused with a different queued operation payload');
        }
        if (persistedItem.status === 'failed' || persistedItem.status === 'cancelled') {
          throw new Error(
            'Idempotency key already belongs to a failed or cancelled queue item; retry that item explicitly or use a fresh key for a new logical payment'
          );
        }
      }
    } else {
      await this.storage.save(persistedItem);
    }

    // Replace a stale local copy of this key with the storage-authoritative record.
    const items = [
      ...this._state$.value.items.filter(
        (stored) =>
          stored.id !== persistedItem.id &&
          (!persistedItem.idempotencyKey || stored.idempotencyKey !== persistedItem.idempotencyKey)
      ),
      persistedItem,
    ];
    this.updateState({
      items,
      pendingCount: items.filter((i) => i.status === 'pending' || i.status === 'syncing').length,
    });

    // Try immediate sync if online and autoSync is enabled
    if (
      persistedItem.status === 'pending' &&
      this.config.autoSync &&
      this._state$.value.isOnline &&
      !this._state$.value.isSyncing
    ) {
      void this.sync();
    }

    return persistedItem as QueuedOperation<T>;
  }

  /**
   * Enqueue a send operation
   */
  async enqueueSend(
    to: string,
    amount: number,
    currency: string,
    memo?: string,
    options?: { priority?: number; idempotencyKey?: string }
  ): Promise<QueuedOperation<SendPayload>> {
    const idempotencyKey = options?.idempotencyKey ?? `send:${generateQueueItemId()}`;
    return this.enqueue<SendPayload>(
      'send',
      { to, amount, currency, memo, idempotencyKey },
      {
        ...options,
        idempotencyKey,
      }
    );
  }

  /**
   * Enqueue a payment request
   */
  async enqueueRequest(
    from: string,
    amount: number,
    currency: string,
    memo?: string,
    expiresAt?: number
  ): Promise<QueuedOperation<RequestPayload>> {
    return this.enqueue<RequestPayload>('request', {
      from,
      amount,
      currency,
      memo,
      expiresAt,
    });
  }

  /**
   * Cancel a queued operation
   */
  async cancel(id: QueueItemId): Promise<boolean> {
    const item = this._state$.value.items.find((i) => i.id === id);
    if (!item) return false;

    // Can only cancel pending items
    if (item.status !== 'pending') {
      return false;
    }

    const updated: QueuedOperation = {
      ...item,
      status: 'cancelled',
    };

    await this.storage.save(updated);
    this.updateState({
      items: this._state$.value.items.map((i) => (i.id === id ? updated : i)),
      pendingCount: this._state$.value.pendingCount - 1,
    });

    return true;
  }

  /**
   * Retry a failed operation
   */
  async retry(id: QueueItemId): Promise<boolean> {
    const item = this._state$.value.items.find((i) => i.id === id);
    if (!item) return false;

    // Can only retry failed items
    if (item.status !== 'failed') {
      return false;
    }

    const updated: QueuedOperation = {
      ...item,
      status: 'pending',
      attempts: 0,
      error: undefined,
    };

    await this.storage.save(updated);
    this.updateState({
      items: this._state$.value.items.map((i) => (i.id === id ? updated : i)),
      pendingCount: this._state$.value.pendingCount + 1,
    });

    // Try immediate sync if autoSync is enabled
    if (this.config.autoSync && this._state$.value.isOnline && !this._state$.value.isSyncing) {
      void this.sync();
    }

    return true;
  }

  /**
   * Get a queued item by ID
   */
  get(id: QueueItemId): QueuedOperation | undefined {
    return this._state$.value.items.find((i) => i.id === id);
  }

  /**
   * Get items by status
   */
  getByStatus(status: QueueItemStatus): QueuedOperation[] {
    return this._state$.value.items.filter((i) => i.status === status);
  }

  // ===========================================================================
  // Sync
  // ===========================================================================

  /**
   * Sync all pending operations
   */
  async sync(): Promise<SyncResult> {
    if (this._state$.value.isSyncing) {
      return { processed: 0, failed: 0, remaining: this._state$.value.pendingCount };
    }

    if (!this._state$.value.isOnline) {
      return { processed: 0, failed: 0, remaining: this._state$.value.pendingCount, error: 'offline' };
    }

    this.updateState({ isSyncing: true });
    this.emitSyncEvent({ type: 'sync_started', timestamp: Date.now() });

    let processed = 0;
    let failed = 0;

    try {
      // Get pending items, sorted by priority and creation time
      const pending = this._state$.value.items
        .filter((i) => i.status === 'pending')
        .sort((a, b) => {
          if (a.priority !== b.priority) return b.priority - a.priority;
          return a.createdAt - b.createdAt;
        });

      for (const item of pending) {
        // Check dependencies
        if (item.dependsOn?.length) {
          const dependenciesMet = item.dependsOn.every((depId) => {
            const dep = this._state$.value.items.find((i) => i.id === depId);
            return dep?.status === 'completed';
          });
          if (!dependenciesMet) continue;
        }

        // Check if executor can handle this
        if (!this.executor.canExecute(item)) continue;

        // Process the item
        const result = await this.processItem(item);
        if (result.success) {
          processed++;
        } else {
          failed++;
        }
      }

      this.updateState({
        lastSyncAt: Date.now(),
        lastError: undefined,
      });

      this.emitSyncEvent({
        type: 'sync_completed',
        timestamp: Date.now(),
        itemsProcessed: processed,
        itemsFailed: failed,
      });
    } catch (error) {
      const errorMsg = error instanceof Error ? error.message : 'Sync failed';
      this.updateState({ lastError: errorMsg });
      this.emitSyncEvent({
        type: 'sync_failed',
        timestamp: Date.now(),
        error: errorMsg,
      });
    } finally {
      this.updateState({ isSyncing: false });
    }

    return {
      processed,
      failed,
      remaining: this._state$.value.items.filter(
        (i) => i.status === 'pending' || i.status === 'syncing'
      ).length,
    };
  }

  /**
   * Process a single item
   */
  private async processItem(item: QueuedOperation): Promise<{ success: boolean }> {
    // Mark as syncing
    const syncing: QueuedOperation = {
      ...item,
      status: 'syncing',
      lastAttemptAt: Date.now(),
      attempts: item.attempts + 1,
    };
    await this.storage.save(syncing);
    this.updateState({
      items: this._state$.value.items.map((i) => (i.id === item.id ? syncing : i)),
    });

    try {
      const result = await this.executor.execute(syncing);

      if (result.success) {
        // Mark as completed
        const completed: QueuedOperation = {
          ...syncing,
          status: 'completed',
        };
        await this.storage.save(completed);
        this.updateState({
          items: this._state$.value.items.map((i) => (i.id === item.id ? completed : i)),
          pendingCount: this._state$.value.pendingCount - 1,
        });

        this.emitSyncEvent({
          type: 'item_synced',
          timestamp: Date.now(),
          itemId: item.id,
        });

        return { success: true };
      } else {
        // Handle failure
        const shouldRetry = result.retryable !== false && syncing.attempts < syncing.maxAttempts;

        const updated: QueuedOperation = {
          ...syncing,
          status: shouldRetry ? 'pending' : 'failed',
          error: result.error,
        };
        await this.storage.save(updated);

        const newPendingCount = shouldRetry
          ? this._state$.value.pendingCount
          : this._state$.value.pendingCount - 1;

        this.updateState({
          items: this._state$.value.items.map((i) => (i.id === item.id ? updated : i)),
          pendingCount: newPendingCount,
        });

        if (!shouldRetry) {
          this.emitSyncEvent({
            type: 'item_failed',
            timestamp: Date.now(),
            itemId: item.id,
            error: result.error,
          });
        }

        return { success: false };
      }
    } catch (error) {
      const errorMsg = error instanceof Error ? error.message : 'Unknown error';
      const shouldRetry = syncing.attempts < syncing.maxAttempts;

      const updated: QueuedOperation = {
        ...syncing,
        status: shouldRetry ? 'pending' : 'failed',
        error: errorMsg,
      };
      await this.storage.save(updated);

      const newPendingCount = shouldRetry
        ? this._state$.value.pendingCount
        : this._state$.value.pendingCount - 1;

      this.updateState({
        items: this._state$.value.items.map((i) => (i.id === item.id ? updated : i)),
        pendingCount: newPendingCount,
      });

      if (!shouldRetry) {
        this.emitSyncEvent({
          type: 'item_failed',
          timestamp: Date.now(),
          itemId: item.id,
          error: errorMsg,
        });
      }

      return { success: false };
    }
  }

  // ===========================================================================
  // Cleanup
  // ===========================================================================

  /**
   * Clean up old completed/cancelled/failed items
   */
  async cleanup(): Promise<number> {
    const now = Date.now();
    const cutoff = now - this.config.completedRetentionMs;

    const toRemove = this._state$.value.items.filter(
      (i) =>
        (i.status === 'completed' || i.status === 'cancelled' || i.status === 'failed') &&
        (i.lastAttemptAt ?? i.createdAt) < cutoff
    );

    for (const item of toRemove) {
      await this.storage.delete(item.id);
    }

    if (toRemove.length > 0) {
      this.updateState({
        items: this._state$.value.items.filter((i) => !toRemove.includes(i)),
      });
    }

    return toRemove.length;
  }

  /**
   * Clear all items (use with caution!)
   */
  async clear(): Promise<void> {
    await this.storage.clear();
    this.updateState({
      items: [],
      pendingCount: 0,
    });
  }

  // ===========================================================================
  // Private Helpers
  // ===========================================================================

  private updateState(partial: Partial<QueueState>): void {
    this._state$.next({
      ...this._state$.value,
      ...partial,
    });
  }

  private emitSyncEvent(event: SyncEvent): void {
    for (const listener of this.syncEventListeners) {
      try {
        listener(event);
      } catch {
        // Ignore listener errors
      }
    }
  }
}

// =============================================================================
// Sync Result
// =============================================================================

export interface SyncResult {
  processed: number;
  failed: number;
  remaining: number;
  error?: string;
}

// =============================================================================
// Utilities
// =============================================================================

function generateQueueItemId(): QueueItemId {
  return `q-${Date.now()}-${Math.random().toString(36).slice(2, 8)}` as QueueItemId;
}

/**
 * Create a no-op executor (for testing)
 */
function createNoOpExecutor(): QueueExecutor {
  return {
    execute: async () => ({ success: true }),
    canExecute: () => true,
  };
}

/**
 * Create an executor that delegates to the wallet
 */
export function createWalletExecutor(wallet: {
  send(
    to: string,
    amount: number,
    currency: string,
    memo?: string,
    idempotencyKey?: string
  ): Promise<unknown>;
}): QueueExecutor {
  return {
    execute: async (operation) => {
      if (operation.type === 'send') {
        const payload = operation.payload as SendPayload;
        try {
          const idempotencyKey = payload.idempotencyKey ?? operation.idempotencyKey;
          if (!idempotencyKey) {
            return {
              success: false,
              error: 'Queued payment has no stable idempotency key; refusing automatic dispatch',
              retryable: false,
            };
          }
          await wallet.send(
            payload.to,
            payload.amount,
            payload.currency,
            payload.memo,
            idempotencyKey
          );
          return { success: true };
        } catch (error) {
          return {
            success: false,
            error: error instanceof Error ? error.message : 'Send failed',
            retryable: true,
          };
        }
      }
      return { success: false, error: 'Unsupported operation type', retryable: false };
    },
    canExecute: (operation) => operation.type === 'send',
  };
}

// =============================================================================
// Factory
// =============================================================================

/**
 * Create an offline queue with sensible defaults
 */
export function createOfflineQueue(
  storage?: QueueStorage,
  executor?: QueueExecutor,
  config?: QueueConfig
): OfflineQueue {
  return new OfflineQueue(storage, executor, config);
}

/**
 * Create an IndexedDB-backed offline queue (for browsers)
 */
export function createPersistentOfflineQueue(
  dbName?: string,
  executor?: QueueExecutor,
  config?: QueueConfig
): OfflineQueue {
  const storage = new IndexedDBQueueStorage(dbName);
  return new OfflineQueue(storage, executor, config);
}
