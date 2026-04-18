// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Cache Service for Mycelix Mail
 *
 * Multi-tier caching with:
 * - Memory cache (LRU)
 * - IndexedDB persistence
 * - Smart invalidation
 * - TTL support
 */

import type { ActionHash, AgentPubKey } from '@holochain/client';

// ==================== TYPES ====================

export interface CacheConfig {
  /** Maximum memory cache entries */
  maxMemoryEntries: number;
  /** Default TTL in milliseconds */
  defaultTTL: number;
  /** IndexedDB database name */
  dbName: string;
  /** Enable persistence */
  enablePersistence: boolean;
  /** Enable compression for large values */
  enableCompression: boolean;
  /** Compression threshold in bytes */
  compressionThreshold: number;
}

export interface CacheEntry<T> {
  key: string;
  value: T;
  createdAt: number;
  accessedAt: number;
  ttl: number;
  size: number;
  compressed: boolean;
}

export interface CacheStats {
  memoryHits: number;
  memoryMisses: number;
  persistenceHits: number;
  persistenceMisses: number;
  evictions: number;
  totalSize: number;
  entryCount: number;
}

// ==================== CACHE SERVICE ====================

export class CacheService {
  private config: CacheConfig;
  private memoryCache: Map<string, CacheEntry<unknown>> = new Map();
  private accessOrder: string[] = [];
  private stats: CacheStats = {
    memoryHits: 0,
    memoryMisses: 0,
    persistenceHits: 0,
    persistenceMisses: 0,
    evictions: 0,
    totalSize: 0,
    entryCount: 0,
  };
  private db: IDBDatabase | null = null;
  private pendingWrites: Map<string, Promise<void>> = new Map();

  constructor(config?: Partial<CacheConfig>) {
    this.config = {
      maxMemoryEntries: config?.maxMemoryEntries ?? 1000,
      defaultTTL: config?.defaultTTL ?? 5 * 60 * 1000, // 5 minutes
      dbName: config?.dbName ?? 'mycelix-cache',
      enablePersistence: config?.enablePersistence ?? true,
      enableCompression: config?.enableCompression ?? true,
      compressionThreshold: config?.compressionThreshold ?? 10000,
    };

    if (this.config.enablePersistence) {
      this.initIndexedDB();
    }
  }

  // ==================== INITIALIZATION ====================

  private async initIndexedDB(): Promise<void> {
    return new Promise((resolve, reject) => {
      const request = indexedDB.open(this.config.dbName, 1);

      request.onerror = () => reject(request.error);

      request.onsuccess = () => {
        this.db = request.result;
        resolve();
      };

      request.onupgradeneeded = (event) => {
        const db = (event.target as IDBOpenDBRequest).result;

        // Create cache store
        if (!db.objectStoreNames.contains('cache')) {
          const store = db.createObjectStore('cache', { keyPath: 'key' });
          store.createIndex('createdAt', 'createdAt');
          store.createIndex('accessedAt', 'accessedAt');
        }
      };
    });
  }

  // ==================== CORE OPERATIONS ====================

  /**
   * Get value from cache
   */
  async get<T>(key: string): Promise<T | null> {
    // Try memory cache first
    const memoryEntry = this.memoryCache.get(key) as CacheEntry<T> | undefined;

    if (memoryEntry) {
      if (this.isExpired(memoryEntry)) {
        this.delete(key);
        this.stats.memoryMisses++;
        return null;
      }

      // Update access time
      memoryEntry.accessedAt = Date.now();
      this.updateAccessOrder(key);
      this.stats.memoryHits++;

      return memoryEntry.value;
    }

    this.stats.memoryMisses++;

    // Try persistence
    if (this.config.enablePersistence && this.db) {
      const persistedEntry = await this.getFromDB<T>(key);

      if (persistedEntry) {
        if (this.isExpired(persistedEntry)) {
          await this.deleteFromDB(key);
          this.stats.persistenceMisses++;
          return null;
        }

        // Promote to memory cache
        this.setMemory(key, persistedEntry.value, persistedEntry.ttl);
        this.stats.persistenceHits++;

        return persistedEntry.value;
      }

      this.stats.persistenceMisses++;
    }

    return null;
  }

  /**
   * Set value in cache
   */
  async set<T>(key: string, value: T, ttl?: number): Promise<void> {
    const effectiveTTL = ttl ?? this.config.defaultTTL;

    // Set in memory
    this.setMemory(key, value, effectiveTTL);

    // Persist asynchronously
    if (this.config.enablePersistence && this.db) {
      const writePromise = this.setInDB(key, value, effectiveTTL);
      this.pendingWrites.set(key, writePromise);
      writePromise.finally(() => this.pendingWrites.delete(key));
    }
  }

  /**
   * Set in memory cache with LRU eviction
   */
  private setMemory<T>(key: string, value: T, ttl: number): void {
    const now = Date.now();
    const size = this.estimateSize(value);

    const entry: CacheEntry<T> = {
      key,
      value,
      createdAt: now,
      accessedAt: now,
      ttl,
      size,
      compressed: false,
    };

    // Evict if necessary
    while (this.memoryCache.size >= this.config.maxMemoryEntries) {
      this.evictOldest();
    }

    this.memoryCache.set(key, entry as CacheEntry<unknown>);
    this.updateAccessOrder(key);
    this.stats.entryCount = this.memoryCache.size;
    this.stats.totalSize += size;
  }

  /**
   * Delete from cache
   */
  async delete(key: string): Promise<void> {
    const entry = this.memoryCache.get(key);
    if (entry) {
      this.stats.totalSize -= entry.size;
    }

    this.memoryCache.delete(key);
    this.accessOrder = this.accessOrder.filter((k) => k !== key);
    this.stats.entryCount = this.memoryCache.size;

    if (this.config.enablePersistence && this.db) {
      await this.deleteFromDB(key);
    }
  }

  /**
   * Clear all cache
   */
  async clear(): Promise<void> {
    this.memoryCache.clear();
    this.accessOrder = [];
    this.stats.totalSize = 0;
    this.stats.entryCount = 0;

    if (this.config.enablePersistence && this.db) {
      await this.clearDB();
    }
  }

  /**
   * Check if key exists
   */
  async has(key: string): Promise<boolean> {
    const value = await this.get(key);
    return value !== null;
  }

  // ==================== SPECIALIZED CACHE METHODS ====================

  /**
   * Cache email by hash
   */
  async cacheEmail(hash: ActionHash, email: unknown, ttl?: number): Promise<void> {
    await this.set(`email:${hashToString(hash)}`, email, ttl ?? 60000);
  }

  /**
   * Get cached email
   */
  async getCachedEmail<T>(hash: ActionHash): Promise<T | null> {
    return this.get(`email:${hashToString(hash)}`);
  }

  /**
   * Cache inbox
   */
  async cacheInbox(emails: unknown[], ttl?: number): Promise<void> {
    await this.set('inbox', emails, ttl ?? 30000);
  }

  /**
   * Get cached inbox
   */
  async getCachedInbox<T>(): Promise<T[] | null> {
    return this.get('inbox');
  }

  /**
   * Cache trust score
   */
  async cacheTrustScore(
    agent: AgentPubKey,
    category: string | null,
    score: unknown,
    ttl?: number
  ): Promise<void> {
    const key = `trust:${agentToString(agent)}:${category ?? 'all'}`;
    await this.set(key, score, ttl ?? 120000); // 2 minutes
  }

  /**
   * Get cached trust score
   */
  async getCachedTrustScore<T>(
    agent: AgentPubKey,
    category: string | null
  ): Promise<T | null> {
    const key = `trust:${agentToString(agent)}:${category ?? 'all'}`;
    return this.get(key);
  }

  /**
   * Invalidate trust scores for an agent
   */
  async invalidateTrustScores(agent: AgentPubKey): Promise<void> {
    const prefix = `trust:${agentToString(agent)}:`;
    const keysToDelete: string[] = [];

    for (const key of this.memoryCache.keys()) {
      if (key.startsWith(prefix)) {
        keysToDelete.push(key);
      }
    }

    for (const key of keysToDelete) {
      await this.delete(key);
    }
  }

  /**
   * Cache search results
   */
  async cacheSearchResults(
    query: string,
    results: unknown[],
    ttl?: number
  ): Promise<void> {
    const key = `search:${hashString(query)}`;
    await this.set(key, results, ttl ?? 60000);
  }

  /**
   * Get cached search results
   */
  async getCachedSearchResults<T>(query: string): Promise<T[] | null> {
    const key = `search:${hashString(query)}`;
    return this.get(key);
  }

  // ==================== INVALIDATION ====================

  /**
   * Invalidate cache by pattern
   */
  async invalidatePattern(pattern: RegExp): Promise<number> {
    let count = 0;
    const keysToDelete: string[] = [];

    for (const key of this.memoryCache.keys()) {
      if (pattern.test(key)) {
        keysToDelete.push(key);
        count++;
      }
    }

    for (const key of keysToDelete) {
      await this.delete(key);
    }

    return count;
  }

  /**
   * Invalidate all email caches
   */
  async invalidateEmails(): Promise<void> {
    await this.invalidatePattern(/^(email:|inbox|sent|drafts)/);
  }

  /**
   * Invalidate all trust caches
   */
  async invalidateTrust(): Promise<void> {
    await this.invalidatePattern(/^trust:/);
  }

  // ==================== PERSISTENCE ====================

  private async getFromDB<T>(key: string): Promise<CacheEntry<T> | null> {
    if (!this.db) return null;

    return new Promise((resolve, reject) => {
      const transaction = this.db!.transaction('cache', 'readonly');
      const store = transaction.objectStore('cache');
      const request = store.get(key);

      request.onerror = () => reject(request.error);
      request.onsuccess = () => {
        resolve(request.result as CacheEntry<T> | null);
      };
    });
  }

  private async setInDB<T>(key: string, value: T, ttl: number): Promise<void> {
    if (!this.db) return;

    const now = Date.now();
    const size = this.estimateSize(value);

    const entry: CacheEntry<T> = {
      key,
      value,
      createdAt: now,
      accessedAt: now,
      ttl,
      size,
      compressed: false,
    };

    return new Promise((resolve, reject) => {
      const transaction = this.db!.transaction('cache', 'readwrite');
      const store = transaction.objectStore('cache');
      const request = store.put(entry);

      request.onerror = () => reject(request.error);
      request.onsuccess = () => resolve();
    });
  }

  private async deleteFromDB(key: string): Promise<void> {
    if (!this.db) return;

    return new Promise((resolve, reject) => {
      const transaction = this.db!.transaction('cache', 'readwrite');
      const store = transaction.objectStore('cache');
      const request = store.delete(key);

      request.onerror = () => reject(request.error);
      request.onsuccess = () => resolve();
    });
  }

  private async clearDB(): Promise<void> {
    if (!this.db) return;

    return new Promise((resolve, reject) => {
      const transaction = this.db!.transaction('cache', 'readwrite');
      const store = transaction.objectStore('cache');
      const request = store.clear();

      request.onerror = () => reject(request.error);
      request.onsuccess = () => resolve();
    });
  }

  // ==================== LRU MANAGEMENT ====================

  private updateAccessOrder(key: string): void {
    this.accessOrder = this.accessOrder.filter((k) => k !== key);
    this.accessOrder.push(key);
  }

  private evictOldest(): void {
    const oldestKey = this.accessOrder.shift();
    if (oldestKey) {
      const entry = this.memoryCache.get(oldestKey);
      if (entry) {
        this.stats.totalSize -= entry.size;
      }
      this.memoryCache.delete(oldestKey);
      this.stats.evictions++;
    }
  }

  // ==================== UTILITIES ====================

  private isExpired(entry: CacheEntry<unknown>): boolean {
    return Date.now() > entry.createdAt + entry.ttl;
  }

  private estimateSize(value: unknown): number {
    try {
      return JSON.stringify(value).length * 2; // Rough estimate
    } catch {
      return 1000;
    }
  }

  /**
   * Get cache statistics
   */
  getStats(): CacheStats {
    return { ...this.stats };
  }

  /**
   * Get hit rate
   */
  getHitRate(): number {
    const total =
      this.stats.memoryHits +
      this.stats.memoryMisses +
      this.stats.persistenceHits +
      this.stats.persistenceMisses;

    if (total === 0) return 0;

    return (this.stats.memoryHits + this.stats.persistenceHits) / total;
  }

  /**
   * Wait for pending writes
   */
  async flush(): Promise<void> {
    await Promise.all(this.pendingWrites.values());
  }
}

// ==================== HELPERS ====================

function hashToString(hash: ActionHash): string {
  if (typeof hash === 'string') return hash;
  return Array.from(hash as Uint8Array)
    .map((b) => b.toString(16).padStart(2, '0'))
    .join('');
}

function agentToString(agent: AgentPubKey): string {
  if (typeof agent === 'string') return agent;
  return Array.from(agent as Uint8Array)
    .map((b) => b.toString(16).padStart(2, '0'))
    .join('');
}

function hashString(str: string): string {
  let hash = 0;
  for (let i = 0; i < str.length; i++) {
    const char = str.charCodeAt(i);
    hash = (hash << 5) - hash + char;
    hash = hash & hash;
  }
  return Math.abs(hash).toString(36);
}

/**
 * Default cache service instance
 */
export const cacheService = new CacheService();

export default CacheService;
