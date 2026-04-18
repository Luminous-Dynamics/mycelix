// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Cache Service
 *
 * Provides a unified caching abstraction supporting:
 * - Redis (for distributed caching)
 * - In-memory (for development/testing)
 * - Stale-while-revalidate pattern
 * - Cache tags for invalidation
 */

import Redis from 'ioredis';

/**
 * Cache options
 */
export interface CacheOptions {
  ttl?: number;           // Time to live in seconds
  tags?: string[];        // Tags for grouped invalidation
  staleTime?: number;     // Stale-while-revalidate window in seconds
}

/**
 * Cached value with metadata
 */
interface CachedValue<T> {
  value: T;
  expiresAt: number;
  staleAt?: number;
  tags?: string[];
}

/**
 * Cache interface for different implementations
 */
export interface CacheProvider {
  get<T>(key: string): Promise<T | null>;
  set<T>(key: string, value: T, options?: CacheOptions): Promise<void>;
  delete(key: string): Promise<void>;
  deleteByTag(tag: string): Promise<void>;
  clear(): Promise<void>;
  has(key: string): Promise<boolean>;
}

/**
 * In-memory cache implementation
 */
export class MemoryCache implements CacheProvider {
  private store = new Map<string, CachedValue<unknown>>();
  private tagIndex = new Map<string, Set<string>>();

  async get<T>(key: string): Promise<T | null> {
    const cached = this.store.get(key);

    if (!cached) return null;

    // Check if expired
    if (Date.now() > cached.expiresAt) {
      this.store.delete(key);
      return null;
    }

    return cached.value as T;
  }

  async set<T>(key: string, value: T, options: CacheOptions = {}): Promise<void> {
    const ttl = options.ttl || 300; // Default 5 minutes
    const now = Date.now();

    const cached: CachedValue<T> = {
      value,
      expiresAt: now + ttl * 1000,
      staleAt: options.staleTime ? now + options.staleTime * 1000 : undefined,
      tags: options.tags,
    };

    this.store.set(key, cached);

    // Index by tags
    if (options.tags) {
      for (const tag of options.tags) {
        if (!this.tagIndex.has(tag)) {
          this.tagIndex.set(tag, new Set());
        }
        this.tagIndex.get(tag)!.add(key);
      }
    }
  }

  async delete(key: string): Promise<void> {
    const cached = this.store.get(key);
    if (cached?.tags) {
      for (const tag of cached.tags) {
        this.tagIndex.get(tag)?.delete(key);
      }
    }
    this.store.delete(key);
  }

  async deleteByTag(tag: string): Promise<void> {
    const keys = this.tagIndex.get(tag);
    if (keys) {
      for (const key of keys) {
        this.store.delete(key);
      }
      this.tagIndex.delete(tag);
    }
  }

  async clear(): Promise<void> {
    this.store.clear();
    this.tagIndex.clear();
  }

  async has(key: string): Promise<boolean> {
    const value = await this.get(key);
    return value !== null;
  }

  /**
   * Check if value is stale (for SWR pattern)
   */
  isStale(key: string): boolean {
    const cached = this.store.get(key);
    if (!cached || !cached.staleAt) return false;
    return Date.now() > cached.staleAt;
  }
}

/**
 * Redis cache implementation
 */
export class RedisCache implements CacheProvider {
  private client: Redis;
  private prefix: string;

  constructor(client: Redis, prefix = 'cache:') {
    this.client = client;
    this.prefix = prefix;
  }

  private key(key: string): string {
    return `${this.prefix}${key}`;
  }

  private tagKey(tag: string): string {
    return `${this.prefix}tag:${tag}`;
  }

  async get<T>(key: string): Promise<T | null> {
    const data = await this.client.get(this.key(key));
    if (!data) return null;

    try {
      const cached: CachedValue<T> = JSON.parse(data);

      // Check if expired (Redis handles TTL, but check for safety)
      if (Date.now() > cached.expiresAt) {
        await this.delete(key);
        return null;
      }

      return cached.value;
    } catch {
      return null;
    }
  }

  async set<T>(key: string, value: T, options: CacheOptions = {}): Promise<void> {
    const ttl = options.ttl || 300;
    const now = Date.now();

    const cached: CachedValue<T> = {
      value,
      expiresAt: now + ttl * 1000,
      staleAt: options.staleTime ? now + options.staleTime * 1000 : undefined,
      tags: options.tags,
    };

    const pipeline = this.client.pipeline();

    // Set the value with TTL
    pipeline.setex(this.key(key), ttl, JSON.stringify(cached));

    // Add to tag sets
    if (options.tags) {
      for (const tag of options.tags) {
        pipeline.sadd(this.tagKey(tag), key);
        pipeline.expire(this.tagKey(tag), ttl + 60); // Keep tag set a bit longer
      }
    }

    await pipeline.exec();
  }

  async delete(key: string): Promise<void> {
    // Get cached value to find tags
    const data = await this.client.get(this.key(key));
    if (data) {
      try {
        const cached = JSON.parse(data);
        if (cached.tags) {
          const pipeline = this.client.pipeline();
          for (const tag of cached.tags) {
            pipeline.srem(this.tagKey(tag), key);
          }
          await pipeline.exec();
        }
      } catch {
        // Ignore parse errors
      }
    }

    await this.client.del(this.key(key));
  }

  async deleteByTag(tag: string): Promise<void> {
    const keys = await this.client.smembers(this.tagKey(tag));

    if (keys.length > 0) {
      const pipeline = this.client.pipeline();
      for (const key of keys) {
        pipeline.del(this.key(key));
      }
      pipeline.del(this.tagKey(tag));
      await pipeline.exec();
    }
  }

  async clear(): Promise<void> {
    const keys = await this.client.keys(`${this.prefix}*`);
    if (keys.length > 0) {
      await this.client.del(...keys);
    }
  }

  async has(key: string): Promise<boolean> {
    const exists = await this.client.exists(this.key(key));
    return exists === 1;
  }

  /**
   * Check if value is stale
   */
  async isStale(key: string): Promise<boolean> {
    const data = await this.client.get(this.key(key));
    if (!data) return false;

    try {
      const cached = JSON.parse(data);
      if (!cached.staleAt) return false;
      return Date.now() > cached.staleAt;
    } catch {
      return false;
    }
  }
}

/**
 * Cache service with high-level operations
 */
export class CacheService {
  private provider: CacheProvider;

  constructor(provider: CacheProvider) {
    this.provider = provider;
  }

  /**
   * Get a value from cache
   */
  async get<T>(key: string): Promise<T | null> {
    return this.provider.get<T>(key);
  }

  /**
   * Set a value in cache
   */
  async set<T>(key: string, value: T, options?: CacheOptions): Promise<void> {
    return this.provider.set(key, value, options);
  }

  /**
   * Delete a value from cache
   */
  async delete(key: string): Promise<void> {
    return this.provider.delete(key);
  }

  /**
   * Delete all values with a tag
   */
  async invalidateTag(tag: string): Promise<void> {
    return this.provider.deleteByTag(tag);
  }

  /**
   * Clear all cache
   */
  async clear(): Promise<void> {
    return this.provider.clear();
  }

  /**
   * Get or compute a value (cache-aside pattern)
   */
  async getOrSet<T>(
    key: string,
    factory: () => Promise<T>,
    options?: CacheOptions
  ): Promise<T> {
    const cached = await this.get<T>(key);
    if (cached !== null) {
      return cached;
    }

    const value = await factory();
    await this.set(key, value, options);
    return value;
  }

  /**
   * Memoize a function
   */
  memoize<TArgs extends unknown[], TResult>(
    fn: (...args: TArgs) => Promise<TResult>,
    keyFn: (...args: TArgs) => string,
    options?: CacheOptions
  ): (...args: TArgs) => Promise<TResult> {
    return async (...args: TArgs): Promise<TResult> => {
      const key = keyFn(...args);
      return this.getOrSet(key, () => fn(...args), options);
    };
  }
}

/**
 * Create cache service based on environment
 */
export function createCacheService(redisClient?: Redis): CacheService {
  if (redisClient) {
    return new CacheService(new RedisCache(redisClient));
  }
  return new CacheService(new MemoryCache());
}

/**
 * Cache key generators
 */
export const CacheKeys = {
  song: (id: string) => `song:${id}`,
  songs: (filters: string) => `songs:${filters}`,
  artistStats: (address: string) => `artist:${address}:stats`,
  topSongs: (limit: number) => `top:songs:${limit}`,
  recentPlays: (limit: number) => `recent:plays:${limit}`,
  genreStats: () => 'genre:stats',
};

/**
 * Cache tags for invalidation
 */
export const CacheTags = {
  SONGS: 'songs',
  PLAYS: 'plays',
  ARTISTS: 'artists',
  ANALYTICS: 'analytics',
};
