// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Services Index
 *
 * Central export for all service modules.
 * Services encapsulate business logic and external integrations.
 */

export { CacheService, createCacheService, MemoryCache, RedisCache, CacheKeys, CacheTags } from './cache.service';
export { SongService } from './song.service';
export { PlayService } from './play.service';

export type { CacheOptions, CacheProvider } from './cache.service';
export type { SongServiceConfig } from './song.service';
export type { PlayServiceConfig, DailyStats, HourlyStats } from './play.service';
