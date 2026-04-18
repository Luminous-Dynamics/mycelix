// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Play Service
 *
 * Business logic layer for play/streaming operations.
 * Handles play recording, confirmation, and analytics.
 */

import { PlayRepository, Play, PlayWithSong, TimeRange } from '../repositories';
import { SongRepository } from '../repositories';
import { CacheService, CacheKeys, CacheTags } from './cache.service';
import { CreatePlayInput } from '../schemas';
import { AppError } from '../middleware/error-handler';

/**
 * Play service configuration
 */
export interface PlayServiceConfig {
  maxPendingAge: number; // Max age of pending plays in minutes
  recentPlaysLimit: number;
  cacheTtl: number;
}

const DEFAULT_CONFIG: PlayServiceConfig = {
  maxPendingAge: 30,
  recentPlaysLimit: 20,
  cacheTtl: 60, // 1 minute (plays are dynamic)
};

/**
 * Daily statistics structure
 */
export interface DailyStats {
  date: string;
  plays: number;
  earnings: string;
  unique_listeners: number;
}

/**
 * Hourly statistics structure
 */
export interface HourlyStats {
  hour: string;
  plays: number;
  earnings: string;
}

/**
 * Play service class
 */
export class PlayService {
  constructor(
    private readonly playRepo: PlayRepository,
    private readonly songRepo: SongRepository,
    private readonly cache: CacheService,
    private readonly config: PlayServiceConfig = DEFAULT_CONFIG
  ) {}

  /**
   * Record a new play
   */
  async recordPlay(input: CreatePlayInput): Promise<{ play: Play; song: any }> {
    // Verify the song exists
    const song = await this.songRepo.findById(input.song_id);
    if (!song) {
      throw AppError.notFound('Song');
    }

    // Use transaction to ensure consistency
    const result = await this.playRepo.transaction(async (client) => {
      // Create play record
      const play = await this.playRepo.createPlay(input, client);

      // Update song play count and earnings
      const updatedSong = await this.songRepo.recordPlay(
        input.song_id,
        input.amount,
        client
      );

      return { play, song: updatedSong };
    });

    // Invalidate caches
    await this.invalidatePlayCaches(input.song_id);

    return result;
  }

  /**
   * Confirm a play after blockchain confirmation
   */
  async confirmPlay(playId: string, transactionHash: string): Promise<Play> {
    const play = await this.playRepo.confirmPlay(playId, transactionHash);
    if (!play) {
      throw AppError.notFound('Play');
    }

    // Invalidate caches
    await this.invalidatePlayCaches(play.song_id);

    return play;
  }

  /**
   * Mark a play as failed
   */
  async failPlay(playId: string): Promise<Play> {
    const play = await this.playRepo.failPlay(playId);
    if (!play) {
      throw AppError.notFound('Play');
    }

    return play;
  }

  /**
   * Get plays for a song
   */
  async getPlaysForSong(
    songId: string,
    limit = 50,
    offset = 0
  ): Promise<Play[]> {
    // Verify song exists
    const song = await this.songRepo.findById(songId);
    if (!song) {
      throw AppError.notFound('Song');
    }

    return this.playRepo.findBySong(songId, limit, offset);
  }

  /**
   * Get plays by listener
   */
  async getPlaysByListener(
    listenerAddress: string,
    limit = 50,
    offset = 0
  ): Promise<PlayWithSong[]> {
    return this.playRepo.findByListener(listenerAddress, limit, offset);
  }

  /**
   * Get recent plays (global feed)
   */
  async getRecentPlays(limit?: number): Promise<PlayWithSong[]> {
    const actualLimit = limit || this.config.recentPlaysLimit;
    const cacheKey = CacheKeys.recentPlays(actualLimit);

    // Try cache
    const cached = await this.cache.get<PlayWithSong[]>(cacheKey);
    if (cached) {
      return cached;
    }

    // Fetch from database
    const plays = await this.playRepo.getRecentPlays(actualLimit);

    // Cache with short TTL
    await this.cache.set(cacheKey, plays, {
      ttl: 30, // 30 seconds
      tags: [CacheTags.PLAYS],
    });

    return plays;
  }

  /**
   * Get play count for a song in a time range
   */
  async getPlayCount(songId: string, range?: TimeRange): Promise<number> {
    return this.playRepo.countPlaysForSong(songId, range);
  }

  /**
   * Get artist earnings in a time range
   */
  async getArtistEarnings(
    artistAddress: string,
    range: TimeRange
  ): Promise<string> {
    return this.playRepo.getArtistEarningsInRange(artistAddress, range);
  }

  /**
   * Get daily statistics
   */
  async getDailyStats(days = 30): Promise<DailyStats[]> {
    return this.playRepo.getDailyStats(days);
  }

  /**
   * Get hourly statistics (last 24 hours)
   */
  async getHourlyStats(): Promise<HourlyStats[]> {
    return this.playRepo.getHourlyStats();
  }

  /**
   * Get top listeners
   */
  async getTopListeners(limit = 10): Promise<{
    listener_address: string;
    play_count: number;
    total_spent: string;
  }[]> {
    return this.playRepo.getTopListeners(limit);
  }

  /**
   * Get statistics summary
   */
  async getStatsSummary(): Promise<{
    today: DailyStats | null;
    yesterday: DailyStats | null;
    thisWeek: { plays: number; earnings: string };
    thisMonth: { plays: number; earnings: string };
  }> {
    const dailyStats = await this.getDailyStats(30);

    const today = dailyStats[0] || null;
    const yesterday = dailyStats[1] || null;

    // Calculate week totals (last 7 days)
    const weekStats = dailyStats.slice(0, 7);
    const thisWeek = {
      plays: weekStats.reduce((sum, d) => sum + d.plays, 0),
      earnings: weekStats.reduce((sum, d) => sum + parseFloat(d.earnings), 0).toFixed(6),
    };

    // Calculate month totals
    const thisMonth = {
      plays: dailyStats.reduce((sum, d) => sum + d.plays, 0),
      earnings: dailyStats.reduce((sum, d) => sum + parseFloat(d.earnings), 0).toFixed(6),
    };

    return { today, yesterday, thisWeek, thisMonth };
  }

  /**
   * Process pending plays (for cleanup/retry job)
   */
  async processPendingPlays(): Promise<{
    processed: number;
    failed: number;
  }> {
    const pendingPlays = await this.playRepo.getPendingPlays(this.config.maxPendingAge);

    let processed = 0;
    let failed = 0;

    for (const play of pendingPlays) {
      try {
        // Here you would check blockchain for confirmation
        // For now, we'll mark old pending plays as failed
        await this.failPlay(play.id);
        failed++;
      } catch (error) {
        console.error(`Failed to process pending play ${play.id}:`, error);
      }
    }

    return { processed, failed };
  }

  /**
   * Invalidate play-related caches
   */
  private async invalidatePlayCaches(songId?: string): Promise<void> {
    await this.cache.invalidateTag(CacheTags.PLAYS);
    await this.cache.invalidateTag(CacheTags.ANALYTICS);

    if (songId) {
      await this.cache.delete(CacheKeys.song(songId));
    }
  }
}
