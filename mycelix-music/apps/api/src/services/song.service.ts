// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Song Service
 *
 * Business logic layer for song operations.
 * Coordinates between repositories, cache, and external services.
 */

import { SongRepository, Song, PaginatedResult } from '../repositories';
import { CacheService, CacheKeys, CacheTags } from './cache.service';
import { CreateSongInput, UpdateSongInput, SongQueryParams } from '../schemas';
import { AppError } from '../middleware/error-handler';

/**
 * Song service configuration
 */
export interface SongServiceConfig {
  defaultPageSize: number;
  maxPageSize: number;
  cacheTtl: number;
}

const DEFAULT_CONFIG: SongServiceConfig = {
  defaultPageSize: 20,
  maxPageSize: 100,
  cacheTtl: 300, // 5 minutes
};

/**
 * Song service class
 */
export class SongService {
  constructor(
    private readonly songRepo: SongRepository,
    private readonly cache: CacheService,
    private readonly config: SongServiceConfig = DEFAULT_CONFIG
  ) {}

  /**
   * Get a song by ID
   */
  async getSongById(id: string): Promise<Song> {
    // Try cache first
    const cached = await this.cache.get<Song>(CacheKeys.song(id));
    if (cached) {
      return cached;
    }

    // Fetch from database
    const song = await this.songRepo.findById(id);
    if (!song) {
      throw AppError.notFound('Song');
    }

    // Cache for future requests
    await this.cache.set(CacheKeys.song(id), song, {
      ttl: this.config.cacheTtl,
      tags: [CacheTags.SONGS],
    });

    return song;
  }

  /**
   * Get multiple songs by IDs
   */
  async getSongsByIds(ids: string[]): Promise<Song[]> {
    if (ids.length === 0) return [];

    // Check cache for each ID
    const results: Song[] = [];
    const uncachedIds: string[] = [];

    for (const id of ids) {
      const cached = await this.cache.get<Song>(CacheKeys.song(id));
      if (cached) {
        results.push(cached);
      } else {
        uncachedIds.push(id);
      }
    }

    // Fetch uncached songs from database
    if (uncachedIds.length > 0) {
      const songs = await this.songRepo.findByIds(uncachedIds);

      // Cache and add to results
      for (const song of songs) {
        await this.cache.set(CacheKeys.song(song.id), song, {
          ttl: this.config.cacheTtl,
          tags: [CacheTags.SONGS],
        });
        results.push(song);
      }
    }

    // Return in original order
    const songMap = new Map(results.map((s) => [s.id, s]));
    return ids.map((id) => songMap.get(id)).filter((s): s is Song => s !== undefined);
  }

  /**
   * Search and list songs
   */
  async searchSongs(params: SongQueryParams): Promise<PaginatedResult<Song>> {
    const limit = Math.min(params.limit || this.config.defaultPageSize, this.config.maxPageSize);
    const offset = params.offset || 0;

    // Build search options
    const searchOptions = {
      query: params.q,
      genre: params.genre,
      artistAddress: params.artist_address,
      paymentModel: params.payment_model,
    };

    // Fetch from repository
    const result = await this.songRepo.search(searchOptions, limit, offset);

    return result;
  }

  /**
   * Get songs by artist
   */
  async getSongsByArtist(artistAddress: string): Promise<Song[]> {
    return this.songRepo.findByArtist(artistAddress);
  }

  /**
   * Get songs by genre
   */
  async getSongsByGenre(genre: string, limit = 50): Promise<Song[]> {
    return this.songRepo.findByGenre(genre, limit);
  }

  /**
   * Get top songs by plays
   */
  async getTopSongs(limit = 10): Promise<Song[]> {
    const cacheKey = CacheKeys.topSongs(limit);

    // Try cache
    const cached = await this.cache.get<Song[]>(cacheKey);
    if (cached) {
      return cached;
    }

    // Fetch from database
    const songs = await this.songRepo.getTopByPlays(limit);

    // Cache with shorter TTL (changes frequently)
    await this.cache.set(cacheKey, songs, {
      ttl: 60, // 1 minute
      tags: [CacheTags.SONGS, CacheTags.ANALYTICS],
    });

    return songs;
  }

  /**
   * Get recently added songs
   */
  async getRecentSongs(limit = 10): Promise<Song[]> {
    return this.songRepo.getRecent(limit);
  }

  /**
   * Create a new song
   */
  async createSong(input: CreateSongInput): Promise<Song> {
    // Check for duplicate IPFS hash
    const existing = await this.songRepo.findOne({ ipfs_hash: input.ipfs_hash });
    if (existing) {
      throw AppError.conflict('A song with this IPFS hash already exists');
    }

    // Create the song
    const song = await this.songRepo.create({
      ...input,
      plays: 0,
      earnings: '0',
    });

    // Invalidate relevant caches
    await this.invalidateSongCaches();

    return song;
  }

  /**
   * Update a song
   */
  async updateSong(id: string, input: UpdateSongInput): Promise<Song> {
    // Verify song exists
    const existing = await this.songRepo.findById(id);
    if (!existing) {
      throw AppError.notFound('Song');
    }

    // Update the song
    const updated = await this.songRepo.update(id, input);
    if (!updated) {
      throw AppError.notFound('Song');
    }

    // Invalidate caches
    await this.cache.delete(CacheKeys.song(id));
    await this.invalidateSongCaches();

    return updated;
  }

  /**
   * Delete a song
   */
  async deleteSong(id: string): Promise<void> {
    const song = await this.songRepo.findById(id);
    if (!song) {
      throw AppError.notFound('Song');
    }

    // Check if song has plays (might want to prevent deletion)
    if (song.plays > 0) {
      throw AppError.conflict('Cannot delete a song that has been played');
    }

    const deleted = await this.songRepo.delete(id);
    if (!deleted) {
      throw AppError.notFound('Song');
    }

    // Invalidate caches
    await this.cache.delete(CacheKeys.song(id));
    await this.invalidateSongCaches();
  }

  /**
   * Record a play and update earnings
   */
  async recordPlay(songId: string, earnings: string): Promise<Song> {
    const song = await this.songRepo.recordPlay(songId, earnings);
    if (!song) {
      throw AppError.notFound('Song');
    }

    // Invalidate caches
    await this.cache.delete(CacheKeys.song(songId));
    await this.cache.invalidateTag(CacheTags.ANALYTICS);

    return song;
  }

  /**
   * Set claim stream ID after blockchain registration
   */
  async setClaimStreamId(songId: string, claimStreamId: string): Promise<Song> {
    const song = await this.songRepo.setClaimStreamId(songId, claimStreamId);
    if (!song) {
      throw AppError.notFound('Song');
    }

    // Invalidate cache
    await this.cache.delete(CacheKeys.song(songId));

    return song;
  }

  /**
   * Get artist statistics
   */
  async getArtistStats(artistAddress: string): Promise<{
    total_songs: number;
    total_plays: number;
    total_earnings: string;
  }> {
    const cacheKey = CacheKeys.artistStats(artistAddress);

    // Try cache
    const cached = await this.cache.get<{
      total_songs: number;
      total_plays: number;
      total_earnings: string;
    }>(cacheKey);
    if (cached) {
      return cached;
    }

    // Fetch from database
    const stats = await this.songRepo.getArtistStats(artistAddress);
    if (!stats) {
      return {
        total_songs: 0,
        total_plays: 0,
        total_earnings: '0',
      };
    }

    // Cache
    await this.cache.set(cacheKey, stats, {
      ttl: this.config.cacheTtl,
      tags: [CacheTags.ARTISTS, CacheTags.ANALYTICS],
    });

    return stats;
  }

  /**
   * Get genre statistics
   */
  async getGenreStats(): Promise<{ genre: string; count: number; total_plays: number }[]> {
    const cacheKey = CacheKeys.genreStats();

    // Try cache
    const cached = await this.cache.get<{ genre: string; count: number; total_plays: number }[]>(cacheKey);
    if (cached) {
      return cached;
    }

    // Fetch from database
    const stats = await this.songRepo.getGenreStats();

    // Cache
    await this.cache.set(cacheKey, stats, {
      ttl: 3600, // 1 hour (doesn't change often)
      tags: [CacheTags.ANALYTICS],
    });

    return stats;
  }

  /**
   * Invalidate song-related caches
   */
  private async invalidateSongCaches(): Promise<void> {
    await this.cache.invalidateTag(CacheTags.SONGS);
  }
}
