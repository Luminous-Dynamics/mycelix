// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Search Service
 *
 * Full-text search and autocomplete for songs, artists, and playlists.
 * Uses PostgreSQL full-text search with trigram similarity.
 */

import { Pool } from 'pg';

/**
 * Search result types
 */
export interface SearchResult {
  id: string;
  type: 'song' | 'artist' | 'playlist' | 'genre';
  title: string;
  subtitle?: string;
  imageUrl?: string;
  rank: number;
  highlight?: string;
  metadata?: Record<string, unknown>;
}

export interface SearchResponse {
  results: SearchResult[];
  total: number;
  query: string;
  took: number;
  suggestions?: string[];
}

export interface AutocompleteResult {
  value: string;
  type: 'song' | 'artist' | 'genre';
  count?: number;
}

/**
 * Search options
 */
export interface SearchOptions {
  types?: ('song' | 'artist' | 'playlist' | 'genre')[];
  limit?: number;
  offset?: number;
  genre?: string;
  minPlays?: number;
  sortBy?: 'relevance' | 'plays' | 'recent';
}

/**
 * Search Service
 */
export class SearchService {
  constructor(private pool: Pool) {}

  /**
   * Full-text search across all content
   */
  async search(query: string, options: SearchOptions = {}): Promise<SearchResponse> {
    const startTime = Date.now();
    const {
      types = ['song', 'artist'],
      limit = 20,
      offset = 0,
      genre,
      minPlays = 0,
      sortBy = 'relevance',
    } = options;

    // Sanitize and prepare query
    const sanitizedQuery = this.sanitizeQuery(query);
    const tsQuery = this.toTsQuery(sanitizedQuery);

    const results: SearchResult[] = [];
    let total = 0;

    // Search songs
    if (types.includes('song')) {
      const songResults = await this.searchSongs(tsQuery, sanitizedQuery, {
        limit,
        offset,
        genre,
        minPlays,
        sortBy,
      });
      results.push(...songResults.results);
      total += songResults.total;
    }

    // Search artists
    if (types.includes('artist')) {
      const artistResults = await this.searchArtists(tsQuery, sanitizedQuery, {
        limit,
        offset,
      });
      results.push(...artistResults.results);
      total += artistResults.total;
    }

    // Sort by rank
    results.sort((a, b) => b.rank - a.rank);

    // Get suggestions for low results
    let suggestions: string[] | undefined;
    if (results.length < 3) {
      suggestions = await this.getSuggestions(query);
    }

    return {
      results: results.slice(0, limit),
      total,
      query,
      took: Date.now() - startTime,
      suggestions,
    };
  }

  /**
   * Search songs
   */
  private async searchSongs(
    tsQuery: string,
    rawQuery: string,
    options: {
      limit: number;
      offset: number;
      genre?: string;
      minPlays: number;
      sortBy: string;
    }
  ): Promise<{ results: SearchResult[]; total: number }> {
    const { limit, offset, genre, minPlays, sortBy } = options;

    // Build the query using both full-text search and trigram similarity
    const sql = `
      WITH ranked_songs AS (
        SELECT
          s.id,
          s.title,
          s.artist_name,
          s.genre,
          s.play_count,
          s.image_url,
          s.created_at,
          -- Full-text search rank
          COALESCE(
            ts_rank_cd(
              to_tsvector('english', s.title || ' ' || COALESCE(s.artist_name, '')),
              to_tsquery('english', $1)
            ),
            0
          ) AS ts_rank,
          -- Trigram similarity for fuzzy matching
          GREATEST(
            similarity(s.title, $2),
            similarity(COALESCE(s.artist_name, ''), $2)
          ) AS sim_rank,
          -- Headline with highlights
          ts_headline(
            'english',
            s.title,
            to_tsquery('english', $1),
            'StartSel=<mark>, StopSel=</mark>, MaxWords=20, MinWords=10'
          ) AS highlight
        FROM songs s
        WHERE
          s.deleted_at IS NULL
          AND (
            to_tsvector('english', s.title || ' ' || COALESCE(s.artist_name, ''))
            @@ to_tsquery('english', $1)
            OR similarity(s.title, $2) > 0.1
            OR similarity(COALESCE(s.artist_name, ''), $2) > 0.1
          )
          ${genre ? 'AND s.genre = $5' : ''}
          AND s.play_count >= $3
      )
      SELECT
        *,
        (ts_rank * 2 + sim_rank + (play_count::float / 10000)) AS combined_rank,
        COUNT(*) OVER() AS total_count
      FROM ranked_songs
      ORDER BY
        ${sortBy === 'plays' ? 'play_count DESC' :
          sortBy === 'recent' ? 'created_at DESC' :
          'combined_rank DESC'}
      LIMIT $4 OFFSET ${offset}
    `;

    const params: unknown[] = [tsQuery, rawQuery, minPlays, limit];
    if (genre) params.push(genre);

    try {
      const result = await this.pool.query(sql, params);

      const results: SearchResult[] = result.rows.map(row => ({
        id: row.id,
        type: 'song' as const,
        title: row.title,
        subtitle: row.artist_name,
        imageUrl: row.image_url,
        rank: parseFloat(row.combined_rank) || 0,
        highlight: row.highlight,
        metadata: {
          genre: row.genre,
          playCount: row.play_count,
        },
      }));

      return {
        results,
        total: parseInt(result.rows[0]?.total_count || '0', 10),
      };
    } catch (error) {
      // Fallback to simple LIKE search if full-text fails
      return this.fallbackSongSearch(rawQuery, options);
    }
  }

  /**
   * Fallback song search using LIKE
   */
  private async fallbackSongSearch(
    query: string,
    options: { limit: number; offset: number; genre?: string; minPlays: number }
  ): Promise<{ results: SearchResult[]; total: number }> {
    const { limit, offset, genre, minPlays } = options;
    const likeQuery = `%${query}%`;

    const sql = `
      SELECT
        id, title, artist_name, genre, play_count, image_url,
        COUNT(*) OVER() AS total_count
      FROM songs
      WHERE
        deleted_at IS NULL
        AND (
          title ILIKE $1
          OR artist_name ILIKE $1
        )
        ${genre ? 'AND genre = $4' : ''}
        AND play_count >= $2
      ORDER BY play_count DESC
      LIMIT $3 OFFSET ${offset}
    `;

    const params: unknown[] = [likeQuery, minPlays, limit];
    if (genre) params.push(genre);

    const result = await this.pool.query(sql, params);

    return {
      results: result.rows.map(row => ({
        id: row.id,
        type: 'song' as const,
        title: row.title,
        subtitle: row.artist_name,
        imageUrl: row.image_url,
        rank: row.play_count / 1000,
        metadata: { genre: row.genre, playCount: row.play_count },
      })),
      total: parseInt(result.rows[0]?.total_count || '0', 10),
    };
  }

  /**
   * Search artists
   */
  private async searchArtists(
    tsQuery: string,
    rawQuery: string,
    options: { limit: number; offset: number }
  ): Promise<{ results: SearchResult[]; total: number }> {
    const { limit, offset } = options;

    const sql = `
      SELECT
        artist_address AS id,
        artist_name AS name,
        COUNT(*) AS song_count,
        SUM(play_count) AS total_plays,
        MAX(similarity(artist_name, $2)) AS sim_rank,
        COUNT(*) OVER() AS total_count
      FROM songs
      WHERE
        deleted_at IS NULL
        AND artist_name IS NOT NULL
        AND (
          to_tsvector('english', artist_name) @@ to_tsquery('english', $1)
          OR similarity(artist_name, $2) > 0.2
        )
      GROUP BY artist_address, artist_name
      ORDER BY sim_rank DESC, total_plays DESC
      LIMIT $3 OFFSET $4
    `;

    try {
      const result = await this.pool.query(sql, [tsQuery, rawQuery, limit, offset]);

      return {
        results: result.rows.map(row => ({
          id: row.id,
          type: 'artist' as const,
          title: row.name,
          subtitle: `${row.song_count} songs`,
          rank: parseFloat(row.sim_rank) || 0,
          metadata: {
            songCount: parseInt(row.song_count, 10),
            totalPlays: parseInt(row.total_plays, 10),
          },
        })),
        total: parseInt(result.rows[0]?.total_count || '0', 10),
      };
    } catch {
      return { results: [], total: 0 };
    }
  }

  /**
   * Autocomplete suggestions
   */
  async autocomplete(prefix: string, limit = 10): Promise<AutocompleteResult[]> {
    const sanitizedPrefix = this.sanitizeQuery(prefix);
    if (sanitizedPrefix.length < 2) return [];

    const likePrefix = `${sanitizedPrefix}%`;

    // Get song title suggestions
    const songsSql = `
      SELECT DISTINCT title AS value, 'song' AS type, play_count
      FROM songs
      WHERE
        deleted_at IS NULL
        AND title ILIKE $1
      ORDER BY play_count DESC
      LIMIT $2
    `;

    // Get artist suggestions
    const artistsSql = `
      SELECT
        artist_name AS value,
        'artist' AS type,
        COUNT(*) AS count
      FROM songs
      WHERE
        deleted_at IS NULL
        AND artist_name ILIKE $1
      GROUP BY artist_name
      ORDER BY count DESC
      LIMIT $2
    `;

    // Get genre suggestions
    const genresSql = `
      SELECT DISTINCT genre AS value, 'genre' AS type, COUNT(*) AS count
      FROM songs
      WHERE
        deleted_at IS NULL
        AND genre ILIKE $1
      GROUP BY genre
      ORDER BY count DESC
      LIMIT 5
    `;

    const [songsResult, artistsResult, genresResult] = await Promise.all([
      this.pool.query(songsSql, [likePrefix, limit]),
      this.pool.query(artistsSql, [likePrefix, limit]),
      this.pool.query(genresSql, [likePrefix]),
    ]);

    const results: AutocompleteResult[] = [
      ...genresResult.rows.map(row => ({
        value: row.value,
        type: 'genre' as const,
        count: parseInt(row.count, 10),
      })),
      ...artistsResult.rows.map(row => ({
        value: row.value,
        type: 'artist' as const,
        count: parseInt(row.count, 10),
      })),
      ...songsResult.rows.map(row => ({
        value: row.value,
        type: 'song' as const,
      })),
    ];

    return results.slice(0, limit);
  }

  /**
   * Get search suggestions (for did-you-mean)
   */
  async getSuggestions(query: string): Promise<string[]> {
    const sql = `
      SELECT DISTINCT title
      FROM songs
      WHERE
        deleted_at IS NULL
        AND similarity(title, $1) > 0.3
      ORDER BY similarity(title, $1) DESC
      LIMIT 5
    `;

    try {
      const result = await this.pool.query(sql, [query]);
      return result.rows.map(row => row.title);
    } catch {
      return [];
    }
  }

  /**
   * Get trending searches
   */
  async getTrendingSearches(limit = 10): Promise<string[]> {
    // In a real implementation, this would query a search_logs table
    // For now, return top played song titles
    const sql = `
      SELECT title
      FROM songs
      WHERE deleted_at IS NULL
      ORDER BY play_count DESC
      LIMIT $1
    `;

    const result = await this.pool.query(sql, [limit]);
    return result.rows.map(row => row.title);
  }

  /**
   * Search by genre
   */
  async searchByGenre(genre: string, options: { limit?: number; offset?: number } = {}): Promise<SearchResult[]> {
    const { limit = 20, offset = 0 } = options;

    const sql = `
      SELECT id, title, artist_name, genre, play_count, image_url
      FROM songs
      WHERE
        deleted_at IS NULL
        AND genre ILIKE $1
      ORDER BY play_count DESC
      LIMIT $2 OFFSET $3
    `;

    const result = await this.pool.query(sql, [genre, limit, offset]);

    return result.rows.map(row => ({
      id: row.id,
      type: 'song' as const,
      title: row.title,
      subtitle: row.artist_name,
      imageUrl: row.image_url,
      rank: row.play_count / 1000,
      metadata: { genre: row.genre, playCount: row.play_count },
    }));
  }

  /**
   * Get all genres
   */
  async getGenres(): Promise<{ genre: string; count: number }[]> {
    const sql = `
      SELECT genre, COUNT(*) AS count
      FROM songs
      WHERE deleted_at IS NULL AND genre IS NOT NULL
      GROUP BY genre
      ORDER BY count DESC
    `;

    const result = await this.pool.query(sql);
    return result.rows.map(row => ({
      genre: row.genre,
      count: parseInt(row.count, 10),
    }));
  }

  /**
   * Sanitize search query
   */
  private sanitizeQuery(query: string): string {
    return query
      .trim()
      .replace(/[^\w\s\-']/g, '')
      .slice(0, 100);
  }

  /**
   * Convert query to PostgreSQL tsquery
   */
  private toTsQuery(query: string): string {
    const words = query
      .split(/\s+/)
      .filter(word => word.length > 1)
      .map(word => word.replace(/'/g, "''"));

    if (words.length === 0) return '';

    // Use prefix matching for better partial matching
    return words.map(word => `${word}:*`).join(' & ');
  }
}

/**
 * Migration SQL for enabling trigram extension
 */
export const SEARCH_MIGRATION_SQL = `
-- Enable trigram extension for fuzzy search
CREATE EXTENSION IF NOT EXISTS pg_trgm;

-- Create GIN index for full-text search
CREATE INDEX IF NOT EXISTS idx_songs_fts ON songs
USING GIN(to_tsvector('english', title || ' ' || COALESCE(artist_name, '')));

-- Create trigram indexes for fuzzy matching
CREATE INDEX IF NOT EXISTS idx_songs_title_trgm ON songs
USING GIN(title gin_trgm_ops);

CREATE INDEX IF NOT EXISTS idx_songs_artist_name_trgm ON songs
USING GIN(artist_name gin_trgm_ops);

-- Create index for genre filtering
CREATE INDEX IF NOT EXISTS idx_songs_genre ON songs(genre)
WHERE deleted_at IS NULL;
`;

export default SearchService;
