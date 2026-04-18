// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Resonance Graph
 *
 * Social graph service that models connections between users based on
 * shared music taste, listening behavior, and social interactions.
 * Uses "resonance" as the metric for connection strength.
 */

import { Redis } from 'ioredis';
import { Pool } from 'pg';

// ============================================================================
// Types
// ============================================================================

export interface UserNode {
  id: string;
  displayName: string;
  avatar?: string;
  resonanceScore: number;
  topGenres: string[];
  connectionCount: number;
  joinedAt: Date;
}

export interface Connection {
  userId: string;
  connectedUserId: string;
  resonance: number;          // 0-100 connection strength
  sharedArtists: number;
  sharedTracks: number;
  sharedCircles: number;
  mutualConnections: number;
  interactionScore: number;
  tasteOverlap: number;       // 0-1 how similar their taste is
  lastInteraction: Date;
  connectionType: 'discovered' | 'followed' | 'mutual' | 'circle_mate' | 'collaborator';
}

export interface ResonanceBreakdown {
  total: number;
  components: {
    tasteMatch: number;       // Similar music taste
    interactions: number;     // Direct interactions
    mutuals: number;          // Shared connections
    circleTime: number;       // Time in circles together
    engagement: number;       // Likes, comments, shares
  };
}

export interface SuggestedConnection {
  user: UserNode;
  resonance: ResonanceBreakdown;
  reasons: string[];
  mutualConnections: UserNode[];
  sharedContent: {
    artists: string[];
    tracks: string[];
    circles: string[];
  };
}

// ============================================================================
// Resonance Graph Service
// ============================================================================

export class ResonanceGraphService {
  private redis: Redis;
  private db: Pool;

  constructor(
    private readonly config: {
      redisUrl: string;
      databaseUrl: string;
    }
  ) {
    this.redis = new Redis(config.redisUrl);
    this.db = new Pool({ connectionString: config.databaseUrl });
  }

  // ============================================================================
  // Connection Discovery
  // ============================================================================

  async discoverConnections(userId: string, limit = 20): Promise<SuggestedConnection[]> {
    // Get user's listening profile
    const userProfile = await this.getUserProfile(userId);

    // Find potential connections through multiple signals
    const [
      tasteMatches,
      circleMatches,
      mutualMatches,
      interactionCandidates,
    ] = await Promise.all([
      this.findByTasteMatch(userId, userProfile, limit),
      this.findByCircleActivity(userId, limit),
      this.findByMutualConnections(userId, limit),
      this.findByInteractions(userId, limit),
    ]);

    // Merge and deduplicate candidates
    const candidateMap = new Map<string, SuggestedConnection>();

    const mergeCandidates = (candidates: SuggestedConnection[]) => {
      for (const candidate of candidates) {
        if (candidate.user.id === userId) continue;

        const existing = candidateMap.get(candidate.user.id);
        if (existing) {
          // Merge resonance scores
          existing.resonance.total = Math.max(existing.resonance.total, candidate.resonance.total);
          existing.resonance.components.tasteMatch = Math.max(
            existing.resonance.components.tasteMatch,
            candidate.resonance.components.tasteMatch
          );
          existing.reasons = [...new Set([...existing.reasons, ...candidate.reasons])];
          existing.mutualConnections = [
            ...new Map([
              ...existing.mutualConnections,
              ...candidate.mutualConnections
            ].map(m => [m.id, m])).values()
          ];
        } else {
          candidateMap.set(candidate.user.id, candidate);
        }
      }
    };

    mergeCandidates(tasteMatches);
    mergeCandidates(circleMatches);
    mergeCandidates(mutualMatches);
    mergeCandidates(interactionCandidates);

    // Filter out existing connections
    const existingConnections = await this.getConnectionIds(userId);
    const filteredCandidates = Array.from(candidateMap.values())
      .filter(c => !existingConnections.has(c.user.id));

    // Sort by resonance score
    filteredCandidates.sort((a, b) => b.resonance.total - a.resonance.total);

    return filteredCandidates.slice(0, limit);
  }

  private async findByTasteMatch(
    userId: string,
    profile: UserProfile,
    limit: number
  ): Promise<SuggestedConnection[]> {
    // Find users with similar listening history using vector similarity
    const result = await this.db.query(`
      WITH user_taste AS (
        SELECT
          user_id,
          taste_vector,
          top_artists,
          top_genres,
          1 - (taste_vector <=> $2) as similarity
        FROM user_taste_profiles
        WHERE user_id != $1
          AND array_length(top_genres, 1) > 0
        ORDER BY taste_vector <=> $2
        LIMIT $3
      )
      SELECT
        ut.*,
        u.display_name,
        u.avatar_url,
        u.resonance_score,
        u.created_at
      FROM user_taste ut
      JOIN users u ON ut.user_id = u.id
      WHERE ut.similarity > 0.3
    `, [userId, profile.tasteVector, limit * 2]);

    return result.rows.map(row => this.buildSuggestion(row, {
      tasteMatch: row.similarity * 100,
      interactions: 0,
      mutuals: 0,
      circleTime: 0,
      engagement: 0,
    }, [`${Math.round(row.similarity * 100)}% taste match`, `Both love ${row.top_genres[0]}`]));
  }

  private async findByCircleActivity(userId: string, limit: number): Promise<SuggestedConnection[]> {
    const result = await this.db.query(`
      WITH circle_mates AS (
        SELECT
          cm2.user_id,
          COUNT(DISTINCT c.id) as shared_circles,
          SUM(cm1.listen_time + cm2.listen_time) as total_time
        FROM circle_members cm1
        JOIN circles c ON cm1.circle_id = c.id
        JOIN circle_members cm2 ON c.id = cm2.circle_id
        WHERE cm1.user_id = $1
          AND cm2.user_id != $1
          AND c.created_at > NOW() - INTERVAL '30 days'
        GROUP BY cm2.user_id
        HAVING COUNT(DISTINCT c.id) >= 2
      )
      SELECT
        cm.*,
        u.display_name,
        u.avatar_url,
        u.resonance_score,
        u.created_at
      FROM circle_mates cm
      JOIN users u ON cm.user_id = u.id
      ORDER BY shared_circles DESC, total_time DESC
      LIMIT $2
    `, [userId, limit]);

    return result.rows.map(row => this.buildSuggestion(row, {
      tasteMatch: 0,
      interactions: 0,
      mutuals: 0,
      circleTime: Math.min(100, row.total_time / 60), // hours to score
      engagement: 0,
    }, [
      `Shared ${row.shared_circles} listening circles`,
      `${Math.round(row.total_time / 60)} hours listening together`
    ]));
  }

  private async findByMutualConnections(userId: string, limit: number): Promise<SuggestedConnection[]> {
    const result = await this.db.query(`
      WITH mutuals AS (
        SELECT
          c2.connected_user_id as user_id,
          COUNT(*) as mutual_count,
          ARRAY_AGG(c1.connected_user_id) as mutual_ids
        FROM connections c1
        JOIN connections c2 ON c1.connected_user_id = c2.user_id
        WHERE c1.user_id = $1
          AND c2.connected_user_id != $1
          AND NOT EXISTS (
            SELECT 1 FROM connections
            WHERE user_id = $1 AND connected_user_id = c2.connected_user_id
          )
        GROUP BY c2.connected_user_id
        HAVING COUNT(*) >= 2
      )
      SELECT
        m.*,
        u.display_name,
        u.avatar_url,
        u.resonance_score,
        u.created_at
      FROM mutuals m
      JOIN users u ON m.user_id = u.id
      ORDER BY mutual_count DESC
      LIMIT $2
    `, [userId, limit]);

    return Promise.all(result.rows.map(async row => {
      const mutualUsers = await this.getUserNodes(row.mutual_ids.slice(0, 3));
      return this.buildSuggestion(row, {
        tasteMatch: 0,
        interactions: 0,
        mutuals: Math.min(100, row.mutual_count * 10),
        circleTime: 0,
        engagement: 0,
      }, [
        `${row.mutual_count} mutual connections`,
        `Connected with ${mutualUsers[0]?.displayName || 'friends'}`
      ], mutualUsers);
    }));
  }

  private async findByInteractions(userId: string, limit: number): Promise<SuggestedConnection[]> {
    const result = await this.db.query(`
      WITH interactions AS (
        SELECT
          target_user_id as user_id,
          COUNT(*) as interaction_count,
          MAX(created_at) as last_interaction
        FROM user_interactions
        WHERE user_id = $1
          AND interaction_type IN ('like', 'comment', 'share', 'reply')
          AND NOT EXISTS (
            SELECT 1 FROM connections
            WHERE user_id = $1 AND connected_user_id = target_user_id
          )
        GROUP BY target_user_id
        HAVING COUNT(*) >= 3
      )
      SELECT
        i.*,
        u.display_name,
        u.avatar_url,
        u.resonance_score,
        u.created_at
      FROM interactions i
      JOIN users u ON i.user_id = u.id
      ORDER BY interaction_count DESC, last_interaction DESC
      LIMIT $2
    `, [userId, limit]);

    return result.rows.map(row => this.buildSuggestion(row, {
      tasteMatch: 0,
      interactions: Math.min(100, row.interaction_count * 5),
      mutuals: 0,
      circleTime: 0,
      engagement: Math.min(100, row.interaction_count * 5),
    }, [`You've interacted ${row.interaction_count} times`]));
  }

  // ============================================================================
  // Connection Management
  // ============================================================================

  async connect(userId: string, targetUserId: string): Promise<Connection> {
    // Calculate initial resonance
    const resonance = await this.calculateResonance(userId, targetUserId);

    // Create connection (bidirectional)
    await this.db.query(`
      INSERT INTO connections (user_id, connected_user_id, resonance, connection_type, created_at)
      VALUES ($1, $2, $3, 'followed', NOW())
      ON CONFLICT (user_id, connected_user_id) DO UPDATE
        SET resonance = $3, updated_at = NOW()
    `, [userId, targetUserId, resonance.total]);

    // Check if mutual
    const isMutual = await this.isMutualConnection(userId, targetUserId);
    if (isMutual) {
      await this.db.query(`
        UPDATE connections
        SET connection_type = 'mutual'
        WHERE (user_id = $1 AND connected_user_id = $2)
           OR (user_id = $2 AND connected_user_id = $1)
      `, [userId, targetUserId]);
    }

    // Update resonance in Redis for fast lookup
    await this.redis.zadd(`resonance:${userId}`, resonance.total, targetUserId);
    await this.redis.zadd(`resonance:${targetUserId}`, resonance.total * 0.5, userId);

    return this.getConnection(userId, targetUserId);
  }

  async disconnect(userId: string, targetUserId: string): Promise<void> {
    await this.db.query(`
      DELETE FROM connections
      WHERE user_id = $1 AND connected_user_id = $2
    `, [userId, targetUserId]);

    await this.redis.zrem(`resonance:${userId}`, targetUserId);

    // Update mutual status if it was mutual
    await this.db.query(`
      UPDATE connections
      SET connection_type = 'followed'
      WHERE user_id = $2 AND connected_user_id = $1
        AND connection_type = 'mutual'
    `, [userId, targetUserId]);
  }

  async getConnections(
    userId: string,
    options: {
      limit?: number;
      offset?: number;
      sortBy?: 'resonance' | 'recent' | 'alphabetical';
      filter?: string;
    } = {}
  ): Promise<{ connections: Connection[]; total: number }> {
    const { limit = 20, offset = 0, sortBy = 'resonance', filter } = options;

    let orderBy = 'c.resonance DESC';
    if (sortBy === 'recent') orderBy = 'c.last_interaction DESC';
    if (sortBy === 'alphabetical') orderBy = 'u.display_name ASC';

    const result = await this.db.query(`
      SELECT
        c.*,
        u.display_name,
        u.avatar_url,
        COUNT(*) OVER() as total
      FROM connections c
      JOIN users u ON c.connected_user_id = u.id
      WHERE c.user_id = $1
        ${filter ? 'AND u.display_name ILIKE $4' : ''}
      ORDER BY ${orderBy}
      LIMIT $2 OFFSET $3
    `, filter
      ? [userId, limit, offset, `%${filter}%`]
      : [userId, limit, offset]
    );

    return {
      connections: result.rows.map(row => ({
        userId,
        connectedUserId: row.connected_user_id,
        resonance: row.resonance,
        sharedArtists: row.shared_artists || 0,
        sharedTracks: row.shared_tracks || 0,
        sharedCircles: row.shared_circles || 0,
        mutualConnections: row.mutual_connections || 0,
        interactionScore: row.interaction_score || 0,
        tasteOverlap: row.taste_overlap || 0,
        lastInteraction: new Date(row.last_interaction || row.created_at),
        connectionType: row.connection_type,
      })),
      total: result.rows[0]?.total || 0,
    };
  }

  async getConnection(userId: string, targetUserId: string): Promise<Connection> {
    const result = await this.db.query(`
      SELECT * FROM connections
      WHERE user_id = $1 AND connected_user_id = $2
    `, [userId, targetUserId]);

    const row = result.rows[0];
    if (!row) {
      throw new Error('Connection not found');
    }

    return {
      userId,
      connectedUserId: targetUserId,
      resonance: row.resonance,
      sharedArtists: row.shared_artists || 0,
      sharedTracks: row.shared_tracks || 0,
      sharedCircles: row.shared_circles || 0,
      mutualConnections: row.mutual_connections || 0,
      interactionScore: row.interaction_score || 0,
      tasteOverlap: row.taste_overlap || 0,
      lastInteraction: new Date(row.last_interaction || row.created_at),
      connectionType: row.connection_type,
    };
  }

  // ============================================================================
  // Resonance Calculation
  // ============================================================================

  async calculateResonance(userId: string, targetUserId: string): Promise<ResonanceBreakdown> {
    const [tasteMatch, interactions, mutuals, circleTime, engagement] = await Promise.all([
      this.calculateTasteMatch(userId, targetUserId),
      this.calculateInteractionScore(userId, targetUserId),
      this.calculateMutualScore(userId, targetUserId),
      this.calculateCircleTimeScore(userId, targetUserId),
      this.calculateEngagementScore(userId, targetUserId),
    ]);

    // Weighted combination
    const weights = {
      tasteMatch: 0.35,
      interactions: 0.25,
      mutuals: 0.15,
      circleTime: 0.15,
      engagement: 0.10,
    };

    const total =
      tasteMatch * weights.tasteMatch +
      interactions * weights.interactions +
      mutuals * weights.mutuals +
      circleTime * weights.circleTime +
      engagement * weights.engagement;

    return {
      total: Math.round(total),
      components: {
        tasteMatch,
        interactions,
        mutuals,
        circleTime,
        engagement,
      },
    };
  }

  private async calculateTasteMatch(userId: string, targetUserId: string): Promise<number> {
    const result = await this.db.query(`
      SELECT 1 - (p1.taste_vector <=> p2.taste_vector) as similarity
      FROM user_taste_profiles p1
      CROSS JOIN user_taste_profiles p2
      WHERE p1.user_id = $1 AND p2.user_id = $2
    `, [userId, targetUserId]);

    return Math.round((result.rows[0]?.similarity || 0) * 100);
  }

  private async calculateInteractionScore(userId: string, targetUserId: string): Promise<number> {
    const result = await this.db.query(`
      SELECT COUNT(*) as count
      FROM user_interactions
      WHERE (user_id = $1 AND target_user_id = $2)
         OR (user_id = $2 AND target_user_id = $1)
    `, [userId, targetUserId]);

    return Math.min(100, parseInt(result.rows[0].count) * 5);
  }

  private async calculateMutualScore(userId: string, targetUserId: string): Promise<number> {
    const result = await this.db.query(`
      SELECT COUNT(*) as count
      FROM connections c1
      JOIN connections c2 ON c1.connected_user_id = c2.connected_user_id
      WHERE c1.user_id = $1 AND c2.user_id = $2
    `, [userId, targetUserId]);

    return Math.min(100, parseInt(result.rows[0].count) * 10);
  }

  private async calculateCircleTimeScore(userId: string, targetUserId: string): Promise<number> {
    const result = await this.db.query(`
      SELECT COALESCE(SUM(LEAST(cm1.listen_time, cm2.listen_time)), 0) as shared_time
      FROM circle_members cm1
      JOIN circle_members cm2 ON cm1.circle_id = cm2.circle_id
      WHERE cm1.user_id = $1 AND cm2.user_id = $2
    `, [userId, targetUserId]);

    const hours = parseFloat(result.rows[0].shared_time) / 3600;
    return Math.min(100, hours * 10);
  }

  private async calculateEngagementScore(userId: string, targetUserId: string): Promise<number> {
    const result = await this.db.query(`
      SELECT
        (SELECT COUNT(*) FROM feed_likes WHERE user_id = $1 AND creator_id = $2) +
        (SELECT COUNT(*) FROM feed_comments WHERE user_id = $1 AND creator_id = $2) +
        (SELECT COUNT(*) FROM shares WHERE user_id = $1 AND creator_id = $2)
        as total_engagement
    `, [userId, targetUserId]);

    return Math.min(100, parseInt(result.rows[0].total_engagement) * 5);
  }

  // ============================================================================
  // Background Jobs
  // ============================================================================

  async updateAllResonanceScores(): Promise<void> {
    // Run periodically to keep resonance scores fresh
    const connections = await this.db.query(`
      SELECT user_id, connected_user_id FROM connections
      WHERE updated_at < NOW() - INTERVAL '1 day'
      LIMIT 10000
    `);

    for (const conn of connections.rows) {
      const resonance = await this.calculateResonance(conn.user_id, conn.connected_user_id);
      await this.db.query(`
        UPDATE connections
        SET resonance = $3, updated_at = NOW()
        WHERE user_id = $1 AND connected_user_id = $2
      `, [conn.user_id, conn.connected_user_id, resonance.total]);
    }
  }

  // ============================================================================
  // Helpers
  // ============================================================================

  private async getUserProfile(userId: string): Promise<UserProfile> {
    const result = await this.db.query(`
      SELECT taste_vector, top_artists, top_genres
      FROM user_taste_profiles
      WHERE user_id = $1
    `, [userId]);

    return result.rows[0] || { tasteVector: null, topArtists: [], topGenres: [] };
  }

  private async getConnectionIds(userId: string): Promise<Set<string>> {
    const result = await this.db.query(
      'SELECT connected_user_id FROM connections WHERE user_id = $1',
      [userId]
    );
    return new Set(result.rows.map(r => r.connected_user_id));
  }

  private async isMutualConnection(userId: string, targetUserId: string): Promise<boolean> {
    const result = await this.db.query(`
      SELECT 1 FROM connections
      WHERE user_id = $2 AND connected_user_id = $1
    `, [userId, targetUserId]);
    return result.rows.length > 0;
  }

  private async getUserNodes(userIds: string[]): Promise<UserNode[]> {
    if (userIds.length === 0) return [];

    const result = await this.db.query(`
      SELECT id, display_name, avatar_url, resonance_score, created_at
      FROM users WHERE id = ANY($1)
    `, [userIds]);

    return result.rows.map(row => ({
      id: row.id,
      displayName: row.display_name,
      avatar: row.avatar_url,
      resonanceScore: row.resonance_score,
      topGenres: [],
      connectionCount: 0,
      joinedAt: new Date(row.created_at),
    }));
  }

  private buildSuggestion(
    row: any,
    components: ResonanceBreakdown['components'],
    reasons: string[],
    mutualConnections: UserNode[] = []
  ): SuggestedConnection {
    const total = Object.values(components).reduce((sum, v) => sum + v, 0) / 5;

    return {
      user: {
        id: row.user_id,
        displayName: row.display_name,
        avatar: row.avatar_url,
        resonanceScore: row.resonance_score || 0,
        topGenres: row.top_genres || [],
        connectionCount: 0,
        joinedAt: new Date(row.created_at),
      },
      resonance: { total: Math.round(total), components },
      reasons,
      mutualConnections,
      sharedContent: {
        artists: [],
        tracks: [],
        circles: [],
      },
    };
  }
}

interface UserProfile {
  tasteVector: number[] | null;
  topArtists: string[];
  topGenres: string[];
}

export default ResonanceGraphService;
