// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Recommendation Engine
 *
 * Multi-signal recommendation system combining:
 * - Audio similarity (features from Essentia)
 * - Collaborative filtering (listening patterns)
 * - Social signals (mycelium connections)
 * - Contextual factors (time, weather, mood)
 */

import { Pool } from 'pg';
import Redis from 'ioredis';
import { HierarchicalNSW } from 'hnswlib-node';

// ============================================================================
// Types
// ============================================================================

export interface AudioFeatures {
  bpm: number;
  key: string;
  scale: 'major' | 'minor';
  energy: number;
  valence: number;
  arousal: number;
  danceability: number;
  brightness: number;
  mfcc: number[];
}

export interface UserContext {
  timeOfDay: 'dawn' | 'morning' | 'afternoon' | 'evening' | 'night';
  dayOfWeek: number;
  weather?: string;
  recentMood?: 'energetic' | 'calm' | 'focused' | 'melancholic';
  currentActivity?: 'working' | 'relaxing' | 'exercising' | 'commuting' | 'sleeping';
}

export interface RecommendationRequest {
  userId: string;
  soulId?: string;
  context?: UserContext;
  seedTracks?: string[];
  seedArtists?: string[];
  filters?: {
    minBpm?: number;
    maxBpm?: number;
    keys?: string[];
    moods?: string[];
    excludeArtists?: string[];
    excludeTracks?: string[];
  };
  limit?: number;
  diversityFactor?: number; // 0-1, higher = more diverse
}

export interface RecommendedTrack {
  trackId: string;
  score: number;
  reasons: string[];
  signals: {
    audioSimilarity?: number;
    collaborative?: number;
    social?: number;
    contextual?: number;
  };
}

export interface RecommendationResult {
  tracks: RecommendedTrack[];
  metadata: {
    algorithm: string;
    generatedAt: Date;
    contextUsed: boolean;
    seedsUsed: string[];
  };
}

// ============================================================================
// Vector Index
// ============================================================================

class AudioVectorIndex {
  private index: HierarchicalNSW;
  private trackIdToIndex: Map<string, number> = new Map();
  private indexToTrackId: Map<number, string> = new Map();
  private dimension: number;
  private initialized = false;

  constructor(dimension: number = 20) {
    this.dimension = dimension;
    this.index = new HierarchicalNSW('cosine', dimension);
  }

  async initialize(maxElements: number = 1000000) {
    this.index.initIndex(maxElements, 16, 200, 100);
    this.initialized = true;
  }

  addTrack(trackId: string, features: number[]) {
    if (!this.initialized) throw new Error('Index not initialized');
    if (features.length !== this.dimension) {
      throw new Error(`Expected ${this.dimension} dimensions, got ${features.length}`);
    }

    const idx = this.trackIdToIndex.size;
    this.trackIdToIndex.set(trackId, idx);
    this.indexToTrackId.set(idx, trackId);
    this.index.addPoint(features, idx);
  }

  findSimilar(features: number[], k: number = 20): { trackId: string; distance: number }[] {
    if (!this.initialized) throw new Error('Index not initialized');

    const result = this.index.searchKnn(features, k);
    const similar: { trackId: string; distance: number }[] = [];

    for (let i = 0; i < result.neighbors.length; i++) {
      const trackId = this.indexToTrackId.get(result.neighbors[i]);
      if (trackId) {
        similar.push({
          trackId,
          distance: result.distances[i],
        });
      }
    }

    return similar;
  }

  findSimilarToTrack(trackId: string, k: number = 20): { trackId: string; distance: number }[] {
    const idx = this.trackIdToIndex.get(trackId);
    if (idx === undefined) return [];

    // Get the vector for this track and search
    // In practice, we'd need to store vectors or retrieve from DB
    return [];
  }
}

// ============================================================================
// Recommendation Engine
// ============================================================================

export class RecommendationEngine {
  private db: Pool;
  private redis: Redis;
  private audioIndex: AudioVectorIndex;

  // Signal weights
  private weights = {
    audioSimilarity: 0.35,
    collaborative: 0.25,
    social: 0.20,
    contextual: 0.20,
  };

  constructor(db: Pool, redis: Redis) {
    this.db = db;
    this.redis = redis;
    this.audioIndex = new AudioVectorIndex(20); // 13 MFCC + 7 other features
  }

  async initialize() {
    await this.audioIndex.initialize();
    await this.loadAudioFeatures();
  }

  /**
   * Load all audio features into the vector index
   */
  private async loadAudioFeatures() {
    const result = await this.db.query(`
      SELECT track_id, audio_features FROM tracks
      WHERE audio_features IS NOT NULL
    `);

    for (const row of result.rows) {
      const features = this.extractFeatureVector(row.audio_features);
      this.audioIndex.addTrack(row.track_id, features);
    }

    console.log(`Loaded ${result.rows.length} tracks into audio index`);
  }

  /**
   * Generate recommendations for a user
   */
  async recommend(request: RecommendationRequest): Promise<RecommendationResult> {
    const limit = request.limit || 20;
    const diversityFactor = request.diversityFactor || 0.3;

    const signals = await Promise.all([
      this.getAudioSimilaritySignal(request),
      this.getCollaborativeSignal(request),
      this.getSocialSignal(request),
      this.getContextualSignal(request),
    ]);

    // Combine signals
    const trackScores = new Map<string, {
      score: number;
      reasons: string[];
      signals: Record<string, number>;
    }>();

    const signalNames = ['audioSimilarity', 'collaborative', 'social', 'contextual'];

    for (let i = 0; i < signals.length; i++) {
      const signalName = signalNames[i];
      const weight = this.weights[signalName as keyof typeof this.weights];

      for (const [trackId, score, reason] of signals[i]) {
        const existing = trackScores.get(trackId) || {
          score: 0,
          reasons: [],
          signals: {},
        };

        existing.score += score * weight;
        existing.signals[signalName] = score;
        if (reason) existing.reasons.push(reason);

        trackScores.set(trackId, existing);
      }
    }

    // Apply filters
    let candidates = Array.from(trackScores.entries())
      .map(([trackId, data]) => ({
        trackId,
        ...data,
      }));

    if (request.filters) {
      candidates = await this.applyFilters(candidates, request.filters);
    }

    // Sort by score
    candidates.sort((a, b) => b.score - a.score);

    // Apply diversity (MMR - Maximal Marginal Relevance)
    const selected = this.applyDiversity(candidates, limit, diversityFactor);

    return {
      tracks: selected.map((t) => ({
        trackId: t.trackId,
        score: t.score,
        reasons: t.reasons,
        signals: t.signals as RecommendedTrack['signals'],
      })),
      metadata: {
        algorithm: 'hybrid-v1',
        generatedAt: new Date(),
        contextUsed: !!request.context,
        seedsUsed: [...(request.seedTracks || []), ...(request.seedArtists || [])],
      },
    };
  }

  /**
   * Audio similarity signal using vector search
   */
  private async getAudioSimilaritySignal(
    request: RecommendationRequest
  ): Promise<[string, number, string][]> {
    const results: [string, number, string][] = [];

    if (!request.seedTracks?.length) {
      // Use user's recent listening history as seeds
      const history = await this.getUserListeningHistory(request.userId, 10);
      request.seedTracks = history;
    }

    for (const seedTrack of request.seedTracks || []) {
      const similar = this.audioIndex.findSimilarToTrack(seedTrack, 50);

      for (const { trackId, distance } of similar) {
        if (trackId !== seedTrack) {
          const similarity = 1 - distance; // Convert distance to similarity
          results.push([trackId, similarity, `Similar sound to tracks you like`]);
        }
      }
    }

    return results;
  }

  /**
   * Collaborative filtering signal
   */
  private async getCollaborativeSignal(
    request: RecommendationRequest
  ): Promise<[string, number, string][]> {
    const results: [string, number, string][] = [];

    // Find users with similar listening patterns
    const similarUsers = await this.db.query(`
      WITH user_tracks AS (
        SELECT track_id FROM listening_history
        WHERE user_id = $1
        ORDER BY played_at DESC LIMIT 100
      ),
      similar_users AS (
        SELECT lh.user_id, COUNT(*) as overlap
        FROM listening_history lh
        JOIN user_tracks ut ON lh.track_id = ut.track_id
        WHERE lh.user_id != $1
        GROUP BY lh.user_id
        HAVING COUNT(*) > 5
        ORDER BY overlap DESC
        LIMIT 50
      )
      SELECT t.track_id, COUNT(*) as score
      FROM listening_history lh
      JOIN similar_users su ON lh.user_id = su.user_id
      JOIN tracks t ON lh.track_id = t.track_id
      WHERE lh.track_id NOT IN (SELECT track_id FROM user_tracks)
      GROUP BY t.track_id
      ORDER BY score DESC
      LIMIT 100
    `, [request.userId]);

    const maxScore = similarUsers.rows[0]?.score || 1;

    for (const row of similarUsers.rows) {
      const normalizedScore = row.score / maxScore;
      results.push([row.track_id, normalizedScore, 'Loved by listeners like you']);
    }

    return results;
  }

  /**
   * Social signal from mycelium connections
   */
  private async getSocialSignal(
    request: RecommendationRequest
  ): Promise<[string, number, string][]> {
    const results: [string, number, string][] = [];

    if (!request.soulId) return results;

    // Get tracks from connected souls
    const connected = await this.db.query(`
      WITH connections AS (
        SELECT connected_soul_id, connection_strength
        FROM soul_connections
        WHERE soul_id = $1
        ORDER BY connection_strength DESC
        LIMIT 20
      )
      SELECT t.track_id, SUM(c.connection_strength * lh.play_count) as weighted_score
      FROM listening_history lh
      JOIN connections c ON lh.soul_id = c.connected_soul_id
      JOIN tracks t ON lh.track_id = t.track_id
      WHERE lh.played_at > NOW() - INTERVAL '30 days'
      GROUP BY t.track_id
      ORDER BY weighted_score DESC
      LIMIT 50
    `, [request.soulId]);

    const maxScore = connected.rows[0]?.weighted_score || 1;

    for (const row of connected.rows) {
      const normalizedScore = row.weighted_score / maxScore;
      results.push([row.track_id, normalizedScore, 'Popular in your network']);
    }

    return results;
  }

  /**
   * Contextual signal based on time, mood, activity
   */
  private async getContextualSignal(
    request: RecommendationRequest
  ): Promise<[string, number, string][]> {
    const results: [string, number, string][] = [];
    const context = request.context;

    if (!context) return results;

    // Build context-aware query
    let energyRange: [number, number] = [0, 1];
    let valenceRange: [number, number] = [0, 1];

    // Time-based adjustments
    switch (context.timeOfDay) {
      case 'dawn':
      case 'morning':
        energyRange = [0.3, 0.7];
        valenceRange = [0.5, 1.0];
        break;
      case 'afternoon':
        energyRange = [0.5, 0.9];
        valenceRange = [0.4, 0.9];
        break;
      case 'evening':
        energyRange = [0.2, 0.6];
        valenceRange = [0.3, 0.8];
        break;
      case 'night':
        energyRange = [0.1, 0.4];
        valenceRange = [0.2, 0.7];
        break;
    }

    // Activity-based adjustments
    if (context.currentActivity === 'exercising') {
      energyRange = [0.7, 1.0];
    } else if (context.currentActivity === 'sleeping') {
      energyRange = [0.0, 0.2];
    } else if (context.currentActivity === 'working') {
      energyRange = [0.3, 0.6]; // Focus music
    }

    const contextual = await this.db.query(`
      SELECT track_id,
        ABS(energy - $1) + ABS(valence - $2) as context_distance
      FROM tracks
      WHERE energy BETWEEN $3 AND $4
        AND valence BETWEEN $5 AND $6
      ORDER BY context_distance ASC
      LIMIT 50
    `, [
      (energyRange[0] + energyRange[1]) / 2,
      (valenceRange[0] + valenceRange[1]) / 2,
      energyRange[0],
      energyRange[1],
      valenceRange[0],
      valenceRange[1],
    ]);

    const reason = `Perfect for ${context.timeOfDay}${context.currentActivity ? ` ${context.currentActivity}` : ''}`;

    for (const row of contextual.rows) {
      const score = 1 - row.context_distance / 2;
      results.push([row.track_id, Math.max(0, score), reason]);
    }

    return results;
  }

  // ============================================================================
  // Helpers
  // ============================================================================

  private extractFeatureVector(features: AudioFeatures): number[] {
    // Combine MFCC with other normalized features
    const normalized = [
      features.bpm / 200, // Normalize BPM (assume max 200)
      features.energy,
      features.valence,
      features.arousal,
      features.danceability,
      features.brightness,
      features.scale === 'major' ? 1 : 0,
    ];

    return [...(features.mfcc?.slice(0, 13) || new Array(13).fill(0)), ...normalized];
  }

  private async getUserListeningHistory(userId: string, limit: number): Promise<string[]> {
    const result = await this.db.query(`
      SELECT track_id FROM listening_history
      WHERE user_id = $1
      ORDER BY played_at DESC
      LIMIT $2
    `, [userId, limit]);

    return result.rows.map((r) => r.track_id);
  }

  private async applyFilters(
    candidates: Array<{ trackId: string; score: number; reasons: string[]; signals: Record<string, number> }>,
    filters: RecommendationRequest['filters']
  ) {
    if (!filters) return candidates;

    const trackIds = candidates.map((c) => c.trackId);

    const result = await this.db.query(`
      SELECT track_id, bpm, key, artist_id FROM tracks
      WHERE track_id = ANY($1)
    `, [trackIds]);

    const trackData = new Map(result.rows.map((r) => [r.track_id, r]));

    return candidates.filter((c) => {
      const track = trackData.get(c.trackId);
      if (!track) return false;

      if (filters.minBpm && track.bpm < filters.minBpm) return false;
      if (filters.maxBpm && track.bpm > filters.maxBpm) return false;
      if (filters.keys?.length && !filters.keys.includes(track.key)) return false;
      if (filters.excludeArtists?.includes(track.artist_id)) return false;
      if (filters.excludeTracks?.includes(c.trackId)) return false;

      return true;
    });
  }

  private applyDiversity(
    candidates: Array<{ trackId: string; score: number; reasons: string[]; signals: Record<string, number> }>,
    limit: number,
    diversityFactor: number
  ): typeof candidates {
    // Maximal Marginal Relevance for diversity
    const selected: typeof candidates = [];
    const remaining = [...candidates];

    while (selected.length < limit && remaining.length > 0) {
      if (selected.length === 0) {
        // First item is highest scored
        selected.push(remaining.shift()!);
        continue;
      }

      // Calculate MMR for each remaining candidate
      let bestIdx = 0;
      let bestMMR = -Infinity;

      for (let i = 0; i < remaining.length; i++) {
        const candidate = remaining[i];

        // Relevance score
        const relevance = candidate.score;

        // Max similarity to already selected (simplified - use score difference)
        let maxSimilarity = 0;
        for (const sel of selected) {
          // In practice, compare audio features
          const similarity = 1 - Math.abs(candidate.score - sel.score);
          maxSimilarity = Math.max(maxSimilarity, similarity);
        }

        // MMR formula
        const mmr = (1 - diversityFactor) * relevance - diversityFactor * maxSimilarity;

        if (mmr > bestMMR) {
          bestMMR = mmr;
          bestIdx = i;
        }
      }

      selected.push(remaining.splice(bestIdx, 1)[0]);
    }

    return selected;
  }
}
