// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Auto-DJ Service
 *
 * AI-powered DJ that creates continuous mixes with smooth transitions.
 * Uses audio analysis to match tempo, key, and energy levels.
 */

import { Redis } from 'ioredis';
import { Pool } from 'pg';

// ============================================================================
// Types
// ============================================================================

export interface Track {
  id: string;
  title: string;
  artist: string;
  duration: number;           // seconds
  audioUrl: string;
  analysis: AudioAnalysis;
}

export interface AudioAnalysis {
  bpm: number;
  key: string;                // e.g., "Am", "C", "F#m"
  keyConfidence: number;      // 0-1
  energy: number;             // 0-1
  danceability: number;       // 0-1
  valence: number;            // 0-1 (happiness)
  acousticness: number;       // 0-1
  instrumentalness: number;   // 0-1
  sections: Section[];
  beats: Beat[];
  segments: Segment[];
}

export interface Section {
  start: number;
  duration: number;
  confidence: number;
  loudness: number;
  tempo: number;
  key: number;
  mode: number;
}

export interface Beat {
  start: number;
  duration: number;
  confidence: number;
}

export interface Segment {
  start: number;
  duration: number;
  loudnessStart: number;
  loudnessMax: number;
  loudnessMaxTime: number;
  pitches: number[];
  timbre: number[];
}

export interface DJSession {
  id: string;
  userId: string;
  mood: DJMood;
  seedTracks: string[];
  playedTracks: string[];
  currentTrack: Track | null;
  nextTrack: Track | null;
  transitionTime: number;      // When to start transition
  transitionDuration: number;  // Length of crossfade
  isActive: boolean;
}

export type DJMood =
  | 'energetic'
  | 'chill'
  | 'focus'
  | 'party'
  | 'melancholic'
  | 'uplifting'
  | 'late_night'
  | 'workout';

export interface TransitionPlan {
  outTrack: Track;
  inTrack: Track;
  outPoint: number;            // Where to start fading out
  inPoint: number;             // Where to start the incoming track
  duration: number;            // Crossfade duration
  tempoMatch: {
    adjust: 'speed_up' | 'slow_down' | 'none';
    factor: number;            // 0.95 to 1.05
  };
  filterSweep: boolean;
  eqCrossover: boolean;
  beatSync: boolean;
}

// ============================================================================
// Auto-DJ Service
// ============================================================================

export class AutoDJService {
  private redis: Redis;
  private db: Pool;
  private sessions: Map<string, DJSession> = new Map();

  // Key compatibility chart (Camelot wheel)
  private readonly KEY_COMPATIBILITY: Record<string, string[]> = {
    'Am': ['Am', 'Em', 'Dm', 'C', 'G', 'F'],
    'Em': ['Em', 'Am', 'Bm', 'G', 'D', 'C'],
    'Bm': ['Bm', 'Em', 'F#m', 'D', 'A', 'G'],
    'F#m': ['F#m', 'Bm', 'C#m', 'A', 'E', 'D'],
    'C#m': ['C#m', 'F#m', 'G#m', 'E', 'B', 'A'],
    'G#m': ['G#m', 'C#m', 'D#m', 'B', 'F#', 'E'],
    'D#m': ['D#m', 'G#m', 'A#m', 'F#', 'C#', 'B'],
    'A#m': ['A#m', 'D#m', 'Fm', 'C#', 'G#', 'F#'],
    'Fm': ['Fm', 'A#m', 'Cm', 'G#', 'D#', 'C#'],
    'Cm': ['Cm', 'Fm', 'Gm', 'D#', 'A#', 'G#'],
    'Gm': ['Gm', 'Cm', 'Dm', 'A#', 'F', 'D#'],
    'Dm': ['Dm', 'Gm', 'Am', 'F', 'C', 'A#'],
    'C': ['C', 'F', 'G', 'Am', 'Em', 'Dm'],
    'G': ['G', 'C', 'D', 'Em', 'Bm', 'Am'],
    'D': ['D', 'G', 'A', 'Bm', 'F#m', 'Em'],
    'A': ['A', 'D', 'E', 'F#m', 'C#m', 'Bm'],
    'E': ['E', 'A', 'B', 'C#m', 'G#m', 'F#m'],
    'B': ['B', 'E', 'F#', 'G#m', 'D#m', 'C#m'],
    'F#': ['F#', 'B', 'C#', 'D#m', 'A#m', 'G#m'],
    'C#': ['C#', 'F#', 'G#', 'A#m', 'Fm', 'D#m'],
    'G#': ['G#', 'C#', 'D#', 'Fm', 'Cm', 'A#m'],
    'D#': ['D#', 'G#', 'A#', 'Cm', 'Gm', 'Fm'],
    'A#': ['A#', 'D#', 'F', 'Gm', 'Dm', 'Cm'],
    'F': ['F', 'A#', 'C', 'Dm', 'Am', 'Gm'],
  };

  // Mood to audio feature mapping
  private readonly MOOD_PROFILES: Record<DJMood, {
    energy: [number, number];
    valence: [number, number];
    danceability: [number, number];
    bpmRange: [number, number];
  }> = {
    energetic: { energy: [0.7, 1], valence: [0.5, 1], danceability: [0.6, 1], bpmRange: [120, 150] },
    chill: { energy: [0.1, 0.5], valence: [0.3, 0.7], danceability: [0.2, 0.6], bpmRange: [70, 110] },
    focus: { energy: [0.2, 0.6], valence: [0.3, 0.6], danceability: [0.2, 0.5], bpmRange: [80, 120] },
    party: { energy: [0.7, 1], valence: [0.6, 1], danceability: [0.7, 1], bpmRange: [115, 135] },
    melancholic: { energy: [0.1, 0.4], valence: [0, 0.4], danceability: [0.1, 0.5], bpmRange: [60, 100] },
    uplifting: { energy: [0.5, 0.9], valence: [0.6, 1], danceability: [0.5, 0.9], bpmRange: [100, 140] },
    late_night: { energy: [0.3, 0.7], valence: [0.2, 0.6], danceability: [0.4, 0.8], bpmRange: [115, 130] },
    workout: { energy: [0.7, 1], valence: [0.5, 0.9], danceability: [0.6, 1], bpmRange: [130, 180] },
  };

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
  // Session Management
  // ============================================================================

  async createSession(
    userId: string,
    mood: DJMood,
    seedTrackIds: string[]
  ): Promise<DJSession> {
    const sessionId = `dj_${Date.now()}_${Math.random().toString(36).slice(2)}`;

    // Load seed tracks
    const seedTracks = await this.loadTracks(seedTrackIds);

    // Select first track
    const firstTrack = seedTracks[0] || await this.selectTrackForMood(mood, []);

    // Select next track
    const nextTrack = await this.selectNextTrack(firstTrack, mood, [firstTrack.id]);

    const session: DJSession = {
      id: sessionId,
      userId,
      mood,
      seedTracks: seedTrackIds,
      playedTracks: [firstTrack.id],
      currentTrack: firstTrack,
      nextTrack,
      transitionTime: this.calculateTransitionTime(firstTrack),
      transitionDuration: 8, // Default 8 second crossfade
      isActive: true,
    };

    this.sessions.set(sessionId, session);

    // Cache in Redis
    await this.redis.setex(
      `dj:session:${sessionId}`,
      3600 * 4, // 4 hour TTL
      JSON.stringify(session)
    );

    return session;
  }

  async getSession(sessionId: string): Promise<DJSession | null> {
    // Check memory cache
    if (this.sessions.has(sessionId)) {
      return this.sessions.get(sessionId)!;
    }

    // Check Redis
    const cached = await this.redis.get(`dj:session:${sessionId}`);
    if (cached) {
      const session = JSON.parse(cached) as DJSession;
      this.sessions.set(sessionId, session);
      return session;
    }

    return null;
  }

  async endSession(sessionId: string): Promise<void> {
    this.sessions.delete(sessionId);
    await this.redis.del(`dj:session:${sessionId}`);
  }

  // ============================================================================
  // Track Selection
  // ============================================================================

  async selectNextTrack(
    currentTrack: Track,
    mood: DJMood,
    playedIds: string[]
  ): Promise<Track> {
    const profile = this.MOOD_PROFILES[mood];

    // Find compatible tracks
    const candidates = await this.db.query(`
      WITH compatible_keys AS (
        SELECT unnest($1::text[]) as key
      )
      SELECT
        t.id,
        t.title,
        t.artist_name as artist,
        t.duration,
        t.audio_url,
        ta.bpm,
        ta.key,
        ta.key_confidence,
        ta.energy,
        ta.danceability,
        ta.valence,
        ta.acousticness,
        ta.instrumentalness,
        ta.sections,
        ta.beats,
        ta.segments,
        ABS(ta.bpm - $2) as bpm_diff,
        ta.energy as track_energy
      FROM tracks t
      JOIN track_analysis ta ON t.id = ta.track_id
      WHERE t.id != $3
        AND t.id != ALL($4)
        AND ta.key = ANY(SELECT key FROM compatible_keys)
        AND ta.energy BETWEEN $5 AND $6
        AND ta.valence BETWEEN $7 AND $8
        AND ta.danceability BETWEEN $9 AND $10
        AND ta.bpm BETWEEN $11 AND $12
      ORDER BY
        -- Prefer similar BPM
        ABS(ta.bpm - $2),
        -- Prefer gradual energy changes
        ABS(ta.energy - $13),
        RANDOM()
      LIMIT 20
    `, [
      this.KEY_COMPATIBILITY[currentTrack.analysis.key] || [currentTrack.analysis.key],
      currentTrack.analysis.bpm,
      currentTrack.id,
      playedIds,
      profile.energy[0],
      profile.energy[1],
      profile.valence[0],
      profile.valence[1],
      profile.danceability[0],
      profile.danceability[1],
      profile.bpmRange[0],
      profile.bpmRange[1],
      currentTrack.analysis.energy,
    ]);

    if (candidates.rows.length === 0) {
      // Fallback: select any track matching mood
      return this.selectTrackForMood(mood, playedIds);
    }

    // Score candidates
    const scored = candidates.rows.map(row => ({
      track: this.rowToTrack(row),
      score: this.scoreTransition(currentTrack, this.rowToTrack(row)),
    }));

    // Sort by score and add some randomness
    scored.sort((a, b) => b.score - a.score);

    // Pick from top 5 with weighted randomness
    const topCandidates = scored.slice(0, 5);
    const weights = topCandidates.map((_, i) => Math.pow(0.6, i));
    const totalWeight = weights.reduce((a, b) => a + b, 0);
    let random = Math.random() * totalWeight;

    for (let i = 0; i < topCandidates.length; i++) {
      random -= weights[i];
      if (random <= 0) {
        return topCandidates[i].track;
      }
    }

    return topCandidates[0].track;
  }

  async selectTrackForMood(mood: DJMood, excludeIds: string[]): Promise<Track> {
    const profile = this.MOOD_PROFILES[mood];

    const result = await this.db.query(`
      SELECT
        t.id,
        t.title,
        t.artist_name as artist,
        t.duration,
        t.audio_url,
        ta.bpm,
        ta.key,
        ta.key_confidence,
        ta.energy,
        ta.danceability,
        ta.valence,
        ta.acousticness,
        ta.instrumentalness,
        ta.sections,
        ta.beats,
        ta.segments
      FROM tracks t
      JOIN track_analysis ta ON t.id = ta.track_id
      WHERE t.id != ALL($1)
        AND ta.energy BETWEEN $2 AND $3
        AND ta.valence BETWEEN $4 AND $5
        AND ta.danceability BETWEEN $6 AND $7
        AND ta.bpm BETWEEN $8 AND $9
      ORDER BY RANDOM()
      LIMIT 1
    `, [
      excludeIds,
      profile.energy[0],
      profile.energy[1],
      profile.valence[0],
      profile.valence[1],
      profile.danceability[0],
      profile.danceability[1],
      profile.bpmRange[0],
      profile.bpmRange[1],
    ]);

    return this.rowToTrack(result.rows[0]);
  }

  // ============================================================================
  // Transition Planning
  // ============================================================================

  planTransition(outTrack: Track, inTrack: Track): TransitionPlan {
    const bpmRatio = inTrack.analysis.bpm / outTrack.analysis.bpm;

    // Determine tempo adjustment
    let tempoAdjust: 'speed_up' | 'slow_down' | 'none' = 'none';
    let tempoFactor = 1.0;

    if (bpmRatio > 1.02 && bpmRatio < 1.06) {
      tempoAdjust = 'slow_down';
      tempoFactor = 1 / bpmRatio;
    } else if (bpmRatio < 0.98 && bpmRatio > 0.94) {
      tempoAdjust = 'speed_up';
      tempoFactor = 1 / bpmRatio;
    }

    // Find best out point (end of a phrase, usually 8 or 16 bars before end)
    const outPoint = this.findOutPoint(outTrack);

    // Find best in point (start of a phrase in incoming track)
    const inPoint = this.findInPoint(inTrack);

    // Calculate crossfade duration based on tempo
    const beatsPerSecond = outTrack.analysis.bpm / 60;
    const barsForTransition = 8; // 8 bars crossfade
    const transitionDuration = (barsForTransition * 4) / beatsPerSecond;

    // Determine if we should use effects
    const energyDiff = Math.abs(outTrack.analysis.energy - inTrack.analysis.energy);
    const useFilterSweep = energyDiff > 0.3;
    const useEqCrossover = !useFilterSweep && energyDiff > 0.1;

    return {
      outTrack,
      inTrack,
      outPoint,
      inPoint,
      duration: transitionDuration,
      tempoMatch: {
        adjust: tempoAdjust,
        factor: tempoFactor,
      },
      filterSweep: useFilterSweep,
      eqCrossover: useEqCrossover,
      beatSync: Math.abs(bpmRatio - 1) < 0.08, // Beat sync if BPMs within 8%
    };
  }

  private findOutPoint(track: Track): number {
    const sections = track.analysis.sections;
    if (sections.length === 0) {
      return track.duration - 16; // 16 seconds before end
    }

    // Look for a section break near the end
    const targetTime = track.duration - 32; // 32 seconds before end

    for (let i = sections.length - 1; i >= 0; i--) {
      const section = sections[i];
      if (section.start <= targetTime && section.start > track.duration - 60) {
        return section.start;
      }
    }

    // Fallback: find nearest beat to target time
    return this.findNearestBeat(track, targetTime);
  }

  private findInPoint(track: Track): number {
    const sections = track.analysis.sections;
    if (sections.length === 0) {
      return 0;
    }

    // Often best to start at the beginning of first section
    // Some tracks have intros we want to use
    if (sections[0].start < 2 && sections[0].duration < 16) {
      // Short intro, start from there
      return sections[0].start;
    }

    // Skip very long intros
    if (sections[0].duration > 30) {
      return sections[1]?.start || 0;
    }

    return 0;
  }

  private findNearestBeat(track: Track, targetTime: number): number {
    const beats = track.analysis.beats;
    if (beats.length === 0) return targetTime;

    let nearest = beats[0].start;
    let minDiff = Math.abs(beats[0].start - targetTime);

    for (const beat of beats) {
      const diff = Math.abs(beat.start - targetTime);
      if (diff < minDiff) {
        minDiff = diff;
        nearest = beat.start;
      }
    }

    return nearest;
  }

  // ============================================================================
  // Scoring
  // ============================================================================

  private scoreTransition(current: Track, next: Track): number {
    let score = 100;

    // BPM compatibility (±6% is good)
    const bpmRatio = next.analysis.bpm / current.analysis.bpm;
    if (bpmRatio > 0.94 && bpmRatio < 1.06) {
      score += 20 * (1 - Math.abs(1 - bpmRatio) / 0.06);
    } else {
      score -= 30;
    }

    // Key compatibility
    const compatibleKeys = this.KEY_COMPATIBILITY[current.analysis.key] || [];
    if (compatibleKeys.includes(next.analysis.key)) {
      score += 25;
      if (next.analysis.key === current.analysis.key) {
        score += 10; // Bonus for same key
      }
    } else {
      score -= 20;
    }

    // Energy flow (gradual changes preferred)
    const energyDiff = Math.abs(next.analysis.energy - current.analysis.energy);
    if (energyDiff < 0.2) {
      score += 15;
    } else if (energyDiff > 0.4) {
      score -= 15;
    }

    // Danceability similarity
    const danceDiff = Math.abs(next.analysis.danceability - current.analysis.danceability);
    if (danceDiff < 0.2) {
      score += 10;
    }

    // Valence flow
    const valenceDiff = Math.abs(next.analysis.valence - current.analysis.valence);
    if (valenceDiff < 0.3) {
      score += 10;
    }

    return score;
  }

  // ============================================================================
  // Session Advancement
  // ============================================================================

  async advanceSession(sessionId: string): Promise<{
    transition: TransitionPlan;
    nextTrack: Track;
  }> {
    const session = await this.getSession(sessionId);
    if (!session || !session.currentTrack || !session.nextTrack) {
      throw new Error('Invalid session');
    }

    // Plan transition
    const transition = this.planTransition(session.currentTrack, session.nextTrack);

    // Select track after next
    const upcomingTrack = await this.selectNextTrack(
      session.nextTrack,
      session.mood,
      [...session.playedTracks, session.nextTrack.id]
    );

    // Update session
    session.playedTracks.push(session.nextTrack.id);
    session.currentTrack = session.nextTrack;
    session.nextTrack = upcomingTrack;
    session.transitionTime = this.calculateTransitionTime(session.currentTrack);

    // Save session
    this.sessions.set(sessionId, session);
    await this.redis.setex(
      `dj:session:${sessionId}`,
      3600 * 4,
      JSON.stringify(session)
    );

    return { transition, nextTrack: upcomingTrack };
  }

  async changeMood(sessionId: string, newMood: DJMood): Promise<Track> {
    const session = await this.getSession(sessionId);
    if (!session) throw new Error('Session not found');

    session.mood = newMood;

    // Select new next track matching new mood
    const nextTrack = await this.selectNextTrack(
      session.currentTrack!,
      newMood,
      session.playedTracks
    );

    session.nextTrack = nextTrack;

    this.sessions.set(sessionId, session);
    await this.redis.setex(
      `dj:session:${sessionId}`,
      3600 * 4,
      JSON.stringify(session)
    );

    return nextTrack;
  }

  // ============================================================================
  // Helpers
  // ============================================================================

  private async loadTracks(trackIds: string[]): Promise<Track[]> {
    if (trackIds.length === 0) return [];

    const result = await this.db.query(`
      SELECT
        t.id,
        t.title,
        t.artist_name as artist,
        t.duration,
        t.audio_url,
        ta.bpm,
        ta.key,
        ta.key_confidence,
        ta.energy,
        ta.danceability,
        ta.valence,
        ta.acousticness,
        ta.instrumentalness,
        ta.sections,
        ta.beats,
        ta.segments
      FROM tracks t
      JOIN track_analysis ta ON t.id = ta.track_id
      WHERE t.id = ANY($1)
    `, [trackIds]);

    return result.rows.map(row => this.rowToTrack(row));
  }

  private rowToTrack(row: any): Track {
    return {
      id: row.id,
      title: row.title,
      artist: row.artist,
      duration: row.duration,
      audioUrl: row.audio_url,
      analysis: {
        bpm: row.bpm,
        key: row.key,
        keyConfidence: row.key_confidence,
        energy: row.energy,
        danceability: row.danceability,
        valence: row.valence,
        acousticness: row.acousticness,
        instrumentalness: row.instrumentalness,
        sections: row.sections || [],
        beats: row.beats || [],
        segments: row.segments || [],
      },
    };
  }

  private calculateTransitionTime(track: Track): number {
    // Start transition 16-32 seconds before track end
    const transitionBuffer = Math.min(32, Math.max(16, track.duration * 0.1));
    return track.duration - transitionBuffer;
  }
}

export default AutoDJService;
