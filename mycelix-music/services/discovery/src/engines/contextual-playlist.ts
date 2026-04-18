// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Contextual Playlist Generator
 *
 * Creates dynamic playlists based on:
 * - Time of day and day of week
 * - Weather conditions
 * - User mood and activity
 * - Listening history patterns
 * - Special occasions and seasons
 */

import { Pool } from 'pg';
import Redis from 'ioredis';

// ============================================================================
// Types
// ============================================================================

export interface PlaylistContext {
  // Temporal
  timeOfDay: 'dawn' | 'morning' | 'afternoon' | 'evening' | 'night';
  dayOfWeek: number; // 0-6
  isWeekend: boolean;
  season: 'spring' | 'summer' | 'autumn' | 'winter';

  // Environmental
  weather?: {
    condition: 'clear' | 'cloudy' | 'rainy' | 'stormy' | 'snowy' | 'foggy';
    temperature: number; // Celsius
    humidity: number;
  };

  // User state
  mood?: 'energetic' | 'calm' | 'focused' | 'melancholic' | 'romantic' | 'adventurous';
  activity?: 'working' | 'relaxing' | 'exercising' | 'commuting' | 'cooking' | 'studying' | 'sleeping';

  // Special
  occasion?: 'birthday' | 'holiday' | 'celebration' | 'meditation' | 'party';

  // Location
  location?: {
    latitude: number;
    longitude: number;
    timezone: string;
  };
}

export interface GeneratedPlaylist {
  id: string;
  name: string;
  description: string;
  tracks: string[];
  context: PlaylistContext;
  duration: number; // Total duration in seconds
  mood: {
    averageEnergy: number;
    averageValence: number;
    dominantGenres: string[];
  };
  generatedAt: Date;
  expiresAt: Date;
}

interface TrackData {
  track_id: string;
  title: string;
  artist_name: string;
  duration: number;
  bpm: number;
  energy: number;
  valence: number;
  key: string;
  genres: string[];
}

// ============================================================================
// Playlist Templates
// ============================================================================

interface PlaylistTemplate {
  name: (ctx: PlaylistContext) => string;
  description: (ctx: PlaylistContext) => string;
  energyRange: [number, number];
  valenceRange: [number, number];
  bpmRange?: [number, number];
  targetDuration: number; // minutes
  flowPattern: 'steady' | 'building' | 'winding-down' | 'wave';
}

const TEMPLATES: Record<string, PlaylistTemplate> = {
  'morning-energy': {
    name: () => 'Morning Energy',
    description: () => 'Start your day with uplifting vibes',
    energyRange: [0.5, 0.8],
    valenceRange: [0.6, 1.0],
    bpmRange: [100, 140],
    targetDuration: 45,
    flowPattern: 'building',
  },
  'deep-focus': {
    name: () => 'Deep Focus',
    description: () => 'Ambient soundscapes for concentration',
    energyRange: [0.1, 0.4],
    valenceRange: [0.3, 0.7],
    bpmRange: [60, 100],
    targetDuration: 120,
    flowPattern: 'steady',
  },
  'evening-unwind': {
    name: () => 'Evening Unwind',
    description: () => 'Gentle sounds to decompress',
    energyRange: [0.1, 0.4],
    valenceRange: [0.4, 0.8],
    bpmRange: [60, 100],
    targetDuration: 60,
    flowPattern: 'winding-down',
  },
  'workout-intensity': {
    name: () => 'Workout Intensity',
    description: () => 'High energy tracks to fuel your exercise',
    energyRange: [0.7, 1.0],
    valenceRange: [0.5, 1.0],
    bpmRange: [120, 180],
    targetDuration: 60,
    flowPattern: 'wave',
  },
  'rainy-day': {
    name: () => 'Rainy Day Vibes',
    description: () => 'Cozy sounds for grey skies',
    energyRange: [0.1, 0.4],
    valenceRange: [0.2, 0.6],
    targetDuration: 90,
    flowPattern: 'steady',
  },
  'summer-vibes': {
    name: () => 'Summer Vibes',
    description: () => 'Warm, sunny sounds',
    energyRange: [0.5, 0.8],
    valenceRange: [0.6, 1.0],
    bpmRange: [90, 130],
    targetDuration: 60,
    flowPattern: 'wave',
  },
  'late-night': {
    name: () => 'Late Night',
    description: () => 'Intimate sounds for the midnight hours',
    energyRange: [0.1, 0.3],
    valenceRange: [0.2, 0.6],
    bpmRange: [60, 90],
    targetDuration: 60,
    flowPattern: 'winding-down',
  },
  'celebration': {
    name: () => 'Celebration',
    description: () => 'Party-ready tracks',
    energyRange: [0.7, 1.0],
    valenceRange: [0.7, 1.0],
    bpmRange: [110, 140],
    targetDuration: 120,
    flowPattern: 'wave',
  },
};

// ============================================================================
// Playlist Generator
// ============================================================================

export class ContextualPlaylistGenerator {
  private db: Pool;
  private redis: Redis;
  private cachePrefix = 'playlist:contextual:';
  private cacheTTL = 3600; // 1 hour

  constructor(db: Pool, redis: Redis) {
    this.db = db;
    this.redis = redis;
  }

  /**
   * Generate a playlist based on context
   */
  async generate(
    userId: string,
    context: PlaylistContext
  ): Promise<GeneratedPlaylist> {
    // Check cache first
    const cacheKey = this.getCacheKey(userId, context);
    const cached = await this.redis.get(cacheKey);
    if (cached) {
      return JSON.parse(cached);
    }

    // Select appropriate template
    const template = this.selectTemplate(context);

    // Get candidate tracks
    const candidates = await this.getCandidateTracks(userId, template, context);

    // Order tracks according to flow pattern
    const orderedTracks = this.orderByFlow(candidates, template.flowPattern);

    // Trim to target duration
    const selectedTracks = this.trimToTarget(orderedTracks, template.targetDuration);

    // Calculate mood summary
    const mood = this.calculateMood(selectedTracks);

    const playlist: GeneratedPlaylist = {
      id: `ctx_${Date.now()}_${Math.random().toString(36).slice(2, 8)}`,
      name: template.name(context),
      description: template.description(context),
      tracks: selectedTracks.map((t) => t.track_id),
      context,
      duration: selectedTracks.reduce((sum, t) => sum + t.duration, 0),
      mood,
      generatedAt: new Date(),
      expiresAt: new Date(Date.now() + this.cacheTTL * 1000),
    };

    // Cache the result
    await this.redis.setex(cacheKey, this.cacheTTL, JSON.stringify(playlist));

    return playlist;
  }

  /**
   * Generate a "Your Day" playlist with multiple sections
   */
  async generateDayPlaylist(userId: string, location?: PlaylistContext['location']): Promise<{
    morning: GeneratedPlaylist;
    afternoon: GeneratedPlaylist;
    evening: GeneratedPlaylist;
    night: GeneratedPlaylist;
  }> {
    const baseContext: Partial<PlaylistContext> = {
      dayOfWeek: new Date().getDay(),
      isWeekend: [0, 6].includes(new Date().getDay()),
      season: this.getSeason(location?.latitude),
      location,
    };

    const [morning, afternoon, evening, night] = await Promise.all([
      this.generate(userId, { ...baseContext, timeOfDay: 'morning' } as PlaylistContext),
      this.generate(userId, { ...baseContext, timeOfDay: 'afternoon' } as PlaylistContext),
      this.generate(userId, { ...baseContext, timeOfDay: 'evening' } as PlaylistContext),
      this.generate(userId, { ...baseContext, timeOfDay: 'night' } as PlaylistContext),
    ]);

    return { morning, afternoon, evening, night };
  }

  /**
   * Generate activity-specific playlist
   */
  async generateForActivity(
    userId: string,
    activity: NonNullable<PlaylistContext['activity']>,
    duration?: number
  ): Promise<GeneratedPlaylist> {
    const context: PlaylistContext = {
      timeOfDay: this.getCurrentTimeOfDay(),
      dayOfWeek: new Date().getDay(),
      isWeekend: [0, 6].includes(new Date().getDay()),
      season: this.getSeason(),
      activity,
    };

    const playlist = await this.generate(userId, context);

    // Adjust duration if specified
    if (duration && playlist.duration !== duration * 60) {
      // Re-generate with specific duration
    }

    return playlist;
  }

  // ============================================================================
  // Private Methods
  // ============================================================================

  private selectTemplate(context: PlaylistContext): PlaylistTemplate {
    // Priority-based template selection
    if (context.occasion === 'celebration' || context.occasion === 'party') {
      return TEMPLATES['celebration'];
    }

    if (context.activity === 'exercising') {
      return TEMPLATES['workout-intensity'];
    }

    if (context.activity === 'working' || context.activity === 'studying') {
      return TEMPLATES['deep-focus'];
    }

    if (context.weather?.condition === 'rainy' || context.weather?.condition === 'stormy') {
      return TEMPLATES['rainy-day'];
    }

    if (context.season === 'summer' && context.weather?.temperature && context.weather.temperature > 25) {
      return TEMPLATES['summer-vibes'];
    }

    switch (context.timeOfDay) {
      case 'dawn':
      case 'morning':
        return TEMPLATES['morning-energy'];
      case 'afternoon':
        return TEMPLATES['deep-focus'];
      case 'evening':
        return TEMPLATES['evening-unwind'];
      case 'night':
        return TEMPLATES['late-night'];
      default:
        return TEMPLATES['deep-focus'];
    }
  }

  private async getCandidateTracks(
    userId: string,
    template: PlaylistTemplate,
    context: PlaylistContext
  ): Promise<TrackData[]> {
    const query = `
      WITH user_preferences AS (
        SELECT genre, COUNT(*) as listen_count
        FROM listening_history lh
        JOIN tracks t ON lh.track_id = t.track_id
        JOIN track_genres tg ON t.track_id = tg.track_id
        WHERE lh.user_id = $1
        GROUP BY genre
        ORDER BY listen_count DESC
        LIMIT 10
      )
      SELECT
        t.track_id,
        t.title,
        a.name as artist_name,
        t.duration,
        t.bpm,
        t.energy,
        t.valence,
        t.key,
        array_agg(tg.genre) as genres
      FROM tracks t
      JOIN artists a ON t.artist_id = a.artist_id
      LEFT JOIN track_genres tg ON t.track_id = tg.track_id
      WHERE t.energy BETWEEN $2 AND $3
        AND t.valence BETWEEN $4 AND $5
        ${template.bpmRange ? 'AND t.bpm BETWEEN $6 AND $7' : ''}
        AND EXISTS (
          SELECT 1 FROM user_preferences up
          WHERE tg.genre = up.genre
        )
      GROUP BY t.track_id, t.title, a.name, t.duration, t.bpm, t.energy, t.valence, t.key
      ORDER BY RANDOM()
      LIMIT 200
    `;

    const params = [
      userId,
      template.energyRange[0],
      template.energyRange[1],
      template.valenceRange[0],
      template.valenceRange[1],
    ];

    if (template.bpmRange) {
      params.push(template.bpmRange[0], template.bpmRange[1]);
    }

    const result = await this.db.query(query, params);
    return result.rows;
  }

  private orderByFlow(tracks: TrackData[], pattern: PlaylistTemplate['flowPattern']): TrackData[] {
    switch (pattern) {
      case 'building':
        // Start low energy, build up
        return [...tracks].sort((a, b) => a.energy - b.energy);

      case 'winding-down':
        // Start higher, wind down
        return [...tracks].sort((a, b) => b.energy - a.energy);

      case 'wave':
        // Alternating highs and lows
        const sorted = [...tracks].sort((a, b) => a.energy - b.energy);
        const result: TrackData[] = [];
        let high = true;

        while (sorted.length > 0) {
          if (high) {
            result.push(sorted.pop()!);
          } else {
            result.push(sorted.shift()!);
          }
          high = !high;
        }
        return result;

      case 'steady':
      default:
        // Keep similar energy levels, vary other aspects
        return [...tracks].sort((a, b) => {
          // Group by similar energy, then randomize within groups
          const energyDiff = Math.floor(a.energy * 5) - Math.floor(b.energy * 5);
          if (energyDiff !== 0) return energyDiff;
          return Math.random() - 0.5;
        });
    }
  }

  private trimToTarget(tracks: TrackData[], targetMinutes: number): TrackData[] {
    const targetSeconds = targetMinutes * 60;
    const result: TrackData[] = [];
    let totalDuration = 0;

    for (const track of tracks) {
      if (totalDuration + track.duration <= targetSeconds * 1.1) {
        result.push(track);
        totalDuration += track.duration;
      }

      if (totalDuration >= targetSeconds) break;
    }

    return result;
  }

  private calculateMood(tracks: TrackData[]): GeneratedPlaylist['mood'] {
    if (tracks.length === 0) {
      return { averageEnergy: 0, averageValence: 0, dominantGenres: [] };
    }

    const avgEnergy = tracks.reduce((sum, t) => sum + t.energy, 0) / tracks.length;
    const avgValence = tracks.reduce((sum, t) => sum + t.valence, 0) / tracks.length;

    const genreCounts = new Map<string, number>();
    for (const track of tracks) {
      for (const genre of track.genres || []) {
        genreCounts.set(genre, (genreCounts.get(genre) || 0) + 1);
      }
    }

    const dominantGenres = Array.from(genreCounts.entries())
      .sort((a, b) => b[1] - a[1])
      .slice(0, 3)
      .map(([genre]) => genre);

    return { averageEnergy: avgEnergy, averageValence: avgValence, dominantGenres };
  }

  private getCacheKey(userId: string, context: PlaylistContext): string {
    const contextHash = `${context.timeOfDay}_${context.activity || 'none'}_${context.weather?.condition || 'none'}`;
    return `${this.cachePrefix}${userId}:${contextHash}`;
  }

  private getCurrentTimeOfDay(): PlaylistContext['timeOfDay'] {
    const hour = new Date().getHours();
    if (hour < 6) return 'night';
    if (hour < 9) return 'dawn';
    if (hour < 12) return 'morning';
    if (hour < 17) return 'afternoon';
    if (hour < 21) return 'evening';
    return 'night';
  }

  private getSeason(latitude?: number): PlaylistContext['season'] {
    const month = new Date().getMonth();
    const isNorthern = !latitude || latitude >= 0;

    if (month >= 2 && month <= 4) return isNorthern ? 'spring' : 'autumn';
    if (month >= 5 && month <= 7) return isNorthern ? 'summer' : 'winter';
    if (month >= 8 && month <= 10) return isNorthern ? 'autumn' : 'spring';
    return isNorthern ? 'winter' : 'summer';
  }
}
