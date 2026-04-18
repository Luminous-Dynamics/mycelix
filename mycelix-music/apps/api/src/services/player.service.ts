// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Player State Service
 *
 * Manages player state synchronization across devices,
 * queue management, and playback sessions.
 */

import { Pool } from 'pg';
import { EventEmitter } from '../events';
import { getLogger } from '../logging';
import { getMetrics } from '../metrics';
import Redis from 'ioredis';

const logger = getLogger();

/**
 * Playback state
 */
export interface PlaybackState {
  isPlaying: boolean;
  currentSongId: string | null;
  position: number; // Current position in seconds
  duration: number;
  volume: number;
  shuffle: boolean;
  repeat: 'off' | 'one' | 'all';
  quality: 'low' | 'medium' | 'high' | 'lossless';
  updatedAt: Date;
}

/**
 * Queue item
 */
export interface QueueItem {
  id: string;
  songId: string;
  title: string;
  artist: string;
  artistAddress: string;
  duration: number;
  coverArt: string;
  addedAt: Date;
  addedBy: 'user' | 'autoplay' | 'radio';
}

/**
 * Player session
 */
export interface PlayerSession {
  sessionId: string;
  walletAddress: string;
  deviceId: string;
  deviceName: string;
  deviceType: 'web' | 'mobile' | 'desktop' | 'speaker';
  isActive: boolean;
  state: PlaybackState;
  queue: QueueItem[];
  queuePosition: number;
  history: string[]; // Song IDs
  createdAt: Date;
  lastActivity: Date;
}

/**
 * Device info
 */
export interface DeviceInfo {
  deviceId: string;
  deviceName: string;
  deviceType: 'web' | 'mobile' | 'desktop' | 'speaker';
  isActive: boolean;
  volume: number;
  lastSeen: Date;
}

/**
 * Listening session for analytics
 */
export interface ListeningSession {
  sessionId: string;
  songId: string;
  startedAt: Date;
  duration: number;
  completed: boolean;
  skipped: boolean;
  source: 'library' | 'search' | 'playlist' | 'radio' | 'recommendation';
}

/**
 * Player Service
 */
export class PlayerService {
  private redis: Redis | null = null;

  constructor(
    private pool: Pool,
    private events: EventEmitter,
    redisUrl?: string
  ) {
    if (redisUrl) {
      this.redis = new Redis(redisUrl);
    }

    const metrics = getMetrics();
    metrics.createCounter('player_sessions_total', 'Player sessions', ['action']);
    metrics.createGauge('player_active_sessions', 'Active player sessions', []);
    metrics.createCounter('player_queue_operations', 'Queue operations', ['operation']);
  }

  /**
   * Create or get player session
   */
  async getOrCreateSession(
    walletAddress: string,
    deviceId: string,
    deviceInfo: Partial<DeviceInfo> = {}
  ): Promise<PlayerSession> {
    // Try to get existing session
    const existing = await this.getSession(walletAddress, deviceId);
    if (existing) {
      return existing;
    }

    // Create new session
    const sessionId = this.generateSessionId();
    const session: PlayerSession = {
      sessionId,
      walletAddress,
      deviceId,
      deviceName: deviceInfo.deviceName || 'Unknown Device',
      deviceType: deviceInfo.deviceType || 'web',
      isActive: true,
      state: {
        isPlaying: false,
        currentSongId: null,
        position: 0,
        duration: 0,
        volume: 1.0,
        shuffle: false,
        repeat: 'off',
        quality: 'high',
        updatedAt: new Date(),
      },
      queue: [],
      queuePosition: -1,
      history: [],
      createdAt: new Date(),
      lastActivity: new Date(),
    };

    await this.saveSession(session);

    getMetrics().incCounter('player_sessions_total', { action: 'created' });
    this.events.emit('player:session:created', { session });

    return session;
  }

  /**
   * Get session
   */
  async getSession(
    walletAddress: string,
    deviceId: string
  ): Promise<PlayerSession | null> {
    if (this.redis) {
      const key = `player:session:${walletAddress}:${deviceId}`;
      const data = await this.redis.get(key);
      if (data) {
        return JSON.parse(data);
      }
    }

    // Fallback to database
    const result = await this.pool.query(
      `SELECT * FROM player_sessions WHERE wallet_address = $1 AND device_id = $2`,
      [walletAddress, deviceId]
    );

    if (result.rows.length === 0) return null;

    const row = result.rows[0];
    return {
      sessionId: row.session_id,
      walletAddress: row.wallet_address,
      deviceId: row.device_id,
      deviceName: row.device_name,
      deviceType: row.device_type,
      isActive: row.is_active,
      state: row.state,
      queue: row.queue || [],
      queuePosition: row.queue_position,
      history: row.history || [],
      createdAt: row.created_at,
      lastActivity: row.last_activity,
    };
  }

  /**
   * Save session
   */
  private async saveSession(session: PlayerSession): Promise<void> {
    if (this.redis) {
      const key = `player:session:${session.walletAddress}:${session.deviceId}`;
      await this.redis.setex(key, 86400, JSON.stringify(session));
    }

    await this.pool.query(
      `
      INSERT INTO player_sessions (
        session_id, wallet_address, device_id, device_name, device_type,
        is_active, state, queue, queue_position, history, created_at, last_activity
      ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12)
      ON CONFLICT (wallet_address, device_id)
      DO UPDATE SET
        is_active = $6,
        state = $7,
        queue = $8,
        queue_position = $9,
        history = $10,
        last_activity = $12
      `,
      [
        session.sessionId,
        session.walletAddress,
        session.deviceId,
        session.deviceName,
        session.deviceType,
        session.isActive,
        JSON.stringify(session.state),
        JSON.stringify(session.queue),
        session.queuePosition,
        JSON.stringify(session.history),
        session.createdAt,
        session.lastActivity,
      ]
    );
  }

  /**
   * Update playback state
   */
  async updateState(
    walletAddress: string,
    deviceId: string,
    state: Partial<PlaybackState>
  ): Promise<PlayerSession> {
    const session = await this.getSession(walletAddress, deviceId);
    if (!session) {
      throw new Error('Session not found');
    }

    session.state = {
      ...session.state,
      ...state,
      updatedAt: new Date(),
    };
    session.lastActivity = new Date();

    await this.saveSession(session);

    // Emit state change event
    this.events.emit('player:state:updated', {
      walletAddress,
      deviceId,
      state: session.state,
    });

    return session;
  }

  /**
   * Play a song
   */
  async play(
    walletAddress: string,
    deviceId: string,
    songId: string,
    source: ListeningSession['source'] = 'library'
  ): Promise<PlayerSession> {
    const session = await this.getSession(walletAddress, deviceId);
    if (!session) {
      throw new Error('Session not found');
    }

    // Get song details
    const songResult = await this.pool.query(
      `SELECT id, title, artist_name, artist_address, duration, cover_art_url
       FROM songs WHERE id = $1`,
      [songId]
    );

    if (songResult.rows.length === 0) {
      throw new Error('Song not found');
    }

    const song = songResult.rows[0];

    // End previous listening session if exists
    if (session.state.currentSongId && session.state.isPlaying) {
      await this.recordListeningSession(session, false);
    }

    // Update state
    session.state = {
      ...session.state,
      isPlaying: true,
      currentSongId: songId,
      position: 0,
      duration: song.duration,
      updatedAt: new Date(),
    };

    // Add to history
    if (!session.history.includes(songId)) {
      session.history = [songId, ...session.history.slice(0, 99)];
    }

    session.lastActivity = new Date();
    await this.saveSession(session);

    // Start listening session
    await this.startListeningSession(session, source);

    // Emit events
    this.events.emit('player:play', {
      walletAddress,
      deviceId,
      songId,
      song,
    });

    getMetrics().incCounter('player_sessions_total', { action: 'play' });

    return session;
  }

  /**
   * Pause playback
   */
  async pause(walletAddress: string, deviceId: string): Promise<PlayerSession> {
    const session = await this.getSession(walletAddress, deviceId);
    if (!session) {
      throw new Error('Session not found');
    }

    session.state.isPlaying = false;
    session.state.updatedAt = new Date();
    session.lastActivity = new Date();

    await this.saveSession(session);

    this.events.emit('player:pause', {
      walletAddress,
      deviceId,
      songId: session.state.currentSongId,
      position: session.state.position,
    });

    return session;
  }

  /**
   * Seek to position
   */
  async seek(
    walletAddress: string,
    deviceId: string,
    position: number
  ): Promise<PlayerSession> {
    const session = await this.getSession(walletAddress, deviceId);
    if (!session) {
      throw new Error('Session not found');
    }

    session.state.position = Math.max(0, Math.min(position, session.state.duration));
    session.state.updatedAt = new Date();
    session.lastActivity = new Date();

    await this.saveSession(session);

    this.events.emit('player:seek', {
      walletAddress,
      deviceId,
      position: session.state.position,
    });

    return session;
  }

  /**
   * Add to queue
   */
  async addToQueue(
    walletAddress: string,
    deviceId: string,
    songIds: string[],
    position: 'next' | 'last' = 'last'
  ): Promise<PlayerSession> {
    const session = await this.getSession(walletAddress, deviceId);
    if (!session) {
      throw new Error('Session not found');
    }

    // Get song details
    const songsResult = await this.pool.query(
      `SELECT id, title, artist_name, artist_address, duration, cover_art_url
       FROM songs WHERE id = ANY($1)`,
      [songIds]
    );

    const songs = new Map(songsResult.rows.map((s) => [s.id, s]));
    const newItems: QueueItem[] = songIds
      .map((songId) => {
        const song = songs.get(songId);
        if (!song) return null;
        return {
          id: this.generateQueueItemId(),
          songId,
          title: song.title,
          artist: song.artist_name,
          artistAddress: song.artist_address,
          duration: song.duration,
          coverArt: song.cover_art_url,
          addedAt: new Date(),
          addedBy: 'user' as const,
        };
      })
      .filter(Boolean) as QueueItem[];

    if (position === 'next') {
      session.queue.splice(session.queuePosition + 1, 0, ...newItems);
    } else {
      session.queue.push(...newItems);
    }

    session.lastActivity = new Date();
    await this.saveSession(session);

    getMetrics().incCounter('player_queue_operations', { operation: 'add' });

    this.events.emit('player:queue:updated', {
      walletAddress,
      deviceId,
      queue: session.queue,
    });

    return session;
  }

  /**
   * Remove from queue
   */
  async removeFromQueue(
    walletAddress: string,
    deviceId: string,
    queueItemId: string
  ): Promise<PlayerSession> {
    const session = await this.getSession(walletAddress, deviceId);
    if (!session) {
      throw new Error('Session not found');
    }

    const index = session.queue.findIndex((item) => item.id === queueItemId);
    if (index !== -1) {
      session.queue.splice(index, 1);
      if (index <= session.queuePosition) {
        session.queuePosition = Math.max(-1, session.queuePosition - 1);
      }
    }

    session.lastActivity = new Date();
    await this.saveSession(session);

    getMetrics().incCounter('player_queue_operations', { operation: 'remove' });

    return session;
  }

  /**
   * Reorder queue
   */
  async reorderQueue(
    walletAddress: string,
    deviceId: string,
    fromIndex: number,
    toIndex: number
  ): Promise<PlayerSession> {
    const session = await this.getSession(walletAddress, deviceId);
    if (!session) {
      throw new Error('Session not found');
    }

    if (fromIndex < 0 || fromIndex >= session.queue.length) {
      throw new Error('Invalid from index');
    }
    if (toIndex < 0 || toIndex >= session.queue.length) {
      throw new Error('Invalid to index');
    }

    const [item] = session.queue.splice(fromIndex, 1);
    session.queue.splice(toIndex, 0, item);

    // Adjust queue position if needed
    if (fromIndex === session.queuePosition) {
      session.queuePosition = toIndex;
    } else if (fromIndex < session.queuePosition && toIndex >= session.queuePosition) {
      session.queuePosition--;
    } else if (fromIndex > session.queuePosition && toIndex <= session.queuePosition) {
      session.queuePosition++;
    }

    session.lastActivity = new Date();
    await this.saveSession(session);

    getMetrics().incCounter('player_queue_operations', { operation: 'reorder' });

    return session;
  }

  /**
   * Clear queue
   */
  async clearQueue(
    walletAddress: string,
    deviceId: string
  ): Promise<PlayerSession> {
    const session = await this.getSession(walletAddress, deviceId);
    if (!session) {
      throw new Error('Session not found');
    }

    session.queue = [];
    session.queuePosition = -1;
    session.lastActivity = new Date();

    await this.saveSession(session);

    getMetrics().incCounter('player_queue_operations', { operation: 'clear' });

    return session;
  }

  /**
   * Skip to next
   */
  async next(walletAddress: string, deviceId: string): Promise<PlayerSession> {
    const session = await this.getSession(walletAddress, deviceId);
    if (!session) {
      throw new Error('Session not found');
    }

    // Record skip for current song
    if (session.state.currentSongId) {
      await this.recordListeningSession(session, true);
    }

    // Handle shuffle
    let nextPosition: number;
    if (session.state.shuffle) {
      const availablePositions = session.queue
        .map((_, i) => i)
        .filter((i) => i !== session.queuePosition);
      if (availablePositions.length === 0) {
        nextPosition = 0;
      } else {
        nextPosition =
          availablePositions[Math.floor(Math.random() * availablePositions.length)];
      }
    } else {
      nextPosition = session.queuePosition + 1;
    }

    // Handle repeat
    if (nextPosition >= session.queue.length) {
      if (session.state.repeat === 'all') {
        nextPosition = 0;
      } else {
        // End of queue
        session.state.isPlaying = false;
        session.state.currentSongId = null;
        session.queuePosition = -1;
        await this.saveSession(session);
        return session;
      }
    }

    session.queuePosition = nextPosition;
    const nextItem = session.queue[nextPosition];

    if (nextItem) {
      return this.play(walletAddress, deviceId, nextItem.songId, 'library');
    }

    return session;
  }

  /**
   * Skip to previous
   */
  async previous(walletAddress: string, deviceId: string): Promise<PlayerSession> {
    const session = await this.getSession(walletAddress, deviceId);
    if (!session) {
      throw new Error('Session not found');
    }

    // If more than 3 seconds in, restart current song
    if (session.state.position > 3) {
      session.state.position = 0;
      session.state.updatedAt = new Date();
      await this.saveSession(session);
      return session;
    }

    let prevPosition = session.queuePosition - 1;

    if (prevPosition < 0) {
      if (session.state.repeat === 'all') {
        prevPosition = session.queue.length - 1;
      } else {
        prevPosition = 0;
      }
    }

    session.queuePosition = prevPosition;
    const prevItem = session.queue[prevPosition];

    if (prevItem) {
      return this.play(walletAddress, deviceId, prevItem.songId, 'library');
    }

    return session;
  }

  /**
   * Get all devices for user
   */
  async getDevices(walletAddress: string): Promise<DeviceInfo[]> {
    const result = await this.pool.query(
      `
      SELECT device_id, device_name, device_type, is_active,
             (state->>'volume')::float as volume, last_activity
      FROM player_sessions
      WHERE wallet_address = $1
      ORDER BY last_activity DESC
      `,
      [walletAddress]
    );

    return result.rows.map((row) => ({
      deviceId: row.device_id,
      deviceName: row.device_name,
      deviceType: row.device_type,
      isActive: row.is_active,
      volume: row.volume || 1.0,
      lastSeen: row.last_activity,
    }));
  }

  /**
   * Transfer playback to another device
   */
  async transferPlayback(
    walletAddress: string,
    fromDeviceId: string,
    toDeviceId: string
  ): Promise<PlayerSession> {
    const fromSession = await this.getSession(walletAddress, fromDeviceId);
    const toSession = await this.getSession(walletAddress, toDeviceId);

    if (!fromSession || !toSession) {
      throw new Error('Session not found');
    }

    // Transfer state
    toSession.state = { ...fromSession.state };
    toSession.queue = [...fromSession.queue];
    toSession.queuePosition = fromSession.queuePosition;
    toSession.isActive = true;
    toSession.lastActivity = new Date();

    // Deactivate source
    fromSession.isActive = false;
    fromSession.state.isPlaying = false;

    await Promise.all([this.saveSession(toSession), this.saveSession(fromSession)]);

    this.events.emit('player:transfer', {
      walletAddress,
      fromDeviceId,
      toDeviceId,
    });

    return toSession;
  }

  /**
   * Start listening session
   */
  private async startListeningSession(
    session: PlayerSession,
    source: ListeningSession['source']
  ): Promise<void> {
    await this.pool.query(
      `
      INSERT INTO listening_sessions (
        session_id, wallet_address, song_id, started_at, source
      ) VALUES ($1, $2, $3, $4, $5)
      `,
      [
        session.sessionId,
        session.walletAddress,
        session.state.currentSongId,
        new Date(),
        source,
      ]
    );
  }

  /**
   * Record completed listening session
   */
  private async recordListeningSession(
    session: PlayerSession,
    skipped: boolean
  ): Promise<void> {
    const completed = session.state.position >= session.state.duration * 0.9;

    await this.pool.query(
      `
      UPDATE listening_sessions
      SET duration = $3, completed = $4, skipped = $5, ended_at = $6
      WHERE session_id = $1 AND song_id = $2 AND ended_at IS NULL
      `,
      [
        session.sessionId,
        session.state.currentSongId,
        session.state.position,
        completed,
        skipped,
        new Date(),
      ]
    );
  }

  /**
   * Generate session ID
   */
  private generateSessionId(): string {
    return `sess_${Date.now()}_${Math.random().toString(36).slice(2, 11)}`;
  }

  /**
   * Generate queue item ID
   */
  private generateQueueItemId(): string {
    return `qi_${Date.now()}_${Math.random().toString(36).slice(2, 8)}`;
  }

  /**
   * Cleanup inactive sessions
   */
  async cleanupInactiveSessions(maxAge = 86400000): Promise<number> {
    const cutoff = new Date(Date.now() - maxAge);

    const result = await this.pool.query(
      `DELETE FROM player_sessions WHERE last_activity < $1`,
      [cutoff]
    );

    logger.info(`Cleaned up ${result.rowCount} inactive player sessions`);
    return result.rowCount || 0;
  }
}

/**
 * Migration SQL
 */
export const PLAYER_MIGRATION_SQL = `
CREATE TABLE IF NOT EXISTS player_sessions (
  session_id VARCHAR(50) PRIMARY KEY,
  wallet_address VARCHAR(42) NOT NULL,
  device_id VARCHAR(100) NOT NULL,
  device_name VARCHAR(200),
  device_type VARCHAR(20) DEFAULT 'web',
  is_active BOOLEAN DEFAULT true,
  state JSONB DEFAULT '{}',
  queue JSONB DEFAULT '[]',
  queue_position INTEGER DEFAULT -1,
  history JSONB DEFAULT '[]',
  created_at TIMESTAMPTZ DEFAULT NOW(),
  last_activity TIMESTAMPTZ DEFAULT NOW(),
  UNIQUE(wallet_address, device_id)
);

CREATE TABLE IF NOT EXISTS listening_sessions (
  id BIGSERIAL PRIMARY KEY,
  session_id VARCHAR(50) NOT NULL,
  wallet_address VARCHAR(42) NOT NULL,
  song_id UUID NOT NULL REFERENCES songs(id),
  started_at TIMESTAMPTZ NOT NULL,
  ended_at TIMESTAMPTZ,
  duration INTEGER DEFAULT 0,
  completed BOOLEAN DEFAULT false,
  skipped BOOLEAN DEFAULT false,
  source VARCHAR(20)
);

CREATE INDEX idx_player_sessions_wallet ON player_sessions(wallet_address);
CREATE INDEX idx_player_sessions_activity ON player_sessions(last_activity);
CREATE INDEX idx_listening_sessions_wallet ON listening_sessions(wallet_address);
CREATE INDEX idx_listening_sessions_song ON listening_sessions(song_id);
`;

export default PlayerService;
