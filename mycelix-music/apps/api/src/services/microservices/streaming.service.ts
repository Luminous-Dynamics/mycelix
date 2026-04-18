// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Streaming Microservice
 * Handles audio streaming, adaptive bitrate, CDN integration
 */

import { Redis } from 'ioredis';
import { S3Client, GetObjectCommand } from '@aws-sdk/client-s3';
import { getSignedUrl } from '@aws-sdk/s3-request-presigner';

export type AudioQuality = 'LOW_128' | 'STANDARD_256' | 'HIGH_320' | 'LOSSLESS_FLAC' | 'HI_RES_24BIT';

interface StreamSession {
  id: string;
  userId: string;
  trackId: string;
  quality: AudioQuality;
  startedAt: Date;
  bytesStreamed: number;
  lastActivity: Date;
}

interface TrackAudioFiles {
  trackId: string;
  files: Map<AudioQuality, AudioFileInfo>;
}

interface AudioFileInfo {
  quality: AudioQuality;
  format: string;
  bitrate: number;
  sampleRate: number;
  bitDepth?: number;
  size: number;
  key: string;
  duration: number;
}

interface StreamingConfig {
  cdnBaseUrl: string;
  s3Bucket: string;
  chunkSize: number;
  sessionTimeout: number;
  maxConcurrentStreams: number;
}

export class StreamingService {
  private redis: Redis;
  private s3: S3Client;
  private config: StreamingConfig;
  private activeSessions: Map<string, StreamSession> = new Map();

  constructor(redis: Redis, s3: S3Client, config: StreamingConfig) {
    this.redis = redis;
    this.s3 = s3;
    this.config = config;
  }

  /**
   * Get streaming URL for a track
   */
  async getStreamUrl(
    trackId: string,
    userId: string,
    requestedQuality: AudioQuality,
    userTier: string
  ): Promise<{ url: string; quality: AudioQuality; expiresAt: Date }> {
    // Validate user can access requested quality
    const allowedQuality = this.getAllowedQuality(requestedQuality, userTier);

    // Get audio file info
    const audioFile = await this.getAudioFile(trackId, allowedQuality);
    if (!audioFile) {
      throw new Error(`Audio file not found for track ${trackId} at quality ${allowedQuality}`);
    }

    // Create stream session
    const session = await this.createStreamSession(userId, trackId, allowedQuality);

    // Generate signed URL (15 minute expiry)
    const expiresAt = new Date(Date.now() + 15 * 60 * 1000);
    const signedUrl = await this.generateSignedUrl(audioFile.key, session.id, expiresAt);

    // Use CDN URL if available
    const url = this.config.cdnBaseUrl
      ? `${this.config.cdnBaseUrl}/${audioFile.key}?session=${session.id}&sig=${encodeURIComponent(signedUrl)}`
      : signedUrl;

    return {
      url,
      quality: allowedQuality,
      expiresAt,
    };
  }

  /**
   * Get adaptive streaming manifest (HLS/DASH)
   */
  async getAdaptiveManifest(
    trackId: string,
    userId: string,
    userTier: string
  ): Promise<{ manifestUrl: string; type: 'hls' | 'dash' }> {
    const maxQuality = this.getMaxQualityForTier(userTier);
    const session = await this.createStreamSession(userId, trackId, maxQuality);

    // Generate manifest with quality variants user can access
    const qualities = this.getQualitiesUpTo(maxQuality);
    const manifestKey = `manifests/${trackId}/master.m3u8`;

    const signedUrl = await this.generateSignedUrl(manifestKey, session.id, new Date(Date.now() + 4 * 60 * 60 * 1000));

    return {
      manifestUrl: this.config.cdnBaseUrl
        ? `${this.config.cdnBaseUrl}/${manifestKey}?session=${session.id}`
        : signedUrl,
      type: 'hls',
    };
  }

  /**
   * Get download URL for offline playback
   */
  async getDownloadUrl(
    trackId: string,
    userId: string,
    quality: AudioQuality,
    userTier: string
  ): Promise<{ url: string; expiresAt: Date }> {
    // Check if user can download
    if (!this.canDownload(userTier)) {
      throw new Error('Download not available for your subscription tier');
    }

    const allowedQuality = this.getAllowedQuality(quality, userTier);
    const audioFile = await this.getAudioFile(trackId, allowedQuality);
    if (!audioFile) {
      throw new Error(`Audio file not found for track ${trackId}`);
    }

    // Generate download URL (1 hour expiry)
    const expiresAt = new Date(Date.now() + 60 * 60 * 1000);
    const signedUrl = await this.generateSignedUrl(audioFile.key, `download-${userId}`, expiresAt);

    // Log download for royalty tracking
    await this.logDownload(trackId, userId, allowedQuality);

    return { url: signedUrl, expiresAt };
  }

  /**
   * Record streaming progress for royalty calculation
   */
  async recordStreamProgress(
    sessionId: string,
    bytesStreamed: number,
    currentPosition: number
  ): Promise<void> {
    const session = this.activeSessions.get(sessionId);
    if (!session) return;

    session.bytesStreamed = bytesStreamed;
    session.lastActivity = new Date();

    // Check if this qualifies as a "play" (>30 seconds)
    if (currentPosition >= 30 && !(session as any).playRecorded) {
      await this.recordPlay(session);
      (session as any).playRecorded = true;
    }

    // Update session in Redis for distributed tracking
    await this.redis.hset(
      `stream:session:${sessionId}`,
      {
        bytesStreamed: bytesStreamed.toString(),
        currentPosition: currentPosition.toString(),
        lastActivity: session.lastActivity.toISOString(),
      }
    );
  }

  /**
   * End streaming session
   */
  async endStreamSession(sessionId: string): Promise<void> {
    const session = this.activeSessions.get(sessionId);
    if (!session) return;

    // Record final stats
    await this.redis.hset(
      `stream:session:${sessionId}`,
      { endedAt: new Date().toISOString() }
    );

    // Move to analytics queue
    await this.redis.lpush('analytics:streams', JSON.stringify({
      ...session,
      endedAt: new Date(),
    }));

    this.activeSessions.delete(sessionId);
    await this.redis.del(`stream:session:${sessionId}`);
  }

  /**
   * Get real-time listener count for a track
   */
  async getListenerCount(trackId: string): Promise<number> {
    const keys = await this.redis.keys(`stream:session:*`);
    let count = 0;

    for (const key of keys) {
      const session = await this.redis.hgetall(key);
      if (session.trackId === trackId && !session.endedAt) {
        // Check if session is still active (last activity within 60 seconds)
        const lastActivity = new Date(session.lastActivity);
        if (Date.now() - lastActivity.getTime() < 60000) {
          count++;
        }
      }
    }

    return count;
  }

  /**
   * Get audio files for a track (all quality versions)
   */
  async getAudioFiles(trackId: string, maxQuality: AudioQuality): Promise<AudioFileInfo[]> {
    const cacheKey = `track:audio:${trackId}`;
    const cached = await this.redis.get(cacheKey);

    if (cached) {
      const files: AudioFileInfo[] = JSON.parse(cached);
      return files.filter(f => this.qualityRank(f.quality) <= this.qualityRank(maxQuality));
    }

    // Fetch from database (this would be injected)
    const files = await this.fetchTrackAudioFiles(trackId);
    await this.redis.setex(cacheKey, 3600, JSON.stringify(files));

    return files.filter(f => this.qualityRank(f.quality) <= this.qualityRank(maxQuality));
  }

  // Private helper methods

  private async createStreamSession(
    userId: string,
    trackId: string,
    quality: AudioQuality
  ): Promise<StreamSession> {
    // Check concurrent stream limit
    const userStreams = await this.getUserActiveStreams(userId);
    if (userStreams >= this.config.maxConcurrentStreams) {
      // End oldest session
      await this.endOldestUserSession(userId);
    }

    const session: StreamSession = {
      id: this.generateSessionId(),
      userId,
      trackId,
      quality,
      startedAt: new Date(),
      bytesStreamed: 0,
      lastActivity: new Date(),
    };

    this.activeSessions.set(session.id, session);

    await this.redis.hset(
      `stream:session:${session.id}`,
      {
        ...session,
        startedAt: session.startedAt.toISOString(),
        lastActivity: session.lastActivity.toISOString(),
      }
    );
    await this.redis.expire(`stream:session:${session.id}`, this.config.sessionTimeout);

    return session;
  }

  private async generateSignedUrl(
    key: string,
    sessionId: string,
    expiresAt: Date
  ): Promise<string> {
    const command = new GetObjectCommand({
      Bucket: this.config.s3Bucket,
      Key: key,
    });

    const expiresIn = Math.floor((expiresAt.getTime() - Date.now()) / 1000);
    return getSignedUrl(this.s3, command, { expiresIn });
  }

  private async getAudioFile(trackId: string, quality: AudioQuality): Promise<AudioFileInfo | null> {
    const files = await this.getAudioFiles(trackId, quality);
    return files.find(f => f.quality === quality) || files[files.length - 1] || null;
  }

  private getAllowedQuality(requested: AudioQuality, userTier: string): AudioQuality {
    const maxAllowed = this.getMaxQualityForTier(userTier);
    return this.qualityRank(requested) > this.qualityRank(maxAllowed)
      ? maxAllowed
      : requested;
  }

  private getMaxQualityForTier(tier: string): AudioQuality {
    const tierQuality: Record<string, AudioQuality> = {
      FREE: 'LOW_128',
      PREMIUM: 'HIGH_320',
      ARTIST_PRO: 'LOSSLESS_FLAC',
      LABEL_ENTERPRISE: 'HI_RES_24BIT',
    };
    return tierQuality[tier] || 'STANDARD_256';
  }

  private getQualitiesUpTo(maxQuality: AudioQuality): AudioQuality[] {
    const allQualities: AudioQuality[] = ['LOW_128', 'STANDARD_256', 'HIGH_320', 'LOSSLESS_FLAC', 'HI_RES_24BIT'];
    const maxIndex = allQualities.indexOf(maxQuality);
    return allQualities.slice(0, maxIndex + 1);
  }

  private qualityRank(quality: AudioQuality): number {
    const ranks: Record<AudioQuality, number> = {
      LOW_128: 1,
      STANDARD_256: 2,
      HIGH_320: 3,
      LOSSLESS_FLAC: 4,
      HI_RES_24BIT: 5,
    };
    return ranks[quality];
  }

  private canDownload(tier: string): boolean {
    return ['PREMIUM', 'ARTIST_PRO', 'LABEL_ENTERPRISE'].includes(tier);
  }

  private async recordPlay(session: StreamSession): Promise<void> {
    await this.redis.lpush('analytics:plays', JSON.stringify({
      trackId: session.trackId,
      userId: session.userId,
      quality: session.quality,
      playedAt: new Date(),
    }));
  }

  private async logDownload(trackId: string, userId: string, quality: AudioQuality): Promise<void> {
    await this.redis.lpush('analytics:downloads', JSON.stringify({
      trackId,
      userId,
      quality,
      downloadedAt: new Date(),
    }));
  }

  private async getUserActiveStreams(userId: string): Promise<number> {
    let count = 0;
    for (const session of this.activeSessions.values()) {
      if (session.userId === userId) count++;
    }
    return count;
  }

  private async endOldestUserSession(userId: string): Promise<void> {
    let oldest: StreamSession | null = null;
    for (const session of this.activeSessions.values()) {
      if (session.userId === userId) {
        if (!oldest || session.startedAt < oldest.startedAt) {
          oldest = session;
        }
      }
    }
    if (oldest) {
      await this.endStreamSession(oldest.id);
    }
  }

  private generateSessionId(): string {
    return `${Date.now()}-${Math.random().toString(36).slice(2)}`;
  }

  private async fetchTrackAudioFiles(trackId: string): Promise<AudioFileInfo[]> {
    // This would be implemented to fetch from database
    // Placeholder implementation
    return [];
  }
}

export default StreamingService;
