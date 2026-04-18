// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { Redis } from 'ioredis';
import { S3Client } from '@aws-sdk/client-s3';
import { StreamingService, AudioQuality } from '../../services/microservices/streaming.service';

// Mock Redis
const mockRedis = {
  get: vi.fn(),
  set: vi.fn(),
  setex: vi.fn(),
  del: vi.fn(),
  hset: vi.fn(),
  hget: vi.fn(),
  hgetall: vi.fn(),
  keys: vi.fn(),
  lpush: vi.fn(),
  expire: vi.fn(),
  zadd: vi.fn(),
  zrem: vi.fn(),
  publish: vi.fn(),
  subscribe: vi.fn(),
  duplicate: vi.fn().mockReturnThis(),
  on: vi.fn(),
} as unknown as Redis;

// Mock S3Client
const mockS3 = {
  send: vi.fn(),
} as unknown as S3Client;

// Mock config
const mockConfig = {
  cdnBaseUrl: 'https://cdn.example.com',
  s3Bucket: 'mycelix-audio',
  chunkSize: 1024 * 1024,
  sessionTimeout: 3600,
  maxConcurrentStreams: 5,
};

describe('StreamingService', () => {
  let service: StreamingService;

  beforeEach(() => {
    vi.clearAllMocks();
    service = new StreamingService(mockRedis, mockS3, mockConfig);
  });

  describe('getStreamUrl', () => {
    it('returns streaming URL for valid track and user', async () => {
      // Mock audio file data
      const mockAudioFiles = [
        {
          quality: 'STANDARD_256',
          format: 'aac',
          bitrate: 256000,
          sampleRate: 44100,
          size: 5000000,
          key: 'tracks/track-1/standard.m4a',
          duration: 180,
        },
      ];

      (mockRedis.get as any).mockResolvedValueOnce(JSON.stringify(mockAudioFiles));

      const result = await service.getStreamUrl(
        'track-1',
        'user-1',
        'STANDARD_256',
        'PREMIUM'
      );

      expect(result).toHaveProperty('url');
      expect(result).toHaveProperty('quality', 'STANDARD_256');
      expect(result).toHaveProperty('expiresAt');
      expect(new Date(result.expiresAt).getTime()).toBeGreaterThan(Date.now());
    });

    it('downgrades quality for free tier users', async () => {
      const mockAudioFiles = [
        {
          quality: 'LOW_128',
          format: 'aac',
          bitrate: 128000,
          sampleRate: 44100,
          size: 3000000,
          key: 'tracks/track-1/low.m4a',
          duration: 180,
        },
        {
          quality: 'HIGH_320',
          format: 'mp3',
          bitrate: 320000,
          sampleRate: 44100,
          size: 7000000,
          key: 'tracks/track-1/high.mp3',
          duration: 180,
        },
      ];

      (mockRedis.get as any).mockResolvedValueOnce(JSON.stringify(mockAudioFiles));

      const result = await service.getStreamUrl(
        'track-1',
        'user-1',
        'HIGH_320', // Requested high quality
        'FREE' // But user is free tier
      );

      // Should be downgraded to LOW_128
      expect(result.quality).toBe('LOW_128');
    });

    it('allows lossless for premium pro users', async () => {
      const mockAudioFiles = [
        {
          quality: 'LOSSLESS_FLAC',
          format: 'flac',
          bitrate: 0,
          sampleRate: 44100,
          size: 30000000,
          key: 'tracks/track-1/lossless.flac',
          duration: 180,
        },
      ];

      (mockRedis.get as any).mockResolvedValueOnce(JSON.stringify(mockAudioFiles));

      const result = await service.getStreamUrl(
        'track-1',
        'user-1',
        'LOSSLESS_FLAC',
        'ARTIST_PRO'
      );

      expect(result.quality).toBe('LOSSLESS_FLAC');
    });

    it('creates a stream session', async () => {
      const mockAudioFiles = [
        {
          quality: 'STANDARD_256',
          format: 'aac',
          bitrate: 256000,
          sampleRate: 44100,
          size: 5000000,
          key: 'tracks/track-1/standard.m4a',
          duration: 180,
        },
      ];

      (mockRedis.get as any).mockResolvedValueOnce(JSON.stringify(mockAudioFiles));

      await service.getStreamUrl('track-1', 'user-1', 'STANDARD_256', 'PREMIUM');

      expect(mockRedis.hset).toHaveBeenCalled();
      expect(mockRedis.expire).toHaveBeenCalled();
    });

    it('throws error when audio file not found', async () => {
      (mockRedis.get as any).mockResolvedValueOnce(JSON.stringify([]));

      await expect(
        service.getStreamUrl('nonexistent', 'user-1', 'STANDARD_256', 'PREMIUM')
      ).rejects.toThrow('Audio file not found');
    });
  });

  describe('getDownloadUrl', () => {
    it('returns download URL for premium users', async () => {
      const mockAudioFiles = [
        {
          quality: 'HIGH_320',
          format: 'mp3',
          bitrate: 320000,
          sampleRate: 44100,
          size: 7000000,
          key: 'tracks/track-1/high.mp3',
          duration: 180,
        },
      ];

      (mockRedis.get as any).mockResolvedValueOnce(JSON.stringify(mockAudioFiles));

      const result = await service.getDownloadUrl(
        'track-1',
        'user-1',
        'HIGH_320',
        'PREMIUM'
      );

      expect(result).toHaveProperty('url');
      expect(result).toHaveProperty('expiresAt');
    });

    it('throws error for free tier users attempting download', async () => {
      await expect(
        service.getDownloadUrl('track-1', 'user-1', 'HIGH_320', 'FREE')
      ).rejects.toThrow('Download not available');
    });

    it('logs download for royalty tracking', async () => {
      const mockAudioFiles = [
        {
          quality: 'HIGH_320',
          format: 'mp3',
          bitrate: 320000,
          sampleRate: 44100,
          size: 7000000,
          key: 'tracks/track-1/high.mp3',
          duration: 180,
        },
      ];

      (mockRedis.get as any).mockResolvedValueOnce(JSON.stringify(mockAudioFiles));

      await service.getDownloadUrl('track-1', 'user-1', 'HIGH_320', 'PREMIUM');

      expect(mockRedis.lpush).toHaveBeenCalledWith(
        'analytics:downloads',
        expect.stringContaining('track-1')
      );
    });
  });

  describe('recordStreamProgress', () => {
    it('updates session with progress', async () => {
      // Create a session first
      const mockAudioFiles = [
        {
          quality: 'STANDARD_256',
          format: 'aac',
          bitrate: 256000,
          sampleRate: 44100,
          size: 5000000,
          key: 'tracks/track-1/standard.m4a',
          duration: 180,
        },
      ];

      (mockRedis.get as any).mockResolvedValueOnce(JSON.stringify(mockAudioFiles));

      const { url } = await service.getStreamUrl('track-1', 'user-1', 'STANDARD_256', 'PREMIUM');

      // Extract session ID from URL
      const sessionMatch = url.match(/session=([^&]+)/);
      const sessionId = sessionMatch?.[1] || '';

      await service.recordStreamProgress(sessionId, 1024000, 45);

      expect(mockRedis.hset).toHaveBeenCalledWith(
        expect.stringContaining('stream:session'),
        expect.objectContaining({
          bytesStreamed: '1024000',
          currentPosition: '45',
        })
      );
    });

    it('records play when position exceeds 30 seconds', async () => {
      const mockAudioFiles = [
        {
          quality: 'STANDARD_256',
          format: 'aac',
          bitrate: 256000,
          sampleRate: 44100,
          size: 5000000,
          key: 'tracks/track-1/standard.m4a',
          duration: 180,
        },
      ];

      (mockRedis.get as any).mockResolvedValueOnce(JSON.stringify(mockAudioFiles));

      const { url } = await service.getStreamUrl('track-1', 'user-1', 'STANDARD_256', 'PREMIUM');
      const sessionMatch = url.match(/session=([^&]+)/);
      const sessionId = sessionMatch?.[1] || '';

      await service.recordStreamProgress(sessionId, 2048000, 35);

      expect(mockRedis.lpush).toHaveBeenCalledWith(
        'analytics:plays',
        expect.any(String)
      );
    });
  });

  describe('endStreamSession', () => {
    it('removes session and records final stats', async () => {
      const mockAudioFiles = [
        {
          quality: 'STANDARD_256',
          format: 'aac',
          bitrate: 256000,
          sampleRate: 44100,
          size: 5000000,
          key: 'tracks/track-1/standard.m4a',
          duration: 180,
        },
      ];

      (mockRedis.get as any).mockResolvedValueOnce(JSON.stringify(mockAudioFiles));

      const { url } = await service.getStreamUrl('track-1', 'user-1', 'STANDARD_256', 'PREMIUM');
      const sessionMatch = url.match(/session=([^&]+)/);
      const sessionId = sessionMatch?.[1] || '';

      await service.endStreamSession(sessionId);

      expect(mockRedis.del).toHaveBeenCalledWith(
        expect.stringContaining('stream:session')
      );
      expect(mockRedis.lpush).toHaveBeenCalledWith(
        'analytics:streams',
        expect.any(String)
      );
    });
  });

  describe('getListenerCount', () => {
    it('returns count of active listeners', async () => {
      (mockRedis.keys as any).mockResolvedValueOnce([
        'stream:session:1',
        'stream:session:2',
        'stream:session:3',
      ]);

      (mockRedis.hgetall as any)
        .mockResolvedValueOnce({
          trackId: 'track-1',
          lastActivity: new Date().toISOString(),
        })
        .mockResolvedValueOnce({
          trackId: 'track-1',
          lastActivity: new Date().toISOString(),
        })
        .mockResolvedValueOnce({
          trackId: 'track-2',
          lastActivity: new Date().toISOString(),
        });

      const count = await service.getListenerCount('track-1');
      expect(count).toBe(2);
    });

    it('excludes stale sessions', async () => {
      const staleTime = new Date(Date.now() - 120000).toISOString(); // 2 minutes ago

      (mockRedis.keys as any).mockResolvedValueOnce(['stream:session:1']);
      (mockRedis.hgetall as any).mockResolvedValueOnce({
        trackId: 'track-1',
        lastActivity: staleTime,
      });

      const count = await service.getListenerCount('track-1');
      expect(count).toBe(0);
    });
  });

  describe('getAudioFiles', () => {
    it('returns filtered audio files based on max quality', async () => {
      const mockAudioFiles = [
        { quality: 'LOW_128', format: 'aac', bitrate: 128000 },
        { quality: 'STANDARD_256', format: 'aac', bitrate: 256000 },
        { quality: 'HIGH_320', format: 'mp3', bitrate: 320000 },
        { quality: 'LOSSLESS_FLAC', format: 'flac', bitrate: 0 },
      ];

      (mockRedis.get as any).mockResolvedValueOnce(JSON.stringify(mockAudioFiles));

      const files = await service.getAudioFiles('track-1', 'STANDARD_256');

      expect(files).toHaveLength(2);
      expect(files.map((f: any) => f.quality)).toEqual(['LOW_128', 'STANDARD_256']);
    });

    it('caches audio file data', async () => {
      const mockAudioFiles = [{ quality: 'LOW_128', format: 'aac', bitrate: 128000 }];

      (mockRedis.get as any).mockResolvedValueOnce(JSON.stringify(mockAudioFiles));

      await service.getAudioFiles('track-1', 'LOW_128');
      await service.getAudioFiles('track-1', 'LOW_128');

      // Should use cached data
      expect(mockRedis.get).toHaveBeenCalledTimes(2);
    });
  });

  describe('Quality Tier Restrictions', () => {
    const testCases: Array<{
      tier: string;
      maxQuality: AudioQuality;
    }> = [
      { tier: 'FREE', maxQuality: 'LOW_128' },
      { tier: 'PREMIUM', maxQuality: 'HIGH_320' },
      { tier: 'ARTIST_PRO', maxQuality: 'LOSSLESS_FLAC' },
      { tier: 'LABEL_ENTERPRISE', maxQuality: 'HI_RES_24BIT' },
    ];

    testCases.forEach(({ tier, maxQuality }) => {
      it(`limits ${tier} tier to ${maxQuality}`, async () => {
        const mockAudioFiles = [
          { quality: 'LOW_128', format: 'aac', bitrate: 128000, key: 'tracks/1/low.m4a', sampleRate: 44100, size: 3000000, duration: 180 },
          { quality: 'STANDARD_256', format: 'aac', bitrate: 256000, key: 'tracks/1/std.m4a', sampleRate: 44100, size: 5000000, duration: 180 },
          { quality: 'HIGH_320', format: 'mp3', bitrate: 320000, key: 'tracks/1/high.mp3', sampleRate: 44100, size: 7000000, duration: 180 },
          { quality: 'LOSSLESS_FLAC', format: 'flac', bitrate: 0, key: 'tracks/1/lossless.flac', sampleRate: 44100, size: 30000000, duration: 180 },
          { quality: 'HI_RES_24BIT', format: 'flac', bitrate: 0, key: 'tracks/1/hires.flac', sampleRate: 96000, bitDepth: 24, size: 60000000, duration: 180 },
        ];

        (mockRedis.get as any).mockResolvedValueOnce(JSON.stringify(mockAudioFiles));

        const result = await service.getStreamUrl('track-1', 'user-1', 'HI_RES_24BIT', tier);

        expect(result.quality).toBe(maxQuality);
      });
    });
  });
});
