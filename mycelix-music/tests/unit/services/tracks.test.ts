// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Track Service Unit Tests
 */

import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';

// Mock dependencies
const mockPrisma = {
  track: {
    findUnique: vi.fn(),
    findMany: vi.fn(),
    create: vi.fn(),
    update: vi.fn(),
    delete: vi.fn(),
    count: vi.fn(),
  },
  $transaction: vi.fn((fn) => fn(mockPrisma)),
};

const mockRedis = {
  get: vi.fn(),
  set: vi.fn(),
  del: vi.fn(),
  expire: vi.fn(),
};

const mockS3 = {
  upload: vi.fn(),
  getSignedUrl: vi.fn(),
  deleteObject: vi.fn(),
};

// Track Service (simplified for testing)
class TrackService {
  constructor(
    private prisma: typeof mockPrisma,
    private redis: typeof mockRedis,
    private s3: typeof mockS3
  ) {}

  async getTrack(id: string) {
    // Check cache first
    const cached = await this.redis.get(`track:${id}`);
    if (cached) return JSON.parse(cached);

    const track = await this.prisma.track.findUnique({
      where: { id },
      include: { artist: true, album: true },
    });

    if (track) {
      await this.redis.set(`track:${id}`, JSON.stringify(track));
      await this.redis.expire(`track:${id}`, 3600);
    }

    return track;
  }

  async searchTracks(query: string, limit = 20, offset = 0) {
    return this.prisma.track.findMany({
      where: {
        OR: [
          { title: { contains: query, mode: 'insensitive' } },
          { artist: { name: { contains: query, mode: 'insensitive' } } },
        ],
      },
      take: limit,
      skip: offset,
      include: { artist: true },
    });
  }

  async createTrack(data: CreateTrackInput, audioFile: Buffer) {
    // Upload audio file
    const audioUrl = await this.uploadAudio(audioFile, data.title);

    // Create track in database
    const track = await this.prisma.track.create({
      data: {
        ...data,
        audioUrl,
      },
    });

    return track;
  }

  async updateTrack(id: string, data: UpdateTrackInput) {
    const track = await this.prisma.track.update({
      where: { id },
      data,
    });

    // Invalidate cache
    await this.redis.del(`track:${id}`);

    return track;
  }

  async deleteTrack(id: string) {
    const track = await this.prisma.track.findUnique({ where: { id } });
    if (!track) throw new Error('Track not found');

    // Delete from S3
    await this.s3.deleteObject({ Key: this.extractS3Key(track.audioUrl) });

    // Delete from database
    await this.prisma.track.delete({ where: { id } });

    // Invalidate cache
    await this.redis.del(`track:${id}`);

    return { success: true };
  }

  async incrementPlayCount(id: string) {
    const track = await this.prisma.track.update({
      where: { id },
      data: { playCount: { increment: 1 } },
    });

    // Invalidate cache
    await this.redis.del(`track:${id}`);

    return track;
  }

  async getPopularTracks(limit = 50) {
    return this.prisma.track.findMany({
      orderBy: { playCount: 'desc' },
      take: limit,
      include: { artist: true },
    });
  }

  async getRecommendations(trackId: string, limit = 10) {
    const track = await this.getTrack(trackId);
    if (!track) throw new Error('Track not found');

    // Simple recommendation based on genre and mood
    return this.prisma.track.findMany({
      where: {
        id: { not: trackId },
        OR: [
          { genre: track.genre },
          { mood: { hasSome: track.mood } },
        ],
      },
      take: limit,
      orderBy: { playCount: 'desc' },
    });
  }

  private async uploadAudio(file: Buffer, title: string): Promise<string> {
    const key = `tracks/${Date.now()}-${title.replace(/\s+/g, '-')}.mp3`;
    const result = await this.s3.upload({
      Key: key,
      Body: file,
      ContentType: 'audio/mpeg',
    });
    return result.Location;
  }

  private extractS3Key(url: string): string {
    return url.split('/').slice(-2).join('/');
  }
}

interface CreateTrackInput {
  title: string;
  artistId: string;
  albumId?: string;
  duration: number;
  genre: string;
  mood?: string[];
  bpm?: number;
  key?: string;
}

interface UpdateTrackInput {
  title?: string;
  genre?: string;
  mood?: string[];
  bpm?: number;
  key?: string;
}

// ============================================================
// TESTS
// ============================================================

describe('TrackService', () => {
  let trackService: TrackService;

  beforeEach(() => {
    vi.clearAllMocks();
    trackService = new TrackService(mockPrisma, mockRedis, mockS3);
  });

  afterEach(() => {
    vi.resetAllMocks();
  });

  describe('getTrack', () => {
    const mockTrack = {
      id: 'track_1',
      title: 'Test Track',
      artistId: 'artist_1',
      duration: 180,
      audioUrl: 'https://s3.example.com/track.mp3',
      genre: 'electronic',
      playCount: 1000,
      artist: { id: 'artist_1', name: 'Test Artist' },
    };

    it('should return track from cache if available', async () => {
      mockRedis.get.mockResolvedValue(JSON.stringify(mockTrack));

      const result = await trackService.getTrack('track_1');

      expect(result).toEqual(mockTrack);
      expect(mockRedis.get).toHaveBeenCalledWith('track:track_1');
      expect(mockPrisma.track.findUnique).not.toHaveBeenCalled();
    });

    it('should fetch from database if not in cache', async () => {
      mockRedis.get.mockResolvedValue(null);
      mockPrisma.track.findUnique.mockResolvedValue(mockTrack);

      const result = await trackService.getTrack('track_1');

      expect(result).toEqual(mockTrack);
      expect(mockPrisma.track.findUnique).toHaveBeenCalledWith({
        where: { id: 'track_1' },
        include: { artist: true, album: true },
      });
      expect(mockRedis.set).toHaveBeenCalledWith(
        'track:track_1',
        JSON.stringify(mockTrack)
      );
      expect(mockRedis.expire).toHaveBeenCalledWith('track:track_1', 3600);
    });

    it('should return null for non-existent track', async () => {
      mockRedis.get.mockResolvedValue(null);
      mockPrisma.track.findUnique.mockResolvedValue(null);

      const result = await trackService.getTrack('nonexistent');

      expect(result).toBeNull();
      expect(mockRedis.set).not.toHaveBeenCalled();
    });
  });

  describe('searchTracks', () => {
    it('should search tracks by title and artist name', async () => {
      const mockTracks = [
        { id: 'track_1', title: 'Love Song', artist: { name: 'Artist 1' } },
        { id: 'track_2', title: 'Lovely Day', artist: { name: 'Artist 2' } },
      ];
      mockPrisma.track.findMany.mockResolvedValue(mockTracks);

      const result = await trackService.searchTracks('love', 20, 0);

      expect(result).toEqual(mockTracks);
      expect(mockPrisma.track.findMany).toHaveBeenCalledWith({
        where: {
          OR: [
            { title: { contains: 'love', mode: 'insensitive' } },
            { artist: { name: { contains: 'love', mode: 'insensitive' } } },
          ],
        },
        take: 20,
        skip: 0,
        include: { artist: true },
      });
    });

    it('should respect pagination parameters', async () => {
      mockPrisma.track.findMany.mockResolvedValue([]);

      await trackService.searchTracks('test', 10, 50);

      expect(mockPrisma.track.findMany).toHaveBeenCalledWith(
        expect.objectContaining({
          take: 10,
          skip: 50,
        })
      );
    });
  });

  describe('createTrack', () => {
    const createInput: CreateTrackInput = {
      title: 'New Track',
      artistId: 'artist_1',
      duration: 240,
      genre: 'pop',
      mood: ['happy', 'energetic'],
      bpm: 128,
      key: 'C major',
    };

    it('should upload audio and create track', async () => {
      const audioFile = Buffer.from('fake audio data');
      const uploadResult = { Location: 'https://s3.example.com/new-track.mp3' };
      mockS3.upload.mockResolvedValue(uploadResult);

      const createdTrack = { id: 'track_new', ...createInput, audioUrl: uploadResult.Location };
      mockPrisma.track.create.mockResolvedValue(createdTrack);

      const result = await trackService.createTrack(createInput, audioFile);

      expect(mockS3.upload).toHaveBeenCalledWith(
        expect.objectContaining({
          Body: audioFile,
          ContentType: 'audio/mpeg',
        })
      );
      expect(mockPrisma.track.create).toHaveBeenCalledWith({
        data: {
          ...createInput,
          audioUrl: uploadResult.Location,
        },
      });
      expect(result.audioUrl).toBe(uploadResult.Location);
    });
  });

  describe('updateTrack', () => {
    it('should update track and invalidate cache', async () => {
      const updateData = { title: 'Updated Title', genre: 'rock' };
      const updatedTrack = { id: 'track_1', ...updateData };
      mockPrisma.track.update.mockResolvedValue(updatedTrack);

      const result = await trackService.updateTrack('track_1', updateData);

      expect(mockPrisma.track.update).toHaveBeenCalledWith({
        where: { id: 'track_1' },
        data: updateData,
      });
      expect(mockRedis.del).toHaveBeenCalledWith('track:track_1');
      expect(result).toEqual(updatedTrack);
    });
  });

  describe('deleteTrack', () => {
    it('should delete track from S3, database, and cache', async () => {
      const mockTrack = {
        id: 'track_1',
        audioUrl: 'https://s3.example.com/tracks/track.mp3',
      };
      mockPrisma.track.findUnique.mockResolvedValue(mockTrack);
      mockS3.deleteObject.mockResolvedValue({});
      mockPrisma.track.delete.mockResolvedValue(mockTrack);

      const result = await trackService.deleteTrack('track_1');

      expect(mockS3.deleteObject).toHaveBeenCalledWith({
        Key: 'tracks/track.mp3',
      });
      expect(mockPrisma.track.delete).toHaveBeenCalledWith({
        where: { id: 'track_1' },
      });
      expect(mockRedis.del).toHaveBeenCalledWith('track:track_1');
      expect(result).toEqual({ success: true });
    });

    it('should throw error if track not found', async () => {
      mockPrisma.track.findUnique.mockResolvedValue(null);

      await expect(trackService.deleteTrack('nonexistent')).rejects.toThrow(
        'Track not found'
      );
    });
  });

  describe('incrementPlayCount', () => {
    it('should increment play count and invalidate cache', async () => {
      const updatedTrack = { id: 'track_1', playCount: 1001 };
      mockPrisma.track.update.mockResolvedValue(updatedTrack);

      const result = await trackService.incrementPlayCount('track_1');

      expect(mockPrisma.track.update).toHaveBeenCalledWith({
        where: { id: 'track_1' },
        data: { playCount: { increment: 1 } },
      });
      expect(mockRedis.del).toHaveBeenCalledWith('track:track_1');
      expect(result.playCount).toBe(1001);
    });
  });

  describe('getPopularTracks', () => {
    it('should return tracks ordered by play count', async () => {
      const popularTracks = [
        { id: 'track_1', playCount: 10000 },
        { id: 'track_2', playCount: 5000 },
      ];
      mockPrisma.track.findMany.mockResolvedValue(popularTracks);

      const result = await trackService.getPopularTracks(50);

      expect(mockPrisma.track.findMany).toHaveBeenCalledWith({
        orderBy: { playCount: 'desc' },
        take: 50,
        include: { artist: true },
      });
      expect(result).toEqual(popularTracks);
    });
  });

  describe('getRecommendations', () => {
    it('should return recommendations based on genre and mood', async () => {
      const sourceTrack = {
        id: 'track_1',
        genre: 'electronic',
        mood: ['energetic', 'uplifting'],
      };
      const recommendations = [
        { id: 'track_2', genre: 'electronic', mood: ['energetic'] },
        { id: 'track_3', genre: 'house', mood: ['uplifting'] },
      ];

      mockRedis.get.mockResolvedValue(null);
      mockPrisma.track.findUnique.mockResolvedValue(sourceTrack);
      mockPrisma.track.findMany.mockResolvedValue(recommendations);

      const result = await trackService.getRecommendations('track_1', 10);

      expect(mockPrisma.track.findMany).toHaveBeenCalledWith({
        where: {
          id: { not: 'track_1' },
          OR: [
            { genre: 'electronic' },
            { mood: { hasSome: ['energetic', 'uplifting'] } },
          ],
        },
        take: 10,
        orderBy: { playCount: 'desc' },
      });
      expect(result).toEqual(recommendations);
    });

    it('should throw error if source track not found', async () => {
      mockRedis.get.mockResolvedValue(null);
      mockPrisma.track.findUnique.mockResolvedValue(null);

      await expect(
        trackService.getRecommendations('nonexistent', 10)
      ).rejects.toThrow('Track not found');
    });
  });
});
