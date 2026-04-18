// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Song Service Tests
 */

import { SongService } from '../../services/song.service';
import { SongRepository } from '../../repositories/song.repository';
import { CacheService, MemoryCache } from '../../services/cache.service';
import { AppError } from '../../middleware/error-handler';
import { FixedIds, SongFixtures } from '../../db/seeds/fixtures';

// Create mock repository
const createMockRepository = () => ({
  findById: jest.fn(),
  findByIds: jest.fn(),
  findByArtist: jest.fn(),
  findByGenre: jest.fn(),
  findOne: jest.fn(),
  search: jest.fn(),
  getTopByPlays: jest.fn(),
  getRecent: jest.fn(),
  create: jest.fn(),
  update: jest.fn(),
  delete: jest.fn(),
  recordPlay: jest.fn(),
  setClaimStreamId: jest.fn(),
  getArtistStats: jest.fn(),
  getGenreStats: jest.fn(),
});

describe('SongService', () => {
  let service: SongService;
  let mockRepo: ReturnType<typeof createMockRepository>;
  let cache: CacheService;

  beforeEach(() => {
    mockRepo = createMockRepository();
    cache = new CacheService(new MemoryCache());
    service = new SongService(
      mockRepo as unknown as SongRepository,
      cache
    );
    jest.clearAllMocks();
  });

  describe('getSongById', () => {
    it('should return song from cache if available', async () => {
      const song = SongFixtures.validSong;
      await cache.set(`song:${song.id}`, song);

      const result = await service.getSongById(song.id);

      expect(result).toEqual(song);
      expect(mockRepo.findById).not.toHaveBeenCalled();
    });

    it('should fetch from repository and cache on miss', async () => {
      const song = SongFixtures.validSong;
      mockRepo.findById.mockResolvedValue(song);

      const result = await service.getSongById(song.id);

      expect(result).toEqual(song);
      expect(mockRepo.findById).toHaveBeenCalledWith(song.id);

      // Verify it was cached
      const cached = await cache.get(`song:${song.id}`);
      expect(cached).toEqual(song);
    });

    it('should throw NotFound error when song does not exist', async () => {
      mockRepo.findById.mockResolvedValue(null);

      await expect(service.getSongById('non-existent'))
        .rejects
        .toThrow(AppError);
    });
  });

  describe('getSongsByIds', () => {
    it('should return empty array for empty input', async () => {
      const result = await service.getSongsByIds([]);

      expect(result).toEqual([]);
      expect(mockRepo.findByIds).not.toHaveBeenCalled();
    });

    it('should use cache for cached songs', async () => {
      const song1 = SongFixtures.validSong;
      const song2 = SongFixtures.subscriptionSong;

      // Cache one song
      await cache.set(`song:${song1.id}`, song1);

      // Mock repository for uncached song
      mockRepo.findByIds.mockResolvedValue([song2]);

      const result = await service.getSongsByIds([song1.id, song2.id]);

      expect(result).toHaveLength(2);
      expect(mockRepo.findByIds).toHaveBeenCalledWith([song2.id]);
    });

    it('should maintain original order', async () => {
      const songs = [SongFixtures.validSong, SongFixtures.subscriptionSong];
      mockRepo.findByIds.mockResolvedValue(songs);

      const ids = songs.map(s => s.id);
      const result = await service.getSongsByIds(ids);

      expect(result.map(s => s.id)).toEqual(ids);
    });
  });

  describe('searchSongs', () => {
    it('should pass parameters to repository', async () => {
      const params = {
        q: 'test',
        genre: 'electronic',
        limit: 20,
        offset: 0,
      };

      mockRepo.search.mockResolvedValue({
        data: [],
        total: 0,
        limit: 20,
        offset: 0,
        hasMore: false,
      });

      await service.searchSongs(params);

      expect(mockRepo.search).toHaveBeenCalledWith(
        expect.objectContaining({
          query: 'test',
          genre: 'electronic',
        }),
        20,
        0
      );
    });

    it('should enforce max page size', async () => {
      mockRepo.search.mockResolvedValue({
        data: [],
        total: 0,
        limit: 100,
        offset: 0,
        hasMore: false,
      });

      await service.searchSongs({ limit: 500 } as any);

      expect(mockRepo.search).toHaveBeenCalledWith(
        expect.anything(),
        100, // Max enforced
        0
      );
    });
  });

  describe('getTopSongs', () => {
    it('should return cached top songs', async () => {
      const songs = [SongFixtures.popularSong];
      await cache.set('top:songs:10', songs);

      const result = await service.getTopSongs(10);

      expect(result).toEqual(songs);
      expect(mockRepo.getTopByPlays).not.toHaveBeenCalled();
    });

    it('should fetch and cache top songs on miss', async () => {
      const songs = [SongFixtures.popularSong];
      mockRepo.getTopByPlays.mockResolvedValue(songs);

      const result = await service.getTopSongs(10);

      expect(result).toEqual(songs);
      expect(mockRepo.getTopByPlays).toHaveBeenCalledWith(10);
    });
  });

  describe('createSong', () => {
    it('should create song when IPFS hash is unique', async () => {
      const input = {
        title: 'New Song',
        artist: 'Artist',
        artist_address: FixedIds.artists.artist1 as `0x${string}`,
        genre: 'electronic',
        ipfs_hash: 'QmUniqueHash123',
        payment_model: 'per_play' as const,
      };

      mockRepo.findOne.mockResolvedValue(null); // No existing song
      mockRepo.create.mockResolvedValue({ ...input, id: 'new-id', plays: 0, earnings: '0' });

      const result = await service.createSong(input);

      expect(result.title).toBe(input.title);
      expect(mockRepo.create).toHaveBeenCalled();
    });

    it('should throw conflict error for duplicate IPFS hash', async () => {
      const input = {
        title: 'Duplicate Song',
        artist: 'Artist',
        artist_address: FixedIds.artists.artist1 as `0x${string}`,
        genre: 'electronic',
        ipfs_hash: SongFixtures.validSong.ipfs_hash,
        payment_model: 'per_play' as const,
      };

      mockRepo.findOne.mockResolvedValue(SongFixtures.validSong);

      await expect(service.createSong(input))
        .rejects
        .toThrow('A song with this IPFS hash already exists');
    });
  });

  describe('updateSong', () => {
    it('should update existing song', async () => {
      const updates = { title: 'Updated Title' };
      const updated = { ...SongFixtures.validSong, ...updates };

      mockRepo.findById.mockResolvedValue(SongFixtures.validSong);
      mockRepo.update.mockResolvedValue(updated);

      const result = await service.updateSong(SongFixtures.validSong.id, updates);

      expect(result.title).toBe('Updated Title');
    });

    it('should throw NotFound for non-existent song', async () => {
      mockRepo.findById.mockResolvedValue(null);

      await expect(service.updateSong('non-existent', { title: 'Test' }))
        .rejects
        .toThrow(AppError);
    });

    it('should invalidate cache after update', async () => {
      const song = SongFixtures.validSong;
      await cache.set(`song:${song.id}`, song);

      mockRepo.findById.mockResolvedValue(song);
      mockRepo.update.mockResolvedValue({ ...song, title: 'Updated' });

      await service.updateSong(song.id, { title: 'Updated' });

      const cached = await cache.get(`song:${song.id}`);
      expect(cached).toBeNull();
    });
  });

  describe('deleteSong', () => {
    it('should delete song with no plays', async () => {
      const song = SongFixtures.newSong; // has 0 plays
      mockRepo.findById.mockResolvedValue(song);
      mockRepo.delete.mockResolvedValue(true);

      await expect(service.deleteSong(song.id)).resolves.not.toThrow();
      expect(mockRepo.delete).toHaveBeenCalledWith(song.id);
    });

    it('should throw conflict error for song with plays', async () => {
      const song = SongFixtures.popularSong; // has plays
      mockRepo.findById.mockResolvedValue(song);

      await expect(service.deleteSong(song.id))
        .rejects
        .toThrow('Cannot delete a song that has been played');
    });

    it('should throw NotFound for non-existent song', async () => {
      mockRepo.findById.mockResolvedValue(null);

      await expect(service.deleteSong('non-existent'))
        .rejects
        .toThrow(AppError);
    });
  });

  describe('recordPlay', () => {
    it('should update play count and earnings', async () => {
      const updated = {
        ...SongFixtures.validSong,
        plays: 101,
        earnings: '0.101000',
      };

      mockRepo.recordPlay.mockResolvedValue(updated);

      const result = await service.recordPlay(SongFixtures.validSong.id, '0.001000');

      expect(result.plays).toBe(101);
      expect(mockRepo.recordPlay).toHaveBeenCalledWith(
        SongFixtures.validSong.id,
        '0.001000'
      );
    });

    it('should invalidate caches after play', async () => {
      const song = SongFixtures.validSong;
      await cache.set(`song:${song.id}`, song);

      mockRepo.recordPlay.mockResolvedValue({ ...song, plays: song.plays + 1 });

      await service.recordPlay(song.id, '0.001000');

      const cached = await cache.get(`song:${song.id}`);
      expect(cached).toBeNull();
    });
  });

  describe('getArtistStats', () => {
    it('should return cached stats', async () => {
      const stats = { total_songs: 5, total_plays: 100, total_earnings: '1.000000' };
      const address = FixedIds.artists.artist1;
      await cache.set(`artist:${address}:stats`, stats);

      const result = await service.getArtistStats(address);

      expect(result).toEqual(stats);
      expect(mockRepo.getArtistStats).not.toHaveBeenCalled();
    });

    it('should return default stats for new artist', async () => {
      mockRepo.getArtistStats.mockResolvedValue(null);

      const result = await service.getArtistStats(FixedIds.artists.artist3);

      expect(result).toEqual({
        total_songs: 0,
        total_plays: 0,
        total_earnings: '0',
      });
    });
  });
});
