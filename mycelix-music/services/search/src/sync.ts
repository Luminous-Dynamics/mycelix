// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Real-time Index Sync
 *
 * Subscribes to Redis pub/sub for real-time index updates.
 */

import Redis from 'ioredis';
import { config } from './config';
import { createLogger } from './logger';
import { IndexManager, SongDocument, ArtistDocument, PlaylistDocument } from './indexer';

const logger = createLogger('sync');

interface IndexEvent {
  type: 'song' | 'artist' | 'playlist';
  action: 'index' | 'update' | 'delete';
  data: SongDocument | ArtistDocument | PlaylistDocument | { id: string };
}

export async function createRedisSubscriber(indexManager: IndexManager): Promise<Redis> {
  const subscriber = new Redis({
    host: config.redis.host,
    port: config.redis.port,
    password: config.redis.password,
    retryStrategy: (times) => Math.min(times * 50, 2000),
  });

  subscriber.on('connect', () => {
    logger.info('Connected to Redis for index sync');
  });

  subscriber.on('error', (error) => {
    logger.error({ error }, 'Redis subscriber error');
  });

  // Subscribe to index update channel
  await subscriber.subscribe('mycelix:index:updates');

  subscriber.on('message', async (channel, message) => {
    if (channel !== 'mycelix:index:updates') return;

    try {
      const event: IndexEvent = JSON.parse(message);
      logger.debug({ event }, 'Received index event');

      switch (event.action) {
        case 'index':
        case 'update':
          if (event.type === 'song') {
            await indexManager.indexSong(event.data as SongDocument);
          } else if (event.type === 'artist') {
            await indexManager.indexArtist(event.data as ArtistDocument);
          } else if (event.type === 'playlist') {
            await indexManager.indexPlaylist(event.data as PlaylistDocument);
          }
          break;

        case 'delete':
          const id = (event.data as { id: string }).id;
          if (event.type === 'song') {
            await indexManager.removeSong(id);
          } else if (event.type === 'artist') {
            await indexManager.removeArtist(id);
          } else if (event.type === 'playlist') {
            await indexManager.removePlaylist(id);
          }
          break;
      }
    } catch (error) {
      logger.error({ error, message }, 'Failed to process index event');
    }
  });

  return subscriber;
}

/**
 * Publish an index update event
 * (Used by API service when data changes)
 */
export async function publishIndexUpdate(
  redis: Redis,
  event: IndexEvent
): Promise<void> {
  await redis.publish('mycelix:index:updates', JSON.stringify(event));
}
