// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Search Service
 *
 * Elasticsearch-powered search for songs, artists, and playlists.
 * Provides full-text search, autocomplete, and fuzzy matching.
 */

import express from 'express';
import { Client } from '@elastic/elasticsearch';
import { createLogger } from './logger';
import { config } from './config';
import { SearchController } from './controller';
import { IndexManager } from './indexer';
import { createRedisSubscriber } from './sync';

const logger = createLogger('search-service');

async function main() {
  const app = express();
  app.use(express.json());

  // Initialize Elasticsearch client
  const elastic = new Client({
    node: config.elasticsearch.node,
    auth: config.elasticsearch.auth,
  });

  // Verify connection
  try {
    const info = await elastic.info();
    logger.info({ version: info.version.number }, 'Connected to Elasticsearch');
  } catch (error) {
    logger.error({ error }, 'Failed to connect to Elasticsearch');
    process.exit(1);
  }

  // Initialize index manager and controller
  const indexManager = new IndexManager(elastic);
  const controller = new SearchController(elastic);

  // Ensure indices exist
  await indexManager.ensureIndices();

  // Health check
  app.get('/health', async (req, res) => {
    try {
      await elastic.ping();
      res.json({ status: 'healthy', service: 'search' });
    } catch {
      res.status(503).json({ status: 'unhealthy', service: 'search' });
    }
  });

  // Search API
  app.get('/api/search', controller.search.bind(controller));
  app.get('/api/search/autocomplete', controller.autocomplete.bind(controller));
  app.get('/api/search/similar/:songId', controller.findSimilar.bind(controller));

  // Index management (internal)
  app.post('/api/index/song', controller.indexSong.bind(controller));
  app.post('/api/index/artist', controller.indexArtist.bind(controller));
  app.post('/api/index/playlist', controller.indexPlaylist.bind(controller));
  app.delete('/api/index/:type/:id', controller.removeFromIndex.bind(controller));

  // Stats
  app.get('/api/stats', async (req, res) => {
    const stats = await indexManager.getStats();
    res.json(stats);
  });

  // Start Redis subscriber for real-time index updates
  const subscriber = await createRedisSubscriber(indexManager);

  // Start server
  app.listen(config.port, () => {
    logger.info(`Search service started on port ${config.port}`);
  });

  // Graceful shutdown
  const shutdown = async () => {
    logger.info('Shutting down...');
    await subscriber.disconnect();
    await elastic.close();
    process.exit(0);
  };

  process.on('SIGTERM', shutdown);
  process.on('SIGINT', shutdown);
}

main().catch((err) => {
  logger.error({ err }, 'Failed to start search service');
  process.exit(1);
});
