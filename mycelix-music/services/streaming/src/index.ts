// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Adaptive Streaming Service
 *
 * Handles audio transcoding to HLS format for adaptive bitrate streaming.
 * Uses ffmpeg for transcoding and Bull for job queue management.
 */

import express from 'express';
import { createTranscodeQueue, TranscodeJob } from './queue';
import { TranscodeWorker } from './worker';
import { createLogger } from './logger';
import { StreamingController } from './controller';
import { config } from './config';

const logger = createLogger('streaming-service');

async function main() {
  const app = express();
  app.use(express.json());

  // Initialize queue and worker
  const queue = createTranscodeQueue();
  const worker = new TranscodeWorker(queue);
  const controller = new StreamingController(queue);

  // Health check
  app.get('/health', (req, res) => {
    res.json({ status: 'healthy', service: 'streaming' });
  });

  // API Routes
  app.post('/api/transcode', controller.startTranscode.bind(controller));
  app.get('/api/transcode/:jobId', controller.getJobStatus.bind(controller));
  app.get('/api/transcode/:jobId/cancel', controller.cancelJob.bind(controller));

  // Queue stats
  app.get('/api/stats', async (req, res) => {
    const [waiting, active, completed, failed] = await Promise.all([
      queue.getWaitingCount(),
      queue.getActiveCount(),
      queue.getCompletedCount(),
      queue.getFailedCount(),
    ]);
    res.json({ waiting, active, completed, failed });
  });

  // Start worker
  await worker.start();

  // Start server
  app.listen(config.port, () => {
    logger.info(`Streaming service started on port ${config.port}`);
  });

  // Graceful shutdown
  const shutdown = async () => {
    logger.info('Shutting down...');
    await worker.stop();
    await queue.close();
    process.exit(0);
  };

  process.on('SIGTERM', shutdown);
  process.on('SIGINT', shutdown);
}

main().catch((err) => {
  logger.error({ err }, 'Failed to start streaming service');
  process.exit(1);
});
