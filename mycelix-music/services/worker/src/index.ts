// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Worker Service
 *
 * Background job processor for async tasks like emails, analytics, cleanup.
 */

import { Worker, Queue, Job } from 'bullmq';
import Redis from 'ioredis';
import { config } from './config';
import { createLogger } from './logger';
import { emailProcessor } from './processors/email';
import { analyticsProcessor } from './processors/analytics';
import { cleanupProcessor } from './processors/cleanup';
import { notificationProcessor } from './processors/notification';
import { indexProcessor } from './processors/index-sync';

const logger = createLogger('worker-service');

const connection = new Redis({
  host: config.redis.host,
  port: config.redis.port,
  password: config.redis.password,
  maxRetriesPerRequest: null,
});

// Queue definitions
export const queues = {
  email: new Queue('email', { connection }),
  analytics: new Queue('analytics', { connection }),
  cleanup: new Queue('cleanup', { connection }),
  notification: new Queue('notification', { connection }),
  indexSync: new Queue('index-sync', { connection }),
};

// Worker definitions
const workers: Worker[] = [];

async function startWorkers() {
  // Email worker
  const emailWorker = new Worker(
    'email',
    async (job: Job) => emailProcessor(job),
    {
      connection,
      concurrency: config.workers.email.concurrency,
      limiter: {
        max: config.workers.email.rateLimit,
        duration: 1000,
      },
    }
  );
  workers.push(emailWorker);

  // Analytics worker
  const analyticsWorker = new Worker(
    'analytics',
    async (job: Job) => analyticsProcessor(job),
    {
      connection,
      concurrency: config.workers.analytics.concurrency,
    }
  );
  workers.push(analyticsWorker);

  // Cleanup worker
  const cleanupWorker = new Worker(
    'cleanup',
    async (job: Job) => cleanupProcessor(job),
    {
      connection,
      concurrency: 1, // Run cleanup jobs sequentially
    }
  );
  workers.push(cleanupWorker);

  // Notification worker
  const notificationWorker = new Worker(
    'notification',
    async (job: Job) => notificationProcessor(job),
    {
      connection,
      concurrency: config.workers.notification.concurrency,
    }
  );
  workers.push(notificationWorker);

  // Index sync worker
  const indexWorker = new Worker(
    'index-sync',
    async (job: Job) => indexProcessor(job),
    {
      connection,
      concurrency: config.workers.indexSync.concurrency,
    }
  );
  workers.push(indexWorker);

  // Set up event handlers for all workers
  for (const worker of workers) {
    worker.on('completed', (job) => {
      logger.info({ jobId: job.id, queue: job.queueName }, 'Job completed');
    });

    worker.on('failed', (job, error) => {
      logger.error(
        { jobId: job?.id, queue: job?.queueName, error: error.message },
        'Job failed'
      );
    });

    worker.on('error', (error) => {
      logger.error({ error: error.message }, 'Worker error');
    });
  }

  logger.info('All workers started');
}

// Schedule recurring jobs
async function scheduleRecurringJobs() {
  // Daily analytics aggregation
  await queues.analytics.add(
    'daily-aggregation',
    {},
    {
      repeat: { pattern: '0 2 * * *' }, // 2 AM daily
      removeOnComplete: true,
    }
  );

  // Hourly cleanup of expired sessions
  await queues.cleanup.add(
    'expire-sessions',
    {},
    {
      repeat: { pattern: '0 * * * *' }, // Every hour
      removeOnComplete: true,
    }
  );

  // Weekly cleanup of old data
  await queues.cleanup.add(
    'weekly-cleanup',
    {},
    {
      repeat: { pattern: '0 3 * * 0' }, // 3 AM on Sundays
      removeOnComplete: true,
    }
  );

  logger.info('Recurring jobs scheduled');
}

async function main() {
  logger.info('Starting worker service...');

  await startWorkers();
  await scheduleRecurringJobs();

  logger.info('Worker service running');
}

// Graceful shutdown
async function shutdown() {
  logger.info('Shutting down workers...');

  // Close all workers
  await Promise.all(workers.map(w => w.close()));

  // Close queues
  await Promise.all(Object.values(queues).map(q => q.close()));

  // Close Redis
  connection.disconnect();

  logger.info('Shutdown complete');
  process.exit(0);
}

process.on('SIGTERM', shutdown);
process.on('SIGINT', shutdown);

main().catch((error) => {
  logger.error({ error }, 'Failed to start worker service');
  process.exit(1);
});
