// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Bull queue for transcode jobs
 */

import Bull from 'bull';
import { config } from './config';

export interface TranscodeJob {
  songId: string;
  sourceUrl: string;
  outputPath: string;
  profiles: string[];
  metadata?: {
    title?: string;
    artist?: string;
    album?: string;
    genre?: string;
  };
  callbackUrl?: string;
}

export interface TranscodeResult {
  songId: string;
  manifests: {
    master: string;
    profiles: Record<string, string>;
  };
  segments: string[];
  duration: number;
  waveform?: number[];
}

export function createTranscodeQueue() {
  const queue = new Bull<TranscodeJob>('transcode', {
    redis: {
      host: config.redis.host,
      port: config.redis.port,
      password: config.redis.password,
    },
    defaultJobOptions: {
      attempts: 3,
      backoff: {
        type: 'exponential',
        delay: 5000,
      },
      removeOnComplete: 100,
      removeOnFail: 50,
    },
  });

  queue.on('error', (error) => {
    console.error('Queue error:', error);
  });

  return queue;
}

export type TranscodeQueue = ReturnType<typeof createTranscodeQueue>;
