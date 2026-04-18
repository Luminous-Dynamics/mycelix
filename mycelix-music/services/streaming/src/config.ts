// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Configuration for the streaming service
 */

import { z } from 'zod';

const configSchema = z.object({
  port: z.number().default(3003),
  redis: z.object({
    host: z.string().default('localhost'),
    port: z.number().default(6379),
    password: z.string().optional(),
  }),
  s3: z.object({
    bucket: z.string(),
    region: z.string().default('us-east-1'),
    accessKeyId: z.string().optional(),
    secretAccessKey: z.string().optional(),
    endpoint: z.string().optional(),
  }),
  ffmpeg: z.object({
    path: z.string().default('ffmpeg'),
    threads: z.number().default(2),
  }),
  transcode: z.object({
    tempDir: z.string().default('/tmp/mycelix-transcode'),
    concurrency: z.number().default(2),
    // Bitrate profiles for adaptive streaming
    profiles: z.array(z.object({
      name: z.string(),
      bitrate: z.string(),
      codec: z.string(),
      sampleRate: z.number(),
    })).default([
      { name: 'low', bitrate: '64k', codec: 'aac', sampleRate: 22050 },
      { name: 'medium', bitrate: '128k', codec: 'aac', sampleRate: 44100 },
      { name: 'high', bitrate: '256k', codec: 'aac', sampleRate: 44100 },
      { name: 'lossless', bitrate: '320k', codec: 'aac', sampleRate: 48000 },
    ]),
    segmentDuration: z.number().default(6), // HLS segment duration in seconds
  }),
  cdn: z.object({
    baseUrl: z.string().default(''),
  }),
});

type Config = z.infer<typeof configSchema>;

function loadConfig(): Config {
  return configSchema.parse({
    port: parseInt(process.env.PORT || '3003', 10),
    redis: {
      host: process.env.REDIS_HOST || 'localhost',
      port: parseInt(process.env.REDIS_PORT || '6379', 10),
      password: process.env.REDIS_PASSWORD,
    },
    s3: {
      bucket: process.env.S3_BUCKET || 'mycelix-audio',
      region: process.env.S3_REGION || 'us-east-1',
      accessKeyId: process.env.AWS_ACCESS_KEY_ID,
      secretAccessKey: process.env.AWS_SECRET_ACCESS_KEY,
      endpoint: process.env.S3_ENDPOINT,
    },
    ffmpeg: {
      path: process.env.FFMPEG_PATH || 'ffmpeg',
      threads: parseInt(process.env.FFMPEG_THREADS || '2', 10),
    },
    transcode: {
      tempDir: process.env.TRANSCODE_TEMP_DIR || '/tmp/mycelix-transcode',
      concurrency: parseInt(process.env.TRANSCODE_CONCURRENCY || '2', 10),
    },
    cdn: {
      baseUrl: process.env.CDN_BASE_URL || '',
    },
  });
}

export const config = loadConfig();
export type { Config };
