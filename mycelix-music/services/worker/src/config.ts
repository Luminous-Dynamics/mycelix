// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Worker Service Configuration
 */

import { z } from 'zod';

const configSchema = z.object({
  redis: z.object({
    host: z.string().default('localhost'),
    port: z.number().default(6379),
    password: z.string().optional(),
  }),
  workers: z.object({
    email: z.object({
      concurrency: z.number().default(5),
      rateLimit: z.number().default(10), // per second
    }),
    analytics: z.object({
      concurrency: z.number().default(3),
    }),
    notification: z.object({
      concurrency: z.number().default(10),
    }),
    indexSync: z.object({
      concurrency: z.number().default(5),
    }),
  }),
  email: z.object({
    host: z.string().default('smtp.example.com'),
    port: z.number().default(587),
    user: z.string().optional(),
    password: z.string().optional(),
    from: z.string().default('noreply@mycelix.io'),
  }),
});

type Config = z.infer<typeof configSchema>;

function loadConfig(): Config {
  return configSchema.parse({
    redis: {
      host: process.env.REDIS_HOST || 'localhost',
      port: parseInt(process.env.REDIS_PORT || '6379', 10),
      password: process.env.REDIS_PASSWORD,
    },
    workers: {
      email: {
        concurrency: parseInt(process.env.EMAIL_WORKER_CONCURRENCY || '5', 10),
        rateLimit: parseInt(process.env.EMAIL_RATE_LIMIT || '10', 10),
      },
      analytics: {
        concurrency: parseInt(process.env.ANALYTICS_WORKER_CONCURRENCY || '3', 10),
      },
      notification: {
        concurrency: parseInt(process.env.NOTIFICATION_WORKER_CONCURRENCY || '10', 10),
      },
      indexSync: {
        concurrency: parseInt(process.env.INDEX_WORKER_CONCURRENCY || '5', 10),
      },
    },
    email: {
      host: process.env.SMTP_HOST || 'smtp.example.com',
      port: parseInt(process.env.SMTP_PORT || '587', 10),
      user: process.env.SMTP_USER,
      password: process.env.SMTP_PASSWORD,
      from: process.env.EMAIL_FROM || 'noreply@mycelix.io',
    },
  });
}

export const config = loadConfig();
