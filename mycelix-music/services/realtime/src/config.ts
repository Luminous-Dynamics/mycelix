// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Real-time Service Configuration
 */

import { z } from 'zod';

const configSchema = z.object({
  port: z.number().default(3005),
  redis: z.object({
    host: z.string().default('localhost'),
    port: z.number().default(6379),
    password: z.string().optional(),
  }),
  jwt: z.object({
    secret: z.string(),
  }),
  heartbeat: z.object({
    interval: z.number().default(30000),
    timeout: z.number().default(60000),
  }),
});

type Config = z.infer<typeof configSchema>;

function loadConfig(): Config {
  return configSchema.parse({
    port: parseInt(process.env.PORT || '3005', 10),
    redis: {
      host: process.env.REDIS_HOST || 'localhost',
      port: parseInt(process.env.REDIS_PORT || '6379', 10),
      password: process.env.REDIS_PASSWORD,
    },
    jwt: {
      secret: process.env.JWT_SECRET || 'dev-secret',
    },
  });
}

export const config = loadConfig();
export type { Config };
