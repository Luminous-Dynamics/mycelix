// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Search Service Configuration
 */

import { z } from 'zod';

const configSchema = z.object({
  port: z.number().default(3004),
  elasticsearch: z.object({
    node: z.string().default('http://localhost:9200'),
    auth: z.object({
      username: z.string().optional(),
      password: z.string().optional(),
    }).optional(),
  }),
  redis: z.object({
    host: z.string().default('localhost'),
    port: z.number().default(6379),
    password: z.string().optional(),
  }),
  indices: z.object({
    songs: z.string().default('mycelix_songs'),
    artists: z.string().default('mycelix_artists'),
    playlists: z.string().default('mycelix_playlists'),
  }),
  search: z.object({
    maxResults: z.number().default(50),
    fuzziness: z.string().default('AUTO'),
    highlightFragmentSize: z.number().default(150),
  }),
});

type Config = z.infer<typeof configSchema>;

function loadConfig(): Config {
  return configSchema.parse({
    port: parseInt(process.env.PORT || '3004', 10),
    elasticsearch: {
      node: process.env.ELASTICSEARCH_NODE || 'http://localhost:9200',
      auth: process.env.ELASTICSEARCH_USERNAME ? {
        username: process.env.ELASTICSEARCH_USERNAME,
        password: process.env.ELASTICSEARCH_PASSWORD,
      } : undefined,
    },
    redis: {
      host: process.env.REDIS_HOST || 'localhost',
      port: parseInt(process.env.REDIS_PORT || '6379', 10),
      password: process.env.REDIS_PASSWORD,
    },
  });
}

export const config = loadConfig();
export type { Config };
