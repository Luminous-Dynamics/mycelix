// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Analytics Processor
 *
 * Handles analytics aggregation and processing.
 */

import { Job } from 'bullmq';
import { createLogger } from '../logger';

const logger = createLogger('analytics-processor');

export interface TrackPlayJob {
  type: 'track-play';
  songId: string;
  userId?: string;
  artistId: string;
  duration: number; // seconds listened
  completed: boolean;
  source: 'web' | 'mobile' | 'api';
  timestamp: string;
}

export interface DailyAggregationJob {
  type: 'daily-aggregation';
  date?: string; // defaults to yesterday
}

export interface ArtistStatsUpdateJob {
  type: 'artist-stats-update';
  artistId: string;
}

export type AnalyticsJob =
  | TrackPlayJob
  | DailyAggregationJob
  | ArtistStatsUpdateJob;

export async function analyticsProcessor(job: Job<AnalyticsJob>): Promise<void> {
  const { data } = job;

  logger.info({ type: data.type }, 'Processing analytics job');

  switch (data.type) {
    case 'track-play':
      await processTrackPlay(data);
      break;

    case 'daily-aggregation':
      await processDailyAggregation(data);
      break;

    case 'artist-stats-update':
      await processArtistStatsUpdate(data);
      break;

    default:
      throw new Error(`Unknown analytics job type: ${(data as any).type}`);
  }
}

async function processTrackPlay(data: TrackPlayJob): Promise<void> {
  // In production, this would:
  // 1. Increment play counts in database
  // 2. Update real-time stats in Redis
  // 3. Track for royalty calculation
  // 4. Update recommendation engine

  logger.info({
    songId: data.songId,
    artistId: data.artistId,
    duration: data.duration,
    completed: data.completed,
  }, 'Track play recorded');

  // Simulate processing
  await new Promise(resolve => setTimeout(resolve, 10));
}

async function processDailyAggregation(data: DailyAggregationJob): Promise<void> {
  const date = data.date || getYesterday();

  logger.info({ date }, 'Running daily aggregation');

  // In production, this would:
  // 1. Aggregate play counts by song, artist, genre
  // 2. Calculate trending scores
  // 3. Generate daily stats reports
  // 4. Update leaderboards

  const steps = [
    'Aggregating play counts',
    'Calculating trending scores',
    'Updating artist stats',
    'Generating reports',
  ];

  for (const step of steps) {
    logger.info({ date, step }, 'Aggregation step');
    await new Promise(resolve => setTimeout(resolve, 100));
  }

  logger.info({ date }, 'Daily aggregation complete');
}

async function processArtistStatsUpdate(data: ArtistStatsUpdateJob): Promise<void> {
  logger.info({ artistId: data.artistId }, 'Updating artist stats');

  // In production, this would:
  // 1. Recalculate total plays
  // 2. Update follower count
  // 3. Calculate engagement rate
  // 4. Update monthly listeners

  await new Promise(resolve => setTimeout(resolve, 50));

  logger.info({ artistId: data.artistId }, 'Artist stats updated');
}

function getYesterday(): string {
  const date = new Date();
  date.setDate(date.getDate() - 1);
  return date.toISOString().split('T')[0];
}
