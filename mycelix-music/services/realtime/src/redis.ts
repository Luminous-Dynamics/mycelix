// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Redis Pub/Sub Utilities
 *
 * Creates Redis clients for publishing and subscribing to events.
 */

import Redis from 'ioredis';
import { config } from './config';
import { createLogger } from './logger';

const logger = createLogger('redis');

/**
 * Create a Redis subscriber client
 */
export async function createRedisSubscriber(): Promise<Redis> {
  const subscriber = new Redis({
    host: config.redis.host,
    port: config.redis.port,
    password: config.redis.password,
    retryStrategy: (times) => {
      const delay = Math.min(times * 50, 2000);
      logger.warn({ attempt: times, delay }, 'Retrying Redis subscriber connection');
      return delay;
    },
    maxRetriesPerRequest: null,
  });

  subscriber.on('connect', () => {
    logger.info('Redis subscriber connected');
  });

  subscriber.on('error', (error) => {
    logger.error({ error }, 'Redis subscriber error');
  });

  subscriber.on('close', () => {
    logger.warn('Redis subscriber connection closed');
  });

  return subscriber;
}

/**
 * Create a Redis publisher client
 */
export async function createRedisPublisher(): Promise<Redis> {
  const publisher = new Redis({
    host: config.redis.host,
    port: config.redis.port,
    password: config.redis.password,
    retryStrategy: (times) => {
      const delay = Math.min(times * 50, 2000);
      logger.warn({ attempt: times, delay }, 'Retrying Redis publisher connection');
      return delay;
    },
    maxRetriesPerRequest: null,
  });

  publisher.on('connect', () => {
    logger.info('Redis publisher connected');
  });

  publisher.on('error', (error) => {
    logger.error({ error }, 'Redis publisher error');
  });

  publisher.on('close', () => {
    logger.warn('Redis publisher connection closed');
  });

  return publisher;
}

/**
 * Redis channels used by the real-time service
 */
export const CHANNELS = {
  NOTIFICATIONS: 'mycelix:notifications',
  PLAYS: 'mycelix:plays',
  FOLLOWS: 'mycelix:follows',
  LIKES: 'mycelix:likes',
  ACTIVITY: 'mycelix:activity',
  PRESENCE: 'mycelix:presence',
  CHAT: 'mycelix:chat',
} as const;

/**
 * Event types for notifications
 */
export interface NotificationEvent {
  type: 'notification';
  userId: string;
  notification: {
    id: string;
    type: 'follow' | 'like' | 'comment' | 'mention' | 'royalty' | 'milestone';
    title: string;
    message: string;
    data?: Record<string, unknown>;
    timestamp: string;
  };
}

/**
 * Event for play count updates
 */
export interface PlayEvent {
  type: 'play';
  songId: string;
  artistId: string;
  userId?: string;
  playCount: number;
  timestamp: string;
}

/**
 * Event for follow updates
 */
export interface FollowEvent {
  type: 'follow' | 'unfollow';
  followerId: string;
  followingId: string;
  followerName: string;
  followerAvatar?: string;
  timestamp: string;
}

/**
 * Event for like updates
 */
export interface LikeEvent {
  type: 'like' | 'unlike';
  userId: string;
  targetType: 'song' | 'playlist' | 'comment';
  targetId: string;
  userName: string;
  timestamp: string;
}

/**
 * Event for activity feed
 */
export interface ActivityEvent {
  type: 'activity';
  activityType: 'upload' | 'playlist_create' | 'follow' | 'like' | 'comment' | 'achievement';
  userId: string;
  userName: string;
  userAvatar?: string;
  data: Record<string, unknown>;
  timestamp: string;
}

/**
 * Event for user presence
 */
export interface PresenceEvent {
  type: 'presence';
  userId: string;
  status: 'online' | 'offline' | 'listening';
  currentSong?: {
    id: string;
    title: string;
    artist: string;
  };
  timestamp: string;
}

export type RedisEvent =
  | NotificationEvent
  | PlayEvent
  | FollowEvent
  | LikeEvent
  | ActivityEvent
  | PresenceEvent;
