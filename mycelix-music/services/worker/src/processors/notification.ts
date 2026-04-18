// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Notification Processor
 *
 * Handles push notifications and in-app notifications.
 */

import { Job } from 'bullmq';
import { createLogger } from '../logger';

const logger = createLogger('notification-processor');

export interface PushNotificationJob {
  type: 'push';
  userId: string;
  title: string;
  body: string;
  data?: Record<string, string>;
  badge?: number;
}

export interface InAppNotificationJob {
  type: 'in-app';
  userId: string;
  notificationType: 'follow' | 'like' | 'comment' | 'mention' | 'royalty' | 'milestone' | 'release';
  title: string;
  message: string;
  actionUrl?: string;
  metadata?: Record<string, unknown>;
}

export interface BroadcastNotificationJob {
  type: 'broadcast';
  targetType: 'all' | 'artists' | 'subscribers';
  title: string;
  message: string;
  actionUrl?: string;
}

export type NotificationJob =
  | PushNotificationJob
  | InAppNotificationJob
  | BroadcastNotificationJob;

export async function notificationProcessor(job: Job<NotificationJob>): Promise<void> {
  const { data } = job;

  logger.info({ type: data.type }, 'Processing notification job');

  switch (data.type) {
    case 'push':
      await sendPushNotification(data);
      break;

    case 'in-app':
      await createInAppNotification(data);
      break;

    case 'broadcast':
      await broadcastNotification(data);
      break;

    default:
      throw new Error(`Unknown notification job type: ${(data as any).type}`);
  }
}

async function sendPushNotification(data: PushNotificationJob): Promise<void> {
  logger.info({ userId: data.userId, title: data.title }, 'Sending push notification');

  // In production:
  // 1. Get user's push tokens from database
  // 2. Send via FCM/APNS
  // 3. Handle failures and retry

  // Simulate sending
  await new Promise(resolve => setTimeout(resolve, 50));

  logger.info({ userId: data.userId }, 'Push notification sent');
}

async function createInAppNotification(data: InAppNotificationJob): Promise<void> {
  logger.info({ userId: data.userId, type: data.notificationType }, 'Creating in-app notification');

  // In production:
  // 1. Insert notification into database
  // 2. Increment user's unread count
  // 3. Publish to WebSocket for real-time delivery

  const notification = {
    id: `notif_${Date.now()}`,
    userId: data.userId,
    type: data.notificationType,
    title: data.title,
    message: data.message,
    actionUrl: data.actionUrl,
    metadata: data.metadata,
    read: false,
    createdAt: new Date().toISOString(),
  };

  // Simulate database insert
  await new Promise(resolve => setTimeout(resolve, 20));

  logger.info({ notificationId: notification.id }, 'In-app notification created');
}

async function broadcastNotification(data: BroadcastNotificationJob): Promise<void> {
  logger.info({ targetType: data.targetType }, 'Broadcasting notification');

  // In production:
  // 1. Query users matching target type
  // 2. Batch insert notifications
  // 3. Send push notifications in batches

  // Simulate getting user count
  const userCounts = {
    all: 10000,
    artists: 500,
    subscribers: 2000,
  };

  const targetCount = userCounts[data.targetType];

  // Process in batches
  const batchSize = 100;
  const batches = Math.ceil(targetCount / batchSize);

  for (let i = 0; i < batches; i++) {
    // Simulate batch processing
    await new Promise(resolve => setTimeout(resolve, 10));

    if (i % 10 === 0) {
      logger.info({ progress: `${i + 1}/${batches}` }, 'Broadcast progress');
    }
  }

  logger.info({ targetCount, targetType: data.targetType }, 'Broadcast complete');
}
