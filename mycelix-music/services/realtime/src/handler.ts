// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * WebSocket Message Handler
 *
 * Handles incoming WebSocket messages and Redis events.
 */

import jwt from 'jsonwebtoken';
import Redis from 'ioredis';
import { ConnectionManager } from './connections';
import { createLogger } from './logger';
import { config } from './config';
import {
  CHANNELS,
  RedisEvent,
  NotificationEvent,
  PlayEvent,
  FollowEvent,
  LikeEvent,
  ActivityEvent,
  PresenceEvent,
} from './redis';

const logger = createLogger('handler');

interface DecodedToken {
  address: string;
  sub?: string;
  iat: number;
  exp: number;
}

interface IncomingMessage {
  type: string;
  [key: string]: unknown;
}

export class MessageHandler {
  constructor(
    private connectionManager: ConnectionManager,
    private publisher: Redis
  ) {}

  /**
   * Authenticate a connection with JWT token
   */
  async authenticateConnection(
    connectionId: string,
    token: string
  ): Promise<DecodedToken | null> {
    try {
      const decoded = jwt.verify(token, config.jwt.secret) as DecodedToken;
      const userId = decoded.address || decoded.sub;

      if (userId) {
        this.connectionManager.setUser(connectionId, userId);

        // Auto-join user's personal notification room
        this.connectionManager.joinRoom(connectionId, `user:${userId}`);

        // Publish online status
        await this.publishPresence(userId, 'online');

        return decoded;
      }

      return null;
    } catch (error) {
      logger.warn({ connectionId, error }, 'Token verification failed');
      return null;
    }
  }

  /**
   * Handle incoming WebSocket message
   */
  async handleMessage(connectionId: string, message: IncomingMessage): Promise<void> {
    const connection = this.connectionManager.getConnection(connectionId);
    if (!connection) return;

    logger.debug({ connectionId, type: message.type }, 'Handling message');

    switch (message.type) {
      case 'ping':
        this.handlePing(connectionId);
        break;

      case 'auth':
        await this.handleAuth(connectionId, message.token as string);
        break;

      case 'subscribe':
        this.handleSubscribe(connectionId, message.room as string);
        break;

      case 'unsubscribe':
        this.handleUnsubscribe(connectionId, message.room as string);
        break;

      case 'presence':
        await this.handlePresenceUpdate(connectionId, message);
        break;

      case 'activity':
        await this.handleActivity(connectionId, message);
        break;

      default:
        logger.warn({ connectionId, type: message.type }, 'Unknown message type');
        this.connectionManager.send(connectionId, {
          type: 'error',
          message: `Unknown message type: ${message.type}`,
        });
    }
  }

  /**
   * Handle ping message (heartbeat)
   */
  private handlePing(connectionId: string): void {
    this.connectionManager.updateHeartbeat(connectionId);
    this.connectionManager.send(connectionId, { type: 'pong', timestamp: Date.now() });
  }

  /**
   * Handle authentication message
   */
  private async handleAuth(connectionId: string, token: string): Promise<void> {
    if (!token) {
      this.connectionManager.send(connectionId, {
        type: 'auth_error',
        message: 'Token required',
      });
      return;
    }

    const user = await this.authenticateConnection(connectionId, token);
    if (user) {
      this.connectionManager.send(connectionId, {
        type: 'auth_success',
        userId: user.address || user.sub,
      });
    } else {
      this.connectionManager.send(connectionId, {
        type: 'auth_error',
        message: 'Invalid token',
      });
    }
  }

  /**
   * Handle room subscription
   */
  private handleSubscribe(connectionId: string, room: string): void {
    const connection = this.connectionManager.getConnection(connectionId);
    if (!connection) return;

    // Validate room access
    if (!this.canAccessRoom(connection.userId, room)) {
      this.connectionManager.send(connectionId, {
        type: 'subscribe_error',
        room,
        message: 'Access denied',
      });
      return;
    }

    this.connectionManager.joinRoom(connectionId, room);
    this.connectionManager.send(connectionId, {
      type: 'subscribed',
      room,
    });

    logger.debug({ connectionId, room }, 'Subscribed to room');
  }

  /**
   * Handle room unsubscription
   */
  private handleUnsubscribe(connectionId: string, room: string): void {
    this.connectionManager.leaveRoom(connectionId, room);
    this.connectionManager.send(connectionId, {
      type: 'unsubscribed',
      room,
    });
  }

  /**
   * Handle presence update from client
   */
  private async handlePresenceUpdate(
    connectionId: string,
    message: IncomingMessage
  ): Promise<void> {
    const connection = this.connectionManager.getConnection(connectionId);
    if (!connection?.userId) return;

    const status = message.status as 'online' | 'listening';
    const currentSong = message.currentSong as { id: string; title: string; artist: string } | undefined;

    await this.publishPresence(connection.userId, status, currentSong);
  }

  /**
   * Handle activity broadcast from client
   */
  private async handleActivity(
    connectionId: string,
    message: IncomingMessage
  ): Promise<void> {
    const connection = this.connectionManager.getConnection(connectionId);
    if (!connection?.userId) return;

    // Broadcast activity to followers
    const activity: ActivityEvent = {
      type: 'activity',
      activityType: message.activityType as ActivityEvent['activityType'],
      userId: connection.userId,
      userName: message.userName as string,
      userAvatar: message.userAvatar as string | undefined,
      data: message.data as Record<string, unknown>,
      timestamp: new Date().toISOString(),
    };

    await this.publisher.publish(CHANNELS.ACTIVITY, JSON.stringify(activity));
  }

  /**
   * Publish user presence update
   */
  private async publishPresence(
    userId: string,
    status: 'online' | 'offline' | 'listening',
    currentSong?: { id: string; title: string; artist: string }
  ): Promise<void> {
    const event: PresenceEvent = {
      type: 'presence',
      userId,
      status,
      currentSong,
      timestamp: new Date().toISOString(),
    };

    await this.publisher.publish(CHANNELS.PRESENCE, JSON.stringify(event));
  }

  /**
   * Check if user can access a room
   */
  private canAccessRoom(userId: string | undefined, room: string): boolean {
    // Public rooms anyone can join
    const publicRooms = ['global', 'trending', 'new-releases'];
    if (publicRooms.includes(room)) return true;

    // Song rooms (for live play counts)
    if (room.startsWith('song:')) return true;

    // Artist rooms (for artist updates)
    if (room.startsWith('artist:')) return true;

    // Playlist rooms
    if (room.startsWith('playlist:')) return true;

    // User-specific rooms require authentication
    if (room.startsWith('user:')) {
      return userId !== undefined;
    }

    // Private rooms require user match
    if (room.startsWith('private:')) {
      const roomUserId = room.split(':')[1];
      return userId === roomUserId;
    }

    return false;
  }

  /**
   * Handle Redis pub/sub events
   */
  handleRedisEvent(channel: string, event: RedisEvent): void {
    logger.debug({ channel, eventType: event.type }, 'Processing Redis event');

    switch (channel) {
      case CHANNELS.NOTIFICATIONS:
        this.handleNotificationEvent(event as NotificationEvent);
        break;

      case CHANNELS.PLAYS:
        this.handlePlayEvent(event as PlayEvent);
        break;

      case CHANNELS.FOLLOWS:
        this.handleFollowEvent(event as FollowEvent);
        break;

      case CHANNELS.LIKES:
        this.handleLikeEvent(event as LikeEvent);
        break;

      case CHANNELS.ACTIVITY:
        this.handleActivityEvent(event as ActivityEvent);
        break;

      case CHANNELS.PRESENCE:
        this.handlePresenceEvent(event as PresenceEvent);
        break;

      default:
        logger.warn({ channel }, 'Unknown Redis channel');
    }
  }

  /**
   * Handle notification event - send to specific user
   */
  private handleNotificationEvent(event: NotificationEvent): void {
    const sent = this.connectionManager.sendToUser(event.userId, {
      type: 'notification',
      notification: event.notification,
    });
    logger.debug({ userId: event.userId, sent }, 'Notification sent');
  }

  /**
   * Handle play event - broadcast to song room
   */
  private handlePlayEvent(event: PlayEvent): void {
    // Broadcast to song room
    this.connectionManager.broadcastToRoom(`song:${event.songId}`, {
      type: 'play_update',
      songId: event.songId,
      playCount: event.playCount,
    });

    // Broadcast to artist room
    this.connectionManager.broadcastToRoom(`artist:${event.artistId}`, {
      type: 'song_played',
      songId: event.songId,
      playCount: event.playCount,
    });
  }

  /**
   * Handle follow event - notify both parties
   */
  private handleFollowEvent(event: FollowEvent): void {
    // Notify the person being followed
    this.connectionManager.sendToUser(event.followingId, {
      type: event.type === 'follow' ? 'new_follower' : 'unfollowed',
      follower: {
        id: event.followerId,
        name: event.followerName,
        avatar: event.followerAvatar,
      },
      timestamp: event.timestamp,
    });

    // Broadcast to artist room if applicable
    this.connectionManager.broadcastToRoom(`artist:${event.followingId}`, {
      type: 'follower_update',
      action: event.type,
    });
  }

  /**
   * Handle like event - notify content owner
   */
  private handleLikeEvent(event: LikeEvent): void {
    // Broadcast to content room
    const roomPrefix = event.targetType === 'song' ? 'song' :
                       event.targetType === 'playlist' ? 'playlist' : 'comment';

    this.connectionManager.broadcastToRoom(`${roomPrefix}:${event.targetId}`, {
      type: event.type === 'like' ? 'new_like' : 'unlike',
      targetType: event.targetType,
      targetId: event.targetId,
      user: {
        id: event.userId,
        name: event.userName,
      },
    });
  }

  /**
   * Handle activity event - broadcast to followers
   */
  private handleActivityEvent(event: ActivityEvent): void {
    // Broadcast to global activity feed
    this.connectionManager.broadcastToRoom('global', {
      type: 'activity',
      activity: event,
    });

    // Could also broadcast to user's followers specifically
    // This would require looking up followers from a database
  }

  /**
   * Handle presence event - broadcast to relevant rooms
   */
  private handlePresenceEvent(event: PresenceEvent): void {
    // Broadcast to user's profile room (for profile pages showing online status)
    this.connectionManager.broadcastToRoom(`user:${event.userId}:presence`, {
      type: 'presence_update',
      userId: event.userId,
      status: event.status,
      currentSong: event.currentSong,
    });

    // If listening, broadcast to song room
    if (event.status === 'listening' && event.currentSong) {
      this.connectionManager.broadcastToRoom(`song:${event.currentSong.id}`, {
        type: 'listener_update',
        userId: event.userId,
        song: event.currentSong,
      });
    }
  }

  /**
   * Handle connection disconnect
   */
  async handleDisconnect(connectionId: string): Promise<void> {
    const connection = this.connectionManager.getConnection(connectionId);
    if (connection?.userId) {
      // Check if user has other connections
      const userConnections = this.connectionManager.getUserConnections(connection.userId);
      if (userConnections.length <= 1) {
        // This is the last connection, user is going offline
        await this.publishPresence(connection.userId, 'offline');
      }
    }
  }
}
