// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Real-time Service
 * Handles WebSocket connections, presence, live collaboration, notifications
 */

import { Redis } from 'ioredis';
import { WebSocket, WebSocketServer } from 'ws';
import { EventEmitter } from 'events';
import { IncomingMessage } from 'http';

// Types
interface Client {
  id: string;
  userId: string;
  ws: WebSocket;
  rooms: Set<string>;
  metadata: ClientMetadata;
  lastPing: Date;
}

interface ClientMetadata {
  device: string;
  platform: string;
  version: string;
  userAgent?: string;
}

interface Message {
  type: MessageType;
  room?: string;
  payload: any;
  timestamp: number;
}

type MessageType =
  | 'join'
  | 'leave'
  | 'message'
  | 'presence'
  | 'typing'
  | 'cursor'
  | 'sync'
  | 'ping'
  | 'pong'
  | 'error'
  | 'notification'
  | 'playback_sync'
  | 'collaboration_update'
  | 'track_update'
  | 'live_stream';

interface Room {
  id: string;
  type: RoomType;
  members: Set<string>;
  state: Record<string, any>;
  createdAt: Date;
}

type RoomType =
  | 'collaboration'
  | 'live_stream'
  | 'listening_party'
  | 'chat'
  | 'user_presence';

interface PresenceInfo {
  userId: string;
  status: 'online' | 'away' | 'busy' | 'offline';
  currentTrack?: {
    id: string;
    position: number;
  };
  lastSeen: Date;
}

interface CollaborationCursor {
  userId: string;
  trackId: string;
  position: number;
  selection?: { start: number; end: number };
  color: string;
}

interface LiveStreamState {
  hostId: string;
  viewerCount: number;
  chatEnabled: boolean;
  currentPosition: number;
}

export class RealtimeService extends EventEmitter {
  private wss: WebSocketServer;
  private redis: Redis;
  private redisSub: Redis;
  private clients: Map<string, Client> = new Map();
  private rooms: Map<string, Room> = new Map();
  private heartbeatInterval: NodeJS.Timeout;

  constructor(wss: WebSocketServer, redis: Redis) {
    super();
    this.wss = wss;
    this.redis = redis;
    this.redisSub = redis.duplicate();

    this.setupRedisSubscriptions();
    this.startHeartbeat();
    this.setupWebSocketHandlers();
  }

  /**
   * Initialize WebSocket handlers
   */
  private setupWebSocketHandlers() {
    this.wss.on('connection', async (ws: WebSocket, request: IncomingMessage) => {
      // Authenticate connection
      const userId = await this.authenticateConnection(request);
      if (!userId) {
        ws.close(4001, 'Unauthorized');
        return;
      }

      const client = await this.registerClient(ws, userId, request);

      ws.on('message', async (data: Buffer) => {
        try {
          const message: Message = JSON.parse(data.toString());
          await this.handleMessage(client, message);
        } catch (error) {
          this.sendError(client, 'Invalid message format');
        }
      });

      ws.on('close', () => this.handleDisconnect(client));
      ws.on('error', (error) => this.handleError(client, error));

      // Send welcome message
      this.send(client, {
        type: 'sync',
        payload: {
          clientId: client.id,
          serverTime: Date.now(),
        },
        timestamp: Date.now(),
      });
    });
  }

  /**
   * Register a new client
   */
  private async registerClient(
    ws: WebSocket,
    userId: string,
    request: IncomingMessage
  ): Promise<Client> {
    const clientId = this.generateClientId();

    const client: Client = {
      id: clientId,
      userId,
      ws,
      rooms: new Set(),
      metadata: this.extractMetadata(request),
      lastPing: new Date(),
    };

    this.clients.set(clientId, client);

    // Update presence
    await this.updatePresence(userId, 'online');

    // Auto-join user's personal room for notifications
    await this.joinRoom(client, `user:${userId}`);

    this.emit('client:connected', { clientId, userId });
    return client;
  }

  /**
   * Handle incoming messages
   */
  private async handleMessage(client: Client, message: Message) {
    switch (message.type) {
      case 'join':
        await this.handleJoin(client, message);
        break;

      case 'leave':
        await this.handleLeave(client, message);
        break;

      case 'message':
        await this.handleRoomMessage(client, message);
        break;

      case 'typing':
        await this.handleTyping(client, message);
        break;

      case 'cursor':
        await this.handleCursor(client, message);
        break;

      case 'playback_sync':
        await this.handlePlaybackSync(client, message);
        break;

      case 'collaboration_update':
        await this.handleCollaborationUpdate(client, message);
        break;

      case 'ping':
        this.handlePing(client);
        break;

      default:
        this.sendError(client, `Unknown message type: ${message.type}`);
    }
  }

  /**
   * Join a room
   */
  async joinRoom(client: Client, roomId: string, metadata?: any): Promise<void> {
    // Get or create room
    let room = this.rooms.get(roomId);
    if (!room) {
      room = {
        id: roomId,
        type: this.getRoomType(roomId),
        members: new Set(),
        state: {},
        createdAt: new Date(),
      };
      this.rooms.set(roomId, room);
    }

    // Add client to room
    room.members.add(client.id);
    client.rooms.add(roomId);

    // Subscribe to Redis channel for this room
    await this.redisSub.subscribe(`room:${roomId}`);

    // Notify other members
    this.broadcastToRoom(roomId, {
      type: 'presence',
      room: roomId,
      payload: {
        action: 'join',
        userId: client.userId,
        metadata,
      },
      timestamp: Date.now(),
    }, client.id);

    // Send room state to new member
    this.send(client, {
      type: 'sync',
      room: roomId,
      payload: {
        members: Array.from(room.members).map(id => {
          const c = this.clients.get(id);
          return c ? { id: c.userId, ...c.metadata } : null;
        }).filter(Boolean),
        state: room.state,
      },
      timestamp: Date.now(),
    });

    this.emit('room:join', { roomId, clientId: client.id, userId: client.userId });
  }

  /**
   * Leave a room
   */
  async leaveRoom(client: Client, roomId: string): Promise<void> {
    const room = this.rooms.get(roomId);
    if (!room) return;

    room.members.delete(client.id);
    client.rooms.delete(roomId);

    // Notify other members
    this.broadcastToRoom(roomId, {
      type: 'presence',
      room: roomId,
      payload: {
        action: 'leave',
        userId: client.userId,
      },
      timestamp: Date.now(),
    });

    // Clean up empty rooms
    if (room.members.size === 0) {
      this.rooms.delete(roomId);
      await this.redisSub.unsubscribe(`room:${roomId}`);
    }

    this.emit('room:leave', { roomId, clientId: client.id, userId: client.userId });
  }

  /**
   * Broadcast message to room
   */
  broadcastToRoom(roomId: string, message: Message, excludeClientId?: string): void {
    const room = this.rooms.get(roomId);
    if (!room) return;

    for (const clientId of room.members) {
      if (clientId === excludeClientId) continue;
      const client = this.clients.get(clientId);
      if (client) {
        this.send(client, message);
      }
    }

    // Also publish to Redis for distributed systems
    this.redis.publish(`room:${roomId}`, JSON.stringify({
      ...message,
      excludeClientId,
      serverId: process.env.SERVER_ID,
    }));
  }

  /**
   * Send notification to specific user
   */
  async sendNotification(userId: string, notification: any): Promise<void> {
    // Store notification
    await this.redis.lpush(`notifications:${userId}`, JSON.stringify(notification));

    // Send to all user's connected clients
    const roomId = `user:${userId}`;
    this.broadcastToRoom(roomId, {
      type: 'notification',
      payload: notification,
      timestamp: Date.now(),
    });
  }

  /**
   * Handle playback synchronization for listening parties
   */
  private async handlePlaybackSync(client: Client, message: Message): Promise<void> {
    const { roomId, trackId, position, action, timestamp } = message.payload;

    // Update room state
    const room = this.rooms.get(roomId);
    if (room) {
      room.state.playback = { trackId, position, action, lastUpdate: timestamp };
    }

    // Broadcast to room
    this.broadcastToRoom(roomId, {
      type: 'playback_sync',
      room: roomId,
      payload: {
        userId: client.userId,
        trackId,
        position,
        action,
        serverTime: Date.now(),
      },
      timestamp: Date.now(),
    }, client.id);
  }

  /**
   * Handle real-time collaboration updates
   */
  private async handleCollaborationUpdate(client: Client, message: Message): Promise<void> {
    const { collaborationId, type, data } = message.payload;
    const roomId = `collaboration:${collaborationId}`;

    // Validate user is part of collaboration
    if (!client.rooms.has(roomId)) {
      this.sendError(client, 'Not a member of this collaboration');
      return;
    }

    // Handle different collaboration update types
    switch (type) {
      case 'track_edit':
        await this.handleTrackEdit(roomId, client, data);
        break;

      case 'cursor_move':
        await this.handleCursorMove(roomId, client, data);
        break;

      case 'clip_add':
      case 'clip_move':
      case 'clip_delete':
        await this.handleClipOperation(roomId, client, type, data);
        break;

      case 'effect_change':
        await this.handleEffectChange(roomId, client, data);
        break;

      case 'mixer_change':
        await this.handleMixerChange(roomId, client, data);
        break;
    }
  }

  /**
   * Handle cursor movement in collaboration
   */
  private async handleCursorMove(roomId: string, client: Client, data: any): Promise<void> {
    const cursor: CollaborationCursor = {
      userId: client.userId,
      trackId: data.trackId,
      position: data.position,
      selection: data.selection,
      color: data.color || this.getUserColor(client.userId),
    };

    this.broadcastToRoom(roomId, {
      type: 'cursor',
      room: roomId,
      payload: cursor,
      timestamp: Date.now(),
    }, client.id);
  }

  /**
   * Handle live streaming
   */
  async startLiveStream(
    hostId: string,
    streamId: string,
    options: { title: string; chatEnabled: boolean }
  ): Promise<Room> {
    const roomId = `live:${streamId}`;

    const room: Room = {
      id: roomId,
      type: 'live_stream',
      members: new Set(),
      state: {
        hostId,
        title: options.title,
        chatEnabled: options.chatEnabled,
        viewerCount: 0,
        startedAt: new Date(),
      } as LiveStreamState,
      createdAt: new Date(),
    };

    this.rooms.set(roomId, room);

    // Notify followers
    const followers = await this.getFollowers(hostId);
    for (const followerId of followers) {
      await this.sendNotification(followerId, {
        type: 'live_stream_started',
        data: {
          streamId,
          hostId,
          title: options.title,
        },
      });
    }

    return room;
  }

  /**
   * Update user presence
   */
  async updatePresence(userId: string, status: PresenceInfo['status'], currentTrack?: any): Promise<void> {
    const presence: PresenceInfo = {
      userId,
      status,
      currentTrack,
      lastSeen: new Date(),
    };

    // Store in Redis
    await this.redis.hset(
      'presence',
      userId,
      JSON.stringify(presence)
    );

    // Broadcast to user's followers
    const followers = await this.getFollowers(userId);
    for (const followerId of followers) {
      this.broadcastToRoom(`user:${followerId}`, {
        type: 'presence',
        payload: presence,
        timestamp: Date.now(),
      });
    }
  }

  /**
   * Get user presence
   */
  async getPresence(userId: string): Promise<PresenceInfo | null> {
    const data = await this.redis.hget('presence', userId);
    return data ? JSON.parse(data) : null;
  }

  /**
   * Get online friends
   */
  async getOnlineFriends(userId: string): Promise<PresenceInfo[]> {
    const following = await this.getFollowing(userId);
    const presenceData = await this.redis.hmget('presence', ...following);

    return presenceData
      .filter(Boolean)
      .map(data => JSON.parse(data as string))
      .filter((p: PresenceInfo) => p.status !== 'offline');
  }

  // Private helper methods

  private async authenticateConnection(request: IncomingMessage): Promise<string | null> {
    // Extract token from query string or headers
    const url = new URL(request.url || '', 'ws://localhost');
    const token = url.searchParams.get('token') || request.headers.authorization?.replace('Bearer ', '');

    if (!token) return null;

    // Validate token (this would verify JWT in production)
    try {
      // Simulated token validation
      const userId = `user_${token.slice(0, 8)}`;
      return userId;
    } catch {
      return null;
    }
  }

  private handleDisconnect(client: Client): void {
    // Leave all rooms
    for (const roomId of client.rooms) {
      this.leaveRoom(client, roomId);
    }

    // Update presence
    this.updatePresence(client.userId, 'offline');

    // Remove client
    this.clients.delete(client.id);

    this.emit('client:disconnected', { clientId: client.id, userId: client.userId });
  }

  private handleError(client: Client, error: Error): void {
    console.error(`WebSocket error for client ${client.id}:`, error);
    this.emit('client:error', { clientId: client.id, error });
  }

  private handlePing(client: Client): void {
    client.lastPing = new Date();
    this.send(client, {
      type: 'pong',
      payload: { serverTime: Date.now() },
      timestamp: Date.now(),
    });
  }

  private send(client: Client, message: Message): void {
    if (client.ws.readyState === WebSocket.OPEN) {
      client.ws.send(JSON.stringify(message));
    }
  }

  private sendError(client: Client, error: string): void {
    this.send(client, {
      type: 'error',
      payload: { message: error },
      timestamp: Date.now(),
    });
  }

  private setupRedisSubscriptions(): void {
    this.redisSub.on('message', (channel: string, message: string) => {
      try {
        const data = JSON.parse(message);
        // Skip messages from this server
        if (data.serverId === process.env.SERVER_ID) return;

        const roomId = channel.replace('room:', '');
        const room = this.rooms.get(roomId);
        if (!room) return;

        // Broadcast to local clients
        for (const clientId of room.members) {
          if (clientId === data.excludeClientId) continue;
          const client = this.clients.get(clientId);
          if (client) {
            this.send(client, data);
          }
        }
      } catch (error) {
        console.error('Redis message error:', error);
      }
    });
  }

  private startHeartbeat(): void {
    this.heartbeatInterval = setInterval(() => {
      const now = Date.now();
      const timeout = 30000; // 30 seconds

      for (const [clientId, client] of this.clients) {
        if (now - client.lastPing.getTime() > timeout) {
          // Client hasn't responded to ping
          client.ws.terminate();
          this.handleDisconnect(client);
        } else {
          // Send ping
          this.send(client, {
            type: 'ping',
            payload: {},
            timestamp: now,
          });
        }
      }
    }, 15000); // Check every 15 seconds
  }

  private generateClientId(): string {
    return `client_${Date.now()}_${Math.random().toString(36).slice(2)}`;
  }

  private extractMetadata(request: IncomingMessage): ClientMetadata {
    const userAgent = request.headers['user-agent'] || '';
    return {
      device: this.parseDevice(userAgent),
      platform: this.parsePlatform(userAgent),
      version: '1.0.0',
      userAgent,
    };
  }

  private parseDevice(userAgent: string): string {
    if (userAgent.includes('Mobile')) return 'mobile';
    if (userAgent.includes('Tablet')) return 'tablet';
    return 'desktop';
  }

  private parsePlatform(userAgent: string): string {
    if (userAgent.includes('Windows')) return 'windows';
    if (userAgent.includes('Mac')) return 'macos';
    if (userAgent.includes('Linux')) return 'linux';
    if (userAgent.includes('Android')) return 'android';
    if (userAgent.includes('iOS')) return 'ios';
    return 'unknown';
  }

  private getRoomType(roomId: string): RoomType {
    if (roomId.startsWith('collaboration:')) return 'collaboration';
    if (roomId.startsWith('live:')) return 'live_stream';
    if (roomId.startsWith('party:')) return 'listening_party';
    if (roomId.startsWith('chat:')) return 'chat';
    return 'user_presence';
  }

  private getUserColor(userId: string): string {
    // Generate consistent color for user
    const colors = ['#ef4444', '#f97316', '#eab308', '#22c55e', '#06b6d4', '#3b82f6', '#8b5cf6', '#ec4899'];
    const hash = userId.split('').reduce((acc, char) => acc + char.charCodeAt(0), 0);
    return colors[hash % colors.length];
  }

  private async getFollowers(userId: string): Promise<string[]> {
    // This would fetch from database
    return [];
  }

  private async getFollowing(userId: string): Promise<string[]> {
    // This would fetch from database
    return [];
  }

  // Stub handlers for collaboration operations
  private async handleJoin(client: Client, message: Message): Promise<void> {
    await this.joinRoom(client, message.payload.roomId, message.payload.metadata);
  }

  private async handleLeave(client: Client, message: Message): Promise<void> {
    await this.leaveRoom(client, message.payload.roomId);
  }

  private async handleRoomMessage(client: Client, message: Message): Promise<void> {
    this.broadcastToRoom(message.room!, {
      type: 'message',
      room: message.room,
      payload: {
        userId: client.userId,
        content: message.payload.content,
      },
      timestamp: Date.now(),
    }, client.id);
  }

  private async handleTyping(client: Client, message: Message): Promise<void> {
    this.broadcastToRoom(message.room!, {
      type: 'typing',
      room: message.room,
      payload: { userId: client.userId, isTyping: message.payload.isTyping },
      timestamp: Date.now(),
    }, client.id);
  }

  private async handleCursor(client: Client, message: Message): Promise<void> {
    await this.handleCursorMove(message.room!, client, message.payload);
  }

  private async handleTrackEdit(roomId: string, client: Client, data: any): Promise<void> {
    this.broadcastToRoom(roomId, {
      type: 'collaboration_update',
      room: roomId,
      payload: { type: 'track_edit', userId: client.userId, data },
      timestamp: Date.now(),
    }, client.id);
  }

  private async handleClipOperation(roomId: string, client: Client, type: string, data: any): Promise<void> {
    this.broadcastToRoom(roomId, {
      type: 'collaboration_update',
      room: roomId,
      payload: { type, userId: client.userId, data },
      timestamp: Date.now(),
    }, client.id);
  }

  private async handleEffectChange(roomId: string, client: Client, data: any): Promise<void> {
    this.broadcastToRoom(roomId, {
      type: 'collaboration_update',
      room: roomId,
      payload: { type: 'effect_change', userId: client.userId, data },
      timestamp: Date.now(),
    }, client.id);
  }

  private async handleMixerChange(roomId: string, client: Client, data: any): Promise<void> {
    this.broadcastToRoom(roomId, {
      type: 'collaboration_update',
      room: roomId,
      payload: { type: 'mixer_change', userId: client.userId, data },
      timestamp: Date.now(),
    }, client.id);
  }

  /**
   * Cleanup
   */
  destroy(): void {
    clearInterval(this.heartbeatInterval);
    this.redisSub.disconnect();

    for (const client of this.clients.values()) {
      client.ws.close();
    }

    this.clients.clear();
    this.rooms.clear();
  }
}

export default RealtimeService;
