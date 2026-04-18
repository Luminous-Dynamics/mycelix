// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * @mycelix/realtime
 *
 * Unified real-time WebSocket service for the Mycelix ecosystem.
 * Handles:
 * - Listening Circles (synchronized playback)
 * - Collaborative Studio (real-time DAW)
 * - Notifications and presence
 * - Mycelium network updates
 */

import { serve } from '@hono/node-server';
import { Hono } from 'hono';
import { cors } from 'hono/cors';
import { WebSocketServer, WebSocket } from 'ws';
import { createServer } from 'http';
import Redis from 'ioredis';
import pino from 'pino';
import { nanoid } from 'nanoid';
import { z } from 'zod';

// ============================================================================
// Configuration
// ============================================================================

const config = {
  port: parseInt(process.env.PORT || '3001'),
  redisUrl: process.env.REDIS_URL || 'redis://localhost:6379',
  jwtSecret: process.env.JWT_SECRET || 'dev-secret',
};

const logger = pino({
  level: process.env.LOG_LEVEL || 'info',
  transport: {
    target: 'pino-pretty',
    options: { colorize: true },
  },
});

// ============================================================================
// Types
// ============================================================================

interface Client {
  id: string;
  ws: WebSocket;
  userId?: string;
  soulId?: string;
  rooms: Set<string>;
  metadata: Record<string, unknown>;
  connectedAt: Date;
  lastPing: Date;
}

interface Room {
  id: string;
  type: RoomType;
  clients: Map<string, Client>;
  state: Record<string, unknown>;
  createdAt: Date;
  metadata: Record<string, unknown>;
}

type RoomType = 'circle' | 'studio' | 'presence' | 'notifications';

// Message schemas
const BaseMessageSchema = z.object({
  type: z.string(),
  payload: z.unknown().optional(),
  roomId: z.string().optional(),
  timestamp: z.number().optional(),
});

const CircleMessageSchema = z.discriminatedUnion('type', [
  z.object({ type: z.literal('circle:join'), payload: z.object({ circleId: z.string() }) }),
  z.object({ type: z.literal('circle:leave'), payload: z.object({ circleId: z.string() }) }),
  z.object({ type: z.literal('circle:sync'), payload: z.object({ position: z.number(), isPlaying: z.boolean() }) }),
  z.object({ type: z.literal('circle:chat'), payload: z.object({ content: z.string(), timestamp: z.number() }) }),
  z.object({ type: z.literal('circle:reaction'), payload: z.object({ emoji: z.string(), position: z.object({ x: z.number(), y: z.number() }) }) }),
]);

const StudioMessageSchema = z.discriminatedUnion('type', [
  z.object({ type: z.literal('studio:join'), payload: z.object({ projectId: z.string() }) }),
  z.object({ type: z.literal('studio:leave'), payload: z.object({ projectId: z.string() }) }),
  z.object({ type: z.literal('studio:cursor'), payload: z.object({ x: z.number(), y: z.number(), trackId: z.string().optional() }) }),
  z.object({ type: z.literal('studio:stem:add'), payload: z.object({ stem: z.unknown() }) }),
  z.object({ type: z.literal('studio:stem:update'), payload: z.object({ stemId: z.string(), updates: z.unknown() }) }),
  z.object({ type: z.literal('studio:stem:delete'), payload: z.object({ stemId: z.string() }) }),
  z.object({ type: z.literal('studio:stem:lock'), payload: z.object({ stemId: z.string() }) }),
  z.object({ type: z.literal('studio:stem:unlock'), payload: z.object({ stemId: z.string() }) }),
  z.object({ type: z.literal('studio:transport'), payload: z.object({ action: z.enum(['play', 'pause', 'stop']), position: z.number().optional() }) }),
]);

// ============================================================================
// Redis Pub/Sub for Horizontal Scaling
// ============================================================================

class RedisPubSub {
  private pub: Redis;
  private sub: Redis;
  private handlers: Map<string, (message: string) => void> = new Map();

  constructor(redisUrl: string) {
    this.pub = new Redis(redisUrl);
    this.sub = new Redis(redisUrl);

    this.sub.on('message', (channel, message) => {
      const handler = this.handlers.get(channel);
      if (handler) {
        handler(message);
      }
    });

    this.pub.on('error', (err) => logger.error({ err }, 'Redis pub error'));
    this.sub.on('error', (err) => logger.error({ err }, 'Redis sub error'));
  }

  async publish(channel: string, message: unknown): Promise<void> {
    await this.pub.publish(channel, JSON.stringify(message));
  }

  async subscribe(channel: string, handler: (message: string) => void): Promise<void> {
    this.handlers.set(channel, handler);
    await this.sub.subscribe(channel);
  }

  async unsubscribe(channel: string): Promise<void> {
    this.handlers.delete(channel);
    await this.sub.unsubscribe(channel);
  }
}

// ============================================================================
// Room Manager
// ============================================================================

class RoomManager {
  private rooms: Map<string, Room> = new Map();
  private redis: RedisPubSub;

  constructor(redis: RedisPubSub) {
    this.redis = redis;
  }

  getRoom(roomId: string): Room | undefined {
    return this.rooms.get(roomId);
  }

  createRoom(roomId: string, type: RoomType, metadata: Record<string, unknown> = {}): Room {
    const room: Room = {
      id: roomId,
      type,
      clients: new Map(),
      state: {},
      createdAt: new Date(),
      metadata,
    };
    this.rooms.set(roomId, room);
    logger.info({ roomId, type }, 'Room created');
    return room;
  }

  getOrCreateRoom(roomId: string, type: RoomType): Room {
    return this.rooms.get(roomId) || this.createRoom(roomId, type);
  }

  joinRoom(roomId: string, client: Client, type: RoomType): Room {
    const room = this.getOrCreateRoom(roomId, type);
    room.clients.set(client.id, client);
    client.rooms.add(roomId);

    // Notify others in room
    this.broadcast(roomId, {
      type: 'participant:joined',
      payload: {
        clientId: client.id,
        userId: client.userId,
        soulId: client.soulId,
        timestamp: Date.now(),
      },
    }, client.id);

    logger.info({ roomId, clientId: client.id }, 'Client joined room');
    return room;
  }

  leaveRoom(roomId: string, clientId: string): void {
    const room = this.rooms.get(roomId);
    if (!room) return;

    const client = room.clients.get(clientId);
    if (client) {
      room.clients.delete(clientId);
      client.rooms.delete(roomId);

      // Notify others in room
      this.broadcast(roomId, {
        type: 'participant:left',
        payload: {
          clientId,
          timestamp: Date.now(),
        },
      });

      logger.info({ roomId, clientId }, 'Client left room');
    }

    // Clean up empty rooms
    if (room.clients.size === 0) {
      this.rooms.delete(roomId);
      logger.info({ roomId }, 'Room deleted (empty)');
    }
  }

  broadcast(roomId: string, message: unknown, excludeClientId?: string): void {
    const room = this.rooms.get(roomId);
    if (!room) return;

    const data = JSON.stringify(message);
    room.clients.forEach((client, id) => {
      if (id !== excludeClientId && client.ws.readyState === WebSocket.OPEN) {
        client.ws.send(data);
      }
    });
  }

  getRoomClients(roomId: string): Client[] {
    const room = this.rooms.get(roomId);
    return room ? Array.from(room.clients.values()) : [];
  }

  updateRoomState(roomId: string, updates: Record<string, unknown>): void {
    const room = this.rooms.get(roomId);
    if (room) {
      room.state = { ...room.state, ...updates };
    }
  }
}

// ============================================================================
// Message Handlers
// ============================================================================

class CircleHandler {
  constructor(private rooms: RoomManager) {}

  handle(client: Client, message: z.infer<typeof CircleMessageSchema>): void {
    switch (message.type) {
      case 'circle:join': {
        const roomId = `circle:${message.payload.circleId}`;
        const room = this.rooms.joinRoom(roomId, client, 'circle');

        // Send current state to new participant
        client.ws.send(JSON.stringify({
          type: 'circle:state',
          payload: {
            participants: this.rooms.getRoomClients(roomId).map(c => ({
              id: c.id,
              userId: c.userId,
              soulId: c.soulId,
            })),
            state: room.state,
          },
        }));
        break;
      }

      case 'circle:leave': {
        const roomId = `circle:${message.payload.circleId}`;
        this.rooms.leaveRoom(roomId, client.id);
        break;
      }

      case 'circle:sync': {
        // Host syncs playback state
        const roomId = Array.from(client.rooms).find(r => r.startsWith('circle:'));
        if (roomId) {
          this.rooms.updateRoomState(roomId, {
            position: message.payload.position,
            isPlaying: message.payload.isPlaying,
          });
          this.rooms.broadcast(roomId, {
            type: 'circle:sync',
            payload: message.payload,
          }, client.id);
        }
        break;
      }

      case 'circle:chat': {
        const roomId = Array.from(client.rooms).find(r => r.startsWith('circle:'));
        if (roomId) {
          this.rooms.broadcast(roomId, {
            type: 'circle:chat',
            payload: {
              ...message.payload,
              userId: client.userId,
              clientId: client.id,
              createdAt: new Date().toISOString(),
            },
          });
        }
        break;
      }

      case 'circle:reaction': {
        const roomId = Array.from(client.rooms).find(r => r.startsWith('circle:'));
        if (roomId) {
          this.rooms.broadcast(roomId, {
            type: 'circle:reaction',
            payload: {
              ...message.payload,
              userId: client.userId,
              id: nanoid(),
            },
          });
        }
        break;
      }
    }
  }
}

class StudioHandler {
  constructor(private rooms: RoomManager) {}

  handle(client: Client, message: z.infer<typeof StudioMessageSchema>): void {
    switch (message.type) {
      case 'studio:join': {
        const roomId = `studio:${message.payload.projectId}`;
        const room = this.rooms.joinRoom(roomId, client, 'studio');

        client.ws.send(JSON.stringify({
          type: 'studio:state',
          payload: {
            participants: this.rooms.getRoomClients(roomId).map(c => ({
              id: c.id,
              userId: c.userId,
              cursor: c.metadata.cursor,
            })),
            state: room.state,
          },
        }));
        break;
      }

      case 'studio:leave': {
        const roomId = `studio:${message.payload.projectId}`;
        this.rooms.leaveRoom(roomId, client.id);
        break;
      }

      case 'studio:cursor': {
        const roomId = Array.from(client.rooms).find(r => r.startsWith('studio:'));
        if (roomId) {
          client.metadata.cursor = message.payload;
          this.rooms.broadcast(roomId, {
            type: 'studio:cursor',
            payload: {
              ...message.payload,
              userId: client.userId,
              clientId: client.id,
            },
          }, client.id);
        }
        break;
      }

      case 'studio:stem:add':
      case 'studio:stem:update':
      case 'studio:stem:delete':
      case 'studio:stem:lock':
      case 'studio:stem:unlock': {
        const roomId = Array.from(client.rooms).find(r => r.startsWith('studio:'));
        if (roomId) {
          this.rooms.broadcast(roomId, {
            type: message.type,
            payload: {
              ...message.payload,
              userId: client.userId,
            },
          }, client.id);
        }
        break;
      }

      case 'studio:transport': {
        const roomId = Array.from(client.rooms).find(r => r.startsWith('studio:'));
        if (roomId) {
          this.rooms.updateRoomState(roomId, {
            transport: message.payload,
          });
          this.rooms.broadcast(roomId, {
            type: 'studio:transport',
            payload: message.payload,
          });
        }
        break;
      }
    }
  }
}

// ============================================================================
// WebSocket Server
// ============================================================================

class RealtimeServer {
  private wss: WebSocketServer;
  private clients: Map<string, Client> = new Map();
  private rooms: RoomManager;
  private redis: RedisPubSub;
  private circleHandler: CircleHandler;
  private studioHandler: StudioHandler;

  constructor(server: ReturnType<typeof createServer>) {
    this.redis = new RedisPubSub(config.redisUrl);
    this.rooms = new RoomManager(this.redis);
    this.circleHandler = new CircleHandler(this.rooms);
    this.studioHandler = new StudioHandler(this.rooms);

    this.wss = new WebSocketServer({ server, path: '/ws' });

    this.wss.on('connection', (ws, req) => {
      this.handleConnection(ws, req);
    });

    // Heartbeat interval
    setInterval(() => this.heartbeat(), 30000);

    logger.info('WebSocket server initialized');
  }

  private handleConnection(ws: WebSocket, req: any): void {
    const clientId = nanoid();
    const client: Client = {
      id: clientId,
      ws,
      rooms: new Set(),
      metadata: {},
      connectedAt: new Date(),
      lastPing: new Date(),
    };

    this.clients.set(clientId, client);
    logger.info({ clientId }, 'Client connected');

    // Send welcome message
    ws.send(JSON.stringify({
      type: 'connected',
      payload: { clientId },
    }));

    ws.on('message', (data) => {
      try {
        const message = JSON.parse(data.toString());
        this.handleMessage(client, message);
      } catch (err) {
        logger.error({ err, clientId }, 'Failed to parse message');
      }
    });

    ws.on('close', () => {
      this.handleDisconnect(client);
    });

    ws.on('error', (err) => {
      logger.error({ err, clientId }, 'WebSocket error');
    });

    ws.on('pong', () => {
      client.lastPing = new Date();
    });
  }

  private handleMessage(client: Client, message: unknown): void {
    const parsed = BaseMessageSchema.safeParse(message);
    if (!parsed.success) {
      logger.warn({ clientId: client.id }, 'Invalid message format');
      return;
    }

    const { type } = parsed.data;

    // Handle authentication
    if (type === 'auth') {
      this.handleAuth(client, parsed.data.payload as any);
      return;
    }

    // Handle ping
    if (type === 'ping') {
      client.lastPing = new Date();
      client.ws.send(JSON.stringify({ type: 'pong' }));
      return;
    }

    // Route to handlers
    if (type.startsWith('circle:')) {
      const circleMessage = CircleMessageSchema.safeParse(message);
      if (circleMessage.success) {
        this.circleHandler.handle(client, circleMessage.data);
      }
    } else if (type.startsWith('studio:')) {
      const studioMessage = StudioMessageSchema.safeParse(message);
      if (studioMessage.success) {
        this.studioHandler.handle(client, studioMessage.data);
      }
    }
  }

  private handleAuth(client: Client, payload: { token?: string; userId?: string; soulId?: string }): void {
    // In production, verify JWT token
    client.userId = payload.userId;
    client.soulId = payload.soulId;

    client.ws.send(JSON.stringify({
      type: 'auth:success',
      payload: { userId: client.userId, soulId: client.soulId },
    }));

    logger.info({ clientId: client.id, userId: client.userId }, 'Client authenticated');
  }

  private handleDisconnect(client: Client): void {
    // Leave all rooms
    client.rooms.forEach((roomId) => {
      this.rooms.leaveRoom(roomId, client.id);
    });

    this.clients.delete(client.id);
    logger.info({ clientId: client.id }, 'Client disconnected');
  }

  private heartbeat(): void {
    const now = Date.now();
    const timeout = 60000; // 60 seconds

    this.clients.forEach((client) => {
      if (now - client.lastPing.getTime() > timeout) {
        logger.info({ clientId: client.id }, 'Client timed out');
        client.ws.terminate();
        this.handleDisconnect(client);
      } else if (client.ws.readyState === WebSocket.OPEN) {
        client.ws.ping();
      }
    });
  }

  getStats() {
    return {
      clients: this.clients.size,
      rooms: this.rooms['rooms'].size,
    };
  }
}

// ============================================================================
// HTTP API
// ============================================================================

const app = new Hono();

app.use('/*', cors());

app.get('/health', (c) => c.json({ status: 'ok' }));

app.get('/stats', (c) => {
  const stats = (global as any).realtimeServer?.getStats() || {};
  return c.json(stats);
});

// ============================================================================
// Start Server
// ============================================================================

const httpServer = createServer(app.fetch);
const realtimeServer = new RealtimeServer(httpServer);
(global as any).realtimeServer = realtimeServer;

httpServer.listen(config.port, () => {
  logger.info({ port: config.port }, 'Realtime server started');
});
