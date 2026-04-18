// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Collaboration Service
 *
 * Real-time collaboration for the Creation Studio.
 * Uses WebSockets + CRDT for conflict-free concurrent editing.
 */

import { WebSocketServer, WebSocket } from 'ws';
import { createClient } from 'redis';
import { nanoid } from 'nanoid';

// Types
interface Participant {
  id: string;
  name: string;
  avatar?: string;
  color: string;
  cursor?: { stemId: string; position: number };
  joinedAt: Date;
}

interface ProjectRoom {
  projectId: string;
  participants: Map<string, { ws: WebSocket; participant: Participant }>;
  state: ProjectState;
}

interface ProjectState {
  stems: Map<string, StemState>;
  playhead: number;
  isPlaying: boolean;
  bpm: number;
  lastModified: Date;
}

interface StemState {
  id: string;
  name: string;
  type: string;
  volume: number;
  pan: number;
  muted: boolean;
  solo: boolean;
  startTime: number;
  lockedBy?: string;
}

interface CollaborationMessage {
  type: string;
  payload: any;
  senderId: string;
  timestamp: number;
}

// Collaboration colors for participants
const COLORS = [
  '#8B5CF6', '#EC4899', '#10B981', '#F59E0B',
  '#3B82F6', '#EF4444', '#06B6D4', '#84CC16'
];

class CollaborationServer {
  private wss: WebSocketServer;
  private redis: ReturnType<typeof createClient>;
  private rooms: Map<string, ProjectRoom> = new Map();
  private userColors: Map<string, string> = new Map();
  private colorIndex = 0;

  constructor(port: number) {
    this.wss = new WebSocketServer({ port });
    this.redis = createClient({ url: process.env.REDIS_URL });

    this.initialize();
  }

  private async initialize() {
    await this.redis.connect();

    // Subscribe to cross-server events
    const subscriber = this.redis.duplicate();
    await subscriber.connect();
    await subscriber.subscribe('collaboration:events', (message) => {
      this.handleRedisEvent(JSON.parse(message));
    });

    this.wss.on('connection', (ws, req) => {
      this.handleConnection(ws, req);
    });

    console.log('Collaboration server started');
  }

  private handleConnection(ws: WebSocket, req: any) {
    const url = new URL(req.url, 'ws://localhost');
    const projectId = url.searchParams.get('projectId');
    const userId = url.searchParams.get('userId');
    const userName = url.searchParams.get('userName') || 'Anonymous';

    if (!projectId || !userId) {
      ws.close(4000, 'Missing projectId or userId');
      return;
    }

    // Assign color
    if (!this.userColors.has(userId)) {
      this.userColors.set(userId, COLORS[this.colorIndex % COLORS.length]);
      this.colorIndex++;
    }

    const participant: Participant = {
      id: userId,
      name: userName,
      color: this.userColors.get(userId)!,
      joinedAt: new Date(),
    };

    // Join room
    this.joinRoom(projectId, ws, participant);

    // Handle messages
    ws.on('message', (data) => {
      try {
        const message: CollaborationMessage = JSON.parse(data.toString());
        this.handleMessage(projectId, userId, message);
      } catch (e) {
        console.error('Invalid message:', e);
      }
    });

    // Handle disconnect
    ws.on('close', () => {
      this.leaveRoom(projectId, userId);
    });
  }

  private joinRoom(projectId: string, ws: WebSocket, participant: Participant) {
    let room = this.rooms.get(projectId);

    if (!room) {
      room = {
        projectId,
        participants: new Map(),
        state: {
          stems: new Map(),
          playhead: 0,
          isPlaying: false,
          bpm: 120,
          lastModified: new Date(),
        },
      };
      this.rooms.set(projectId, room);
    }

    room.participants.set(participant.id, { ws, participant });

    // Send current state to new participant
    ws.send(JSON.stringify({
      type: 'room:joined',
      payload: {
        participants: Array.from(room.participants.values()).map(p => p.participant),
        state: this.serializeState(room.state),
      },
    }));

    // Notify others
    this.broadcast(projectId, {
      type: 'participant:joined',
      payload: participant,
      senderId: 'system',
      timestamp: Date.now(),
    }, participant.id);
  }

  private leaveRoom(projectId: string, userId: string) {
    const room = this.rooms.get(projectId);
    if (!room) return;

    const participant = room.participants.get(userId)?.participant;
    room.participants.delete(userId);

    // Unlock any stems locked by this user
    room.state.stems.forEach((stem) => {
      if (stem.lockedBy === userId) {
        stem.lockedBy = undefined;
      }
    });

    // Notify others
    if (participant) {
      this.broadcast(projectId, {
        type: 'participant:left',
        payload: { id: userId, name: participant.name },
        senderId: 'system',
        timestamp: Date.now(),
      });
    }

    // Clean up empty rooms
    if (room.participants.size === 0) {
      this.rooms.delete(projectId);
    }
  }

  private handleMessage(projectId: string, userId: string, message: CollaborationMessage) {
    const room = this.rooms.get(projectId);
    if (!room) return;

    switch (message.type) {
      case 'cursor:move':
        this.handleCursorMove(room, userId, message.payload);
        break;

      case 'stem:add':
        this.handleStemAdd(room, userId, message.payload);
        break;

      case 'stem:update':
        this.handleStemUpdate(room, userId, message.payload);
        break;

      case 'stem:delete':
        this.handleStemDelete(room, userId, message.payload);
        break;

      case 'stem:lock':
        this.handleStemLock(room, userId, message.payload);
        break;

      case 'stem:unlock':
        this.handleStemUnlock(room, userId, message.payload);
        break;

      case 'playhead:move':
        this.handlePlayheadMove(room, userId, message.payload);
        break;

      case 'transport:play':
      case 'transport:pause':
      case 'transport:stop':
        this.handleTransport(room, userId, message.type);
        break;

      case 'chat:message':
        this.handleChatMessage(room, userId, message.payload);
        break;

      case 'comment:add':
        this.handleCommentAdd(room, userId, message.payload);
        break;

      default:
        console.warn('Unknown message type:', message.type);
    }
  }

  private handleCursorMove(room: ProjectRoom, userId: string, payload: any) {
    const participant = room.participants.get(userId);
    if (!participant) return;

    participant.participant.cursor = payload;

    this.broadcast(room.projectId, {
      type: 'cursor:moved',
      payload: { userId, cursor: payload },
      senderId: userId,
      timestamp: Date.now(),
    }, userId);
  }

  private handleStemAdd(room: ProjectRoom, userId: string, payload: any) {
    const stemId = payload.id || nanoid();
    const stem: StemState = {
      id: stemId,
      name: payload.name,
      type: payload.type,
      volume: payload.volume ?? 1.0,
      pan: payload.pan ?? 0,
      muted: false,
      solo: false,
      startTime: payload.startTime ?? 0,
    };

    room.state.stems.set(stemId, stem);
    room.state.lastModified = new Date();

    this.broadcast(room.projectId, {
      type: 'stem:added',
      payload: { stem, addedBy: userId },
      senderId: userId,
      timestamp: Date.now(),
    });

    // Persist to Redis for recovery
    this.persistState(room);
  }

  private handleStemUpdate(room: ProjectRoom, userId: string, payload: any) {
    const stem = room.state.stems.get(payload.stemId);
    if (!stem) return;

    // Check lock
    if (stem.lockedBy && stem.lockedBy !== userId) {
      this.sendToUser(room, userId, {
        type: 'error',
        payload: { message: 'Stem is locked by another user' },
        senderId: 'system',
        timestamp: Date.now(),
      });
      return;
    }

    // Apply updates
    Object.assign(stem, payload.updates);
    room.state.lastModified = new Date();

    this.broadcast(room.projectId, {
      type: 'stem:updated',
      payload: { stemId: payload.stemId, updates: payload.updates, updatedBy: userId },
      senderId: userId,
      timestamp: Date.now(),
    }, userId);

    this.persistState(room);
  }

  private handleStemDelete(room: ProjectRoom, userId: string, payload: any) {
    const stem = room.state.stems.get(payload.stemId);
    if (!stem) return;

    if (stem.lockedBy && stem.lockedBy !== userId) {
      this.sendToUser(room, userId, {
        type: 'error',
        payload: { message: 'Stem is locked by another user' },
        senderId: 'system',
        timestamp: Date.now(),
      });
      return;
    }

    room.state.stems.delete(payload.stemId);
    room.state.lastModified = new Date();

    this.broadcast(room.projectId, {
      type: 'stem:deleted',
      payload: { stemId: payload.stemId, deletedBy: userId },
      senderId: userId,
      timestamp: Date.now(),
    });

    this.persistState(room);
  }

  private handleStemLock(room: ProjectRoom, userId: string, payload: any) {
    const stem = room.state.stems.get(payload.stemId);
    if (!stem) return;

    if (stem.lockedBy && stem.lockedBy !== userId) {
      this.sendToUser(room, userId, {
        type: 'error',
        payload: { message: 'Stem is already locked' },
        senderId: 'system',
        timestamp: Date.now(),
      });
      return;
    }

    stem.lockedBy = userId;

    this.broadcast(room.projectId, {
      type: 'stem:locked',
      payload: { stemId: payload.stemId, lockedBy: userId },
      senderId: userId,
      timestamp: Date.now(),
    });
  }

  private handleStemUnlock(room: ProjectRoom, userId: string, payload: any) {
    const stem = room.state.stems.get(payload.stemId);
    if (!stem) return;

    if (stem.lockedBy === userId) {
      stem.lockedBy = undefined;

      this.broadcast(room.projectId, {
        type: 'stem:unlocked',
        payload: { stemId: payload.stemId },
        senderId: userId,
        timestamp: Date.now(),
      });
    }
  }

  private handlePlayheadMove(room: ProjectRoom, userId: string, payload: any) {
    room.state.playhead = payload.position;

    this.broadcast(room.projectId, {
      type: 'playhead:moved',
      payload: { position: payload.position, movedBy: userId },
      senderId: userId,
      timestamp: Date.now(),
    }, userId);
  }

  private handleTransport(room: ProjectRoom, userId: string, type: string) {
    switch (type) {
      case 'transport:play':
        room.state.isPlaying = true;
        break;
      case 'transport:pause':
        room.state.isPlaying = false;
        break;
      case 'transport:stop':
        room.state.isPlaying = false;
        room.state.playhead = 0;
        break;
    }

    this.broadcast(room.projectId, {
      type,
      payload: { triggeredBy: userId },
      senderId: userId,
      timestamp: Date.now(),
    });
  }

  private handleChatMessage(room: ProjectRoom, userId: string, payload: any) {
    const participant = room.participants.get(userId);
    if (!participant) return;

    this.broadcast(room.projectId, {
      type: 'chat:message',
      payload: {
        id: nanoid(),
        userId,
        userName: participant.participant.name,
        userColor: participant.participant.color,
        content: payload.content,
        timestamp: Date.now(),
      },
      senderId: userId,
      timestamp: Date.now(),
    });
  }

  private handleCommentAdd(room: ProjectRoom, userId: string, payload: any) {
    const participant = room.participants.get(userId);
    if (!participant) return;

    this.broadcast(room.projectId, {
      type: 'comment:added',
      payload: {
        id: nanoid(),
        userId,
        userName: participant.participant.name,
        userColor: participant.participant.color,
        content: payload.content,
        timestamp: payload.timestamp, // Position in track
        stemId: payload.stemId,
        createdAt: Date.now(),
      },
      senderId: userId,
      timestamp: Date.now(),
    });
  }

  private broadcast(projectId: string, message: CollaborationMessage, excludeUserId?: string) {
    const room = this.rooms.get(projectId);
    if (!room) return;

    const data = JSON.stringify(message);

    room.participants.forEach(({ ws }, oderId) => {
      if (excludeUserId && oderId === excludeUserId) return;
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(data);
      }
    });

    // Publish to Redis for cross-server sync
    this.redis.publish('collaboration:events', JSON.stringify({
      projectId,
      message,
      excludeUserId,
    }));
  }

  private sendToUser(room: ProjectRoom, userId: string, message: CollaborationMessage) {
    const participant = room.participants.get(userId);
    if (participant && participant.ws.readyState === WebSocket.OPEN) {
      participant.ws.send(JSON.stringify(message));
    }
  }

  private handleRedisEvent(event: any) {
    // Handle events from other servers
    // (for horizontal scaling)
  }

  private serializeState(state: ProjectState): any {
    return {
      stems: Array.from(state.stems.values()),
      playhead: state.playhead,
      isPlaying: state.isPlaying,
      bpm: state.bpm,
      lastModified: state.lastModified.toISOString(),
    };
  }

  private async persistState(room: ProjectRoom) {
    const key = `project:${room.projectId}:state`;
    await this.redis.set(key, JSON.stringify(this.serializeState(room.state)));
  }
}

// Start server
const port = parseInt(process.env.COLLABORATION_PORT || '8081');
new CollaborationServer(port);
