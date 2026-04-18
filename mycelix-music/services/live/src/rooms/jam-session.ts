// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Jam Session Room
 *
 * Real-time collaborative music creation with:
 * - Low-latency audio streaming
 * - Synchronized playback
 * - Multi-track recording
 * - Live effects processing
 * - Chat and reactions
 */

import { Server, Socket } from 'socket.io';
import { v4 as uuidv4 } from 'uuid';
import Redis from 'ioredis';

// ============================================================================
// Types
// ============================================================================

export interface JamSession {
  id: string;
  name: string;
  hostId: string;
  hostSoulId: string;
  createdAt: Date;
  status: 'waiting' | 'jamming' | 'recording' | 'ended';
  bpm: number;
  key: string;
  scale: 'major' | 'minor';
  maxParticipants: number;
  isPublic: boolean;
  participants: Map<string, Participant>;
  tracks: SessionTrack[];
  chatMessages: ChatMessage[];
  settings: SessionSettings;
}

export interface Participant {
  id: string;
  soulId: string;
  displayName: string;
  avatarUri?: string;
  instrument: string;
  role: 'host' | 'performer' | 'listener';
  isMuted: boolean;
  isDeafened: boolean;
  joinedAt: Date;
  latency: number;
  audioLevel: number;
}

export interface SessionTrack {
  id: string;
  participantId: string;
  instrument: string;
  startTime: number;
  duration: number;
  audioData?: ArrayBuffer;
  isLive: boolean;
}

export interface ChatMessage {
  id: string;
  participantId: string;
  content: string;
  timestamp: Date;
  type: 'text' | 'reaction' | 'system';
}

export interface SessionSettings {
  metronomeEnabled: boolean;
  metronomVolume: number;
  countIn: number; // bars
  loopLength: number; // bars (0 = no loop)
  quantize: boolean;
  latencyCompensation: boolean;
}

interface AudioFrame {
  participantId: string;
  timestamp: number;
  data: ArrayBuffer;
  sequence: number;
}

// ============================================================================
// Jam Session Manager
// ============================================================================

export class JamSessionManager {
  private io: Server;
  private redis: Redis;
  private sessions: Map<string, JamSession> = new Map();
  private participantToSession: Map<string, string> = new Map();

  // Audio buffer for sync
  private audioBuffers: Map<string, AudioFrame[]> = new Map();
  private bufferWindow = 100; // ms

  constructor(io: Server, redis: Redis) {
    this.io = io;
    this.redis = redis;
    this.setupSocketHandlers();
  }

  private setupSocketHandlers() {
    this.io.of('/jam').on('connection', (socket: Socket) => {
      console.log('Jam session connection:', socket.id);

      socket.on('create-session', (data, callback) => this.handleCreateSession(socket, data, callback));
      socket.on('join-session', (data, callback) => this.handleJoinSession(socket, data, callback));
      socket.on('leave-session', () => this.handleLeaveSession(socket));
      socket.on('audio-frame', (data) => this.handleAudioFrame(socket, data));
      socket.on('update-settings', (data) => this.handleUpdateSettings(socket, data));
      socket.on('toggle-mute', () => this.handleToggleMute(socket));
      socket.on('start-recording', () => this.handleStartRecording(socket));
      socket.on('stop-recording', () => this.handleStopRecording(socket));
      socket.on('chat-message', (data) => this.handleChatMessage(socket, data));
      socket.on('reaction', (data) => this.handleReaction(socket, data));
      socket.on('latency-ping', (timestamp) => this.handleLatencyPing(socket, timestamp));
      socket.on('disconnect', () => this.handleDisconnect(socket));
    });
  }

  // ============================================================================
  // Session Management
  // ============================================================================

  private async handleCreateSession(
    socket: Socket,
    data: {
      name: string;
      soulId: string;
      displayName: string;
      bpm: number;
      key: string;
      scale: 'major' | 'minor';
      instrument: string;
      isPublic: boolean;
      maxParticipants: number;
    },
    callback: (response: { success: boolean; sessionId?: string; error?: string }) => void
  ) {
    try {
      const sessionId = uuidv4();

      const session: JamSession = {
        id: sessionId,
        name: data.name,
        hostId: socket.id,
        hostSoulId: data.soulId,
        createdAt: new Date(),
        status: 'waiting',
        bpm: data.bpm || 120,
        key: data.key || 'C',
        scale: data.scale || 'major',
        maxParticipants: data.maxParticipants || 8,
        isPublic: data.isPublic,
        participants: new Map(),
        tracks: [],
        chatMessages: [],
        settings: {
          metronomeEnabled: true,
          metronomVolume: 0.5,
          countIn: 1,
          loopLength: 4,
          quantize: true,
          latencyCompensation: true,
        },
      };

      // Add host as first participant
      const host: Participant = {
        id: socket.id,
        soulId: data.soulId,
        displayName: data.displayName,
        instrument: data.instrument,
        role: 'host',
        isMuted: false,
        isDeafened: false,
        joinedAt: new Date(),
        latency: 0,
        audioLevel: 0,
      };

      session.participants.set(socket.id, host);
      this.sessions.set(sessionId, session);
      this.participantToSession.set(socket.id, sessionId);

      socket.join(sessionId);

      // Store in Redis for discovery
      await this.redis.hset(`jam:sessions`, sessionId, JSON.stringify({
        id: sessionId,
        name: session.name,
        hostName: data.displayName,
        bpm: session.bpm,
        key: session.key,
        participantCount: 1,
        maxParticipants: session.maxParticipants,
        isPublic: session.isPublic,
        createdAt: session.createdAt,
      }));

      callback({ success: true, sessionId });

      // Notify for public sessions
      if (session.isPublic) {
        this.io.of('/jam').emit('session-created', {
          id: sessionId,
          name: session.name,
          hostName: data.displayName,
          bpm: session.bpm,
          key: session.key,
        });
      }
    } catch (error) {
      callback({ success: false, error: (error as Error).message });
    }
  }

  private async handleJoinSession(
    socket: Socket,
    data: {
      sessionId: string;
      soulId: string;
      displayName: string;
      instrument: string;
      role: 'performer' | 'listener';
    },
    callback: (response: { success: boolean; session?: unknown; error?: string }) => void
  ) {
    const session = this.sessions.get(data.sessionId);

    if (!session) {
      callback({ success: false, error: 'Session not found' });
      return;
    }

    if (session.participants.size >= session.maxParticipants) {
      callback({ success: false, error: 'Session is full' });
      return;
    }

    const participant: Participant = {
      id: socket.id,
      soulId: data.soulId,
      displayName: data.displayName,
      instrument: data.instrument,
      role: data.role,
      isMuted: data.role === 'listener',
      isDeafened: false,
      joinedAt: new Date(),
      latency: 0,
      audioLevel: 0,
    };

    session.participants.set(socket.id, participant);
    this.participantToSession.set(socket.id, data.sessionId);

    socket.join(data.sessionId);

    // Notify other participants
    socket.to(data.sessionId).emit('participant-joined', {
      id: socket.id,
      displayName: data.displayName,
      instrument: data.instrument,
      role: data.role,
    });

    // Send current session state
    callback({
      success: true,
      session: {
        id: session.id,
        name: session.name,
        bpm: session.bpm,
        key: session.key,
        scale: session.scale,
        status: session.status,
        settings: session.settings,
        participants: Array.from(session.participants.values()).map((p) => ({
          id: p.id,
          displayName: p.displayName,
          instrument: p.instrument,
          role: p.role,
          isMuted: p.isMuted,
        })),
      },
    });

    // Add system message
    this.addSystemMessage(session, `${data.displayName} joined the session`);
  }

  private handleLeaveSession(socket: Socket) {
    const sessionId = this.participantToSession.get(socket.id);
    if (!sessionId) return;

    const session = this.sessions.get(sessionId);
    if (!session) return;

    const participant = session.participants.get(socket.id);
    if (!participant) return;

    session.participants.delete(socket.id);
    this.participantToSession.delete(socket.id);
    socket.leave(sessionId);

    // Notify others
    socket.to(sessionId).emit('participant-left', { id: socket.id });
    this.addSystemMessage(session, `${participant.displayName} left the session`);

    // If host left, transfer or end
    if (participant.role === 'host') {
      const nextHost = Array.from(session.participants.values())
        .find((p) => p.role === 'performer');

      if (nextHost) {
        nextHost.role = 'host';
        session.hostId = nextHost.id;
        session.hostSoulId = nextHost.soulId;

        this.io.of('/jam').to(sessionId).emit('host-changed', {
          newHostId: nextHost.id,
          newHostName: nextHost.displayName,
        });
      } else if (session.participants.size === 0) {
        this.endSession(sessionId);
      }
    }
  }

  // ============================================================================
  // Audio Streaming
  // ============================================================================

  private handleAudioFrame(socket: Socket, data: { timestamp: number; data: ArrayBuffer; sequence: number }) {
    const sessionId = this.participantToSession.get(socket.id);
    if (!sessionId) return;

    const session = this.sessions.get(sessionId);
    if (!session) return;

    const participant = session.participants.get(socket.id);
    if (!participant || participant.isMuted) return;

    const frame: AudioFrame = {
      participantId: socket.id,
      timestamp: data.timestamp,
      data: data.data,
      sequence: data.sequence,
    };

    // Add to buffer for sync
    const bufferKey = `${sessionId}:${socket.id}`;
    let buffer = this.audioBuffers.get(bufferKey);
    if (!buffer) {
      buffer = [];
      this.audioBuffers.set(bufferKey, buffer);
    }
    buffer.push(frame);

    // Clean old frames
    const cutoff = Date.now() - this.bufferWindow * 2;
    this.audioBuffers.set(bufferKey, buffer.filter((f) => f.timestamp > cutoff));

    // Broadcast to other participants with latency compensation
    for (const [pid, p] of session.participants) {
      if (pid !== socket.id && !p.isDeafened) {
        const targetSocket = this.io.of('/jam').sockets.get(pid);
        if (targetSocket) {
          // Apply latency compensation
          const compensatedTimestamp = session.settings.latencyCompensation
            ? data.timestamp + (p.latency / 2)
            : data.timestamp;

          targetSocket.emit('audio-frame', {
            participantId: socket.id,
            timestamp: compensatedTimestamp,
            data: data.data,
            sequence: data.sequence,
          });
        }
      }
    }

    // Update audio level for visualization
    participant.audioLevel = this.calculateAudioLevel(data.data);
  }

  private calculateAudioLevel(audioData: ArrayBuffer): number {
    // Simplified RMS calculation
    const view = new Float32Array(audioData);
    let sum = 0;
    for (let i = 0; i < view.length; i++) {
      sum += view[i] * view[i];
    }
    return Math.sqrt(sum / view.length);
  }

  // ============================================================================
  // Recording
  // ============================================================================

  private handleStartRecording(socket: Socket) {
    const sessionId = this.participantToSession.get(socket.id);
    if (!sessionId) return;

    const session = this.sessions.get(sessionId);
    if (!session) return;

    const participant = session.participants.get(socket.id);
    if (participant?.role !== 'host') return;

    session.status = 'recording';

    // Broadcast recording state
    this.io.of('/jam').to(sessionId).emit('recording-started', {
      startTime: Date.now(),
    });

    this.addSystemMessage(session, 'Recording started');
  }

  private async handleStopRecording(socket: Socket) {
    const sessionId = this.participantToSession.get(socket.id);
    if (!sessionId) return;

    const session = this.sessions.get(sessionId);
    if (!session) return;

    const participant = session.participants.get(socket.id);
    if (participant?.role !== 'host') return;

    session.status = 'jamming';

    // Collect all recorded tracks
    // In production, this would merge audio streams

    this.io.of('/jam').to(sessionId).emit('recording-stopped', {
      trackCount: session.tracks.length,
    });

    this.addSystemMessage(session, 'Recording stopped');
  }

  // ============================================================================
  // Settings & Controls
  // ============================================================================

  private handleUpdateSettings(
    socket: Socket,
    data: Partial<SessionSettings> & { bpm?: number; key?: string }
  ) {
    const sessionId = this.participantToSession.get(socket.id);
    if (!sessionId) return;

    const session = this.sessions.get(sessionId);
    if (!session) return;

    const participant = session.participants.get(socket.id);
    if (participant?.role !== 'host') return;

    if (data.bpm) session.bpm = data.bpm;
    if (data.key) session.key = data.key;

    Object.assign(session.settings, data);

    this.io.of('/jam').to(sessionId).emit('settings-updated', {
      bpm: session.bpm,
      key: session.key,
      settings: session.settings,
    });
  }

  private handleToggleMute(socket: Socket) {
    const sessionId = this.participantToSession.get(socket.id);
    if (!sessionId) return;

    const session = this.sessions.get(sessionId);
    if (!session) return;

    const participant = session.participants.get(socket.id);
    if (!participant) return;

    participant.isMuted = !participant.isMuted;

    socket.to(sessionId).emit('participant-mute-changed', {
      id: socket.id,
      isMuted: participant.isMuted,
    });
  }

  // ============================================================================
  // Chat & Reactions
  // ============================================================================

  private handleChatMessage(socket: Socket, data: { content: string }) {
    const sessionId = this.participantToSession.get(socket.id);
    if (!sessionId) return;

    const session = this.sessions.get(sessionId);
    if (!session) return;

    const participant = session.participants.get(socket.id);
    if (!participant) return;

    const message: ChatMessage = {
      id: uuidv4(),
      participantId: socket.id,
      content: data.content,
      timestamp: new Date(),
      type: 'text',
    };

    session.chatMessages.push(message);

    this.io.of('/jam').to(sessionId).emit('chat-message', {
      ...message,
      displayName: participant.displayName,
    });
  }

  private handleReaction(socket: Socket, data: { emoji: string }) {
    const sessionId = this.participantToSession.get(socket.id);
    if (!sessionId) return;

    const session = this.sessions.get(sessionId);
    if (!session) return;

    const participant = session.participants.get(socket.id);
    if (!participant) return;

    this.io.of('/jam').to(sessionId).emit('reaction', {
      participantId: socket.id,
      displayName: participant.displayName,
      emoji: data.emoji,
    });
  }

  // ============================================================================
  // Latency Management
  // ============================================================================

  private handleLatencyPing(socket: Socket, timestamp: number) {
    const sessionId = this.participantToSession.get(socket.id);
    if (!sessionId) return;

    const session = this.sessions.get(sessionId);
    if (!session) return;

    const participant = session.participants.get(socket.id);
    if (!participant) return;

    const latency = Date.now() - timestamp;
    participant.latency = latency;

    socket.emit('latency-pong', { latency });
  }

  // ============================================================================
  // Helpers
  // ============================================================================

  private addSystemMessage(session: JamSession, content: string) {
    const message: ChatMessage = {
      id: uuidv4(),
      participantId: 'system',
      content,
      timestamp: new Date(),
      type: 'system',
    };

    session.chatMessages.push(message);

    this.io.of('/jam').to(session.id).emit('chat-message', {
      ...message,
      displayName: 'System',
    });
  }

  private async endSession(sessionId: string) {
    const session = this.sessions.get(sessionId);
    if (!session) return;

    session.status = 'ended';

    // Remove from Redis
    await this.redis.hdel('jam:sessions', sessionId);

    // Notify all remaining participants
    this.io.of('/jam').to(sessionId).emit('session-ended');

    // Clean up
    this.sessions.delete(sessionId);

    // Clear audio buffers
    for (const [key] of this.audioBuffers) {
      if (key.startsWith(sessionId)) {
        this.audioBuffers.delete(key);
      }
    }
  }

  private handleDisconnect(socket: Socket) {
    this.handleLeaveSession(socket);
  }

  // ============================================================================
  // Public API
  // ============================================================================

  async getPublicSessions(): Promise<unknown[]> {
    const sessions = await this.redis.hgetall('jam:sessions');
    return Object.values(sessions)
      .map((s) => JSON.parse(s))
      .filter((s) => s.isPublic);
  }

  getSession(sessionId: string): JamSession | undefined {
    return this.sessions.get(sessionId);
  }
}
