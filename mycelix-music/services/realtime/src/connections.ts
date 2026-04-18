// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * WebSocket Connection Manager
 *
 * Manages active connections, rooms/channels, and user mappings.
 */

import { WebSocket } from 'ws';
import { createLogger } from './logger';
import { config } from './config';

const logger = createLogger('connections');

interface Connection {
  id: string;
  ws: WebSocket;
  userId?: string;
  rooms: Set<string>;
  lastHeartbeat: number;
  metadata: Record<string, unknown>;
}

export class ConnectionManager {
  private connections: Map<string, Connection> = new Map();
  private userConnections: Map<string, Set<string>> = new Map();
  private rooms: Map<string, Set<string>> = new Map();
  private connectionCounter = 0;

  /**
   * Add a new WebSocket connection
   */
  addConnection(ws: WebSocket): string {
    const id = `conn_${Date.now()}_${++this.connectionCounter}`;

    const connection: Connection = {
      id,
      ws,
      rooms: new Set(),
      lastHeartbeat: Date.now(),
      metadata: {},
    };

    this.connections.set(id, connection);
    logger.debug({ connectionId: id }, 'Connection added');

    return id;
  }

  /**
   * Remove a connection and clean up all associations
   */
  removeConnection(connectionId: string): void {
    const connection = this.connections.get(connectionId);
    if (!connection) return;

    // Remove from user connections
    if (connection.userId) {
      const userConns = this.userConnections.get(connection.userId);
      if (userConns) {
        userConns.delete(connectionId);
        if (userConns.size === 0) {
          this.userConnections.delete(connection.userId);
        }
      }
    }

    // Remove from all rooms
    for (const room of connection.rooms) {
      this.leaveRoom(connectionId, room);
    }

    // Close WebSocket if still open
    if (connection.ws.readyState === WebSocket.OPEN) {
      connection.ws.close();
    }

    this.connections.delete(connectionId);
    logger.debug({ connectionId }, 'Connection removed');
  }

  /**
   * Associate a connection with a user
   */
  setUser(connectionId: string, userId: string): void {
    const connection = this.connections.get(connectionId);
    if (!connection) return;

    // Remove from old user mapping if exists
    if (connection.userId && connection.userId !== userId) {
      const oldUserConns = this.userConnections.get(connection.userId);
      if (oldUserConns) {
        oldUserConns.delete(connectionId);
        if (oldUserConns.size === 0) {
          this.userConnections.delete(connection.userId);
        }
      }
    }

    connection.userId = userId;

    // Add to new user mapping
    if (!this.userConnections.has(userId)) {
      this.userConnections.set(userId, new Set());
    }
    this.userConnections.get(userId)!.add(connectionId);

    logger.debug({ connectionId, userId }, 'User set for connection');
  }

  /**
   * Get connection by ID
   */
  getConnection(connectionId: string): Connection | undefined {
    return this.connections.get(connectionId);
  }

  /**
   * Get all connections for a user
   */
  getUserConnections(userId: string): Connection[] {
    const connectionIds = this.userConnections.get(userId);
    if (!connectionIds) return [];

    return Array.from(connectionIds)
      .map(id => this.connections.get(id))
      .filter((c): c is Connection => c !== undefined);
  }

  /**
   * Join a room/channel
   */
  joinRoom(connectionId: string, room: string): void {
    const connection = this.connections.get(connectionId);
    if (!connection) return;

    connection.rooms.add(room);

    if (!this.rooms.has(room)) {
      this.rooms.set(room, new Set());
    }
    this.rooms.get(room)!.add(connectionId);

    logger.debug({ connectionId, room }, 'Joined room');
  }

  /**
   * Leave a room/channel
   */
  leaveRoom(connectionId: string, room: string): void {
    const connection = this.connections.get(connectionId);
    if (connection) {
      connection.rooms.delete(room);
    }

    const roomConnections = this.rooms.get(room);
    if (roomConnections) {
      roomConnections.delete(connectionId);
      if (roomConnections.size === 0) {
        this.rooms.delete(room);
      }
    }

    logger.debug({ connectionId, room }, 'Left room');
  }

  /**
   * Get all connections in a room
   */
  getRoomConnections(room: string): Connection[] {
    const connectionIds = this.rooms.get(room);
    if (!connectionIds) return [];

    return Array.from(connectionIds)
      .map(id => this.connections.get(id))
      .filter((c): c is Connection => c !== undefined);
  }

  /**
   * Send message to a specific connection
   */
  send(connectionId: string, message: unknown): boolean {
    const connection = this.connections.get(connectionId);
    if (!connection || connection.ws.readyState !== WebSocket.OPEN) {
      return false;
    }

    try {
      connection.ws.send(JSON.stringify(message));
      return true;
    } catch (error) {
      logger.error({ connectionId, error }, 'Failed to send message');
      return false;
    }
  }

  /**
   * Send message to all connections of a user
   */
  sendToUser(userId: string, message: unknown): number {
    const connections = this.getUserConnections(userId);
    let sent = 0;

    for (const connection of connections) {
      if (this.send(connection.id, message)) {
        sent++;
      }
    }

    return sent;
  }

  /**
   * Broadcast message to all connections in a room
   */
  broadcastToRoom(room: string, message: unknown, excludeConnectionId?: string): number {
    const connections = this.getRoomConnections(room);
    let sent = 0;

    for (const connection of connections) {
      if (connection.id !== excludeConnectionId) {
        if (this.send(connection.id, message)) {
          sent++;
        }
      }
    }

    return sent;
  }

  /**
   * Broadcast to all connections
   */
  broadcast(message: unknown, excludeConnectionId?: string): number {
    let sent = 0;

    for (const [id, connection] of this.connections) {
      if (id !== excludeConnectionId) {
        if (this.send(id, message)) {
          sent++;
        }
      }
    }

    return sent;
  }

  /**
   * Update heartbeat timestamp for connection
   */
  updateHeartbeat(connectionId: string): void {
    const connection = this.connections.get(connectionId);
    if (connection) {
      connection.lastHeartbeat = Date.now();
    }
  }

  /**
   * Check for stale connections and remove them
   */
  checkHeartbeats(): void {
    const now = Date.now();
    const timeout = config.heartbeat.timeout;
    const staleConnections: string[] = [];

    for (const [id, connection] of this.connections) {
      if (now - connection.lastHeartbeat > timeout) {
        staleConnections.push(id);
      }
    }

    for (const id of staleConnections) {
      logger.info({ connectionId: id }, 'Removing stale connection');
      this.removeConnection(id);
    }

    if (staleConnections.length > 0) {
      logger.info({ count: staleConnections.length }, 'Removed stale connections');
    }
  }

  /**
   * Close all connections
   */
  closeAll(): void {
    for (const [id] of this.connections) {
      this.removeConnection(id);
    }
    logger.info('All connections closed');
  }

  /**
   * Get total connection count
   */
  getConnectionCount(): number {
    return this.connections.size;
  }

  /**
   * Get room statistics
   */
  getRoomStats(): Record<string, number> {
    const stats: Record<string, number> = {};
    for (const [room, connections] of this.rooms) {
      stats[room] = connections.size;
    }
    return stats;
  }

  /**
   * Get online users count
   */
  getOnlineUsersCount(): number {
    return this.userConnections.size;
  }

  /**
   * Check if user is online
   */
  isUserOnline(userId: string): boolean {
    const connections = this.userConnections.get(userId);
    return connections !== undefined && connections.size > 0;
  }

  /**
   * Set metadata for a connection
   */
  setMetadata(connectionId: string, key: string, value: unknown): void {
    const connection = this.connections.get(connectionId);
    if (connection) {
      connection.metadata[key] = value;
    }
  }

  /**
   * Get metadata for a connection
   */
  getMetadata(connectionId: string, key: string): unknown {
    const connection = this.connections.get(connectionId);
    return connection?.metadata[key];
  }
}
