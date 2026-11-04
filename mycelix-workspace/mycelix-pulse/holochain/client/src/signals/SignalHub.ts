// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Signal Hub - Unified signal processing for all Holochain zomes
 * Aggregates, filters, and dispatches real-time events
 */

import type { AppClient, AppSignal, AgentPubKey } from '@holochain/client';
import type {
  MycelixSignal,
  MessageSignal,
  TrustSignal,
  SyncSignal,
  FederationSignal,
} from '../types';

export type SignalHandler<T> = (signal: T) => void | Promise<void>;
export type SignalFilter = (signal: MycelixSignal) => boolean;

interface SignalSubscription {
  id: string;
  handler: SignalHandler<MycelixSignal>;
  filter?: SignalFilter;
  zomeFilter?: string[];
}

export class SignalHub {
  private subscriptions: Map<string, SignalSubscription> = new Map();
  private messageHandlers: Set<SignalHandler<MessageSignal>> = new Set();
  private trustHandlers: Set<SignalHandler<TrustSignal>> = new Set();
  private syncHandlers: Set<SignalHandler<SyncSignal>> = new Set();
  private federationHandlers: Set<SignalHandler<FederationSignal>> = new Set();
  private errorHandlers: Set<(error: Error, signal: unknown) => void> = new Set();

  private isConnected = false;
  private signalBuffer: MycelixSignal[] = [];
  private bufferSize = 100;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 5;

  constructor(private client: AppClient) {
    this.setupSignalHandler();
  }

  private setupSignalHandler(): void {
    this.client.on('signal', (signal: AppSignal) => {
      try {
        const parsed = this.parseSignal(signal);
        if (parsed) {
          this.dispatch(parsed);
        }
      } catch (error) {
        this.handleError(error as Error, signal);
      }
    });
    this.isConnected = true;
  }

  private parseSignal(appSignal: AppSignal): MycelixSignal | null {
    const payload = appSignal.payload as unknown;
    if (!payload || typeof payload !== 'object' || Array.isArray(payload)) {
      return null;
    }

    const envelope = payload as { type?: unknown; data?: unknown };
    if (typeof envelope.type !== 'string') {
      return null;
    }

    const type = envelope.type;
    const unitVariant = type === 'Online' || type === 'Offline';
    let data: Record<string, unknown>;
    if (envelope.data === undefined || envelope.data === null) {
      // Serde omits the content field for unit variants in the adjacent-tag format.
      if (!unitVariant) return null;
      data = {};
    } else if (
      typeof envelope.data === 'object' &&
      !Array.isArray(envelope.data)
    ) {
      data = envelope.data as Record<string, unknown>;
    } else {
      return null;
    }

    // Exact field sets are part of the wire contract. This rejects both
    // missing required fields and unrecognized additions before a typed signal
    // reaches application handlers.
    const fieldsByType: Record<string, readonly string[]> = {
      EmailReceived: ['email_hash', 'sender', 'encrypted_subject', 'timestamp', 'priority'],
      DeliveryConfirmed: ['email_hash', 'recipient', 'delivered_at'],
      ReadReceiptReceived: ['email_hash', 'reader', 'read_at'],
      EmailStateChanged: ['email_hash', 'state'],
      ThreadActivity: ['thread_id', 'email_hash', 'sender'],
      DraftSaved: ['draft_hash'],
      ScheduledEmailSent: ['email_hash', 'recipients'],
      TypingIndicator: ['sender', 'thread_id'],
      AttestationReceived: ['attestation_hash', 'truster', 'trust_level', 'category'],
      TrustScoreUpdated: ['agent', 'new_score', 'confidence'],
      IntroductionReceived: ['introduction_hash', 'introducer', 'introduced', 'recommendation'],
      DisputeFiled: ['dispute_hash', 'attestation_hash', 'disputer'],
      ByzantineDetected: ['agent', 'flag_type', 'severity'],
      SyncStateChanged: ['vector_clock', 'pending_ops'],
      OperationReceived: ['op_id', 'op_type', 'from_agent'],
      ConflictDetected: ['conflict_id', 'conflict_type'],
      ConflictResolved: ['conflict_id', 'resolution'],
      Online: [],
      Offline: [],
      SyncProgress: ['synced', 'total', 'peer'],
      SyncComplete: ['peer', 'ops_synced'],
      NetworkDiscovered: ['network_id', 'network_type'],
      EnvelopeReceived: ['envelope_id', 'from_network'],
      EnvelopeDelivered: ['envelope_id', 'to_network'],
      DeliveryFailed: ['envelope_id', 'error'],
      BridgeStatusChanged: ['bridge_agent', 'is_active'],
      RouteChanged: ['route_id', 'is_active'],
      PeerStatusChanged: ['network_id', 'is_reachable'],
    };
    const expectedFields = fieldsByType[type];
    if (!expectedFields) return null;
    const actualFields = Object.keys(data);
    if (
      actualFields.length !== expectedFields.length ||
      !expectedFields.every((field) => Object.prototype.hasOwnProperty.call(data, field))
    ) {
      return null;
    }

    if (type === 'EmailStateChanged') {
      const state = data.state as unknown;
      if (!state || typeof state !== 'object' || Array.isArray(state)) return null;
      const stateFields = ['is_read', 'is_starred', 'is_archived', 'is_trashed', 'folders'];
      const stateRecord = state as Record<string, unknown>;
      if (
        Object.keys(stateRecord).length !== stateFields.length ||
        !stateFields.every((field) => Object.prototype.hasOwnProperty.call(stateRecord, field)) ||
        !['is_read', 'is_starred', 'is_archived', 'is_trashed'].every(
          (field) => stateRecord[field] === null || typeof stateRecord[field] === 'boolean'
        ) ||
        !(stateRecord.folders === null || Array.isArray(stateRecord.folders))
      ) {
        return null;
      }
    }

    const numericFields: Record<string, readonly string[]> = {
      TrustScoreUpdated: ['new_score', 'confidence'],
      AttestationReceived: ['trust_level'],
      IntroductionReceived: ['recommendation'],
      ByzantineDetected: ['severity'],
      SyncStateChanged: ['pending_ops'],
      SyncProgress: ['synced', 'total'],
      SyncComplete: ['ops_synced'],
    };
    for (const field of numericFields[type] ?? []) {
      if (typeof data[field] !== 'number' || !Number.isFinite(data[field] as number)) {
        return null;
      }
    }

    // Normalize the validated Rust type/data v1 envelope to the flat union
    // used by current consumers. The outer discriminator always wins.
    const normalized = { ...data, type } as Record<string, unknown>;

    // Message signals
    if (['EmailReceived', 'DeliveryConfirmed', 'ReadReceiptReceived',
         'EmailStateChanged', 'ThreadActivity', 'DraftSaved',
         'ScheduledEmailSent', 'TypingIndicator'].includes(type)) {
      return normalized as MessageSignal;
    }

    // Trust notifications remain advisory wake-up hints. A consumer must fetch
    // and validate the exact record before promoting any claimed state.
    // Trust signals
    if (['AttestationReceived', 'TrustScoreUpdated', 'IntroductionReceived',
         'DisputeFiled', 'ByzantineDetected'].includes(type)) {
      return normalized as TrustSignal;
    }

    // Sync signals
    if (['SyncStateChanged', 'OperationReceived', 'ConflictDetected',
         'ConflictResolved', 'Online', 'Offline', 'SyncProgress',
         'SyncComplete'].includes(type)) {
      return normalized as SyncSignal;
    }

    // Federation signals
    if (['NetworkDiscovered', 'EnvelopeReceived', 'EnvelopeDelivered',
         'DeliveryFailed', 'BridgeStatusChanged', 'RouteChanged',
         'PeerStatusChanged'].includes(type)) {
      return normalized as FederationSignal;
    }

    // Unknown variants fail closed; never coerce them into FederationSignal.
    return null;
  }

  private dispatch(signal: MycelixSignal): void {
    // Add to buffer for replay
    this.signalBuffer.push(signal);
    if (this.signalBuffer.length > this.bufferSize) {
      this.signalBuffer.shift();
    }

    // Dispatch to category-specific handlers
    this.dispatchToCategory(signal);

    // Dispatch to general subscribers
    for (const sub of this.subscriptions.values()) {
      if (this.matchesSubscription(signal, sub)) {
        try {
          sub.handler(signal);
        } catch (error) {
          this.handleError(error as Error, signal);
        }
      }
    }
  }

  private dispatchToCategory(signal: MycelixSignal): void {
    const type = signal.type;

    // Message signals
    if (['EmailReceived', 'DeliveryConfirmed', 'ReadReceiptReceived',
         'EmailStateChanged', 'ThreadActivity', 'DraftSaved',
         'ScheduledEmailSent', 'TypingIndicator'].includes(type)) {
      this.messageHandlers.forEach(handler => {
        try { handler(signal as MessageSignal); }
        catch (e) { this.handleError(e as Error, signal); }
      });
    }

    // Trust signals
    if (['AttestationReceived', 'TrustScoreUpdated', 'IntroductionReceived',
         'DisputeFiled', 'ByzantineDetected'].includes(type)) {
      this.trustHandlers.forEach(handler => {
        try { handler(signal as TrustSignal); }
        catch (e) { this.handleError(e as Error, signal); }
      });
    }

    // Sync signals
    if (['SyncStateChanged', 'OperationReceived', 'ConflictDetected',
         'ConflictResolved', 'Online', 'Offline', 'SyncProgress',
         'SyncComplete'].includes(type)) {
      this.syncHandlers.forEach(handler => {
        try { handler(signal as SyncSignal); }
        catch (e) { this.handleError(e as Error, signal); }
      });
    }

    // Federation signals
    if (['NetworkDiscovered', 'EnvelopeReceived', 'EnvelopeDelivered',
         'DeliveryFailed', 'BridgeStatusChanged', 'RouteChanged',
         'PeerStatusChanged'].includes(type)) {
      this.federationHandlers.forEach(handler => {
        try { handler(signal as FederationSignal); }
        catch (e) { this.handleError(e as Error, signal); }
      });
    }
  }

  private matchesSubscription(signal: MycelixSignal, sub: SignalSubscription): boolean {
    if (sub.filter && !sub.filter(signal)) {
      return false;
    }
    return true;
  }

  private handleError(error: Error, signal: unknown): void {
    console.error('Signal processing error:', error, signal);
    this.errorHandlers.forEach(handler => handler(error, signal));
  }

  // ==================== PUBLIC API ====================

  /**
   * Subscribe to all signals with optional filter
   */
  subscribe(handler: SignalHandler<MycelixSignal>, filter?: SignalFilter): () => void {
    const id = crypto.randomUUID();
    this.subscriptions.set(id, { id, handler, filter });
    return () => this.subscriptions.delete(id);
  }

  /**
   * Subscribe to message signals only
   */
  onMessage(handler: SignalHandler<MessageSignal>): () => void {
    this.messageHandlers.add(handler);
    return () => this.messageHandlers.delete(handler);
  }

  /**
   * Subscribe to trust signals only
   */
  onTrust(handler: SignalHandler<TrustSignal>): () => void {
    this.trustHandlers.add(handler);
    return () => this.trustHandlers.delete(handler);
  }

  /**
   * Subscribe to sync signals only
   */
  onSync(handler: SignalHandler<SyncSignal>): () => void {
    this.syncHandlers.add(handler);
    return () => this.syncHandlers.delete(handler);
  }

  /**
   * Subscribe to federation signals only
   */
  onFederation(handler: SignalHandler<FederationSignal>): () => void {
    this.federationHandlers.add(handler);
    return () => this.federationHandlers.delete(handler);
  }

  /**
   * Subscribe to errors
   */
  onError(handler: (error: Error, signal: unknown) => void): () => void {
    this.errorHandlers.add(handler);
    return () => this.errorHandlers.delete(handler);
  }

  /**
   * Subscribe to specific signal type
   */
  on<T extends MycelixSignal['type']>(
    type: T,
    handler: SignalHandler<Extract<MycelixSignal, { type: T }>>
  ): () => void {
    return this.subscribe(
      handler as SignalHandler<MycelixSignal>,
      (signal) => signal.type === type
    );
  }

  /**
   * Wait for a specific signal (one-time)
   */
  waitFor<T extends MycelixSignal['type']>(
    type: T,
    timeout?: number
  ): Promise<Extract<MycelixSignal, { type: T }>> {
    return new Promise((resolve, reject) => {
      const timer = timeout ? setTimeout(() => {
        unsubscribe();
        reject(new Error(`Timeout waiting for signal: ${type}`));
      }, timeout) : null;

      const unsubscribe = this.on(type, (signal) => {
        if (timer) clearTimeout(timer);
        unsubscribe();
        resolve(signal as Extract<MycelixSignal, { type: T }>);
      });
    });
  }

  /**
   * Get recent signals from buffer
   */
  getRecentSignals(count?: number): MycelixSignal[] {
    const n = count ?? this.bufferSize;
    return this.signalBuffer.slice(-n);
  }

  /**
   * Get signals by type from buffer
   */
  getSignalsByType<T extends MycelixSignal['type']>(
    type: T
  ): Array<Extract<MycelixSignal, { type: T }>> {
    return this.signalBuffer.filter(
      (s) => s.type === type
    ) as Array<Extract<MycelixSignal, { type: T }>>;
  }

  /**
   * Clear signal buffer
   */
  clearBuffer(): void {
    this.signalBuffer = [];
  }

  /**
   * Check if connected
   */
  get connected(): boolean {
    return this.isConnected;
  }

  /**
   * Unsubscribe all handlers
   */
  clear(): void {
    this.subscriptions.clear();
    this.messageHandlers.clear();
    this.trustHandlers.clear();
    this.syncHandlers.clear();
    this.federationHandlers.clear();
    this.errorHandlers.clear();
  }
}

/**
 * Create typed event emitter for UI frameworks
 */
export function createSignalEmitter(hub: SignalHub) {
  const listeners = new Map<string, Set<Function>>();

  hub.subscribe((signal) => {
    const handlers = listeners.get(signal.type);
    if (handlers) {
      handlers.forEach(fn => fn(signal));
    }
  });

  return {
    on(type: string, handler: Function) {
      if (!listeners.has(type)) {
        listeners.set(type, new Set());
      }
      listeners.get(type)!.add(handler);
      return () => listeners.get(type)?.delete(handler);
    },
    off(type: string, handler: Function) {
      listeners.get(type)?.delete(handler);
    },
    once(type: string, handler: Function) {
      const wrapper = (signal: unknown) => {
        handler(signal);
        listeners.get(type)?.delete(wrapper);
      };
      if (!listeners.has(type)) {
        listeners.set(type, new Set());
      }
      listeners.get(type)!.add(wrapper);
    },
  };
}
