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

type SignalRecord = Record<string, unknown>;

const isSignalRecord = (value: unknown): value is SignalRecord =>
  value !== null && typeof value === 'object' && !Array.isArray(value);

const hasExactKeys = (value: SignalRecord, expected: readonly string[]): boolean => {
  const actual = Object.keys(value);
  return actual.length === expected.length &&
    expected.every((key) => Object.prototype.hasOwnProperty.call(value, key));
};

const isBoundedString = (value: unknown, maxBytes = 1024): value is string =>
  typeof value === 'string' && value.trim().length > 0 &&
  new TextEncoder().encode(value).length <= maxBytes;

const isFiniteNumber = (value: unknown): value is number =>
  typeof value === 'number' && Number.isFinite(value);

const isSafeInteger = (value: unknown): value is number =>
  typeof value === 'number' && Number.isSafeInteger(value);

const isNonNegativeInteger = (value: unknown): value is number =>
  isSafeInteger(value) && value >= 0;

const isTimestamp = (value: unknown): value is number => isSafeInteger(value);

const isHashBytes = (value: unknown): value is Uint8Array =>
  value instanceof Uint8Array && value.byteLength === 39;

const isByteVector = (value: unknown, minLength = 0, maxLength = 1_048_576): value is Uint8Array =>
  value instanceof Uint8Array && value.byteLength >= minLength && value.byteLength <= maxLength;

const isHashArray = (value: unknown, maxItems = 4096): value is Uint8Array[] =>
  Array.isArray(value) && value.length <= maxItems && value.every(isHashBytes);

const isBoundedHashArray = (value: unknown, maxItems = 4096): value is Uint8Array[] =>
  Array.isArray(value) && value.length > 0 && value.length <= maxItems && value.every(isHashBytes);

const isUnitEnum = (value: unknown, variants: readonly string[]): boolean =>
  typeof value === 'string' && variants.includes(value);

const isExternallyTaggedStringEnum = (
  value: unknown,
  variants: readonly string[],
  customVariant: string
): boolean => {
  if (isUnitEnum(value, variants)) return true;
  if (!isSignalRecord(value) || !hasExactKeys(value, [customVariant])) return false;
  return isBoundedString(value[customVariant], 256);
};

const isConflictResolution = (value: unknown): boolean => {
  if (isUnitEnum(value, ['Pending', 'Merged', 'BothKept'])) return true;
  if (!isSignalRecord(value) || Object.keys(value).length !== 1) return false;
  if (hasExactKeys(value, ['AutoResolved'])) {
    const item = value.AutoResolved;
    return isSignalRecord(item) && hasExactKeys(item, ['strategy']) &&
      isBoundedString(item.strategy, 256);
  }
  if (hasExactKeys(value, ['LastWriterWins'])) {
    const item = value.LastWriterWins;
    return isSignalRecord(item) && hasExactKeys(item, ['winner']) && isHashBytes(item.winner);
  }
  if (hasExactKeys(value, ['ManualResolution'])) {
    const item = value.ManualResolution;
    return isSignalRecord(item) && hasExactKeys(item, ['resolver']) && isHashBytes(item.resolver);
  }
  return false;
};

const isNetworkType = (value: unknown): boolean => {
  if (isUnitEnum(value, [
    'HolochainNative', 'SmtpBridge', 'MatrixBridge', 'ActivityPubBridge',
    'XmppBridge', 'OrganizationPrivate', 'PublicOpen'
  ])) return true;
  if (!isSignalRecord(value) || !hasExactKeys(value, ['CustomBridge'])) return false;
  const item = value.CustomBridge;
  return isSignalRecord(item) && hasExactKeys(item, ['protocol']) &&
    isBoundedString(item.protocol, 128);
};

const isSyncOperationType = (value: unknown): boolean =>
  isExternallyTaggedStringEnum(value, [
    'EmailReceived', 'EmailRead', 'EmailDeleted', 'EmailMoved', 'EmailFlagged',
    'EmailLabeled', 'FolderCreated', 'FolderRenamed', 'FolderDeleted',
    'FolderMoved', 'DraftCreated', 'DraftUpdated', 'DraftDeleted', 'DraftSent',
    'SettingChanged', 'FilterCreated', 'FilterUpdated', 'FilterDeleted',
    'TrustAttestationCreated', 'TrustAttestationRevoked',
    'ContactCreated', 'ContactUpdated', 'ContactDeleted'
  ], 'Custom');

const isVectorClock = (value: unknown): boolean => {
  if (!isSignalRecord(value) || !hasExactKeys(value, ['clocks'])) return false;
  const clocks = value.clocks;
  return Array.isArray(clocks) && clocks.length <= 4096 &&
    clocks.every((item) =>
      Array.isArray(item) && item.length === 2 &&
      isHashBytes(item[0]) && isNonNegativeInteger(item[1])
    );
};

const isEmailStateUpdate = (value: unknown): boolean => {
  if (!isSignalRecord(value)) return false;
  const fields = ['is_read', 'is_starred', 'is_archived', 'is_trashed', 'folders'];
  return hasExactKeys(value, fields) &&
    ['is_read', 'is_starred', 'is_archived', 'is_trashed'].every(
      (field) => value[field] === null || typeof value[field] === 'boolean'
    ) &&
    (value.folders === null || isHashArray(value.folders));
};

/**
 * Runtime validation for the fully decoded adjacent-tag signal payload.
 * Rust newtypes/aliases remain typed Uint8Array/number at the JS boundary;
 * JSON Schema separately documents their portable JSON projection.
 */
function validateSignalData(type: string, data: SignalRecord): boolean {
  switch (type) {
    case 'EmailReceived':
      return isHashBytes(data.email_hash) && isHashBytes(data.sender) &&
        isByteVector(data.encrypted_subject, 1, 65_536) &&
        isTimestamp(data.timestamp) &&
        isUnitEnum(data.priority, ['Low', 'Normal', 'High', 'Urgent']);
    case 'DeliveryConfirmed':
      return isHashBytes(data.email_hash) && isHashBytes(data.recipient) &&
        isTimestamp(data.delivered_at);
    case 'ReadReceiptReceived':
      return isHashBytes(data.email_hash) && isHashBytes(data.reader) &&
        isTimestamp(data.read_at);
    case 'EmailStateChanged':
      return isHashBytes(data.email_hash) && isEmailStateUpdate(data.state);
    case 'ThreadActivity':
      return isBoundedString(data.thread_id, 256) &&
        isHashBytes(data.email_hash) && isHashBytes(data.sender);
    case 'DraftSaved':
      return isHashBytes(data.draft_hash);
    case 'ScheduledEmailSent':
      return isHashBytes(data.email_hash) && isBoundedHashArray(data.recipients);
    case 'TypingIndicator':
      return isHashBytes(data.sender) &&
        (data.thread_id === null || isBoundedString(data.thread_id, 256));

    case 'AttestationReceived':
      return isHashBytes(data.attestation_hash) && isHashBytes(data.truster) &&
        isFiniteNumber(data.trust_level) && data.trust_level >= -1 && data.trust_level <= 1 &&
        isExternallyTaggedStringEnum(data.category, [
          'Identity', 'Communication', 'FileSharing', 'Scheduling', 'Financial',
          'CredentialIssuer', 'Organization', 'Personal', 'Professional'
        ], 'Custom');
    case 'TrustScoreUpdated':
      return isHashBytes(data.agent) && isFiniteNumber(data.new_score) &&
        data.new_score >= 0 && data.new_score <= 1 &&
        isFiniteNumber(data.confidence) && data.confidence >= 0 && data.confidence <= 1;
    case 'IntroductionReceived':
      return isHashBytes(data.introduction_hash) && isHashBytes(data.introducer) &&
        isHashBytes(data.introduced) && isFiniteNumber(data.recommendation);
    case 'DisputeFiled':
      return isHashBytes(data.dispute_hash) && isHashBytes(data.attestation_hash) &&
        isHashBytes(data.disputer);
    case 'ByzantineDetected':
      return isHashBytes(data.agent) &&
        isUnitEnum(data.flag_type, [
          'SybilSuspicion', 'CollusionPattern', 'TrustVolatility',
          'InconsistentAttestations', 'SelfDealing', 'AttestationFarming'
        ]) &&
        isFiniteNumber(data.severity) && data.severity >= 0 && data.severity <= 1;

    case 'SyncStateChanged':
      return isVectorClock(data.vector_clock) && isNonNegativeInteger(data.pending_ops);
    case 'OperationReceived':
      return isBoundedString(data.op_id, 256) && isSyncOperationType(data.op_type) &&
        isHashBytes(data.from_agent);
    case 'ConflictDetected':
      return isBoundedString(data.conflict_id, 256) &&
        isUnitEnum(data.conflict_type, [
          'ConcurrentEdit', 'DeleteUpdate', 'ConcurrentMove',
          'CausalityViolation', 'VersionMismatch'
        ]);
    case 'ConflictResolved':
      return isBoundedString(data.conflict_id, 256) && isConflictResolution(data.resolution);
    case 'Online':
    case 'Offline':
      return hasExactKeys(data, []);
    case 'SyncProgress':
      return isNonNegativeInteger(data.synced) && isNonNegativeInteger(data.total) &&
        data.synced <= data.total && isHashBytes(data.peer);
    case 'SyncComplete':
      return isHashBytes(data.peer) && isNonNegativeInteger(data.ops_synced);

    case 'NetworkDiscovered':
      return isBoundedString(data.network_id, 256) && isNetworkType(data.network_type);
    case 'EnvelopeReceived':
      return isBoundedString(data.envelope_id, 256) && isBoundedString(data.from_network, 256);
    case 'EnvelopeDelivered':
      return isBoundedString(data.envelope_id, 256) && isBoundedString(data.to_network, 256);
    case 'DeliveryFailed':
      return isBoundedString(data.envelope_id, 256) && isBoundedString(data.error, 4096);
    case 'BridgeStatusChanged':
      return isHashBytes(data.bridge_agent) && typeof data.is_active === 'boolean';
    case 'RouteChanged':
      return isBoundedString(data.route_id, 256) && typeof data.is_active === 'boolean';
    case 'PeerStatusChanged':
      return isBoundedString(data.network_id, 256) && typeof data.is_reachable === 'boolean';
    default:
      return false;
  }
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

    if (!validateSignalData(type, data)) return null;

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
