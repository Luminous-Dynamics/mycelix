// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * End-to-End Key Exchange Service for Mycelix Mail
 *
 * Secure key management:
 * - X3DH (Extended Triple Diffie-Hellman) key agreement
 * - Double Ratchet for perfect forward secrecy
 * - Pre-key bundles for async key exchange
 * - Key rotation and revocation
 * - Multi-device support
 */

import type { AgentPubKey, ActionHash } from '@holochain/client';
import type { MycelixMailClient } from '../index';

// ==================== TYPES ====================

export interface KeyPair {
  publicKey: Uint8Array;
  privateKey: Uint8Array;
}

export interface IdentityKeyPair extends KeyPair {
  keyId: string;
  createdAt: number;
}

export interface SignedPreKey extends KeyPair {
  keyId: number;
  signature: Uint8Array;
  createdAt: number;
  expiresAt: number;
}

export interface OneTimePreKey extends KeyPair {
  keyId: number;
  used: boolean;
}

export interface PreKeyBundle {
  identityKey: Uint8Array;
  signedPreKey: Uint8Array;
  signedPreKeyId: number;
  signedPreKeySignature: Uint8Array;
  oneTimePreKey?: Uint8Array;
  oneTimePreKeyId?: number;
}

export interface SessionState {
  remoteAgent: AgentPubKey;
  rootKey: Uint8Array;
  chainKey: Uint8Array;
  sendingChainKey?: Uint8Array;
  receivingChainKey?: Uint8Array;
  sendingRatchetKey?: KeyPair;
  receivingRatchetKey?: Uint8Array;
  previousChainLength: number;
  messageNumber: number;
  receivedMessageNumbers: Set<number>;
  createdAt: number;
  lastActivity: number;
}

export interface EncryptedMessage {
  ciphertext: Uint8Array;
  header: MessageHeader;
  mac: Uint8Array;
}

export interface MessageHeader {
  senderRatchetKey: Uint8Array;
  previousChainLength: number;
  messageNumber: number;
  sessionId: string;
}

export interface KeyExchangeConfig {
  /** Number of one-time pre-keys to maintain */
  oneTimePreKeyCount?: number;
  /** Signed pre-key rotation interval (ms) */
  signedPreKeyRotation?: number;
  /** Session timeout (ms) */
  sessionTimeout?: number;
  /** Max skipped message keys to store */
  maxSkippedKeys?: number;
  /** Enable perfect forward secrecy */
  enablePFS?: boolean;
}

export interface KeyExchangeStats {
  activeSession: number;
  totalExchanges: number;
  preKeysRemaining: number;
  lastRotation: number;
}

// ==================== KEY EXCHANGE SERVICE ====================

export class KeyExchangeService {
  private identityKeyPair: IdentityKeyPair | null = null;
  private signedPreKey: SignedPreKey | null = null;
  private oneTimePreKeys: Map<number, OneTimePreKey> = new Map();
  private sessions: Map<string, SessionState> = new Map();
  private skippedMessageKeys: Map<string, Uint8Array> = new Map();
  private config: KeyExchangeConfig;
  private rotationInterval: NodeJS.Timeout | null = null;
  private nextPreKeyId = 1;

  private readonly DB_NAME = 'mycelix_keys';
  private readonly IDENTITY_STORE = 'identity';
  private readonly PREKEY_STORE = 'prekeys';
  private readonly SESSION_STORE = 'sessions';

  constructor(
    private client?: MycelixMailClient,
    config: Partial<KeyExchangeConfig> = {}
  ) {
    this.config = {
      oneTimePreKeyCount: 100,
      signedPreKeyRotation: 7 * 24 * 60 * 60 * 1000, // 7 days
      sessionTimeout: 30 * 24 * 60 * 60 * 1000, // 30 days
      maxSkippedKeys: 1000,
      enablePFS: true,
      ...config,
    };
  }

  // ==================== INITIALIZATION ====================

  /**
   * Initialize key exchange service
   */
  async initialize(): Promise<void> {
    // Load or generate identity key
    await this.loadOrGenerateIdentityKey();

    // Load or generate signed pre-key
    await this.loadOrGenerateSignedPreKey();

    // Generate initial one-time pre-keys
    await this.replenishOneTimePreKeys();

    // Start rotation interval
    this.startRotationInterval();

    // Load existing sessions
    await this.loadSessions();
  }

  /**
   * Load or generate identity key pair
   */
  private async loadOrGenerateIdentityKey(): Promise<void> {
    // Try to load from storage
    const stored = await this.loadFromStorage<IdentityKeyPair>(
      this.IDENTITY_STORE,
      'identity'
    );

    if (stored) {
      this.identityKeyPair = stored;
      return;
    }

    // Generate new identity key
    const keyPair = await this.generateKeyPair();
    this.identityKeyPair = {
      ...keyPair,
      keyId: this.generateKeyId(),
      createdAt: Date.now(),
    };

    await this.saveToStorage(this.IDENTITY_STORE, 'identity', this.identityKeyPair);
  }

  /**
   * Load or generate signed pre-key
   */
  private async loadOrGenerateSignedPreKey(): Promise<void> {
    const stored = await this.loadFromStorage<SignedPreKey>(
      this.PREKEY_STORE,
      'signed'
    );

    if (stored && stored.expiresAt > Date.now()) {
      this.signedPreKey = stored;
      return;
    }

    await this.rotateSignedPreKey();
  }

  /**
   * Rotate signed pre-key
   */
  async rotateSignedPreKey(): Promise<void> {
    if (!this.identityKeyPair) {
      throw new Error('Identity key not initialized');
    }

    const keyPair = await this.generateKeyPair();
    const keyId = this.nextPreKeyId++;
    const signature = await this.sign(
      keyPair.publicKey,
      this.identityKeyPair.privateKey
    );

    this.signedPreKey = {
      ...keyPair,
      keyId,
      signature,
      createdAt: Date.now(),
      expiresAt: Date.now() + this.config.signedPreKeyRotation!,
    };

    await this.saveToStorage(this.PREKEY_STORE, 'signed', this.signedPreKey);

    // Publish to DHT
    if (this.client) {
      await this.publishPreKeyBundle();
    }
  }

  /**
   * Replenish one-time pre-keys
   */
  async replenishOneTimePreKeys(): Promise<void> {
    const needed = this.config.oneTimePreKeyCount! - this.oneTimePreKeys.size;

    for (let i = 0; i < needed; i++) {
      const keyPair = await this.generateKeyPair();
      const keyId = this.nextPreKeyId++;

      const preKey: OneTimePreKey = {
        ...keyPair,
        keyId,
        used: false,
      };

      this.oneTimePreKeys.set(keyId, preKey);
    }

    // Save to storage
    await this.saveToStorage(
      this.PREKEY_STORE,
      'onetime',
      Array.from(this.oneTimePreKeys.values())
    );

    // Publish to DHT
    if (this.client) {
      await this.publishPreKeyBundle();
    }
  }

  // ==================== PRE-KEY BUNDLE ====================

  /**
   * Get pre-key bundle for publishing
   */
  getPreKeyBundle(): PreKeyBundle | null {
    if (!this.identityKeyPair || !this.signedPreKey) {
      return null;
    }

    // Find an unused one-time pre-key
    let oneTimePreKey: OneTimePreKey | undefined;
    for (const preKey of this.oneTimePreKeys.values()) {
      if (!preKey.used) {
        oneTimePreKey = preKey;
        break;
      }
    }

    return {
      identityKey: this.identityKeyPair.publicKey,
      signedPreKey: this.signedPreKey.publicKey,
      signedPreKeyId: this.signedPreKey.keyId,
      signedPreKeySignature: this.signedPreKey.signature,
      oneTimePreKey: oneTimePreKey?.publicKey,
      oneTimePreKeyId: oneTimePreKey?.keyId,
    };
  }

  /**
   * Publish pre-key bundle to DHT
   */
  async publishPreKeyBundle(): Promise<void> {
    if (!this.client) return;

    const bundle = this.getPreKeyBundle();
    if (!bundle) return;

    // Store in Holochain DHT
    await this.client.callZome('keys', 'publish_pre_key_bundle', bundle);
  }

  /**
   * Fetch pre-key bundle for a remote agent
   */
  async fetchPreKeyBundle(agent: AgentPubKey): Promise<PreKeyBundle | null> {
    if (!this.client) return null;

    try {
      return await this.client.callZome('keys', 'get_pre_key_bundle', agent);
    } catch {
      return null;
    }
  }

  // ==================== X3DH KEY AGREEMENT ====================

  /**
   * Initiate key exchange with remote agent (X3DH sender)
   */
  async initiateKeyExchange(
    remoteAgent: AgentPubKey,
    remoteBundle: PreKeyBundle
  ): Promise<SessionState> {
    if (!this.identityKeyPair) {
      throw new Error('Identity key not initialized');
    }

    // Verify signed pre-key signature
    const valid = await this.verify(
      remoteBundle.signedPreKey,
      remoteBundle.signedPreKeySignature,
      remoteBundle.identityKey
    );

    if (!valid) {
      throw new Error('Invalid signed pre-key signature');
    }

    // Generate ephemeral key pair
    const ephemeralKey = await this.generateKeyPair();

    // X3DH: Calculate shared secrets
    // DH1 = DH(IK_A, SPK_B)
    const dh1 = await this.diffieHellman(
      this.identityKeyPair.privateKey,
      remoteBundle.signedPreKey
    );

    // DH2 = DH(EK_A, IK_B)
    const dh2 = await this.diffieHellman(
      ephemeralKey.privateKey,
      remoteBundle.identityKey
    );

    // DH3 = DH(EK_A, SPK_B)
    const dh3 = await this.diffieHellman(
      ephemeralKey.privateKey,
      remoteBundle.signedPreKey
    );

    // DH4 = DH(EK_A, OPK_B) - if available
    let dh4: Uint8Array | null = null;
    if (remoteBundle.oneTimePreKey) {
      dh4 = await this.diffieHellman(
        ephemeralKey.privateKey,
        remoteBundle.oneTimePreKey
      );
    }

    // Derive shared secret
    const sharedSecret = await this.kdf(
      this.concat(dh1, dh2, dh3, dh4 ? dh4 : new Uint8Array(0)),
      'X3DH'
    );

    // Initialize session
    const sessionId = this.generateSessionId(remoteAgent);
    const session: SessionState = {
      remoteAgent,
      rootKey: sharedSecret,
      chainKey: await this.kdf(sharedSecret, 'ChainKey'),
      sendingRatchetKey: ephemeralKey,
      receivingRatchetKey: remoteBundle.signedPreKey,
      previousChainLength: 0,
      messageNumber: 0,
      receivedMessageNumbers: new Set(),
      createdAt: Date.now(),
      lastActivity: Date.now(),
    };

    this.sessions.set(sessionId, session);
    await this.saveSession(sessionId, session);

    // Send initial message with ephemeral key
    // (This would be included in the first encrypted message)

    return session;
  }

  /**
   * Complete key exchange (X3DH receiver)
   */
  async completeKeyExchange(
    remoteAgent: AgentPubKey,
    remoteIdentityKey: Uint8Array,
    remoteEphemeralKey: Uint8Array,
    oneTimePreKeyId?: number
  ): Promise<SessionState> {
    if (!this.identityKeyPair || !this.signedPreKey) {
      throw new Error('Keys not initialized');
    }

    // X3DH: Calculate shared secrets (receiver side)
    // DH1 = DH(SPK_B, IK_A)
    const dh1 = await this.diffieHellman(
      this.signedPreKey.privateKey,
      remoteIdentityKey
    );

    // DH2 = DH(IK_B, EK_A)
    const dh2 = await this.diffieHellman(
      this.identityKeyPair.privateKey,
      remoteEphemeralKey
    );

    // DH3 = DH(SPK_B, EK_A)
    const dh3 = await this.diffieHellman(
      this.signedPreKey.privateKey,
      remoteEphemeralKey
    );

    // DH4 = DH(OPK_B, EK_A) - if used
    let dh4: Uint8Array | null = null;
    if (oneTimePreKeyId !== undefined) {
      const oneTimePreKey = this.oneTimePreKeys.get(oneTimePreKeyId);
      if (oneTimePreKey) {
        dh4 = await this.diffieHellman(
          oneTimePreKey.privateKey,
          remoteEphemeralKey
        );
        // Mark as used
        oneTimePreKey.used = true;
        // Replenish if needed
        this.replenishOneTimePreKeys();
      }
    }

    // Derive shared secret
    const sharedSecret = await this.kdf(
      this.concat(dh1, dh2, dh3, dh4 ? dh4 : new Uint8Array(0)),
      'X3DH'
    );

    // Initialize session
    const sessionId = this.generateSessionId(remoteAgent);
    const session: SessionState = {
      remoteAgent,
      rootKey: sharedSecret,
      chainKey: await this.kdf(sharedSecret, 'ChainKey'),
      receivingRatchetKey: remoteEphemeralKey,
      previousChainLength: 0,
      messageNumber: 0,
      receivedMessageNumbers: new Set(),
      createdAt: Date.now(),
      lastActivity: Date.now(),
    };

    this.sessions.set(sessionId, session);
    await this.saveSession(sessionId, session);

    return session;
  }

  // ==================== DOUBLE RATCHET ====================

  /**
   * Encrypt a message using Double Ratchet
   */
  async encrypt(
    remoteAgent: AgentPubKey,
    plaintext: Uint8Array
  ): Promise<EncryptedMessage> {
    const sessionId = this.generateSessionId(remoteAgent);
    let session = this.sessions.get(sessionId);

    if (!session) {
      // Need to initiate key exchange first
      const bundle = await this.fetchPreKeyBundle(remoteAgent);
      if (!bundle) {
        throw new Error('Cannot establish session: no pre-key bundle');
      }
      session = await this.initiateKeyExchange(remoteAgent, bundle);
    }

    // Perform sending ratchet step if needed
    if (!session.sendingChainKey) {
      session.sendingChainKey = session.chainKey;
    }

    // Derive message key
    const { messageKey, nextChainKey } = await this.deriveMessageKey(
      session.sendingChainKey
    );
    session.sendingChainKey = nextChainKey;

    // Encrypt
    const nonce = crypto.getRandomValues(new Uint8Array(12));
    const ciphertext = await this.aesEncrypt(plaintext, messageKey, nonce);

    // Create header
    const header: MessageHeader = {
      senderRatchetKey: session.sendingRatchetKey?.publicKey ?? new Uint8Array(0),
      previousChainLength: session.previousChainLength,
      messageNumber: session.messageNumber++,
      sessionId,
    };

    // Calculate MAC
    const mac = await this.hmac(
      this.concat(this.encodeHeader(header), ciphertext),
      messageKey
    );

    // Update session
    session.lastActivity = Date.now();
    await this.saveSession(sessionId, session);

    return {
      ciphertext: this.concat(nonce, ciphertext),
      header,
      mac,
    };
  }

  /**
   * Decrypt a message using Double Ratchet
   */
  async decrypt(
    remoteAgent: AgentPubKey,
    message: EncryptedMessage
  ): Promise<Uint8Array> {
    const sessionId = this.generateSessionId(remoteAgent);
    let session = this.sessions.get(sessionId);

    if (!session) {
      throw new Error('No session for decryption');
    }

    // Check for duplicate
    if (session.receivedMessageNumbers.has(message.header.messageNumber)) {
      throw new Error('Duplicate message');
    }

    // Check if we need to perform DH ratchet
    const ratchetKeyChanged = !this.arraysEqual(
      message.header.senderRatchetKey,
      session.receivingRatchetKey ?? new Uint8Array(0)
    );

    if (ratchetKeyChanged && this.config.enablePFS) {
      // Skip any missed messages in the current chain
      await this.skipMessageKeys(
        session,
        message.header.previousChainLength
      );

      // Perform DH ratchet
      await this.dhRatchet(session, message.header.senderRatchetKey);
    }

    // Try to decrypt with stored skipped key first
    const skippedKey = this.getSkippedKey(sessionId, message.header.messageNumber);
    let messageKey: Uint8Array;

    if (skippedKey) {
      messageKey = skippedKey;
      this.removeSkippedKey(sessionId, message.header.messageNumber);
    } else {
      // Skip any missed messages
      await this.skipMessageKeys(session, message.header.messageNumber);

      // Derive message key
      const result = await this.deriveMessageKey(
        session.receivingChainKey ?? session.chainKey
      );
      messageKey = result.messageKey;
      session.receivingChainKey = result.nextChainKey;
    }

    // Verify MAC
    const expectedMac = await this.hmac(
      this.concat(this.encodeHeader(message.header), message.ciphertext),
      messageKey
    );

    if (!this.arraysEqual(message.mac, expectedMac)) {
      throw new Error('MAC verification failed');
    }

    // Decrypt
    const nonce = message.ciphertext.slice(0, 12);
    const ciphertext = message.ciphertext.slice(12);
    const plaintext = await this.aesDecrypt(ciphertext, messageKey, nonce);

    // Update session
    session.receivedMessageNumbers.add(message.header.messageNumber);
    session.lastActivity = Date.now();
    await this.saveSession(sessionId, session);

    return plaintext;
  }

  /**
   * Perform DH ratchet step
   */
  private async dhRatchet(
    session: SessionState,
    newRatchetKey: Uint8Array
  ): Promise<void> {
    // Store previous chain length
    session.previousChainLength = session.messageNumber;
    session.messageNumber = 0;

    // Update receiving ratchet key
    session.receivingRatchetKey = newRatchetKey;

    // Generate new sending ratchet key
    session.sendingRatchetKey = await this.generateKeyPair();

    // Derive new root key and chain keys
    if (session.sendingRatchetKey) {
      const dhOutput = await this.diffieHellman(
        session.sendingRatchetKey.privateKey,
        newRatchetKey
      );

      const newRootKey = await this.kdf(
        this.concat(session.rootKey, dhOutput),
        'RootKey'
      );

      session.receivingChainKey = await this.kdf(newRootKey, 'ReceivingChain');
      session.sendingChainKey = await this.kdf(newRootKey, 'SendingChain');
      session.rootKey = newRootKey;
    }
  }

  /**
   * Skip and store message keys
   */
  private async skipMessageKeys(
    session: SessionState,
    until: number
  ): Promise<void> {
    if (!session.receivingChainKey) return;

    while (session.messageNumber < until) {
      if (this.skippedMessageKeys.size >= this.config.maxSkippedKeys!) {
        // Remove oldest
        const firstKey = this.skippedMessageKeys.keys().next().value;
        if (firstKey) this.skippedMessageKeys.delete(firstKey);
      }

      const { messageKey, nextChainKey } = await this.deriveMessageKey(
        session.receivingChainKey
      );

      const sessionId = this.generateSessionId(session.remoteAgent);
      this.skippedMessageKeys.set(
        `${sessionId}:${session.messageNumber}`,
        messageKey
      );

      session.receivingChainKey = nextChainKey;
      session.messageNumber++;
    }
  }

  /**
   * Derive message key from chain key
   */
  private async deriveMessageKey(
    chainKey: Uint8Array
  ): Promise<{ messageKey: Uint8Array; nextChainKey: Uint8Array }> {
    const messageKey = await this.kdf(chainKey, 'MessageKey');
    const nextChainKey = await this.kdf(chainKey, 'ChainKey');
    return { messageKey, nextChainKey };
  }

  // ==================== SESSION MANAGEMENT ====================

  /**
   * Get or create session
   */
  async getSession(remoteAgent: AgentPubKey): Promise<SessionState | null> {
    const sessionId = this.generateSessionId(remoteAgent);
    return this.sessions.get(sessionId) ?? null;
  }

  /**
   * Check if session exists
   */
  hasSession(remoteAgent: AgentPubKey): boolean {
    const sessionId = this.generateSessionId(remoteAgent);
    return this.sessions.has(sessionId);
  }

  /**
   * Delete session
   */
  async deleteSession(remoteAgent: AgentPubKey): Promise<void> {
    const sessionId = this.generateSessionId(remoteAgent);
    this.sessions.delete(sessionId);

    // Remove skipped keys for this session
    for (const key of this.skippedMessageKeys.keys()) {
      if (key.startsWith(sessionId)) {
        this.skippedMessageKeys.delete(key);
      }
    }

    await this.deleteFromStorage(this.SESSION_STORE, sessionId);
  }

  /**
   * Clean up expired sessions
   */
  async cleanupSessions(): Promise<number> {
    const now = Date.now();
    let cleaned = 0;

    for (const [sessionId, session] of this.sessions) {
      if (now - session.lastActivity > this.config.sessionTimeout!) {
        this.sessions.delete(sessionId);
        await this.deleteFromStorage(this.SESSION_STORE, sessionId);
        cleaned++;
      }
    }

    return cleaned;
  }

  // ==================== CRYPTO PRIMITIVES ====================

  /**
   * Generate key pair (X25519)
   */
  private async generateKeyPair(): Promise<KeyPair> {
    const keyPair = await crypto.subtle.generateKey(
      { name: 'X25519' },
      true,
      ['deriveBits']
    ) as CryptoKeyPair;

    const publicKey = await crypto.subtle.exportKey('raw', keyPair.publicKey);
    const privateKey = await crypto.subtle.exportKey('pkcs8', keyPair.privateKey);

    return {
      publicKey: new Uint8Array(publicKey),
      privateKey: new Uint8Array(privateKey),
    };
  }

  /**
   * Diffie-Hellman key agreement
   */
  private async diffieHellman(
    privateKey: Uint8Array,
    publicKey: Uint8Array
  ): Promise<Uint8Array> {
    const privKey = await crypto.subtle.importKey(
      'pkcs8',
      privateKey,
      { name: 'X25519' },
      false,
      ['deriveBits']
    );

    const pubKey = await crypto.subtle.importKey(
      'raw',
      publicKey,
      { name: 'X25519' },
      false,
      []
    );

    const sharedBits = await crypto.subtle.deriveBits(
      { name: 'X25519', public: pubKey },
      privKey,
      256
    );

    return new Uint8Array(sharedBits);
  }

  /**
   * Key derivation function (HKDF)
   */
  private async kdf(
    input: Uint8Array,
    info: string
  ): Promise<Uint8Array> {
    const key = await crypto.subtle.importKey(
      'raw',
      input,
      { name: 'HKDF' },
      false,
      ['deriveBits']
    );

    const derived = await crypto.subtle.deriveBits(
      {
        name: 'HKDF',
        hash: 'SHA-256',
        salt: new Uint8Array(32),
        info: new TextEncoder().encode(info),
      },
      key,
      256
    );

    return new Uint8Array(derived);
  }

  /**
   * AES-GCM encryption
   */
  private async aesEncrypt(
    plaintext: Uint8Array,
    key: Uint8Array,
    nonce: Uint8Array
  ): Promise<Uint8Array> {
    const cryptoKey = await crypto.subtle.importKey(
      'raw',
      key,
      { name: 'AES-GCM' },
      false,
      ['encrypt']
    );

    const ciphertext = await crypto.subtle.encrypt(
      { name: 'AES-GCM', iv: nonce },
      cryptoKey,
      plaintext
    );

    return new Uint8Array(ciphertext);
  }

  /**
   * AES-GCM decryption
   */
  private async aesDecrypt(
    ciphertext: Uint8Array,
    key: Uint8Array,
    nonce: Uint8Array
  ): Promise<Uint8Array> {
    const cryptoKey = await crypto.subtle.importKey(
      'raw',
      key,
      { name: 'AES-GCM' },
      false,
      ['decrypt']
    );

    const plaintext = await crypto.subtle.decrypt(
      { name: 'AES-GCM', iv: nonce },
      cryptoKey,
      ciphertext
    );

    return new Uint8Array(plaintext);
  }

  /**
   * Sign data (Ed25519 simulation via ECDSA)
   */
  private async sign(
    data: Uint8Array,
    privateKey: Uint8Array
  ): Promise<Uint8Array> {
    const key = await crypto.subtle.importKey(
      'pkcs8',
      privateKey,
      { name: 'Ed25519' },
      false,
      ['sign']
    );

    const signature = await crypto.subtle.sign('Ed25519', key, data);
    return new Uint8Array(signature);
  }

  /**
   * Verify signature
   */
  private async verify(
    data: Uint8Array,
    signature: Uint8Array,
    publicKey: Uint8Array
  ): Promise<boolean> {
    try {
      const key = await crypto.subtle.importKey(
        'raw',
        publicKey,
        { name: 'Ed25519' },
        false,
        ['verify']
      );

      return await crypto.subtle.verify('Ed25519', key, signature, data);
    } catch {
      return false;
    }
  }

  /**
   * HMAC
   */
  private async hmac(
    data: Uint8Array,
    key: Uint8Array
  ): Promise<Uint8Array> {
    const cryptoKey = await crypto.subtle.importKey(
      'raw',
      key,
      { name: 'HMAC', hash: 'SHA-256' },
      false,
      ['sign']
    );

    const mac = await crypto.subtle.sign('HMAC', cryptoKey, data);
    return new Uint8Array(mac);
  }

  // ==================== UTILITIES ====================

  private generateKeyId(): string {
    return `key_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  private generateSessionId(agent: AgentPubKey): string {
    // Create deterministic session ID from agent public key
    const agentStr = Array.from(agent as Uint8Array)
      .map((b) => b.toString(16).padStart(2, '0'))
      .join('');
    return `session_${agentStr.substring(0, 16)}`;
  }

  private concat(...arrays: Uint8Array[]): Uint8Array {
    const totalLength = arrays.reduce((sum, arr) => sum + arr.length, 0);
    const result = new Uint8Array(totalLength);
    let offset = 0;
    for (const arr of arrays) {
      result.set(arr, offset);
      offset += arr.length;
    }
    return result;
  }

  private arraysEqual(a: Uint8Array, b: Uint8Array): boolean {
    if (a.length !== b.length) return false;
    for (let i = 0; i < a.length; i++) {
      if (a[i] !== b[i]) return false;
    }
    return true;
  }

  private encodeHeader(header: MessageHeader): Uint8Array {
    const json = JSON.stringify({
      senderRatchetKey: Array.from(header.senderRatchetKey),
      previousChainLength: header.previousChainLength,
      messageNumber: header.messageNumber,
      sessionId: header.sessionId,
    });
    return new TextEncoder().encode(json);
  }

  private getSkippedKey(sessionId: string, messageNumber: number): Uint8Array | undefined {
    return this.skippedMessageKeys.get(`${sessionId}:${messageNumber}`);
  }

  private removeSkippedKey(sessionId: string, messageNumber: number): void {
    this.skippedMessageKeys.delete(`${sessionId}:${messageNumber}`);
  }

  // ==================== STORAGE ====================

  private async loadFromStorage<T>(store: string, key: string): Promise<T | null> {
    try {
      const data = localStorage.getItem(`${this.DB_NAME}_${store}_${key}`);
      return data ? JSON.parse(data) : null;
    } catch {
      return null;
    }
  }

  private async saveToStorage(store: string, key: string, value: unknown): Promise<void> {
    try {
      localStorage.setItem(`${this.DB_NAME}_${store}_${key}`, JSON.stringify(value));
    } catch {
      console.error('Failed to save to storage');
    }
  }

  private async deleteFromStorage(store: string, key: string): Promise<void> {
    try {
      localStorage.removeItem(`${this.DB_NAME}_${store}_${key}`);
    } catch {
      console.error('Failed to delete from storage');
    }
  }

  private async saveSession(sessionId: string, session: SessionState): Promise<void> {
    // Convert Set to Array for serialization
    const serializable = {
      ...session,
      receivedMessageNumbers: Array.from(session.receivedMessageNumbers),
    };
    await this.saveToStorage(this.SESSION_STORE, sessionId, serializable);
  }

  private async loadSessions(): Promise<void> {
    // Load sessions from storage
    // Implementation depends on storage mechanism
  }

  private startRotationInterval(): void {
    this.rotationInterval = setInterval(() => {
      // Check if signed pre-key needs rotation
      if (this.signedPreKey && this.signedPreKey.expiresAt < Date.now()) {
        this.rotateSignedPreKey();
      }

      // Replenish one-time pre-keys
      const unusedCount = Array.from(this.oneTimePreKeys.values())
        .filter((k) => !k.used).length;

      if (unusedCount < this.config.oneTimePreKeyCount! / 2) {
        this.replenishOneTimePreKeys();
      }

      // Cleanup expired sessions
      this.cleanupSessions();
    }, 60 * 60 * 1000); // Every hour
  }

  /**
   * Get service statistics
   */
  getStats(): KeyExchangeStats {
    return {
      activeSession: this.sessions.size,
      totalExchanges: this.sessions.size, // Simplified
      preKeysRemaining: Array.from(this.oneTimePreKeys.values())
        .filter((k) => !k.used).length,
      lastRotation: this.signedPreKey?.createdAt ?? 0,
    };
  }

  /**
   * Stop the service
   */
  stop(): void {
    if (this.rotationInterval) {
      clearInterval(this.rotationInterval);
      this.rotationInterval = null;
    }
  }
}

/**
 * Create key exchange service
 */
export function createKeyExchangeService(
  client?: MycelixMailClient,
  config?: Partial<KeyExchangeConfig>
): KeyExchangeService {
  return new KeyExchangeService(client, config);
}

export default KeyExchangeService;
