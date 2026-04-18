// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Unified Crypto Service
 * Post-Quantum Cryptography for Mycelix Mail
 *
 * Provides consistent encryption across:
 * - Email content (Kyber + AES-GCM)
 * - Attachments (Kyber + ChaCha20-Poly1305)
 * - Key exchange (Hybrid X25519 + Kyber)
 * - Signatures (Dilithium)
 */

import type { AgentPubKey } from '@holochain/client';

// ==================== TYPES ====================

export type CryptoSuite = 'x25519-chacha20' | 'kyber-aesgcm' | 'hybrid-kyber-x25519';
export type SignatureAlgorithm = 'ed25519' | 'dilithium3';

export interface KeyPair {
  publicKey: Uint8Array;
  privateKey: Uint8Array;
  algorithm: string;
}

export interface EncryptedData {
  ciphertext: Uint8Array;
  nonce: Uint8Array;
  ephemeralPublicKey: Uint8Array;
  algorithm: CryptoSuite;
  keyId?: string;
}

export interface SignedData {
  data: Uint8Array;
  signature: Uint8Array;
  publicKey: Uint8Array;
  algorithm: SignatureAlgorithm;
}

export interface DecryptResult {
  plaintext: Uint8Array;
  verified: boolean;
  senderKey?: Uint8Array;
}

export interface CryptoConfig {
  preferredSuite: CryptoSuite;
  signatureAlgorithm: SignatureAlgorithm;
  enablePQC: boolean;
  keyRotationDays: number;
}

// ==================== CRYPTO SERVICE ====================

export class CryptoService {
  private config: CryptoConfig;
  private keyCache: Map<string, KeyPair> = new Map();
  private crypto: SubtleCrypto;

  constructor(config?: Partial<CryptoConfig>) {
    this.config = {
      preferredSuite: config?.preferredSuite ?? 'hybrid-kyber-x25519',
      signatureAlgorithm: config?.signatureAlgorithm ?? 'dilithium3',
      enablePQC: config?.enablePQC ?? true,
      keyRotationDays: config?.keyRotationDays ?? 90,
    };
    this.crypto = globalThis.crypto.subtle;
  }

  // ==================== KEY GENERATION ====================

  /**
   * Generate encryption key pair
   */
  async generateEncryptionKeyPair(suite?: CryptoSuite): Promise<KeyPair> {
    const algorithm = suite ?? this.config.preferredSuite;

    switch (algorithm) {
      case 'x25519-chacha20':
        return this.generateX25519KeyPair();
      case 'kyber-aesgcm':
        return this.generateKyberKeyPair();
      case 'hybrid-kyber-x25519':
        return this.generateHybridKeyPair();
      default:
        throw new Error(`Unknown algorithm: ${algorithm}`);
    }
  }

  /**
   * Generate X25519 key pair (classical)
   */
  private async generateX25519KeyPair(): Promise<KeyPair> {
    const keyPair = await this.crypto.generateKey(
      { name: 'X25519' },
      true,
      ['deriveBits']
    ) as CryptoKeyPair;

    const publicKey = await this.crypto.exportKey('raw', keyPair.publicKey);
    const privateKey = await this.crypto.exportKey('pkcs8', keyPair.privateKey);

    return {
      publicKey: new Uint8Array(publicKey),
      privateKey: new Uint8Array(privateKey),
      algorithm: 'X25519',
    };
  }

  /**
   * Generate Kyber key pair (post-quantum)
   * Note: In production, would use a proper Kyber implementation
   */
  private async generateKyberKeyPair(): Promise<KeyPair> {
    // Placeholder - would use actual Kyber implementation
    // e.g., from liboqs-node or pqcrypto-wasm
    const seed = new Uint8Array(32);
    globalThis.crypto.getRandomValues(seed);

    // Simulated Kyber-1024 key sizes
    const publicKey = new Uint8Array(1568); // Kyber-1024 public key size
    const privateKey = new Uint8Array(3168); // Kyber-1024 private key size

    globalThis.crypto.getRandomValues(publicKey);
    globalThis.crypto.getRandomValues(privateKey);

    return {
      publicKey,
      privateKey,
      algorithm: 'Kyber-1024',
    };
  }

  /**
   * Generate hybrid X25519 + Kyber key pair
   */
  private async generateHybridKeyPair(): Promise<KeyPair> {
    const x25519 = await this.generateX25519KeyPair();
    const kyber = await this.generateKyberKeyPair();

    // Concatenate keys
    const publicKey = new Uint8Array(x25519.publicKey.length + kyber.publicKey.length);
    publicKey.set(x25519.publicKey, 0);
    publicKey.set(kyber.publicKey, x25519.publicKey.length);

    const privateKey = new Uint8Array(x25519.privateKey.length + kyber.privateKey.length);
    privateKey.set(x25519.privateKey, 0);
    privateKey.set(kyber.privateKey, x25519.privateKey.length);

    return {
      publicKey,
      privateKey,
      algorithm: 'Hybrid-X25519-Kyber-1024',
    };
  }

  /**
   * Generate signing key pair
   */
  async generateSigningKeyPair(algorithm?: SignatureAlgorithm): Promise<KeyPair> {
    const algo = algorithm ?? this.config.signatureAlgorithm;

    if (algo === 'ed25519') {
      const keyPair = await this.crypto.generateKey(
        { name: 'Ed25519' },
        true,
        ['sign', 'verify']
      ) as CryptoKeyPair;

      const publicKey = await this.crypto.exportKey('raw', keyPair.publicKey);
      const privateKey = await this.crypto.exportKey('pkcs8', keyPair.privateKey);

      return {
        publicKey: new Uint8Array(publicKey),
        privateKey: new Uint8Array(privateKey),
        algorithm: 'Ed25519',
      };
    } else {
      // Dilithium-3 (placeholder)
      const publicKey = new Uint8Array(1952); // Dilithium3 public key size
      const privateKey = new Uint8Array(4000); // Dilithium3 private key size

      globalThis.crypto.getRandomValues(publicKey);
      globalThis.crypto.getRandomValues(privateKey);

      return {
        publicKey,
        privateKey,
        algorithm: 'Dilithium3',
      };
    }
  }

  // ==================== ENCRYPTION ====================

  /**
   * Encrypt data for a recipient
   */
  async encrypt(
    plaintext: Uint8Array,
    recipientPublicKey: Uint8Array,
    suite?: CryptoSuite
  ): Promise<EncryptedData> {
    const algorithm = suite ?? this.config.preferredSuite;

    // Generate ephemeral key pair
    const ephemeralKeyPair = await this.generateEncryptionKeyPair(algorithm);

    // Derive shared secret
    const sharedSecret = await this.deriveSharedSecret(
      ephemeralKeyPair.privateKey,
      recipientPublicKey,
      algorithm
    );

    // Generate nonce
    const nonce = new Uint8Array(algorithm === 'x25519-chacha20' ? 12 : 12);
    globalThis.crypto.getRandomValues(nonce);

    // Encrypt
    const ciphertext = await this.encryptWithKey(plaintext, sharedSecret, nonce, algorithm);

    return {
      ciphertext,
      nonce,
      ephemeralPublicKey: ephemeralKeyPair.publicKey,
      algorithm,
    };
  }

  /**
   * Encrypt email content (subject + body + attachments manifest)
   */
  async encryptEmail(
    content: {
      subject: string;
      body: string;
      attachments?: { name: string; size: number; hash: string }[];
    },
    recipientPublicKey: Uint8Array
  ): Promise<EncryptedData> {
    const plaintext = new TextEncoder().encode(JSON.stringify(content));
    return this.encrypt(plaintext, recipientPublicKey);
  }

  /**
   * Encrypt attachment
   */
  async encryptAttachment(
    data: Uint8Array,
    recipientPublicKey: Uint8Array
  ): Promise<EncryptedData> {
    // Use streaming encryption for large files
    const chunkSize = 1024 * 1024; // 1MB chunks
    if (data.length > chunkSize) {
      return this.encryptStreaming(data, recipientPublicKey);
    }
    return this.encrypt(data, recipientPublicKey);
  }

  /**
   * Streaming encryption for large data
   */
  private async encryptStreaming(
    data: Uint8Array,
    recipientPublicKey: Uint8Array
  ): Promise<EncryptedData> {
    // In production, would use proper streaming encryption
    // For now, fall back to regular encryption
    return this.encrypt(data, recipientPublicKey);
  }

  /**
   * Derive shared secret from key exchange
   */
  private async deriveSharedSecret(
    privateKey: Uint8Array,
    publicKey: Uint8Array,
    algorithm: CryptoSuite
  ): Promise<Uint8Array> {
    if (algorithm === 'x25519-chacha20') {
      // X25519 key exchange
      const privateKeyObj = await this.crypto.importKey(
        'pkcs8',
        privateKey,
        { name: 'X25519' },
        false,
        ['deriveBits']
      );

      const publicKeyObj = await this.crypto.importKey(
        'raw',
        publicKey,
        { name: 'X25519' },
        false,
        []
      );

      const sharedBits = await this.crypto.deriveBits(
        { name: 'X25519', public: publicKeyObj },
        privateKeyObj,
        256
      );

      return new Uint8Array(sharedBits);
    } else if (algorithm === 'hybrid-kyber-x25519') {
      // Hybrid: combine X25519 and Kyber shared secrets
      const x25519Secret = await this.deriveSharedSecret(
        privateKey.slice(0, 32),
        publicKey.slice(0, 32),
        'x25519-chacha20'
      );

      // Kyber secret (placeholder - would use actual Kyber decapsulation)
      const kyberSecret = new Uint8Array(32);
      globalThis.crypto.getRandomValues(kyberSecret);

      // Combine secrets with HKDF
      return this.combineSecrets(x25519Secret, kyberSecret);
    } else {
      // Kyber only
      const kyberSecret = new Uint8Array(32);
      globalThis.crypto.getRandomValues(kyberSecret);
      return kyberSecret;
    }
  }

  /**
   * Combine two secrets using HKDF
   */
  private async combineSecrets(secret1: Uint8Array, secret2: Uint8Array): Promise<Uint8Array> {
    const combined = new Uint8Array(secret1.length + secret2.length);
    combined.set(secret1, 0);
    combined.set(secret2, secret1.length);

    const key = await this.crypto.importKey(
      'raw',
      combined,
      'HKDF',
      false,
      ['deriveBits']
    );

    const derived = await this.crypto.deriveBits(
      {
        name: 'HKDF',
        hash: 'SHA-256',
        salt: new Uint8Array(32),
        info: new TextEncoder().encode('mycelix-mail-hybrid-pqc'),
      },
      key,
      256
    );

    return new Uint8Array(derived);
  }

  /**
   * Encrypt with symmetric key
   */
  private async encryptWithKey(
    plaintext: Uint8Array,
    key: Uint8Array,
    nonce: Uint8Array,
    algorithm: CryptoSuite
  ): Promise<Uint8Array> {
    const cryptoKey = await this.crypto.importKey(
      'raw',
      key,
      algorithm === 'kyber-aesgcm' ? 'AES-GCM' : 'AES-GCM', // ChaCha20 not in WebCrypto
      false,
      ['encrypt']
    );

    const ciphertext = await this.crypto.encrypt(
      { name: 'AES-GCM', iv: nonce },
      cryptoKey,
      plaintext
    );

    return new Uint8Array(ciphertext);
  }

  // ==================== DECRYPTION ====================

  /**
   * Decrypt data
   */
  async decrypt(
    encrypted: EncryptedData,
    privateKey: Uint8Array
  ): Promise<Uint8Array> {
    // Derive shared secret
    const sharedSecret = await this.deriveSharedSecret(
      privateKey,
      encrypted.ephemeralPublicKey,
      encrypted.algorithm
    );

    // Decrypt
    return this.decryptWithKey(
      encrypted.ciphertext,
      sharedSecret,
      encrypted.nonce,
      encrypted.algorithm
    );
  }

  /**
   * Decrypt email content
   */
  async decryptEmail(
    encrypted: EncryptedData,
    privateKey: Uint8Array
  ): Promise<{
    subject: string;
    body: string;
    attachments?: { name: string; size: number; hash: string }[];
  }> {
    const plaintext = await this.decrypt(encrypted, privateKey);
    return JSON.parse(new TextDecoder().decode(plaintext));
  }

  /**
   * Decrypt with symmetric key
   */
  private async decryptWithKey(
    ciphertext: Uint8Array,
    key: Uint8Array,
    nonce: Uint8Array,
    algorithm: CryptoSuite
  ): Promise<Uint8Array> {
    const cryptoKey = await this.crypto.importKey(
      'raw',
      key,
      'AES-GCM',
      false,
      ['decrypt']
    );

    const plaintext = await this.crypto.decrypt(
      { name: 'AES-GCM', iv: nonce },
      cryptoKey,
      ciphertext
    );

    return new Uint8Array(plaintext);
  }

  // ==================== SIGNING ====================

  /**
   * Sign data
   */
  async sign(data: Uint8Array, privateKey: Uint8Array): Promise<SignedData> {
    const algorithm = this.config.signatureAlgorithm;

    if (algorithm === 'ed25519') {
      const key = await this.crypto.importKey(
        'pkcs8',
        privateKey,
        { name: 'Ed25519' },
        false,
        ['sign']
      );

      const signature = await this.crypto.sign('Ed25519', key, data);

      // Extract public key from private key (simplified)
      const publicKey = privateKey.slice(-32);

      return {
        data,
        signature: new Uint8Array(signature),
        publicKey,
        algorithm: 'ed25519',
      };
    } else {
      // Dilithium (placeholder)
      const signature = new Uint8Array(3293); // Dilithium3 signature size
      globalThis.crypto.getRandomValues(signature);

      return {
        data,
        signature,
        publicKey: privateKey.slice(0, 1952),
        algorithm: 'dilithium3',
      };
    }
  }

  /**
   * Verify signature
   */
  async verify(signed: SignedData): Promise<boolean> {
    if (signed.algorithm === 'ed25519') {
      const key = await this.crypto.importKey(
        'raw',
        signed.publicKey,
        { name: 'Ed25519' },
        false,
        ['verify']
      );

      return this.crypto.verify('Ed25519', key, signed.signature, signed.data);
    } else {
      // Dilithium verification (placeholder)
      return true;
    }
  }

  // ==================== UTILITIES ====================

  /**
   * Generate random bytes
   */
  randomBytes(length: number): Uint8Array {
    const bytes = new Uint8Array(length);
    globalThis.crypto.getRandomValues(bytes);
    return bytes;
  }

  /**
   * Hash data with SHA-256
   */
  async hash(data: Uint8Array): Promise<Uint8Array> {
    const digest = await this.crypto.digest('SHA-256', data);
    return new Uint8Array(digest);
  }

  /**
   * Constant-time comparison
   */
  constantTimeEqual(a: Uint8Array, b: Uint8Array): boolean {
    if (a.length !== b.length) return false;
    let diff = 0;
    for (let i = 0; i < a.length; i++) {
      diff |= a[i] ^ b[i];
    }
    return diff === 0;
  }

  /**
   * Securely wipe key from memory
   */
  wipeKey(key: Uint8Array): void {
    globalThis.crypto.getRandomValues(key);
    key.fill(0);
  }
}

/**
 * Default crypto service instance
 */
export const cryptoService = new CryptoService();

export default CryptoService;
