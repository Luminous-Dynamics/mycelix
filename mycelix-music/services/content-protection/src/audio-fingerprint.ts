// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Audio Fingerprinting Service
 *
 * Generates and matches audio fingerprints for content identification,
 * copyright protection, and royalty tracking.
 */

import { Pool } from 'pg';
import { Redis } from 'ioredis';
import * as crypto from 'crypto';

// ============================================================================
// Types
// ============================================================================

export interface AudioFingerprint {
  id: string;
  trackId: string;
  ownerId: string;
  fingerprint: number[];      // Chromaprint-style fingerprint
  duration: number;
  hash: string;               // Quick lookup hash
  metadata: {
    title: string;
    artist: string;
    album?: string;
    isrc?: string;
    releaseDate?: Date;
  };
  registeredAt: Date;
}

export interface FingerprintMatch {
  trackId: string;
  ownerId: string;
  confidence: number;         // 0-1
  matchedSegments: {
    queryStart: number;
    queryEnd: number;
    referenceStart: number;
    referenceEnd: number;
    confidence: number;
  }[];
  metadata: AudioFingerprint['metadata'];
}

export interface ContentMatch {
  matchId: string;
  sourceTrackId: string;
  matchedTrackId: string;
  matchType: 'exact' | 'sample' | 'cover' | 'remix';
  confidence: number;
  duration: number;
  sourceTimestamp: number;
  matchedTimestamp: number;
  status: 'pending' | 'confirmed' | 'disputed' | 'dismissed';
  createdAt: Date;
}

export interface CopyrightClaim {
  id: string;
  claimantId: string;
  targetTrackId: string;
  targetOwnerId: string;
  matchId: string;
  claimType: 'takedown' | 'monetize' | 'track';
  status: 'pending' | 'accepted' | 'disputed' | 'rejected';
  evidence: string[];
  resolution?: {
    action: 'removed' | 'licensed' | 'split_revenue' | 'dismissed';
    details: string;
    resolvedAt: Date;
  };
  createdAt: Date;
}

// ============================================================================
// Audio Fingerprinting Service
// ============================================================================

export class AudioFingerprintService {
  private db: Pool;
  private redis: Redis;

  // Fingerprint parameters
  private readonly SAMPLE_RATE = 11025;
  private readonly FRAME_SIZE = 4096;
  private readonly OVERLAP = 2048;
  private readonly NUM_BANDS = 32;

  constructor(
    private readonly config: {
      databaseUrl: string;
      redisUrl: string;
    }
  ) {
    this.db = new Pool({ connectionString: config.databaseUrl });
    this.redis = new Redis(config.redisUrl);
  }

  // ============================================================================
  // Fingerprint Generation
  // ============================================================================

  async generateFingerprint(
    audioData: Float32Array,
    sampleRate: number
  ): Promise<number[]> {
    // Resample to standard rate if needed
    const resampled = sampleRate !== this.SAMPLE_RATE
      ? this.resample(audioData, sampleRate, this.SAMPLE_RATE)
      : audioData;

    // Generate spectrogram
    const spectrogram = this.computeSpectrogram(resampled);

    // Extract fingerprint from spectrogram
    const fingerprint = this.extractFingerprint(spectrogram);

    return fingerprint;
  }

  private resample(data: Float32Array, fromRate: number, toRate: number): Float32Array {
    const ratio = fromRate / toRate;
    const newLength = Math.floor(data.length / ratio);
    const result = new Float32Array(newLength);

    for (let i = 0; i < newLength; i++) {
      const srcIndex = i * ratio;
      const srcIndexFloor = Math.floor(srcIndex);
      const srcIndexCeil = Math.min(srcIndexFloor + 1, data.length - 1);
      const fraction = srcIndex - srcIndexFloor;

      result[i] = data[srcIndexFloor] * (1 - fraction) + data[srcIndexCeil] * fraction;
    }

    return result;
  }

  private computeSpectrogram(audio: Float32Array): Float32Array[] {
    const numFrames = Math.floor((audio.length - this.FRAME_SIZE) / this.OVERLAP) + 1;
    const spectrogram: Float32Array[] = [];

    for (let i = 0; i < numFrames; i++) {
      const start = i * this.OVERLAP;
      const frame = audio.slice(start, start + this.FRAME_SIZE);

      // Apply Hanning window
      const windowed = this.applyWindow(frame);

      // Compute FFT magnitude
      const spectrum = this.computeFFT(windowed);

      // Group into frequency bands
      const bands = this.groupIntoBands(spectrum);

      spectrogram.push(bands);
    }

    return spectrogram;
  }

  private applyWindow(frame: Float32Array): Float32Array {
    const result = new Float32Array(frame.length);
    for (let i = 0; i < frame.length; i++) {
      const window = 0.5 * (1 - Math.cos((2 * Math.PI * i) / (frame.length - 1)));
      result[i] = frame[i] * window;
    }
    return result;
  }

  private computeFFT(data: Float32Array): Float32Array {
    // Simplified FFT - would use proper FFT library in production
    const N = data.length;
    const result = new Float32Array(N / 2);

    for (let k = 0; k < N / 2; k++) {
      let real = 0;
      let imag = 0;

      for (let n = 0; n < N; n++) {
        const angle = (2 * Math.PI * k * n) / N;
        real += data[n] * Math.cos(angle);
        imag -= data[n] * Math.sin(angle);
      }

      result[k] = Math.sqrt(real * real + imag * imag);
    }

    return result;
  }

  private groupIntoBands(spectrum: Float32Array): Float32Array {
    const bands = new Float32Array(this.NUM_BANDS);
    const bandSize = Math.floor(spectrum.length / this.NUM_BANDS);

    for (let b = 0; b < this.NUM_BANDS; b++) {
      let sum = 0;
      for (let i = 0; i < bandSize; i++) {
        sum += spectrum[b * bandSize + i];
      }
      bands[b] = sum / bandSize;
    }

    return bands;
  }

  private extractFingerprint(spectrogram: Float32Array[]): number[] {
    const fingerprint: number[] = [];

    for (let i = 1; i < spectrogram.length; i++) {
      let bits = 0;

      for (let b = 0; b < this.NUM_BANDS; b++) {
        // Compare with previous frame - positive difference = 1, negative = 0
        if (spectrogram[i][b] > spectrogram[i - 1][b]) {
          bits |= 1 << b;
        }
      }

      fingerprint.push(bits);
    }

    return fingerprint;
  }

  // ============================================================================
  // Fingerprint Registration
  // ============================================================================

  async registerFingerprint(
    trackId: string,
    ownerId: string,
    audioData: Float32Array,
    sampleRate: number,
    metadata: AudioFingerprint['metadata']
  ): Promise<AudioFingerprint> {
    // Generate fingerprint
    const fingerprint = await this.generateFingerprint(audioData, sampleRate);

    // Create hash for quick lookup
    const hash = this.hashFingerprint(fingerprint);

    // Check for existing matches first
    const existingMatches = await this.findMatches(fingerprint, 0.9);
    if (existingMatches.length > 0) {
      throw new Error(`Content matches existing track: ${existingMatches[0].trackId}`);
    }

    // Store in database
    const result = await this.db.query(`
      INSERT INTO audio_fingerprints (
        track_id, owner_id, fingerprint, duration, hash, metadata, registered_at
      ) VALUES ($1, $2, $3, $4, $5, $6, NOW())
      RETURNING *
    `, [
      trackId,
      ownerId,
      JSON.stringify(fingerprint),
      audioData.length / sampleRate,
      hash,
      JSON.stringify(metadata),
    ]);

    // Index fingerprint segments in Redis for fast lookup
    await this.indexFingerprint(trackId, fingerprint);

    return this.mapRowToFingerprint(result.rows[0]);
  }

  private hashFingerprint(fingerprint: number[]): string {
    // Create compact hash from fingerprint
    const buffer = Buffer.alloc(fingerprint.length * 4);
    fingerprint.forEach((v, i) => buffer.writeUInt32LE(v, i * 4));
    return crypto.createHash('sha256').update(buffer).digest('hex');
  }

  private async indexFingerprint(trackId: string, fingerprint: number[]): Promise<void> {
    const pipeline = this.redis.pipeline();

    // Index chunks of fingerprint for lookup
    const chunkSize = 10;
    for (let i = 0; i < fingerprint.length - chunkSize; i += chunkSize / 2) {
      const chunk = fingerprint.slice(i, i + chunkSize);
      const chunkHash = this.hashFingerprint(chunk);

      // Store track ID for this chunk
      pipeline.sadd(`fp:chunk:${chunkHash}`, `${trackId}:${i}`);
      pipeline.expire(`fp:chunk:${chunkHash}`, 86400 * 365); // 1 year
    }

    await pipeline.exec();
  }

  // ============================================================================
  // Fingerprint Matching
  // ============================================================================

  async findMatches(
    queryFingerprint: number[],
    minConfidence = 0.7
  ): Promise<FingerprintMatch[]> {
    // Find candidate tracks using chunk lookup
    const candidates = await this.findCandidates(queryFingerprint);

    if (candidates.size === 0) {
      return [];
    }

    // Load full fingerprints for candidates
    const candidateFps = await this.db.query(`
      SELECT * FROM audio_fingerprints WHERE track_id = ANY($1)
    `, [Array.from(candidates)]);

    // Compare with each candidate
    const matches: FingerprintMatch[] = [];

    for (const row of candidateFps.rows) {
      const refFingerprint = JSON.parse(row.fingerprint) as number[];
      const matchResult = this.compareFingerprints(queryFingerprint, refFingerprint);

      if (matchResult.confidence >= minConfidence) {
        matches.push({
          trackId: row.track_id,
          ownerId: row.owner_id,
          confidence: matchResult.confidence,
          matchedSegments: matchResult.segments,
          metadata: JSON.parse(row.metadata),
        });
      }
    }

    // Sort by confidence
    matches.sort((a, b) => b.confidence - a.confidence);

    return matches;
  }

  private async findCandidates(fingerprint: number[]): Promise<Set<string>> {
    const candidates = new Set<string>();
    const chunkSize = 10;

    for (let i = 0; i < fingerprint.length - chunkSize; i += chunkSize / 2) {
      const chunk = fingerprint.slice(i, i + chunkSize);
      const chunkHash = this.hashFingerprint(chunk);

      const matches = await this.redis.smembers(`fp:chunk:${chunkHash}`);
      for (const match of matches) {
        const trackId = match.split(':')[0];
        candidates.add(trackId);
      }

      // Early exit if we have enough candidates
      if (candidates.size > 100) break;
    }

    return candidates;
  }

  private compareFingerprints(
    query: number[],
    reference: number[]
  ): { confidence: number; segments: FingerprintMatch['matchedSegments'] } {
    const segments: FingerprintMatch['matchedSegments'] = [];
    let totalMatched = 0;

    // Sliding window comparison
    const windowSize = 20;
    const step = 10;

    for (let qStart = 0; qStart < query.length - windowSize; qStart += step) {
      const queryWindow = query.slice(qStart, qStart + windowSize);

      let bestMatchPos = -1;
      let bestMatchScore = 0;

      for (let rStart = 0; rStart < reference.length - windowSize; rStart += step) {
        const refWindow = reference.slice(rStart, rStart + windowSize);
        const score = this.windowSimilarity(queryWindow, refWindow);

        if (score > bestMatchScore) {
          bestMatchScore = score;
          bestMatchPos = rStart;
        }
      }

      if (bestMatchScore > 0.8) {
        segments.push({
          queryStart: qStart,
          queryEnd: qStart + windowSize,
          referenceStart: bestMatchPos,
          referenceEnd: bestMatchPos + windowSize,
          confidence: bestMatchScore,
        });
        totalMatched += windowSize;
      }
    }

    // Merge overlapping segments
    const mergedSegments = this.mergeSegments(segments);

    const confidence = totalMatched / query.length;

    return { confidence, segments: mergedSegments };
  }

  private windowSimilarity(a: number[], b: number[]): number {
    let matches = 0;
    for (let i = 0; i < a.length; i++) {
      // Count matching bits using XOR and popcount
      const xor = a[i] ^ b[i];
      const diffBits = this.popcount(xor);
      const matchBits = this.NUM_BANDS - diffBits;
      matches += matchBits / this.NUM_BANDS;
    }
    return matches / a.length;
  }

  private popcount(n: number): number {
    let count = 0;
    while (n) {
      count += n & 1;
      n >>>= 1;
    }
    return count;
  }

  private mergeSegments(
    segments: FingerprintMatch['matchedSegments']
  ): FingerprintMatch['matchedSegments'] {
    if (segments.length === 0) return [];

    segments.sort((a, b) => a.queryStart - b.queryStart);

    const merged: FingerprintMatch['matchedSegments'] = [segments[0]];

    for (let i = 1; i < segments.length; i++) {
      const current = segments[i];
      const last = merged[merged.length - 1];

      if (current.queryStart <= last.queryEnd + 5) {
        // Merge
        last.queryEnd = Math.max(last.queryEnd, current.queryEnd);
        last.referenceEnd = Math.max(last.referenceEnd, current.referenceEnd);
        last.confidence = (last.confidence + current.confidence) / 2;
      } else {
        merged.push(current);
      }
    }

    return merged;
  }

  // ============================================================================
  // Content Scanning
  // ============================================================================

  async scanTrack(
    trackId: string,
    audioData: Float32Array,
    sampleRate: number
  ): Promise<ContentMatch[]> {
    const fingerprint = await this.generateFingerprint(audioData, sampleRate);
    const matches = await this.findMatches(fingerprint, 0.5);

    const contentMatches: ContentMatch[] = [];

    for (const match of matches) {
      if (match.trackId === trackId) continue; // Skip self-match

      // Determine match type
      let matchType: ContentMatch['matchType'] = 'exact';
      if (match.confidence < 0.9 && match.confidence >= 0.7) {
        matchType = 'sample';
      } else if (match.confidence < 0.7) {
        matchType = 'cover';
      }

      const contentMatch: ContentMatch = {
        matchId: crypto.randomUUID(),
        sourceTrackId: trackId,
        matchedTrackId: match.trackId,
        matchType,
        confidence: match.confidence,
        duration: match.matchedSegments.reduce(
          (sum, s) => sum + (s.queryEnd - s.queryStart),
          0
        ) / this.SAMPLE_RATE,
        sourceTimestamp: match.matchedSegments[0]?.queryStart || 0,
        matchedTimestamp: match.matchedSegments[0]?.referenceStart || 0,
        status: 'pending',
        createdAt: new Date(),
      };

      // Store match
      await this.db.query(`
        INSERT INTO content_matches (
          match_id, source_track_id, matched_track_id, match_type,
          confidence, duration, source_timestamp, matched_timestamp,
          status, created_at
        ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, NOW())
      `, [
        contentMatch.matchId,
        contentMatch.sourceTrackId,
        contentMatch.matchedTrackId,
        contentMatch.matchType,
        contentMatch.confidence,
        contentMatch.duration,
        contentMatch.sourceTimestamp,
        contentMatch.matchedTimestamp,
        contentMatch.status,
      ]);

      contentMatches.push(contentMatch);
    }

    return contentMatches;
  }

  // ============================================================================
  // Copyright Claims
  // ============================================================================

  async createClaim(
    claimantId: string,
    targetTrackId: string,
    matchId: string,
    claimType: CopyrightClaim['claimType'],
    evidence: string[]
  ): Promise<CopyrightClaim> {
    // Verify match exists
    const match = await this.db.query(
      'SELECT * FROM content_matches WHERE match_id = $1',
      [matchId]
    );
    if (match.rows.length === 0) {
      throw new Error('Match not found');
    }

    // Get target track owner
    const target = await this.db.query(
      'SELECT owner_id FROM audio_fingerprints WHERE track_id = $1',
      [targetTrackId]
    );
    const targetOwnerId = target.rows[0]?.owner_id;

    const result = await this.db.query(`
      INSERT INTO copyright_claims (
        claimant_id, target_track_id, target_owner_id, match_id,
        claim_type, status, evidence, created_at
      ) VALUES ($1, $2, $3, $4, $5, 'pending', $6, NOW())
      RETURNING *
    `, [claimantId, targetTrackId, targetOwnerId, matchId, claimType, evidence]);

    // Notify target owner
    await this.redis.publish('notifications', JSON.stringify({
      type: 'copyright_claim',
      userId: targetOwnerId,
      data: { claimId: result.rows[0].id, trackId: targetTrackId, claimType },
    }));

    return this.mapRowToClaim(result.rows[0]);
  }

  async disputeClaim(
    claimId: string,
    disputerId: string,
    reason: string,
    evidence: string[]
  ): Promise<void> {
    await this.db.query(`
      UPDATE copyright_claims
      SET status = 'disputed',
          dispute_reason = $3,
          dispute_evidence = $4,
          disputed_at = NOW()
      WHERE id = $1 AND target_owner_id = $2
    `, [claimId, disputerId, reason, evidence]);
  }

  async resolveClaim(
    claimId: string,
    action: CopyrightClaim['resolution']['action'],
    details: string
  ): Promise<void> {
    await this.db.query(`
      UPDATE copyright_claims
      SET status = $2,
          resolution = $3,
          resolved_at = NOW()
      WHERE id = $1
    `, [
      claimId,
      action === 'dismissed' ? 'rejected' : 'accepted',
      JSON.stringify({ action, details, resolvedAt: new Date() }),
    ]);
  }

  // ============================================================================
  // Helpers
  // ============================================================================

  private mapRowToFingerprint(row: any): AudioFingerprint {
    return {
      id: row.id,
      trackId: row.track_id,
      ownerId: row.owner_id,
      fingerprint: JSON.parse(row.fingerprint),
      duration: row.duration,
      hash: row.hash,
      metadata: JSON.parse(row.metadata),
      registeredAt: new Date(row.registered_at),
    };
  }

  private mapRowToClaim(row: any): CopyrightClaim {
    return {
      id: row.id,
      claimantId: row.claimant_id,
      targetTrackId: row.target_track_id,
      targetOwnerId: row.target_owner_id,
      matchId: row.match_id,
      claimType: row.claim_type,
      status: row.status,
      evidence: row.evidence,
      resolution: row.resolution ? JSON.parse(row.resolution) : undefined,
      createdAt: new Date(row.created_at),
    };
  }
}

export default AudioFingerprintService;
