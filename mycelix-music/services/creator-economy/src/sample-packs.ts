// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Sample Pack Service
 *
 * Manages sample packs, loop libraries, and beat marketplace.
 * Handles creation, licensing, and sales of audio samples.
 */

import { Pool } from 'pg';
import { Redis } from 'ioredis';
import { S3Client, PutObjectCommand } from '@aws-sdk/client-s3';

// ============================================================================
// Types
// ============================================================================

export interface SamplePack {
  id: string;
  creatorId: string;
  title: string;
  description: string;
  coverArt: string;
  samples: Sample[];
  genres: string[];
  tags: string[];
  bpmRange: { min: number; max: number };
  keySignatures: string[];
  price: number;
  currency: 'USD' | 'ETH';
  licenseType: LicenseType;
  salesCount: number;
  rating: number;
  ratingCount: number;
  createdAt: Date;
  updatedAt: Date;
  isPublished: boolean;
  isFeatured: boolean;
}

export interface Sample {
  id: string;
  packId: string;
  name: string;
  category: SampleCategory;
  audioUrl: string;
  previewUrl: string;      // Lower quality preview
  duration: number;
  bpm: number | null;
  key: string | null;
  waveformData: number[];
  format: 'wav' | 'aiff' | 'mp3';
  sampleRate: number;
  bitDepth: number;
  isLoop: boolean;
  stems?: string[];        // If sample has stems
}

export type SampleCategory =
  | 'drums'
  | 'bass'
  | 'synth'
  | 'vocals'
  | 'fx'
  | 'guitar'
  | 'keys'
  | 'strings'
  | 'brass'
  | 'world'
  | 'foley'
  | 'one-shot'
  | 'loop'
  | 'construction-kit';

export type LicenseType =
  | 'royalty-free'        // One-time purchase, unlimited use
  | 'lease'               // Limited uses/streams
  | 'exclusive'           // Exclusive rights
  | 'creative-commons';   // Various CC licenses

export interface Beat {
  id: string;
  producerId: string;
  title: string;
  coverArt: string;
  audioUrl: string;
  previewUrl: string;
  duration: number;
  bpm: number;
  key: string;
  genres: string[];
  tags: string[];
  mood: string[];
  licenses: BeatLicense[];
  stems?: BeatStem[];
  salesCount: number;
  playCount: number;
  rating: number;
  createdAt: Date;
  isPublished: boolean;
}

export interface BeatLicense {
  id: string;
  type: 'mp3' | 'wav' | 'trackout' | 'unlimited' | 'exclusive';
  name: string;
  price: number;
  features: string[];
  streamLimit?: number;
  salesLimit?: number;
  performanceLimit?: number;
  musicVideoAllowed: boolean;
  profitSharingRequired: boolean;
  profitSharingPercent?: number;
}

export interface BeatStem {
  name: string;
  url: string;
  category: string;
}

export interface CollaborationMatch {
  userId: string;
  displayName: string;
  avatar: string;
  skills: string[];
  genres: string[];
  matchScore: number;
  matchReasons: string[];
  recentWork: string[];
  isVerified: boolean;
}

// ============================================================================
// Sample Pack Service
// ============================================================================

export class SamplePackService {
  private db: Pool;
  private redis: Redis;
  private s3: S3Client;

  constructor(
    private readonly config: {
      databaseUrl: string;
      redisUrl: string;
      s3Bucket: string;
      s3Region: string;
    }
  ) {
    this.db = new Pool({ connectionString: config.databaseUrl });
    this.redis = new Redis(config.redisUrl);
    this.s3 = new S3Client({ region: config.s3Region });
  }

  // ============================================================================
  // Sample Pack CRUD
  // ============================================================================

  async createSamplePack(
    creatorId: string,
    data: {
      title: string;
      description: string;
      genres: string[];
      tags: string[];
      price: number;
      currency: 'USD' | 'ETH';
      licenseType: LicenseType;
    }
  ): Promise<SamplePack> {
    const result = await this.db.query(`
      INSERT INTO sample_packs (
        creator_id, title, description, genres, tags,
        price, currency, license_type, created_at, updated_at
      ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, NOW(), NOW())
      RETURNING *
    `, [
      creatorId,
      data.title,
      data.description,
      data.genres,
      data.tags,
      data.price,
      data.currency,
      data.licenseType,
    ]);

    return this.mapRowToSamplePack(result.rows[0]);
  }

  async uploadSample(
    packId: string,
    file: Buffer,
    metadata: {
      name: string;
      category: SampleCategory;
      bpm?: number;
      key?: string;
      isLoop: boolean;
    }
  ): Promise<Sample> {
    // Generate waveform data
    const waveformData = await this.generateWaveform(file);

    // Detect format and properties
    const audioProps = await this.analyzeAudio(file);

    // Upload full quality to S3
    const audioKey = `samples/${packId}/${Date.now()}_${metadata.name}.wav`;
    await this.s3.send(new PutObjectCommand({
      Bucket: this.config.s3Bucket,
      Key: audioKey,
      Body: file,
      ContentType: 'audio/wav',
    }));

    // Generate and upload preview (lower quality)
    const preview = await this.generatePreview(file);
    const previewKey = `samples/${packId}/${Date.now()}_${metadata.name}_preview.mp3`;
    await this.s3.send(new PutObjectCommand({
      Bucket: this.config.s3Bucket,
      Key: previewKey,
      Body: preview,
      ContentType: 'audio/mpeg',
    }));

    const audioUrl = `https://${this.config.s3Bucket}.s3.${this.config.s3Region}.amazonaws.com/${audioKey}`;
    const previewUrl = `https://${this.config.s3Bucket}.s3.${this.config.s3Region}.amazonaws.com/${previewKey}`;

    // Store in database
    const result = await this.db.query(`
      INSERT INTO samples (
        pack_id, name, category, audio_url, preview_url,
        duration, bpm, key, waveform_data, format,
        sample_rate, bit_depth, is_loop, created_at
      ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13, NOW())
      RETURNING *
    `, [
      packId,
      metadata.name,
      metadata.category,
      audioUrl,
      previewUrl,
      audioProps.duration,
      metadata.bpm,
      metadata.key,
      JSON.stringify(waveformData),
      audioProps.format,
      audioProps.sampleRate,
      audioProps.bitDepth,
      metadata.isLoop,
    ]);

    // Update pack BPM range
    await this.updatePackBpmRange(packId);

    return this.mapRowToSample(result.rows[0]);
  }

  async publishSamplePack(packId: string): Promise<void> {
    // Verify pack has samples
    const sampleCount = await this.db.query(
      'SELECT COUNT(*) FROM samples WHERE pack_id = $1',
      [packId]
    );

    if (parseInt(sampleCount.rows[0].count) === 0) {
      throw new Error('Cannot publish pack without samples');
    }

    await this.db.query(
      'UPDATE sample_packs SET is_published = true, updated_at = NOW() WHERE id = $1',
      [packId]
    );

    // Invalidate cache
    await this.redis.del(`pack:${packId}`);
  }

  async getSamplePack(packId: string): Promise<SamplePack | null> {
    // Check cache
    const cached = await this.redis.get(`pack:${packId}`);
    if (cached) {
      return JSON.parse(cached);
    }

    const result = await this.db.query(`
      SELECT p.*, ARRAY_AGG(s.*) as samples
      FROM sample_packs p
      LEFT JOIN samples s ON p.id = s.pack_id
      WHERE p.id = $1
      GROUP BY p.id
    `, [packId]);

    if (result.rows.length === 0) return null;

    const pack = this.mapRowToSamplePack(result.rows[0]);

    // Cache for 1 hour
    await this.redis.setex(`pack:${packId}`, 3600, JSON.stringify(pack));

    return pack;
  }

  // ============================================================================
  // Beat Store
  // ============================================================================

  async createBeat(
    producerId: string,
    data: {
      title: string;
      bpm: number;
      key: string;
      genres: string[];
      tags: string[];
      mood: string[];
      licenses: Omit<BeatLicense, 'id'>[];
    },
    audioFile: Buffer,
    coverArt: Buffer
  ): Promise<Beat> {
    // Upload audio
    const audioKey = `beats/${producerId}/${Date.now()}_${data.title}.wav`;
    await this.s3.send(new PutObjectCommand({
      Bucket: this.config.s3Bucket,
      Key: audioKey,
      Body: audioFile,
      ContentType: 'audio/wav',
    }));

    // Upload cover art
    const coverKey = `beats/${producerId}/${Date.now()}_cover.jpg`;
    await this.s3.send(new PutObjectCommand({
      Bucket: this.config.s3Bucket,
      Key: coverKey,
      Body: coverArt,
      ContentType: 'image/jpeg',
    }));

    // Generate preview
    const preview = await this.generateTaggedPreview(audioFile, data.title);
    const previewKey = `beats/${producerId}/${Date.now()}_preview.mp3`;
    await this.s3.send(new PutObjectCommand({
      Bucket: this.config.s3Bucket,
      Key: previewKey,
      Body: preview,
      ContentType: 'audio/mpeg',
    }));

    const audioProps = await this.analyzeAudio(audioFile);

    const result = await this.db.query(`
      INSERT INTO beats (
        producer_id, title, cover_art, audio_url, preview_url,
        duration, bpm, key, genres, tags, mood, licenses,
        created_at, is_published
      ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, NOW(), true)
      RETURNING *
    `, [
      producerId,
      data.title,
      `https://${this.config.s3Bucket}.s3.${this.config.s3Region}.amazonaws.com/${coverKey}`,
      `https://${this.config.s3Bucket}.s3.${this.config.s3Region}.amazonaws.com/${audioKey}`,
      `https://${this.config.s3Bucket}.s3.${this.config.s3Region}.amazonaws.com/${previewKey}`,
      audioProps.duration,
      data.bpm,
      data.key,
      data.genres,
      data.tags,
      data.mood,
      JSON.stringify(data.licenses),
    ]);

    return this.mapRowToBeat(result.rows[0]);
  }

  async searchBeats(query: {
    genres?: string[];
    bpmRange?: { min: number; max: number };
    key?: string;
    mood?: string[];
    priceRange?: { min: number; max: number };
    sortBy?: 'newest' | 'popular' | 'price_low' | 'price_high';
    limit?: number;
    offset?: number;
  }): Promise<{ beats: Beat[]; total: number }> {
    let whereClause = 'WHERE is_published = true';
    const params: any[] = [];
    let paramIndex = 1;

    if (query.genres?.length) {
      whereClause += ` AND genres && $${paramIndex}`;
      params.push(query.genres);
      paramIndex++;
    }

    if (query.bpmRange) {
      whereClause += ` AND bpm BETWEEN $${paramIndex} AND $${paramIndex + 1}`;
      params.push(query.bpmRange.min, query.bpmRange.max);
      paramIndex += 2;
    }

    if (query.key) {
      whereClause += ` AND key = $${paramIndex}`;
      params.push(query.key);
      paramIndex++;
    }

    if (query.mood?.length) {
      whereClause += ` AND mood && $${paramIndex}`;
      params.push(query.mood);
      paramIndex++;
    }

    let orderBy = 'ORDER BY created_at DESC';
    if (query.sortBy === 'popular') orderBy = 'ORDER BY play_count DESC';

    const limit = query.limit || 20;
    const offset = query.offset || 0;

    const countResult = await this.db.query(
      `SELECT COUNT(*) FROM beats ${whereClause}`,
      params
    );

    params.push(limit, offset);
    const result = await this.db.query(
      `SELECT * FROM beats ${whereClause} ${orderBy} LIMIT $${paramIndex} OFFSET $${paramIndex + 1}`,
      params
    );

    return {
      beats: result.rows.map(r => this.mapRowToBeat(r)),
      total: parseInt(countResult.rows[0].count),
    };
  }

  async purchaseBeat(
    buyerId: string,
    beatId: string,
    licenseType: string
  ): Promise<{ downloadUrl: string; licenseId: string }> {
    const beat = await this.db.query('SELECT * FROM beats WHERE id = $1', [beatId]);
    if (beat.rows.length === 0) throw new Error('Beat not found');

    const licenses = JSON.parse(beat.rows[0].licenses) as BeatLicense[];
    const license = licenses.find(l => l.type === licenseType);
    if (!license) throw new Error('License type not available');

    // Check if exclusive and already sold
    if (license.type === 'exclusive') {
      const existingSale = await this.db.query(
        'SELECT 1 FROM beat_sales WHERE beat_id = $1 AND license_type = $2',
        [beatId, 'exclusive']
      );
      if (existingSale.rows.length > 0) {
        throw new Error('Beat already sold exclusively');
      }
    }

    // Create sale record
    const saleResult = await this.db.query(`
      INSERT INTO beat_sales (
        beat_id, buyer_id, license_type, price, created_at
      ) VALUES ($1, $2, $3, $4, NOW())
      RETURNING id
    `, [beatId, buyerId, licenseType, license.price]);

    // Update sales count
    await this.db.query(
      'UPDATE beats SET sales_count = sales_count + 1 WHERE id = $1',
      [beatId]
    );

    // Generate signed download URL
    const downloadUrl = await this.generateSignedDownloadUrl(
      beat.rows[0].audio_url,
      licenseType
    );

    // Create license NFT (optional)
    const licenseId = saleResult.rows[0].id;

    return { downloadUrl, licenseId };
  }

  // ============================================================================
  // Collaboration Matching
  // ============================================================================

  async findCollaborators(
    userId: string,
    preferences: {
      skills: string[];
      genres: string[];
      lookingFor: string[];
    }
  ): Promise<CollaborationMatch[]> {
    // Get user's profile for matching
    const userProfile = await this.db.query(`
      SELECT skills, genres, collaboration_score, verified
      FROM creator_profiles
      WHERE user_id = $1
    `, [userId]);

    // Find matching collaborators
    const result = await this.db.query(`
      WITH user_skills AS (
        SELECT unnest($1::text[]) as skill
      ),
      user_genres AS (
        SELECT unnest($2::text[]) as genre
      )
      SELECT
        cp.user_id,
        u.display_name,
        u.avatar_url,
        cp.skills,
        cp.genres,
        cp.verified,
        -- Calculate match score
        (
          SELECT COUNT(*) FROM unnest(cp.skills) s
          WHERE s = ANY($1)
        ) * 20 +
        (
          SELECT COUNT(*) FROM unnest(cp.genres) g
          WHERE g = ANY($2)
        ) * 15 +
        cp.collaboration_score as match_score,
        -- Recent work
        (
          SELECT ARRAY_AGG(title)
          FROM (
            SELECT title FROM beats WHERE producer_id = cp.user_id
            UNION
            SELECT title FROM sample_packs WHERE creator_id = cp.user_id
            ORDER BY title LIMIT 3
          ) recent
        ) as recent_work
      FROM creator_profiles cp
      JOIN users u ON cp.user_id = u.id
      WHERE cp.user_id != $3
        AND cp.looking_for && $4
        AND (cp.skills && $1 OR cp.genres && $2)
      ORDER BY match_score DESC
      LIMIT 20
    `, [preferences.skills, preferences.genres, userId, preferences.lookingFor]);

    return result.rows.map(row => ({
      userId: row.user_id,
      displayName: row.display_name,
      avatar: row.avatar_url,
      skills: row.skills,
      genres: row.genres,
      matchScore: row.match_score,
      matchReasons: this.generateMatchReasons(row, preferences),
      recentWork: row.recent_work || [],
      isVerified: row.verified,
    }));
  }

  async sendCollaborationRequest(
    fromUserId: string,
    toUserId: string,
    message: string,
    projectType: string
  ): Promise<void> {
    await this.db.query(`
      INSERT INTO collaboration_requests (
        from_user_id, to_user_id, message, project_type, status, created_at
      ) VALUES ($1, $2, $3, $4, 'pending', NOW())
    `, [fromUserId, toUserId, message, projectType]);

    // Send notification
    await this.redis.publish('notifications', JSON.stringify({
      type: 'collaboration_request',
      userId: toUserId,
      data: { fromUserId, message, projectType },
    }));
  }

  // ============================================================================
  // Crowdfunding
  // ============================================================================

  async createCrowdfundingCampaign(
    artistId: string,
    data: {
      title: string;
      description: string;
      goal: number;
      currency: 'USD' | 'ETH';
      endDate: Date;
      rewards: CrowdfundingReward[];
      coverImage: string;
      previewTracks?: string[];
    }
  ): Promise<string> {
    const result = await this.db.query(`
      INSERT INTO crowdfunding_campaigns (
        artist_id, title, description, goal, currency,
        end_date, rewards, cover_image, preview_tracks,
        raised, backer_count, status, created_at
      ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, 0, 0, 'active', NOW())
      RETURNING id
    `, [
      artistId,
      data.title,
      data.description,
      data.goal,
      data.currency,
      data.endDate,
      JSON.stringify(data.rewards),
      data.coverImage,
      data.previewTracks,
    ]);

    return result.rows[0].id;
  }

  async backCampaign(
    campaignId: string,
    backerId: string,
    amount: number,
    rewardId?: string
  ): Promise<void> {
    // Create pledge
    await this.db.query(`
      INSERT INTO campaign_pledges (
        campaign_id, backer_id, amount, reward_id, created_at
      ) VALUES ($1, $2, $3, $4, NOW())
    `, [campaignId, backerId, amount, rewardId]);

    // Update campaign totals
    await this.db.query(`
      UPDATE crowdfunding_campaigns
      SET raised = raised + $2, backer_count = backer_count + 1
      WHERE id = $1
    `, [campaignId, amount]);
  }

  // ============================================================================
  // Helpers
  // ============================================================================

  private async generateWaveform(audio: Buffer): Promise<number[]> {
    // Would use audio processing library to generate waveform
    return new Array(100).fill(0).map(() => Math.random());
  }

  private async analyzeAudio(audio: Buffer): Promise<{
    duration: number;
    format: string;
    sampleRate: number;
    bitDepth: number;
  }> {
    // Would use ffprobe or similar
    return {
      duration: 30,
      format: 'wav',
      sampleRate: 44100,
      bitDepth: 24,
    };
  }

  private async generatePreview(audio: Buffer): Promise<Buffer> {
    // Would use ffmpeg to create lower quality preview
    return audio;
  }

  private async generateTaggedPreview(audio: Buffer, title: string): Promise<Buffer> {
    // Would add audio watermark/tag to preview
    return audio;
  }

  private async generateSignedDownloadUrl(url: string, licenseType: string): Promise<string> {
    // Would generate time-limited signed URL
    return url;
  }

  private async updatePackBpmRange(packId: string): Promise<void> {
    await this.db.query(`
      UPDATE sample_packs
      SET bpm_range = (
        SELECT jsonb_build_object(
          'min', MIN(bpm),
          'max', MAX(bpm)
        )
        FROM samples WHERE pack_id = $1 AND bpm IS NOT NULL
      )
      WHERE id = $1
    `, [packId]);
  }

  private generateMatchReasons(row: any, preferences: any): string[] {
    const reasons: string[] = [];

    const sharedSkills = row.skills.filter((s: string) => preferences.skills.includes(s));
    if (sharedSkills.length > 0) {
      reasons.push(`Skilled in ${sharedSkills.slice(0, 2).join(', ')}`);
    }

    const sharedGenres = row.genres.filter((g: string) => preferences.genres.includes(g));
    if (sharedGenres.length > 0) {
      reasons.push(`Makes ${sharedGenres.slice(0, 2).join(', ')}`);
    }

    if (row.verified) {
      reasons.push('Verified creator');
    }

    return reasons;
  }

  private mapRowToSamplePack(row: any): SamplePack {
    return {
      id: row.id,
      creatorId: row.creator_id,
      title: row.title,
      description: row.description,
      coverArt: row.cover_art,
      samples: row.samples?.map((s: any) => this.mapRowToSample(s)) || [],
      genres: row.genres,
      tags: row.tags,
      bpmRange: row.bpm_range || { min: 0, max: 0 },
      keySignatures: row.key_signatures || [],
      price: row.price,
      currency: row.currency,
      licenseType: row.license_type,
      salesCount: row.sales_count || 0,
      rating: row.rating || 0,
      ratingCount: row.rating_count || 0,
      createdAt: new Date(row.created_at),
      updatedAt: new Date(row.updated_at),
      isPublished: row.is_published,
      isFeatured: row.is_featured || false,
    };
  }

  private mapRowToSample(row: any): Sample {
    return {
      id: row.id,
      packId: row.pack_id,
      name: row.name,
      category: row.category,
      audioUrl: row.audio_url,
      previewUrl: row.preview_url,
      duration: row.duration,
      bpm: row.bpm,
      key: row.key,
      waveformData: row.waveform_data ? JSON.parse(row.waveform_data) : [],
      format: row.format,
      sampleRate: row.sample_rate,
      bitDepth: row.bit_depth,
      isLoop: row.is_loop,
    };
  }

  private mapRowToBeat(row: any): Beat {
    return {
      id: row.id,
      producerId: row.producer_id,
      title: row.title,
      coverArt: row.cover_art,
      audioUrl: row.audio_url,
      previewUrl: row.preview_url,
      duration: row.duration,
      bpm: row.bpm,
      key: row.key,
      genres: row.genres,
      tags: row.tags,
      mood: row.mood,
      licenses: JSON.parse(row.licenses),
      salesCount: row.sales_count || 0,
      playCount: row.play_count || 0,
      rating: row.rating || 0,
      createdAt: new Date(row.created_at),
      isPublished: row.is_published,
    };
  }
}

interface CrowdfundingReward {
  id: string;
  title: string;
  description: string;
  minAmount: number;
  limit?: number;
  claimed: number;
  deliveryDate?: Date;
}

export default SamplePackService;
