// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Stem Separator Service
 *
 * AI-powered audio stem separation using deep learning models.
 * Separates tracks into vocals, drums, bass, and other instruments.
 */

import { spawn } from 'child_process';
import * as fs from 'fs/promises';
import * as path from 'path';
import { Redis } from 'ioredis';
import { S3Client, PutObjectCommand, GetObjectCommand } from '@aws-sdk/client-s3';

// ============================================================================
// Types
// ============================================================================

export interface StemSeparationRequest {
  trackId: string;
  audioUrl: string;
  outputStems: StemType[];
  quality: 'fast' | 'balanced' | 'high';
  userId: string;
}

export interface StemSeparationResult {
  trackId: string;
  stems: {
    type: StemType;
    url: string;
    duration: number;
    sampleRate: number;
  }[];
  metadata: {
    model: string;
    processingTime: number;
    quality: string;
  };
}

export type StemType =
  | 'vocals'
  | 'drums'
  | 'bass'
  | 'other'
  | 'piano'
  | 'guitar'
  | 'all';

export interface JobStatus {
  id: string;
  status: 'queued' | 'processing' | 'completed' | 'failed';
  progress: number;
  message?: string;
  result?: StemSeparationResult;
  error?: string;
  createdAt: Date;
  updatedAt: Date;
}

// ============================================================================
// Stem Separator Service
// ============================================================================

export class StemSeparatorService {
  private redis: Redis;
  private s3: S3Client;
  private readonly tempDir: string;
  private readonly modelPath: string;
  private isProcessing = false;

  constructor(
    private readonly config: {
      redisUrl: string;
      s3Bucket: string;
      s3Region: string;
      tempDir: string;
      modelPath: string;
    }
  ) {
    this.redis = new Redis(config.redisUrl);
    this.s3 = new S3Client({ region: config.s3Region });
    this.tempDir = config.tempDir;
    this.modelPath = config.modelPath;

    // Start processing loop
    this.processQueue();
  }

  // ============================================================================
  // Job Queue
  // ============================================================================

  async queueSeparation(request: StemSeparationRequest): Promise<string> {
    const jobId = `stem_${Date.now()}_${Math.random().toString(36).slice(2)}`;

    const job: JobStatus = {
      id: jobId,
      status: 'queued',
      progress: 0,
      createdAt: new Date(),
      updatedAt: new Date(),
    };

    // Store job status
    await this.redis.setex(`stem:job:${jobId}`, 86400, JSON.stringify(job));

    // Add to queue
    await this.redis.lpush('stem:queue', JSON.stringify({ ...request, jobId }));

    // Trigger processing if not already running
    if (!this.isProcessing) {
      this.processQueue();
    }

    return jobId;
  }

  async getJobStatus(jobId: string): Promise<JobStatus | null> {
    const data = await this.redis.get(`stem:job:${jobId}`);
    return data ? JSON.parse(data) : null;
  }

  private async updateJobStatus(jobId: string, updates: Partial<JobStatus>): Promise<void> {
    const current = await this.getJobStatus(jobId);
    if (!current) return;

    const updated = {
      ...current,
      ...updates,
      updatedAt: new Date(),
    };

    await this.redis.setex(`stem:job:${jobId}`, 86400, JSON.stringify(updated));
  }

  // ============================================================================
  // Processing
  // ============================================================================

  private async processQueue(): Promise<void> {
    if (this.isProcessing) return;

    this.isProcessing = true;

    try {
      while (true) {
        // Get next job from queue
        const jobData = await this.redis.rpop('stem:queue');
        if (!jobData) {
          break; // Queue empty
        }

        const request = JSON.parse(jobData) as StemSeparationRequest & { jobId: string };

        try {
          await this.updateJobStatus(request.jobId, {
            status: 'processing',
            progress: 0,
            message: 'Downloading audio...',
          });

          const result = await this.separateStems(request);

          await this.updateJobStatus(request.jobId, {
            status: 'completed',
            progress: 100,
            result,
          });
        } catch (error) {
          await this.updateJobStatus(request.jobId, {
            status: 'failed',
            error: (error as Error).message,
          });
        }
      }
    } finally {
      this.isProcessing = false;
    }
  }

  private async separateStems(
    request: StemSeparationRequest & { jobId: string }
  ): Promise<StemSeparationResult> {
    const startTime = Date.now();
    const workDir = path.join(this.tempDir, request.jobId);

    try {
      // Create work directory
      await fs.mkdir(workDir, { recursive: true });

      // Download audio file
      const inputPath = path.join(workDir, 'input.wav');
      await this.downloadAudio(request.audioUrl, inputPath);

      await this.updateJobStatus(request.jobId, {
        progress: 10,
        message: 'Running AI model...',
      });

      // Run separation model
      const outputDir = path.join(workDir, 'stems');
      await fs.mkdir(outputDir, { recursive: true });

      await this.runDemucs(inputPath, outputDir, request.quality, request.outputStems);

      await this.updateJobStatus(request.jobId, {
        progress: 70,
        message: 'Uploading stems...',
      });

      // Upload stems to S3
      const stems = await this.uploadStems(
        request.trackId,
        outputDir,
        request.outputStems
      );

      await this.updateJobStatus(request.jobId, {
        progress: 90,
        message: 'Finalizing...',
      });

      const processingTime = Date.now() - startTime;

      return {
        trackId: request.trackId,
        stems,
        metadata: {
          model: this.getModelName(request.quality),
          processingTime,
          quality: request.quality,
        },
      };
    } finally {
      // Cleanup
      await fs.rm(workDir, { recursive: true, force: true });
    }
  }

  // ============================================================================
  // Audio Processing
  // ============================================================================

  private async downloadAudio(url: string, outputPath: string): Promise<void> {
    // Handle different URL types
    if (url.startsWith('s3://')) {
      // Download from S3
      const match = url.match(/s3:\/\/([^\/]+)\/(.+)/);
      if (!match) throw new Error('Invalid S3 URL');

      const command = new GetObjectCommand({
        Bucket: match[1],
        Key: match[2],
      });

      const response = await this.s3.send(command);
      const body = await response.Body?.transformToByteArray();
      if (!body) throw new Error('Failed to download from S3');

      await fs.writeFile(outputPath, body);
    } else if (url.startsWith('http')) {
      // Download from HTTP
      const response = await fetch(url);
      if (!response.ok) throw new Error('Failed to download audio');

      const buffer = await response.arrayBuffer();
      await fs.writeFile(outputPath, Buffer.from(buffer));
    } else if (url.startsWith('ipfs://')) {
      // Download from IPFS
      const ipfsHash = url.replace('ipfs://', '');
      const gateway = `https://ipfs.io/ipfs/${ipfsHash}`;
      const response = await fetch(gateway);
      const buffer = await response.arrayBuffer();
      await fs.writeFile(outputPath, Buffer.from(buffer));
    } else {
      throw new Error('Unsupported URL scheme');
    }
  }

  private async runDemucs(
    inputPath: string,
    outputDir: string,
    quality: string,
    stems: StemType[]
  ): Promise<void> {
    return new Promise((resolve, reject) => {
      const model = this.getModelName(quality);

      // Build stem selection args
      const stemArgs: string[] = [];
      if (!stems.includes('all')) {
        if (stems.includes('vocals')) stemArgs.push('--two-stems', 'vocals');
        // Demucs 4-stem model always outputs all, we filter after
      }

      const args = [
        '-m', 'demucs',
        '--mp3', // Output as mp3 for smaller files
        '-n', model,
        '-o', outputDir,
        inputPath,
        ...stemArgs,
      ];

      const process = spawn('python3', args, {
        env: {
          ...process.env,
          CUDA_VISIBLE_DEVICES: '0', // Use first GPU
        },
      });

      let stderr = '';

      process.stderr.on('data', (data) => {
        stderr += data.toString();
        // Parse progress from Demucs output
        const match = data.toString().match(/(\d+)%/);
        if (match) {
          const progress = parseInt(match[1]);
          // Scale to 10-70 range
          const scaledProgress = 10 + (progress * 0.6);
          // Update job progress (fire and forget)
          // this.updateJobStatus would need jobId passed in
        }
      });

      process.on('close', (code) => {
        if (code === 0) {
          resolve();
        } else {
          reject(new Error(`Demucs failed: ${stderr}`));
        }
      });

      process.on('error', (err) => {
        reject(err);
      });
    });
  }

  private getModelName(quality: string): string {
    switch (quality) {
      case 'fast':
        return 'htdemucs'; // Faster but lower quality
      case 'balanced':
        return 'htdemucs_ft'; // Fine-tuned, good balance
      case 'high':
        return 'htdemucs_6s'; // 6-stem model, highest quality
      default:
        return 'htdemucs_ft';
    }
  }

  private async uploadStems(
    trackId: string,
    outputDir: string,
    requestedStems: StemType[]
  ): Promise<StemSeparationResult['stems']> {
    const stems: StemSeparationResult['stems'] = [];

    // Find the output subdirectory (Demucs creates one based on model name)
    const entries = await fs.readdir(outputDir);
    const modelDir = entries[0]; // Usually 'htdemucs' or similar
    const stemDir = path.join(outputDir, modelDir, 'input');

    const stemFiles = await fs.readdir(stemDir);

    for (const file of stemFiles) {
      const stemType = file.replace('.mp3', '').replace('.wav', '') as StemType;

      // Skip if not requested (unless 'all' was requested)
      if (!requestedStems.includes('all') && !requestedStems.includes(stemType)) {
        continue;
      }

      const filePath = path.join(stemDir, file);
      const fileContent = await fs.readFile(filePath);

      // Upload to S3
      const s3Key = `stems/${trackId}/${stemType}.mp3`;
      await this.s3.send(new PutObjectCommand({
        Bucket: this.config.s3Bucket,
        Key: s3Key,
        Body: fileContent,
        ContentType: 'audio/mpeg',
        Metadata: {
          trackId,
          stemType,
        },
      }));

      const url = `https://${this.config.s3Bucket}.s3.${this.config.s3Region}.amazonaws.com/${s3Key}`;

      // Get audio metadata (duration, sample rate)
      // Would use ffprobe or similar in production
      stems.push({
        type: stemType,
        url,
        duration: 0, // Would extract from file
        sampleRate: 44100, // Default assumption
      });
    }

    return stems;
  }

  // ============================================================================
  // Utility Methods
  // ============================================================================

  async getStemUrl(trackId: string, stemType: StemType): Promise<string | null> {
    const key = `stems/${trackId}/${stemType}.mp3`;

    try {
      await this.s3.send(new GetObjectCommand({
        Bucket: this.config.s3Bucket,
        Key: key,
      }));

      return `https://${this.config.s3Bucket}.s3.${this.config.s3Region}.amazonaws.com/${key}`;
    } catch {
      return null;
    }
  }

  async hasStems(trackId: string): Promise<boolean> {
    const vocalsUrl = await this.getStemUrl(trackId, 'vocals');
    return vocalsUrl !== null;
  }

  async getAvailableStems(trackId: string): Promise<StemType[]> {
    const allTypes: StemType[] = ['vocals', 'drums', 'bass', 'other', 'piano', 'guitar'];
    const available: StemType[] = [];

    for (const type of allTypes) {
      const url = await this.getStemUrl(trackId, type);
      if (url) {
        available.push(type);
      }
    }

    return available;
  }
}

export default StemSeparatorService;
