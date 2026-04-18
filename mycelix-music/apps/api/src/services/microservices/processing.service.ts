// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Audio Processing Microservice
 * Handles transcoding, waveform generation, stem separation, analysis
 */

import { EventEmitter } from 'events';
import { Redis } from 'ioredis';

export interface ProcessingJob {
  id: string;
  trackId: string;
  type: ProcessingJobType;
  status: JobStatus;
  progress: number;
  input: JobInput;
  output?: JobOutput;
  error?: string;
  priority: number;
  createdAt: Date;
  startedAt?: Date;
  completedAt?: Date;
}

export type ProcessingJobType =
  | 'transcode'
  | 'waveform'
  | 'stem_separation'
  | 'analysis'
  | 'normalize'
  | 'master';

export type JobStatus = 'queued' | 'processing' | 'completed' | 'failed' | 'cancelled';

interface JobInput {
  sourceUrl: string;
  sourceFormat?: string;
  options: Record<string, any>;
}

interface JobOutput {
  files: OutputFile[];
  metadata: Record<string, any>;
}

interface OutputFile {
  type: string;
  url: string;
  size: number;
  format?: string;
  quality?: string;
}

interface TranscodeOptions {
  qualities: AudioQualityConfig[];
  normalize: boolean;
  generateWaveform: boolean;
}

interface AudioQualityConfig {
  quality: string;
  codec: string;
  bitrate: number;
  sampleRate: number;
  channels: number;
}

interface WaveformOptions {
  width: number;
  height: number;
  color: string;
  backgroundColor: string;
  barWidth: number;
  barGap: number;
  peaks: number;
}

interface AnalysisResult {
  bpm: number;
  bpmConfidence: number;
  key: string;
  keyConfidence: number;
  loudness: {
    integrated: number;
    range: number;
    peak: number;
    truePeak: number;
  };
  duration: number;
  sampleRate: number;
  bitDepth: number;
  channels: number;
  codec: string;
  spectralCentroid: number;
  energy: number[];
  segments: AudioSegment[];
}

interface AudioSegment {
  start: number;
  end: number;
  type: 'intro' | 'verse' | 'chorus' | 'bridge' | 'outro' | 'drop' | 'breakdown';
  confidence: number;
}

interface StemSeparationResult {
  vocals: OutputFile;
  drums: OutputFile;
  bass: OutputFile;
  other: OutputFile;
  instrumental: OutputFile;
}

export class ProcessingService extends EventEmitter {
  private redis: Redis;
  private workerCount: number;
  private activeJobs: Map<string, ProcessingJob> = new Map();
  private jobQueue: ProcessingJob[] = [];

  constructor(redis: Redis, workerCount: number = 4) {
    super();
    this.redis = redis;
    this.workerCount = workerCount;
  }

  /**
   * Queue a new processing job
   */
  async queueJob(
    trackId: string,
    type: ProcessingJobType,
    input: JobInput,
    priority: number = 5
  ): Promise<ProcessingJob> {
    const job: ProcessingJob = {
      id: this.generateJobId(),
      trackId,
      type,
      status: 'queued',
      progress: 0,
      input,
      priority,
      createdAt: new Date(),
    };

    // Add to Redis queue
    await this.redis.zadd(
      'processing:queue',
      priority,
      JSON.stringify(job)
    );

    // Store job details
    await this.redis.hset(
      `processing:job:${job.id}`,
      this.serializeJob(job)
    );

    this.emit('job:queued', job);
    return job;
  }

  /**
   * Process a new track upload (full pipeline)
   */
  async processNewTrack(
    trackId: string,
    sourceUrl: string,
    options: {
      transcode?: TranscodeOptions;
      generateWaveform?: boolean;
      analyzeAudio?: boolean;
      separateStems?: boolean;
    } = {}
  ): Promise<{ jobs: ProcessingJob[]; pipelineId: string }> {
    const pipelineId = this.generateJobId();
    const jobs: ProcessingJob[] = [];

    // Default options
    const defaultTranscode: TranscodeOptions = {
      qualities: [
        { quality: 'LOW_128', codec: 'aac', bitrate: 128000, sampleRate: 44100, channels: 2 },
        { quality: 'STANDARD_256', codec: 'aac', bitrate: 256000, sampleRate: 44100, channels: 2 },
        { quality: 'HIGH_320', codec: 'mp3', bitrate: 320000, sampleRate: 44100, channels: 2 },
        { quality: 'LOSSLESS_FLAC', codec: 'flac', bitrate: 0, sampleRate: 44100, channels: 2 },
      ],
      normalize: true,
      generateWaveform: true,
    };

    // 1. Transcode job (highest priority)
    const transcodeJob = await this.queueJob(
      trackId,
      'transcode',
      {
        sourceUrl,
        options: options.transcode || defaultTranscode,
      },
      10
    );
    jobs.push(transcodeJob);

    // 2. Waveform generation
    if (options.generateWaveform !== false) {
      const waveformJob = await this.queueJob(
        trackId,
        'waveform',
        {
          sourceUrl,
          options: {
            width: 1800,
            height: 140,
            peaks: 900,
            color: '#6366f1',
            backgroundColor: 'transparent',
            barWidth: 2,
            barGap: 1,
          } as WaveformOptions,
        },
        8
      );
      jobs.push(waveformJob);
    }

    // 3. Audio analysis
    if (options.analyzeAudio !== false) {
      const analysisJob = await this.queueJob(
        trackId,
        'analysis',
        {
          sourceUrl,
          options: {
            detectBpm: true,
            detectKey: true,
            measureLoudness: true,
            segmentAudio: true,
          },
        },
        7
      );
      jobs.push(analysisJob);
    }

    // 4. Stem separation (optional, lower priority)
    if (options.separateStems) {
      const stemJob = await this.queueJob(
        trackId,
        'stem_separation',
        {
          sourceUrl,
          options: {
            model: 'demucs',
            stems: ['vocals', 'drums', 'bass', 'other'],
            outputFormat: 'wav',
          },
        },
        3
      );
      jobs.push(stemJob);
    }

    // Store pipeline info
    await this.redis.hset(`processing:pipeline:${pipelineId}`, {
      trackId,
      jobIds: JSON.stringify(jobs.map(j => j.id)),
      status: 'processing',
      createdAt: new Date().toISOString(),
    });

    return { jobs, pipelineId };
  }

  /**
   * Get job status
   */
  async getJobStatus(jobId: string): Promise<ProcessingJob | null> {
    const data = await this.redis.hgetall(`processing:job:${jobId}`);
    if (!data || Object.keys(data).length === 0) return null;
    return this.deserializeJob(data);
  }

  /**
   * Get pipeline status
   */
  async getPipelineStatus(pipelineId: string): Promise<{
    status: string;
    progress: number;
    jobs: ProcessingJob[];
  }> {
    const pipeline = await this.redis.hgetall(`processing:pipeline:${pipelineId}`);
    if (!pipeline) throw new Error('Pipeline not found');

    const jobIds: string[] = JSON.parse(pipeline.jobIds);
    const jobs = await Promise.all(jobIds.map(id => this.getJobStatus(id)));

    const completedJobs = jobs.filter(j => j?.status === 'completed').length;
    const failedJobs = jobs.filter(j => j?.status === 'failed').length;
    const progress = Math.round((completedJobs / jobs.length) * 100);

    let status = 'processing';
    if (completedJobs === jobs.length) status = 'completed';
    else if (failedJobs > 0) status = 'partially_failed';

    return {
      status,
      progress,
      jobs: jobs.filter(Boolean) as ProcessingJob[],
    };
  }

  /**
   * Update job progress
   */
  async updateJobProgress(jobId: string, progress: number, message?: string): Promise<void> {
    await this.redis.hset(`processing:job:${jobId}`, {
      progress: progress.toString(),
      ...(message && { message }),
    });

    this.emit('job:progress', { jobId, progress, message });
  }

  /**
   * Complete a job
   */
  async completeJob(jobId: string, output: JobOutput): Promise<void> {
    const job = await this.getJobStatus(jobId);
    if (!job) throw new Error('Job not found');

    job.status = 'completed';
    job.progress = 100;
    job.output = output;
    job.completedAt = new Date();

    await this.redis.hset(`processing:job:${jobId}`, this.serializeJob(job));
    this.activeJobs.delete(jobId);

    this.emit('job:completed', job);
  }

  /**
   * Fail a job
   */
  async failJob(jobId: string, error: string): Promise<void> {
    const job = await this.getJobStatus(jobId);
    if (!job) throw new Error('Job not found');

    job.status = 'failed';
    job.error = error;
    job.completedAt = new Date();

    await this.redis.hset(`processing:job:${jobId}`, this.serializeJob(job));
    this.activeJobs.delete(jobId);

    this.emit('job:failed', job);
  }

  /**
   * Cancel a job
   */
  async cancelJob(jobId: string): Promise<void> {
    const job = await this.getJobStatus(jobId);
    if (!job) throw new Error('Job not found');

    if (job.status === 'processing') {
      // Signal worker to cancel
      await this.redis.publish(`processing:cancel:${jobId}`, 'cancel');
    }

    job.status = 'cancelled';
    job.completedAt = new Date();

    await this.redis.hset(`processing:job:${jobId}`, this.serializeJob(job));
    await this.redis.zrem('processing:queue', JSON.stringify(job));
    this.activeJobs.delete(jobId);

    this.emit('job:cancelled', job);
  }

  /**
   * Retry a failed job
   */
  async retryJob(jobId: string): Promise<ProcessingJob> {
    const job = await this.getJobStatus(jobId);
    if (!job) throw new Error('Job not found');
    if (job.status !== 'failed') throw new Error('Can only retry failed jobs');

    // Create new job with same parameters
    return this.queueJob(job.trackId, job.type, job.input, job.priority);
  }

  // Simulated processing methods (would call actual workers)

  /**
   * Transcode audio to multiple formats
   */
  async transcode(job: ProcessingJob): Promise<OutputFile[]> {
    const options = job.input.options as TranscodeOptions;
    const outputFiles: OutputFile[] = [];

    for (let i = 0; i < options.qualities.length; i++) {
      const config = options.qualities[i];
      await this.updateJobProgress(job.id, ((i + 1) / options.qualities.length) * 80);

      // Simulated output
      outputFiles.push({
        type: 'audio',
        url: `https://storage.example.com/tracks/${job.trackId}/${config.quality}.${config.codec === 'aac' ? 'm4a' : config.codec}`,
        size: Math.round((config.bitrate / 8) * 180), // Estimated for 3-minute track
        format: config.codec,
        quality: config.quality,
      });
    }

    return outputFiles;
  }

  /**
   * Generate waveform data
   */
  async generateWaveform(job: ProcessingJob): Promise<{ peaks: number[]; image: string }> {
    const options = job.input.options as WaveformOptions;

    // Simulated waveform generation
    await this.updateJobProgress(job.id, 50);

    const peaks: number[] = [];
    for (let i = 0; i < options.peaks; i++) {
      peaks.push(Math.random() * 0.8 + 0.1);
    }

    await this.updateJobProgress(job.id, 100);

    return {
      peaks,
      image: `https://storage.example.com/waveforms/${job.trackId}.svg`,
    };
  }

  /**
   * Analyze audio properties
   */
  async analyzeAudio(job: ProcessingJob): Promise<AnalysisResult> {
    await this.updateJobProgress(job.id, 25);

    // Simulated analysis
    const result: AnalysisResult = {
      bpm: 120 + Math.round(Math.random() * 40),
      bpmConfidence: 0.85 + Math.random() * 0.15,
      key: ['C', 'D', 'E', 'F', 'G', 'A', 'B'][Math.floor(Math.random() * 7)] +
           (Math.random() > 0.5 ? 'm' : ''),
      keyConfidence: 0.75 + Math.random() * 0.25,
      loudness: {
        integrated: -14 + Math.random() * 4,
        range: 6 + Math.random() * 4,
        peak: -1 + Math.random() * 0.5,
        truePeak: -0.5 + Math.random() * 0.3,
      },
      duration: 180 + Math.round(Math.random() * 120),
      sampleRate: 44100,
      bitDepth: 16,
      channels: 2,
      codec: 'flac',
      spectralCentroid: 2000 + Math.random() * 3000,
      energy: Array(100).fill(0).map(() => Math.random()),
      segments: [
        { start: 0, end: 15, type: 'intro', confidence: 0.9 },
        { start: 15, end: 60, type: 'verse', confidence: 0.85 },
        { start: 60, end: 90, type: 'chorus', confidence: 0.92 },
        { start: 90, end: 135, type: 'verse', confidence: 0.83 },
        { start: 135, end: 165, type: 'chorus', confidence: 0.91 },
        { start: 165, end: 180, type: 'outro', confidence: 0.88 },
      ],
    };

    await this.updateJobProgress(job.id, 100);
    return result;
  }

  /**
   * Separate audio into stems
   */
  async separateStems(job: ProcessingJob): Promise<StemSeparationResult> {
    const stages = ['Loading model', 'Processing vocals', 'Processing drums', 'Processing bass', 'Processing other', 'Mixing instrumental'];

    for (let i = 0; i < stages.length; i++) {
      await this.updateJobProgress(job.id, ((i + 1) / stages.length) * 100, stages[i]);
    }

    const baseUrl = `https://storage.example.com/stems/${job.trackId}`;

    return {
      vocals: { type: 'stem', url: `${baseUrl}/vocals.wav`, size: 30000000 },
      drums: { type: 'stem', url: `${baseUrl}/drums.wav`, size: 28000000 },
      bass: { type: 'stem', url: `${baseUrl}/bass.wav`, size: 25000000 },
      other: { type: 'stem', url: `${baseUrl}/other.wav`, size: 32000000 },
      instrumental: { type: 'stem', url: `${baseUrl}/instrumental.wav`, size: 35000000 },
    };
  }

  // Helper methods

  private generateJobId(): string {
    return `job_${Date.now()}_${Math.random().toString(36).slice(2)}`;
  }

  private serializeJob(job: ProcessingJob): Record<string, string> {
    return {
      id: job.id,
      trackId: job.trackId,
      type: job.type,
      status: job.status,
      progress: job.progress.toString(),
      priority: job.priority.toString(),
      input: JSON.stringify(job.input),
      output: job.output ? JSON.stringify(job.output) : '',
      error: job.error || '',
      createdAt: job.createdAt.toISOString(),
      startedAt: job.startedAt?.toISOString() || '',
      completedAt: job.completedAt?.toISOString() || '',
    };
  }

  private deserializeJob(data: Record<string, string>): ProcessingJob {
    return {
      id: data.id,
      trackId: data.trackId,
      type: data.type as ProcessingJobType,
      status: data.status as JobStatus,
      progress: parseInt(data.progress, 10),
      priority: parseInt(data.priority, 10),
      input: JSON.parse(data.input),
      output: data.output ? JSON.parse(data.output) : undefined,
      error: data.error || undefined,
      createdAt: new Date(data.createdAt),
      startedAt: data.startedAt ? new Date(data.startedAt) : undefined,
      completedAt: data.completedAt ? new Date(data.completedAt) : undefined,
    };
  }
}

export default ProcessingService;
