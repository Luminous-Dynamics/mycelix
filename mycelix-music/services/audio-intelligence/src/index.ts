// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Audio Intelligence Service
 *
 * Unified service for audio analysis, stem separation, and processing.
 * Integrates Essentia, Demucs, and waveform generation.
 */

import { Queue, Worker, Job } from 'bullmq';
import Redis from 'ioredis';
import express, { Express, Request, Response } from 'express';
import multer from 'multer';
import { mkdir, unlink } from 'fs/promises';
import { join } from 'path';
import { v4 as uuidv4 } from 'uuid';

import { EssentiaAnalyzer, AudioFeatures, calculateFeatureSimilarity } from './analyzers/essentia';
import { DemucsProcessor, StemSeparationResult, SeparationProgress } from './analyzers/demucs';
import { WaveformGenerator, WaveformData } from './analyzers/waveform';

// ============================================================================
// Configuration
// ============================================================================

interface ServiceConfig {
  port: number;
  redisUrl: string;
  uploadDir: string;
  outputDir: string;
  pythonPath: string;
  maxConcurrent: number;
}

const config: ServiceConfig = {
  port: parseInt(process.env.PORT || '4002', 10),
  redisUrl: process.env.REDIS_URL || 'redis://localhost:6379',
  uploadDir: process.env.UPLOAD_DIR || '/tmp/mycelix-uploads',
  outputDir: process.env.OUTPUT_DIR || '/tmp/mycelix-output',
  pythonPath: process.env.PYTHON_PATH || 'python3',
  maxConcurrent: parseInt(process.env.MAX_CONCURRENT || '2', 10),
};

// ============================================================================
// Job Types
// ============================================================================

interface AnalyzeJob {
  trackId: string;
  audioPath: string;
  features?: ('all' | 'bpm' | 'key' | 'mood' | 'waveform')[];
}

interface SeparateJob {
  trackId: string;
  audioPath: string;
  stems?: ('drums' | 'bass' | 'vocals' | 'other')[];
}

type JobResult = {
  success: boolean;
  data?: unknown;
  error?: string;
};

// ============================================================================
// Service Class
// ============================================================================

class AudioIntelligenceService {
  private app: Express;
  private redis: Redis;
  private analysisQueue: Queue;
  private separationQueue: Queue;
  private analysisWorker?: Worker;
  private separationWorker?: Worker;

  private essentia: EssentiaAnalyzer;
  private demucs: DemucsProcessor;
  private waveform: WaveformGenerator;

  private featureCache: Map<string, AudioFeatures> = new Map();

  constructor() {
    this.app = express();
    this.redis = new Redis(config.redisUrl);

    this.analysisQueue = new Queue('audio-analysis', { connection: this.redis });
    this.separationQueue = new Queue('stem-separation', { connection: this.redis });

    this.essentia = new EssentiaAnalyzer({ pythonPath: config.pythonPath });
    this.demucs = new DemucsProcessor({
      pythonPath: config.pythonPath,
      outputDir: join(config.outputDir, 'stems'),
    });
    this.waveform = new WaveformGenerator({
      tempDir: join(config.outputDir, 'waveforms'),
    });

    this.setupRoutes();
  }

  private setupRoutes() {
    this.app.use(express.json());
    const upload = multer({ dest: config.uploadDir, limits: { fileSize: 500 * 1024 * 1024 } });

    this.app.get('/health', (req, res) => {
      res.json({ status: 'ok', service: 'audio-intelligence' });
    });

    this.app.post('/analyze', upload.single('audio'), async (req: Request, res: Response) => {
      try {
        const { trackId, features } = req.body;
        const audioPath = req.file?.path;
        if (!audioPath) { res.status(400).json({ error: 'No audio file' }); return; }
        const job = await this.analysisQueue.add('analyze', {
          trackId: trackId || uuidv4(), audioPath, features: features || ['all'],
        } as AnalyzeJob);
        res.json({ jobId: job.id, trackId, status: 'queued' });
      } catch (error) {
        res.status(500).json({ error: 'Failed to queue analysis' });
      }
    });

    this.app.post('/separate', upload.single('audio'), async (req: Request, res: Response) => {
      try {
        const { trackId, stems } = req.body;
        const audioPath = req.file?.path;
        if (!audioPath) { res.status(400).json({ error: 'No audio file' }); return; }
        const job = await this.separationQueue.add('separate', {
          trackId: trackId || uuidv4(), audioPath, stems,
        } as SeparateJob);
        res.json({ jobId: job.id, trackId, status: 'queued' });
      } catch (error) {
        res.status(500).json({ error: 'Failed to queue separation' });
      }
    });

    this.app.get('/job/:jobId', async (req: Request, res: Response) => {
      const { jobId } = req.params;
      let job = await this.analysisQueue.getJob(jobId) || await this.separationQueue.getJob(jobId);
      if (!job) { res.status(404).json({ error: 'Job not found' }); return; }
      const state = await job.getState();
      res.json({ jobId, state, progress: job.progress, data: state === 'completed' ? job.returnvalue : undefined });
    });

    this.app.post('/quick/bpm', upload.single('audio'), async (req: Request, res: Response) => {
      const audioPath = req.file?.path;
      if (!audioPath) { res.status(400).json({ error: 'No audio file' }); return; }
      const result = await this.essentia.detectBPM(audioPath);
      await unlink(audioPath);
      res.json(result);
    });

    this.app.post('/waveform', upload.single('audio'), async (req: Request, res: Response) => {
      const audioPath = req.file?.path;
      const { resolution, format } = req.body;
      if (!audioPath) { res.status(400).json({ error: 'No audio file' }); return; }
      if (format === 'svg') {
        const svg = await this.waveform.generateSVG(audioPath);
        res.type('image/svg+xml').send(svg);
      } else {
        const data = await this.waveform.generateForDisplay(audioPath, parseInt(resolution || '800', 10));
        res.json(data);
      }
      await unlink(audioPath);
    });

    this.app.post('/similarity', async (req: Request, res: Response) => {
      const { sourceTrackId, targetTrackIds } = req.body;
      const sourceFeatures = this.featureCache.get(sourceTrackId);
      if (!sourceFeatures) { res.status(404).json({ error: 'Source not found' }); return; }
      const similarities = targetTrackIds.map((id: string) => {
        const target = this.featureCache.get(id);
        return target ? { trackId: id, similarity: calculateFeatureSimilarity(sourceFeatures, target) } : null;
      }).filter(Boolean).sort((a: any, b: any) => b.similarity - a.similarity);
      res.json({ source: sourceTrackId, matches: similarities });
    });
  }

  private setupWorkers() {
    this.analysisWorker = new Worker('audio-analysis', async (job: Job<AnalyzeJob>) => {
      const { trackId, audioPath, features = ['all'] } = job.data;
      await job.updateProgress(10);
      const result: any = {};
      if (features.includes('all') || features.some(f => ['bpm', 'key', 'mood'].includes(f))) {
        result.features = await this.essentia.analyze(audioPath);
        this.featureCache.set(trackId, result.features);
        await this.redis.set(`features:${trackId}`, JSON.stringify(result.features), 'EX', 86400 * 30);
      }
      if (features.includes('all') || features.includes('waveform')) {
        result.waveform = await this.waveform.generateForDisplay(audioPath);
      }
      await unlink(audioPath).catch(() => {});
      return { success: true, data: result };
    }, { connection: this.redis, concurrency: config.maxConcurrent });

    this.separationWorker = new Worker('stem-separation', async (job: Job<SeparateJob>) => {
      const { trackId, audioPath, stems } = job.data;
      this.demucs.on('progress', (p: SeparationProgress) => job.updateProgress(p.progress));
      const result = stems?.length < 4 
        ? await this.demucs.separateStems(audioPath, stems)
        : await this.demucs.separate(audioPath);
      await unlink(audioPath).catch(() => {});
      return { success: true, data: result };
    }, { connection: this.redis, concurrency: 1 });
  }

  async start() {
    await mkdir(config.uploadDir, { recursive: true });
    await mkdir(config.outputDir, { recursive: true });
    this.setupWorkers();
    this.app.listen(config.port, () => console.log(`Audio Intelligence on port ${config.port}`));
  }

  async stop() {
    await this.analysisWorker?.close();
    await this.separationWorker?.close();
    await this.redis.quit();
  }
}

const service = new AudioIntelligenceService();
service.start().catch(e => { console.error(e); process.exit(1); });
process.on('SIGTERM', async () => { await service.stop(); process.exit(0); });

export { AudioIntelligenceService, EssentiaAnalyzer, DemucsProcessor, WaveformGenerator };
export type { AudioFeatures, StemSeparationResult, WaveformData };
