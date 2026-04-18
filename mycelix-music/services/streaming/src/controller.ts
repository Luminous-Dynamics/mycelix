// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Streaming Controller
 *
 * HTTP API endpoints for managing transcode jobs.
 */

import { Request, Response } from 'express';
import { z } from 'zod';
import { TranscodeQueue, TranscodeJob } from './queue';
import { createLogger } from './logger';

const logger = createLogger('streaming-controller');

const transcodeRequestSchema = z.object({
  songId: z.string().min(1),
  sourceUrl: z.string().url(),
  outputPath: z.string().optional(),
  profiles: z.array(z.enum(['low', 'medium', 'high', 'lossless'])).optional().default([]),
  metadata: z.object({
    title: z.string().optional(),
    artist: z.string().optional(),
    album: z.string().optional(),
    genre: z.string().optional(),
  }).optional(),
  callbackUrl: z.string().url().optional(),
  priority: z.number().min(1).max(10).optional().default(5),
});

export class StreamingController {
  constructor(private queue: TranscodeQueue) {}

  async startTranscode(req: Request, res: Response) {
    try {
      const body = transcodeRequestSchema.parse(req.body);

      const jobData: TranscodeJob = {
        songId: body.songId,
        sourceUrl: body.sourceUrl,
        outputPath: body.outputPath || `songs/${body.songId}`,
        profiles: body.profiles,
        metadata: body.metadata,
        callbackUrl: body.callbackUrl,
      };

      const job = await this.queue.add(jobData, {
        priority: body.priority,
        jobId: `transcode-${body.songId}-${Date.now()}`,
      });

      logger.info({ songId: body.songId, jobId: job.id }, 'Transcode job queued');

      res.status(202).json({
        success: true,
        data: {
          jobId: job.id,
          songId: body.songId,
          status: 'queued',
        },
      });
    } catch (error) {
      if (error instanceof z.ZodError) {
        res.status(400).json({
          success: false,
          error: {
            code: 'VALIDATION_ERROR',
            message: 'Invalid request body',
            details: error.errors,
          },
        });
        return;
      }

      logger.error({ error }, 'Failed to queue transcode job');
      res.status(500).json({
        success: false,
        error: {
          code: 'INTERNAL_ERROR',
          message: 'Failed to queue transcode job',
        },
      });
    }
  }

  async getJobStatus(req: Request, res: Response) {
    try {
      const { jobId } = req.params;
      const job = await this.queue.getJob(jobId);

      if (!job) {
        res.status(404).json({
          success: false,
          error: {
            code: 'NOT_FOUND',
            message: 'Job not found',
          },
        });
        return;
      }

      const state = await job.getState();
      const progress = job.progress();

      const response: any = {
        jobId: job.id,
        songId: job.data.songId,
        status: state,
        progress: typeof progress === 'number' ? progress : 0,
        createdAt: new Date(job.timestamp).toISOString(),
      };

      if (state === 'completed') {
        response.result = job.returnvalue;
        response.completedAt = job.finishedOn
          ? new Date(job.finishedOn).toISOString()
          : undefined;
      }

      if (state === 'failed') {
        response.error = job.failedReason;
        response.attempts = job.attemptsMade;
      }

      res.json({
        success: true,
        data: response,
      });
    } catch (error) {
      logger.error({ error }, 'Failed to get job status');
      res.status(500).json({
        success: false,
        error: {
          code: 'INTERNAL_ERROR',
          message: 'Failed to get job status',
        },
      });
    }
  }

  async cancelJob(req: Request, res: Response) {
    try {
      const { jobId } = req.params;
      const job = await this.queue.getJob(jobId);

      if (!job) {
        res.status(404).json({
          success: false,
          error: {
            code: 'NOT_FOUND',
            message: 'Job not found',
          },
        });
        return;
      }

      const state = await job.getState();

      if (state === 'completed' || state === 'failed') {
        res.status(400).json({
          success: false,
          error: {
            code: 'INVALID_STATE',
            message: `Cannot cancel job in ${state} state`,
          },
        });
        return;
      }

      await job.remove();

      logger.info({ jobId }, 'Job cancelled');

      res.json({
        success: true,
        data: {
          jobId,
          status: 'cancelled',
        },
      });
    } catch (error) {
      logger.error({ error }, 'Failed to cancel job');
      res.status(500).json({
        success: false,
        error: {
          code: 'INTERNAL_ERROR',
          message: 'Failed to cancel job',
        },
      });
    }
  }
}
