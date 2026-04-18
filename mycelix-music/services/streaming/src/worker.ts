// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Transcode Worker
 *
 * Processes audio files and generates HLS streams with multiple quality levels.
 */

import { Job } from 'bull';
import ffmpeg from 'fluent-ffmpeg';
import { promises as fs } from 'fs';
import path from 'path';
import { nanoid } from 'nanoid';
import { parseFile } from 'music-metadata';
import { S3Client, PutObjectCommand } from '@aws-sdk/client-s3';
import { Upload } from '@aws-sdk/lib-storage';
import { createReadStream } from 'fs';

import { TranscodeQueue, TranscodeJob, TranscodeResult } from './queue';
import { config } from './config';
import { createLogger } from './logger';

const logger = createLogger('transcode-worker');

export class TranscodeWorker {
  private queue: TranscodeQueue;
  private s3: S3Client;
  private isRunning = false;

  constructor(queue: TranscodeQueue) {
    this.queue = queue;
    this.s3 = new S3Client({
      region: config.s3.region,
      ...(config.s3.endpoint && { endpoint: config.s3.endpoint }),
      ...(config.s3.accessKeyId && {
        credentials: {
          accessKeyId: config.s3.accessKeyId,
          secretAccessKey: config.s3.secretAccessKey!,
        },
      }),
    });
  }

  async start() {
    if (this.isRunning) return;
    this.isRunning = true;

    // Ensure temp directory exists
    await fs.mkdir(config.transcode.tempDir, { recursive: true });

    // Set ffmpeg path if configured
    if (config.ffmpeg.path !== 'ffmpeg') {
      ffmpeg.setFfmpegPath(config.ffmpeg.path);
    }

    // Process jobs
    this.queue.process(config.transcode.concurrency, this.processJob.bind(this));

    logger.info('Transcode worker started');
  }

  async stop() {
    this.isRunning = false;
    await this.queue.pause(true);
    logger.info('Transcode worker stopped');
  }

  private async processJob(job: Job<TranscodeJob>): Promise<TranscodeResult> {
    const { songId, sourceUrl, outputPath } = job.data;
    const jobDir = path.join(config.transcode.tempDir, nanoid());

    logger.info({ songId, jobId: job.id }, 'Starting transcode job');

    try {
      // Create job directory
      await fs.mkdir(jobDir, { recursive: true });

      // Download source file
      await job.progress(5);
      const sourcePath = await this.downloadSource(sourceUrl, jobDir);

      // Extract metadata
      await job.progress(10);
      const metadata = await this.extractMetadata(sourcePath);

      // Generate waveform data
      await job.progress(15);
      const waveform = await this.generateWaveform(sourcePath, jobDir);

      // Transcode to each profile
      const profiles = job.data.profiles.length > 0
        ? config.transcode.profiles.filter(p => job.data.profiles.includes(p.name))
        : config.transcode.profiles;

      const transcodeResults: Record<string, string> = {};
      const allSegments: string[] = [];

      for (let i = 0; i < profiles.length; i++) {
        const profile = profiles[i];
        const progress = 20 + (i / profiles.length) * 60;
        await job.progress(progress);

        logger.info({ songId, profile: profile.name }, 'Transcoding profile');

        const { playlistPath, segments } = await this.transcodeProfile(
          sourcePath,
          jobDir,
          profile,
          metadata.duration
        );

        transcodeResults[profile.name] = playlistPath;
        allSegments.push(...segments);
      }

      // Generate master playlist
      await job.progress(85);
      const masterPlaylist = await this.generateMasterPlaylist(
        jobDir,
        profiles,
        transcodeResults
      );

      // Upload to S3
      await job.progress(90);
      const uploadedManifests = await this.uploadToS3(
        jobDir,
        outputPath,
        masterPlaylist,
        transcodeResults,
        allSegments
      );

      // Cleanup temp files
      await job.progress(98);
      await fs.rm(jobDir, { recursive: true, force: true });

      await job.progress(100);

      const result: TranscodeResult = {
        songId,
        manifests: {
          master: uploadedManifests.master,
          profiles: uploadedManifests.profiles,
        },
        segments: uploadedManifests.segments,
        duration: metadata.duration,
        waveform,
      };

      // Send callback if configured
      if (job.data.callbackUrl) {
        await this.sendCallback(job.data.callbackUrl, result);
      }

      logger.info({ songId, jobId: job.id }, 'Transcode job completed');
      return result;
    } catch (error) {
      logger.error({ songId, jobId: job.id, error }, 'Transcode job failed');
      await fs.rm(jobDir, { recursive: true, force: true }).catch(() => {});
      throw error;
    }
  }

  private async downloadSource(url: string, jobDir: string): Promise<string> {
    const sourcePath = path.join(jobDir, 'source');

    // If it's an S3 URL or HTTP URL, download it
    if (url.startsWith('http://') || url.startsWith('https://')) {
      const response = await fetch(url);
      if (!response.ok) throw new Error(`Failed to download source: ${response.status}`);

      const buffer = await response.arrayBuffer();
      await fs.writeFile(sourcePath, Buffer.from(buffer));
    } else if (url.startsWith('s3://')) {
      // Handle S3 URLs - parse and download
      const match = url.match(/^s3:\/\/([^/]+)\/(.+)$/);
      if (!match) throw new Error('Invalid S3 URL');

      const [, bucket, key] = match;
      // Download from S3 implementation would go here
      throw new Error('S3 URL download not implemented yet');
    } else {
      // Assume local file path
      await fs.copyFile(url, sourcePath);
    }

    return sourcePath;
  }

  private async extractMetadata(sourcePath: string): Promise<{ duration: number }> {
    const metadata = await parseFile(sourcePath);
    return {
      duration: metadata.format.duration || 0,
    };
  }

  private async generateWaveform(sourcePath: string, jobDir: string): Promise<number[]> {
    return new Promise((resolve, reject) => {
      const waveformPath = path.join(jobDir, 'waveform.json');
      const samples = 200; // Number of samples for waveform visualization

      ffmpeg(sourcePath)
        .audioFilters([
          `aresample=8000`,
          `asetnsamples=${samples}`,
          'astats=metadata=1:reset=1',
        ])
        .format('null')
        .on('end', async () => {
          // For simplicity, generate mock waveform data
          // In production, you'd parse the audio levels
          const waveform = Array.from({ length: samples }, () =>
            Math.random() * 0.8 + 0.1
          );
          resolve(waveform);
        })
        .on('error', (err) => {
          // Return empty waveform on error
          logger.warn({ err }, 'Failed to generate waveform');
          resolve([]);
        })
        .output('/dev/null')
        .run();
    });
  }

  private async transcodeProfile(
    sourcePath: string,
    jobDir: string,
    profile: typeof config.transcode.profiles[0],
    duration: number
  ): Promise<{ playlistPath: string; segments: string[] }> {
    const profileDir = path.join(jobDir, profile.name);
    await fs.mkdir(profileDir, { recursive: true });

    const playlistName = 'playlist.m3u8';
    const segmentPattern = 'segment%03d.ts';

    return new Promise((resolve, reject) => {
      ffmpeg(sourcePath)
        .audioCodec(profile.codec)
        .audioBitrate(profile.bitrate)
        .audioFrequency(profile.sampleRate)
        .audioChannels(2)
        .outputOptions([
          '-f hls',
          `-hls_time ${config.transcode.segmentDuration}`,
          '-hls_playlist_type vod',
          '-hls_segment_filename', path.join(profileDir, segmentPattern),
          `-threads ${config.ffmpeg.threads}`,
        ])
        .output(path.join(profileDir, playlistName))
        .on('end', async () => {
          const files = await fs.readdir(profileDir);
          const segments = files.filter(f => f.endsWith('.ts'));
          resolve({
            playlistPath: path.join(profileDir, playlistName),
            segments: segments.map(s => path.join(profileDir, s)),
          });
        })
        .on('error', reject)
        .run();
    });
  }

  private async generateMasterPlaylist(
    jobDir: string,
    profiles: typeof config.transcode.profiles,
    profilePlaylists: Record<string, string>
  ): Promise<string> {
    const masterPath = path.join(jobDir, 'master.m3u8');

    let content = '#EXTM3U\n#EXT-X-VERSION:3\n';

    for (const profile of profiles) {
      const bandwidth = parseInt(profile.bitrate) * 1000;
      content += `#EXT-X-STREAM-INF:BANDWIDTH=${bandwidth},CODECS="mp4a.40.2"\n`;
      content += `${profile.name}/playlist.m3u8\n`;
    }

    await fs.writeFile(masterPath, content);
    return masterPath;
  }

  private async uploadToS3(
    jobDir: string,
    outputPath: string,
    masterPlaylist: string,
    profilePlaylists: Record<string, string>,
    segments: string[]
  ): Promise<{
    master: string;
    profiles: Record<string, string>;
    segments: string[];
  }> {
    const baseUrl = config.cdn.baseUrl || `https://${config.s3.bucket}.s3.amazonaws.com`;
    const results = {
      master: '',
      profiles: {} as Record<string, string>,
      segments: [] as string[],
    };

    // Upload master playlist
    const masterKey = `${outputPath}/master.m3u8`;
    await this.uploadFile(masterPlaylist, masterKey, 'application/vnd.apple.mpegurl');
    results.master = `${baseUrl}/${masterKey}`;

    // Upload profile playlists and segments
    for (const [profileName, playlistPath] of Object.entries(profilePlaylists)) {
      const profileKey = `${outputPath}/${profileName}/playlist.m3u8`;
      await this.uploadFile(playlistPath, profileKey, 'application/vnd.apple.mpegurl');
      results.profiles[profileName] = `${baseUrl}/${profileKey}`;
    }

    // Upload segments
    for (const segmentPath of segments) {
      const relativePath = path.relative(jobDir, segmentPath);
      const segmentKey = `${outputPath}/${relativePath}`;
      await this.uploadFile(segmentPath, segmentKey, 'video/MP2T');
      results.segments.push(`${baseUrl}/${segmentKey}`);
    }

    return results;
  }

  private async uploadFile(localPath: string, key: string, contentType: string) {
    const upload = new Upload({
      client: this.s3,
      params: {
        Bucket: config.s3.bucket,
        Key: key,
        Body: createReadStream(localPath),
        ContentType: contentType,
        CacheControl: 'public, max-age=31536000', // 1 year cache for segments
      },
    });

    await upload.done();
  }

  private async sendCallback(url: string, result: TranscodeResult) {
    try {
      await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(result),
      });
    } catch (error) {
      logger.warn({ url, error }, 'Failed to send callback');
    }
  }
}
