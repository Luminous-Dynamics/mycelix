// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Waveform Generator
 *
 * Generates waveform visualizations for audio files.
 * Outputs both raw data and optimized visual representations.
 */

import { spawn } from 'child_process';
import { mkdir, writeFile, readFile, unlink } from 'fs/promises';
import { join } from 'path';

export interface WaveformData {
  peaks: number[];
  duration: number;
  sampleRate: number;
  channels: number;
  resolution: number;
}

export interface WaveformOptions {
  resolution?: number; // Points per second
  normalize?: boolean;
  channels?: 'mono' | 'stereo' | 'mid-side';
}

interface WaveformConfig {
  tempDir?: string;
  ffmpegPath?: string;
}

/**
 * Waveform data generator
 */
export class WaveformGenerator {
  private tempDir: string;
  private ffmpegPath: string;

  constructor(config: WaveformConfig = {}) {
    this.tempDir = config.tempDir || '/tmp/mycelix-waveforms';
    this.ffmpegPath = config.ffmpegPath || 'ffmpeg';
  }

  /**
   * Generate waveform data from audio file
   */
  async generate(audioPath: string, options: WaveformOptions = {}): Promise<WaveformData> {
    const resolution = options.resolution || 100; // 100 points per second default
    const normalize = options.normalize ?? true;

    await mkdir(this.tempDir, { recursive: true });

    const rawPath = join(this.tempDir, `waveform_${Date.now()}.raw`);

    try {
      // Get audio info first
      const info = await this.getAudioInfo(audioPath);

      // Extract raw PCM data at lower sample rate for efficiency
      const targetSampleRate = resolution * 10; // 10x resolution for accuracy
      await this.extractPCM(audioPath, rawPath, targetSampleRate);

      // Read and process raw data
      const rawData = await readFile(rawPath);
      const samples = new Float32Array(rawData.buffer);

      // Calculate peaks
      const samplesPerPeak = Math.floor(targetSampleRate / resolution);
      const peaks: number[] = [];

      for (let i = 0; i < samples.length; i += samplesPerPeak) {
        let max = 0;
        const end = Math.min(i + samplesPerPeak, samples.length);

        for (let j = i; j < end; j++) {
          const abs = Math.abs(samples[j]);
          if (abs > max) max = abs;
        }

        peaks.push(max);
      }

      // Normalize if requested
      let normalizedPeaks = peaks;
      if (normalize) {
        const maxPeak = Math.max(...peaks);
        if (maxPeak > 0) {
          normalizedPeaks = peaks.map((p) => p / maxPeak);
        }
      }

      return {
        peaks: normalizedPeaks,
        duration: info.duration,
        sampleRate: info.sampleRate,
        channels: info.channels,
        resolution,
      };
    } finally {
      // Cleanup
      try {
        await unlink(rawPath);
      } catch {
        // Ignore
      }
    }
  }

  /**
   * Generate optimized waveform for display (fewer points, smoothed)
   */
  async generateForDisplay(
    audioPath: string,
    targetPoints: number = 800
  ): Promise<{ peaks: number[]; duration: number }> {
    const info = await this.getAudioInfo(audioPath);
    const resolution = Math.max(1, Math.ceil(targetPoints / info.duration));

    const waveform = await this.generate(audioPath, { resolution });

    // Downsample to target points if necessary
    if (waveform.peaks.length > targetPoints) {
      const ratio = waveform.peaks.length / targetPoints;
      const downsampled: number[] = [];

      for (let i = 0; i < targetPoints; i++) {
        const start = Math.floor(i * ratio);
        const end = Math.floor((i + 1) * ratio);
        let max = 0;

        for (let j = start; j < end && j < waveform.peaks.length; j++) {
          if (waveform.peaks[j] > max) max = waveform.peaks[j];
        }

        downsampled.push(max);
      }

      return { peaks: downsampled, duration: info.duration };
    }

    return { peaks: waveform.peaks, duration: info.duration };
  }

  /**
   * Generate stereo waveform (left and right channels)
   */
  async generateStereo(audioPath: string, options: WaveformOptions = {}): Promise<{
    left: number[];
    right: number[];
    duration: number;
  }> {
    const resolution = options.resolution || 100;
    await mkdir(this.tempDir, { recursive: true });

    const leftPath = join(this.tempDir, `waveform_left_${Date.now()}.raw`);
    const rightPath = join(this.tempDir, `waveform_right_${Date.now()}.raw`);

    try {
      const info = await this.getAudioInfo(audioPath);
      const targetSampleRate = resolution * 10;

      // Extract left channel
      await this.extractChannel(audioPath, leftPath, targetSampleRate, 'left');
      // Extract right channel
      await this.extractChannel(audioPath, rightPath, targetSampleRate, 'right');

      const leftData = await readFile(leftPath);
      const rightData = await readFile(rightPath);

      const leftSamples = new Float32Array(leftData.buffer);
      const rightSamples = new Float32Array(rightData.buffer);

      const samplesPerPeak = Math.floor(targetSampleRate / resolution);

      const left: number[] = [];
      const right: number[] = [];

      for (let i = 0; i < leftSamples.length; i += samplesPerPeak) {
        let maxLeft = 0;
        let maxRight = 0;
        const end = Math.min(i + samplesPerPeak, leftSamples.length);

        for (let j = i; j < end; j++) {
          const absLeft = Math.abs(leftSamples[j]);
          const absRight = Math.abs(rightSamples[j]);
          if (absLeft > maxLeft) maxLeft = absLeft;
          if (absRight > maxRight) maxRight = absRight;
        }

        left.push(maxLeft);
        right.push(maxRight);
      }

      // Normalize
      const maxPeak = Math.max(...left, ...right);
      if (maxPeak > 0) {
        return {
          left: left.map((p) => p / maxPeak),
          right: right.map((p) => p / maxPeak),
          duration: info.duration,
        };
      }

      return { left, right, duration: info.duration };
    } finally {
      try {
        await Promise.all([unlink(leftPath), unlink(rightPath)]);
      } catch {
        // Ignore
      }
    }
  }

  /**
   * Generate SVG waveform image
   */
  async generateSVG(
    audioPath: string,
    width: number = 800,
    height: number = 100,
    color: string = '#7DD3FC'
  ): Promise<string> {
    const { peaks } = await this.generateForDisplay(audioPath, width);

    const barWidth = width / peaks.length;
    const centerY = height / 2;

    let pathData = '';
    for (let i = 0; i < peaks.length; i++) {
      const x = i * barWidth;
      const barHeight = peaks[i] * height;
      const y = centerY - barHeight / 2;

      pathData += `M${x},${y} L${x},${y + barHeight} `;
    }

    return `
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${width} ${height}" preserveAspectRatio="none">
  <path d="${pathData}" stroke="${color}" stroke-width="${barWidth * 0.8}" fill="none" stroke-linecap="round"/>
</svg>`.trim();
  }

  private async getAudioInfo(
    audioPath: string
  ): Promise<{ duration: number; sampleRate: number; channels: number }> {
    return new Promise((resolve, reject) => {
      const ffprobe = spawn('ffprobe', [
        '-v', 'error',
        '-select_streams', 'a:0',
        '-show_entries', 'stream=duration,sample_rate,channels',
        '-of', 'json',
        audioPath,
      ]);

      let stdout = '';
      let stderr = '';

      ffprobe.stdout.on('data', (data) => {
        stdout += data.toString();
      });

      ffprobe.stderr.on('data', (data) => {
        stderr += data.toString();
      });

      ffprobe.on('close', (code) => {
        if (code !== 0) {
          reject(new Error(`FFprobe failed: ${stderr}`));
          return;
        }

        try {
          const data = JSON.parse(stdout);
          const stream = data.streams?.[0];

          resolve({
            duration: parseFloat(stream?.duration || '0'),
            sampleRate: parseInt(stream?.sample_rate || '44100', 10),
            channels: parseInt(stream?.channels || '2', 10),
          });
        } catch (error) {
          reject(new Error(`Failed to parse audio info: ${stdout}`));
        }
      });
    });
  }

  private extractPCM(
    inputPath: string,
    outputPath: string,
    sampleRate: number
  ): Promise<void> {
    return new Promise((resolve, reject) => {
      const ffmpeg = spawn(this.ffmpegPath, [
        '-i', inputPath,
        '-ac', '1', // Mono
        '-ar', String(sampleRate),
        '-f', 'f32le', // 32-bit float, little endian
        '-y',
        outputPath,
      ]);

      let stderr = '';
      ffmpeg.stderr.on('data', (data) => {
        stderr += data.toString();
      });

      ffmpeg.on('close', (code) => {
        if (code !== 0) {
          reject(new Error(`FFmpeg PCM extraction failed: ${stderr}`));
          return;
        }
        resolve();
      });
    });
  }

  private extractChannel(
    inputPath: string,
    outputPath: string,
    sampleRate: number,
    channel: 'left' | 'right'
  ): Promise<void> {
    const channelMap = channel === 'left' ? 'FL' : 'FR';

    return new Promise((resolve, reject) => {
      const ffmpeg = spawn(this.ffmpegPath, [
        '-i', inputPath,
        '-af', `pan=mono|c0=${channelMap}`,
        '-ar', String(sampleRate),
        '-f', 'f32le',
        '-y',
        outputPath,
      ]);

      let stderr = '';
      ffmpeg.stderr.on('data', (data) => {
        stderr += data.toString();
      });

      ffmpeg.on('close', (code) => {
        if (code !== 0) {
          reject(new Error(`FFmpeg channel extraction failed: ${stderr}`));
          return;
        }
        resolve();
      });
    });
  }
}
