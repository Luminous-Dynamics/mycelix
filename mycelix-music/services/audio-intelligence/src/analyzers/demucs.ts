// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Demucs Stem Separator
 *
 * Uses Demucs (from Facebook/Meta) for audio source separation.
 * Separates tracks into: drums, bass, vocals, other
 */

import { spawn } from 'child_process';
import { mkdir, readdir, stat, unlink, rmdir } from 'fs/promises';
import { join, basename } from 'path';
import { EventEmitter } from 'events';

export interface StemSeparationResult {
  drums: string;
  bass: string;
  vocals: string;
  other: string;
  original: string;
  duration: number;
  model: string;
}

export interface SeparationProgress {
  stage: 'loading' | 'separating' | 'saving' | 'complete';
  progress: number;
  message: string;
}

interface DemucsConfig {
  pythonPath?: string;
  modelName?: string;
  outputDir?: string;
  device?: 'cpu' | 'cuda';
  shifts?: number;
  overlap?: number;
}

/**
 * Demucs-based audio stem separator
 */
export class DemucsProcessor extends EventEmitter {
  private pythonPath: string;
  private modelName: string;
  private outputDir: string;
  private device: string;
  private shifts: number;
  private overlap: number;

  constructor(config: DemucsConfig = {}) {
    super();
    this.pythonPath = config.pythonPath || 'python3';
    this.modelName = config.modelName || 'htdemucs';
    this.outputDir = config.outputDir || '/tmp/mycelix-stems';
    this.device = config.device || 'cpu';
    this.shifts = config.shifts || 1;
    this.overlap = config.overlap || 0.25;
  }

  /**
   * Separate audio into stems
   */
  async separate(audioPath: string): Promise<StemSeparationResult> {
    await mkdir(this.outputDir, { recursive: true });

    const inputBasename = basename(audioPath, '.mp3').replace('.wav', '');
    const outputSubdir = join(this.outputDir, this.modelName, inputBasename);

    this.emitProgress('loading', 0, 'Loading audio file...');

    return new Promise((resolve, reject) => {
      // Run demucs command
      const args = [
        '-m', 'demucs',
        '--name', this.modelName,
        '--out', this.outputDir,
        '--device', this.device,
        '--shifts', String(this.shifts),
        '--overlap', String(this.overlap),
        audioPath,
      ];

      const demucs = spawn(this.pythonPath, args);

      let stderr = '';
      let lastProgress = 0;

      demucs.stderr.on('data', (data) => {
        const output = data.toString();
        stderr += output;

        // Parse progress from demucs output
        const progressMatch = output.match(/(\d+)%/);
        if (progressMatch) {
          const progress = parseInt(progressMatch[1], 10);
          if (progress > lastProgress) {
            lastProgress = progress;
            this.emitProgress('separating', progress, `Separating stems... ${progress}%`);
          }
        }
      });

      demucs.on('close', async (code) => {
        if (code !== 0) {
          reject(new Error(`Demucs failed: ${stderr}`));
          return;
        }

        this.emitProgress('saving', 90, 'Saving separated stems...');

        try {
          // Verify output files exist
          const stemPaths = {
            drums: join(outputSubdir, 'drums.wav'),
            bass: join(outputSubdir, 'bass.wav'),
            vocals: join(outputSubdir, 'vocals.wav'),
            other: join(outputSubdir, 'other.wav'),
          };

          // Check all stems exist
          for (const [stem, path] of Object.entries(stemPaths)) {
            try {
              await stat(path);
            } catch {
              reject(new Error(`Missing stem output: ${stem} at ${path}`));
              return;
            }
          }

          // Get duration from original file
          const duration = await this.getAudioDuration(audioPath);

          this.emitProgress('complete', 100, 'Separation complete!');

          resolve({
            ...stemPaths,
            original: audioPath,
            duration,
            model: this.modelName,
          });
        } catch (error) {
          reject(error);
        }
      });
    });
  }

  /**
   * Separate and return only specific stems
   */
  async separateStems(
    audioPath: string,
    stems: ('drums' | 'bass' | 'vocals' | 'other')[]
  ): Promise<Partial<StemSeparationResult>> {
    const result = await this.separate(audioPath);

    const filtered: Partial<StemSeparationResult> = {
      original: result.original,
      duration: result.duration,
      model: result.model,
    };

    for (const stem of stems) {
      filtered[stem] = result[stem];
    }

    return filtered;
  }

  /**
   * Extract vocals only (for lyric analysis, etc.)
   */
  async extractVocals(audioPath: string): Promise<string> {
    const result = await this.separate(audioPath);
    return result.vocals;
  }

  /**
   * Create instrumental version (all stems except vocals)
   */
  async createInstrumental(audioPath: string): Promise<string> {
    const result = await this.separate(audioPath);

    const outputPath = join(this.outputDir, 'instrumental', `${basename(audioPath)}_instrumental.wav`);
    await mkdir(join(this.outputDir, 'instrumental'), { recursive: true });

    // Mix drums, bass, and other together
    await this.mixStems([result.drums, result.bass, result.other], outputPath);

    return outputPath;
  }

  /**
   * Cleanup temporary files for a specific separation
   */
  async cleanup(result: StemSeparationResult): Promise<void> {
    const stems = [result.drums, result.bass, result.vocals, result.other];

    for (const stem of stems) {
      try {
        await unlink(stem);
      } catch {
        // Ignore errors
      }
    }

    // Try to remove parent directory
    try {
      const parentDir = join(result.drums, '..');
      await rmdir(parentDir);
    } catch {
      // Ignore errors
    }
  }

  /**
   * Mix multiple audio files together
   */
  private async mixStems(inputPaths: string[], outputPath: string): Promise<void> {
    return new Promise((resolve, reject) => {
      // Use ffmpeg for mixing
      const filterComplex = inputPaths
        .map((_, i) => `[${i}:a]`)
        .join('') + `amix=inputs=${inputPaths.length}:duration=longest`;

      const args = [
        ...inputPaths.flatMap((p) => ['-i', p]),
        '-filter_complex', filterComplex,
        '-ac', '2',
        '-y',
        outputPath,
      ];

      const ffmpeg = spawn('ffmpeg', args);

      let stderr = '';
      ffmpeg.stderr.on('data', (data) => {
        stderr += data.toString();
      });

      ffmpeg.on('close', (code) => {
        if (code !== 0) {
          reject(new Error(`FFmpeg mix failed: ${stderr}`));
          return;
        }
        resolve();
      });
    });
  }

  /**
   * Get audio duration using ffprobe
   */
  private async getAudioDuration(audioPath: string): Promise<number> {
    return new Promise((resolve, reject) => {
      const ffprobe = spawn('ffprobe', [
        '-v', 'error',
        '-show_entries', 'format=duration',
        '-of', 'default=noprint_wrappers=1:nokey=1',
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

        const duration = parseFloat(stdout.trim());
        resolve(isNaN(duration) ? 0 : duration);
      });
    });
  }

  private emitProgress(
    stage: SeparationProgress['stage'],
    progress: number,
    message: string
  ) {
    this.emit('progress', { stage, progress, message } as SeparationProgress);
  }
}

/**
 * Analyze isolated vocals for melody and rhythm
 */
export async function analyzeVocalMelody(vocalPath: string): Promise<{
  pitches: { time: number; pitch: number; confidence: number }[];
  vocalRange: { min: number; max: number };
  estimatedGender: 'male' | 'female' | 'unknown';
}> {
  return new Promise((resolve, reject) => {
    const python = spawn('python3', [
      '-c',
      `
import essentia.standard as es
import json
import numpy as np

audio = es.MonoLoader(filename='${vocalPath}')()

# Pitch detection
pitch_extractor = es.PredominantPitchMelodia()
pitch, pitch_confidence = pitch_extractor(audio)

# Filter out zeros (no pitch detected)
valid_pitches = pitch[pitch > 0]

if len(valid_pitches) == 0:
    result = {
        'pitches': [],
        'vocalRange': {'min': 0, 'max': 0},
        'estimatedGender': 'unknown'
    }
else:
    # Sample pitches for output
    hop_size = 128
    sample_rate = 44100
    times = np.arange(len(pitch)) * hop_size / sample_rate

    pitches = []
    for i in range(0, len(pitch), 10):  # Sample every 10 frames
        if pitch[i] > 0:
            pitches.append({
                'time': float(times[i]),
                'pitch': float(pitch[i]),
                'confidence': float(pitch_confidence[i])
            })

    min_pitch = float(np.min(valid_pitches))
    max_pitch = float(np.max(valid_pitches))
    avg_pitch = float(np.mean(valid_pitches))

    # Estimate gender based on average pitch
    gender = 'unknown'
    if avg_pitch < 165:
        gender = 'male'
    elif avg_pitch > 200:
        gender = 'female'

    result = {
        'pitches': pitches[:500],  # Limit output size
        'vocalRange': {'min': min_pitch, 'max': max_pitch},
        'estimatedGender': gender
    }

print(json.dumps(result))
      `,
    ]);

    let stdout = '';
    let stderr = '';

    python.stdout.on('data', (data) => {
      stdout += data.toString();
    });

    python.stderr.on('data', (data) => {
      stderr += data.toString();
    });

    python.on('close', (code) => {
      if (code !== 0) {
        reject(new Error(`Vocal analysis failed: ${stderr}`));
        return;
      }

      try {
        const result = JSON.parse(stdout.trim());
        resolve(result);
      } catch (error) {
        reject(new Error(`Failed to parse vocal analysis: ${stdout}`));
      }
    });
  });
}
