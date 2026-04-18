// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Essentia Audio Analyzer
 *
 * Uses Essentia.js for audio feature extraction including:
 * - Tempo (BPM) detection
 * - Key detection
 * - Energy/loudness analysis
 * - Spectral features
 * - Mood classification
 */

import { spawn, ChildProcess } from 'child_process';
import { createReadStream, createWriteStream } from 'fs';
import { mkdir, unlink, readFile, writeFile } from 'fs/promises';
import { join } from 'path';
import { pipeline } from 'stream/promises';
import { Readable } from 'stream';

export interface AudioFeatures {
  // Rhythm
  bpm: number;
  bpmConfidence: number;
  beats: number[];
  danceability: number;

  // Tonal
  key: string;
  scale: 'major' | 'minor';
  keyConfidence: number;
  chords: ChordEvent[];

  // Energy
  energy: number;
  loudness: number;
  dynamicRange: number;

  // Spectral
  spectralCentroid: number;
  spectralBandwidth: number;
  spectralRolloff: number;
  zeroCrossingRate: number;

  // Timbre
  mfcc: number[];
  brightness: number;
  warmth: number;

  // Mood (predicted)
  mood: MoodPrediction;

  // Segments
  segments: AudioSegment[];

  // Duration
  duration: number;
  sampleRate: number;
}

export interface ChordEvent {
  time: number;
  duration: number;
  chord: string;
  confidence: number;
}

export interface MoodPrediction {
  valence: number; // 0-1: sad to happy
  arousal: number; // 0-1: calm to energetic
  tags: string[];
  dominantMood: string;
}

export interface AudioSegment {
  start: number;
  duration: number;
  label: string;
  confidence: number;
}

interface EssentiaConfig {
  pythonPath?: string;
  modelsPath?: string;
  tempDir?: string;
}

/**
 * Essentia-based audio analyzer
 */
export class EssentiaAnalyzer {
  private pythonPath: string;
  private modelsPath: string;
  private tempDir: string;

  constructor(config: EssentiaConfig = {}) {
    this.pythonPath = config.pythonPath || 'python3';
    this.modelsPath = config.modelsPath || join(__dirname, '../../models/essentia');
    this.tempDir = config.tempDir || '/tmp/mycelix-audio';
  }

  /**
   * Analyze audio file and extract features
   */
  async analyze(audioPath: string): Promise<AudioFeatures> {
    await mkdir(this.tempDir, { recursive: true });

    const outputPath = join(this.tempDir, `features_${Date.now()}.json`);

    try {
      // Run Python analysis script
      await this.runPythonAnalysis(audioPath, outputPath);

      // Read results
      const resultsJson = await readFile(outputPath, 'utf-8');
      const results = JSON.parse(resultsJson);

      return this.normalizeResults(results);
    } finally {
      // Cleanup
      try {
        await unlink(outputPath);
      } catch {
        // Ignore cleanup errors
      }
    }
  }

  /**
   * Quick BPM detection only
   */
  async detectBPM(audioPath: string): Promise<{ bpm: number; confidence: number }> {
    return new Promise((resolve, reject) => {
      const python = spawn(this.pythonPath, [
        '-c',
        `
import essentia.standard as es
import json
import sys

audio = es.MonoLoader(filename='${audioPath}')()
rhythm_extractor = es.RhythmExtractor2013(method="multifeature")
bpm, beats, beats_confidence, _, _ = rhythm_extractor(audio)

print(json.dumps({"bpm": float(bpm), "confidence": float(beats_confidence)}))
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
          reject(new Error(`BPM detection failed: ${stderr}`));
          return;
        }

        try {
          const result = JSON.parse(stdout.trim());
          resolve(result);
        } catch (error) {
          reject(new Error(`Failed to parse BPM result: ${stdout}`));
        }
      });
    });
  }

  /**
   * Quick key detection only
   */
  async detectKey(audioPath: string): Promise<{ key: string; scale: string; confidence: number }> {
    return new Promise((resolve, reject) => {
      const python = spawn(this.pythonPath, [
        '-c',
        `
import essentia.standard as es
import json

audio = es.MonoLoader(filename='${audioPath}')()
key_extractor = es.KeyExtractor()
key, scale, strength = key_extractor(audio)

print(json.dumps({"key": key, "scale": scale, "confidence": float(strength)}))
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
          reject(new Error(`Key detection failed: ${stderr}`));
          return;
        }

        try {
          const result = JSON.parse(stdout.trim());
          resolve(result);
        } catch (error) {
          reject(new Error(`Failed to parse key result: ${stdout}`));
        }
      });
    });
  }

  private runPythonAnalysis(inputPath: string, outputPath: string): Promise<void> {
    return new Promise((resolve, reject) => {
      const pythonScript = `
import essentia.standard as es
import json
import numpy as np

def analyze_audio(input_path, output_path):
    # Load audio
    loader = es.MonoLoader(filename=input_path)
    audio = loader()

    # Get sample rate and duration
    sample_rate = 44100
    duration = len(audio) / sample_rate

    # Rhythm analysis
    rhythm_extractor = es.RhythmExtractor2013(method="multifeature")
    bpm, beats, beats_confidence, _, beats_intervals = rhythm_extractor(audio)

    # Danceability
    danceability_extractor = es.Danceability()
    danceability, _ = danceability_extractor(audio)

    # Key detection
    key_extractor = es.KeyExtractor()
    key, scale, key_strength = key_extractor(audio)

    # Energy and loudness
    energy_extractor = es.Energy()
    loudness_extractor = es.Loudness()
    energy = energy_extractor(audio)
    loudness = loudness_extractor(audio)

    # Dynamic range
    dr_extractor = es.DynamicComplexity()
    dynamic_complexity, _ = dr_extractor(audio)

    # Spectral features
    spectrum = es.Spectrum()(audio)
    spectral_centroid = es.Centroid()(spectrum)
    spectral_bandwidth = es.CentralMoments()(spectrum)[1]
    spectral_rolloff = es.RollOff()(spectrum)
    zcr = es.ZeroCrossingRate()(audio)

    # MFCC
    mfcc_extractor = es.MFCC()
    _, mfcc = mfcc_extractor(spectrum)

    # Brightness and warmth from spectral analysis
    brightness = spectral_centroid / (sample_rate / 2)
    warmth = 1 - brightness

    # Simple mood estimation based on features
    valence = (key_strength * 0.3 + danceability * 0.4 + brightness * 0.3)
    arousal = (bpm / 180 * 0.4 + energy * 0.3 + dynamic_complexity * 0.3)
    valence = min(1, max(0, valence))
    arousal = min(1, max(0, arousal))

    # Mood tags
    mood_tags = []
    if valence > 0.6 and arousal > 0.6:
        mood_tags.extend(['energetic', 'uplifting', 'happy'])
        dominant = 'energetic'
    elif valence > 0.6 and arousal <= 0.6:
        mood_tags.extend(['peaceful', 'calm', 'dreamy'])
        dominant = 'peaceful'
    elif valence <= 0.6 and arousal > 0.6:
        mood_tags.extend(['tense', 'aggressive', 'intense'])
        dominant = 'intense'
    else:
        mood_tags.extend(['melancholic', 'sad', 'dark'])
        dominant = 'melancholic'

    result = {
        'bpm': float(bpm),
        'bpmConfidence': float(beats_confidence),
        'beats': beats.tolist()[:100],  # Limit beats array
        'danceability': float(danceability),
        'key': key,
        'scale': scale,
        'keyConfidence': float(key_strength),
        'chords': [],  # Would need chord detection model
        'energy': float(energy),
        'loudness': float(loudness),
        'dynamicRange': float(dynamic_complexity),
        'spectralCentroid': float(spectral_centroid),
        'spectralBandwidth': float(spectral_bandwidth),
        'spectralRolloff': float(spectral_rolloff),
        'zeroCrossingRate': float(zcr),
        'mfcc': mfcc.tolist(),
        'brightness': float(brightness),
        'warmth': float(warmth),
        'mood': {
            'valence': float(valence),
            'arousal': float(arousal),
            'tags': mood_tags,
            'dominantMood': dominant
        },
        'segments': [],  # Would need segmentation model
        'duration': float(duration),
        'sampleRate': sample_rate
    }

    with open(output_path, 'w') as f:
        json.dump(result, f)

analyze_audio('${inputPath}', '${outputPath}')
      `;

      const python = spawn(this.pythonPath, ['-c', pythonScript]);

      let stderr = '';

      python.stderr.on('data', (data) => {
        stderr += data.toString();
      });

      python.on('close', (code) => {
        if (code !== 0) {
          reject(new Error(`Analysis failed: ${stderr}`));
          return;
        }
        resolve();
      });
    });
  }

  private normalizeResults(results: Record<string, unknown>): AudioFeatures {
    return {
      bpm: results.bpm as number,
      bpmConfidence: results.bpmConfidence as number,
      beats: results.beats as number[],
      danceability: results.danceability as number,
      key: results.key as string,
      scale: results.scale as 'major' | 'minor',
      keyConfidence: results.keyConfidence as number,
      chords: (results.chords as ChordEvent[]) || [],
      energy: results.energy as number,
      loudness: results.loudness as number,
      dynamicRange: results.dynamicRange as number,
      spectralCentroid: results.spectralCentroid as number,
      spectralBandwidth: results.spectralBandwidth as number,
      spectralRolloff: results.spectralRolloff as number,
      zeroCrossingRate: results.zeroCrossingRate as number,
      mfcc: results.mfcc as number[],
      brightness: results.brightness as number,
      warmth: results.warmth as number,
      mood: results.mood as MoodPrediction,
      segments: (results.segments as AudioSegment[]) || [],
      duration: results.duration as number,
      sampleRate: results.sampleRate as number,
    };
  }
}

/**
 * Calculate similarity between two sets of audio features
 */
export function calculateFeatureSimilarity(a: AudioFeatures, b: AudioFeatures): number {
  const weights = {
    tempo: 0.15,
    key: 0.15,
    energy: 0.15,
    mood: 0.2,
    timbre: 0.2,
    spectral: 0.15,
  };

  // Tempo similarity (within 10% is very similar)
  const tempoSim = 1 - Math.min(1, Math.abs(a.bpm - b.bpm) / Math.max(a.bpm, b.bpm));

  // Key similarity (same key = 1, relative keys = 0.7, others = 0.3)
  const keySim = a.key === b.key && a.scale === b.scale
    ? 1
    : a.key === b.key
    ? 0.7
    : 0.3;

  // Energy similarity
  const energySim = 1 - Math.abs(a.energy - b.energy);

  // Mood similarity (Euclidean distance in valence-arousal space)
  const moodDist = Math.sqrt(
    Math.pow(a.mood.valence - b.mood.valence, 2) +
    Math.pow(a.mood.arousal - b.mood.arousal, 2)
  );
  const moodSim = 1 - moodDist / Math.sqrt(2);

  // Timbre similarity (cosine similarity of MFCC)
  const timbreSim = cosineSimilarity(a.mfcc, b.mfcc);

  // Spectral similarity
  const spectralSim = 1 - Math.abs(a.brightness - b.brightness);

  return (
    weights.tempo * tempoSim +
    weights.key * keySim +
    weights.energy * energySim +
    weights.mood * moodSim +
    weights.timbre * timbreSim +
    weights.spectral * spectralSim
  );
}

function cosineSimilarity(a: number[], b: number[]): number {
  if (a.length !== b.length || a.length === 0) return 0;

  let dotProduct = 0;
  let normA = 0;
  let normB = 0;

  for (let i = 0; i < a.length; i++) {
    dotProduct += a[i] * b[i];
    normA += a[i] * a[i];
    normB += b[i] * b[i];
  }

  if (normA === 0 || normB === 0) return 0;

  return dotProduct / (Math.sqrt(normA) * Math.sqrt(normB));
}
