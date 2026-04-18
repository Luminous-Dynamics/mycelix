// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Mood Tagger Service
 *
 * AI-powered audio analysis for automatic mood, genre, and characteristic tagging.
 * Uses deep learning models to analyze audio features and classify tracks.
 */

import * as tf from '@tensorflow/tfjs-node';
import { Pool } from 'pg';
import { Redis } from 'ioredis';

// ============================================================================
// Types
// ============================================================================

export interface AudioFeatures {
  mfcc: number[][];              // Mel-frequency cepstral coefficients
  spectralCentroid: number[];
  spectralRolloff: number[];
  zeroCrossingRate: number[];
  chroma: number[][];            // Chromagram
  tempo: number;
  tempogram: number[];
  rms: number[];                 // Root mean square energy
}

export interface MoodTag {
  mood: Mood;
  confidence: number;
}

export interface GenreTag {
  genre: string;
  confidence: number;
}

export interface CharacteristicTag {
  tag: string;
  confidence: number;
}

export interface TrackAnalysisResult {
  trackId: string;
  moods: MoodTag[];
  genres: GenreTag[];
  characteristics: CharacteristicTag[];
  attributes: {
    energy: number;
    valence: number;
    danceability: number;
    acousticness: number;
    instrumentalness: number;
    speechiness: number;
    liveness: number;
  };
  embeddings: number[];          // Audio embedding vector
}

export type Mood =
  | 'happy'
  | 'sad'
  | 'energetic'
  | 'calm'
  | 'aggressive'
  | 'romantic'
  | 'melancholic'
  | 'uplifting'
  | 'dark'
  | 'peaceful'
  | 'anxious'
  | 'nostalgic'
  | 'epic'
  | 'mysterious'
  | 'playful';

// ============================================================================
// Mood Tagger Service
// ============================================================================

export class MoodTaggerService {
  private moodModel: tf.LayersModel | null = null;
  private genreModel: tf.LayersModel | null = null;
  private attributeModel: tf.LayersModel | null = null;
  private embeddingModel: tf.LayersModel | null = null;

  private db: Pool;
  private redis: Redis;

  private readonly MOODS: Mood[] = [
    'happy', 'sad', 'energetic', 'calm', 'aggressive', 'romantic',
    'melancholic', 'uplifting', 'dark', 'peaceful', 'anxious',
    'nostalgic', 'epic', 'mysterious', 'playful',
  ];

  private readonly GENRES = [
    'electronic', 'hip-hop', 'rock', 'pop', 'jazz', 'classical',
    'r&b', 'country', 'folk', 'metal', 'ambient', 'indie',
    'latin', 'reggae', 'blues', 'soul', 'funk', 'punk',
    'house', 'techno', 'drum-and-bass', 'dubstep', 'trance',
  ];

  private readonly CHARACTERISTICS = [
    'acoustic', 'atmospheric', 'bass-heavy', 'beat-driven', 'cinematic',
    'complex', 'danceable', 'dreamy', 'dynamic', 'ethereal',
    'experimental', 'groovy', 'hypnotic', 'intense', 'layered',
    'lo-fi', 'melodic', 'minimalist', 'organic', 'percussive',
    'psychedelic', 'raw', 'rhythmic', 'smooth', 'spacey',
    'synth-heavy', 'textured', 'tribal', 'vocal-focused', 'warm',
  ];

  constructor(
    private readonly config: {
      modelPath: string;
      databaseUrl: string;
      redisUrl: string;
    }
  ) {
    this.db = new Pool({ connectionString: config.databaseUrl });
    this.redis = new Redis(config.redisUrl);
  }

  // ============================================================================
  // Model Loading
  // ============================================================================

  async loadModels(): Promise<void> {
    const modelPath = this.config.modelPath;

    [this.moodModel, this.genreModel, this.attributeModel, this.embeddingModel] =
      await Promise.all([
        tf.loadLayersModel(`file://${modelPath}/mood_model/model.json`),
        tf.loadLayersModel(`file://${modelPath}/genre_model/model.json`),
        tf.loadLayersModel(`file://${modelPath}/attribute_model/model.json`),
        tf.loadLayersModel(`file://${modelPath}/embedding_model/model.json`),
      ]);

    console.log('All models loaded successfully');
  }

  // ============================================================================
  // Analysis
  // ============================================================================

  async analyzeTrack(
    trackId: string,
    audioFeatures: AudioFeatures
  ): Promise<TrackAnalysisResult> {
    // Check cache first
    const cached = await this.redis.get(`analysis:${trackId}`);
    if (cached) {
      return JSON.parse(cached);
    }

    // Prepare input tensors
    const inputTensor = this.prepareInput(audioFeatures);

    // Run all models
    const [moods, genres, attributes, embeddings] = await Promise.all([
      this.predictMoods(inputTensor),
      this.predictGenres(inputTensor),
      this.predictAttributes(inputTensor),
      this.generateEmbedding(inputTensor),
    ]);

    // Generate characteristic tags from attributes
    const characteristics = this.deriveCharacteristics(attributes);

    const result: TrackAnalysisResult = {
      trackId,
      moods,
      genres,
      characteristics,
      attributes,
      embeddings,
    };

    // Cache result
    await this.redis.setex(`analysis:${trackId}`, 86400, JSON.stringify(result));

    // Store in database
    await this.storeAnalysis(result);

    // Cleanup tensor
    inputTensor.dispose();

    return result;
  }

  private prepareInput(features: AudioFeatures): tf.Tensor {
    // Normalize and combine features
    const mfccMean = this.meanPool(features.mfcc);
    const chromaMean = this.meanPool(features.chroma);

    // Combine all features into a single vector
    const combined = [
      ...mfccMean,
      ...chromaMean,
      ...this.normalize(features.spectralCentroid),
      ...this.normalize(features.spectralRolloff),
      ...this.normalize(features.zeroCrossingRate),
      features.tempo / 200, // Normalize tempo
      ...this.normalize(features.tempogram),
      ...this.normalize(features.rms),
    ];

    // Pad or truncate to fixed size
    const targetSize = 512;
    const padded = combined.length < targetSize
      ? [...combined, ...new Array(targetSize - combined.length).fill(0)]
      : combined.slice(0, targetSize);

    return tf.tensor2d([padded], [1, targetSize]);
  }

  private meanPool(matrix: number[][]): number[] {
    if (matrix.length === 0) return [];
    const numFeatures = matrix[0].length;
    const result = new Array(numFeatures).fill(0);

    for (const row of matrix) {
      for (let i = 0; i < numFeatures; i++) {
        result[i] += row[i];
      }
    }

    return result.map(v => v / matrix.length);
  }

  private normalize(arr: number[]): number[] {
    const max = Math.max(...arr);
    const min = Math.min(...arr);
    const range = max - min || 1;
    return arr.map(v => (v - min) / range);
  }

  // ============================================================================
  // Predictions
  // ============================================================================

  private async predictMoods(input: tf.Tensor): Promise<MoodTag[]> {
    if (!this.moodModel) throw new Error('Mood model not loaded');

    const predictions = this.moodModel.predict(input) as tf.Tensor;
    const values = await predictions.data();
    predictions.dispose();

    return this.MOODS
      .map((mood, i) => ({ mood, confidence: values[i] }))
      .filter(m => m.confidence > 0.3)
      .sort((a, b) => b.confidence - a.confidence)
      .slice(0, 5);
  }

  private async predictGenres(input: tf.Tensor): Promise<GenreTag[]> {
    if (!this.genreModel) throw new Error('Genre model not loaded');

    const predictions = this.genreModel.predict(input) as tf.Tensor;
    const values = await predictions.data();
    predictions.dispose();

    return this.GENRES
      .map((genre, i) => ({ genre, confidence: values[i] }))
      .filter(g => g.confidence > 0.2)
      .sort((a, b) => b.confidence - a.confidence)
      .slice(0, 5);
  }

  private async predictAttributes(input: tf.Tensor): Promise<TrackAnalysisResult['attributes']> {
    if (!this.attributeModel) throw new Error('Attribute model not loaded');

    const predictions = this.attributeModel.predict(input) as tf.Tensor;
    const values = await predictions.data();
    predictions.dispose();

    return {
      energy: values[0],
      valence: values[1],
      danceability: values[2],
      acousticness: values[3],
      instrumentalness: values[4],
      speechiness: values[5],
      liveness: values[6],
    };
  }

  private async generateEmbedding(input: tf.Tensor): Promise<number[]> {
    if (!this.embeddingModel) throw new Error('Embedding model not loaded');

    const embedding = this.embeddingModel.predict(input) as tf.Tensor;
    const values = await embedding.data();
    embedding.dispose();

    return Array.from(values);
  }

  // ============================================================================
  // Characteristic Derivation
  // ============================================================================

  private deriveCharacteristics(
    attributes: TrackAnalysisResult['attributes']
  ): CharacteristicTag[] {
    const tags: CharacteristicTag[] = [];

    // Energy-based characteristics
    if (attributes.energy > 0.7) {
      tags.push({ tag: 'intense', confidence: attributes.energy });
      tags.push({ tag: 'dynamic', confidence: attributes.energy * 0.9 });
    } else if (attributes.energy < 0.3) {
      tags.push({ tag: 'calm', confidence: 1 - attributes.energy });
      tags.push({ tag: 'minimalist', confidence: (1 - attributes.energy) * 0.8 });
    }

    // Danceability-based
    if (attributes.danceability > 0.7) {
      tags.push({ tag: 'danceable', confidence: attributes.danceability });
      tags.push({ tag: 'groovy', confidence: attributes.danceability * 0.85 });
      tags.push({ tag: 'beat-driven', confidence: attributes.danceability * 0.8 });
    }

    // Acousticness-based
    if (attributes.acousticness > 0.7) {
      tags.push({ tag: 'acoustic', confidence: attributes.acousticness });
      tags.push({ tag: 'organic', confidence: attributes.acousticness * 0.9 });
      tags.push({ tag: 'warm', confidence: attributes.acousticness * 0.8 });
    } else if (attributes.acousticness < 0.2) {
      tags.push({ tag: 'synth-heavy', confidence: 1 - attributes.acousticness });
    }

    // Instrumentalness-based
    if (attributes.instrumentalness > 0.8) {
      tags.push({ tag: 'instrumental', confidence: attributes.instrumentalness });
    } else if (attributes.instrumentalness < 0.3) {
      tags.push({ tag: 'vocal-focused', confidence: 1 - attributes.instrumentalness });
    }

    // Valence-based
    if (attributes.valence > 0.7) {
      tags.push({ tag: 'uplifting', confidence: attributes.valence });
    } else if (attributes.valence < 0.3) {
      tags.push({ tag: 'dark', confidence: 1 - attributes.valence });
      tags.push({ tag: 'melancholic', confidence: (1 - attributes.valence) * 0.9 });
    }

    // Liveness-based
    if (attributes.liveness > 0.7) {
      tags.push({ tag: 'live', confidence: attributes.liveness });
      tags.push({ tag: 'raw', confidence: attributes.liveness * 0.8 });
    }

    // Combination-based
    if (attributes.energy < 0.4 && attributes.acousticness > 0.5) {
      tags.push({ tag: 'atmospheric', confidence: 0.7 });
      tags.push({ tag: 'dreamy', confidence: 0.6 });
    }

    if (attributes.energy > 0.6 && attributes.danceability > 0.6) {
      tags.push({ tag: 'rhythmic', confidence: 0.8 });
    }

    if (attributes.energy > 0.8 && attributes.valence < 0.4) {
      tags.push({ tag: 'aggressive', confidence: 0.7 });
    }

    // Sort by confidence and deduplicate
    return tags
      .filter(t => t.confidence > 0.5)
      .sort((a, b) => b.confidence - a.confidence)
      .slice(0, 10);
  }

  // ============================================================================
  // Storage
  // ============================================================================

  private async storeAnalysis(result: TrackAnalysisResult): Promise<void> {
    await this.db.query(`
      INSERT INTO track_analysis (
        track_id,
        moods,
        genres,
        characteristics,
        energy,
        valence,
        danceability,
        acousticness,
        instrumentalness,
        speechiness,
        liveness,
        embeddings,
        analyzed_at
      ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, NOW())
      ON CONFLICT (track_id) DO UPDATE SET
        moods = $2,
        genres = $3,
        characteristics = $4,
        energy = $5,
        valence = $6,
        danceability = $7,
        acousticness = $8,
        instrumentalness = $9,
        speechiness = $10,
        liveness = $11,
        embeddings = $12,
        analyzed_at = NOW()
    `, [
      result.trackId,
      JSON.stringify(result.moods),
      JSON.stringify(result.genres),
      JSON.stringify(result.characteristics),
      result.attributes.energy,
      result.attributes.valence,
      result.attributes.danceability,
      result.attributes.acousticness,
      result.attributes.instrumentalness,
      result.attributes.speechiness,
      result.attributes.liveness,
      JSON.stringify(result.embeddings),
    ]);
  }

  // ============================================================================
  // Search by Mood/Characteristics
  // ============================================================================

  async findSimilarTracks(
    trackId: string,
    limit = 20
  ): Promise<{ trackId: string; similarity: number }[]> {
    // Get track embedding
    const result = await this.db.query(
      'SELECT embeddings FROM track_analysis WHERE track_id = $1',
      [trackId]
    );

    if (result.rows.length === 0) {
      throw new Error('Track not analyzed');
    }

    const embeddings = JSON.parse(result.rows[0].embeddings);

    // Find similar tracks using cosine similarity (using pgvector extension)
    const similar = await this.db.query(`
      SELECT
        track_id,
        1 - (embeddings <=> $1) as similarity
      FROM track_analysis
      WHERE track_id != $2
      ORDER BY embeddings <=> $1
      LIMIT $3
    `, [JSON.stringify(embeddings), trackId, limit]);

    return similar.rows.map(row => ({
      trackId: row.track_id,
      similarity: row.similarity,
    }));
  }

  async searchByMood(
    mood: Mood,
    options: { limit?: number; minConfidence?: number } = {}
  ): Promise<{ trackId: string; confidence: number }[]> {
    const { limit = 50, minConfidence = 0.5 } = options;

    const result = await this.db.query(`
      SELECT
        track_id,
        (moods::jsonb) as mood_data
      FROM track_analysis
      WHERE EXISTS (
        SELECT 1 FROM jsonb_array_elements(moods::jsonb) as m
        WHERE m->>'mood' = $1
          AND (m->>'confidence')::float >= $2
      )
      ORDER BY (
        SELECT (m->>'confidence')::float
        FROM jsonb_array_elements(moods::jsonb) as m
        WHERE m->>'mood' = $1
        LIMIT 1
      ) DESC
      LIMIT $3
    `, [mood, minConfidence, limit]);

    return result.rows.map(row => {
      const moodData = row.mood_data.find((m: any) => m.mood === mood);
      return {
        trackId: row.track_id,
        confidence: moodData?.confidence || 0,
      };
    });
  }

  async searchByGenre(
    genre: string,
    options: { limit?: number; minConfidence?: number } = {}
  ): Promise<{ trackId: string; confidence: number }[]> {
    const { limit = 50, minConfidence = 0.3 } = options;

    const result = await this.db.query(`
      SELECT
        track_id,
        (genres::jsonb) as genre_data
      FROM track_analysis
      WHERE EXISTS (
        SELECT 1 FROM jsonb_array_elements(genres::jsonb) as g
        WHERE g->>'genre' = $1
          AND (g->>'confidence')::float >= $2
      )
      ORDER BY (
        SELECT (g->>'confidence')::float
        FROM jsonb_array_elements(genres::jsonb) as g
        WHERE g->>'genre' = $1
        LIMIT 1
      ) DESC
      LIMIT $3
    `, [genre, minConfidence, limit]);

    return result.rows.map(row => {
      const genreData = row.genre_data.find((g: any) => g.genre === genre);
      return {
        trackId: row.track_id,
        confidence: genreData?.confidence || 0,
      };
    });
  }

  async searchByAttributes(
    attributes: Partial<TrackAnalysisResult['attributes']>,
    tolerance = 0.2,
    limit = 50
  ): Promise<string[]> {
    const conditions: string[] = [];
    const params: any[] = [];
    let paramIndex = 1;

    for (const [key, value] of Object.entries(attributes)) {
      if (value !== undefined) {
        conditions.push(`${key} BETWEEN $${paramIndex} AND $${paramIndex + 1}`);
        params.push(value - tolerance, value + tolerance);
        paramIndex += 2;
      }
    }

    if (conditions.length === 0) {
      throw new Error('At least one attribute required');
    }

    params.push(limit);

    const result = await this.db.query(`
      SELECT track_id
      FROM track_analysis
      WHERE ${conditions.join(' AND ')}
      LIMIT $${paramIndex}
    `, params);

    return result.rows.map(row => row.track_id);
  }
}

export default MoodTaggerService;
