// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Emotion & Context Intelligence Engine
 *
 * Emotion detection, biometric integration, music therapy, adaptive soundscapes
 */

import { EventEmitter } from 'events';

// ============================================================================
// Emotion Detection System
// ============================================================================

interface EmotionState {
  primary: Emotion;
  secondary?: Emotion;
  intensity: number; // 0-1
  confidence: number; // 0-1
  timestamp: Date;
}

type Emotion =
  | 'joy'
  | 'sadness'
  | 'anger'
  | 'fear'
  | 'surprise'
  | 'disgust'
  | 'trust'
  | 'anticipation'
  | 'calm'
  | 'energized'
  | 'focused'
  | 'nostalgic'
  | 'romantic'
  | 'melancholic';

interface EmotionSignal {
  source: 'listening_pattern' | 'biometric' | 'time_context' | 'user_input' | 'interaction';
  emotion: Emotion;
  confidence: number;
  data: any;
}

interface ListeningPattern {
  recentTracks: TrackEmotionData[];
  skipRate: number;
  repeatRate: number;
  sessionLength: number;
  genreDistribution: Map<string, number>;
  energyTrend: number[];
  valenceTrend: number[];
}

interface TrackEmotionData {
  trackId: string;
  valence: number;      // 0-1 (negative to positive)
  energy: number;       // 0-1 (calm to energetic)
  danceability: number; // 0-1
  acousticness: number; // 0-1
  listenDuration: number;
  skipped: boolean;
  repeated: boolean;
  likedDuring: boolean;
}

export class EmotionDetectionEngine extends EventEmitter {
  private currentState: EmotionState;
  private signalHistory: EmotionSignal[] = [];
  private emotionModel: EmotionClassificationModel;
  private patternAnalyzer: ListeningPatternAnalyzer;

  constructor() {
    super();
    this.emotionModel = new EmotionClassificationModel();
    this.patternAnalyzer = new ListeningPatternAnalyzer();
  }

  async processSignal(signal: EmotionSignal): Promise<void> {
    this.signalHistory.push(signal);

    // Keep only recent signals (last hour)
    const hourAgo = Date.now() - 60 * 60 * 1000;
    this.signalHistory = this.signalHistory.filter(
      s => new Date(s.data.timestamp || Date.now()).getTime() > hourAgo
    );

    // Recalculate emotion state
    await this.updateEmotionState();
  }

  private async updateEmotionState(): Promise<void> {
    // Aggregate signals by source weight
    const sourceWeights: Record<string, number> = {
      biometric: 0.35,
      listening_pattern: 0.30,
      user_input: 0.20,
      time_context: 0.10,
      interaction: 0.05,
    };

    const emotionScores = new Map<Emotion, number>();

    for (const signal of this.signalHistory) {
      const weight = sourceWeights[signal.source] * signal.confidence;
      const current = emotionScores.get(signal.emotion) || 0;
      emotionScores.set(signal.emotion, current + weight);
    }

    // Find primary and secondary emotions
    const sorted = Array.from(emotionScores.entries())
      .sort((a, b) => b[1] - a[1]);

    if (sorted.length > 0) {
      const totalScore = sorted.reduce((sum, [, score]) => sum + score, 0);

      this.currentState = {
        primary: sorted[0][0],
        secondary: sorted.length > 1 ? sorted[1][0] : undefined,
        intensity: Math.min(sorted[0][1] / (totalScore * 0.5), 1),
        confidence: this.calculateConfidence(),
        timestamp: new Date(),
      };

      this.emit('emotionUpdated', this.currentState);
    }
  }

  private calculateConfidence(): number {
    // Higher confidence with more diverse signals
    const sources = new Set(this.signalHistory.map(s => s.source));
    return Math.min(sources.size / 3, 1); // Max confidence at 3+ sources
  }

  async analyzeListeningPattern(pattern: ListeningPattern): Promise<EmotionSignal> {
    // Analyze listening patterns for emotional cues
    const avgValence = pattern.recentTracks.reduce((sum, t) => sum + t.valence, 0) / pattern.recentTracks.length;
    const avgEnergy = pattern.recentTracks.reduce((sum, t) => sum + t.energy, 0) / pattern.recentTracks.length;

    // Map valence/energy to emotions
    let emotion: Emotion;
    if (avgValence > 0.6 && avgEnergy > 0.6) {
      emotion = 'joy';
    } else if (avgValence > 0.6 && avgEnergy < 0.4) {
      emotion = 'calm';
    } else if (avgValence < 0.4 && avgEnergy < 0.4) {
      emotion = 'sadness';
    } else if (avgValence < 0.4 && avgEnergy > 0.6) {
      emotion = 'anger';
    } else if (pattern.skipRate > 0.5) {
      emotion = 'anticipation'; // User searching for specific mood
    } else if (pattern.repeatRate > 0.3) {
      emotion = 'nostalgic';
    } else {
      emotion = 'focused';
    }

    return {
      source: 'listening_pattern',
      emotion,
      confidence: pattern.recentTracks.length > 5 ? 0.8 : 0.5,
      data: { avgValence, avgEnergy, pattern },
    };
  }

  getCurrentState(): EmotionState | null {
    return this.currentState;
  }

  async getRecommendationContext(): Promise<EmotionRecommendationContext> {
    return {
      currentEmotion: this.currentState,
      targetEmotion: await this.predictDesiredEmotion(),
      transitionStrategy: this.getTransitionStrategy(),
      constraints: await this.getUserConstraints(),
    };
  }

  private async predictDesiredEmotion(): Promise<Emotion | null> {
    // Predict what emotion user might want based on context
    const hour = new Date().getHours();
    const dayOfWeek = new Date().getDay();

    // Evening winding down
    if (hour >= 21) return 'calm';

    // Morning energy
    if (hour >= 6 && hour <= 9) return 'energized';

    // Weekend relaxation
    if ((dayOfWeek === 0 || dayOfWeek === 6) && hour >= 10 && hour <= 14) {
      return 'joy';
    }

    return null;
  }

  private getTransitionStrategy(): 'gradual' | 'immediate' | 'maintain' {
    if (!this.currentState) return 'immediate';

    // If user seems stressed, use gradual transition
    if (this.currentState.primary === 'anger' || this.currentState.primary === 'fear') {
      return 'gradual';
    }

    return 'immediate';
  }

  private async getUserConstraints(): Promise<any> {
    return {};
  }
}

interface EmotionRecommendationContext {
  currentEmotion: EmotionState | null;
  targetEmotion: Emotion | null;
  transitionStrategy: 'gradual' | 'immediate' | 'maintain';
  constraints: any;
}

// ============================================================================
// Biometric Integration
// ============================================================================

interface BiometricData {
  heartRate?: number;
  heartRateVariability?: number;
  respirationRate?: number;
  skinConductance?: number;
  bodyTemperature?: number;
  bloodOxygen?: number;
  stepCount?: number;
  activity?: 'stationary' | 'walking' | 'running' | 'sleeping';
}

interface BiometricSource {
  type: 'watch' | 'fitness_band' | 'smart_ring' | 'app' | 'medical_device';
  deviceId: string;
  capabilities: (keyof BiometricData)[];
}

export class BiometricIntegration extends EventEmitter {
  private sources: Map<string, BiometricSource> = new Map();
  private currentData: BiometricData = {};
  private emotionEngine: EmotionDetectionEngine;
  private stressModel: StressDetectionModel;

  constructor(emotionEngine: EmotionDetectionEngine) {
    super();
    this.emotionEngine = emotionEngine;
    this.stressModel = new StressDetectionModel();
  }

  async connectSource(source: BiometricSource): Promise<void> {
    this.sources.set(source.deviceId, source);

    // Start listening for data
    this.startDataStream(source);

    this.emit('sourceConnected', source);
  }

  private startDataStream(source: BiometricSource): void {
    // In real implementation, connect to device APIs
    // Apple HealthKit, Google Fit, Fitbit API, etc.
  }

  async processData(deviceId: string, data: Partial<BiometricData>): Promise<void> {
    const source = this.sources.get(deviceId);
    if (!source) return;

    // Update current data
    Object.assign(this.currentData, data);

    // Analyze for emotional signals
    const emotionSignal = await this.analyzeForEmotion(data);
    if (emotionSignal) {
      await this.emotionEngine.processSignal(emotionSignal);
    }

    // Check stress levels
    const stressLevel = await this.stressModel.analyze(this.currentData);
    if (stressLevel > 0.7) {
      this.emit('highStress', { level: stressLevel, data: this.currentData });
    }

    this.emit('dataUpdated', this.currentData);
  }

  private async analyzeForEmotion(data: Partial<BiometricData>): Promise<EmotionSignal | null> {
    // Heart rate analysis
    if (data.heartRate) {
      const restingHR = await this.getUserRestingHeartRate();
      const deviation = (data.heartRate - restingHR) / restingHR;

      if (deviation > 0.3) {
        // Elevated HR - could be exercise, stress, or excitement
        if (data.activity === 'stationary') {
          return {
            source: 'biometric',
            emotion: 'anticipation', // Or could be stress/anxiety
            confidence: 0.6,
            data: { heartRate: data.heartRate, deviation },
          };
        }
      } else if (deviation < -0.1 && data.activity === 'stationary') {
        return {
          source: 'biometric',
          emotion: 'calm',
          confidence: 0.7,
          data: { heartRate: data.heartRate },
        };
      }
    }

    // HRV analysis (higher HRV = more relaxed)
    if (data.heartRateVariability) {
      if (data.heartRateVariability > 50) {
        return {
          source: 'biometric',
          emotion: 'calm',
          confidence: 0.65,
          data: { hrv: data.heartRateVariability },
        };
      } else if (data.heartRateVariability < 20) {
        return {
          source: 'biometric',
          emotion: 'fear', // Low HRV indicates stress
          confidence: 0.55,
          data: { hrv: data.heartRateVariability },
        };
      }
    }

    return null;
  }

  async getSleepQualityRecommendations(): Promise<SleepMusicRecommendation[]> {
    const sleepData = await this.getRecentSleepData();

    const recommendations: SleepMusicRecommendation[] = [];

    if (sleepData.averageQuality < 0.6) {
      recommendations.push({
        type: 'sleep_sounds',
        reason: 'Your sleep quality has been low. Try these calming sounds.',
        duration: 60, // minutes
        fadeOut: true,
      });
    }

    if (sleepData.averageLatency > 30) {
      recommendations.push({
        type: 'sleep_meditation',
        reason: 'Taking time to fall asleep? Try guided sleep meditation.',
        duration: 20,
        fadeOut: true,
      });
    }

    return recommendations;
  }

  private async getUserRestingHeartRate(): Promise<number> {
    return 70; // Would be personalized
  }

  private async getRecentSleepData(): Promise<{ averageQuality: number; averageLatency: number }> {
    return { averageQuality: 0.7, averageLatency: 15 };
  }

  getCurrentData(): BiometricData {
    return this.currentData;
  }
}

interface SleepMusicRecommendation {
  type: string;
  reason: string;
  duration: number;
  fadeOut: boolean;
}

// ============================================================================
// Music Therapy Engine
// ============================================================================

interface TherapySession {
  id: string;
  userId: string;
  type: TherapyType;
  goal: TherapyGoal;
  startTime: Date;
  duration: number;
  playlist: TherapyTrack[];
  progress: TherapyProgress;
  settings: TherapySettings;
}

type TherapyType =
  | 'stress_reduction'
  | 'focus_enhancement'
  | 'sleep_preparation'
  | 'anxiety_relief'
  | 'mood_elevation'
  | 'pain_management'
  | 'meditation'
  | 'energy_boost';

interface TherapyGoal {
  targetEmotion?: Emotion;
  targetHeartRate?: number;
  targetDuration: number;
  intensity: 'gentle' | 'moderate' | 'intensive';
}

interface TherapyTrack {
  trackId: string;
  purpose: string;
  targetBpm: number;
  binaural?: BinauralConfig;
  isochronic?: IsochronicConfig;
  expectedDuration: number;
}

interface BinauralConfig {
  baseFrequency: number;
  beatFrequency: number;
  targetBrainwave: 'delta' | 'theta' | 'alpha' | 'beta' | 'gamma';
}

interface IsochronicConfig {
  frequency: number;
  pattern: number[];
}

interface TherapyProgress {
  tracksCompleted: number;
  totalTracks: number;
  emotionJourney: EmotionState[];
  biometricJourney: BiometricData[];
  effectivenessScore?: number;
}

interface TherapySettings {
  binauralEnabled: boolean;
  guidedVoice: boolean;
  breathingCues: boolean;
  hapticFeedback: boolean;
  ambientSounds: string[];
}

export class MusicTherapyEngine extends EventEmitter {
  private activeSessions: Map<string, TherapySession> = new Map();
  private therapyLibrary: TherapyContentLibrary;
  private emotionEngine: EmotionDetectionEngine;
  private biometricIntegration: BiometricIntegration;

  constructor(
    emotionEngine: EmotionDetectionEngine,
    biometricIntegration: BiometricIntegration
  ) {
    super();
    this.emotionEngine = emotionEngine;
    this.biometricIntegration = biometricIntegration;
    this.therapyLibrary = new TherapyContentLibrary();
  }

  async startSession(params: {
    userId: string;
    type: TherapyType;
    goal: TherapyGoal;
    settings?: Partial<TherapySettings>;
  }): Promise<TherapySession> {
    // Get current emotional state
    const currentEmotion = this.emotionEngine.getCurrentState();
    const currentBiometrics = this.biometricIntegration.getCurrentData();

    // Build therapy playlist
    const playlist = await this.buildTherapyPlaylist({
      type: params.type,
      goal: params.goal,
      currentEmotion,
      currentBiometrics,
    });

    const session: TherapySession = {
      id: generateId(),
      userId: params.userId,
      type: params.type,
      goal: params.goal,
      startTime: new Date(),
      duration: params.goal.targetDuration,
      playlist,
      progress: {
        tracksCompleted: 0,
        totalTracks: playlist.length,
        emotionJourney: currentEmotion ? [currentEmotion] : [],
        biometricJourney: [currentBiometrics],
      },
      settings: {
        binauralEnabled: true,
        guidedVoice: params.type === 'meditation',
        breathingCues: ['stress_reduction', 'anxiety_relief'].includes(params.type),
        hapticFeedback: true,
        ambientSounds: [],
        ...params.settings,
      },
    };

    this.activeSessions.set(session.id, session);

    // Start monitoring
    this.startSessionMonitoring(session);

    this.emit('sessionStarted', session);
    return session;
  }

  private async buildTherapyPlaylist(params: {
    type: TherapyType;
    goal: TherapyGoal;
    currentEmotion: EmotionState | null;
    currentBiometrics: BiometricData;
  }): Promise<TherapyTrack[]> {
    const tracks: TherapyTrack[] = [];

    switch (params.type) {
      case 'stress_reduction':
        // Start with current stress level, gradually reduce
        const stressPhases = this.calculateStressReductionPhases(params.goal.targetDuration);
        for (const phase of stressPhases) {
          const track = await this.therapyLibrary.findTrack({
            targetBpm: phase.targetBpm,
            maxEnergy: phase.maxEnergy,
            binauralTarget: phase.brainwave,
          });
          tracks.push({
            ...track,
            purpose: phase.name,
            expectedDuration: phase.duration,
          });
        }
        break;

      case 'focus_enhancement':
        // Build to beta waves for concentration
        tracks.push(...await this.buildFocusPlaylist(params.goal.targetDuration));
        break;

      case 'sleep_preparation':
        // Delta wave progression
        tracks.push(...await this.buildSleepPlaylist(params.goal.targetDuration));
        break;

      case 'anxiety_relief':
        // Gentle, predictable patterns
        tracks.push(...await this.buildAnxietyReliefPlaylist(params.goal.targetDuration));
        break;

      case 'mood_elevation':
        // Gradual increase in valence and energy
        tracks.push(...await this.buildMoodElevationPlaylist(
          params.currentEmotion,
          params.goal.targetDuration
        ));
        break;
    }

    return tracks;
  }

  private calculateStressReductionPhases(duration: number): StressReductionPhase[] {
    return [
      { name: 'acknowledgment', targetBpm: 80, maxEnergy: 0.5, brainwave: 'beta', duration: duration * 0.15 },
      { name: 'transition', targetBpm: 70, maxEnergy: 0.4, brainwave: 'alpha', duration: duration * 0.25 },
      { name: 'deepening', targetBpm: 60, maxEnergy: 0.3, brainwave: 'alpha', duration: duration * 0.35 },
      { name: 'restoration', targetBpm: 55, maxEnergy: 0.2, brainwave: 'theta', duration: duration * 0.25 },
    ];
  }

  private async buildFocusPlaylist(duration: number): Promise<TherapyTrack[]> {
    return [];
  }

  private async buildSleepPlaylist(duration: number): Promise<TherapyTrack[]> {
    return [];
  }

  private async buildAnxietyReliefPlaylist(duration: number): Promise<TherapyTrack[]> {
    return [];
  }

  private async buildMoodElevationPlaylist(current: EmotionState | null, duration: number): Promise<TherapyTrack[]> {
    return [];
  }

  private startSessionMonitoring(session: TherapySession): void {
    // Monitor biometrics during session
    const checkInterval = setInterval(async () => {
      const biometrics = this.biometricIntegration.getCurrentData();
      const emotion = this.emotionEngine.getCurrentState();

      session.progress.biometricJourney.push(biometrics);
      if (emotion) {
        session.progress.emotionJourney.push(emotion);
      }

      // Adapt playlist if needed
      await this.adaptPlaylistIfNeeded(session);

      this.emit('sessionProgress', {
        sessionId: session.id,
        progress: session.progress,
      });
    }, 30000); // Check every 30 seconds

    // Store interval for cleanup
    (session as any)._checkInterval = checkInterval;
  }

  private async adaptPlaylistIfNeeded(session: TherapySession): Promise<void> {
    const recentBiometrics = session.progress.biometricJourney.slice(-5);

    // Check if user is responding to therapy
    if (session.type === 'stress_reduction') {
      const avgHR = recentBiometrics.reduce((sum, b) => sum + (b.heartRate || 0), 0) / recentBiometrics.length;

      // If HR not decreasing, intensify calming
      if (avgHR > 80) {
        this.emit('adaptationNeeded', {
          sessionId: session.id,
          reason: 'heart_rate_elevated',
          recommendation: 'increase_calming',
        });
      }
    }
  }

  async endSession(sessionId: string): Promise<TherapySessionSummary> {
    const session = this.activeSessions.get(sessionId);
    if (!session) throw new Error('Session not found');

    // Clear monitoring
    clearInterval((session as any)._checkInterval);

    // Calculate effectiveness
    const effectiveness = this.calculateEffectiveness(session);

    const summary: TherapySessionSummary = {
      sessionId,
      type: session.type,
      duration: Date.now() - session.startTime.getTime(),
      tracksCompleted: session.progress.tracksCompleted,
      emotionStart: session.progress.emotionJourney[0],
      emotionEnd: session.progress.emotionJourney[session.progress.emotionJourney.length - 1],
      effectiveness,
      recommendations: this.generateRecommendations(session, effectiveness),
    };

    this.activeSessions.delete(sessionId);
    this.emit('sessionEnded', summary);

    return summary;
  }

  private calculateEffectiveness(session: TherapySession): number {
    // Compare start and end states
    const startEmotion = session.progress.emotionJourney[0];
    const endEmotion = session.progress.emotionJourney[session.progress.emotionJourney.length - 1];

    if (!startEmotion || !endEmotion) return 0.5;

    // Check if we moved toward goal
    if (session.goal.targetEmotion === endEmotion.primary) {
      return 0.9;
    }

    // Check biometric improvements
    const startBio = session.progress.biometricJourney[0];
    const endBio = session.progress.biometricJourney[session.progress.biometricJourney.length - 1];

    if (session.type === 'stress_reduction' && endBio.heartRate && startBio.heartRate) {
      const hrReduction = (startBio.heartRate - endBio.heartRate) / startBio.heartRate;
      return Math.min(0.5 + hrReduction, 1);
    }

    return 0.6;
  }

  private generateRecommendations(session: TherapySession, effectiveness: number): string[] {
    const recommendations: string[] = [];

    if (effectiveness < 0.5) {
      recommendations.push('Try a longer session duration');
      recommendations.push('Consider trying a different therapy type');
    }

    if (session.type === 'sleep_preparation') {
      recommendations.push('For best results, use 30 minutes before bed');
    }

    return recommendations;
  }
}

interface StressReductionPhase {
  name: string;
  targetBpm: number;
  maxEnergy: number;
  brainwave: 'delta' | 'theta' | 'alpha' | 'beta';
  duration: number;
}

interface TherapySessionSummary {
  sessionId: string;
  type: TherapyType;
  duration: number;
  tracksCompleted: number;
  emotionStart: EmotionState;
  emotionEnd: EmotionState;
  effectiveness: number;
  recommendations: string[];
}

// ============================================================================
// Adaptive Soundscape Generator
// ============================================================================

interface SoundscapeConfig {
  type: 'sleep' | 'focus' | 'relaxation' | 'nature' | 'ambient' | 'custom';
  duration: number; // minutes, 0 for infinite
  adaptToBiometrics: boolean;
  layers: SoundLayer[];
  transitions: TransitionConfig;
}

interface SoundLayer {
  id: string;
  soundType: 'nature' | 'ambient' | 'binaural' | 'music' | 'noise';
  source: string;
  volume: number;
  pan: number; // -1 to 1
  lowpassFreq?: number;
  reverb?: number;
  enabled: boolean;
}

interface TransitionConfig {
  fadeInDuration: number;
  fadeOutDuration: number;
  crossfadeDuration: number;
  adaptationSpeed: 'slow' | 'medium' | 'fast';
}

export class AdaptiveSoundscapeGenerator extends EventEmitter {
  private currentConfig: SoundscapeConfig;
  private activeLayers: Map<string, AudioLayer> = new Map();
  private biometricIntegration: BiometricIntegration;
  private dreamAnalyzer: DreamStateAnalyzer;

  constructor(biometricIntegration: BiometricIntegration) {
    super();
    this.biometricIntegration = biometricIntegration;
    this.dreamAnalyzer = new DreamStateAnalyzer();
  }

  async createSoundscape(config: SoundscapeConfig): Promise<string> {
    const soundscapeId = generateId();
    this.currentConfig = config;

    // Initialize layers
    for (const layer of config.layers) {
      const audioLayer = await this.initializeLayer(layer);
      this.activeLayers.set(layer.id, audioLayer);
    }

    // Start adaptation loop if enabled
    if (config.adaptToBiometrics) {
      this.startBiometricAdaptation(soundscapeId);
    }

    // Start duration timer if not infinite
    if (config.duration > 0) {
      this.startDurationTimer(soundscapeId, config.duration);
    }

    this.emit('soundscapeStarted', { id: soundscapeId, config });
    return soundscapeId;
  }

  private async initializeLayer(layer: SoundLayer): Promise<AudioLayer> {
    // Load and configure audio layer
    return new AudioLayer(layer);
  }

  private startBiometricAdaptation(soundscapeId: string): void {
    setInterval(() => {
      const biometrics = this.biometricIntegration.getCurrentData();
      this.adaptToState(biometrics);
    }, 5000);
  }

  private async adaptToState(biometrics: BiometricData): Promise<void> {
    // For sleep soundscapes, detect sleep stage
    if (this.currentConfig.type === 'sleep' && biometrics.heartRate) {
      const sleepStage = await this.dreamAnalyzer.detectSleepStage(biometrics);

      switch (sleepStage) {
        case 'light':
          // Maintain gentle sounds
          this.adjustLayerVolume('ambient', 0.3);
          break;
        case 'deep':
          // Reduce all sounds
          this.adjustLayerVolume('ambient', 0.1);
          break;
        case 'rem':
          // Dream-enhancing sounds
          this.adjustLayerVolume('ambient', 0.2);
          this.enableLayer('dream_enhancement');
          break;
      }
    }

    // For focus, adapt to attention level
    if (this.currentConfig.type === 'focus' && biometrics.heartRateVariability) {
      if (biometrics.heartRateVariability < 30) {
        // User may be stressed, increase calming elements
        this.adjustLayerVolume('nature', 0.5);
      }
    }
  }

  private adjustLayerVolume(layerId: string, volume: number): void {
    const layer = this.activeLayers.get(layerId);
    if (layer) {
      layer.setVolume(volume, this.currentConfig.transitions.adaptationSpeed);
      this.emit('layerAdjusted', { layerId, volume });
    }
  }

  private enableLayer(layerId: string): void {
    const layer = this.activeLayers.get(layerId);
    if (layer) {
      layer.enable();
      this.emit('layerEnabled', { layerId });
    }
  }

  private startDurationTimer(soundscapeId: string, duration: number): void {
    setTimeout(() => {
      this.stopSoundscape(soundscapeId);
    }, duration * 60 * 1000);
  }

  async stopSoundscape(soundscapeId: string): Promise<void> {
    // Fade out all layers
    for (const [, layer] of this.activeLayers) {
      await layer.fadeOut(this.currentConfig.transitions.fadeOutDuration);
    }

    this.activeLayers.clear();
    this.emit('soundscapeStopped', { id: soundscapeId });
  }

  // Preset soundscapes
  static getPresets(): Record<string, SoundscapeConfig> {
    return {
      deep_sleep: {
        type: 'sleep',
        duration: 480, // 8 hours
        adaptToBiometrics: true,
        layers: [
          { id: 'brown_noise', soundType: 'noise', source: 'brown', volume: 0.3, pan: 0, enabled: true },
          { id: 'rain', soundType: 'nature', source: 'gentle_rain', volume: 0.2, pan: 0, enabled: true },
          { id: 'binaural', soundType: 'binaural', source: 'delta_2hz', volume: 0.15, pan: 0, enabled: true },
        ],
        transitions: { fadeInDuration: 300, fadeOutDuration: 600, crossfadeDuration: 30, adaptationSpeed: 'slow' },
      },
      deep_focus: {
        type: 'focus',
        duration: 0, // Infinite
        adaptToBiometrics: true,
        layers: [
          { id: 'pink_noise', soundType: 'noise', source: 'pink', volume: 0.2, pan: 0, enabled: true },
          { id: 'cafe', soundType: 'ambient', source: 'cafe_ambience', volume: 0.15, pan: 0, enabled: true },
          { id: 'binaural', soundType: 'binaural', source: 'beta_15hz', volume: 0.1, pan: 0, enabled: true },
        ],
        transitions: { fadeInDuration: 60, fadeOutDuration: 30, crossfadeDuration: 10, adaptationSpeed: 'medium' },
      },
      nature_escape: {
        type: 'nature',
        duration: 60,
        adaptToBiometrics: false,
        layers: [
          { id: 'forest', soundType: 'nature', source: 'forest_morning', volume: 0.4, pan: -0.3, enabled: true },
          { id: 'stream', soundType: 'nature', source: 'gentle_stream', volume: 0.3, pan: 0.3, enabled: true },
          { id: 'birds', soundType: 'nature', source: 'songbirds', volume: 0.2, pan: 0, enabled: true },
        ],
        transitions: { fadeInDuration: 30, fadeOutDuration: 60, crossfadeDuration: 15, adaptationSpeed: 'medium' },
      },
    };
  }
}

// ============================================================================
// Helper Classes
// ============================================================================

class EmotionClassificationModel {
  async classify(features: any): Promise<Emotion> {
    return 'calm';
  }
}

class ListeningPatternAnalyzer {
  async analyze(tracks: any[]): Promise<ListeningPattern> {
    return {
      recentTracks: [],
      skipRate: 0,
      repeatRate: 0,
      sessionLength: 0,
      genreDistribution: new Map(),
      energyTrend: [],
      valenceTrend: [],
    };
  }
}

class StressDetectionModel {
  async analyze(data: BiometricData): Promise<number> {
    if (!data.heartRate || !data.heartRateVariability) return 0.5;

    // Simple stress calculation
    const hrFactor = Math.max(0, (data.heartRate - 60) / 60);
    const hrvFactor = Math.max(0, (50 - data.heartRateVariability) / 50);

    return (hrFactor + hrvFactor) / 2;
  }
}

class TherapyContentLibrary {
  async findTrack(criteria: any): Promise<TherapyTrack> {
    return {
      trackId: 'therapy-track-1',
      purpose: 'calming',
      targetBpm: 60,
      expectedDuration: 300,
    };
  }
}

class DreamStateAnalyzer {
  async detectSleepStage(biometrics: BiometricData): Promise<'awake' | 'light' | 'deep' | 'rem'> {
    if (!biometrics.heartRate) return 'awake';

    if (biometrics.heartRate < 55) return 'deep';
    if (biometrics.heartRate < 65) return 'light';
    if (biometrics.heartRateVariability && biometrics.heartRateVariability > 60) return 'rem';

    return 'light';
  }
}

class AudioLayer {
  private config: SoundLayer;
  private volume: number;

  constructor(config: SoundLayer) {
    this.config = config;
    this.volume = config.volume;
  }

  setVolume(volume: number, speed: string): void {
    this.volume = volume;
  }

  enable(): void {
    this.config.enabled = true;
  }

  async fadeOut(duration: number): Promise<void> {
    // Fade out audio
  }
}

function generateId(): string {
  return Math.random().toString(36).substring(2, 15);
}
