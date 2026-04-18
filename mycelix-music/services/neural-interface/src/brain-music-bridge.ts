// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Mycelix Neural Music Interface
 * Brain-computer interface for direct musical thought translation
 * Neurofeedback composition and meditation enhancement
 * Dream soundtrack generation during sleep
 */

import { EventEmitter } from 'events';

// ============================================================
// INTERFACES & TYPES
// ============================================================

interface BrainwaveData {
  timestamp: number;
  channels: EEGChannel[];
  quality: SignalQuality;
  artifacts: ArtifactDetection;
  processed: ProcessedBrainwaves;
}

interface EEGChannel {
  name: string;
  position: ElectrodePosition;
  rawData: Float32Array;
  samplingRate: number;
  impedance: number;
}

interface ElectrodePosition {
  label: string; // 10-20 system (e.g., 'Fp1', 'Fz', 'Cz')
  x: number;
  y: number;
  z: number;
}

interface SignalQuality {
  overall: number; // 0-1
  channelQualities: Map<string, number>;
  noiseLevel: number;
  contactQuality: number;
}

interface ArtifactDetection {
  eyeBlinks: ArtifactEvent[];
  muscleArtifacts: ArtifactEvent[];
  electrodeArtifacts: ArtifactEvent[];
  environmentalNoise: ArtifactEvent[];
}

interface ArtifactEvent {
  startTime: number;
  endTime: number;
  type: string;
  severity: number;
  affectedChannels: string[];
}

interface ProcessedBrainwaves {
  delta: FrequencyBand; // 0.5-4 Hz - deep sleep, unconscious
  theta: FrequencyBand; // 4-8 Hz - creativity, meditation
  alpha: FrequencyBand; // 8-12 Hz - relaxation, calm focus
  beta: FrequencyBand; // 12-30 Hz - active thinking, alertness
  gamma: FrequencyBand; // 30-100 Hz - peak focus, perception
  raw: Float32Array;
}

interface FrequencyBand {
  power: number;
  dominance: number; // relative to other bands
  coherence: number; // cross-hemisphere synchronization
  asymmetry: number; // left-right asymmetry
  peaks: FrequencyPeak[];
}

interface FrequencyPeak {
  frequency: number;
  amplitude: number;
  phase: number;
}

interface NeuralMusicSession {
  id: string;
  userId: string;
  startTime: Date;
  mode: SessionMode;
  device: BCIDevice;
  calibration: CalibrationProfile;
  brainwaveHistory: BrainwaveData[];
  generatedMusic: GeneratedNeuralMusic[];
  mentalStates: MentalStateEvent[];
  feedback: NeurofeedbackLog[];
}

type SessionMode =
  | 'thought_to_music'
  | 'meditation_enhancement'
  | 'flow_state'
  | 'dream_recording'
  | 'emotional_expression'
  | 'creative_amplification';

interface BCIDevice {
  id: string;
  type: 'consumer' | 'research' | 'medical';
  brand: string;
  model: string;
  channelCount: number;
  samplingRate: number;
  connectivity: 'bluetooth' | 'wifi' | 'usb';
  batteryLevel: number;
}

interface CalibrationProfile {
  userId: string;
  deviceId: string;
  baselineRecording: BaselineData;
  mentalStateSignatures: Map<string, BrainSignature>;
  musicalIntentPatterns: Map<string, BrainPattern>;
  noiseFloor: number;
  personalThresholds: ThresholdConfig;
  lastCalibrated: Date;
}

interface BaselineData {
  eyesOpen: ProcessedBrainwaves;
  eyesClosed: ProcessedBrainwaves;
  restingState: ProcessedBrainwaves;
  duration: number;
}

interface BrainSignature {
  name: string;
  frequencyProfile: Map<string, number>;
  spatialPattern: number[];
  temporalPattern: number[];
  confidence: number;
}

interface BrainPattern {
  name: string;
  features: number[];
  classifier: string;
  accuracy: number;
}

interface ThresholdConfig {
  alpha: { low: number; high: number };
  theta: { low: number; high: number };
  beta: { low: number; high: number };
  gamma: { low: number; high: number };
  focusThreshold: number;
  relaxationThreshold: number;
  emotionalIntensity: number;
}

interface GeneratedNeuralMusic {
  id: string;
  timestamp: Date;
  brainwaveSource: string; // session ID
  audioData: Float32Array | null;
  midiData: NeuralMIDI;
  parameters: NeuralMusicParams;
  emotionalContent: EmotionalSignature;
  duration: number;
}

interface NeuralMIDI {
  tracks: NeuralMIDITrack[];
  tempo: number;
  brainwaveMapping: BrainwaveMapping;
}

interface NeuralMIDITrack {
  name: string;
  brainwaveSource: string;
  instrument: number;
  notes: NeuralNote[];
}

interface NeuralNote {
  time: number;
  pitch: number;
  velocity: number;
  duration: number;
  brainwaveCorrelation: number;
}

interface BrainwaveMapping {
  alphaToPitch: MappingFunction;
  betaToRhythm: MappingFunction;
  thetaToHarmony: MappingFunction;
  gammaToTimbre: MappingFunction;
  asymmetryToStereo: MappingFunction;
}

interface MappingFunction {
  inputRange: [number, number];
  outputRange: [number, number];
  curve: 'linear' | 'exponential' | 'logarithmic' | 'sigmoid';
  smoothing: number;
}

interface NeuralMusicParams {
  mappingMode: 'direct' | 'interpreted' | 'creative' | 'therapeutic';
  complexity: number;
  tonality: 'major' | 'minor' | 'modal' | 'atonal' | 'adaptive';
  tempo: 'brainwave_synced' | 'fixed' | 'heartbeat_synced';
  instrumentation: string[];
  spatialMode: 'mono' | 'stereo' | 'binaural' | 'spatial';
}

interface EmotionalSignature {
  valence: number;
  arousal: number;
  dominance: number;
  complexity: number;
  tension: number;
  transcendence: number;
}

interface MentalStateEvent {
  timestamp: Date;
  state: MentalState;
  confidence: number;
  duration: number;
  brainwaveProfile: ProcessedBrainwaves;
}

interface MentalState {
  primary: string;
  intensity: number;
  subStates: string[];
  musicAffinity: string[];
}

interface NeurofeedbackLog {
  timestamp: Date;
  targetState: string;
  currentState: string;
  feedback: FeedbackSignal;
  response: UserResponse;
}

interface FeedbackSignal {
  type: 'auditory' | 'visual' | 'haptic' | 'musical';
  intensity: number;
  content: any;
}

interface UserResponse {
  physiological: PhysiologicalResponse;
  behavioral: BehavioralResponse;
}

interface PhysiologicalResponse {
  brainwaveChange: Partial<ProcessedBrainwaves>;
  heartRateChange: number;
  skinConductanceChange: number;
}

interface BehavioralResponse {
  focusImproved: boolean;
  relaxationAchieved: boolean;
  targetStateDuration: number;
}

interface DreamSession {
  id: string;
  userId: string;
  nightOf: Date;
  sleepStages: SleepStage[];
  dreams: DreamCapture[];
  generatedSoundtrack: DreamSoundtrack | null;
  analysis: DreamAnalysis;
}

interface SleepStage {
  startTime: Date;
  endTime: Date;
  stage: 'awake' | 'light' | 'deep' | 'rem';
  brainwaveProfile: ProcessedBrainwaves;
  events: SleepEvent[];
}

interface SleepEvent {
  timestamp: Date;
  type: 'movement' | 'arousal' | 'rem_burst' | 'k_complex' | 'spindle';
  intensity: number;
}

interface DreamCapture {
  timestamp: Date;
  remPeriod: number;
  brainwaveIntensity: ProcessedBrainwaves;
  estimatedContent: DreamContent;
  emotionalProfile: EmotionalSignature;
}

interface DreamContent {
  estimatedThemes: string[];
  emotionalTone: string;
  visualActivity: number;
  narrativeComplexity: number;
  lucidityIndicators: number;
}

interface DreamSoundtrack {
  id: string;
  duration: number;
  sections: DreamMusicSection[];
  audioData: Float32Array | null;
  isRealtime: boolean;
}

interface DreamMusicSection {
  startTime: number;
  duration: number;
  sleepStage: string;
  mood: EmotionalSignature;
  musicalElements: MusicalElement[];
}

interface MusicalElement {
  type: 'melody' | 'harmony' | 'rhythm' | 'texture' | 'effect';
  brainwaveSource: string;
  parameters: Record<string, number>;
}

interface DreamAnalysis {
  totalDuration: number;
  remPercentage: number;
  deepSleepPercentage: number;
  dreamCount: number;
  emotionalJourney: EmotionalSignature[];
  musicalCorrelations: MusicalCorrelation[];
}

interface MusicalCorrelation {
  sleepStage: string;
  musicalCharacteristics: string[];
  brainwavePatterns: string[];
}

// ============================================================
// BRAIN-MUSIC INTERFACE
// ============================================================

export class BrainMusicInterface extends EventEmitter {
  private signalProcessor: NeuralSignalProcessor;
  private musicSynthesizer: NeuralMusicSynthesizer;
  private stateClassifier: MentalStateClassifier;
  private feedbackEngine: NeurofeedbackEngine;
  private activeSessions: Map<string, NeuralMusicSession> = new Map();

  constructor() {
    super();
    this.signalProcessor = new NeuralSignalProcessor();
    this.musicSynthesizer = new NeuralMusicSynthesizer();
    this.stateClassifier = new MentalStateClassifier();
    this.feedbackEngine = new NeurofeedbackEngine();
  }

  async startSession(
    userId: string,
    device: BCIDevice,
    mode: SessionMode
  ): Promise<NeuralMusicSession> {
    // Load or create calibration profile
    const calibration = await this.loadOrCreateCalibration(userId, device);

    const session: NeuralMusicSession = {
      id: this.generateSessionId(),
      userId,
      startTime: new Date(),
      mode,
      device,
      calibration,
      brainwaveHistory: [],
      generatedMusic: [],
      mentalStates: [],
      feedback: []
    };

    this.activeSessions.set(session.id, session);

    // Start brainwave streaming
    this.startBrainwaveStream(session, device);

    this.emit('sessionStarted', session);
    return session;
  }

  async processBrainwaves(sessionId: string, rawData: Float32Array[]): Promise<void> {
    const session = this.activeSessions.get(sessionId);
    if (!session) throw new Error('Session not found');

    // Process raw EEG data
    const processed = await this.signalProcessor.process(rawData, session.calibration);

    // Classify mental state
    const mentalState = await this.stateClassifier.classify(processed);

    // Record state event
    session.mentalStates.push({
      timestamp: new Date(),
      state: mentalState,
      confidence: mentalState.intensity,
      duration: 0,
      brainwaveProfile: processed.processed
    });

    // Generate music based on mode
    await this.generateMusicForMode(session, processed, mentalState);

    session.brainwaveHistory.push(processed);
    this.emit('brainwavesProcessed', { sessionId, processed, mentalState });
  }

  private async generateMusicForMode(
    session: NeuralMusicSession,
    brainwaves: BrainwaveData,
    mentalState: MentalState
  ): Promise<void> {
    switch (session.mode) {
      case 'thought_to_music':
        await this.generateThoughtMusic(session, brainwaves, mentalState);
        break;
      case 'meditation_enhancement':
        await this.generateMeditationMusic(session, brainwaves, mentalState);
        break;
      case 'flow_state':
        await this.generateFlowStateMusic(session, brainwaves, mentalState);
        break;
      case 'emotional_expression':
        await this.generateEmotionalMusic(session, brainwaves, mentalState);
        break;
      case 'creative_amplification':
        await this.generateCreativeMusic(session, brainwaves, mentalState);
        break;
    }
  }

  private async generateThoughtMusic(
    session: NeuralMusicSession,
    brainwaves: BrainwaveData,
    mentalState: MentalState
  ): Promise<void> {
    // Direct mapping of brainwaves to musical parameters
    const music = await this.musicSynthesizer.synthesize({
      brainwaves: brainwaves.processed,
      mapping: {
        alphaToPitch: {
          inputRange: [0, 1],
          outputRange: [48, 84], // MIDI note range
          curve: 'logarithmic',
          smoothing: 0.3
        },
        betaToRhythm: {
          inputRange: [0, 1],
          outputRange: [60, 180], // BPM range
          curve: 'linear',
          smoothing: 0.5
        },
        thetaToHarmony: {
          inputRange: [0, 1],
          outputRange: [0, 7], // Scale degree
          curve: 'linear',
          smoothing: 0.4
        },
        gammaToTimbre: {
          inputRange: [0, 1],
          outputRange: [0, 1], // Brightness
          curve: 'exponential',
          smoothing: 0.2
        },
        asymmetryToStereo: {
          inputRange: [-1, 1],
          outputRange: [-1, 1], // Pan position
          curve: 'linear',
          smoothing: 0.5
        }
      },
      mode: 'direct',
      mentalState
    });

    session.generatedMusic.push(music);
    this.emit('musicGenerated', { sessionId: session.id, music });
  }

  private async generateMeditationMusic(
    session: NeuralMusicSession,
    brainwaves: BrainwaveData,
    mentalState: MentalState
  ): Promise<void> {
    // Generate music that guides toward meditation states
    const targetAlpha = 0.7;
    const currentAlpha = brainwaves.processed.alpha.power;

    // Provide auditory feedback through music
    const feedbackIntensity = targetAlpha - currentAlpha;

    const music = await this.musicSynthesizer.synthesize({
      brainwaves: brainwaves.processed,
      mapping: this.getMeditationMapping(),
      mode: 'therapeutic',
      mentalState,
      feedbackParams: {
        target: 'alpha_increase',
        intensity: feedbackIntensity,
        technique: 'binaural_beats'
      }
    });

    // Apply binaural beats for alpha entrainment
    const binauralFrequency = 10; // 10 Hz for alpha
    await this.musicSynthesizer.applyBinauralBeats(music, binauralFrequency);

    session.generatedMusic.push(music);

    // Log feedback
    session.feedback.push({
      timestamp: new Date(),
      targetState: 'deep_meditation',
      currentState: mentalState.primary,
      feedback: {
        type: 'musical',
        intensity: Math.abs(feedbackIntensity),
        content: { binauralFrequency, alphaGuidance: feedbackIntensity }
      },
      response: {
        physiological: { brainwaveChange: {}, heartRateChange: 0, skinConductanceChange: 0 },
        behavioral: { focusImproved: false, relaxationAchieved: false, targetStateDuration: 0 }
      }
    });
  }

  private async generateFlowStateMusic(
    session: NeuralMusicSession,
    brainwaves: BrainwaveData,
    mentalState: MentalState
  ): Promise<void> {
    // Optimize for alpha-theta border and focused attention
    const flowIndicators = this.calculateFlowIndicators(brainwaves.processed);

    const music = await this.musicSynthesizer.synthesize({
      brainwaves: brainwaves.processed,
      mapping: this.getFlowStateMapping(),
      mode: 'interpreted',
      mentalState,
      flowOptimization: {
        targetGamma: 0.6,
        alphaTheteRatio: 1.5,
        betaSuppression: 0.3
      }
    });

    session.generatedMusic.push(music);
  }

  private async generateEmotionalMusic(
    session: NeuralMusicSession,
    brainwaves: BrainwaveData,
    mentalState: MentalState
  ): Promise<void> {
    // Map emotional content to expressive music
    const emotionalSignature = this.extractEmotionalSignature(brainwaves.processed);

    const music = await this.musicSynthesizer.synthesize({
      brainwaves: brainwaves.processed,
      mapping: this.getEmotionalMapping(),
      mode: 'creative',
      mentalState,
      emotionalParams: emotionalSignature
    });

    session.generatedMusic.push(music);
  }

  private async generateCreativeMusic(
    session: NeuralMusicSession,
    brainwaves: BrainwaveData,
    mentalState: MentalState
  ): Promise<void> {
    // Amplify creative patterns in brainwaves
    const creativeIndicators = this.detectCreativePatterns(brainwaves.processed);

    const music = await this.musicSynthesizer.synthesize({
      brainwaves: brainwaves.processed,
      mapping: this.getCreativeMapping(),
      mode: 'creative',
      mentalState,
      creativeParams: {
        noveltyBoost: creativeIndicators.novelty,
        divergentThinking: creativeIndicators.divergence,
        associativeLinks: creativeIndicators.associations
      }
    });

    session.generatedMusic.push(music);
  }

  private getMeditationMapping(): BrainwaveMapping {
    return {
      alphaToPitch: { inputRange: [0, 1], outputRange: [60, 72], curve: 'linear', smoothing: 0.7 },
      betaToRhythm: { inputRange: [0, 1], outputRange: [40, 60], curve: 'linear', smoothing: 0.8 },
      thetaToHarmony: { inputRange: [0, 1], outputRange: [0, 4], curve: 'linear', smoothing: 0.6 },
      gammaToTimbre: { inputRange: [0, 1], outputRange: [0.2, 0.5], curve: 'linear', smoothing: 0.5 },
      asymmetryToStereo: { inputRange: [-1, 1], outputRange: [-0.5, 0.5], curve: 'linear', smoothing: 0.9 }
    };
  }

  private getFlowStateMapping(): BrainwaveMapping {
    return {
      alphaToPitch: { inputRange: [0, 1], outputRange: [48, 84], curve: 'logarithmic', smoothing: 0.4 },
      betaToRhythm: { inputRange: [0, 1], outputRange: [90, 130], curve: 'linear', smoothing: 0.3 },
      thetaToHarmony: { inputRange: [0, 1], outputRange: [0, 7], curve: 'linear', smoothing: 0.4 },
      gammaToTimbre: { inputRange: [0, 1], outputRange: [0.4, 0.9], curve: 'exponential', smoothing: 0.2 },
      asymmetryToStereo: { inputRange: [-1, 1], outputRange: [-1, 1], curve: 'linear', smoothing: 0.3 }
    };
  }

  private getEmotionalMapping(): BrainwaveMapping {
    return {
      alphaToPitch: { inputRange: [0, 1], outputRange: [36, 96], curve: 'sigmoid', smoothing: 0.5 },
      betaToRhythm: { inputRange: [0, 1], outputRange: [50, 180], curve: 'exponential', smoothing: 0.4 },
      thetaToHarmony: { inputRange: [0, 1], outputRange: [0, 11], curve: 'linear', smoothing: 0.3 },
      gammaToTimbre: { inputRange: [0, 1], outputRange: [0, 1], curve: 'linear', smoothing: 0.2 },
      asymmetryToStereo: { inputRange: [-1, 1], outputRange: [-1, 1], curve: 'sigmoid', smoothing: 0.4 }
    };
  }

  private getCreativeMapping(): BrainwaveMapping {
    return {
      alphaToPitch: { inputRange: [0, 1], outputRange: [24, 108], curve: 'logarithmic', smoothing: 0.2 },
      betaToRhythm: { inputRange: [0, 1], outputRange: [30, 200], curve: 'exponential', smoothing: 0.1 },
      thetaToHarmony: { inputRange: [0, 1], outputRange: [0, 11], curve: 'sigmoid', smoothing: 0.2 },
      gammaToTimbre: { inputRange: [0, 1], outputRange: [0, 1], curve: 'exponential', smoothing: 0.1 },
      asymmetryToStereo: { inputRange: [-1, 1], outputRange: [-1, 1], curve: 'linear', smoothing: 0.2 }
    };
  }

  private calculateFlowIndicators(brainwaves: ProcessedBrainwaves): FlowIndicators {
    return {
      alphaTheta: brainwaves.alpha.power / (brainwaves.theta.power + 0.001),
      gammaBurst: brainwaves.gamma.power > 0.5,
      coherence: brainwaves.alpha.coherence,
      focus: brainwaves.beta.power * 0.3 + brainwaves.gamma.power * 0.7
    };
  }

  private extractEmotionalSignature(brainwaves: ProcessedBrainwaves): EmotionalSignature {
    // Frontal alpha asymmetry correlates with approach/withdrawal
    const valence = brainwaves.alpha.asymmetry;

    // Beta and gamma activity correlates with arousal
    const arousal = brainwaves.beta.power * 0.5 + brainwaves.gamma.power * 0.5;

    return {
      valence: valence * 2 - 1, // Scale to -1 to 1
      arousal: arousal * 2 - 1,
      dominance: brainwaves.beta.dominance,
      complexity: brainwaves.gamma.power,
      tension: brainwaves.beta.power,
      transcendence: brainwaves.theta.power * brainwaves.alpha.power
    };
  }

  private detectCreativePatterns(brainwaves: ProcessedBrainwaves): CreativeIndicators {
    return {
      novelty: brainwaves.theta.power * brainwaves.gamma.power,
      divergence: brainwaves.alpha.coherence * (1 - brainwaves.beta.power),
      associations: brainwaves.gamma.peaks.length / 10
    };
  }

  private async loadOrCreateCalibration(userId: string, device: BCIDevice): Promise<CalibrationProfile> {
    // In production, load from database
    return this.createDefaultCalibration(userId, device);
  }

  private createDefaultCalibration(userId: string, device: BCIDevice): CalibrationProfile {
    return {
      userId,
      deviceId: device.id,
      baselineRecording: {
        eyesOpen: this.emptyBrainwaves(),
        eyesClosed: this.emptyBrainwaves(),
        restingState: this.emptyBrainwaves(),
        duration: 60
      },
      mentalStateSignatures: new Map(),
      musicalIntentPatterns: new Map(),
      noiseFloor: 0.1,
      personalThresholds: {
        alpha: { low: 0.3, high: 0.7 },
        theta: { low: 0.2, high: 0.6 },
        beta: { low: 0.3, high: 0.7 },
        gamma: { low: 0.2, high: 0.5 },
        focusThreshold: 0.6,
        relaxationThreshold: 0.5,
        emotionalIntensity: 0.5
      },
      lastCalibrated: new Date()
    };
  }

  private emptyBrainwaves(): ProcessedBrainwaves {
    return {
      delta: { power: 0, dominance: 0, coherence: 0, asymmetry: 0, peaks: [] },
      theta: { power: 0, dominance: 0, coherence: 0, asymmetry: 0, peaks: [] },
      alpha: { power: 0, dominance: 0, coherence: 0, asymmetry: 0, peaks: [] },
      beta: { power: 0, dominance: 0, coherence: 0, asymmetry: 0, peaks: [] },
      gamma: { power: 0, dominance: 0, coherence: 0, asymmetry: 0, peaks: [] },
      raw: new Float32Array(0)
    };
  }

  private startBrainwaveStream(session: NeuralMusicSession, device: BCIDevice): void {
    // In production, connect to actual BCI device
    this.emit('streamStarted', { sessionId: session.id, device });
  }

  private generateSessionId(): string {
    return `neural_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
}

interface FlowIndicators {
  alphaTheta: number;
  gammaBurst: boolean;
  coherence: number;
  focus: number;
}

interface CreativeIndicators {
  novelty: number;
  divergence: number;
  associations: number;
}

// ============================================================
// DREAM SOUNDTRACK SYSTEM
// ============================================================

export class DreamSoundtrackSystem extends EventEmitter {
  private sleepMonitor: SleepStageMonitor;
  private dreamAnalyzer: DreamAnalyzer;
  private soundtrackGenerator: DreamMusicGenerator;
  private activeSessions: Map<string, DreamSession> = new Map();

  constructor() {
    super();
    this.sleepMonitor = new SleepStageMonitor();
    this.dreamAnalyzer = new DreamAnalyzer();
    this.soundtrackGenerator = new DreamMusicGenerator();
  }

  async startNightSession(userId: string, device: BCIDevice): Promise<DreamSession> {
    const session: DreamSession = {
      id: this.generateSessionId(),
      userId,
      nightOf: new Date(),
      sleepStages: [],
      dreams: [],
      generatedSoundtrack: null,
      analysis: {
        totalDuration: 0,
        remPercentage: 0,
        deepSleepPercentage: 0,
        dreamCount: 0,
        emotionalJourney: [],
        musicalCorrelations: []
      }
    };

    this.activeSessions.set(session.id, session);

    // Start monitoring
    await this.beginSleepMonitoring(session, device);

    this.emit('nightSessionStarted', session);
    return session;
  }

  async processSleepData(sessionId: string, brainwaves: BrainwaveData): Promise<void> {
    const session = this.activeSessions.get(sessionId);
    if (!session) throw new Error('Session not found');

    // Detect sleep stage
    const stage = await this.sleepMonitor.detectStage(brainwaves);

    // Update or create stage entry
    this.updateSleepStage(session, stage, brainwaves);

    // Detect REM and dreams
    if (stage === 'rem') {
      await this.processDreamActivity(session, brainwaves);
    }

    // Generate real-time dream music if enabled
    if (session.generatedSoundtrack?.isRealtime) {
      await this.generateRealtimeDreamMusic(session, brainwaves, stage);
    }

    this.emit('sleepDataProcessed', { sessionId, stage });
  }

  async endNightSession(sessionId: string): Promise<DreamSession> {
    const session = this.activeSessions.get(sessionId);
    if (!session) throw new Error('Session not found');

    // Finalize analysis
    session.analysis = await this.dreamAnalyzer.analyzeSleepSession(session);

    // Generate complete dream soundtrack
    session.generatedSoundtrack = await this.soundtrackGenerator.generateCompleteSoundtrack(
      session.dreams,
      session.sleepStages
    );

    this.activeSessions.delete(sessionId);
    this.emit('nightSessionEnded', session);

    return session;
  }

  async playDreamSoundtrack(sessionId: string): Promise<PlaybackController> {
    const session = this.activeSessions.get(sessionId);
    if (!session?.generatedSoundtrack) {
      throw new Error('No soundtrack available');
    }

    return {
      play: () => this.emit('playbackStarted', { sessionId }),
      pause: () => this.emit('playbackPaused', { sessionId }),
      seek: (time: number) => this.emit('playbackSeeked', { sessionId, time }),
      getCurrentDream: () => this.findDreamAtTime(session, Date.now()),
      getSleepStage: () => this.getCurrentSleepStage(session)
    };
  }

  private async beginSleepMonitoring(session: DreamSession, device: BCIDevice): Promise<void> {
    // Initialize real-time soundtrack generation
    session.generatedSoundtrack = {
      id: `soundtrack_${session.id}`,
      duration: 0,
      sections: [],
      audioData: null,
      isRealtime: true
    };

    this.emit('monitoringStarted', { sessionId: session.id });
  }

  private updateSleepStage(
    session: DreamSession,
    stage: 'awake' | 'light' | 'deep' | 'rem',
    brainwaves: BrainwaveData
  ): void {
    const lastStage = session.sleepStages[session.sleepStages.length - 1];

    if (lastStage && lastStage.stage === stage) {
      // Extend current stage
      lastStage.endTime = new Date();
    } else {
      // Start new stage
      if (lastStage) {
        lastStage.endTime = new Date();
      }

      session.sleepStages.push({
        startTime: new Date(),
        endTime: new Date(),
        stage,
        brainwaveProfile: brainwaves.processed,
        events: []
      });
    }
  }

  private async processDreamActivity(session: DreamSession, brainwaves: BrainwaveData): Promise<void> {
    // Detect REM bursts and estimate dream content
    const dreamContent = await this.dreamAnalyzer.estimateDreamContent(brainwaves);
    const emotionalProfile = this.extractEmotionalProfile(brainwaves);

    const dreamCapture: DreamCapture = {
      timestamp: new Date(),
      remPeriod: session.dreams.length + 1,
      brainwaveIntensity: brainwaves.processed,
      estimatedContent: dreamContent,
      emotionalProfile
    };

    session.dreams.push(dreamCapture);
    this.emit('dreamCaptured', { sessionId: session.id, dream: dreamCapture });
  }

  private async generateRealtimeDreamMusic(
    session: DreamSession,
    brainwaves: BrainwaveData,
    stage: string
  ): Promise<void> {
    const section = await this.soundtrackGenerator.generateSection({
      brainwaves: brainwaves.processed,
      sleepStage: stage,
      emotionalProfile: this.extractEmotionalProfile(brainwaves),
      duration: 10 // 10 second sections
    });

    session.generatedSoundtrack!.sections.push(section);
    session.generatedSoundtrack!.duration += section.duration;

    this.emit('realtimeMusicGenerated', { sessionId: session.id, section });
  }

  private extractEmotionalProfile(brainwaves: BrainwaveData): EmotionalSignature {
    const processed = brainwaves.processed;
    return {
      valence: processed.alpha.asymmetry,
      arousal: processed.beta.power,
      dominance: processed.gamma.power,
      complexity: processed.theta.power * processed.gamma.power,
      tension: processed.beta.dominance,
      transcendence: processed.theta.power
    };
  }

  private findDreamAtTime(session: DreamSession, time: number): DreamCapture | null {
    return session.dreams.find(d => d.timestamp.getTime() <= time) || null;
  }

  private getCurrentSleepStage(session: DreamSession): SleepStage | null {
    return session.sleepStages[session.sleepStages.length - 1] || null;
  }

  private generateSessionId(): string {
    return `dream_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }
}

interface PlaybackController {
  play: () => void;
  pause: () => void;
  seek: (time: number) => void;
  getCurrentDream: () => DreamCapture | null;
  getSleepStage: () => SleepStage | null;
}

// ============================================================
// NEUROFEEDBACK ENGINE
// ============================================================

export class NeurofeedbackMusicEngine extends EventEmitter {
  private targetStates: Map<string, TargetState> = new Map();
  private musicAdaptation: MusicAdaptationSystem;
  private effectivenessTracker: EffectivenessTracker;

  constructor() {
    super();
    this.musicAdaptation = new MusicAdaptationSystem();
    this.effectivenessTracker = new EffectivenessTracker();
    this.initializeTargetStates();
  }

  private initializeTargetStates(): void {
    this.targetStates.set('focus', {
      name: 'focus',
      brainwaveProfile: {
        delta: { power: 0.1, range: [0, 0.2] },
        theta: { power: 0.2, range: [0.1, 0.3] },
        alpha: { power: 0.4, range: [0.3, 0.5] },
        beta: { power: 0.6, range: [0.5, 0.8] },
        gamma: { power: 0.5, range: [0.3, 0.7] }
      },
      musicalCharacteristics: {
        tempo: 100,
        complexity: 0.4,
        tonality: 'major',
        instrumentation: ['piano', 'strings']
      }
    });

    this.targetStates.set('relaxation', {
      name: 'relaxation',
      brainwaveProfile: {
        delta: { power: 0.2, range: [0.1, 0.3] },
        theta: { power: 0.4, range: [0.3, 0.5] },
        alpha: { power: 0.7, range: [0.6, 0.9] },
        beta: { power: 0.2, range: [0.1, 0.3] },
        gamma: { power: 0.1, range: [0, 0.2] }
      },
      musicalCharacteristics: {
        tempo: 60,
        complexity: 0.2,
        tonality: 'minor',
        instrumentation: ['ambient', 'pad']
      }
    });

    this.targetStates.set('creativity', {
      name: 'creativity',
      brainwaveProfile: {
        delta: { power: 0.15, range: [0.1, 0.2] },
        theta: { power: 0.5, range: [0.4, 0.6] },
        alpha: { power: 0.5, range: [0.4, 0.6] },
        beta: { power: 0.4, range: [0.3, 0.5] },
        gamma: { power: 0.6, range: [0.5, 0.8] }
      },
      musicalCharacteristics: {
        tempo: 90,
        complexity: 0.6,
        tonality: 'modal',
        instrumentation: ['synth', 'world']
      }
    });

    this.targetStates.set('sleep', {
      name: 'sleep',
      brainwaveProfile: {
        delta: { power: 0.8, range: [0.7, 1.0] },
        theta: { power: 0.3, range: [0.2, 0.4] },
        alpha: { power: 0.1, range: [0, 0.2] },
        beta: { power: 0.05, range: [0, 0.1] },
        gamma: { power: 0.02, range: [0, 0.05] }
      },
      musicalCharacteristics: {
        tempo: 50,
        complexity: 0.1,
        tonality: 'drone',
        instrumentation: ['ambient', 'nature']
      }
    });
  }

  async trainTowardsState(
    sessionId: string,
    targetState: string,
    currentBrainwaves: ProcessedBrainwaves
  ): Promise<NeurofeedbackMusic> {
    const target = this.targetStates.get(targetState);
    if (!target) throw new Error('Unknown target state');

    // Calculate deviation from target
    const deviation = this.calculateDeviation(currentBrainwaves, target.brainwaveProfile);

    // Generate adaptive music
    const music = await this.musicAdaptation.generateAdaptiveMusic({
      target: target.musicalCharacteristics,
      deviation,
      currentBrainwaves,
      reinforcementStrength: this.calculateReinforcementStrength(deviation)
    });

    // Track effectiveness
    this.effectivenessTracker.recordFeedback(sessionId, targetState, deviation);

    return music;
  }

  private calculateDeviation(
    current: ProcessedBrainwaves,
    target: BrainwaveProfile
  ): BrainwaveDeviation {
    return {
      delta: current.delta.power - target.delta.power,
      theta: current.theta.power - target.theta.power,
      alpha: current.alpha.power - target.alpha.power,
      beta: current.beta.power - target.beta.power,
      gamma: current.gamma.power - target.gamma.power,
      overall: this.calculateOverallDeviation(current, target)
    };
  }

  private calculateOverallDeviation(
    current: ProcessedBrainwaves,
    target: BrainwaveProfile
  ): number {
    const deviations = [
      Math.abs(current.delta.power - target.delta.power),
      Math.abs(current.theta.power - target.theta.power),
      Math.abs(current.alpha.power - target.alpha.power),
      Math.abs(current.beta.power - target.beta.power),
      Math.abs(current.gamma.power - target.gamma.power)
    ];

    return deviations.reduce((sum, d) => sum + d, 0) / deviations.length;
  }

  private calculateReinforcementStrength(deviation: BrainwaveDeviation): number {
    // Higher strength when closer to target
    return 1 - Math.min(1, deviation.overall);
  }

  getStateProgress(sessionId: string, targetState: string): StateProgress {
    return this.effectivenessTracker.getProgress(sessionId, targetState);
  }
}

interface TargetState {
  name: string;
  brainwaveProfile: BrainwaveProfile;
  musicalCharacteristics: MusicalCharacteristics;
}

interface BrainwaveProfile {
  delta: { power: number; range: [number, number] };
  theta: { power: number; range: [number, number] };
  alpha: { power: number; range: [number, number] };
  beta: { power: number; range: [number, number] };
  gamma: { power: number; range: [number, number] };
}

interface MusicalCharacteristics {
  tempo: number;
  complexity: number;
  tonality: string;
  instrumentation: string[];
}

interface BrainwaveDeviation {
  delta: number;
  theta: number;
  alpha: number;
  beta: number;
  gamma: number;
  overall: number;
}

interface NeurofeedbackMusic {
  audioData: Float32Array | null;
  binauralComponent: BinauralComponent;
  adaptiveElements: AdaptiveElement[];
  reinforcementSignals: ReinforcementSignal[];
}

interface BinauralComponent {
  frequency: number;
  intensity: number;
  carrier: number;
}

interface AdaptiveElement {
  type: 'tempo' | 'harmony' | 'timbre' | 'dynamics';
  value: number;
  target: number;
}

interface ReinforcementSignal {
  type: 'reward' | 'guidance';
  intensity: number;
  timing: number;
}

interface StateProgress {
  sessionId: string;
  targetState: string;
  progress: number;
  timeInState: number;
  bestStreak: number;
}

// ============================================================
// HELPER CLASSES
// ============================================================

class NeuralSignalProcessor {
  async process(rawData: Float32Array[], calibration: CalibrationProfile): Promise<BrainwaveData> {
    // In production: FFT, artifact removal, frequency band extraction
    return {
      timestamp: Date.now(),
      channels: [],
      quality: { overall: 0.9, channelQualities: new Map(), noiseLevel: 0.1, contactQuality: 0.95 },
      artifacts: { eyeBlinks: [], muscleArtifacts: [], electrodeArtifacts: [], environmentalNoise: [] },
      processed: {
        delta: { power: 0.2, dominance: 0.1, coherence: 0.5, asymmetry: 0, peaks: [] },
        theta: { power: 0.3, dominance: 0.15, coherence: 0.6, asymmetry: 0.1, peaks: [] },
        alpha: { power: 0.5, dominance: 0.3, coherence: 0.7, asymmetry: -0.1, peaks: [] },
        beta: { power: 0.4, dominance: 0.25, coherence: 0.6, asymmetry: 0.05, peaks: [] },
        gamma: { power: 0.2, dominance: 0.1, coherence: 0.5, asymmetry: 0, peaks: [] },
        raw: new Float32Array(rawData.flat())
      }
    };
  }
}

class NeuralMusicSynthesizer {
  async synthesize(params: any): Promise<GeneratedNeuralMusic> {
    return {
      id: `neural_music_${Date.now()}`,
      timestamp: new Date(),
      brainwaveSource: params.sessionId || 'live',
      audioData: null,
      midiData: { tracks: [], tempo: 120, brainwaveMapping: params.mapping },
      parameters: {
        mappingMode: params.mode,
        complexity: 0.5,
        tonality: 'adaptive',
        tempo: 'brainwave_synced',
        instrumentation: ['synth', 'pad'],
        spatialMode: 'binaural'
      },
      emotionalContent: params.mentalState ? this.stateToEmotion(params.mentalState) : {
        valence: 0, arousal: 0, dominance: 0, complexity: 0, tension: 0, transcendence: 0
      },
      duration: 10
    };
  }

  async applyBinauralBeats(music: GeneratedNeuralMusic, frequency: number): Promise<void> {
    // Apply binaural beat processing
  }

  private stateToEmotion(state: MentalState): EmotionalSignature {
    return {
      valence: state.intensity * 0.5,
      arousal: state.intensity,
      dominance: 0.5,
      complexity: 0.5,
      tension: state.primary.includes('focus') ? 0.3 : 0.1,
      transcendence: state.primary.includes('meditation') ? 0.7 : 0.2
    };
  }
}

class MentalStateClassifier {
  async classify(brainwaves: BrainwaveData): Promise<MentalState> {
    const processed = brainwaves.processed;

    // Simple classification based on dominant frequency bands
    let primary = 'neutral';
    let intensity = 0.5;

    if (processed.alpha.dominance > 0.4) {
      primary = 'relaxed';
      intensity = processed.alpha.power;
    } else if (processed.beta.dominance > 0.4) {
      primary = 'focused';
      intensity = processed.beta.power;
    } else if (processed.theta.dominance > 0.4) {
      primary = 'creative';
      intensity = processed.theta.power;
    } else if (processed.gamma.dominance > 0.3) {
      primary = 'peak_performance';
      intensity = processed.gamma.power;
    }

    return {
      primary,
      intensity,
      subStates: [],
      musicAffinity: this.getMusicAffinityForState(primary)
    };
  }

  private getMusicAffinityForState(state: string): string[] {
    const affinities: Record<string, string[]> = {
      'relaxed': ['ambient', 'classical', 'nature'],
      'focused': ['electronic', 'minimal', 'instrumental'],
      'creative': ['experimental', 'jazz', 'world'],
      'peak_performance': ['epic', 'electronic', 'energetic'],
      'neutral': ['pop', 'indie', 'varied']
    };
    return affinities[state] || affinities['neutral'];
  }
}

class NeurofeedbackEngine {
  async provideFeedback(session: NeuralMusicSession, target: string): Promise<void> {
    // Provide real-time neurofeedback through music
  }
}

class SleepStageMonitor {
  async detectStage(brainwaves: BrainwaveData): Promise<'awake' | 'light' | 'deep' | 'rem'> {
    const processed = brainwaves.processed;

    // Simplified sleep stage detection
    if (processed.delta.dominance > 0.5) return 'deep';
    if (processed.theta.dominance > 0.4 && processed.beta.power < 0.2) return 'rem';
    if (processed.alpha.power < 0.3 && processed.beta.power < 0.4) return 'light';
    return 'awake';
  }
}

class DreamAnalyzer {
  async estimateDreamContent(brainwaves: BrainwaveData): Promise<DreamContent> {
    const processed = brainwaves.processed;

    return {
      estimatedThemes: this.inferThemes(processed),
      emotionalTone: this.inferEmotionalTone(processed),
      visualActivity: processed.gamma.power,
      narrativeComplexity: processed.theta.power * processed.beta.power,
      lucidityIndicators: processed.gamma.power * processed.beta.power
    };
  }

  async analyzeSleepSession(session: DreamSession): Promise<DreamAnalysis> {
    const totalDuration = session.sleepStages.reduce((sum, stage) =>
      sum + (stage.endTime.getTime() - stage.startTime.getTime()), 0
    );

    const remTime = session.sleepStages
      .filter(s => s.stage === 'rem')
      .reduce((sum, stage) => sum + (stage.endTime.getTime() - stage.startTime.getTime()), 0);

    const deepTime = session.sleepStages
      .filter(s => s.stage === 'deep')
      .reduce((sum, stage) => sum + (stage.endTime.getTime() - stage.startTime.getTime()), 0);

    return {
      totalDuration,
      remPercentage: remTime / totalDuration,
      deepSleepPercentage: deepTime / totalDuration,
      dreamCount: session.dreams.length,
      emotionalJourney: session.dreams.map(d => d.emotionalProfile),
      musicalCorrelations: this.analyzeMusicalCorrelations(session)
    };
  }

  private inferThemes(brainwaves: ProcessedBrainwaves): string[] {
    const themes: string[] = [];
    if (brainwaves.gamma.power > 0.5) themes.push('visual', 'vivid');
    if (brainwaves.theta.power > 0.5) themes.push('emotional', 'abstract');
    if (brainwaves.alpha.power > 0.4) themes.push('peaceful', 'familiar');
    return themes.length > 0 ? themes : ['neutral'];
  }

  private inferEmotionalTone(brainwaves: ProcessedBrainwaves): string {
    if (brainwaves.alpha.asymmetry > 0.2) return 'positive';
    if (brainwaves.alpha.asymmetry < -0.2) return 'negative';
    if (brainwaves.beta.power > 0.4) return 'anxious';
    return 'neutral';
  }

  private analyzeMusicalCorrelations(session: DreamSession): MusicalCorrelation[] {
    return session.sleepStages.map(stage => ({
      sleepStage: stage.stage,
      musicalCharacteristics: this.stageToMusic(stage.stage),
      brainwavePatterns: this.describeBrainwaves(stage.brainwaveProfile)
    }));
  }

  private stageToMusic(stage: string): string[] {
    const mapping: Record<string, string[]> = {
      'awake': ['ambient', 'light'],
      'light': ['drone', 'minimal'],
      'deep': ['sub-bass', 'slow'],
      'rem': ['dreamlike', 'surreal', 'evolving']
    };
    return mapping[stage] || ['ambient'];
  }

  private describeBrainwaves(brainwaves: ProcessedBrainwaves): string[] {
    const patterns: string[] = [];
    if (brainwaves.delta.dominance > 0.3) patterns.push('delta-dominant');
    if (brainwaves.theta.dominance > 0.3) patterns.push('theta-prominent');
    if (brainwaves.alpha.dominance > 0.3) patterns.push('alpha-present');
    return patterns;
  }
}

class DreamMusicGenerator {
  async generateSection(params: {
    brainwaves: ProcessedBrainwaves;
    sleepStage: string;
    emotionalProfile: EmotionalSignature;
    duration: number;
  }): Promise<DreamMusicSection> {
    return {
      startTime: Date.now(),
      duration: params.duration,
      sleepStage: params.sleepStage,
      mood: params.emotionalProfile,
      musicalElements: this.createElements(params.brainwaves, params.sleepStage)
    };
  }

  async generateCompleteSoundtrack(
    dreams: DreamCapture[],
    sleepStages: SleepStage[]
  ): Promise<DreamSoundtrack> {
    const sections: DreamMusicSection[] = sleepStages.map((stage, i) => ({
      startTime: stage.startTime.getTime(),
      duration: stage.endTime.getTime() - stage.startTime.getTime(),
      sleepStage: stage.stage,
      mood: dreams[i]?.emotionalProfile || {
        valence: 0, arousal: 0, dominance: 0, complexity: 0, tension: 0, transcendence: 0
      },
      musicalElements: this.createElements(stage.brainwaveProfile, stage.stage)
    }));

    return {
      id: `soundtrack_${Date.now()}`,
      duration: sections.reduce((sum, s) => sum + s.duration, 0),
      sections,
      audioData: null,
      isRealtime: false
    };
  }

  private createElements(brainwaves: ProcessedBrainwaves, stage: string): MusicalElement[] {
    return [
      {
        type: 'texture',
        brainwaveSource: 'theta',
        parameters: { depth: brainwaves.theta.power, movement: brainwaves.theta.coherence }
      },
      {
        type: 'melody',
        brainwaveSource: 'alpha',
        parameters: { pitch: brainwaves.alpha.power, contour: brainwaves.alpha.asymmetry }
      }
    ];
  }
}

class MusicAdaptationSystem {
  async generateAdaptiveMusic(params: any): Promise<NeurofeedbackMusic> {
    return {
      audioData: null,
      binauralComponent: {
        frequency: 10, // Alpha entrainment
        intensity: params.reinforcementStrength,
        carrier: 200
      },
      adaptiveElements: [
        { type: 'tempo', value: 80, target: params.target.tempo },
        { type: 'harmony', value: 0.5, target: 0.6 }
      ],
      reinforcementSignals: [
        { type: 'reward', intensity: params.reinforcementStrength, timing: 0 }
      ]
    };
  }
}

class EffectivenessTracker {
  private progress: Map<string, StateProgress> = new Map();

  recordFeedback(sessionId: string, targetState: string, deviation: BrainwaveDeviation): void {
    const key = `${sessionId}_${targetState}`;
    const existing = this.progress.get(key) || {
      sessionId,
      targetState,
      progress: 0,
      timeInState: 0,
      bestStreak: 0
    };

    existing.progress = 1 - deviation.overall;
    if (deviation.overall < 0.2) {
      existing.timeInState += 1;
      existing.bestStreak = Math.max(existing.bestStreak, existing.timeInState);
    } else {
      existing.timeInState = 0;
    }

    this.progress.set(key, existing);
  }

  getProgress(sessionId: string, targetState: string): StateProgress {
    return this.progress.get(`${sessionId}_${targetState}`) || {
      sessionId,
      targetState,
      progress: 0,
      timeInState: 0,
      bestStreak: 0
    };
  }
}
