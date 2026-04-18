// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Native Audio Service
 * Enhanced audio playback with native features for mobile
 */

import { Audio, AVPlaybackStatus, AVPlaybackStatusSuccess } from 'expo-av';
import * as BackgroundFetch from 'expo-background-fetch';
import * as TaskManager from 'expo-task-manager';
import { Platform } from 'react-native';
import { OfflineManager } from '../offline/OfflineManager';

// Types
export interface Track {
  id: string;
  title: string;
  artist: string;
  album?: string;
  duration: number;
  coverArt: string;
  audioUrl: string;
  waveformData?: number[];
}

export interface PlaybackState {
  isPlaying: boolean;
  isLoading: boolean;
  isBuffering: boolean;
  currentTrack: Track | null;
  currentTime: number;
  duration: number;
  bufferedTime: number;
  volume: number;
  isMuted: boolean;
  playbackRate: number;
  repeatMode: RepeatMode;
  shuffleMode: boolean;
  error: string | null;
}

export type RepeatMode = 'off' | 'all' | 'one';

type PlaybackListener = (state: PlaybackState) => void;
type EventListener = (event: AudioEvent) => void;

export interface AudioEvent {
  type: 'track_started' | 'track_ended' | 'track_error' | 'queue_ended' | 'seek' | 'playback_rate_change';
  data?: any;
}

interface AudioSession {
  category: 'playback' | 'ambient' | 'soloAmbient';
  mode: 'default' | 'moviePlayback' | 'spokenAudio';
  interruptionMode: 'doNotMix' | 'duckOthers' | 'mixWithOthers';
  allowsRecording: boolean;
  playsInSilentMode: boolean;
  staysActiveInBackground: boolean;
}

const BACKGROUND_AUDIO_TASK = 'BACKGROUND_AUDIO_TASK';

// Register background task
TaskManager.defineTask(BACKGROUND_AUDIO_TASK, async () => {
  // Keep audio session alive in background
  return BackgroundFetch.BackgroundFetchResult.NewData;
});

export class NativeAudioService {
  private static instance: NativeAudioService;
  private sound: Audio.Sound | null = null;
  private offlineManager: OfflineManager;

  private currentTrack: Track | null = null;
  private queue: Track[] = [];
  private queueIndex: number = -1;
  private originalQueue: Track[] = [];

  private playbackState: PlaybackState = {
    isPlaying: false,
    isLoading: false,
    isBuffering: false,
    currentTrack: null,
    currentTime: 0,
    duration: 0,
    bufferedTime: 0,
    volume: 1,
    isMuted: false,
    playbackRate: 1,
    repeatMode: 'off',
    shuffleMode: false,
    error: null,
  };

  private playbackListeners: Set<PlaybackListener> = new Set();
  private eventListeners: Set<EventListener> = new Set();
  private statusUpdateInterval: NodeJS.Timeout | null = null;

  private constructor() {
    this.offlineManager = OfflineManager.getInstance();
    this.initialize();
  }

  static getInstance(): NativeAudioService {
    if (!NativeAudioService.instance) {
      NativeAudioService.instance = new NativeAudioService();
    }
    return NativeAudioService.instance;
  }

  /**
   * Initialize audio service
   */
  private async initialize(): Promise<void> {
    await this.setupAudioSession();
    await this.setupBackgroundAudio();
  }

  /**
   * Setup audio session for optimal playback
   */
  private async setupAudioSession(): Promise<void> {
    try {
      await Audio.setAudioModeAsync({
        allowsRecordingIOS: false,
        staysActiveInBackground: true,
        playsInSilentModeIOS: true,
        shouldDuckAndroid: true,
        playThroughEarpieceAndroid: false,
        interruptionModeIOS: 1, // DoNotMix
        interruptionModeAndroid: 1, // DoNotMix
      });
    } catch (error) {
      console.error('Failed to setup audio session:', error);
    }
  }

  /**
   * Setup background audio
   */
  private async setupBackgroundAudio(): Promise<void> {
    try {
      await BackgroundFetch.registerTaskAsync(BACKGROUND_AUDIO_TASK, {
        minimumInterval: 60,
        stopOnTerminate: false,
        startOnBoot: true,
      });
    } catch (error) {
      console.warn('Background audio task registration failed:', error);
    }
  }

  /**
   * Load and play a track
   */
  async loadTrack(track: Track): Promise<void> {
    this.updateState({ isLoading: true, error: null });

    try {
      // Unload previous track
      await this.unloadSound();

      // Check for offline version
      let audioSource: { uri: string } | number;
      const offlinePath = this.offlineManager.getLocalAudioPath(track.id);

      if (offlinePath) {
        audioSource = { uri: offlinePath };
        // Update last played for offline track
        await this.offlineManager.updateLastPlayed(track.id);
      } else {
        audioSource = { uri: track.audioUrl };
      }

      // Create and load sound
      const { sound } = await Audio.Sound.createAsync(
        audioSource,
        {
          shouldPlay: true,
          volume: this.playbackState.volume,
          rate: this.playbackState.playbackRate,
          shouldCorrectPitch: true,
          progressUpdateIntervalMillis: 100,
        },
        this.handlePlaybackStatusUpdate.bind(this)
      );

      this.sound = sound;
      this.currentTrack = track;

      this.updateState({
        isLoading: false,
        isPlaying: true,
        currentTrack: track,
        duration: track.duration,
      });

      this.emitEvent({ type: 'track_started', data: { track } });
      this.startStatusUpdateInterval();

      // Update Now Playing info
      await this.updateNowPlayingInfo();
    } catch (error: any) {
      console.error('Failed to load track:', error);
      this.updateState({
        isLoading: false,
        error: error.message || 'Failed to load track',
      });
      this.emitEvent({ type: 'track_error', data: { error } });
    }
  }

  /**
   * Play
   */
  async play(): Promise<void> {
    if (this.sound) {
      await this.sound.playAsync();
      this.updateState({ isPlaying: true });
    } else if (this.currentTrack) {
      await this.loadTrack(this.currentTrack);
    }
  }

  /**
   * Pause
   */
  async pause(): Promise<void> {
    if (this.sound) {
      await this.sound.pauseAsync();
      this.updateState({ isPlaying: false });
    }
  }

  /**
   * Toggle play/pause
   */
  async togglePlayPause(): Promise<void> {
    if (this.playbackState.isPlaying) {
      await this.pause();
    } else {
      await this.play();
    }
  }

  /**
   * Stop
   */
  async stop(): Promise<void> {
    this.stopStatusUpdateInterval();
    await this.unloadSound();
    this.updateState({
      isPlaying: false,
      currentTime: 0,
    });
  }

  /**
   * Seek to position
   */
  async seekTo(position: number): Promise<void> {
    if (this.sound) {
      await this.sound.setPositionAsync(position * 1000);
      this.updateState({ currentTime: position });
      this.emitEvent({ type: 'seek', data: { position } });
    }
  }

  /**
   * Skip forward
   */
  async skipForward(seconds: number = 15): Promise<void> {
    const newPosition = Math.min(
      this.playbackState.currentTime + seconds,
      this.playbackState.duration
    );
    await this.seekTo(newPosition);
  }

  /**
   * Skip backward
   */
  async skipBackward(seconds: number = 15): Promise<void> {
    const newPosition = Math.max(this.playbackState.currentTime - seconds, 0);
    await this.seekTo(newPosition);
  }

  /**
   * Set volume
   */
  async setVolume(volume: number): Promise<void> {
    const clampedVolume = Math.max(0, Math.min(1, volume));
    if (this.sound) {
      await this.sound.setVolumeAsync(clampedVolume);
    }
    this.updateState({ volume: clampedVolume, isMuted: clampedVolume === 0 });
  }

  /**
   * Toggle mute
   */
  async toggleMute(): Promise<void> {
    if (this.sound) {
      const newMuted = !this.playbackState.isMuted;
      await this.sound.setIsMutedAsync(newMuted);
      this.updateState({ isMuted: newMuted });
    }
  }

  /**
   * Set playback rate
   */
  async setPlaybackRate(rate: number): Promise<void> {
    const clampedRate = Math.max(0.5, Math.min(2, rate));
    if (this.sound) {
      await this.sound.setRateAsync(clampedRate, true);
    }
    this.updateState({ playbackRate: clampedRate });
    this.emitEvent({ type: 'playback_rate_change', data: { rate: clampedRate } });
  }

  /**
   * Set repeat mode
   */
  setRepeatMode(mode: RepeatMode): void {
    this.updateState({ repeatMode: mode });
  }

  /**
   * Cycle repeat mode
   */
  cycleRepeatMode(): void {
    const modes: RepeatMode[] = ['off', 'all', 'one'];
    const currentIndex = modes.indexOf(this.playbackState.repeatMode);
    const nextMode = modes[(currentIndex + 1) % modes.length];
    this.setRepeatMode(nextMode);
  }

  /**
   * Toggle shuffle
   */
  toggleShuffle(): void {
    const newShuffleMode = !this.playbackState.shuffleMode;

    if (newShuffleMode) {
      // Save original queue and shuffle
      this.originalQueue = [...this.queue];
      this.queue = this.shuffleArray([...this.queue]);
      // Keep current track at front
      if (this.currentTrack) {
        const currentIndex = this.queue.findIndex(t => t.id === this.currentTrack!.id);
        if (currentIndex > 0) {
          [this.queue[0], this.queue[currentIndex]] = [this.queue[currentIndex], this.queue[0]];
        }
        this.queueIndex = 0;
      }
    } else {
      // Restore original queue
      this.queue = [...this.originalQueue];
      if (this.currentTrack) {
        this.queueIndex = this.queue.findIndex(t => t.id === this.currentTrack!.id);
      }
    }

    this.updateState({ shuffleMode: newShuffleMode });
  }

  /**
   * Set queue
   */
  setQueue(tracks: Track[], startIndex: number = 0): void {
    this.queue = [...tracks];
    this.originalQueue = [...tracks];
    this.queueIndex = startIndex;

    if (this.playbackState.shuffleMode) {
      this.queue = this.shuffleArray([...tracks]);
      if (startIndex < tracks.length) {
        const currentTrack = tracks[startIndex];
        const newIndex = this.queue.findIndex(t => t.id === currentTrack.id);
        [this.queue[0], this.queue[newIndex]] = [this.queue[newIndex], this.queue[0]];
        this.queueIndex = 0;
      }
    }
  }

  /**
   * Add to queue
   */
  addToQueue(track: Track): void {
    this.queue.push(track);
    this.originalQueue.push(track);
  }

  /**
   * Add track to play next
   */
  playNext(track: Track): void {
    const insertIndex = this.queueIndex + 1;
    this.queue.splice(insertIndex, 0, track);
    this.originalQueue.splice(insertIndex, 0, track);
  }

  /**
   * Remove from queue
   */
  removeFromQueue(index: number): void {
    if (index < 0 || index >= this.queue.length) return;

    this.queue.splice(index, 1);
    this.originalQueue.splice(index, 1);

    if (index < this.queueIndex) {
      this.queueIndex--;
    }
  }

  /**
   * Clear queue
   */
  clearQueue(): void {
    this.queue = [];
    this.originalQueue = [];
    this.queueIndex = -1;
  }

  /**
   * Get queue
   */
  getQueue(): Track[] {
    return [...this.queue];
  }

  /**
   * Get current queue index
   */
  getCurrentQueueIndex(): number {
    return this.queueIndex;
  }

  /**
   * Play next track
   */
  async playNext(): Promise<void> {
    if (this.playbackState.repeatMode === 'one') {
      await this.seekTo(0);
      await this.play();
      return;
    }

    const nextIndex = this.queueIndex + 1;

    if (nextIndex >= this.queue.length) {
      if (this.playbackState.repeatMode === 'all') {
        this.queueIndex = 0;
        await this.loadTrack(this.queue[0]);
      } else {
        this.emitEvent({ type: 'queue_ended' });
        await this.stop();
      }
    } else {
      this.queueIndex = nextIndex;
      await this.loadTrack(this.queue[nextIndex]);
    }
  }

  /**
   * Play previous track
   */
  async playPrevious(): Promise<void> {
    // If more than 3 seconds in, restart current track
    if (this.playbackState.currentTime > 3) {
      await this.seekTo(0);
      return;
    }

    const prevIndex = this.queueIndex - 1;

    if (prevIndex < 0) {
      if (this.playbackState.repeatMode === 'all') {
        this.queueIndex = this.queue.length - 1;
        await this.loadTrack(this.queue[this.queueIndex]);
      } else {
        await this.seekTo(0);
      }
    } else {
      this.queueIndex = prevIndex;
      await this.loadTrack(this.queue[prevIndex]);
    }
  }

  /**
   * Get current playback state
   */
  getPlaybackState(): PlaybackState {
    return { ...this.playbackState };
  }

  /**
   * Add playback state listener
   */
  addPlaybackListener(listener: PlaybackListener): () => void {
    this.playbackListeners.add(listener);
    // Send current state immediately
    listener(this.playbackState);
    return () => this.playbackListeners.delete(listener);
  }

  /**
   * Add event listener
   */
  addEventListener(listener: EventListener): () => void {
    this.eventListeners.add(listener);
    return () => this.eventListeners.delete(listener);
  }

  /**
   * Cleanup
   */
  async destroy(): Promise<void> {
    this.stopStatusUpdateInterval();
    await this.unloadSound();
    this.playbackListeners.clear();
    this.eventListeners.clear();
    await BackgroundFetch.unregisterTaskAsync(BACKGROUND_AUDIO_TASK);
  }

  // Private methods

  private async unloadSound(): Promise<void> {
    if (this.sound) {
      try {
        await this.sound.stopAsync();
        await this.sound.unloadAsync();
      } catch (error) {
        console.warn('Error unloading sound:', error);
      }
      this.sound = null;
    }
  }

  private handlePlaybackStatusUpdate(status: AVPlaybackStatus): void {
    if (!status.isLoaded) {
      if (status.error) {
        console.error('Playback error:', status.error);
        this.updateState({ error: status.error });
      }
      return;
    }

    const successStatus = status as AVPlaybackStatusSuccess;

    this.updateState({
      isPlaying: successStatus.isPlaying,
      isBuffering: successStatus.isBuffering,
      currentTime: successStatus.positionMillis / 1000,
      duration: (successStatus.durationMillis || 0) / 1000,
      bufferedTime: (successStatus.playableDurationMillis || 0) / 1000,
    });

    // Handle track end
    if (successStatus.didJustFinish && !successStatus.isLooping) {
      this.emitEvent({ type: 'track_ended' });
      this.playNext();
    }
  }

  private updateState(updates: Partial<PlaybackState>): void {
    this.playbackState = { ...this.playbackState, ...updates };
    this.playbackListeners.forEach(listener => listener(this.playbackState));
  }

  private emitEvent(event: AudioEvent): void {
    this.eventListeners.forEach(listener => listener(event));
  }

  private startStatusUpdateInterval(): void {
    this.stopStatusUpdateInterval();
    this.statusUpdateInterval = setInterval(() => {
      if (this.sound) {
        this.sound.getStatusAsync().then(this.handlePlaybackStatusUpdate.bind(this));
      }
    }, 250);
  }

  private stopStatusUpdateInterval(): void {
    if (this.statusUpdateInterval) {
      clearInterval(this.statusUpdateInterval);
      this.statusUpdateInterval = null;
    }
  }

  private shuffleArray<T>(array: T[]): T[] {
    const shuffled = [...array];
    for (let i = shuffled.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [shuffled[i], shuffled[j]] = [shuffled[j], shuffled[i]];
    }
    return shuffled;
  }

  private async updateNowPlayingInfo(): Promise<void> {
    // For iOS, update MPNowPlayingInfoCenter
    // This would use a native module or expo-media-library
    if (this.currentTrack) {
      console.log('Updating Now Playing:', this.currentTrack.title);
    }
  }
}

export default NativeAudioService;
