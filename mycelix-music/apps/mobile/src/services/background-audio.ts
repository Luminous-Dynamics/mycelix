// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Background Audio Service
 *
 * Handles background audio playback, media controls, lock screen controls,
 * CarPlay/Android Auto integration, and audio session management.
 */

import TrackPlayer, {
  Event,
  State,
  Capability,
  RepeatMode,
  AppKilledPlaybackBehavior,
  Track,
  Progress,
} from 'react-native-track-player';
import { Platform } from 'react-native';

// ============================================================================
// Types
// ============================================================================

export interface AudioTrack {
  id: string;
  url: string;
  title: string;
  artist: string;
  album?: string;
  artwork?: string;
  duration: number;
  isLiked?: boolean;
  artistId?: string;
  albumId?: string;
}

export interface PlaybackState {
  isPlaying: boolean;
  isPaused: boolean;
  isBuffering: boolean;
  isStopped: boolean;
  position: number;
  duration: number;
  buffered: number;
  currentTrack: AudioTrack | null;
  queue: AudioTrack[];
  queuePosition: number;
  repeatMode: 'off' | 'track' | 'queue';
  shuffleEnabled: boolean;
}

export interface AudioQuality {
  bitrate: number;
  format: 'mp3' | 'aac' | 'flac';
}

type PlaybackEventCallback = (state: PlaybackState) => void;

// ============================================================================
// Background Audio Service
// ============================================================================

class BackgroundAudioService {
  private isInitialized = false;
  private playbackListeners: Set<PlaybackEventCallback> = new Set();
  private currentState: PlaybackState;
  private shuffleOrder: number[] = [];
  private originalQueue: AudioTrack[] = [];

  constructor() {
    this.currentState = this.getInitialState();
  }

  // ============================================================================
  // Initialization
  // ============================================================================

  async initialize(): Promise<void> {
    if (this.isInitialized) return;

    await TrackPlayer.setupPlayer({
      maxCacheSize: 1024 * 50, // 50MB cache
      autoHandleInterruptions: true,
    });

    await TrackPlayer.updateOptions({
      capabilities: [
        Capability.Play,
        Capability.Pause,
        Capability.Stop,
        Capability.SkipToNext,
        Capability.SkipToPrevious,
        Capability.SeekTo,
        Capability.Like,
        Capability.Dislike,
      ],
      compactCapabilities: [
        Capability.Play,
        Capability.Pause,
        Capability.SkipToNext,
      ],
      notificationCapabilities: [
        Capability.Play,
        Capability.Pause,
        Capability.SkipToNext,
        Capability.SkipToPrevious,
        Capability.SeekTo,
      ],
      progressUpdateEventInterval: 1,
      android: {
        appKilledPlaybackBehavior: AppKilledPlaybackBehavior.ContinuePlayback,
      },
    });

    // Register event listeners
    this.setupEventListeners();

    this.isInitialized = true;
  }

  private setupEventListeners(): void {
    TrackPlayer.addEventListener(Event.PlaybackState, async (event) => {
      await this.updatePlaybackState();
    });

    TrackPlayer.addEventListener(Event.PlaybackActiveTrackChanged, async (event) => {
      await this.updatePlaybackState();

      // Report track play to analytics
      if (event.track) {
        this.reportTrackPlay(event.track.id);
      }
    });

    TrackPlayer.addEventListener(Event.PlaybackProgressUpdated, async (event) => {
      this.currentState.position = event.position;
      this.currentState.duration = event.duration;
      this.currentState.buffered = event.buffered;
      this.notifyListeners();
    });

    TrackPlayer.addEventListener(Event.PlaybackQueueEnded, async () => {
      if (this.currentState.repeatMode === 'queue') {
        await TrackPlayer.seekTo(0);
        await TrackPlayer.skip(0);
        await TrackPlayer.play();
      }
    });

    TrackPlayer.addEventListener(Event.RemotePlay, () => this.play());
    TrackPlayer.addEventListener(Event.RemotePause, () => this.pause());
    TrackPlayer.addEventListener(Event.RemoteStop, () => this.stop());
    TrackPlayer.addEventListener(Event.RemoteNext, () => this.skipToNext());
    TrackPlayer.addEventListener(Event.RemotePrevious, () => this.skipToPrevious());
    TrackPlayer.addEventListener(Event.RemoteSeek, (event) => this.seekTo(event.position));
    TrackPlayer.addEventListener(Event.RemoteLike, () => this.toggleLike());
    TrackPlayer.addEventListener(Event.RemoteDislike, () => this.dislike());

    // CarPlay/Android Auto events
    TrackPlayer.addEventListener(Event.RemoteJumpForward, async (event) => {
      const position = await TrackPlayer.getPosition();
      await this.seekTo(position + (event.interval || 15));
    });

    TrackPlayer.addEventListener(Event.RemoteJumpBackward, async (event) => {
      const position = await TrackPlayer.getPosition();
      await this.seekTo(Math.max(0, position - (event.interval || 15)));
    });
  }

  // ============================================================================
  // Playback Controls
  // ============================================================================

  async play(): Promise<void> {
    await TrackPlayer.play();
    await this.updatePlaybackState();
  }

  async pause(): Promise<void> {
    await TrackPlayer.pause();
    await this.updatePlaybackState();
  }

  async stop(): Promise<void> {
    await TrackPlayer.stop();
    await TrackPlayer.reset();
    this.currentState = this.getInitialState();
    this.notifyListeners();
  }

  async seekTo(position: number): Promise<void> {
    await TrackPlayer.seekTo(position);
    this.currentState.position = position;
    this.notifyListeners();
  }

  async skipToNext(): Promise<void> {
    const queue = await TrackPlayer.getQueue();
    const currentIndex = await TrackPlayer.getActiveTrackIndex();

    if (currentIndex !== null && currentIndex < queue.length - 1) {
      await TrackPlayer.skipToNext();
    } else if (this.currentState.repeatMode === 'queue') {
      await TrackPlayer.skip(0);
    }

    await this.updatePlaybackState();
  }

  async skipToPrevious(): Promise<void> {
    const position = await TrackPlayer.getPosition();

    // If more than 3 seconds into track, restart it
    if (position > 3) {
      await TrackPlayer.seekTo(0);
    } else {
      const currentIndex = await TrackPlayer.getActiveTrackIndex();
      if (currentIndex !== null && currentIndex > 0) {
        await TrackPlayer.skipToPrevious();
      }
    }

    await this.updatePlaybackState();
  }

  async skipTo(index: number): Promise<void> {
    await TrackPlayer.skip(index);
    await this.updatePlaybackState();
  }

  // ============================================================================
  // Queue Management
  // ============================================================================

  async playTrack(track: AudioTrack): Promise<void> {
    await TrackPlayer.reset();
    await TrackPlayer.add(this.convertToPlayerTrack(track));
    await TrackPlayer.play();
    this.originalQueue = [track];
    await this.updatePlaybackState();
  }

  async playTracks(tracks: AudioTrack[], startIndex = 0): Promise<void> {
    await TrackPlayer.reset();

    this.originalQueue = tracks;
    const playerTracks = tracks.map(t => this.convertToPlayerTrack(t));

    if (this.currentState.shuffleEnabled) {
      this.shuffleOrder = this.generateShuffleOrder(tracks.length, startIndex);
      const shuffledTracks = this.shuffleOrder.map(i => playerTracks[i]);
      await TrackPlayer.add(shuffledTracks);
    } else {
      await TrackPlayer.add(playerTracks);
      if (startIndex > 0) {
        await TrackPlayer.skip(startIndex);
      }
    }

    await TrackPlayer.play();
    await this.updatePlaybackState();
  }

  async addToQueue(track: AudioTrack): Promise<void> {
    await TrackPlayer.add(this.convertToPlayerTrack(track));
    this.originalQueue.push(track);
    await this.updatePlaybackState();
  }

  async addNext(track: AudioTrack): Promise<void> {
    const currentIndex = await TrackPlayer.getActiveTrackIndex();
    const insertIndex = currentIndex !== null ? currentIndex + 1 : 0;

    await TrackPlayer.add(this.convertToPlayerTrack(track), insertIndex);
    this.originalQueue.splice(insertIndex, 0, track);
    await this.updatePlaybackState();
  }

  async removeFromQueue(index: number): Promise<void> {
    await TrackPlayer.remove(index);
    this.originalQueue.splice(index, 1);
    await this.updatePlaybackState();
  }

  async clearQueue(): Promise<void> {
    await TrackPlayer.reset();
    this.originalQueue = [];
    this.currentState.queue = [];
    this.notifyListeners();
  }

  async moveTrack(fromIndex: number, toIndex: number): Promise<void> {
    await TrackPlayer.move(fromIndex, toIndex);

    const track = this.originalQueue.splice(fromIndex, 1)[0];
    this.originalQueue.splice(toIndex, 0, track);

    await this.updatePlaybackState();
  }

  // ============================================================================
  // Repeat & Shuffle
  // ============================================================================

  async setRepeatMode(mode: 'off' | 'track' | 'queue'): Promise<void> {
    const trackPlayerMode = {
      off: RepeatMode.Off,
      track: RepeatMode.Track,
      queue: RepeatMode.Queue,
    }[mode];

    await TrackPlayer.setRepeatMode(trackPlayerMode);
    this.currentState.repeatMode = mode;
    this.notifyListeners();
  }

  async toggleShuffle(): Promise<void> {
    this.currentState.shuffleEnabled = !this.currentState.shuffleEnabled;

    if (this.currentState.shuffleEnabled) {
      // Shuffle the remaining queue
      const currentIndex = await TrackPlayer.getActiveTrackIndex();
      const queue = await TrackPlayer.getQueue();

      if (currentIndex !== null && queue.length > 1) {
        this.shuffleOrder = this.generateShuffleOrder(
          this.originalQueue.length,
          currentIndex
        );

        // Rebuild queue with current track first, then shuffled rest
        const currentTrack = queue[currentIndex];
        await TrackPlayer.removeUpcomingTracks();

        const remaining = this.shuffleOrder
          .filter(i => i !== currentIndex)
          .map(i => this.convertToPlayerTrack(this.originalQueue[i]));

        await TrackPlayer.add(remaining);
      }
    } else {
      // Restore original order
      const currentTrack = await TrackPlayer.getActiveTrack();
      if (currentTrack) {
        const originalIndex = this.originalQueue.findIndex(t => t.id === currentTrack.id);

        await TrackPlayer.removeUpcomingTracks();

        const remaining = this.originalQueue
          .slice(originalIndex + 1)
          .map(t => this.convertToPlayerTrack(t));

        await TrackPlayer.add(remaining);
      }
    }

    this.notifyListeners();
  }

  private generateShuffleOrder(length: number, currentIndex: number): number[] {
    const indices = Array.from({ length }, (_, i) => i);
    indices.splice(currentIndex, 1); // Remove current

    // Fisher-Yates shuffle
    for (let i = indices.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [indices[i], indices[j]] = [indices[j], indices[i]];
    }

    return [currentIndex, ...indices];
  }

  // ============================================================================
  // Like/Dislike
  // ============================================================================

  async toggleLike(): Promise<void> {
    const currentTrack = await TrackPlayer.getActiveTrack();
    if (!currentTrack) return;

    // Would call API to toggle like
    // For now, just update UI state
    const index = await TrackPlayer.getActiveTrackIndex();
    if (index !== null && this.currentState.currentTrack) {
      this.currentState.currentTrack.isLiked = !this.currentState.currentTrack.isLiked;
      this.notifyListeners();
    }
  }

  async dislike(): Promise<void> {
    const currentTrack = await TrackPlayer.getActiveTrack();
    if (!currentTrack) return;

    // Would call API to dislike/skip
    await this.skipToNext();
  }

  // ============================================================================
  // Audio Quality
  // ============================================================================

  async setAudioQuality(quality: 'auto' | 'low' | 'normal' | 'high' | 'lossless'): Promise<void> {
    // Would adjust streaming quality
    // This would require updating the track URLs in the queue
  }

  // ============================================================================
  // State Management
  // ============================================================================

  getPlaybackState(): PlaybackState {
    return { ...this.currentState };
  }

  onPlaybackStateChange(callback: PlaybackEventCallback): () => void {
    this.playbackListeners.add(callback);
    return () => this.playbackListeners.delete(callback);
  }

  private async updatePlaybackState(): Promise<void> {
    const [state, position, duration, buffered, queue, trackIndex, track] = await Promise.all([
      TrackPlayer.getPlaybackState(),
      TrackPlayer.getPosition(),
      TrackPlayer.getDuration(),
      TrackPlayer.getBufferedPosition(),
      TrackPlayer.getQueue(),
      TrackPlayer.getActiveTrackIndex(),
      TrackPlayer.getActiveTrack(),
    ]);

    this.currentState = {
      isPlaying: state.state === State.Playing,
      isPaused: state.state === State.Paused,
      isBuffering: state.state === State.Buffering || state.state === State.Loading,
      isStopped: state.state === State.Stopped || state.state === State.None,
      position,
      duration,
      buffered,
      currentTrack: track ? this.convertFromPlayerTrack(track) : null,
      queue: queue.map(t => this.convertFromPlayerTrack(t)),
      queuePosition: trackIndex ?? 0,
      repeatMode: this.currentState.repeatMode,
      shuffleEnabled: this.currentState.shuffleEnabled,
    };

    this.notifyListeners();
  }

  private notifyListeners(): void {
    for (const listener of this.playbackListeners) {
      listener(this.currentState);
    }
  }

  private getInitialState(): PlaybackState {
    return {
      isPlaying: false,
      isPaused: false,
      isBuffering: false,
      isStopped: true,
      position: 0,
      duration: 0,
      buffered: 0,
      currentTrack: null,
      queue: [],
      queuePosition: 0,
      repeatMode: 'off',
      shuffleEnabled: false,
    };
  }

  // ============================================================================
  // Helpers
  // ============================================================================

  private convertToPlayerTrack(track: AudioTrack): Track {
    return {
      id: track.id,
      url: track.url,
      title: track.title,
      artist: track.artist,
      album: track.album,
      artwork: track.artwork,
      duration: track.duration,
    };
  }

  private convertFromPlayerTrack(track: Track): AudioTrack {
    return {
      id: track.id || '',
      url: track.url,
      title: track.title || '',
      artist: track.artist || '',
      album: track.album,
      artwork: track.artwork,
      duration: track.duration || 0,
    };
  }

  private async reportTrackPlay(trackId: string): Promise<void> {
    // Would call analytics API
    try {
      await fetch('/api/analytics/track-play', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ trackId, timestamp: new Date().toISOString() }),
      });
    } catch (error) {
      console.error('Failed to report track play:', error);
    }
  }

  // ============================================================================
  // Sleep Timer
  // ============================================================================

  private sleepTimer: NodeJS.Timeout | null = null;
  private sleepEndTime: Date | null = null;

  async setSleepTimer(minutes: number): Promise<void> {
    this.clearSleepTimer();

    if (minutes <= 0) return;

    this.sleepEndTime = new Date(Date.now() + minutes * 60 * 1000);

    this.sleepTimer = setTimeout(async () => {
      await this.pause();
      this.sleepEndTime = null;
    }, minutes * 60 * 1000);
  }

  clearSleepTimer(): void {
    if (this.sleepTimer) {
      clearTimeout(this.sleepTimer);
      this.sleepTimer = null;
      this.sleepEndTime = null;
    }
  }

  getSleepTimerRemaining(): number | null {
    if (!this.sleepEndTime) return null;
    const remaining = this.sleepEndTime.getTime() - Date.now();
    return remaining > 0 ? Math.ceil(remaining / 1000 / 60) : null;
  }

  // ============================================================================
  // Equalizer (Platform Specific)
  // ============================================================================

  async setEqualizerPreset(preset: string): Promise<void> {
    // Would integrate with platform-specific EQ APIs
    // Android: AudioEffect API
    // iOS: AVAudioUnitEQ
  }

  async setCustomEqualizer(bands: number[]): Promise<void> {
    // Would set custom EQ bands
  }
}

export const backgroundAudio = new BackgroundAudioService();
export default backgroundAudio;
