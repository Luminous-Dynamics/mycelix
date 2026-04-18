// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Offline Manager
 *
 * Manages offline downloads, storage, and sync for the mobile app.
 * Handles track downloads, playlist caching, and smart storage management.
 */

import AsyncStorage from '@react-native-async-storage/async-storage';
import RNFS from 'react-native-fs';
import NetInfo from '@react-native-community/netinfo';
import BackgroundFetch from 'react-native-background-fetch';

// ============================================================================
// Types
// ============================================================================

export interface DownloadedTrack {
  id: string;
  title: string;
  artist: string;
  albumArt: string;
  duration: number;
  filePath: string;
  fileSize: number;
  downloadedAt: Date;
  lastPlayedAt?: Date;
  playCount: number;
  quality: AudioQuality;
}

export interface DownloadQueue {
  items: QueueItem[];
  isProcessing: boolean;
  currentItem?: QueueItem;
  progress: number;
}

export interface QueueItem {
  trackId: string;
  priority: 'high' | 'normal' | 'low';
  addedAt: Date;
  retryCount: number;
}

export interface StorageStats {
  used: number;
  available: number;
  trackCount: number;
  oldestTrack?: Date;
  newestTrack?: Date;
}

export interface OfflineSettings {
  maxStorageGB: number;
  autoDownloadOnWifi: boolean;
  autoDownloadLiked: boolean;
  autoDownloadPlaylists: string[];
  downloadQuality: AudioQuality;
  smartCleanupEnabled: boolean;
  keepRecentDays: number;
}

export type AudioQuality = 'low' | 'normal' | 'high' | 'lossless';

type DownloadEventCallback = (event: DownloadEvent) => void;

interface DownloadEvent {
  type: 'started' | 'progress' | 'completed' | 'failed' | 'cancelled';
  trackId: string;
  progress?: number;
  error?: string;
}

// ============================================================================
// Constants
// ============================================================================

const STORAGE_KEYS = {
  DOWNLOADED_TRACKS: 'offline:tracks',
  DOWNLOAD_QUEUE: 'offline:queue',
  SETTINGS: 'offline:settings',
  LAST_SYNC: 'offline:lastSync',
};

const DOWNLOAD_DIR = `${RNFS.DocumentDirectoryPath}/offline-tracks`;

const QUALITY_BITRATES: Record<AudioQuality, number> = {
  low: 96,
  normal: 160,
  high: 320,
  lossless: 1411,
};

// ============================================================================
// Offline Manager
// ============================================================================

class OfflineManager {
  private downloadedTracks: Map<string, DownloadedTrack> = new Map();
  private queue: DownloadQueue = { items: [], isProcessing: false, progress: 0 };
  private settings: OfflineSettings;
  private listeners: Set<DownloadEventCallback> = new Set();
  private currentDownloadJob: { jobId: number; trackId: string } | null = null;
  private isOnline = true;

  constructor() {
    this.settings = this.getDefaultSettings();
    this.initialize();
  }

  // ============================================================================
  // Initialization
  // ============================================================================

  private async initialize(): Promise<void> {
    // Ensure download directory exists
    const exists = await RNFS.exists(DOWNLOAD_DIR);
    if (!exists) {
      await RNFS.mkdir(DOWNLOAD_DIR);
    }

    // Load persisted data
    await Promise.all([
      this.loadDownloadedTracks(),
      this.loadQueue(),
      this.loadSettings(),
    ]);

    // Monitor network state
    NetInfo.addEventListener(state => {
      const wasOffline = !this.isOnline;
      this.isOnline = state.isConnected ?? false;

      if (wasOffline && this.isOnline) {
        // Back online, resume downloads
        this.processQueue();
      }
    });

    // Setup background fetch for auto-downloads
    this.setupBackgroundFetch();

    // Resume any pending downloads
    if (this.queue.items.length > 0 && this.isOnline) {
      this.processQueue();
    }
  }

  private getDefaultSettings(): OfflineSettings {
    return {
      maxStorageGB: 2,
      autoDownloadOnWifi: true,
      autoDownloadLiked: true,
      autoDownloadPlaylists: [],
      downloadQuality: 'high',
      smartCleanupEnabled: true,
      keepRecentDays: 30,
    };
  }

  // ============================================================================
  // Download Management
  // ============================================================================

  async downloadTrack(
    trackId: string,
    metadata: {
      title: string;
      artist: string;
      albumArt: string;
      duration: number;
      audioUrl: string;
    },
    priority: 'high' | 'normal' | 'low' = 'normal'
  ): Promise<void> {
    // Check if already downloaded
    if (this.downloadedTracks.has(trackId)) {
      return;
    }

    // Check if already in queue
    if (this.queue.items.some(item => item.trackId === trackId)) {
      return;
    }

    // Check storage space
    const stats = await this.getStorageStats();
    const estimatedSize = this.estimateTrackSize(metadata.duration);

    if (stats.used + estimatedSize > this.settings.maxStorageGB * 1024 * 1024 * 1024) {
      if (this.settings.smartCleanupEnabled) {
        await this.smartCleanup(estimatedSize);
      } else {
        throw new Error('Insufficient storage space');
      }
    }

    // Add to queue
    this.queue.items.push({
      trackId,
      priority,
      addedAt: new Date(),
      retryCount: 0,
    });

    // Sort queue by priority
    this.queue.items.sort((a, b) => {
      const priorityOrder = { high: 0, normal: 1, low: 2 };
      return priorityOrder[a.priority] - priorityOrder[b.priority];
    });

    await this.saveQueue();

    // Store metadata for later
    await AsyncStorage.setItem(`track:meta:${trackId}`, JSON.stringify(metadata));

    // Start processing if not already
    if (!this.queue.isProcessing && this.isOnline) {
      this.processQueue();
    }
  }

  async downloadPlaylist(playlistId: string, tracks: Array<{
    id: string;
    title: string;
    artist: string;
    albumArt: string;
    duration: number;
    audioUrl: string;
  }>): Promise<void> {
    for (const track of tracks) {
      await this.downloadTrack(track.id, track, 'normal');
    }
  }

  async cancelDownload(trackId: string): Promise<void> {
    // Remove from queue
    this.queue.items = this.queue.items.filter(item => item.trackId !== trackId);
    await this.saveQueue();

    // Cancel current download if it's this track
    if (this.currentDownloadJob?.trackId === trackId) {
      await RNFS.stopDownload(this.currentDownloadJob.jobId);
      this.currentDownloadJob = null;
      this.emitEvent({ type: 'cancelled', trackId });
    }
  }

  async removeDownload(trackId: string): Promise<void> {
    const track = this.downloadedTracks.get(trackId);
    if (!track) return;

    // Delete file
    try {
      await RNFS.unlink(track.filePath);
    } catch (e) {
      // File may already be deleted
    }

    // Remove from map and persist
    this.downloadedTracks.delete(trackId);
    await this.saveDownloadedTracks();
  }

  // ============================================================================
  // Queue Processing
  // ============================================================================

  private async processQueue(): Promise<void> {
    if (this.queue.isProcessing || this.queue.items.length === 0) {
      return;
    }

    this.queue.isProcessing = true;

    while (this.queue.items.length > 0 && this.isOnline) {
      const item = this.queue.items[0];
      this.queue.currentItem = item;

      try {
        await this.downloadTrackFile(item.trackId);

        // Remove from queue on success
        this.queue.items.shift();
        await this.saveQueue();
      } catch (error) {
        item.retryCount++;

        if (item.retryCount >= 3) {
          // Max retries reached, remove from queue
          this.queue.items.shift();
          this.emitEvent({
            type: 'failed',
            trackId: item.trackId,
            error: (error as Error).message,
          });
        } else {
          // Move to end of queue for retry
          this.queue.items.shift();
          this.queue.items.push(item);
        }

        await this.saveQueue();
      }
    }

    this.queue.isProcessing = false;
    this.queue.currentItem = undefined;
  }

  private async downloadTrackFile(trackId: string): Promise<void> {
    // Get stored metadata
    const metaJson = await AsyncStorage.getItem(`track:meta:${trackId}`);
    if (!metaJson) {
      throw new Error('Track metadata not found');
    }

    const metadata = JSON.parse(metaJson);
    const quality = this.settings.downloadQuality;
    const audioUrl = `${metadata.audioUrl}?quality=${quality}`;
    const filePath = `${DOWNLOAD_DIR}/${trackId}.mp3`;

    this.emitEvent({ type: 'started', trackId });

    // Download with progress tracking
    const { jobId, promise } = RNFS.downloadFile({
      fromUrl: audioUrl,
      toFile: filePath,
      progress: (res) => {
        const progress = res.bytesWritten / res.contentLength;
        this.queue.progress = progress;
        this.emitEvent({ type: 'progress', trackId, progress });
      },
      progressDivider: 10,
    });

    this.currentDownloadJob = { jobId, trackId };

    const result = await promise;

    if (result.statusCode !== 200) {
      throw new Error(`Download failed with status ${result.statusCode}`);
    }

    // Get file info
    const fileInfo = await RNFS.stat(filePath);

    // Download album art
    const artPath = `${DOWNLOAD_DIR}/${trackId}_art.jpg`;
    try {
      await RNFS.downloadFile({
        fromUrl: metadata.albumArt,
        toFile: artPath,
      }).promise;
    } catch (e) {
      // Album art download failed, continue anyway
    }

    // Store downloaded track info
    const downloadedTrack: DownloadedTrack = {
      id: trackId,
      title: metadata.title,
      artist: metadata.artist,
      albumArt: artPath,
      duration: metadata.duration,
      filePath,
      fileSize: parseInt(fileInfo.size),
      downloadedAt: new Date(),
      playCount: 0,
      quality,
    };

    this.downloadedTracks.set(trackId, downloadedTrack);
    await this.saveDownloadedTracks();

    // Cleanup metadata
    await AsyncStorage.removeItem(`track:meta:${trackId}`);

    this.currentDownloadJob = null;
    this.emitEvent({ type: 'completed', trackId });
  }

  // ============================================================================
  // Playback
  // ============================================================================

  isTrackDownloaded(trackId: string): boolean {
    return this.downloadedTracks.has(trackId);
  }

  getDownloadedTrack(trackId: string): DownloadedTrack | undefined {
    return this.downloadedTracks.get(trackId);
  }

  getOfflineTrackUrl(trackId: string): string | null {
    const track = this.downloadedTracks.get(trackId);
    return track ? `file://${track.filePath}` : null;
  }

  getAllDownloadedTracks(): DownloadedTrack[] {
    return Array.from(this.downloadedTracks.values());
  }

  async markPlayed(trackId: string): Promise<void> {
    const track = this.downloadedTracks.get(trackId);
    if (track) {
      track.lastPlayedAt = new Date();
      track.playCount++;
      await this.saveDownloadedTracks();
    }
  }

  // ============================================================================
  // Storage Management
  // ============================================================================

  async getStorageStats(): Promise<StorageStats> {
    const tracks = Array.from(this.downloadedTracks.values());
    const used = tracks.reduce((sum, t) => sum + t.fileSize, 0);

    const fsInfo = await RNFS.getFSInfo();

    let oldestTrack: Date | undefined;
    let newestTrack: Date | undefined;

    for (const track of tracks) {
      const downloadedAt = new Date(track.downloadedAt);
      if (!oldestTrack || downloadedAt < oldestTrack) {
        oldestTrack = downloadedAt;
      }
      if (!newestTrack || downloadedAt > newestTrack) {
        newestTrack = downloadedAt;
      }
    }

    return {
      used,
      available: fsInfo.freeSpace,
      trackCount: tracks.length,
      oldestTrack,
      newestTrack,
    };
  }

  private async smartCleanup(neededBytes: number): Promise<void> {
    const tracks = Array.from(this.downloadedTracks.values());

    // Sort by last played (oldest first) and play count (lowest first)
    tracks.sort((a, b) => {
      const aLastPlayed = a.lastPlayedAt?.getTime() || 0;
      const bLastPlayed = b.lastPlayedAt?.getTime() || 0;

      if (aLastPlayed !== bLastPlayed) {
        return aLastPlayed - bLastPlayed;
      }

      return a.playCount - b.playCount;
    });

    let freedBytes = 0;
    const keepAfter = Date.now() - this.settings.keepRecentDays * 24 * 60 * 60 * 1000;

    for (const track of tracks) {
      if (freedBytes >= neededBytes) break;

      const downloadedAt = new Date(track.downloadedAt).getTime();
      if (downloadedAt > keepAfter) continue; // Keep recent downloads

      await this.removeDownload(track.id);
      freedBytes += track.fileSize;
    }

    if (freedBytes < neededBytes) {
      throw new Error('Unable to free enough space');
    }
  }

  async clearAllDownloads(): Promise<void> {
    for (const trackId of this.downloadedTracks.keys()) {
      await this.removeDownload(trackId);
    }
  }

  // ============================================================================
  // Background Sync
  // ============================================================================

  private setupBackgroundFetch(): void {
    BackgroundFetch.configure(
      {
        minimumFetchInterval: 15, // minutes
        stopOnTerminate: false,
        startOnBoot: true,
        enableHeadless: true,
      },
      async (taskId) => {
        await this.backgroundSync();
        BackgroundFetch.finish(taskId);
      },
      (taskId) => {
        BackgroundFetch.finish(taskId);
      }
    );
  }

  private async backgroundSync(): Promise<void> {
    const netInfo = await NetInfo.fetch();

    if (!netInfo.isConnected) return;
    if (!netInfo.isWifiEnabled && this.settings.autoDownloadOnWifi) return;

    // Auto-download liked tracks
    if (this.settings.autoDownloadLiked) {
      // Would fetch liked tracks from API and queue downloads
    }

    // Auto-download playlist tracks
    for (const playlistId of this.settings.autoDownloadPlaylists) {
      // Would fetch playlist tracks from API and queue downloads
    }

    // Process queue
    await this.processQueue();
  }

  // ============================================================================
  // Settings
  // ============================================================================

  async updateSettings(updates: Partial<OfflineSettings>): Promise<void> {
    this.settings = { ...this.settings, ...updates };
    await this.saveSettings();
  }

  getSettings(): OfflineSettings {
    return { ...this.settings };
  }

  // ============================================================================
  // Events
  // ============================================================================

  onDownloadEvent(callback: DownloadEventCallback): () => void {
    this.listeners.add(callback);
    return () => this.listeners.delete(callback);
  }

  private emitEvent(event: DownloadEvent): void {
    for (const listener of this.listeners) {
      listener(event);
    }
  }

  // ============================================================================
  // Persistence
  // ============================================================================

  private async loadDownloadedTracks(): Promise<void> {
    const json = await AsyncStorage.getItem(STORAGE_KEYS.DOWNLOADED_TRACKS);
    if (json) {
      const tracks = JSON.parse(json) as DownloadedTrack[];
      for (const track of tracks) {
        // Verify file still exists
        const exists = await RNFS.exists(track.filePath);
        if (exists) {
          this.downloadedTracks.set(track.id, track);
        }
      }
    }
  }

  private async saveDownloadedTracks(): Promise<void> {
    const tracks = Array.from(this.downloadedTracks.values());
    await AsyncStorage.setItem(STORAGE_KEYS.DOWNLOADED_TRACKS, JSON.stringify(tracks));
  }

  private async loadQueue(): Promise<void> {
    const json = await AsyncStorage.getItem(STORAGE_KEYS.DOWNLOAD_QUEUE);
    if (json) {
      this.queue = JSON.parse(json);
      this.queue.isProcessing = false; // Reset on app restart
    }
  }

  private async saveQueue(): Promise<void> {
    await AsyncStorage.setItem(STORAGE_KEYS.DOWNLOAD_QUEUE, JSON.stringify(this.queue));
  }

  private async loadSettings(): Promise<void> {
    const json = await AsyncStorage.getItem(STORAGE_KEYS.SETTINGS);
    if (json) {
      this.settings = { ...this.getDefaultSettings(), ...JSON.parse(json) };
    }
  }

  private async saveSettings(): Promise<void> {
    await AsyncStorage.setItem(STORAGE_KEYS.SETTINGS, JSON.stringify(this.settings));
  }

  // ============================================================================
  // Helpers
  // ============================================================================

  private estimateTrackSize(durationSeconds: number): number {
    const bitrate = QUALITY_BITRATES[this.settings.downloadQuality];
    return (bitrate * 1000 * durationSeconds) / 8;
  }

  getQueueStatus(): {
    pending: number;
    currentTrack?: string;
    progress: number;
  } {
    return {
      pending: this.queue.items.length,
      currentTrack: this.queue.currentItem?.trackId,
      progress: this.queue.progress,
    };
  }
}

export const offlineManager = new OfflineManager();
export default offlineManager;
