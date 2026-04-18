// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Offline Manager
 * Handles offline audio storage, sync, and playback for mobile app
 */

import AsyncStorage from '@react-native-async-storage/async-storage';
import * as FileSystem from 'expo-file-system';
import * as MediaLibrary from 'expo-media-library';
import NetInfo, { NetInfoState } from '@react-native-community/netinfo';
import { Platform } from 'react-native';

// Types
export interface DownloadedTrack {
  id: string;
  title: string;
  artist: string;
  album?: string;
  duration: number;
  coverArt: string;
  localAudioPath: string;
  localCoverPath?: string;
  quality: AudioQuality;
  fileSize: number;
  downloadedAt: Date;
  lastPlayedAt?: Date;
  expiresAt?: Date;
}

export interface DownloadProgress {
  trackId: string;
  bytesWritten: number;
  totalBytes: number;
  progress: number;
  status: 'queued' | 'downloading' | 'completed' | 'failed' | 'paused';
  error?: string;
}

export interface OfflineSettings {
  maxStorageGB: number;
  autoDownloadOnWifi: boolean;
  autoRemoveUnplayed: boolean;
  unplayedDaysThreshold: number;
  preferredQuality: AudioQuality;
  downloadOverCellular: boolean;
}

export type AudioQuality = 'LOW_128' | 'STANDARD_256' | 'HIGH_320' | 'LOSSLESS_FLAC';

type DownloadListener = (progress: DownloadProgress) => void;
type ConnectionListener = (isOnline: boolean) => void;

const STORAGE_KEYS = {
  DOWNLOADED_TRACKS: 'offline:downloaded_tracks',
  DOWNLOAD_QUEUE: 'offline:download_queue',
  SETTINGS: 'offline:settings',
  PENDING_SYNCS: 'offline:pending_syncs',
};

const DEFAULT_SETTINGS: OfflineSettings = {
  maxStorageGB: 5,
  autoDownloadOnWifi: true,
  autoRemoveUnplayed: false,
  unplayedDaysThreshold: 30,
  preferredQuality: 'HIGH_320',
  downloadOverCellular: false,
};

export class OfflineManager {
  private static instance: OfflineManager;
  private downloadedTracks: Map<string, DownloadedTrack> = new Map();
  private downloadQueue: string[] = [];
  private activeDownloads: Map<string, FileSystem.DownloadResumable> = new Map();
  private downloadProgress: Map<string, DownloadProgress> = new Map();
  private settings: OfflineSettings = DEFAULT_SETTINGS;
  private isOnline: boolean = true;
  private downloadListeners: Set<DownloadListener> = new Set();
  private connectionListeners: Set<ConnectionListener> = new Set();

  private constructor() {
    this.initialize();
  }

  static getInstance(): OfflineManager {
    if (!OfflineManager.instance) {
      OfflineManager.instance = new OfflineManager();
    }
    return OfflineManager.instance;
  }

  /**
   * Initialize offline manager
   */
  private async initialize(): Promise<void> {
    // Load persisted data
    await this.loadPersistedData();

    // Setup network listener
    NetInfo.addEventListener(this.handleConnectionChange.bind(this));

    // Check initial connection
    const state = await NetInfo.fetch();
    this.isOnline = !!state.isConnected;

    // Resume any interrupted downloads
    await this.resumeInterruptedDownloads();

    // Start background cleanup if needed
    this.startBackgroundCleanup();
  }

  /**
   * Load persisted data from storage
   */
  private async loadPersistedData(): Promise<void> {
    try {
      // Load downloaded tracks
      const tracksJson = await AsyncStorage.getItem(STORAGE_KEYS.DOWNLOADED_TRACKS);
      if (tracksJson) {
        const tracks: DownloadedTrack[] = JSON.parse(tracksJson);
        tracks.forEach(track => {
          this.downloadedTracks.set(track.id, {
            ...track,
            downloadedAt: new Date(track.downloadedAt),
            lastPlayedAt: track.lastPlayedAt ? new Date(track.lastPlayedAt) : undefined,
            expiresAt: track.expiresAt ? new Date(track.expiresAt) : undefined,
          });
        });
      }

      // Load download queue
      const queueJson = await AsyncStorage.getItem(STORAGE_KEYS.DOWNLOAD_QUEUE);
      if (queueJson) {
        this.downloadQueue = JSON.parse(queueJson);
      }

      // Load settings
      const settingsJson = await AsyncStorage.getItem(STORAGE_KEYS.SETTINGS);
      if (settingsJson) {
        this.settings = { ...DEFAULT_SETTINGS, ...JSON.parse(settingsJson) };
      }
    } catch (error) {
      console.error('Failed to load offline data:', error);
    }
  }

  /**
   * Persist data to storage
   */
  private async persistData(): Promise<void> {
    try {
      await AsyncStorage.setItem(
        STORAGE_KEYS.DOWNLOADED_TRACKS,
        JSON.stringify(Array.from(this.downloadedTracks.values()))
      );
      await AsyncStorage.setItem(
        STORAGE_KEYS.DOWNLOAD_QUEUE,
        JSON.stringify(this.downloadQueue)
      );
    } catch (error) {
      console.error('Failed to persist offline data:', error);
    }
  }

  /**
   * Download a track for offline playback
   */
  async downloadTrack(
    trackId: string,
    audioUrl: string,
    metadata: {
      title: string;
      artist: string;
      album?: string;
      duration: number;
      coverArt: string;
      quality: AudioQuality;
    }
  ): Promise<void> {
    // Check if already downloaded
    if (this.downloadedTracks.has(trackId)) {
      return;
    }

    // Check storage space
    const hasSpace = await this.checkStorageSpace();
    if (!hasSpace) {
      throw new Error('Not enough storage space');
    }

    // Check network conditions
    if (!this.isOnline) {
      this.addToQueue(trackId);
      throw new Error('No network connection. Added to download queue.');
    }

    const netState = await NetInfo.fetch();
    if (netState.type === 'cellular' && !this.settings.downloadOverCellular) {
      this.addToQueue(trackId);
      throw new Error('Cellular download disabled. Added to download queue.');
    }

    // Update progress
    this.updateProgress(trackId, {
      trackId,
      bytesWritten: 0,
      totalBytes: 0,
      progress: 0,
      status: 'downloading',
    });

    try {
      // Create directory for track
      const trackDir = `${FileSystem.documentDirectory}offline/${trackId}/`;
      await FileSystem.makeDirectoryAsync(trackDir, { intermediates: true });

      // Download audio file
      const audioPath = `${trackDir}audio.${this.getExtension(metadata.quality)}`;
      const downloadResumable = FileSystem.createDownloadResumable(
        audioUrl,
        audioPath,
        {},
        (downloadProgress) => {
          const progress = downloadProgress.totalBytesWritten / downloadProgress.totalBytesExpectedToWrite;
          this.updateProgress(trackId, {
            trackId,
            bytesWritten: downloadProgress.totalBytesWritten,
            totalBytes: downloadProgress.totalBytesExpectedToWrite,
            progress,
            status: 'downloading',
          });
        }
      );

      this.activeDownloads.set(trackId, downloadResumable);

      const result = await downloadResumable.downloadAsync();
      if (!result) {
        throw new Error('Download failed');
      }

      // Download cover art
      let localCoverPath: string | undefined;
      if (metadata.coverArt) {
        try {
          const coverPath = `${trackDir}cover.jpg`;
          await FileSystem.downloadAsync(metadata.coverArt, coverPath);
          localCoverPath = coverPath;
        } catch (coverError) {
          console.warn('Failed to download cover art:', coverError);
        }
      }

      // Get file size
      const fileInfo = await FileSystem.getInfoAsync(audioPath);
      const fileSize = (fileInfo as any).size || 0;

      // Save track info
      const downloadedTrack: DownloadedTrack = {
        id: trackId,
        title: metadata.title,
        artist: metadata.artist,
        album: metadata.album,
        duration: metadata.duration,
        coverArt: metadata.coverArt,
        localAudioPath: audioPath,
        localCoverPath,
        quality: metadata.quality,
        fileSize,
        downloadedAt: new Date(),
      };

      this.downloadedTracks.set(trackId, downloadedTrack);
      this.activeDownloads.delete(trackId);
      await this.persistData();

      this.updateProgress(trackId, {
        trackId,
        bytesWritten: fileSize,
        totalBytes: fileSize,
        progress: 1,
        status: 'completed',
      });
    } catch (error: any) {
      this.updateProgress(trackId, {
        trackId,
        bytesWritten: 0,
        totalBytes: 0,
        progress: 0,
        status: 'failed',
        error: error.message,
      });
      throw error;
    }
  }

  /**
   * Download multiple tracks (e.g., playlist or album)
   */
  async downloadTracks(
    tracks: Array<{
      id: string;
      audioUrl: string;
      title: string;
      artist: string;
      album?: string;
      duration: number;
      coverArt: string;
    }>
  ): Promise<void> {
    for (const track of tracks) {
      try {
        await this.downloadTrack(track.id, track.audioUrl, {
          ...track,
          quality: this.settings.preferredQuality,
        });
      } catch (error) {
        console.error(`Failed to download track ${track.id}:`, error);
      }
    }
  }

  /**
   * Remove downloaded track
   */
  async removeTrack(trackId: string): Promise<void> {
    const track = this.downloadedTracks.get(trackId);
    if (!track) return;

    try {
      // Delete files
      const trackDir = `${FileSystem.documentDirectory}offline/${trackId}/`;
      await FileSystem.deleteAsync(trackDir, { idempotent: true });

      // Remove from map
      this.downloadedTracks.delete(trackId);
      await this.persistData();
    } catch (error) {
      console.error('Failed to remove track:', error);
      throw error;
    }
  }

  /**
   * Pause a download
   */
  async pauseDownload(trackId: string): Promise<void> {
    const download = this.activeDownloads.get(trackId);
    if (download) {
      const snapshot = await download.pauseAsync();
      // Save snapshot for resume
      await AsyncStorage.setItem(
        `offline:download_snapshot:${trackId}`,
        JSON.stringify(snapshot)
      );

      this.updateProgress(trackId, {
        ...this.downloadProgress.get(trackId)!,
        status: 'paused',
      });
    }
  }

  /**
   * Resume a paused download
   */
  async resumeDownload(trackId: string): Promise<void> {
    const snapshotJson = await AsyncStorage.getItem(`offline:download_snapshot:${trackId}`);
    if (!snapshotJson) return;

    try {
      const snapshot = JSON.parse(snapshotJson);
      const download = FileSystem.createDownloadResumable(
        snapshot.url,
        snapshot.fileUri,
        snapshot.options,
        (progress) => {
          this.updateProgress(trackId, {
            trackId,
            bytesWritten: progress.totalBytesWritten,
            totalBytes: progress.totalBytesExpectedToWrite,
            progress: progress.totalBytesWritten / progress.totalBytesExpectedToWrite,
            status: 'downloading',
          });
        },
        snapshot.resumeData
      );

      this.activeDownloads.set(trackId, download);
      await download.resumeAsync();

      await AsyncStorage.removeItem(`offline:download_snapshot:${trackId}`);
    } catch (error) {
      console.error('Failed to resume download:', error);
    }
  }

  /**
   * Cancel a download
   */
  async cancelDownload(trackId: string): Promise<void> {
    const download = this.activeDownloads.get(trackId);
    if (download) {
      await download.cancelAsync();
      this.activeDownloads.delete(trackId);
    }

    // Remove any partial files
    const trackDir = `${FileSystem.documentDirectory}offline/${trackId}/`;
    await FileSystem.deleteAsync(trackDir, { idempotent: true });

    // Remove from queue
    this.downloadQueue = this.downloadQueue.filter(id => id !== trackId);
    await this.persistData();

    this.downloadProgress.delete(trackId);
  }

  /**
   * Get all downloaded tracks
   */
  getDownloadedTracks(): DownloadedTrack[] {
    return Array.from(this.downloadedTracks.values());
  }

  /**
   * Check if track is downloaded
   */
  isTrackDownloaded(trackId: string): boolean {
    return this.downloadedTracks.has(trackId);
  }

  /**
   * Get downloaded track info
   */
  getDownloadedTrack(trackId: string): DownloadedTrack | undefined {
    return this.downloadedTracks.get(trackId);
  }

  /**
   * Get local audio path for offline playback
   */
  getLocalAudioPath(trackId: string): string | null {
    const track = this.downloadedTracks.get(trackId);
    return track?.localAudioPath || null;
  }

  /**
   * Update last played timestamp
   */
  async updateLastPlayed(trackId: string): Promise<void> {
    const track = this.downloadedTracks.get(trackId);
    if (track) {
      track.lastPlayedAt = new Date();
      await this.persistData();
    }
  }

  /**
   * Get storage usage
   */
  async getStorageInfo(): Promise<{
    usedBytes: number;
    totalBytes: number;
    trackCount: number;
    availableBytes: number;
  }> {
    let usedBytes = 0;
    for (const track of this.downloadedTracks.values()) {
      usedBytes += track.fileSize;
    }

    const diskInfo = await FileSystem.getFreeDiskStorageAsync();

    return {
      usedBytes,
      totalBytes: this.settings.maxStorageGB * 1024 * 1024 * 1024,
      trackCount: this.downloadedTracks.size,
      availableBytes: diskInfo,
    };
  }

  /**
   * Update settings
   */
  async updateSettings(settings: Partial<OfflineSettings>): Promise<void> {
    this.settings = { ...this.settings, ...settings };
    await AsyncStorage.setItem(STORAGE_KEYS.SETTINGS, JSON.stringify(this.settings));
  }

  /**
   * Get current settings
   */
  getSettings(): OfflineSettings {
    return { ...this.settings };
  }

  /**
   * Add listener for download progress
   */
  addDownloadListener(listener: DownloadListener): () => void {
    this.downloadListeners.add(listener);
    return () => this.downloadListeners.delete(listener);
  }

  /**
   * Add listener for connection changes
   */
  addConnectionListener(listener: ConnectionListener): () => void {
    this.connectionListeners.add(listener);
    return () => this.connectionListeners.delete(listener);
  }

  /**
   * Check if online
   */
  isNetworkOnline(): boolean {
    return this.isOnline;
  }

  /**
   * Get download queue
   */
  getDownloadQueue(): string[] {
    return [...this.downloadQueue];
  }

  /**
   * Get download progress for a track
   */
  getDownloadProgress(trackId: string): DownloadProgress | undefined {
    return this.downloadProgress.get(trackId);
  }

  // Private helper methods

  private handleConnectionChange(state: NetInfoState): void {
    const wasOnline = this.isOnline;
    this.isOnline = !!state.isConnected;

    // Notify listeners
    this.connectionListeners.forEach(listener => listener(this.isOnline));

    // Resume downloads when back online
    if (!wasOnline && this.isOnline) {
      this.processDownloadQueue();
    }
  }

  private async checkStorageSpace(): Promise<boolean> {
    const { usedBytes, totalBytes } = await this.getStorageInfo();
    return usedBytes < totalBytes;
  }

  private addToQueue(trackId: string): void {
    if (!this.downloadQueue.includes(trackId)) {
      this.downloadQueue.push(trackId);
      this.persistData();
    }
  }

  private async processDownloadQueue(): Promise<void> {
    if (this.downloadQueue.length === 0) return;

    const netState = await NetInfo.fetch();
    if (netState.type === 'wifi' || this.settings.downloadOverCellular) {
      // Process queue
      const trackId = this.downloadQueue.shift();
      if (trackId) {
        // Would need track info to resume - this is simplified
        console.log(`Would resume download for ${trackId}`);
      }
    }
  }

  private async resumeInterruptedDownloads(): Promise<void> {
    // Resume any paused downloads
    for (const trackId of this.downloadQueue) {
      const snapshot = await AsyncStorage.getItem(`offline:download_snapshot:${trackId}`);
      if (snapshot && this.isOnline) {
        await this.resumeDownload(trackId);
      }
    }
  }

  private startBackgroundCleanup(): void {
    if (!this.settings.autoRemoveUnplayed) return;

    setInterval(async () => {
      const now = new Date();
      const thresholdMs = this.settings.unplayedDaysThreshold * 24 * 60 * 60 * 1000;

      for (const [trackId, track] of this.downloadedTracks) {
        const lastPlayed = track.lastPlayedAt || track.downloadedAt;
        if (now.getTime() - lastPlayed.getTime() > thresholdMs) {
          await this.removeTrack(trackId);
        }
      }
    }, 24 * 60 * 60 * 1000); // Check daily
  }

  private updateProgress(trackId: string, progress: DownloadProgress): void {
    this.downloadProgress.set(trackId, progress);
    this.downloadListeners.forEach(listener => listener(progress));
  }

  private getExtension(quality: AudioQuality): string {
    switch (quality) {
      case 'LOSSLESS_FLAC':
        return 'flac';
      default:
        return 'm4a';
    }
  }
}

export default OfflineManager;
