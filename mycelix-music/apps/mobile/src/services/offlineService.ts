// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Offline Service
 *
 * Handles downloading, caching, and offline playback.
 * Manages storage, sync, and background downloads.
 */

import * as FileSystem from 'expo-file-system';
import * as MediaLibrary from 'expo-media-library';
import NetInfo, { NetInfoState } from '@react-native-community/netinfo';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';

import { Song } from '../store/playerStore';
import { api } from './api';

// ==================== Types ====================

export interface DownloadTask {
  id: string;
  songId: string;
  title: string;
  artist: string;
  coverArt: string;
  url: string;
  progress: number;
  status: 'pending' | 'downloading' | 'completed' | 'failed' | 'paused';
  error?: string;
  startedAt?: number;
  completedAt?: number;
  size?: number;
  resumable?: FileSystem.DownloadResumable;
}

export interface OfflineStats {
  totalDownloaded: number;
  totalSize: number;
  lastSync: number;
}

export interface OfflineState {
  // Connection
  isOnline: boolean;
  connectionType: string | null;

  // Downloads
  downloads: Map<string, DownloadTask>;
  downloadQueue: string[];
  activeDownloads: number;
  maxConcurrentDownloads: number;

  // Storage
  stats: OfflineStats;
  storageLimit: number; // in bytes
  downloadOnWifiOnly: boolean;

  // Actions
  startDownload: (song: Song) => Promise<void>;
  pauseDownload: (songId: string) => Promise<void>;
  resumeDownload: (songId: string) => Promise<void>;
  cancelDownload: (songId: string) => Promise<void>;
  deleteDownload: (songId: string) => Promise<void>;

  // Batch
  downloadPlaylist: (songs: Song[]) => Promise<void>;
  downloadAlbum: (songs: Song[]) => Promise<void>;

  // Settings
  setDownloadOnWifiOnly: (value: boolean) => void;
  setStorageLimit: (bytes: number) => void;
  setMaxConcurrentDownloads: (count: number) => void;

  // Sync
  syncDownloads: () => Promise<void>;
  cleanupOrphanedFiles: () => Promise<void>;

  // Internal
  updateConnectionStatus: (state: NetInfoState) => void;
  processQueue: () => Promise<void>;
}

// ==================== Constants ====================

const DOWNLOAD_DIR = `${FileSystem.documentDirectory}downloads/`;
const METADATA_KEY = 'offline_downloads_metadata';
const MAX_CONCURRENT = 3;
const DEFAULT_STORAGE_LIMIT = 5 * 1024 * 1024 * 1024; // 5GB

// ==================== Store ====================

export const useOfflineStore = create<OfflineState>()(
  persist(
    (set, get) => ({
      // Initial state
      isOnline: true,
      connectionType: null,
      downloads: new Map(),
      downloadQueue: [],
      activeDownloads: 0,
      maxConcurrentDownloads: MAX_CONCURRENT,
      stats: {
        totalDownloaded: 0,
        totalSize: 0,
        lastSync: 0,
      },
      storageLimit: DEFAULT_STORAGE_LIMIT,
      downloadOnWifiOnly: true,

      // ==================== Download Actions ====================

      startDownload: async (song) => {
        const state = get();

        // Check if already downloaded
        if (state.downloads.has(song.id)) {
          const existing = state.downloads.get(song.id)!;
          if (existing.status === 'completed') return;
          if (existing.status === 'downloading') return;
          if (existing.status === 'paused') {
            await state.resumeDownload(song.id);
            return;
          }
        }

        // Check connection
        if (state.downloadOnWifiOnly && state.connectionType !== 'wifi') {
          console.log('Download skipped: WiFi only mode');
          return;
        }

        // Check storage
        const availableStorage = await getAvailableStorage();
        if (availableStorage < 50 * 1024 * 1024) {
          // Less than 50MB
          console.log('Download skipped: Low storage');
          return;
        }

        // Create download task
        const task: DownloadTask = {
          id: `dl_${song.id}_${Date.now()}`,
          songId: song.id,
          title: song.title,
          artist: song.artist,
          coverArt: song.coverArt,
          url: song.audioUrl,
          progress: 0,
          status: 'pending',
        };

        // Add to queue
        set((state) => {
          const newDownloads = new Map(state.downloads);
          newDownloads.set(song.id, task);

          return {
            downloads: newDownloads,
            downloadQueue: [...state.downloadQueue, song.id],
          };
        });

        // Process queue
        await get().processQueue();
      },

      pauseDownload: async (songId) => {
        const state = get();
        const task = state.downloads.get(songId);

        if (!task || task.status !== 'downloading') return;

        if (task.resumable) {
          await task.resumable.pauseAsync();
        }

        set((state) => {
          const newDownloads = new Map(state.downloads);
          newDownloads.set(songId, { ...task, status: 'paused' });
          return {
            downloads: newDownloads,
            activeDownloads: Math.max(0, state.activeDownloads - 1),
          };
        });
      },

      resumeDownload: async (songId) => {
        const state = get();
        const task = state.downloads.get(songId);

        if (!task || task.status !== 'paused') return;

        set((state) => {
          const newDownloads = new Map(state.downloads);
          newDownloads.set(songId, { ...task, status: 'pending' });
          return { downloads: newDownloads };
        });

        await get().processQueue();
      },

      cancelDownload: async (songId) => {
        const state = get();
        const task = state.downloads.get(songId);

        if (!task) return;

        if (task.resumable) {
          await task.resumable.cancelAsync();
        }

        // Delete partial file
        const filePath = getFilePath(songId);
        try {
          await FileSystem.deleteAsync(filePath, { idempotent: true });
        } catch (e) {
          // Ignore
        }

        set((state) => {
          const newDownloads = new Map(state.downloads);
          newDownloads.delete(songId);

          return {
            downloads: newDownloads,
            downloadQueue: state.downloadQueue.filter((id) => id !== songId),
            activeDownloads:
              task.status === 'downloading'
                ? Math.max(0, state.activeDownloads - 1)
                : state.activeDownloads,
          };
        });
      },

      deleteDownload: async (songId) => {
        const state = get();
        const task = state.downloads.get(songId);

        if (!task) return;

        // Delete file
        const filePath = getFilePath(songId);
        try {
          await FileSystem.deleteAsync(filePath, { idempotent: true });
        } catch (e) {
          // Ignore
        }

        // Update stats
        const fileSize = task.size || 0;

        set((state) => {
          const newDownloads = new Map(state.downloads);
          newDownloads.delete(songId);

          return {
            downloads: newDownloads,
            stats: {
              ...state.stats,
              totalDownloaded: Math.max(0, state.stats.totalDownloaded - 1),
              totalSize: Math.max(0, state.stats.totalSize - fileSize),
            },
          };
        });
      },

      // ==================== Batch Downloads ====================

      downloadPlaylist: async (songs) => {
        const { startDownload } = get();
        for (const song of songs) {
          await startDownload(song);
        }
      },

      downloadAlbum: async (songs) => {
        const { startDownload } = get();
        for (const song of songs) {
          await startDownload(song);
        }
      },

      // ==================== Settings ====================

      setDownloadOnWifiOnly: (value) => {
        set({ downloadOnWifiOnly: value });
      },

      setStorageLimit: (bytes) => {
        set({ storageLimit: bytes });
      },

      setMaxConcurrentDownloads: (count) => {
        set({ maxConcurrentDownloads: count });
      },

      // ==================== Sync ====================

      syncDownloads: async () => {
        const state = get();

        // Verify all downloaded files exist
        const verified = new Map<string, DownloadTask>();

        for (const [songId, task] of state.downloads) {
          if (task.status === 'completed') {
            const filePath = getFilePath(songId);
            const info = await FileSystem.getInfoAsync(filePath);

            if (info.exists) {
              verified.set(songId, task);
            }
          } else {
            verified.set(songId, task);
          }
        }

        // Calculate stats
        let totalSize = 0;
        let totalDownloaded = 0;

        for (const task of verified.values()) {
          if (task.status === 'completed' && task.size) {
            totalSize += task.size;
            totalDownloaded++;
          }
        }

        set({
          downloads: verified,
          stats: {
            totalDownloaded,
            totalSize,
            lastSync: Date.now(),
          },
        });
      },

      cleanupOrphanedFiles: async () => {
        const state = get();

        try {
          const files = await FileSystem.readDirectoryAsync(DOWNLOAD_DIR);

          for (const file of files) {
            const songId = file.replace('.mp3', '');
            if (!state.downloads.has(songId)) {
              await FileSystem.deleteAsync(`${DOWNLOAD_DIR}${file}`, {
                idempotent: true,
              });
            }
          }
        } catch (e) {
          // Directory might not exist
        }
      },

      // ==================== Internal ====================

      updateConnectionStatus: (netInfo) => {
        set({
          isOnline: netInfo.isConnected ?? false,
          connectionType: netInfo.type,
        });
      },

      processQueue: async () => {
        const state = get();

        // Check if we can start more downloads
        if (state.activeDownloads >= state.maxConcurrentDownloads) {
          return;
        }

        if (state.downloadQueue.length === 0) {
          return;
        }

        // Check connection for WiFi-only mode
        if (state.downloadOnWifiOnly && state.connectionType !== 'wifi') {
          return;
        }

        // Find next pending download
        const nextSongId = state.downloadQueue.find((id) => {
          const task = state.downloads.get(id);
          return task && task.status === 'pending';
        });

        if (!nextSongId) return;

        const task = state.downloads.get(nextSongId)!;

        // Start download
        set((state) => ({
          activeDownloads: state.activeDownloads + 1,
          downloads: new Map(state.downloads).set(nextSongId, {
            ...task,
            status: 'downloading',
            startedAt: Date.now(),
          }),
        }));

        try {
          await ensureDownloadDir();

          const filePath = getFilePath(nextSongId);
          const callback = (downloadProgress: FileSystem.DownloadProgressData) => {
            const progress =
              downloadProgress.totalBytesWritten /
              downloadProgress.totalBytesExpectedToWrite;

            set((state) => {
              const newDownloads = new Map(state.downloads);
              const current = newDownloads.get(nextSongId);
              if (current) {
                newDownloads.set(nextSongId, {
                  ...current,
                  progress: progress * 100,
                  size: downloadProgress.totalBytesExpectedToWrite,
                });
              }
              return { downloads: newDownloads };
            });
          };

          const downloadResumable = FileSystem.createDownloadResumable(
            task.url,
            filePath,
            {},
            callback
          );

          // Store resumable reference
          set((state) => {
            const newDownloads = new Map(state.downloads);
            const current = newDownloads.get(nextSongId);
            if (current) {
              newDownloads.set(nextSongId, {
                ...current,
                resumable: downloadResumable,
              });
            }
            return { downloads: newDownloads };
          });

          const result = await downloadResumable.downloadAsync();

          if (result) {
            // Get file size
            const info = await FileSystem.getInfoAsync(result.uri);
            const fileSize = (info as any).size || 0;

            set((state) => {
              const newDownloads = new Map(state.downloads);
              newDownloads.set(nextSongId, {
                ...task,
                status: 'completed',
                progress: 100,
                completedAt: Date.now(),
                size: fileSize,
                resumable: undefined,
              });

              return {
                downloads: newDownloads,
                downloadQueue: state.downloadQueue.filter((id) => id !== nextSongId),
                activeDownloads: Math.max(0, state.activeDownloads - 1),
                stats: {
                  ...state.stats,
                  totalDownloaded: state.stats.totalDownloaded + 1,
                  totalSize: state.stats.totalSize + fileSize,
                },
              };
            });
          }
        } catch (error) {
          set((state) => {
            const newDownloads = new Map(state.downloads);
            newDownloads.set(nextSongId, {
              ...task,
              status: 'failed',
              error: (error as Error).message,
              resumable: undefined,
            });

            return {
              downloads: newDownloads,
              downloadQueue: state.downloadQueue.filter((id) => id !== nextSongId),
              activeDownloads: Math.max(0, state.activeDownloads - 1),
            };
          });
        }

        // Process next in queue
        await get().processQueue();
      },
    }),
    {
      name: 'mycelix-offline',
      storage: createJSONStorage(() => AsyncStorage),
      partialize: (state) => ({
        downloads: Array.from(state.downloads.entries()).map(([id, task]) => [
          id,
          { ...task, resumable: undefined },
        ]),
        stats: state.stats,
        downloadOnWifiOnly: state.downloadOnWifiOnly,
        storageLimit: state.storageLimit,
        maxConcurrentDownloads: state.maxConcurrentDownloads,
      }),
      merge: (persisted: any, current) => ({
        ...current,
        ...persisted,
        downloads: new Map(persisted?.downloads || []),
      }),
    }
  )
);

// ==================== Helpers ====================

async function ensureDownloadDir(): Promise<void> {
  const info = await FileSystem.getInfoAsync(DOWNLOAD_DIR);
  if (!info.exists) {
    await FileSystem.makeDirectoryAsync(DOWNLOAD_DIR, { intermediates: true });
  }
}

function getFilePath(songId: string): string {
  return `${DOWNLOAD_DIR}${songId}.mp3`;
}

async function getAvailableStorage(): Promise<number> {
  const info = await FileSystem.getFreeDiskStorageAsync();
  return info;
}

// ==================== Network Listener ====================

export function initNetworkListener(): () => void {
  const unsubscribe = NetInfo.addEventListener((state) => {
    useOfflineStore.getState().updateConnectionStatus(state);

    // Resume downloads when back online
    if (state.isConnected) {
      useOfflineStore.getState().processQueue();
    }
  });

  return unsubscribe;
}

// ==================== Utilities ====================

export function getLocalPath(songId: string): string | null {
  const task = useOfflineStore.getState().downloads.get(songId);
  if (task?.status === 'completed') {
    return getFilePath(songId);
  }
  return null;
}

export function isDownloaded(songId: string): boolean {
  const task = useOfflineStore.getState().downloads.get(songId);
  return task?.status === 'completed' ?? false;
}

export function getDownloadProgress(songId: string): number {
  const task = useOfflineStore.getState().downloads.get(songId);
  return task?.progress ?? 0;
}

export default useOfflineStore;
