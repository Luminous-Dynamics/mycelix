// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Batch Operations Service for Mycelix Mail
 *
 * High-performance bulk operations:
 * - Batch read/archive/delete
 * - Parallel processing with concurrency control
 * - Progress tracking
 * - Rollback support
 */

import type { ActionHash, AgentPubKey } from '@holochain/client';
import type { MycelixMailClient } from '../index';
import type { DecryptedEmail, EmailState, TrustCategory, TrustLevel } from '../types';

// ==================== TYPES ====================

export interface BatchOptions {
  /** Maximum concurrent operations */
  concurrency?: number;
  /** Continue on error */
  continueOnError?: boolean;
  /** Progress callback */
  onProgress?: (progress: BatchProgress) => void;
  /** Delay between batches (ms) */
  batchDelay?: number;
}

export interface BatchProgress {
  total: number;
  completed: number;
  failed: number;
  percent: number;
  currentItem?: string;
  estimatedTimeLeft?: number;
}

export interface BatchResult<T = void> {
  success: boolean;
  completed: number;
  failed: number;
  errors: BatchError[];
  results: T[];
  duration: number;
}

export interface BatchError {
  itemId: string;
  error: Error;
  index: number;
}

export type BatchOperation<T, R> = (item: T, index: number) => Promise<R>;

// ==================== BATCH SERVICE ====================

export class BatchService {
  private defaultOptions: BatchOptions = {
    concurrency: 5,
    continueOnError: true,
    batchDelay: 0,
  };

  constructor(private client: MycelixMailClient) {}

  // ==================== CORE BATCH PROCESSOR ====================

  /**
   * Process items in batches with concurrency control
   */
  async processBatch<T, R>(
    items: T[],
    operation: BatchOperation<T, R>,
    options?: BatchOptions
  ): Promise<BatchResult<R>> {
    const opts = { ...this.defaultOptions, ...options };
    const startTime = Date.now();
    const results: R[] = [];
    const errors: BatchError[] = [];
    let completed = 0;

    // Process in chunks
    const chunks = this.chunkArray(items, opts.concurrency!);

    for (const chunk of chunks) {
      const chunkPromises = chunk.map(async (item, chunkIndex) => {
        const index = completed + chunkIndex;
        try {
          const result = await operation(item, index);
          results.push(result);
          return { success: true, result };
        } catch (error) {
          const batchError: BatchError = {
            itemId: this.getItemId(item),
            error: error as Error,
            index,
          };
          errors.push(batchError);

          if (!opts.continueOnError) {
            throw error;
          }

          return { success: false, error };
        }
      });

      await Promise.all(chunkPromises);
      completed += chunk.length;

      // Report progress
      if (opts.onProgress) {
        const elapsed = Date.now() - startTime;
        const rate = completed / elapsed;
        const remaining = items.length - completed;
        const estimatedTimeLeft = remaining / rate;

        opts.onProgress({
          total: items.length,
          completed,
          failed: errors.length,
          percent: Math.round((completed / items.length) * 100),
          estimatedTimeLeft,
        });
      }

      // Delay between batches
      if (opts.batchDelay && completed < items.length) {
        await this.delay(opts.batchDelay);
      }
    }

    return {
      success: errors.length === 0,
      completed: completed - errors.length,
      failed: errors.length,
      errors,
      results,
      duration: Date.now() - startTime,
    };
  }

  // ==================== EMAIL BATCH OPERATIONS ====================

  /**
   * Mark multiple emails as read
   */
  async batchMarkAsRead(
    emailHashes: ActionHash[],
    options?: BatchOptions
  ): Promise<BatchResult> {
    return this.processBatch(
      emailHashes,
      async (hash) => {
        await this.client.messages.markAsRead(hash);
      },
      options
    );
  }

  /**
   * Mark multiple emails as unread
   */
  async batchMarkAsUnread(
    emailHashes: ActionHash[],
    options?: BatchOptions
  ): Promise<BatchResult> {
    return this.processBatch(
      emailHashes,
      async (hash) => {
        await this.client.messages.markAsUnread(hash);
      },
      options
    );
  }

  /**
   * Archive multiple emails
   */
  async batchArchive(
    emailHashes: ActionHash[],
    options?: BatchOptions
  ): Promise<BatchResult> {
    return this.processBatch(
      emailHashes,
      async (hash) => {
        await this.client.messages.archiveEmail(hash);
      },
      options
    );
  }

  /**
   * Delete multiple emails
   */
  async batchDelete(
    emailHashes: ActionHash[],
    options?: BatchOptions
  ): Promise<BatchResult> {
    return this.processBatch(
      emailHashes,
      async (hash) => {
        await this.client.messages.deleteEmail(hash);
      },
      options
    );
  }

  /**
   * Permanently delete multiple emails
   */
  async batchPermanentDelete(
    emailHashes: ActionHash[],
    options?: BatchOptions
  ): Promise<BatchResult> {
    return this.processBatch(
      emailHashes,
      async (hash) => {
        await this.client.messages.permanentlyDelete(hash);
      },
      options
    );
  }

  /**
   * Move multiple emails to folder
   */
  async batchMoveToFolder(
    emailHashes: ActionHash[],
    folderHash: ActionHash,
    options?: BatchOptions
  ): Promise<BatchResult> {
    return this.processBatch(
      emailHashes,
      async (hash) => {
        await this.client.messages.moveToFolder(hash, folderHash);
      },
      options
    );
  }

  /**
   * Add label to multiple emails
   */
  async batchAddLabel(
    emailHashes: ActionHash[],
    label: string,
    options?: BatchOptions
  ): Promise<BatchResult> {
    return this.processBatch(
      emailHashes,
      async (hash) => {
        await this.client.messages.addLabel(hash, label);
      },
      options
    );
  }

  /**
   * Remove label from multiple emails
   */
  async batchRemoveLabel(
    emailHashes: ActionHash[],
    label: string,
    options?: BatchOptions
  ): Promise<BatchResult> {
    return this.processBatch(
      emailHashes,
      async (hash) => {
        await this.client.messages.removeLabel(hash, label);
      },
      options
    );
  }

  /**
   * Star multiple emails
   */
  async batchStar(
    emailHashes: ActionHash[],
    options?: BatchOptions
  ): Promise<BatchResult> {
    return this.processBatch(
      emailHashes,
      async (hash) => {
        await this.client.messages.starEmail(hash);
      },
      options
    );
  }

  /**
   * Unstar multiple emails
   */
  async batchUnstar(
    emailHashes: ActionHash[],
    options?: BatchOptions
  ): Promise<BatchResult> {
    return this.processBatch(
      emailHashes,
      async (hash) => {
        await this.client.messages.unstarEmail(hash);
      },
      options
    );
  }

  // ==================== TRUST BATCH OPERATIONS ====================

  /**
   * Get trust scores for multiple agents
   */
  async batchGetTrustScores(
    agents: AgentPubKey[],
    category?: TrustCategory,
    options?: BatchOptions
  ): Promise<BatchResult<{ agent: AgentPubKey; score: number }>> {
    return this.processBatch(
      agents,
      async (agent) => {
        const score = await this.client.trust.getTrustScore({
          subject: agent,
          category,
          include_transitive: true,
        });
        return { agent, score: score.combined_score };
      },
      { ...options, concurrency: options?.concurrency ?? 10 }
    );
  }

  /**
   * Create attestations for multiple agents
   */
  async batchCreateAttestations(
    attestations: Array<{
      subject: AgentPubKey;
      category: TrustCategory;
      level: TrustLevel;
      confidence: number;
    }>,
    options?: BatchOptions
  ): Promise<BatchResult<ActionHash>> {
    return this.processBatch(
      attestations,
      async (attestation) => {
        return this.client.trust.createAttestation(attestation);
      },
      options
    );
  }

  // ==================== SYNC BATCH OPERATIONS ====================

  /**
   * Sync with multiple peers
   */
  async batchSyncWithPeers(
    peers: AgentPubKey[],
    options?: BatchOptions
  ): Promise<BatchResult<{ peer: AgentPubKey; synced: number }>> {
    return this.processBatch(
      peers,
      async (peer) => {
        const result = await this.client.sync.syncWithPeer(peer);
        return { peer, synced: result.operations_received };
      },
      { ...options, concurrency: options?.concurrency ?? 3 }
    );
  }

  // ==================== COMPOUND OPERATIONS ====================

  /**
   * Cleanup: Archive old read emails and delete old archived emails
   */
  async cleanupMailbox(
    archiveAfterDays: number = 30,
    deleteAfterDays: number = 90,
    options?: BatchOptions
  ): Promise<{
    archived: BatchResult;
    deleted: BatchResult;
  }> {
    const inbox = await this.client.messages.getInbox(1000);
    const now = Date.now();
    const archiveThreshold = now - archiveAfterDays * 24 * 60 * 60 * 1000;
    const deleteThreshold = now - deleteAfterDays * 24 * 60 * 60 * 1000;

    // Find emails to archive (read and old)
    const toArchive = inbox
      .filter((e) => e.state === 'Read' && new Date(e.timestamp).getTime() < archiveThreshold)
      .map((e) => e.hash);

    // Find emails to delete (archived and very old)
    const toDelete = inbox
      .filter((e) => e.state === 'Archived' && new Date(e.timestamp).getTime() < deleteThreshold)
      .map((e) => e.hash);

    const archived = await this.batchArchive(toArchive, options);
    const deleted = await this.batchDelete(toDelete, options);

    return { archived, deleted };
  }

  /**
   * Apply label to emails matching a filter
   */
  async labelByFilter(
    filter: (email: DecryptedEmail) => boolean,
    label: string,
    options?: BatchOptions
  ): Promise<BatchResult> {
    const inbox = await this.client.messages.getInbox(1000);
    const matching = inbox.filter(filter).map((e) => e.hash);

    return this.batchAddLabel(matching, label, options);
  }

  /**
   * Export emails (get all data for backup)
   */
  async exportEmails(
    emailHashes: ActionHash[],
    options?: BatchOptions
  ): Promise<BatchResult<DecryptedEmail>> {
    return this.processBatch(
      emailHashes,
      async (hash) => {
        const email = await this.client.messages.getEmail(hash);
        if (!email) throw new Error('Email not found');
        return email;
      },
      { ...options, concurrency: options?.concurrency ?? 20 }
    );
  }

  // ==================== UTILITIES ====================

  private chunkArray<T>(array: T[], size: number): T[][] {
    const chunks: T[][] = [];
    for (let i = 0; i < array.length; i += size) {
      chunks.push(array.slice(i, i + size));
    }
    return chunks;
  }

  private delay(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  private getItemId(item: unknown): string {
    if (typeof item === 'string') return item;
    if (item instanceof Uint8Array) {
      return Array.from(item).map((b) => b.toString(16).padStart(2, '0')).join('');
    }
    if (typeof item === 'object' && item !== null) {
      if ('hash' in item) return String((item as any).hash);
      if ('id' in item) return String((item as any).id);
    }
    return String(item);
  }
}

/**
 * Create batch service for a client
 */
export function createBatchService(client: MycelixMailClient): BatchService {
  return new BatchService(client);
}

export default BatchService;
