// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Thread/Conversation Service for Mycelix Mail
 *
 * Smart email threading:
 * - Message-ID/References-based threading
 * - Subject-based grouping
 * - Conversation view
 * - Thread actions (mute, archive all)
 * - Participant tracking
 * - Read/unread roll-up
 */

import type { ActionHash, AgentPubKey } from '@holochain/client';
import type { MycelixMailClient } from '../index';
import type { DecryptedEmail } from '../types';

// ==================== TYPES ====================

export interface EmailThread {
  id: string;
  hash?: ActionHash;
  subject: string;
  normalizedSubject: string;
  participants: ThreadParticipant[];
  messages: ThreadMessage[];
  messageCount: number;
  unreadCount: number;
  hasAttachments: boolean;
  isStarred: boolean;
  isMuted: boolean;
  isArchived: boolean;
  labels: string[];
  folders: ActionHash[];
  firstMessageDate: number;
  lastMessageDate: number;
  lastActivity: number;
  snippet: string;
}

export interface ThreadParticipant {
  email: string;
  displayName?: string;
  agentPubKey?: AgentPubKey;
  messageCount: number;
  isOriginator: boolean;
  lastSeen: number;
}

export interface ThreadMessage {
  hash: ActionHash;
  messageId: string;
  inReplyTo?: string;
  references: string[];
  sender: string;
  timestamp: number;
  isRead: boolean;
  isStarred: boolean;
  hasAttachments: boolean;
  snippet: string;
  depth: number; // For nested view
}

export interface ThreadFilter {
  query?: string;
  unreadOnly?: boolean;
  starredOnly?: boolean;
  hasAttachments?: boolean;
  participants?: string[];
  labels?: string[];
  folders?: ActionHash[];
  dateRange?: { start: number; end: number };
  excludeMuted?: boolean;
}

export interface ThreadSortOptions {
  field: 'lastActivity' | 'firstMessage' | 'messageCount' | 'unreadCount';
  direction: 'asc' | 'desc';
}

export interface ThreadStats {
  totalThreads: number;
  totalMessages: number;
  unreadThreads: number;
  mutedThreads: number;
  averageMessagesPerThread: number;
  topParticipants: Array<{ email: string; threadCount: number }>;
}

// ==================== THREAD SERVICE ====================

export class ThreadService {
  private threads: Map<string, EmailThread> = new Map();
  private messageIdToThread: Map<string, string> = new Map();
  private subjectToThreads: Map<string, Set<string>> = new Map();

  constructor(private client?: MycelixMailClient) {}

  // ==================== THREADING ====================

  /**
   * Build threads from emails
   */
  async buildThreads(emails: DecryptedEmail[]): Promise<EmailThread[]> {
    // Sort by date for proper threading
    const sorted = [...emails].sort(
      (a, b) => new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime()
    );

    for (const email of sorted) {
      await this.addEmailToThread(email);
    }

    return Array.from(this.threads.values());
  }

  /**
   * Add email to appropriate thread
   */
  async addEmailToThread(email: DecryptedEmail): Promise<EmailThread> {
    // Try to find existing thread by References/In-Reply-To
    let threadId = this.findThreadByReferences(email);

    // Try to find by subject if no reference match
    if (!threadId) {
      threadId = this.findThreadBySubject(email);
    }

    let thread: EmailThread;

    if (threadId) {
      thread = this.threads.get(threadId)!;
      this.addMessageToThread(thread, email);
    } else {
      thread = this.createThread(email);
      this.threads.set(thread.id, thread);
    }

    // Index the message
    if (email.message_id) {
      this.messageIdToThread.set(email.message_id, thread.id);
    }

    // Index by subject
    const normalizedSubject = this.normalizeSubject(email.subject);
    if (!this.subjectToThreads.has(normalizedSubject)) {
      this.subjectToThreads.set(normalizedSubject, new Set());
    }
    this.subjectToThreads.get(normalizedSubject)!.add(thread.id);

    return thread;
  }

  /**
   * Find thread by References or In-Reply-To
   */
  private findThreadByReferences(email: DecryptedEmail): string | null {
    // Check In-Reply-To
    if (email.in_reply_to) {
      const threadId = this.messageIdToThread.get(email.in_reply_to);
      if (threadId) return threadId;
    }

    // Check References (in reverse order - most recent first)
    if (email.references) {
      for (let i = email.references.length - 1; i >= 0; i--) {
        const threadId = this.messageIdToThread.get(email.references[i]);
        if (threadId) return threadId;
      }
    }

    return null;
  }

  /**
   * Find thread by subject
   */
  private findThreadBySubject(email: DecryptedEmail): string | null {
    const normalizedSubject = this.normalizeSubject(email.subject);
    const threadIds = this.subjectToThreads.get(normalizedSubject);

    if (!threadIds || threadIds.size === 0) return null;

    // Find the best matching thread
    const emailTime = new Date(email.timestamp).getTime();
    const sender = email.sender;

    for (const threadId of threadIds) {
      const thread = this.threads.get(threadId);
      if (!thread) continue;

      // Check if sender is a participant or recipient
      const isParticipant = thread.participants.some((p) => p.email === sender);
      const isRecipient = thread.messages.some((m) => {
        // Would need full email to check recipients
        return true;
      });

      // Check time proximity (within 30 days)
      const timeDiff = emailTime - thread.lastMessageDate;
      if (timeDiff > 0 && timeDiff < 30 * 24 * 60 * 60 * 1000) {
        if (isParticipant || isRecipient) {
          return threadId;
        }
      }
    }

    return null;
  }

  /**
   * Create new thread from email
   */
  private createThread(email: DecryptedEmail): EmailThread {
    const timestamp = new Date(email.timestamp).getTime();
    const normalizedSubject = this.normalizeSubject(email.subject);

    const thread: EmailThread = {
      id: this.generateId(),
      subject: email.subject,
      normalizedSubject,
      participants: [
        {
          email: email.sender,
          messageCount: 1,
          isOriginator: true,
          lastSeen: timestamp,
        },
      ],
      messages: [
        {
          hash: email.hash,
          messageId: email.message_id ?? '',
          inReplyTo: email.in_reply_to,
          references: email.references ?? [],
          sender: email.sender,
          timestamp,
          isRead: email.state !== 'Unread',
          isStarred: email.is_starred ?? false,
          hasAttachments: (email.attachments?.length ?? 0) > 0,
          snippet: this.createSnippet(email.body),
          depth: 0,
        },
      ],
      messageCount: 1,
      unreadCount: email.state === 'Unread' ? 1 : 0,
      hasAttachments: (email.attachments?.length ?? 0) > 0,
      isStarred: email.is_starred ?? false,
      isMuted: false,
      isArchived: email.state === 'Archived',
      labels: email.labels ?? [],
      folders: email.folder ? [email.folder] : [],
      firstMessageDate: timestamp,
      lastMessageDate: timestamp,
      lastActivity: timestamp,
      snippet: this.createSnippet(email.body),
    };

    // Add recipients as participants
    for (const recipient of email.recipients.to) {
      if (recipient !== email.sender) {
        thread.participants.push({
          email: recipient,
          messageCount: 0,
          isOriginator: false,
          lastSeen: timestamp,
        });
      }
    }

    return thread;
  }

  /**
   * Add message to existing thread
   */
  private addMessageToThread(thread: EmailThread, email: DecryptedEmail): void {
    const timestamp = new Date(email.timestamp).getTime();

    // Calculate depth based on references
    let depth = 0;
    if (email.in_reply_to) {
      const parentMessage = thread.messages.find(
        (m) => m.messageId === email.in_reply_to
      );
      if (parentMessage) {
        depth = parentMessage.depth + 1;
      }
    }

    // Add message
    thread.messages.push({
      hash: email.hash,
      messageId: email.message_id ?? '',
      inReplyTo: email.in_reply_to,
      references: email.references ?? [],
      sender: email.sender,
      timestamp,
      isRead: email.state !== 'Unread',
      isStarred: email.is_starred ?? false,
      hasAttachments: (email.attachments?.length ?? 0) > 0,
      snippet: this.createSnippet(email.body),
      depth,
    });

    // Sort messages by timestamp
    thread.messages.sort((a, b) => a.timestamp - b.timestamp);

    // Update thread metadata
    thread.messageCount++;
    if (email.state === 'Unread') {
      thread.unreadCount++;
    }
    if (email.attachments?.length) {
      thread.hasAttachments = true;
    }
    if (email.is_starred) {
      thread.isStarred = true;
    }

    thread.lastMessageDate = Math.max(thread.lastMessageDate, timestamp);
    thread.lastActivity = Math.max(thread.lastActivity, timestamp);
    thread.snippet = this.createSnippet(email.body);

    // Update/add participant
    const existingParticipant = thread.participants.find(
      (p) => p.email === email.sender
    );
    if (existingParticipant) {
      existingParticipant.messageCount++;
      existingParticipant.lastSeen = Math.max(existingParticipant.lastSeen, timestamp);
    } else {
      thread.participants.push({
        email: email.sender,
        messageCount: 1,
        isOriginator: false,
        lastSeen: timestamp,
      });
    }

    // Merge labels
    for (const label of email.labels ?? []) {
      if (!thread.labels.includes(label)) {
        thread.labels.push(label);
      }
    }
  }

  /**
   * Normalize subject for comparison
   */
  private normalizeSubject(subject: string): string {
    return subject
      .toLowerCase()
      .replace(/^(re:|fwd?:|fw:|\[.*?\])\s*/gi, '')
      .replace(/\s+/g, ' ')
      .trim();
  }

  /**
   * Create snippet from body
   */
  private createSnippet(body: string, maxLength = 150): string {
    return body
      .replace(/\s+/g, ' ')
      .trim()
      .substring(0, maxLength)
      .trim() + (body.length > maxLength ? '...' : '');
  }

  // ==================== THREAD OPERATIONS ====================

  /**
   * Get thread by ID
   */
  getThread(threadId: string): EmailThread | null {
    return this.threads.get(threadId) ?? null;
  }

  /**
   * Get all threads
   */
  getAllThreads(
    sort: ThreadSortOptions = { field: 'lastActivity', direction: 'desc' }
  ): EmailThread[] {
    const threads = Array.from(this.threads.values());
    return this.sortThreads(threads, sort);
  }

  /**
   * Filter threads
   */
  filterThreads(
    filter: ThreadFilter,
    sort?: ThreadSortOptions
  ): EmailThread[] {
    let results = Array.from(this.threads.values());

    if (filter.query) {
      const query = filter.query.toLowerCase();
      results = results.filter(
        (t) =>
          t.subject.toLowerCase().includes(query) ||
          t.snippet.toLowerCase().includes(query) ||
          t.participants.some((p) => p.email.toLowerCase().includes(query))
      );
    }

    if (filter.unreadOnly) {
      results = results.filter((t) => t.unreadCount > 0);
    }

    if (filter.starredOnly) {
      results = results.filter((t) => t.isStarred);
    }

    if (filter.hasAttachments) {
      results = results.filter((t) => t.hasAttachments);
    }

    if (filter.excludeMuted) {
      results = results.filter((t) => !t.isMuted);
    }

    if (filter.participants?.length) {
      results = results.filter((t) =>
        t.participants.some((p) => filter.participants!.includes(p.email))
      );
    }

    if (filter.labels?.length) {
      results = results.filter((t) =>
        t.labels.some((l) => filter.labels!.includes(l))
      );
    }

    if (filter.dateRange) {
      results = results.filter(
        (t) =>
          t.lastMessageDate >= filter.dateRange!.start &&
          t.lastMessageDate <= filter.dateRange!.end
      );
    }

    return sort ? this.sortThreads(results, sort) : results;
  }

  /**
   * Sort threads
   */
  private sortThreads(
    threads: EmailThread[],
    sort: ThreadSortOptions
  ): EmailThread[] {
    const multiplier = sort.direction === 'asc' ? 1 : -1;

    return threads.sort((a, b) => {
      switch (sort.field) {
        case 'lastActivity':
          return (a.lastActivity - b.lastActivity) * multiplier;
        case 'firstMessage':
          return (a.firstMessageDate - b.firstMessageDate) * multiplier;
        case 'messageCount':
          return (a.messageCount - b.messageCount) * multiplier;
        case 'unreadCount':
          return (a.unreadCount - b.unreadCount) * multiplier;
        default:
          return 0;
      }
    });
  }

  // ==================== THREAD ACTIONS ====================

  /**
   * Mark entire thread as read
   */
  async markThreadAsRead(threadId: string): Promise<boolean> {
    const thread = this.threads.get(threadId);
    if (!thread) return false;

    if (this.client) {
      for (const message of thread.messages) {
        if (!message.isRead) {
          await this.client.messages.markAsRead(message.hash);
          message.isRead = true;
        }
      }
    }

    thread.unreadCount = 0;
    return true;
  }

  /**
   * Mark entire thread as unread
   */
  async markThreadAsUnread(threadId: string): Promise<boolean> {
    const thread = this.threads.get(threadId);
    if (!thread) return false;

    // Mark only the last message as unread
    const lastMessage = thread.messages[thread.messages.length - 1];
    if (lastMessage && this.client) {
      await this.client.messages.markAsUnread(lastMessage.hash);
      lastMessage.isRead = false;
      thread.unreadCount = 1;
    }

    return true;
  }

  /**
   * Archive entire thread
   */
  async archiveThread(threadId: string): Promise<boolean> {
    const thread = this.threads.get(threadId);
    if (!thread) return false;

    if (this.client) {
      for (const message of thread.messages) {
        await this.client.messages.archiveEmail(message.hash);
      }
    }

    thread.isArchived = true;
    return true;
  }

  /**
   * Unarchive thread (move back to inbox)
   */
  async unarchiveThread(threadId: string): Promise<boolean> {
    const thread = this.threads.get(threadId);
    if (!thread) return false;

    if (this.client) {
      for (const message of thread.messages) {
        await this.client.messages.unarchiveEmail(message.hash);
      }
    }

    thread.isArchived = false;
    return true;
  }

  /**
   * Delete entire thread
   */
  async deleteThread(threadId: string): Promise<boolean> {
    const thread = this.threads.get(threadId);
    if (!thread) return false;

    if (this.client) {
      for (const message of thread.messages) {
        await this.client.messages.deleteEmail(message.hash);
      }
    }

    // Clean up indexes
    for (const message of thread.messages) {
      this.messageIdToThread.delete(message.messageId);
    }
    this.subjectToThreads.get(thread.normalizedSubject)?.delete(threadId);
    this.threads.delete(threadId);

    return true;
  }

  /**
   * Star/unstar thread
   */
  async toggleThreadStar(threadId: string): Promise<boolean> {
    const thread = this.threads.get(threadId);
    if (!thread) return false;

    thread.isStarred = !thread.isStarred;

    // Star/unstar the first message
    if (this.client && thread.messages.length > 0) {
      const firstMessage = thread.messages[0];
      if (thread.isStarred) {
        await this.client.messages.starEmail(firstMessage.hash);
      } else {
        await this.client.messages.unstarEmail(firstMessage.hash);
      }
    }

    return true;
  }

  /**
   * Mute thread (no notifications)
   */
  async muteThread(threadId: string): Promise<boolean> {
    const thread = this.threads.get(threadId);
    if (!thread) return false;

    thread.isMuted = true;

    if (this.client) {
      await this.client.callZome('threads', 'mute_thread', threadId);
    }

    return true;
  }

  /**
   * Unmute thread
   */
  async unmuteThread(threadId: string): Promise<boolean> {
    const thread = this.threads.get(threadId);
    if (!thread) return false;

    thread.isMuted = false;

    if (this.client) {
      await this.client.callZome('threads', 'unmute_thread', threadId);
    }

    return true;
  }

  /**
   * Add label to all messages in thread
   */
  async addLabelToThread(threadId: string, label: string): Promise<boolean> {
    const thread = this.threads.get(threadId);
    if (!thread) return false;

    if (this.client) {
      for (const message of thread.messages) {
        await this.client.messages.addLabel(message.hash, label);
      }
    }

    if (!thread.labels.includes(label)) {
      thread.labels.push(label);
    }

    return true;
  }

  /**
   * Remove label from all messages in thread
   */
  async removeLabelFromThread(threadId: string, label: string): Promise<boolean> {
    const thread = this.threads.get(threadId);
    if (!thread) return false;

    if (this.client) {
      for (const message of thread.messages) {
        await this.client.messages.removeLabel(message.hash, label);
      }
    }

    thread.labels = thread.labels.filter((l) => l !== label);
    return true;
  }

  // ==================== CONVERSATION VIEW ====================

  /**
   * Get messages in conversation tree format
   */
  getConversationTree(threadId: string): ThreadMessage[] {
    const thread = this.threads.get(threadId);
    if (!thread) return [];

    // Build tree structure
    const messageMap = new Map<string, ThreadMessage>();
    const roots: ThreadMessage[] = [];

    for (const message of thread.messages) {
      messageMap.set(message.messageId, message);
    }

    for (const message of thread.messages) {
      if (!message.inReplyTo || !messageMap.has(message.inReplyTo)) {
        roots.push(message);
      }
    }

    // Sort by timestamp within each level
    return this.flattenTree(roots, messageMap);
  }

  /**
   * Flatten tree to list with depths
   */
  private flattenTree(
    roots: ThreadMessage[],
    messageMap: Map<string, ThreadMessage>
  ): ThreadMessage[] {
    const result: ThreadMessage[] = [];

    const traverse = (messages: ThreadMessage[], depth: number) => {
      const sorted = messages.sort((a, b) => a.timestamp - b.timestamp);

      for (const message of sorted) {
        message.depth = depth;
        result.push(message);

        // Find children
        const children = Array.from(messageMap.values()).filter(
          (m) => m.inReplyTo === message.messageId
        );

        if (children.length > 0) {
          traverse(children, depth + 1);
        }
      }
    };

    traverse(roots, 0);
    return result;
  }

  /**
   * Get quick reply context
   */
  getReplyContext(threadId: string): {
    lastMessage: ThreadMessage | null;
    participants: string[];
    subject: string;
  } {
    const thread = this.threads.get(threadId);
    if (!thread) {
      return { lastMessage: null, participants: [], subject: '' };
    }

    return {
      lastMessage: thread.messages[thread.messages.length - 1] ?? null,
      participants: thread.participants.map((p) => p.email),
      subject: thread.subject.startsWith('Re:')
        ? thread.subject
        : `Re: ${thread.subject}`,
    };
  }

  // ==================== STATISTICS ====================

  /**
   * Get thread statistics
   */
  getStats(): ThreadStats {
    const threads = Array.from(this.threads.values());
    const participantCounts = new Map<string, number>();

    let totalMessages = 0;
    let unreadThreads = 0;
    let mutedThreads = 0;

    for (const thread of threads) {
      totalMessages += thread.messageCount;

      if (thread.unreadCount > 0) unreadThreads++;
      if (thread.isMuted) mutedThreads++;

      for (const participant of thread.participants) {
        const current = participantCounts.get(participant.email) ?? 0;
        participantCounts.set(participant.email, current + 1);
      }
    }

    const topParticipants = Array.from(participantCounts.entries())
      .map(([email, threadCount]) => ({ email, threadCount }))
      .sort((a, b) => b.threadCount - a.threadCount)
      .slice(0, 10);

    return {
      totalThreads: threads.length,
      totalMessages,
      unreadThreads,
      mutedThreads,
      averageMessagesPerThread: threads.length > 0
        ? totalMessages / threads.length
        : 0,
      topParticipants,
    };
  }

  // ==================== UTILITIES ====================

  private generateId(): string {
    return `thread_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * Clear all threads (for refresh)
   */
  clear(): void {
    this.threads.clear();
    this.messageIdToThread.clear();
    this.subjectToThreads.clear();
  }

  /**
   * Get thread count
   */
  getThreadCount(): number {
    return this.threads.size;
  }
}

/**
 * Create thread service
 */
export function createThreadService(client?: MycelixMailClient): ThreadService {
  return new ThreadService(client);
}

export default ThreadService;
