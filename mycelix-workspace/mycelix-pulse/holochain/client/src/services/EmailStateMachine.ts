// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Email State Machine for Mycelix Mail
 *
 * Type-safe email lifecycle management:
 * - Defined states and transitions
 * - Guards for conditional transitions
 * - Actions on state entry/exit
 * - Event-driven architecture
 * - Undo/redo support
 */

import type { ActionHash } from '@holochain/client';
import type { MycelixMailClient } from '../index';
import type { DecryptedEmail } from '../types';

// ==================== TYPES ====================

export type EmailState =
  | 'draft'
  | 'sending'
  | 'sent'
  | 'delivered'
  | 'unread'
  | 'read'
  | 'replied'
  | 'forwarded'
  | 'archived'
  | 'trashed'
  | 'deleted'
  | 'spam'
  | 'quarantined';

export type EmailEvent =
  | 'SAVE'
  | 'SEND'
  | 'SEND_SUCCESS'
  | 'SEND_FAILURE'
  | 'DELIVER'
  | 'RECEIVE'
  | 'OPEN'
  | 'REPLY'
  | 'FORWARD'
  | 'ARCHIVE'
  | 'UNARCHIVE'
  | 'TRASH'
  | 'RESTORE'
  | 'DELETE'
  | 'MARK_SPAM'
  | 'MARK_NOT_SPAM'
  | 'QUARANTINE'
  | 'RELEASE'
  | 'MARK_UNREAD'
  | 'EXPIRE';

export interface StateTransition {
  from: EmailState | EmailState[];
  to: EmailState;
  event: EmailEvent;
  guard?: (context: EmailContext) => boolean;
  action?: (context: EmailContext) => void | Promise<void>;
}

export interface EmailContext {
  email: DecryptedEmail;
  emailHash: ActionHash;
  previousState?: EmailState;
  metadata?: Record<string, unknown>;
  timestamp: number;
}

export interface StateChangeEvent {
  emailHash: ActionHash;
  fromState: EmailState;
  toState: EmailState;
  event: EmailEvent;
  timestamp: number;
  context: EmailContext;
}

export interface UndoAction {
  id: string;
  emailHash: ActionHash;
  fromState: EmailState;
  toState: EmailState;
  event: EmailEvent;
  timestamp: number;
  expiresAt: number;
}

export type StateChangeHandler = (event: StateChangeEvent) => void;

// ==================== STATE MACHINE ====================

export class EmailStateMachine {
  private transitions: StateTransition[] = [];
  private entryActions: Map<EmailState, Array<(ctx: EmailContext) => void | Promise<void>>> = new Map();
  private exitActions: Map<EmailState, Array<(ctx: EmailContext) => void | Promise<void>>> = new Map();
  private handlers: Set<StateChangeHandler> = new Set();
  private undoStack: UndoAction[] = [];
  private redoStack: UndoAction[] = [];

  private readonly MAX_UNDO = 50;
  private readonly UNDO_TIMEOUT = 5 * 60 * 1000; // 5 minutes

  constructor(private client?: MycelixMailClient) {
    this.defineTransitions();
    this.defineActions();
  }

  // ==================== TRANSITION DEFINITIONS ====================

  /**
   * Define all valid state transitions
   */
  private defineTransitions(): void {
    // Draft transitions
    this.addTransition({
      from: 'draft',
      to: 'draft',
      event: 'SAVE',
    });

    this.addTransition({
      from: 'draft',
      to: 'sending',
      event: 'SEND',
    });

    this.addTransition({
      from: 'draft',
      to: 'trashed',
      event: 'TRASH',
    });

    // Sending transitions
    this.addTransition({
      from: 'sending',
      to: 'sent',
      event: 'SEND_SUCCESS',
    });

    this.addTransition({
      from: 'sending',
      to: 'draft',
      event: 'SEND_FAILURE',
    });

    // Sent transitions
    this.addTransition({
      from: 'sent',
      to: 'delivered',
      event: 'DELIVER',
    });

    this.addTransition({
      from: 'sent',
      to: 'archived',
      event: 'ARCHIVE',
    });

    this.addTransition({
      from: 'sent',
      to: 'trashed',
      event: 'TRASH',
    });

    // Delivered transitions
    this.addTransition({
      from: 'delivered',
      to: 'archived',
      event: 'ARCHIVE',
    });

    // Received email transitions
    this.addTransition({
      from: 'unread',
      to: 'read',
      event: 'OPEN',
    });

    this.addTransition({
      from: 'unread',
      to: 'spam',
      event: 'MARK_SPAM',
    });

    this.addTransition({
      from: 'unread',
      to: 'quarantined',
      event: 'QUARANTINE',
    });

    this.addTransition({
      from: 'unread',
      to: 'archived',
      event: 'ARCHIVE',
    });

    this.addTransition({
      from: 'unread',
      to: 'trashed',
      event: 'TRASH',
    });

    // Read email transitions
    this.addTransition({
      from: 'read',
      to: 'unread',
      event: 'MARK_UNREAD',
    });

    this.addTransition({
      from: 'read',
      to: 'replied',
      event: 'REPLY',
    });

    this.addTransition({
      from: 'read',
      to: 'forwarded',
      event: 'FORWARD',
    });

    this.addTransition({
      from: 'read',
      to: 'archived',
      event: 'ARCHIVE',
    });

    this.addTransition({
      from: 'read',
      to: 'trashed',
      event: 'TRASH',
    });

    this.addTransition({
      from: 'read',
      to: 'spam',
      event: 'MARK_SPAM',
    });

    // Replied/Forwarded transitions
    this.addTransition({
      from: ['replied', 'forwarded'],
      to: 'archived',
      event: 'ARCHIVE',
    });

    this.addTransition({
      from: ['replied', 'forwarded'],
      to: 'trashed',
      event: 'TRASH',
    });

    // Archived transitions
    this.addTransition({
      from: 'archived',
      to: 'read',
      event: 'UNARCHIVE',
    });

    this.addTransition({
      from: 'archived',
      to: 'trashed',
      event: 'TRASH',
    });

    // Trashed transitions
    this.addTransition({
      from: 'trashed',
      to: 'read',
      event: 'RESTORE',
      guard: (ctx) => ctx.previousState === 'read' || !ctx.previousState,
    });

    this.addTransition({
      from: 'trashed',
      to: 'unread',
      event: 'RESTORE',
      guard: (ctx) => ctx.previousState === 'unread',
    });

    this.addTransition({
      from: 'trashed',
      to: 'archived',
      event: 'RESTORE',
      guard: (ctx) => ctx.previousState === 'archived',
    });

    this.addTransition({
      from: 'trashed',
      to: 'deleted',
      event: 'DELETE',
    });

    this.addTransition({
      from: 'trashed',
      to: 'deleted',
      event: 'EXPIRE',
    });

    // Spam transitions
    this.addTransition({
      from: 'spam',
      to: 'read',
      event: 'MARK_NOT_SPAM',
    });

    this.addTransition({
      from: 'spam',
      to: 'deleted',
      event: 'DELETE',
    });

    // Quarantined transitions
    this.addTransition({
      from: 'quarantined',
      to: 'unread',
      event: 'RELEASE',
    });

    this.addTransition({
      from: 'quarantined',
      to: 'deleted',
      event: 'DELETE',
    });

    this.addTransition({
      from: 'quarantined',
      to: 'deleted',
      event: 'EXPIRE',
    });
  }

  /**
   * Define entry/exit actions for states
   */
  private defineActions(): void {
    // Entry actions
    this.onEnter('read', async (ctx) => {
      if (this.client) {
        await this.client.messages.markAsRead(ctx.emailHash);
      }
    });

    this.onEnter('archived', async (ctx) => {
      if (this.client) {
        await this.client.messages.archiveEmail(ctx.emailHash);
      }
    });

    this.onEnter('trashed', async (ctx) => {
      if (this.client) {
        await this.client.messages.deleteEmail(ctx.emailHash);
      }
    });

    this.onEnter('deleted', async (ctx) => {
      if (this.client) {
        await this.client.messages.permanentlyDelete(ctx.emailHash);
      }
    });

    this.onEnter('spam', async (ctx) => {
      if (this.client) {
        await this.client.messages.addLabel(ctx.emailHash, 'spam');
      }
    });

    // Exit actions
    this.onExit('spam', async (ctx) => {
      if (this.client) {
        await this.client.messages.removeLabel(ctx.emailHash, 'spam');
      }
    });
  }

  // ==================== TRANSITION MANAGEMENT ====================

  /**
   * Add a state transition
   */
  addTransition(transition: StateTransition): void {
    this.transitions.push(transition);
  }

  /**
   * Add entry action for a state
   */
  onEnter(
    state: EmailState,
    action: (ctx: EmailContext) => void | Promise<void>
  ): void {
    if (!this.entryActions.has(state)) {
      this.entryActions.set(state, []);
    }
    this.entryActions.get(state)!.push(action);
  }

  /**
   * Add exit action for a state
   */
  onExit(
    state: EmailState,
    action: (ctx: EmailContext) => void | Promise<void>
  ): void {
    if (!this.exitActions.has(state)) {
      this.exitActions.set(state, []);
    }
    this.exitActions.get(state)!.push(action);
  }

  // ==================== STATE TRANSITIONS ====================

  /**
   * Check if a transition is valid
   */
  canTransition(
    currentState: EmailState,
    event: EmailEvent,
    context?: Partial<EmailContext>
  ): boolean {
    const transition = this.findTransition(currentState, event, context);
    return transition !== null;
  }

  /**
   * Get valid events for current state
   */
  getValidEvents(currentState: EmailState): EmailEvent[] {
    return this.transitions
      .filter((t) => {
        const fromStates = Array.isArray(t.from) ? t.from : [t.from];
        return fromStates.includes(currentState);
      })
      .map((t) => t.event)
      .filter((event, index, self) => self.indexOf(event) === index);
  }

  /**
   * Get possible next states from current state
   */
  getNextStates(currentState: EmailState): EmailState[] {
    return this.transitions
      .filter((t) => {
        const fromStates = Array.isArray(t.from) ? t.from : [t.from];
        return fromStates.includes(currentState);
      })
      .map((t) => t.to)
      .filter((state, index, self) => self.indexOf(state) === index);
  }

  /**
   * Perform state transition
   */
  async transition(
    currentState: EmailState,
    event: EmailEvent,
    context: EmailContext
  ): Promise<EmailState | null> {
    const transition = this.findTransition(currentState, event, context);

    if (!transition) {
      console.warn(`Invalid transition: ${currentState} + ${event}`);
      return null;
    }

    // Store for undo
    context.previousState = currentState;

    // Run exit actions
    const exitActions = this.exitActions.get(currentState) || [];
    for (const action of exitActions) {
      await action(context);
    }

    // Run transition action
    if (transition.action) {
      await transition.action(context);
    }

    // Run entry actions
    const entryActions = this.entryActions.get(transition.to) || [];
    for (const action of entryActions) {
      await action(context);
    }

    // Add to undo stack
    this.addUndoAction({
      emailHash: context.emailHash,
      fromState: currentState,
      toState: transition.to,
      event,
    });

    // Notify handlers
    const changeEvent: StateChangeEvent = {
      emailHash: context.emailHash,
      fromState: currentState,
      toState: transition.to,
      event,
      timestamp: Date.now(),
      context,
    };

    for (const handler of this.handlers) {
      try {
        handler(changeEvent);
      } catch (e) {
        console.error('State change handler error:', e);
      }
    }

    return transition.to;
  }

  /**
   * Find matching transition
   */
  private findTransition(
    currentState: EmailState,
    event: EmailEvent,
    context?: Partial<EmailContext>
  ): StateTransition | null {
    for (const transition of this.transitions) {
      const fromStates = Array.isArray(transition.from)
        ? transition.from
        : [transition.from];

      if (!fromStates.includes(currentState)) continue;
      if (transition.event !== event) continue;

      // Check guard
      if (transition.guard && context) {
        const fullContext: EmailContext = {
          email: context.email ?? ({} as DecryptedEmail),
          emailHash: context.emailHash ?? (new Uint8Array() as ActionHash),
          previousState: context.previousState,
          metadata: context.metadata,
          timestamp: context.timestamp ?? Date.now(),
        };

        if (!transition.guard(fullContext)) continue;
      }

      return transition;
    }

    return null;
  }

  // ==================== UNDO/REDO ====================

  /**
   * Add action to undo stack
   */
  private addUndoAction(action: Omit<UndoAction, 'id' | 'timestamp' | 'expiresAt'>): void {
    const now = Date.now();
    const undoAction: UndoAction = {
      ...action,
      id: `undo_${now}_${Math.random().toString(36).substr(2, 9)}`,
      timestamp: now,
      expiresAt: now + this.UNDO_TIMEOUT,
    };

    this.undoStack.push(undoAction);
    this.redoStack = []; // Clear redo stack on new action

    // Trim stack if too large
    while (this.undoStack.length > this.MAX_UNDO) {
      this.undoStack.shift();
    }

    // Clean expired
    this.cleanExpiredUndo();
  }

  /**
   * Clean expired undo actions
   */
  private cleanExpiredUndo(): void {
    const now = Date.now();
    this.undoStack = this.undoStack.filter((a) => a.expiresAt > now);
  }

  /**
   * Check if undo is available
   */
  canUndo(): boolean {
    this.cleanExpiredUndo();
    return this.undoStack.length > 0;
  }

  /**
   * Check if redo is available
   */
  canRedo(): boolean {
    return this.redoStack.length > 0;
  }

  /**
   * Get undo preview
   */
  getUndoPreview(): UndoAction | null {
    this.cleanExpiredUndo();
    return this.undoStack[this.undoStack.length - 1] ?? null;
  }

  /**
   * Undo last action
   */
  async undo(context: Partial<EmailContext> = {}): Promise<boolean> {
    this.cleanExpiredUndo();
    const action = this.undoStack.pop();
    if (!action) return false;

    // Find reverse event
    const reverseEvent = this.getReverseEvent(action.event);
    if (!reverseEvent) return false;

    const fullContext: EmailContext = {
      email: context.email ?? ({} as DecryptedEmail),
      emailHash: action.emailHash,
      previousState: action.fromState,
      metadata: context.metadata,
      timestamp: Date.now(),
    };

    const result = await this.transition(action.toState, reverseEvent, fullContext);

    if (result === action.fromState) {
      this.redoStack.push(action);
      return true;
    }

    return false;
  }

  /**
   * Redo last undone action
   */
  async redo(context: Partial<EmailContext> = {}): Promise<boolean> {
    const action = this.redoStack.pop();
    if (!action) return false;

    const fullContext: EmailContext = {
      email: context.email ?? ({} as DecryptedEmail),
      emailHash: action.emailHash,
      previousState: action.fromState,
      metadata: context.metadata,
      timestamp: Date.now(),
    };

    const result = await this.transition(action.fromState, action.event, fullContext);
    return result === action.toState;
  }

  /**
   * Get reverse event for undo
   */
  private getReverseEvent(event: EmailEvent): EmailEvent | null {
    const reverseMap: Partial<Record<EmailEvent, EmailEvent>> = {
      ARCHIVE: 'UNARCHIVE',
      UNARCHIVE: 'ARCHIVE',
      TRASH: 'RESTORE',
      RESTORE: 'TRASH',
      OPEN: 'MARK_UNREAD',
      MARK_UNREAD: 'OPEN',
      MARK_SPAM: 'MARK_NOT_SPAM',
      MARK_NOT_SPAM: 'MARK_SPAM',
    };

    return reverseMap[event] ?? null;
  }

  // ==================== EVENT HANDLERS ====================

  /**
   * Subscribe to state changes
   */
  subscribe(handler: StateChangeHandler): () => void {
    this.handlers.add(handler);
    return () => this.handlers.delete(handler);
  }

  // ==================== UTILITIES ====================

  /**
   * Get state display info
   */
  getStateInfo(state: EmailState): {
    label: string;
    icon: string;
    color: string;
  } {
    const stateInfo: Record<EmailState, { label: string; icon: string; color: string }> = {
      draft: { label: 'Draft', icon: 'edit', color: 'gray' },
      sending: { label: 'Sending', icon: 'send', color: 'blue' },
      sent: { label: 'Sent', icon: 'check', color: 'green' },
      delivered: { label: 'Delivered', icon: 'check-double', color: 'green' },
      unread: { label: 'Unread', icon: 'envelope', color: 'blue' },
      read: { label: 'Read', icon: 'envelope-open', color: 'gray' },
      replied: { label: 'Replied', icon: 'reply', color: 'purple' },
      forwarded: { label: 'Forwarded', icon: 'share', color: 'purple' },
      archived: { label: 'Archived', icon: 'archive', color: 'yellow' },
      trashed: { label: 'Trashed', icon: 'trash', color: 'red' },
      deleted: { label: 'Deleted', icon: 'times', color: 'red' },
      spam: { label: 'Spam', icon: 'ban', color: 'orange' },
      quarantined: { label: 'Quarantined', icon: 'shield', color: 'orange' },
    };

    return stateInfo[state];
  }

  /**
   * Get all states
   */
  getAllStates(): EmailState[] {
    return [
      'draft', 'sending', 'sent', 'delivered',
      'unread', 'read', 'replied', 'forwarded',
      'archived', 'trashed', 'deleted', 'spam', 'quarantined',
    ];
  }

  /**
   * Get state diagram (for visualization)
   */
  getStateDiagram(): { states: EmailState[]; transitions: Array<{ from: string; to: string; event: string }> } {
    const diagram = {
      states: this.getAllStates(),
      transitions: this.transitions.map((t) => ({
        from: Array.isArray(t.from) ? t.from.join('|') : t.from,
        to: t.to,
        event: t.event,
      })),
    };

    return diagram;
  }
}

/**
 * Create email state machine with optional client
 */
export function createEmailStateMachine(client?: MycelixMailClient): EmailStateMachine {
  return new EmailStateMachine(client);
}

/**
 * Singleton instance
 */
let defaultMachine: EmailStateMachine | null = null;

export function getEmailStateMachine(client?: MycelixMailClient): EmailStateMachine {
  if (!defaultMachine) {
    defaultMachine = new EmailStateMachine(client);
  }
  return defaultMachine;
}

export default EmailStateMachine;
