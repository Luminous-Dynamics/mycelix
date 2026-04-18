// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Contact Service for Mycelix Mail
 *
 * Address book with trust integration:
 * - Contact CRUD with metadata
 * - Groups and labels
 * - Trust score integration
 * - Smart suggestions
 * - Import/export (vCard)
 * - Frequency-based ranking
 */

import type { AgentPubKey, ActionHash } from '@holochain/client';
import type { MycelixMailClient } from '../index';
import type { TrustScore, TrustCategory } from '../types';

// ==================== TYPES ====================

export interface Contact {
  id: string;
  hash?: ActionHash;
  agentPubKey?: AgentPubKey;
  displayName: string;
  nickname?: string;
  emails: ContactEmail[];
  phones: ContactPhone[];
  addresses: ContactAddress[];
  organization?: string;
  title?: string;
  notes?: string;
  avatar?: string;
  groups: string[];
  labels: string[];
  trustScore?: number;
  isFavorite: boolean;
  isBlocked: boolean;
  metadata: ContactMetadata;
  createdAt: number;
  updatedAt: number;
}

export interface ContactEmail {
  email: string;
  type: 'home' | 'work' | 'other';
  isPrimary: boolean;
  verified?: boolean;
}

export interface ContactPhone {
  number: string;
  type: 'mobile' | 'home' | 'work' | 'other';
  isPrimary: boolean;
}

export interface ContactAddress {
  street?: string;
  city?: string;
  state?: string;
  postalCode?: string;
  country?: string;
  type: 'home' | 'work' | 'other';
}

export interface ContactMetadata {
  source?: 'manual' | 'imported' | 'auto' | 'federation';
  lastEmailSent?: number;
  lastEmailReceived?: number;
  emailCount: number;
  responseRate?: number;
  averageResponseTime?: number;
}

export interface ContactGroup {
  id: string;
  hash?: ActionHash;
  name: string;
  description?: string;
  color?: string;
  icon?: string;
  memberCount: number;
  createdAt: number;
}

export interface ContactSuggestion {
  contact: Contact;
  score: number;
  reason: SuggestionReason;
}

export type SuggestionReason =
  | 'frequent'
  | 'recent'
  | 'trusted'
  | 'same_group'
  | 'similar_name'
  | 'thread_participant';

export interface ContactFilter {
  query?: string;
  groups?: string[];
  labels?: string[];
  hasEmail?: boolean;
  isFavorite?: boolean;
  isBlocked?: boolean;
  minTrustScore?: number;
  recentlyContacted?: number; // days
}

export interface ContactStats {
  total: number;
  favorites: number;
  blocked: number;
  withTrustScore: number;
  byGroup: Record<string, number>;
  topContacted: Contact[];
}

// ==================== CONTACT SERVICE ====================

export class ContactService {
  private contacts: Map<string, Contact> = new Map();
  private groups: Map<string, ContactGroup> = new Map();
  private emailIndex: Map<string, string> = new Map(); // email -> contact id
  private nameIndex: Map<string, Set<string>> = new Map(); // name prefix -> contact ids

  constructor(private client?: MycelixMailClient) {}

  // ==================== CONTACT CRUD ====================

  /**
   * Create a new contact
   */
  async createContact(data: Partial<Contact>): Promise<Contact> {
    const id = this.generateId();
    const now = Date.now();

    const contact: Contact = {
      id,
      displayName: data.displayName ?? 'Unknown',
      emails: data.emails ?? [],
      phones: data.phones ?? [],
      addresses: data.addresses ?? [],
      groups: data.groups ?? [],
      labels: data.labels ?? [],
      isFavorite: data.isFavorite ?? false,
      isBlocked: data.isBlocked ?? false,
      metadata: {
        source: 'manual',
        emailCount: 0,
        ...data.metadata,
      },
      createdAt: now,
      updatedAt: now,
      ...data,
    };

    // Fetch trust score if agent key available
    if (contact.agentPubKey && this.client) {
      contact.trustScore = await this.fetchTrustScore(contact.agentPubKey);
    }

    // Store in Holochain if client available
    if (this.client) {
      contact.hash = await this.client.callZome('contacts', 'create_contact', contact);
    }

    // Update local cache
    this.contacts.set(id, contact);
    this.indexContact(contact);

    return contact;
  }

  /**
   * Get contact by ID
   */
  getContact(id: string): Contact | null {
    return this.contacts.get(id) ?? null;
  }

  /**
   * Get contact by email
   */
  getContactByEmail(email: string): Contact | null {
    const normalizedEmail = email.toLowerCase().trim();
    const contactId = this.emailIndex.get(normalizedEmail);
    return contactId ? this.contacts.get(contactId) ?? null : null;
  }

  /**
   * Update contact
   */
  async updateContact(id: string, updates: Partial<Contact>): Promise<Contact | null> {
    const contact = this.contacts.get(id);
    if (!contact) return null;

    // Remove from indexes
    this.unindexContact(contact);

    // Apply updates
    Object.assign(contact, updates, { updatedAt: Date.now() });

    // Update in Holochain
    if (this.client && contact.hash) {
      await this.client.callZome('contacts', 'update_contact', {
        hash: contact.hash,
        contact,
      });
    }

    // Re-index
    this.indexContact(contact);

    return contact;
  }

  /**
   * Delete contact
   */
  async deleteContact(id: string): Promise<boolean> {
    const contact = this.contacts.get(id);
    if (!contact) return false;

    // Remove from Holochain
    if (this.client && contact.hash) {
      await this.client.callZome('contacts', 'delete_contact', contact.hash);
    }

    // Remove from cache
    this.unindexContact(contact);
    this.contacts.delete(id);

    return true;
  }

  /**
   * Get all contacts
   */
  getAllContacts(): Contact[] {
    return Array.from(this.contacts.values());
  }

  // ==================== SEARCH & FILTER ====================

  /**
   * Search contacts
   */
  search(query: string, limit = 20): Contact[] {
    const normalizedQuery = query.toLowerCase().trim();
    if (!normalizedQuery) return [];

    const results: Array<{ contact: Contact; score: number }> = [];

    for (const contact of this.contacts.values()) {
      const score = this.calculateSearchScore(contact, normalizedQuery);
      if (score > 0) {
        results.push({ contact, score });
      }
    }

    return results
      .sort((a, b) => b.score - a.score)
      .slice(0, limit)
      .map((r) => r.contact);
  }

  /**
   * Calculate search relevance score
   */
  private calculateSearchScore(contact: Contact, query: string): number {
    let score = 0;

    // Display name match
    const displayName = contact.displayName.toLowerCase();
    if (displayName === query) {
      score += 100;
    } else if (displayName.startsWith(query)) {
      score += 50;
    } else if (displayName.includes(query)) {
      score += 20;
    }

    // Nickname match
    if (contact.nickname) {
      const nickname = contact.nickname.toLowerCase();
      if (nickname.startsWith(query)) {
        score += 40;
      } else if (nickname.includes(query)) {
        score += 15;
      }
    }

    // Email match
    for (const email of contact.emails) {
      const emailLower = email.email.toLowerCase();
      if (emailLower.startsWith(query)) {
        score += 60;
      } else if (emailLower.includes(query)) {
        score += 25;
      }
    }

    // Organization match
    if (contact.organization) {
      const org = contact.organization.toLowerCase();
      if (org.includes(query)) {
        score += 10;
      }
    }

    // Boost favorites
    if (contact.isFavorite) {
      score *= 1.5;
    }

    // Boost trusted contacts
    if (contact.trustScore && contact.trustScore > 0.7) {
      score *= 1.3;
    }

    // Boost frequently contacted
    if (contact.metadata.emailCount > 10) {
      score *= 1.2;
    }

    return score;
  }

  /**
   * Filter contacts
   */
  filter(filter: ContactFilter): Contact[] {
    let results = Array.from(this.contacts.values());

    if (filter.query) {
      results = this.search(filter.query, 1000);
    }

    if (filter.groups?.length) {
      results = results.filter((c) =>
        c.groups.some((g) => filter.groups!.includes(g))
      );
    }

    if (filter.labels?.length) {
      results = results.filter((c) =>
        c.labels.some((l) => filter.labels!.includes(l))
      );
    }

    if (filter.hasEmail !== undefined) {
      results = results.filter((c) =>
        filter.hasEmail ? c.emails.length > 0 : c.emails.length === 0
      );
    }

    if (filter.isFavorite !== undefined) {
      results = results.filter((c) => c.isFavorite === filter.isFavorite);
    }

    if (filter.isBlocked !== undefined) {
      results = results.filter((c) => c.isBlocked === filter.isBlocked);
    }

    if (filter.minTrustScore !== undefined) {
      results = results.filter(
        (c) => (c.trustScore ?? 0) >= filter.minTrustScore!
      );
    }

    if (filter.recentlyContacted !== undefined) {
      const threshold = Date.now() - filter.recentlyContacted * 24 * 60 * 60 * 1000;
      results = results.filter(
        (c) =>
          (c.metadata.lastEmailSent ?? 0) > threshold ||
          (c.metadata.lastEmailReceived ?? 0) > threshold
      );
    }

    return results;
  }

  // ==================== SUGGESTIONS ====================

  /**
   * Get contact suggestions for autocomplete
   */
  getSuggestions(
    query: string,
    context?: { threadParticipants?: string[]; recentRecipients?: string[] }
  ): ContactSuggestion[] {
    const suggestions: ContactSuggestion[] = [];
    const normalizedQuery = query.toLowerCase().trim();

    for (const contact of this.contacts.values()) {
      if (contact.isBlocked) continue;

      // Check if matches query
      const matchesQuery =
        contact.displayName.toLowerCase().includes(normalizedQuery) ||
        contact.emails.some((e) => e.email.toLowerCase().includes(normalizedQuery));

      if (!matchesQuery) continue;

      let score = 0;
      let reason: SuggestionReason = 'similar_name';

      // Check various scoring factors
      if (context?.threadParticipants?.some((p) =>
        contact.emails.some((e) => e.email === p)
      )) {
        score += 50;
        reason = 'thread_participant';
      }

      if (context?.recentRecipients?.some((r) =>
        contact.emails.some((e) => e.email === r)
      )) {
        score += 40;
        reason = 'recent';
      }

      if (contact.metadata.emailCount > 20) {
        score += 30;
        if (reason === 'similar_name') reason = 'frequent';
      }

      if (contact.trustScore && contact.trustScore > 0.8) {
        score += 20;
        if (reason === 'similar_name') reason = 'trusted';
      }

      if (contact.isFavorite) {
        score += 25;
      }

      // Recency bonus
      const lastContact = Math.max(
        contact.metadata.lastEmailSent ?? 0,
        contact.metadata.lastEmailReceived ?? 0
      );
      const daysSinceContact = (Date.now() - lastContact) / (24 * 60 * 60 * 1000);
      if (daysSinceContact < 7) {
        score += 15;
      } else if (daysSinceContact < 30) {
        score += 5;
      }

      suggestions.push({ contact, score, reason });
    }

    return suggestions
      .sort((a, b) => b.score - a.score)
      .slice(0, 10);
  }

  // ==================== GROUPS ====================

  /**
   * Create a contact group
   */
  async createGroup(name: string, description?: string): Promise<ContactGroup> {
    const id = this.generateId();
    const group: ContactGroup = {
      id,
      name,
      description,
      memberCount: 0,
      createdAt: Date.now(),
    };

    if (this.client) {
      group.hash = await this.client.callZome('contacts', 'create_group', group);
    }

    this.groups.set(id, group);
    return group;
  }

  /**
   * Add contact to group
   */
  async addToGroup(contactId: string, groupId: string): Promise<boolean> {
    const contact = this.contacts.get(contactId);
    const group = this.groups.get(groupId);

    if (!contact || !group) return false;

    if (!contact.groups.includes(groupId)) {
      contact.groups.push(groupId);
      contact.updatedAt = Date.now();
      group.memberCount++;

      if (this.client && contact.hash) {
        await this.client.callZome('contacts', 'add_to_group', {
          contactHash: contact.hash,
          groupId,
        });
      }
    }

    return true;
  }

  /**
   * Remove contact from group
   */
  async removeFromGroup(contactId: string, groupId: string): Promise<boolean> {
    const contact = this.contacts.get(contactId);
    const group = this.groups.get(groupId);

    if (!contact || !group) return false;

    const index = contact.groups.indexOf(groupId);
    if (index !== -1) {
      contact.groups.splice(index, 1);
      contact.updatedAt = Date.now();
      group.memberCount--;

      if (this.client && contact.hash) {
        await this.client.callZome('contacts', 'remove_from_group', {
          contactHash: contact.hash,
          groupId,
        });
      }
    }

    return true;
  }

  /**
   * Get contacts in group
   */
  getGroupMembers(groupId: string): Contact[] {
    return Array.from(this.contacts.values())
      .filter((c) => c.groups.includes(groupId));
  }

  /**
   * Get all groups
   */
  getAllGroups(): ContactGroup[] {
    return Array.from(this.groups.values());
  }

  // ==================== TRUST INTEGRATION ====================

  /**
   * Fetch trust score for agent
   */
  private async fetchTrustScore(agent: AgentPubKey): Promise<number> {
    if (!this.client) return 0;

    try {
      const score: TrustScore = await this.client.trust.getTrustScore({
        subject: agent,
        include_transitive: true,
      });
      return score.combined_score;
    } catch {
      return 0;
    }
  }

  /**
   * Refresh trust scores for all contacts
   */
  async refreshTrustScores(): Promise<void> {
    for (const contact of this.contacts.values()) {
      if (contact.agentPubKey) {
        contact.trustScore = await this.fetchTrustScore(contact.agentPubKey);
      }
    }
  }

  /**
   * Block contact
   */
  async blockContact(id: string): Promise<boolean> {
    const contact = this.contacts.get(id);
    if (!contact) return false;

    contact.isBlocked = true;
    contact.updatedAt = Date.now();

    if (this.client && contact.hash) {
      await this.client.callZome('contacts', 'block_contact', contact.hash);
    }

    return true;
  }

  /**
   * Unblock contact
   */
  async unblockContact(id: string): Promise<boolean> {
    const contact = this.contacts.get(id);
    if (!contact) return false;

    contact.isBlocked = false;
    contact.updatedAt = Date.now();

    if (this.client && contact.hash) {
      await this.client.callZome('contacts', 'unblock_contact', contact.hash);
    }

    return true;
  }

  // ==================== IMPORT/EXPORT ====================

  /**
   * Import contacts from vCard format
   */
  async importVCard(vcardData: string): Promise<{ imported: number; errors: string[] }> {
    const errors: string[] = [];
    let imported = 0;

    const vcards = vcardData.split('END:VCARD').filter((v) => v.includes('BEGIN:VCARD'));

    for (const vcard of vcards) {
      try {
        const contact = this.parseVCard(vcard + 'END:VCARD');
        if (contact) {
          await this.createContact(contact);
          imported++;
        }
      } catch (e) {
        errors.push(String(e));
      }
    }

    return { imported, errors };
  }

  /**
   * Parse a single vCard
   */
  private parseVCard(vcard: string): Partial<Contact> | null {
    const lines = vcard.split(/\r?\n/);
    const contact: Partial<Contact> = {
      emails: [],
      phones: [],
      addresses: [],
      groups: [],
      labels: [],
      metadata: { source: 'imported', emailCount: 0 },
    };

    for (const line of lines) {
      const colonIndex = line.indexOf(':');
      if (colonIndex === -1) continue;

      const fieldPart = line.substring(0, colonIndex);
      const value = line.substring(colonIndex + 1).trim();
      const [field, ...params] = fieldPart.split(';');

      switch (field.toUpperCase()) {
        case 'FN':
          contact.displayName = value;
          break;

        case 'N':
          if (!contact.displayName) {
            const [last, first] = value.split(';');
            contact.displayName = `${first ?? ''} ${last ?? ''}`.trim();
          }
          break;

        case 'NICKNAME':
          contact.nickname = value;
          break;

        case 'EMAIL':
          const emailType = this.parseVCardType(params);
          contact.emails!.push({
            email: value,
            type: emailType as 'home' | 'work' | 'other',
            isPrimary: params.some((p) => p.toUpperCase() === 'PREF'),
          });
          break;

        case 'TEL':
          const phoneType = this.parseVCardType(params);
          contact.phones!.push({
            number: value,
            type: phoneType as 'mobile' | 'home' | 'work' | 'other',
            isPrimary: params.some((p) => p.toUpperCase() === 'PREF'),
          });
          break;

        case 'ORG':
          contact.organization = value.split(';')[0];
          break;

        case 'TITLE':
          contact.title = value;
          break;

        case 'NOTE':
          contact.notes = value;
          break;

        case 'PHOTO':
          // Base64 encoded photo
          if (value.startsWith('data:') || params.some((p) => p.includes('BASE64'))) {
            contact.avatar = value;
          }
          break;
      }
    }

    if (!contact.displayName && contact.emails!.length === 0) {
      return null;
    }

    return contact;
  }

  /**
   * Parse vCard type parameter
   */
  private parseVCardType(params: string[]): string {
    for (const param of params) {
      const [key, value] = param.split('=');
      if (key.toUpperCase() === 'TYPE' && value) {
        const type = value.toLowerCase();
        if (['home', 'work', 'mobile', 'cell'].includes(type)) {
          return type === 'cell' ? 'mobile' : type;
        }
      }
    }
    return 'other';
  }

  /**
   * Export contacts to vCard format
   */
  exportToVCard(contactIds?: string[]): string {
    const contacts = contactIds
      ? contactIds.map((id) => this.contacts.get(id)).filter(Boolean) as Contact[]
      : Array.from(this.contacts.values());

    return contacts.map((c) => this.contactToVCard(c)).join('\n');
  }

  /**
   * Convert contact to vCard format
   */
  private contactToVCard(contact: Contact): string {
    const lines: string[] = ['BEGIN:VCARD', 'VERSION:3.0'];

    lines.push(`FN:${contact.displayName}`);

    const [first, ...rest] = contact.displayName.split(' ');
    const last = rest.join(' ');
    lines.push(`N:${last};${first};;;`);

    if (contact.nickname) {
      lines.push(`NICKNAME:${contact.nickname}`);
    }

    for (const email of contact.emails) {
      const pref = email.isPrimary ? ';PREF' : '';
      lines.push(`EMAIL;TYPE=${email.type.toUpperCase()}${pref}:${email.email}`);
    }

    for (const phone of contact.phones) {
      const pref = phone.isPrimary ? ';PREF' : '';
      lines.push(`TEL;TYPE=${phone.type.toUpperCase()}${pref}:${phone.number}`);
    }

    if (contact.organization) {
      lines.push(`ORG:${contact.organization}`);
    }

    if (contact.title) {
      lines.push(`TITLE:${contact.title}`);
    }

    if (contact.notes) {
      lines.push(`NOTE:${contact.notes}`);
    }

    lines.push('END:VCARD');
    return lines.join('\r\n');
  }

  // ==================== AUTO-DISCOVERY ====================

  /**
   * Auto-create contact from email interaction
   */
  async autoCreateFromEmail(
    email: string,
    displayName?: string,
    isSender = false
  ): Promise<Contact> {
    // Check if contact already exists
    const existing = this.getContactByEmail(email);
    if (existing) {
      // Update metadata
      const now = Date.now();
      if (isSender) {
        existing.metadata.lastEmailReceived = now;
      } else {
        existing.metadata.lastEmailSent = now;
      }
      existing.metadata.emailCount++;
      existing.updatedAt = now;
      return existing;
    }

    // Create new contact
    return this.createContact({
      displayName: displayName ?? this.extractNameFromEmail(email),
      emails: [{ email, type: 'other', isPrimary: true }],
      metadata: {
        source: 'auto',
        emailCount: 1,
        lastEmailSent: isSender ? undefined : Date.now(),
        lastEmailReceived: isSender ? Date.now() : undefined,
      },
    });
  }

  /**
   * Extract name from email address
   */
  private extractNameFromEmail(email: string): string {
    const localPart = email.split('@')[0];
    // Convert dots and underscores to spaces, capitalize words
    return localPart
      .replace(/[._]/g, ' ')
      .replace(/\b\w/g, (c) => c.toUpperCase());
  }

  // ==================== INDEXING ====================

  /**
   * Index a contact for fast lookups
   */
  private indexContact(contact: Contact): void {
    // Index by email
    for (const email of contact.emails) {
      this.emailIndex.set(email.email.toLowerCase(), contact.id);
    }

    // Index by name prefix
    const namePrefixes = this.getNamePrefixes(contact.displayName);
    for (const prefix of namePrefixes) {
      if (!this.nameIndex.has(prefix)) {
        this.nameIndex.set(prefix, new Set());
      }
      this.nameIndex.get(prefix)!.add(contact.id);
    }
  }

  /**
   * Remove contact from indexes
   */
  private unindexContact(contact: Contact): void {
    // Remove from email index
    for (const email of contact.emails) {
      this.emailIndex.delete(email.email.toLowerCase());
    }

    // Remove from name index
    const namePrefixes = this.getNamePrefixes(contact.displayName);
    for (const prefix of namePrefixes) {
      this.nameIndex.get(prefix)?.delete(contact.id);
    }
  }

  /**
   * Get name prefixes for indexing
   */
  private getNamePrefixes(name: string): string[] {
    const prefixes: string[] = [];
    const words = name.toLowerCase().split(/\s+/);

    for (const word of words) {
      for (let i = 1; i <= Math.min(word.length, 4); i++) {
        prefixes.push(word.substring(0, i));
      }
    }

    return prefixes;
  }

  // ==================== STATISTICS ====================

  /**
   * Get contact statistics
   */
  getStats(): ContactStats {
    const contacts = Array.from(this.contacts.values());
    const byGroup: Record<string, number> = {};

    for (const group of this.groups.values()) {
      byGroup[group.name] = group.memberCount;
    }

    const topContacted = contacts
      .sort((a, b) => b.metadata.emailCount - a.metadata.emailCount)
      .slice(0, 10);

    return {
      total: contacts.length,
      favorites: contacts.filter((c) => c.isFavorite).length,
      blocked: contacts.filter((c) => c.isBlocked).length,
      withTrustScore: contacts.filter((c) => c.trustScore !== undefined).length,
      byGroup,
      topContacted,
    };
  }

  // ==================== UTILITIES ====================

  private generateId(): string {
    return `contact_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * Load contacts from Holochain
   */
  async loadFromHolochain(): Promise<void> {
    if (!this.client) return;

    try {
      const contacts: Contact[] = await this.client.callZome('contacts', 'get_all_contacts', null);
      for (const contact of contacts) {
        this.contacts.set(contact.id, contact);
        this.indexContact(contact);
      }

      const groups: ContactGroup[] = await this.client.callZome('contacts', 'get_all_groups', null);
      for (const group of groups) {
        this.groups.set(group.id, group);
      }
    } catch (e) {
      console.error('Failed to load contacts:', e);
    }
  }
}

/**
 * Create contact service
 */
export function createContactService(client?: MycelixMailClient): ContactService {
  return new ContactService(client);
}

export default ContactService;
