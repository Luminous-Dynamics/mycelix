// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Search Service for Mycelix Mail
 *
 * Provides comprehensive search functionality with filters,
 * sorting, and pagination for emails and contacts.
 */

import type { AppClient, AgentPubKey, ActionHash } from '@holochain/client';
import type { Email, EmailThread, Contact } from '../types';

// Search filter types
export interface DateRange {
  from?: Date;
  to?: Date;
}

export interface SearchFilters {
  // Text search
  query?: string;
  searchIn?: ('subject' | 'body' | 'sender' | 'recipient')[];

  // Folder filters
  folder?: 'inbox' | 'sent' | 'drafts' | 'archive' | 'trash' | 'all';

  // Status filters
  isRead?: boolean;
  isStarred?: boolean;
  hasAttachments?: boolean;

  // Trust filters
  minTrustLevel?: number;
  maxTrustLevel?: number;
  trustedOnly?: boolean;

  // Date filters
  dateRange?: DateRange;
  after?: Date;
  before?: Date;

  // Sender/recipient filters
  from?: AgentPubKey[];
  to?: AgentPubKey[];
  participants?: AgentPubKey[];

  // Label/tag filters
  labels?: string[];
  hasLabels?: boolean;

  // Size filters
  minSize?: number;
  maxSize?: number;

  // Thread filters
  inThread?: ActionHash;
  isThreadRoot?: boolean;
}

export interface SearchSort {
  field: 'date' | 'sender' | 'subject' | 'trust' | 'size' | 'relevance';
  direction: 'asc' | 'desc';
}

export interface SearchOptions {
  filters?: SearchFilters;
  sort?: SearchSort;
  limit?: number;
  offset?: number;
  includeBody?: boolean;
  highlightMatches?: boolean;
}

export interface SearchResult<T> {
  items: T[];
  total: number;
  hasMore: boolean;
  took: number; // milliseconds
  query: string;
  filters: SearchFilters;
}

export interface EmailSearchResult extends Email {
  highlights?: {
    subject?: string[];
    body?: string[];
  };
  score?: number;
}

export interface ContactSearchResult extends Contact {
  highlights?: {
    name?: string[];
    email?: string[];
  };
  score?: number;
}

export interface SearchStats {
  totalEmails: number;
  totalContacts: number;
  indexedAt: Date;
  indexSize: number;
}

// Saved search
export interface SavedSearch {
  id: string;
  name: string;
  filters: SearchFilters;
  sort?: SearchSort;
  createdAt: Date;
  lastUsed?: Date;
  useCount: number;
}

/**
 * Search service implementation
 */
export class SearchService {
  private client: AppClient;
  private roleName: string;
  private cache: Map<string, { result: any; timestamp: number }> = new Map();
  private cacheTimeout = 30000; // 30 seconds

  constructor(client: AppClient, roleName: string = 'mycelix_mail') {
    this.client = client;
    this.roleName = roleName;
  }

  /**
   * Search emails with filters
   */
  async searchEmails(options: SearchOptions = {}): Promise<SearchResult<EmailSearchResult>> {
    const startTime = Date.now();
    const {
      filters = {},
      sort = { field: 'date', direction: 'desc' },
      limit = 50,
      offset = 0,
      includeBody = false,
      highlightMatches = true,
    } = options;

    // Build cache key
    const cacheKey = this.buildCacheKey('emails', options);
    const cached = this.getFromCache(cacheKey);
    if (cached) return cached;

    try {
      // Call search zome
      const result = await this.client.callZome({
        role_name: this.roleName,
        zome_name: 'search',
        fn_name: 'search_emails',
        payload: {
          filters: this.serializeFilters(filters),
          sort,
          limit,
          offset,
          include_body: includeBody,
        },
      });

      const emails = this.processEmailResults(result.items, filters, highlightMatches);

      const searchResult: SearchResult<EmailSearchResult> = {
        items: emails,
        total: result.total,
        hasMore: offset + emails.length < result.total,
        took: Date.now() - startTime,
        query: filters.query || '',
        filters,
      };

      this.setCache(cacheKey, searchResult);
      return searchResult;
    } catch (error) {
      console.error('Email search failed:', error);
      throw error;
    }
  }

  /**
   * Search contacts
   */
  async searchContacts(
    query: string,
    options: { limit?: number; includeBlocked?: boolean } = {}
  ): Promise<SearchResult<ContactSearchResult>> {
    const startTime = Date.now();
    const { limit = 20, includeBlocked = false } = options;

    try {
      const result = await this.client.callZome({
        role_name: this.roleName,
        zome_name: 'contacts',
        fn_name: 'search_contacts',
        payload: {
          query,
          limit,
          include_blocked: includeBlocked,
        },
      });

      const contacts = result.items.map((contact: any) => ({
        ...contact,
        highlights: this.generateHighlights(query, {
          name: contact.name,
          email: contact.email_address,
        }),
      }));

      return {
        items: contacts,
        total: result.total,
        hasMore: contacts.length < result.total,
        took: Date.now() - startTime,
        query,
        filters: {},
      };
    } catch (error) {
      console.error('Contact search failed:', error);
      throw error;
    }
  }

  /**
   * Search across all content types
   */
  async searchAll(
    query: string,
    options: { limit?: number } = {}
  ): Promise<{
    emails: SearchResult<EmailSearchResult>;
    contacts: SearchResult<ContactSearchResult>;
    took: number;
  }> {
    const startTime = Date.now();
    const { limit = 10 } = options;

    const [emails, contacts] = await Promise.all([
      this.searchEmails({ filters: { query }, limit }),
      this.searchContacts(query, { limit }),
    ]);

    return {
      emails,
      contacts,
      took: Date.now() - startTime,
    };
  }

  /**
   * Get search suggestions/autocomplete
   */
  async getSuggestions(
    query: string,
    context: 'emails' | 'contacts' | 'all' = 'all'
  ): Promise<string[]> {
    if (query.length < 2) return [];

    try {
      const result = await this.client.callZome({
        role_name: this.roleName,
        zome_name: 'search',
        fn_name: 'get_suggestions',
        payload: { query, context, limit: 10 },
      });

      return result.suggestions;
    } catch (error) {
      console.error('Suggestions failed:', error);
      return [];
    }
  }

  /**
   * Save a search for later use
   */
  async saveSearch(name: string, filters: SearchFilters, sort?: SearchSort): Promise<SavedSearch> {
    const savedSearch: SavedSearch = {
      id: crypto.randomUUID(),
      name,
      filters,
      sort,
      createdAt: new Date(),
      useCount: 0,
    };

    await this.client.callZome({
      role_name: this.roleName,
      zome_name: 'search',
      fn_name: 'save_search',
      payload: savedSearch,
    });

    return savedSearch;
  }

  /**
   * Get saved searches
   */
  async getSavedSearches(): Promise<SavedSearch[]> {
    const result = await this.client.callZome({
      role_name: this.roleName,
      zome_name: 'search',
      fn_name: 'get_saved_searches',
      payload: null,
    });

    return result.searches;
  }

  /**
   * Delete a saved search
   */
  async deleteSavedSearch(id: string): Promise<void> {
    await this.client.callZome({
      role_name: this.roleName,
      zome_name: 'search',
      fn_name: 'delete_saved_search',
      payload: { id },
    });
  }

  /**
   * Get search statistics
   */
  async getStats(): Promise<SearchStats> {
    const result = await this.client.callZome({
      role_name: this.roleName,
      zome_name: 'search',
      fn_name: 'get_stats',
      payload: null,
    });

    return {
      totalEmails: result.total_emails,
      totalContacts: result.total_contacts,
      indexedAt: new Date(result.indexed_at),
      indexSize: result.index_size,
    };
  }

  /**
   * Rebuild search index
   */
  async rebuildIndex(): Promise<void> {
    await this.client.callZome({
      role_name: this.roleName,
      zome_name: 'search',
      fn_name: 'rebuild_index',
      payload: null,
    });

    // Clear cache after rebuild
    this.clearCache();
  }

  /**
   * Parse search query with operators
   * Supports: from:, to:, subject:, has:, is:, before:, after:, label:
   */
  parseQuery(query: string): SearchFilters {
    const filters: SearchFilters = {};
    let textQuery = query;

    // Parse operators
    const operators: Record<string, RegExp> = {
      from: /from:(\S+)/gi,
      to: /to:(\S+)/gi,
      subject: /subject:"([^"]+)"|subject:(\S+)/gi,
      has: /has:(attachment|star)/gi,
      is: /is:(read|unread|starred|unstarred)/gi,
      before: /before:(\d{4}-\d{2}-\d{2})/gi,
      after: /after:(\d{4}-\d{2}-\d{2})/gi,
      label: /label:(\S+)/gi,
      trust: /trust:(>|<|>=|<=)?(\d+(?:\.\d+)?)/gi,
    };

    for (const [op, regex] of Object.entries(operators)) {
      let match;
      while ((match = regex.exec(query)) !== null) {
        textQuery = textQuery.replace(match[0], '').trim();

        switch (op) {
          case 'has':
            if (match[1] === 'attachment') filters.hasAttachments = true;
            if (match[1] === 'star') filters.isStarred = true;
            break;
          case 'is':
            if (match[1] === 'read') filters.isRead = true;
            if (match[1] === 'unread') filters.isRead = false;
            if (match[1] === 'starred') filters.isStarred = true;
            if (match[1] === 'unstarred') filters.isStarred = false;
            break;
          case 'before':
            filters.before = new Date(match[1]);
            break;
          case 'after':
            filters.after = new Date(match[1]);
            break;
          case 'label':
            filters.labels = filters.labels || [];
            filters.labels.push(match[1]);
            break;
          case 'trust':
            const comparator = match[1] || '>=';
            const value = parseFloat(match[2]);
            if (comparator.includes('>')) filters.minTrustLevel = value;
            if (comparator.includes('<')) filters.maxTrustLevel = value;
            break;
        }
      }
    }

    if (textQuery.trim()) {
      filters.query = textQuery.trim();
    }

    return filters;
  }

  /**
   * Build query string from filters
   */
  buildQueryString(filters: SearchFilters): string {
    const parts: string[] = [];

    if (filters.query) parts.push(filters.query);
    if (filters.isRead === true) parts.push('is:read');
    if (filters.isRead === false) parts.push('is:unread');
    if (filters.isStarred === true) parts.push('is:starred');
    if (filters.hasAttachments) parts.push('has:attachment');
    if (filters.before) parts.push(`before:${filters.before.toISOString().split('T')[0]}`);
    if (filters.after) parts.push(`after:${filters.after.toISOString().split('T')[0]}`);
    if (filters.labels) {
      filters.labels.forEach(l => parts.push(`label:${l}`));
    }
    if (filters.minTrustLevel !== undefined) {
      parts.push(`trust:>=${filters.minTrustLevel}`);
    }

    return parts.join(' ');
  }

  // Private helper methods

  private buildCacheKey(type: string, options: any): string {
    return `${type}:${JSON.stringify(options)}`;
  }

  private getFromCache(key: string): any | null {
    const cached = this.cache.get(key);
    if (cached && Date.now() - cached.timestamp < this.cacheTimeout) {
      return cached.result;
    }
    return null;
  }

  private setCache(key: string, result: any): void {
    this.cache.set(key, { result, timestamp: Date.now() });

    // Limit cache size
    if (this.cache.size > 100) {
      const oldest = Array.from(this.cache.entries())
        .sort((a, b) => a[1].timestamp - b[1].timestamp)[0];
      this.cache.delete(oldest[0]);
    }
  }

  private clearCache(): void {
    this.cache.clear();
  }

  private serializeFilters(filters: SearchFilters): any {
    return {
      ...filters,
      before: filters.before?.toISOString(),
      after: filters.after?.toISOString(),
      dateRange: filters.dateRange ? {
        from: filters.dateRange.from?.toISOString(),
        to: filters.dateRange.to?.toISOString(),
      } : undefined,
    };
  }

  private processEmailResults(
    items: any[],
    filters: SearchFilters,
    highlightMatches: boolean
  ): EmailSearchResult[] {
    return items.map(item => ({
      ...item,
      createdAt: new Date(item.created_at),
      highlights: highlightMatches && filters.query
        ? this.generateHighlights(filters.query, {
            subject: item.subject,
            body: item.body,
          })
        : undefined,
    }));
  }

  private generateHighlights(
    query: string,
    fields: Record<string, string | undefined>
  ): Record<string, string[]> {
    const highlights: Record<string, string[]> = {};
    const terms = query.toLowerCase().split(/\s+/).filter(Boolean);

    for (const [field, value] of Object.entries(fields)) {
      if (!value) continue;

      const matches: string[] = [];
      const lowerValue = value.toLowerCase();

      for (const term of terms) {
        const index = lowerValue.indexOf(term);
        if (index !== -1) {
          // Extract context around match
          const start = Math.max(0, index - 30);
          const end = Math.min(value.length, index + term.length + 30);
          let snippet = value.slice(start, end);

          // Add ellipsis
          if (start > 0) snippet = '...' + snippet;
          if (end < value.length) snippet = snippet + '...';

          // Highlight term
          const highlightedTerm = `<mark>${value.slice(index, index + term.length)}</mark>`;
          snippet = snippet.replace(
            new RegExp(term, 'gi'),
            highlightedTerm
          );

          matches.push(snippet);
        }
      }

      if (matches.length > 0) {
        highlights[field] = matches;
      }
    }

    return highlights;
  }
}

/**
 * React hook for search
 */
export function useSearch(service: SearchService) {
  return {
    searchEmails: service.searchEmails.bind(service),
    searchContacts: service.searchContacts.bind(service),
    searchAll: service.searchAll.bind(service),
    getSuggestions: service.getSuggestions.bind(service),
    parseQuery: service.parseQuery.bind(service),
    buildQueryString: service.buildQueryString.bind(service),
  };
}

export default SearchService;
