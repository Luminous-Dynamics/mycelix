// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Public API & Webhooks Service
 *
 * Public API for third-party integrations with OAuth, rate limiting,
 * webhooks, and SDK generation.
 */

import { EventEmitter } from 'events';
import crypto from 'crypto';

// ============================================================================
// Types
// ============================================================================

export interface APIApplication {
  id: string;
  name: string;
  description: string;
  ownerId: string;
  clientId: string;
  clientSecret: string;
  redirectUris: string[];
  scopes: APIScope[];
  rateLimit: RateLimitConfig;
  webhooks: WebhookConfig[];
  isApproved: boolean;
  createdAt: Date;
  stats: ApplicationStats;
}

export interface RateLimitConfig {
  requestsPerMinute: number;
  requestsPerDay: number;
  burstLimit: number;
}

export interface ApplicationStats {
  totalRequests: number;
  requestsToday: number;
  activeUsers: number;
  webhookDeliveries: number;
  errorRate: number;
}

export type APIScope =
  | 'user:read'
  | 'user:write'
  | 'library:read'
  | 'library:write'
  | 'playback:read'
  | 'playback:write'
  | 'playlists:read'
  | 'playlists:write'
  | 'social:read'
  | 'social:write'
  | 'analytics:read'
  | 'webhooks:manage';

export interface OAuthToken {
  accessToken: string;
  refreshToken: string;
  tokenType: 'Bearer';
  expiresIn: number;
  expiresAt: Date;
  scopes: APIScope[];
  userId: string;
  applicationId: string;
}

export interface WebhookConfig {
  id: string;
  url: string;
  events: WebhookEventType[];
  secret: string;
  isActive: boolean;
  failureCount: number;
  lastDelivery?: Date;
}

export type WebhookEventType =
  | 'track.played'
  | 'track.liked'
  | 'track.unliked'
  | 'playlist.created'
  | 'playlist.updated'
  | 'playlist.deleted'
  | 'user.followed'
  | 'user.unfollowed'
  | 'playback.started'
  | 'playback.paused'
  | 'playback.ended'
  | 'circle.joined'
  | 'circle.left'
  | 'achievement.unlocked';

export interface WebhookPayload {
  id: string;
  event: WebhookEventType;
  timestamp: string;
  data: Record<string, any>;
  userId?: string;
}

export interface APIRequest {
  method: string;
  path: string;
  headers: Record<string, string>;
  query: Record<string, string>;
  body?: any;
  applicationId: string;
  userId?: string;
  scopes: APIScope[];
}

export interface APIResponse {
  status: number;
  body: any;
  headers?: Record<string, string>;
}

export interface RateLimitStatus {
  remaining: number;
  limit: number;
  reset: Date;
  retryAfter?: number;
}

// ============================================================================
// Rate Limiter
// ============================================================================

class RateLimiter {
  private requests: Map<string, { count: number; resetAt: number }[]> = new Map();

  check(
    applicationId: string,
    config: RateLimitConfig
  ): { allowed: boolean; status: RateLimitStatus } {
    const now = Date.now();
    const minuteAgo = now - 60000;
    const dayAgo = now - 86400000;

    let entries = this.requests.get(applicationId) || [];

    // Clean old entries
    entries = entries.filter(e => e.resetAt > dayAgo);

    const minuteRequests = entries.filter(e => e.resetAt > minuteAgo).reduce((sum, e) => sum + e.count, 0);
    const dayRequests = entries.reduce((sum, e) => sum + e.count, 0);

    if (minuteRequests >= config.requestsPerMinute) {
      return {
        allowed: false,
        status: {
          remaining: 0,
          limit: config.requestsPerMinute,
          reset: new Date(now + 60000),
          retryAfter: 60,
        },
      };
    }

    if (dayRequests >= config.requestsPerDay) {
      const oldestEntry = entries[0];
      return {
        allowed: false,
        status: {
          remaining: 0,
          limit: config.requestsPerDay,
          reset: new Date(oldestEntry?.resetAt || now + 86400000),
          retryAfter: Math.ceil((86400000 - (now - dayAgo)) / 1000),
        },
      };
    }

    // Add request
    entries.push({ count: 1, resetAt: now + 86400000 });
    this.requests.set(applicationId, entries);

    return {
      allowed: true,
      status: {
        remaining: config.requestsPerMinute - minuteRequests - 1,
        limit: config.requestsPerMinute,
        reset: new Date(now + 60000),
      },
    };
  }
}

// ============================================================================
// Public API Service
// ============================================================================

class PublicAPIService extends EventEmitter {
  private applications: Map<string, APIApplication> = new Map();
  private tokens: Map<string, OAuthToken> = new Map();
  private authCodes: Map<string, { applicationId: string; userId: string; scopes: APIScope[]; expiresAt: Date }> = new Map();
  private rateLimiter = new RateLimiter();
  private webhookQueue: WebhookPayload[] = [];
  private isProcessingWebhooks = false;

  // ============================================================================
  // Application Management
  // ============================================================================

  async createApplication(
    ownerId: string,
    data: {
      name: string;
      description: string;
      redirectUris: string[];
      scopes: APIScope[];
    }
  ): Promise<APIApplication> {
    const applicationId = `app_${crypto.randomBytes(16).toString('hex')}`;
    const clientId = `client_${crypto.randomBytes(16).toString('hex')}`;
    const clientSecret = `secret_${crypto.randomBytes(32).toString('hex')}`;

    const application: APIApplication = {
      id: applicationId,
      name: data.name,
      description: data.description,
      ownerId,
      clientId,
      clientSecret,
      redirectUris: data.redirectUris,
      scopes: data.scopes,
      rateLimit: {
        requestsPerMinute: 60,
        requestsPerDay: 10000,
        burstLimit: 10,
      },
      webhooks: [],
      isApproved: false,
      createdAt: new Date(),
      stats: {
        totalRequests: 0,
        requestsToday: 0,
        activeUsers: 0,
        webhookDeliveries: 0,
        errorRate: 0,
      },
    };

    this.applications.set(applicationId, application);

    return application;
  }

  async getApplication(applicationId: string): Promise<APIApplication | null> {
    return this.applications.get(applicationId) || null;
  }

  async updateApplication(
    applicationId: string,
    updates: Partial<Pick<APIApplication, 'name' | 'description' | 'redirectUris' | 'scopes'>>
  ): Promise<APIApplication | null> {
    const app = this.applications.get(applicationId);
    if (!app) return null;

    Object.assign(app, updates);
    return app;
  }

  async regenerateClientSecret(applicationId: string): Promise<string> {
    const app = this.applications.get(applicationId);
    if (!app) throw new Error('Application not found');

    app.clientSecret = `secret_${crypto.randomBytes(32).toString('hex')}`;

    // Invalidate all existing tokens
    for (const [tokenId, token] of this.tokens) {
      if (token.applicationId === applicationId) {
        this.tokens.delete(tokenId);
      }
    }

    return app.clientSecret;
  }

  // ============================================================================
  // OAuth 2.0
  // ============================================================================

  generateAuthorizationUrl(
    clientId: string,
    redirectUri: string,
    scopes: APIScope[],
    state: string
  ): string {
    const app = Array.from(this.applications.values()).find(a => a.clientId === clientId);
    if (!app) throw new Error('Invalid client_id');

    if (!app.redirectUris.includes(redirectUri)) {
      throw new Error('Invalid redirect_uri');
    }

    const invalidScopes = scopes.filter(s => !app.scopes.includes(s));
    if (invalidScopes.length > 0) {
      throw new Error(`Invalid scopes: ${invalidScopes.join(', ')}`);
    }

    const params = new URLSearchParams({
      client_id: clientId,
      redirect_uri: redirectUri,
      scope: scopes.join(' '),
      state,
      response_type: 'code',
    });

    return `https://mycelix.io/oauth/authorize?${params.toString()}`;
  }

  async authorizeUser(
    clientId: string,
    userId: string,
    scopes: APIScope[]
  ): Promise<string> {
    const app = Array.from(this.applications.values()).find(a => a.clientId === clientId);
    if (!app) throw new Error('Invalid client_id');

    const code = crypto.randomBytes(32).toString('hex');

    this.authCodes.set(code, {
      applicationId: app.id,
      userId,
      scopes,
      expiresAt: new Date(Date.now() + 600000), // 10 minutes
    });

    return code;
  }

  async exchangeAuthorizationCode(
    clientId: string,
    clientSecret: string,
    code: string,
    redirectUri: string
  ): Promise<OAuthToken> {
    const app = Array.from(this.applications.values()).find(
      a => a.clientId === clientId && a.clientSecret === clientSecret
    );
    if (!app) throw new Error('Invalid client credentials');

    const authCode = this.authCodes.get(code);
    if (!authCode) throw new Error('Invalid authorization code');

    if (new Date() > authCode.expiresAt) {
      this.authCodes.delete(code);
      throw new Error('Authorization code expired');
    }

    if (authCode.applicationId !== app.id) {
      throw new Error('Authorization code does not match application');
    }

    this.authCodes.delete(code);

    const token = this.generateToken(app.id, authCode.userId, authCode.scopes);

    return token;
  }

  async refreshAccessToken(
    clientId: string,
    clientSecret: string,
    refreshToken: string
  ): Promise<OAuthToken> {
    const app = Array.from(this.applications.values()).find(
      a => a.clientId === clientId && a.clientSecret === clientSecret
    );
    if (!app) throw new Error('Invalid client credentials');

    const existingToken = Array.from(this.tokens.values()).find(
      t => t.refreshToken === refreshToken && t.applicationId === app.id
    );
    if (!existingToken) throw new Error('Invalid refresh token');

    // Delete old token
    for (const [key, token] of this.tokens) {
      if (token.refreshToken === refreshToken) {
        this.tokens.delete(key);
        break;
      }
    }

    return this.generateToken(app.id, existingToken.userId, existingToken.scopes);
  }

  private generateToken(applicationId: string, userId: string, scopes: APIScope[]): OAuthToken {
    const accessToken = `at_${crypto.randomBytes(32).toString('hex')}`;
    const refreshToken = `rt_${crypto.randomBytes(32).toString('hex')}`;

    const token: OAuthToken = {
      accessToken,
      refreshToken,
      tokenType: 'Bearer',
      expiresIn: 3600,
      expiresAt: new Date(Date.now() + 3600000),
      scopes,
      userId,
      applicationId,
    };

    this.tokens.set(accessToken, token);

    return token;
  }

  async validateToken(accessToken: string): Promise<OAuthToken | null> {
    const token = this.tokens.get(accessToken);

    if (!token) return null;

    if (new Date() > token.expiresAt) {
      this.tokens.delete(accessToken);
      return null;
    }

    return token;
  }

  async revokeToken(accessToken: string): Promise<void> {
    const token = this.tokens.get(accessToken);
    if (!token) return;

    // Delete both access and refresh tokens
    for (const [key, t] of this.tokens) {
      if (t.refreshToken === token.refreshToken || t.accessToken === accessToken) {
        this.tokens.delete(key);
      }
    }
  }

  // ============================================================================
  // Request Handling
  // ============================================================================

  async handleRequest(request: APIRequest): Promise<APIResponse> {
    const app = this.applications.get(request.applicationId);
    if (!app) {
      return { status: 401, body: { error: 'Invalid application' } };
    }

    // Check rate limit
    const { allowed, status } = this.rateLimiter.check(app.id, app.rateLimit);

    if (!allowed) {
      return {
        status: 429,
        body: { error: 'Rate limit exceeded', retryAfter: status.retryAfter },
        headers: {
          'X-RateLimit-Remaining': '0',
          'X-RateLimit-Limit': status.limit.toString(),
          'X-RateLimit-Reset': status.reset.toISOString(),
          'Retry-After': status.retryAfter?.toString() || '60',
        },
      };
    }

    // Update stats
    app.stats.totalRequests++;
    app.stats.requestsToday++;

    // Check scope permissions
    const requiredScope = this.getRequiredScope(request.method, request.path);
    if (requiredScope && !request.scopes.includes(requiredScope)) {
      return {
        status: 403,
        body: { error: 'Insufficient scope', required: requiredScope },
      };
    }

    // Route request
    const response = await this.routeRequest(request);

    return {
      ...response,
      headers: {
        ...response.headers,
        'X-RateLimit-Remaining': status.remaining.toString(),
        'X-RateLimit-Limit': status.limit.toString(),
        'X-RateLimit-Reset': status.reset.toISOString(),
      },
    };
  }

  private getRequiredScope(method: string, path: string): APIScope | null {
    const scopeMap: Record<string, APIScope> = {
      'GET /me': 'user:read',
      'PATCH /me': 'user:write',
      'GET /me/library': 'library:read',
      'POST /me/library': 'library:write',
      'DELETE /me/library': 'library:write',
      'GET /me/playlists': 'playlists:read',
      'POST /playlists': 'playlists:write',
      'PATCH /playlists': 'playlists:write',
      'DELETE /playlists': 'playlists:write',
      'GET /me/player': 'playback:read',
      'PUT /me/player': 'playback:write',
      'POST /me/player': 'playback:write',
      'GET /me/following': 'social:read',
      'POST /me/following': 'social:write',
      'GET /me/analytics': 'analytics:read',
    };

    const key = `${method} ${path.split('/').slice(0, 3).join('/')}`;
    return scopeMap[key] || null;
  }

  private async routeRequest(request: APIRequest): Promise<APIResponse> {
    // Would route to actual API handlers
    // For now, return mock responses

    const { method, path } = request;

    if (path === '/me' && method === 'GET') {
      return {
        status: 200,
        body: {
          id: request.userId,
          username: 'user',
          displayName: 'User',
          profileUrl: `https://mycelix.io/user/${request.userId}`,
        },
      };
    }

    if (path.startsWith('/tracks/') && method === 'GET') {
      const trackId = path.split('/')[2];
      return {
        status: 200,
        body: {
          id: trackId,
          title: 'Example Track',
          artist: 'Example Artist',
          duration: 180000,
        },
      };
    }

    return {
      status: 404,
      body: { error: 'Not found' },
    };
  }

  // ============================================================================
  // Webhooks
  // ============================================================================

  async registerWebhook(
    applicationId: string,
    url: string,
    events: WebhookEventType[]
  ): Promise<WebhookConfig> {
    const app = this.applications.get(applicationId);
    if (!app) throw new Error('Application not found');

    const webhookId = `wh_${crypto.randomBytes(16).toString('hex')}`;
    const secret = crypto.randomBytes(32).toString('hex');

    const webhook: WebhookConfig = {
      id: webhookId,
      url,
      events,
      secret,
      isActive: true,
      failureCount: 0,
    };

    app.webhooks.push(webhook);

    return webhook;
  }

  async updateWebhook(
    applicationId: string,
    webhookId: string,
    updates: Partial<Pick<WebhookConfig, 'url' | 'events' | 'isActive'>>
  ): Promise<WebhookConfig | null> {
    const app = this.applications.get(applicationId);
    if (!app) return null;

    const webhook = app.webhooks.find(w => w.id === webhookId);
    if (!webhook) return null;

    Object.assign(webhook, updates);

    return webhook;
  }

  async deleteWebhook(applicationId: string, webhookId: string): Promise<boolean> {
    const app = this.applications.get(applicationId);
    if (!app) return false;

    const index = app.webhooks.findIndex(w => w.id === webhookId);
    if (index === -1) return false;

    app.webhooks.splice(index, 1);

    return true;
  }

  async emitWebhookEvent(
    event: WebhookEventType,
    data: Record<string, any>,
    userId?: string
  ): Promise<void> {
    const payload: WebhookPayload = {
      id: `evt_${crypto.randomBytes(16).toString('hex')}`,
      event,
      timestamp: new Date().toISOString(),
      data,
      userId,
    };

    this.webhookQueue.push(payload);

    if (!this.isProcessingWebhooks) {
      this.processWebhookQueue();
    }
  }

  private async processWebhookQueue(): Promise<void> {
    if (this.isProcessingWebhooks) return;

    this.isProcessingWebhooks = true;

    while (this.webhookQueue.length > 0) {
      const payload = this.webhookQueue.shift()!;

      // Find all webhooks subscribed to this event
      for (const app of this.applications.values()) {
        for (const webhook of app.webhooks) {
          if (!webhook.isActive) continue;
          if (!webhook.events.includes(payload.event)) continue;

          await this.deliverWebhook(webhook, payload, app);
        }
      }
    }

    this.isProcessingWebhooks = false;
  }

  private async deliverWebhook(
    webhook: WebhookConfig,
    payload: WebhookPayload,
    app: APIApplication
  ): Promise<void> {
    const body = JSON.stringify(payload);
    const signature = crypto
      .createHmac('sha256', webhook.secret)
      .update(body)
      .digest('hex');

    try {
      const response = await fetch(webhook.url, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Mycelix-Signature': `sha256=${signature}`,
          'X-Mycelix-Event': payload.event,
          'X-Mycelix-Delivery': payload.id,
        },
        body,
      });

      if (response.ok) {
        webhook.failureCount = 0;
        webhook.lastDelivery = new Date();
        app.stats.webhookDeliveries++;
      } else {
        webhook.failureCount++;

        // Disable webhook after 10 consecutive failures
        if (webhook.failureCount >= 10) {
          webhook.isActive = false;
        }
      }
    } catch (error) {
      webhook.failureCount++;

      if (webhook.failureCount >= 10) {
        webhook.isActive = false;
      }
    }
  }

  // ============================================================================
  // API Documentation
  // ============================================================================

  generateOpenAPISpec(): object {
    return {
      openapi: '3.0.3',
      info: {
        title: 'Mycelix Public API',
        version: '1.0.0',
        description: 'API for third-party integrations with Mycelix',
        termsOfService: 'https://mycelix.io/api/terms',
        contact: {
          email: 'api@mycelix.io',
        },
      },
      servers: [
        { url: 'https://api.mycelix.io/v1' },
      ],
      security: [
        { oauth2: [] },
      ],
      paths: {
        '/me': {
          get: {
            summary: 'Get current user',
            operationId: 'getCurrentUser',
            tags: ['Users'],
            security: [{ oauth2: ['user:read'] }],
            responses: {
              '200': {
                description: 'Current user',
                content: {
                  'application/json': {
                    schema: { $ref: '#/components/schemas/User' },
                  },
                },
              },
            },
          },
        },
        '/me/player': {
          get: {
            summary: 'Get playback state',
            operationId: 'getPlaybackState',
            tags: ['Player'],
            security: [{ oauth2: ['playback:read'] }],
            responses: {
              '200': {
                description: 'Current playback state',
                content: {
                  'application/json': {
                    schema: { $ref: '#/components/schemas/PlaybackState' },
                  },
                },
              },
            },
          },
          put: {
            summary: 'Control playback',
            operationId: 'controlPlayback',
            tags: ['Player'],
            security: [{ oauth2: ['playback:write'] }],
            requestBody: {
              content: {
                'application/json': {
                  schema: { $ref: '#/components/schemas/PlaybackControl' },
                },
              },
            },
            responses: {
              '204': { description: 'Success' },
            },
          },
        },
        '/tracks/{id}': {
          get: {
            summary: 'Get track',
            operationId: 'getTrack',
            tags: ['Tracks'],
            parameters: [
              { name: 'id', in: 'path', required: true, schema: { type: 'string' } },
            ],
            responses: {
              '200': {
                description: 'Track details',
                content: {
                  'application/json': {
                    schema: { $ref: '#/components/schemas/Track' },
                  },
                },
              },
            },
          },
        },
        '/playlists': {
          post: {
            summary: 'Create playlist',
            operationId: 'createPlaylist',
            tags: ['Playlists'],
            security: [{ oauth2: ['playlists:write'] }],
            requestBody: {
              content: {
                'application/json': {
                  schema: { $ref: '#/components/schemas/CreatePlaylist' },
                },
              },
            },
            responses: {
              '201': {
                description: 'Created playlist',
                content: {
                  'application/json': {
                    schema: { $ref: '#/components/schemas/Playlist' },
                  },
                },
              },
            },
          },
        },
        '/search': {
          get: {
            summary: 'Search tracks, artists, albums',
            operationId: 'search',
            tags: ['Search'],
            parameters: [
              { name: 'q', in: 'query', required: true, schema: { type: 'string' } },
              { name: 'type', in: 'query', schema: { type: 'string', enum: ['track', 'artist', 'album', 'playlist'] } },
              { name: 'limit', in: 'query', schema: { type: 'integer', default: 20 } },
              { name: 'offset', in: 'query', schema: { type: 'integer', default: 0 } },
            ],
            responses: {
              '200': {
                description: 'Search results',
                content: {
                  'application/json': {
                    schema: { $ref: '#/components/schemas/SearchResults' },
                  },
                },
              },
            },
          },
        },
      },
      components: {
        securitySchemes: {
          oauth2: {
            type: 'oauth2',
            flows: {
              authorizationCode: {
                authorizationUrl: 'https://mycelix.io/oauth/authorize',
                tokenUrl: 'https://api.mycelix.io/oauth/token',
                scopes: {
                  'user:read': 'Read user profile',
                  'user:write': 'Update user profile',
                  'library:read': 'Read user library',
                  'library:write': 'Modify user library',
                  'playback:read': 'Read playback state',
                  'playback:write': 'Control playback',
                  'playlists:read': 'Read playlists',
                  'playlists:write': 'Create and modify playlists',
                  'social:read': 'Read social data',
                  'social:write': 'Follow/unfollow users',
                },
              },
            },
          },
        },
        schemas: {
          User: {
            type: 'object',
            properties: {
              id: { type: 'string' },
              username: { type: 'string' },
              displayName: { type: 'string' },
              avatarUrl: { type: 'string' },
              followers: { type: 'integer' },
              following: { type: 'integer' },
            },
          },
          Track: {
            type: 'object',
            properties: {
              id: { type: 'string' },
              title: { type: 'string' },
              artists: { type: 'array', items: { $ref: '#/components/schemas/Artist' } },
              album: { $ref: '#/components/schemas/Album' },
              duration: { type: 'integer' },
              isrc: { type: 'string' },
            },
          },
          Artist: {
            type: 'object',
            properties: {
              id: { type: 'string' },
              name: { type: 'string' },
              imageUrl: { type: 'string' },
            },
          },
          Album: {
            type: 'object',
            properties: {
              id: { type: 'string' },
              name: { type: 'string' },
              artworkUrl: { type: 'string' },
              releaseDate: { type: 'string' },
            },
          },
          Playlist: {
            type: 'object',
            properties: {
              id: { type: 'string' },
              name: { type: 'string' },
              description: { type: 'string' },
              imageUrl: { type: 'string' },
              trackCount: { type: 'integer' },
              isPublic: { type: 'boolean' },
            },
          },
          PlaybackState: {
            type: 'object',
            properties: {
              isPlaying: { type: 'boolean' },
              track: { $ref: '#/components/schemas/Track' },
              position: { type: 'integer' },
              shuffle: { type: 'boolean' },
              repeat: { type: 'string', enum: ['off', 'track', 'context'] },
            },
          },
          PlaybackControl: {
            type: 'object',
            properties: {
              action: { type: 'string', enum: ['play', 'pause', 'next', 'previous', 'seek'] },
              position: { type: 'integer' },
              trackId: { type: 'string' },
            },
          },
          CreatePlaylist: {
            type: 'object',
            required: ['name'],
            properties: {
              name: { type: 'string' },
              description: { type: 'string' },
              isPublic: { type: 'boolean', default: true },
            },
          },
          SearchResults: {
            type: 'object',
            properties: {
              tracks: { type: 'array', items: { $ref: '#/components/schemas/Track' } },
              artists: { type: 'array', items: { $ref: '#/components/schemas/Artist' } },
              albums: { type: 'array', items: { $ref: '#/components/schemas/Album' } },
              playlists: { type: 'array', items: { $ref: '#/components/schemas/Playlist' } },
            },
          },
        },
      },
    };
  }

  generateTypeScriptSDK(): string {
    return `
/**
 * Mycelix API TypeScript SDK
 * Auto-generated from OpenAPI spec
 */

export interface MycelixConfig {
  clientId: string;
  clientSecret?: string;
  accessToken?: string;
  baseUrl?: string;
}

export interface User {
  id: string;
  username: string;
  displayName: string;
  avatarUrl?: string;
  followers: number;
  following: number;
}

export interface Track {
  id: string;
  title: string;
  artists: Artist[];
  album: Album;
  duration: number;
  isrc?: string;
}

export interface Artist {
  id: string;
  name: string;
  imageUrl?: string;
}

export interface Album {
  id: string;
  name: string;
  artworkUrl?: string;
  releaseDate: string;
}

export interface Playlist {
  id: string;
  name: string;
  description?: string;
  imageUrl?: string;
  trackCount: number;
  isPublic: boolean;
}

export interface PlaybackState {
  isPlaying: boolean;
  track: Track | null;
  position: number;
  shuffle: boolean;
  repeat: 'off' | 'track' | 'context';
}

export class MycelixClient {
  private config: MycelixConfig;
  private baseUrl: string;

  constructor(config: MycelixConfig) {
    this.config = config;
    this.baseUrl = config.baseUrl || 'https://api.mycelix.io/v1';
  }

  private async request<T>(
    method: string,
    path: string,
    body?: any
  ): Promise<T> {
    const response = await fetch(\`\${this.baseUrl}\${path}\`, {
      method,
      headers: {
        'Authorization': \`Bearer \${this.config.accessToken}\`,
        'Content-Type': 'application/json',
      },
      body: body ? JSON.stringify(body) : undefined,
    });

    if (!response.ok) {
      throw new Error(\`API error: \${response.status}\`);
    }

    return response.json();
  }

  // Users
  async getCurrentUser(): Promise<User> {
    return this.request<User>('GET', '/me');
  }

  // Playback
  async getPlaybackState(): Promise<PlaybackState> {
    return this.request<PlaybackState>('GET', '/me/player');
  }

  async play(trackId?: string): Promise<void> {
    await this.request('PUT', '/me/player', { action: 'play', trackId });
  }

  async pause(): Promise<void> {
    await this.request('PUT', '/me/player', { action: 'pause' });
  }

  async next(): Promise<void> {
    await this.request('PUT', '/me/player', { action: 'next' });
  }

  async previous(): Promise<void> {
    await this.request('PUT', '/me/player', { action: 'previous' });
  }

  async seek(position: number): Promise<void> {
    await this.request('PUT', '/me/player', { action: 'seek', position });
  }

  // Tracks
  async getTrack(trackId: string): Promise<Track> {
    return this.request<Track>('GET', \`/tracks/\${trackId}\`);
  }

  // Playlists
  async createPlaylist(name: string, options?: {
    description?: string;
    isPublic?: boolean;
  }): Promise<Playlist> {
    return this.request<Playlist>('POST', '/playlists', { name, ...options });
  }

  async addTracksToPlaylist(playlistId: string, trackIds: string[]): Promise<void> {
    await this.request('POST', \`/playlists/\${playlistId}/tracks\`, { trackIds });
  }

  // Search
  async search(query: string, options?: {
    type?: ('track' | 'artist' | 'album' | 'playlist')[];
    limit?: number;
    offset?: number;
  }): Promise<{
    tracks?: Track[];
    artists?: Artist[];
    albums?: Album[];
    playlists?: Playlist[];
  }> {
    const params = new URLSearchParams({ q: query });
    if (options?.type) params.set('type', options.type.join(','));
    if (options?.limit) params.set('limit', options.limit.toString());
    if (options?.offset) params.set('offset', options.offset.toString());

    return this.request('GET', \`/search?\${params.toString()}\`);
  }
}

export default MycelixClient;
`.trim();
  }
}

export const publicAPI = new PublicAPIService();
export default publicAPI;
