// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Activity Feed Service
 *
 * Generates personalized activity feeds for users showing:
 * - Followed artists' new releases
 * - Friend activity (what they're listening to)
 * - Community highlights
 * - Recommendations based on network
 */

import { Redis } from 'ioredis';
import { Pool } from 'pg';

// ============================================================================
// Types
// ============================================================================

export interface FeedItem {
  id: string;
  type: FeedItemType;
  actorId: string;
  actorName: string;
  actorAvatar?: string;
  targetId?: string;
  targetType?: 'track' | 'album' | 'artist' | 'playlist' | 'circle' | 'user';
  targetName?: string;
  targetImage?: string;
  metadata: Record<string, any>;
  timestamp: Date;
  engagements: {
    likes: number;
    comments: number;
    shares: number;
  };
  userEngagement?: {
    liked: boolean;
    commented: boolean;
    shared: boolean;
  };
}

export type FeedItemType =
  | 'release'           // Artist released new music
  | 'listening'         // User listening to track
  | 'circle_created'    // User created a listening circle
  | 'circle_joined'     // User joined a circle
  | 'follow'            // User followed an artist
  | 'patron'            // User became a patron
  | 'playlist_created'  // User created a playlist
  | 'playlist_updated'  // User updated a playlist
  | 'achievement'       // User earned achievement
  | 'milestone'         // Artist milestone (plays, followers)
  | 'collaboration'     // Artists collaborated
  | 'comment'           // User commented on track
  | 'review'            // User reviewed album
  | 'share'             // User shared content
  | 'nft_minted'        // Artist minted NFT
  | 'live_started';     // Artist started live session

export interface FeedOptions {
  userId: string;
  cursor?: string;
  limit?: number;
  filter?: FeedItemType[];
  includeEngagements?: boolean;
}

export interface FeedResult {
  items: FeedItem[];
  nextCursor?: string;
  hasMore: boolean;
}

// ============================================================================
// Activity Feed Service
// ============================================================================

export class ActivityFeedService {
  private redis: Redis;
  private db: Pool;

  private readonly FEED_TTL = 60 * 60 * 24; // 24 hours
  private readonly MAX_FEED_SIZE = 1000;
  private readonly DEFAULT_LIMIT = 20;

  constructor(
    private readonly config: {
      redisUrl: string;
      databaseUrl: string;
    }
  ) {
    this.redis = new Redis(config.redisUrl);
    this.db = new Pool({ connectionString: config.databaseUrl });
  }

  // ============================================================================
  // Feed Generation
  // ============================================================================

  async getFeed(options: FeedOptions): Promise<FeedResult> {
    const { userId, cursor, limit = this.DEFAULT_LIMIT, filter, includeEngagements = true } = options;

    // Try to get cached feed
    const cacheKey = `feed:${userId}`;
    const cachedFeed = await this.getCachedFeed(cacheKey, cursor, limit, filter);

    if (cachedFeed.items.length > 0) {
      // Enhance with user engagement data if needed
      if (includeEngagements) {
        await this.attachUserEngagements(userId, cachedFeed.items);
      }
      return cachedFeed;
    }

    // Generate fresh feed
    const freshFeed = await this.generateFeed(userId, limit, filter);
    await this.cacheFeed(cacheKey, freshFeed);

    if (includeEngagements) {
      await this.attachUserEngagements(userId, freshFeed);
    }

    return {
      items: freshFeed.slice(0, limit),
      nextCursor: freshFeed.length > limit ? freshFeed[limit - 1].id : undefined,
      hasMore: freshFeed.length > limit,
    };
  }

  private async getCachedFeed(
    cacheKey: string,
    cursor: string | undefined,
    limit: number,
    filter?: FeedItemType[]
  ): Promise<FeedResult> {
    const start = cursor ? await this.redis.zrank(cacheKey, cursor) : 0;
    if (start === null) {
      return { items: [], hasMore: false };
    }

    const rawItems = await this.redis.zrevrange(
      cacheKey,
      start,
      start + limit,
      'WITHSCORES'
    );

    const items: FeedItem[] = [];
    for (let i = 0; i < rawItems.length; i += 2) {
      const item = JSON.parse(rawItems[i]) as FeedItem;
      if (!filter || filter.includes(item.type)) {
        items.push(item);
      }
    }

    return {
      items: items.slice(0, limit),
      nextCursor: items.length > limit ? items[limit - 1].id : undefined,
      hasMore: items.length > limit,
    };
  }

  private async generateFeed(
    userId: string,
    limit: number,
    filter?: FeedItemType[]
  ): Promise<FeedItem[]> {
    // Get user's social graph
    const [following, friends, patrons] = await Promise.all([
      this.getFollowing(userId),
      this.getFriends(userId),
      this.getPatronArtists(userId),
    ]);

    // Build feed from multiple sources
    const feedPromises = [
      this.getArtistActivity(following, limit),
      this.getFriendActivity(friends, limit),
      this.getPatronActivity(patrons, limit),
      this.getCircleActivity(userId, limit),
      this.getCommunityHighlights(userId, limit),
    ];

    const feedSources = await Promise.all(feedPromises);
    let allItems = feedSources.flat();

    // Filter by type if specified
    if (filter && filter.length > 0) {
      allItems = allItems.filter(item => filter.includes(item.type));
    }

    // Sort by timestamp and relevance
    allItems.sort((a, b) => {
      // Combine recency and relevance
      const aScore = this.calculateItemScore(a, userId);
      const bScore = this.calculateItemScore(b, userId);
      return bScore - aScore;
    });

    // Deduplicate
    const seen = new Set<string>();
    const uniqueItems = allItems.filter(item => {
      const key = `${item.type}:${item.actorId}:${item.targetId}`;
      if (seen.has(key)) return false;
      seen.add(key);
      return true;
    });

    return uniqueItems.slice(0, this.MAX_FEED_SIZE);
  }

  // ============================================================================
  // Feed Sources
  // ============================================================================

  private async getArtistActivity(artistIds: string[], limit: number): Promise<FeedItem[]> {
    if (artistIds.length === 0) return [];

    const result = await this.db.query(`
      SELECT
        a.id,
        a.type,
        a.artist_id as actor_id,
        ar.name as actor_name,
        ar.avatar_url as actor_avatar,
        a.target_id,
        a.target_type,
        a.target_name,
        a.target_image,
        a.metadata,
        a.created_at as timestamp
      FROM artist_activities a
      JOIN artists ar ON a.artist_id = ar.id
      WHERE a.artist_id = ANY($1)
        AND a.created_at > NOW() - INTERVAL '7 days'
      ORDER BY a.created_at DESC
      LIMIT $2
    `, [artistIds, limit]);

    return result.rows.map(row => this.mapToFeedItem(row));
  }

  private async getFriendActivity(friendIds: string[], limit: number): Promise<FeedItem[]> {
    if (friendIds.length === 0) return [];

    const result = await this.db.query(`
      SELECT
        ua.id,
        ua.type,
        ua.user_id as actor_id,
        u.display_name as actor_name,
        u.avatar_url as actor_avatar,
        ua.target_id,
        ua.target_type,
        ua.target_name,
        ua.target_image,
        ua.metadata,
        ua.created_at as timestamp
      FROM user_activities ua
      JOIN users u ON ua.user_id = u.id
      WHERE ua.user_id = ANY($1)
        AND ua.is_public = true
        AND ua.created_at > NOW() - INTERVAL '24 hours'
      ORDER BY ua.created_at DESC
      LIMIT $2
    `, [friendIds, limit]);

    return result.rows.map(row => this.mapToFeedItem(row));
  }

  private async getPatronActivity(artistIds: string[], limit: number): Promise<FeedItem[]> {
    if (artistIds.length === 0) return [];

    // Patron-only content from supported artists
    const result = await this.db.query(`
      SELECT
        pc.id,
        'patron_content' as type,
        pc.artist_id as actor_id,
        ar.name as actor_name,
        ar.avatar_url as actor_avatar,
        pc.content_id as target_id,
        pc.content_type as target_type,
        pc.title as target_name,
        pc.thumbnail_url as target_image,
        pc.metadata,
        pc.created_at as timestamp
      FROM patron_content pc
      JOIN artists ar ON pc.artist_id = ar.id
      WHERE pc.artist_id = ANY($1)
        AND pc.created_at > NOW() - INTERVAL '7 days'
      ORDER BY pc.created_at DESC
      LIMIT $2
    `, [artistIds, limit]);

    return result.rows.map(row => this.mapToFeedItem(row));
  }

  private async getCircleActivity(userId: string, limit: number): Promise<FeedItem[]> {
    const result = await this.db.query(`
      SELECT
        ca.id,
        ca.type,
        ca.user_id as actor_id,
        u.display_name as actor_name,
        u.avatar_url as actor_avatar,
        ca.circle_id as target_id,
        'circle' as target_type,
        c.name as target_name,
        c.image_url as target_image,
        ca.metadata,
        ca.created_at as timestamp
      FROM circle_activities ca
      JOIN users u ON ca.user_id = u.id
      JOIN circles c ON ca.circle_id = c.id
      JOIN circle_members cm ON c.id = cm.circle_id
      WHERE cm.user_id = $1
        AND ca.created_at > NOW() - INTERVAL '24 hours'
      ORDER BY ca.created_at DESC
      LIMIT $2
    `, [userId, limit]);

    return result.rows.map(row => this.mapToFeedItem(row));
  }

  private async getCommunityHighlights(userId: string, limit: number): Promise<FeedItem[]> {
    // Get trending content the user might like
    const result = await this.db.query(`
      WITH user_genres AS (
        SELECT unnest(preferred_genres) as genre FROM user_preferences WHERE user_id = $1
      )
      SELECT
        t.id,
        'trending' as type,
        t.artist_id as actor_id,
        ar.name as actor_name,
        ar.avatar_url as actor_avatar,
        t.id as target_id,
        'track' as target_type,
        t.title as target_name,
        t.artwork_url as target_image,
        jsonb_build_object('plays_24h', ts.plays_24h, 'trend', ts.trend) as metadata,
        NOW() as timestamp
      FROM tracks t
      JOIN artists ar ON t.artist_id = ar.id
      JOIN track_stats ts ON t.id = ts.track_id
      WHERE t.primary_genre IN (SELECT genre FROM user_genres)
        AND ts.plays_24h > 1000
        AND ts.trend > 1.5
      ORDER BY ts.plays_24h * ts.trend DESC
      LIMIT $2
    `, [userId, limit / 2]);

    return result.rows.map(row => this.mapToFeedItem(row));
  }

  // ============================================================================
  // Event Publishing
  // ============================================================================

  async publishActivity(activity: {
    type: FeedItemType;
    actorId: string;
    targetId?: string;
    targetType?: string;
    metadata?: Record<string, any>;
  }): Promise<void> {
    // Store activity
    const result = await this.db.query(`
      INSERT INTO activities (type, actor_id, target_id, target_type, metadata, created_at)
      VALUES ($1, $2, $3, $4, $5, NOW())
      RETURNING id
    `, [activity.type, activity.actorId, activity.targetId, activity.targetType, activity.metadata]);

    const activityId = result.rows[0].id;

    // Fan out to followers' feeds
    const followers = await this.getFollowers(activity.actorId);

    // Build feed item
    const feedItem = await this.buildFeedItem(activityId);
    if (!feedItem) return;

    // Add to each follower's feed
    const pipeline = this.redis.pipeline();
    for (const followerId of followers) {
      const cacheKey = `feed:${followerId}`;
      pipeline.zadd(cacheKey, feedItem.timestamp.getTime(), JSON.stringify(feedItem));
      pipeline.zremrangebyrank(cacheKey, 0, -this.MAX_FEED_SIZE - 1); // Trim
      pipeline.expire(cacheKey, this.FEED_TTL);
    }

    await pipeline.exec();

    // Publish real-time notification
    await this.redis.publish('feed:updates', JSON.stringify({
      followers,
      item: feedItem,
    }));
  }

  // ============================================================================
  // Engagements
  // ============================================================================

  async likeItem(userId: string, itemId: string): Promise<void> {
    await this.db.query(`
      INSERT INTO feed_likes (user_id, item_id, created_at)
      VALUES ($1, $2, NOW())
      ON CONFLICT (user_id, item_id) DO NOTHING
    `, [userId, itemId]);

    await this.redis.hincrby(`feed:item:${itemId}`, 'likes', 1);
  }

  async unlikeItem(userId: string, itemId: string): Promise<void> {
    await this.db.query(`
      DELETE FROM feed_likes WHERE user_id = $1 AND item_id = $2
    `, [userId, itemId]);

    await this.redis.hincrby(`feed:item:${itemId}`, 'likes', -1);
  }

  async commentOnItem(
    userId: string,
    itemId: string,
    content: string
  ): Promise<{ id: string }> {
    const result = await this.db.query(`
      INSERT INTO feed_comments (user_id, item_id, content, created_at)
      VALUES ($1, $2, $3, NOW())
      RETURNING id
    `, [userId, itemId, content]);

    await this.redis.hincrby(`feed:item:${itemId}`, 'comments', 1);

    return { id: result.rows[0].id };
  }

  async getComments(itemId: string, limit = 20, offset = 0): Promise<{
    comments: Array<{
      id: string;
      userId: string;
      userName: string;
      userAvatar?: string;
      content: string;
      createdAt: Date;
    }>;
    total: number;
  }> {
    const result = await this.db.query(`
      SELECT
        fc.id,
        fc.user_id,
        u.display_name as user_name,
        u.avatar_url as user_avatar,
        fc.content,
        fc.created_at,
        COUNT(*) OVER() as total
      FROM feed_comments fc
      JOIN users u ON fc.user_id = u.id
      WHERE fc.item_id = $1
      ORDER BY fc.created_at DESC
      LIMIT $2 OFFSET $3
    `, [itemId, limit, offset]);

    return {
      comments: result.rows.map(row => ({
        id: row.id,
        userId: row.user_id,
        userName: row.user_name,
        userAvatar: row.user_avatar,
        content: row.content,
        createdAt: new Date(row.created_at),
      })),
      total: result.rows[0]?.total || 0,
    };
  }

  // ============================================================================
  // Helpers
  // ============================================================================

  private async getFollowing(userId: string): Promise<string[]> {
    const result = await this.db.query(
      'SELECT artist_id FROM follows WHERE user_id = $1',
      [userId]
    );
    return result.rows.map(r => r.artist_id);
  }

  private async getFollowers(actorId: string): Promise<string[]> {
    const result = await this.db.query(
      'SELECT user_id FROM follows WHERE artist_id = $1',
      [actorId]
    );
    return result.rows.map(r => r.user_id);
  }

  private async getFriends(userId: string): Promise<string[]> {
    const result = await this.db.query(
      'SELECT friend_id FROM friendships WHERE user_id = $1 AND status = $2',
      [userId, 'accepted']
    );
    return result.rows.map(r => r.friend_id);
  }

  private async getPatronArtists(userId: string): Promise<string[]> {
    const result = await this.db.query(
      'SELECT artist_id FROM patronages WHERE patron_id = $1 AND status = $2',
      [userId, 'active']
    );
    return result.rows.map(r => r.artist_id);
  }

  private async attachUserEngagements(userId: string, items: FeedItem[]): Promise<void> {
    if (items.length === 0) return;

    const itemIds = items.map(i => i.id);

    const [likes, comments] = await Promise.all([
      this.db.query(
        'SELECT item_id FROM feed_likes WHERE user_id = $1 AND item_id = ANY($2)',
        [userId, itemIds]
      ),
      this.db.query(
        'SELECT DISTINCT item_id FROM feed_comments WHERE user_id = $1 AND item_id = ANY($2)',
        [userId, itemIds]
      ),
    ]);

    const likedSet = new Set(likes.rows.map(r => r.item_id));
    const commentedSet = new Set(comments.rows.map(r => r.item_id));

    for (const item of items) {
      item.userEngagement = {
        liked: likedSet.has(item.id),
        commented: commentedSet.has(item.id),
        shared: false, // Would check shares table
      };
    }
  }

  private async cacheFeed(cacheKey: string, items: FeedItem[]): Promise<void> {
    if (items.length === 0) return;

    const pipeline = this.redis.pipeline();
    for (const item of items) {
      pipeline.zadd(cacheKey, item.timestamp.getTime(), JSON.stringify(item));
    }
    pipeline.expire(cacheKey, this.FEED_TTL);
    await pipeline.exec();
  }

  private async buildFeedItem(activityId: string): Promise<FeedItem | null> {
    const result = await this.db.query(`
      SELECT
        a.id,
        a.type,
        a.actor_id,
        COALESCE(ar.name, u.display_name) as actor_name,
        COALESCE(ar.avatar_url, u.avatar_url) as actor_avatar,
        a.target_id,
        a.target_type,
        a.metadata,
        a.created_at as timestamp
      FROM activities a
      LEFT JOIN artists ar ON a.actor_id = ar.id
      LEFT JOIN users u ON a.actor_id = u.id
      WHERE a.id = $1
    `, [activityId]);

    if (result.rows.length === 0) return null;

    return this.mapToFeedItem(result.rows[0]);
  }

  private mapToFeedItem(row: any): FeedItem {
    return {
      id: row.id,
      type: row.type,
      actorId: row.actor_id,
      actorName: row.actor_name,
      actorAvatar: row.actor_avatar,
      targetId: row.target_id,
      targetType: row.target_type,
      targetName: row.target_name,
      targetImage: row.target_image,
      metadata: row.metadata || {},
      timestamp: new Date(row.timestamp),
      engagements: {
        likes: 0,
        comments: 0,
        shares: 0,
      },
    };
  }

  private calculateItemScore(item: FeedItem, userId: string): number {
    const now = Date.now();
    const age = now - item.timestamp.getTime();
    const ageHours = age / (1000 * 60 * 60);

    // Base recency score (decays over time)
    let score = Math.pow(0.95, ageHours);

    // Boost for different item types
    const typeBoosts: Record<FeedItemType, number> = {
      release: 2.0,
      live_started: 3.0,
      milestone: 1.5,
      collaboration: 1.8,
      nft_minted: 1.3,
      circle_created: 1.2,
      listening: 0.5,
      follow: 0.3,
      patron: 1.0,
      playlist_created: 0.8,
      playlist_updated: 0.4,
      achievement: 0.7,
      circle_joined: 0.6,
      comment: 0.4,
      review: 1.0,
      share: 0.5,
    };

    score *= typeBoosts[item.type] || 1.0;

    // Boost for engagement
    score *= 1 + Math.log10(1 + item.engagements.likes + item.engagements.comments);

    return score;
  }
}

export default ActivityFeedService;
