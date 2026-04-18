// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Social Service
 *
 * Social features: follows, activity feeds, playlists,
 * comments, likes, shares, and notifications.
 */

import { Pool } from 'pg';
import { EventEmitter } from '../events';
import { getLogger } from '../logging';
import { getMetrics } from '../metrics';

const logger = getLogger();

// ==================== Types ====================

export interface Follow {
  id: string;
  followerAddress: string;
  followedAddress: string;
  followedType: 'user' | 'artist';
  createdAt: Date;
}

export interface ActivityItem {
  id: string;
  type: ActivityType;
  actorAddress: string;
  actorName: string;
  actorAvatar?: string;
  targetId: string;
  targetType: string;
  targetTitle: string;
  targetImage?: string;
  metadata?: Record<string, unknown>;
  createdAt: Date;
}

export type ActivityType =
  | 'follow'
  | 'like_song'
  | 'like_playlist'
  | 'add_to_playlist'
  | 'create_playlist'
  | 'share_song'
  | 'share_playlist'
  | 'comment'
  | 'reply'
  | 'listen'
  | 'release'
  | 'milestone';

export interface Playlist {
  id: string;
  ownerAddress: string;
  ownerName: string;
  name: string;
  description?: string;
  coverImage?: string;
  isPublic: boolean;
  isCollaborative: boolean;
  songCount: number;
  duration: number;
  followers: number;
  createdAt: Date;
  updatedAt: Date;
}

export interface PlaylistSong {
  id: string;
  playlistId: string;
  songId: string;
  addedBy: string;
  position: number;
  addedAt: Date;
  song?: {
    id: string;
    title: string;
    artist: string;
    artistAddress: string;
    duration: number;
    coverArt: string;
  };
}

export interface Comment {
  id: string;
  authorAddress: string;
  authorName: string;
  authorAvatar?: string;
  targetId: string;
  targetType: 'song' | 'playlist' | 'profile';
  parentId?: string;
  content: string;
  likes: number;
  replyCount: number;
  createdAt: Date;
  updatedAt: Date;
}

export interface Like {
  id: string;
  userAddress: string;
  targetId: string;
  targetType: 'song' | 'playlist' | 'comment';
  createdAt: Date;
}

export interface Share {
  id: string;
  userAddress: string;
  targetId: string;
  targetType: 'song' | 'playlist';
  platform?: string;
  createdAt: Date;
}

export interface Notification {
  id: string;
  recipientAddress: string;
  type: NotificationType;
  actorAddress?: string;
  actorName?: string;
  actorAvatar?: string;
  title: string;
  body: string;
  targetId?: string;
  targetType?: string;
  read: boolean;
  createdAt: Date;
}

export type NotificationType =
  | 'new_follower'
  | 'like'
  | 'comment'
  | 'reply'
  | 'mention'
  | 'playlist_add'
  | 'new_release'
  | 'milestone'
  | 'payout'
  | 'system';

// ==================== Social Service ====================

export class SocialService {
  constructor(
    private pool: Pool,
    private events: EventEmitter
  ) {
    const metrics = getMetrics();
    metrics.createCounter('social_actions_total', 'Social actions', ['type']);
    metrics.createGauge('social_active_users', 'Active social users', []);
  }

  // ==================== Follows ====================

  /**
   * Follow a user or artist
   */
  async follow(
    followerAddress: string,
    followedAddress: string,
    followedType: 'user' | 'artist' = 'user'
  ): Promise<Follow> {
    // Check if already following
    const existing = await this.pool.query(
      `SELECT id FROM follows WHERE follower_address = $1 AND followed_address = $2`,
      [followerAddress, followedAddress]
    );

    if (existing.rows.length > 0) {
      throw new Error('Already following');
    }

    const result = await this.pool.query(
      `
      INSERT INTO follows (follower_address, followed_address, followed_type)
      VALUES ($1, $2, $3)
      RETURNING *
      `,
      [followerAddress, followedAddress, followedType]
    );

    const follow = this.mapFollow(result.rows[0]);

    // Create activity
    await this.createActivity({
      type: 'follow',
      actorAddress: followerAddress,
      targetId: followedAddress,
      targetType: followedType,
    });

    // Create notification
    await this.createNotification({
      recipientAddress: followedAddress,
      type: 'new_follower',
      actorAddress: followerAddress,
      title: 'New follower',
      body: 'Someone started following you',
    });

    // Emit event
    this.events.emit('social:follow', { followerAddress, followedAddress });

    getMetrics().incCounter('social_actions_total', { type: 'follow' });

    return follow;
  }

  /**
   * Unfollow
   */
  async unfollow(
    followerAddress: string,
    followedAddress: string
  ): Promise<void> {
    await this.pool.query(
      `DELETE FROM follows WHERE follower_address = $1 AND followed_address = $2`,
      [followerAddress, followedAddress]
    );

    this.events.emit('social:unfollow', { followerAddress, followedAddress });
  }

  /**
   * Get followers
   */
  async getFollowers(
    address: string,
    limit = 20,
    offset = 0
  ): Promise<{ followers: Follow[]; total: number }> {
    const [result, countResult] = await Promise.all([
      this.pool.query(
        `
        SELECT f.*, u.display_name, u.avatar_url
        FROM follows f
        LEFT JOIN user_profiles u ON f.follower_address = u.wallet_address
        WHERE f.followed_address = $1
        ORDER BY f.created_at DESC
        LIMIT $2 OFFSET $3
        `,
        [address, limit, offset]
      ),
      this.pool.query(
        `SELECT COUNT(*) FROM follows WHERE followed_address = $1`,
        [address]
      ),
    ]);

    return {
      followers: result.rows.map(this.mapFollow),
      total: parseInt(countResult.rows[0].count),
    };
  }

  /**
   * Get following
   */
  async getFollowing(
    address: string,
    limit = 20,
    offset = 0
  ): Promise<{ following: Follow[]; total: number }> {
    const [result, countResult] = await Promise.all([
      this.pool.query(
        `
        SELECT f.*, u.display_name, u.avatar_url
        FROM follows f
        LEFT JOIN user_profiles u ON f.followed_address = u.wallet_address
        WHERE f.follower_address = $1
        ORDER BY f.created_at DESC
        LIMIT $2 OFFSET $3
        `,
        [address, limit, offset]
      ),
      this.pool.query(
        `SELECT COUNT(*) FROM follows WHERE follower_address = $1`,
        [address]
      ),
    ]);

    return {
      following: result.rows.map(this.mapFollow),
      total: parseInt(countResult.rows[0].count),
    };
  }

  /**
   * Check if following
   */
  async isFollowing(
    followerAddress: string,
    followedAddress: string
  ): Promise<boolean> {
    const result = await this.pool.query(
      `SELECT 1 FROM follows WHERE follower_address = $1 AND followed_address = $2`,
      [followerAddress, followedAddress]
    );
    return result.rows.length > 0;
  }

  // ==================== Activity Feed ====================

  /**
   * Get activity feed for user
   */
  async getFeed(
    walletAddress: string,
    options: {
      limit?: number;
      offset?: number;
      types?: ActivityType[];
    } = {}
  ): Promise<ActivityItem[]> {
    const { limit = 20, offset = 0, types } = options;

    let query = `
      SELECT a.*, u.display_name as actor_name, u.avatar_url as actor_avatar
      FROM activities a
      LEFT JOIN user_profiles u ON a.actor_address = u.wallet_address
      WHERE a.actor_address IN (
        SELECT followed_address FROM follows WHERE follower_address = $1
      )
      OR a.actor_address = $1
    `;

    const params: unknown[] = [walletAddress];

    if (types && types.length > 0) {
      params.push(types);
      query += ` AND a.type = ANY($${params.length})`;
    }

    query += ` ORDER BY a.created_at DESC LIMIT $${params.length + 1} OFFSET $${params.length + 2}`;
    params.push(limit, offset);

    const result = await this.pool.query(query, params);

    return result.rows.map(this.mapActivity);
  }

  /**
   * Get user's activity
   */
  async getUserActivity(
    address: string,
    limit = 20,
    offset = 0
  ): Promise<ActivityItem[]> {
    const result = await this.pool.query(
      `
      SELECT a.*, u.display_name as actor_name, u.avatar_url as actor_avatar
      FROM activities a
      LEFT JOIN user_profiles u ON a.actor_address = u.wallet_address
      WHERE a.actor_address = $1
      ORDER BY a.created_at DESC
      LIMIT $2 OFFSET $3
      `,
      [address, limit, offset]
    );

    return result.rows.map(this.mapActivity);
  }

  /**
   * Create activity
   */
  private async createActivity(params: {
    type: ActivityType;
    actorAddress: string;
    targetId: string;
    targetType: string;
    metadata?: Record<string, unknown>;
  }): Promise<void> {
    await this.pool.query(
      `
      INSERT INTO activities (type, actor_address, target_id, target_type, metadata)
      VALUES ($1, $2, $3, $4, $5)
      `,
      [
        params.type,
        params.actorAddress,
        params.targetId,
        params.targetType,
        JSON.stringify(params.metadata || {}),
      ]
    );
  }

  // ==================== Playlists ====================

  /**
   * Create playlist
   */
  async createPlaylist(
    ownerAddress: string,
    data: {
      name: string;
      description?: string;
      coverImage?: string;
      isPublic?: boolean;
      isCollaborative?: boolean;
    }
  ): Promise<Playlist> {
    const result = await this.pool.query(
      `
      INSERT INTO playlists (
        owner_address, name, description, cover_image, is_public, is_collaborative
      )
      SELECT $1, $2, $3, $4, $5, $6
      RETURNING *
      `,
      [
        ownerAddress,
        data.name,
        data.description,
        data.coverImage,
        data.isPublic ?? true,
        data.isCollaborative ?? false,
      ]
    );

    await this.createActivity({
      type: 'create_playlist',
      actorAddress: ownerAddress,
      targetId: result.rows[0].id,
      targetType: 'playlist',
    });

    getMetrics().incCounter('social_actions_total', { type: 'create_playlist' });

    return this.mapPlaylist(result.rows[0]);
  }

  /**
   * Add song to playlist
   */
  async addToPlaylist(
    playlistId: string,
    songId: string,
    addedBy: string
  ): Promise<PlaylistSong> {
    // Verify access
    const playlist = await this.getPlaylist(playlistId);
    if (!playlist) throw new Error('Playlist not found');

    if (playlist.ownerAddress !== addedBy && !playlist.isCollaborative) {
      throw new Error('Unauthorized');
    }

    // Get next position
    const posResult = await this.pool.query(
      `SELECT COALESCE(MAX(position), 0) + 1 as next_pos FROM playlist_songs WHERE playlist_id = $1`,
      [playlistId]
    );

    const result = await this.pool.query(
      `
      INSERT INTO playlist_songs (playlist_id, song_id, added_by, position)
      VALUES ($1, $2, $3, $4)
      ON CONFLICT (playlist_id, song_id) DO NOTHING
      RETURNING *
      `,
      [playlistId, songId, addedBy, posResult.rows[0].next_pos]
    );

    if (result.rows.length === 0) {
      throw new Error('Song already in playlist');
    }

    // Update playlist counts
    await this.updatePlaylistCounts(playlistId);

    await this.createActivity({
      type: 'add_to_playlist',
      actorAddress: addedBy,
      targetId: playlistId,
      targetType: 'playlist',
      metadata: { songId },
    });

    getMetrics().incCounter('social_actions_total', { type: 'add_to_playlist' });

    return this.mapPlaylistSong(result.rows[0]);
  }

  /**
   * Remove song from playlist
   */
  async removeFromPlaylist(
    playlistId: string,
    songId: string,
    userAddress: string
  ): Promise<void> {
    const playlist = await this.getPlaylist(playlistId);
    if (!playlist) throw new Error('Playlist not found');

    if (playlist.ownerAddress !== userAddress) {
      throw new Error('Unauthorized');
    }

    await this.pool.query(
      `DELETE FROM playlist_songs WHERE playlist_id = $1 AND song_id = $2`,
      [playlistId, songId]
    );

    await this.updatePlaylistCounts(playlistId);
  }

  /**
   * Get playlist
   */
  async getPlaylist(playlistId: string): Promise<Playlist | null> {
    const result = await this.pool.query(
      `
      SELECT p.*, u.display_name as owner_name
      FROM playlists p
      LEFT JOIN user_profiles u ON p.owner_address = u.wallet_address
      WHERE p.id = $1
      `,
      [playlistId]
    );

    if (result.rows.length === 0) return null;

    return this.mapPlaylist(result.rows[0]);
  }

  /**
   * Get playlist songs
   */
  async getPlaylistSongs(
    playlistId: string,
    limit = 100,
    offset = 0
  ): Promise<PlaylistSong[]> {
    const result = await this.pool.query(
      `
      SELECT ps.*, s.title, s.artist_name, s.artist_address, s.duration, s.cover_art_url
      FROM playlist_songs ps
      JOIN songs s ON ps.song_id = s.id
      WHERE ps.playlist_id = $1
      ORDER BY ps.position
      LIMIT $2 OFFSET $3
      `,
      [playlistId, limit, offset]
    );

    return result.rows.map(this.mapPlaylistSong);
  }

  /**
   * Get user's playlists
   */
  async getUserPlaylists(
    address: string,
    includePrivate = false
  ): Promise<Playlist[]> {
    let query = `
      SELECT p.*, u.display_name as owner_name
      FROM playlists p
      LEFT JOIN user_profiles u ON p.owner_address = u.wallet_address
      WHERE p.owner_address = $1
    `;

    if (!includePrivate) {
      query += ` AND p.is_public = true`;
    }

    query += ` ORDER BY p.updated_at DESC`;

    const result = await this.pool.query(query, [address]);

    return result.rows.map(this.mapPlaylist);
  }

  /**
   * Follow playlist
   */
  async followPlaylist(
    userAddress: string,
    playlistId: string
  ): Promise<void> {
    await this.pool.query(
      `
      INSERT INTO playlist_followers (playlist_id, follower_address)
      VALUES ($1, $2)
      ON CONFLICT DO NOTHING
      `,
      [playlistId, userAddress]
    );

    await this.pool.query(
      `UPDATE playlists SET followers = followers + 1 WHERE id = $1`,
      [playlistId]
    );
  }

  private async updatePlaylistCounts(playlistId: string): Promise<void> {
    await this.pool.query(
      `
      UPDATE playlists
      SET
        song_count = (SELECT COUNT(*) FROM playlist_songs WHERE playlist_id = $1),
        duration = (
          SELECT COALESCE(SUM(s.duration), 0)
          FROM playlist_songs ps
          JOIN songs s ON ps.song_id = s.id
          WHERE ps.playlist_id = $1
        ),
        updated_at = NOW()
      WHERE id = $1
      `,
      [playlistId]
    );
  }

  // ==================== Likes ====================

  /**
   * Like a target
   */
  async like(
    userAddress: string,
    targetId: string,
    targetType: 'song' | 'playlist' | 'comment'
  ): Promise<Like> {
    const result = await this.pool.query(
      `
      INSERT INTO likes (user_address, target_id, target_type)
      VALUES ($1, $2, $3)
      ON CONFLICT (user_address, target_id, target_type) DO NOTHING
      RETURNING *
      `,
      [userAddress, targetId, targetType]
    );

    if (result.rows.length === 0) {
      throw new Error('Already liked');
    }

    // Update like count on target
    if (targetType === 'song') {
      await this.pool.query(
        `UPDATE songs SET like_count = like_count + 1 WHERE id = $1`,
        [targetId]
      );
    }

    await this.createActivity({
      type: `like_${targetType}` as ActivityType,
      actorAddress: userAddress,
      targetId,
      targetType,
    });

    getMetrics().incCounter('social_actions_total', { type: 'like' });

    return this.mapLike(result.rows[0]);
  }

  /**
   * Unlike
   */
  async unlike(
    userAddress: string,
    targetId: string,
    targetType: 'song' | 'playlist' | 'comment'
  ): Promise<void> {
    const result = await this.pool.query(
      `
      DELETE FROM likes
      WHERE user_address = $1 AND target_id = $2 AND target_type = $3
      RETURNING *
      `,
      [userAddress, targetId, targetType]
    );

    if (result.rows.length > 0) {
      if (targetType === 'song') {
        await this.pool.query(
          `UPDATE songs SET like_count = GREATEST(0, like_count - 1) WHERE id = $1`,
          [targetId]
        );
      }
    }
  }

  /**
   * Check if liked
   */
  async isLiked(
    userAddress: string,
    targetId: string,
    targetType: 'song' | 'playlist' | 'comment'
  ): Promise<boolean> {
    const result = await this.pool.query(
      `SELECT 1 FROM likes WHERE user_address = $1 AND target_id = $2 AND target_type = $3`,
      [userAddress, targetId, targetType]
    );
    return result.rows.length > 0;
  }

  /**
   * Get user's liked songs
   */
  async getLikedSongs(
    userAddress: string,
    limit = 50,
    offset = 0
  ): Promise<{ songId: string; likedAt: Date }[]> {
    const result = await this.pool.query(
      `
      SELECT target_id as song_id, created_at as liked_at
      FROM likes
      WHERE user_address = $1 AND target_type = 'song'
      ORDER BY created_at DESC
      LIMIT $2 OFFSET $3
      `,
      [userAddress, limit, offset]
    );

    return result.rows;
  }

  // ==================== Comments ====================

  /**
   * Add comment
   */
  async addComment(
    authorAddress: string,
    targetId: string,
    targetType: 'song' | 'playlist' | 'profile',
    content: string,
    parentId?: string
  ): Promise<Comment> {
    const result = await this.pool.query(
      `
      INSERT INTO comments (
        author_address, target_id, target_type, parent_id, content
      )
      VALUES ($1, $2, $3, $4, $5)
      RETURNING *
      `,
      [authorAddress, targetId, targetType, parentId, content]
    );

    // Update reply count on parent
    if (parentId) {
      await this.pool.query(
        `UPDATE comments SET reply_count = reply_count + 1 WHERE id = $1`,
        [parentId]
      );
    }

    await this.createActivity({
      type: parentId ? 'reply' : 'comment',
      actorAddress: authorAddress,
      targetId,
      targetType,
    });

    getMetrics().incCounter('social_actions_total', { type: 'comment' });

    return this.mapComment(result.rows[0]);
  }

  /**
   * Get comments
   */
  async getComments(
    targetId: string,
    targetType: 'song' | 'playlist' | 'profile',
    options: { limit?: number; offset?: number; parentId?: string } = {}
  ): Promise<Comment[]> {
    const { limit = 20, offset = 0, parentId } = options;

    const result = await this.pool.query(
      `
      SELECT c.*, u.display_name as author_name, u.avatar_url as author_avatar
      FROM comments c
      LEFT JOIN user_profiles u ON c.author_address = u.wallet_address
      WHERE c.target_id = $1
        AND c.target_type = $2
        AND c.parent_id ${parentId ? '= $5' : 'IS NULL'}
      ORDER BY c.created_at DESC
      LIMIT $3 OFFSET $4
      `,
      parentId
        ? [targetId, targetType, limit, offset, parentId]
        : [targetId, targetType, limit, offset]
    );

    return result.rows.map(this.mapComment);
  }

  // ==================== Notifications ====================

  /**
   * Create notification
   */
  async createNotification(params: {
    recipientAddress: string;
    type: NotificationType;
    actorAddress?: string;
    title: string;
    body: string;
    targetId?: string;
    targetType?: string;
  }): Promise<void> {
    await this.pool.query(
      `
      INSERT INTO notifications (
        recipient_address, type, actor_address, title, body, target_id, target_type
      )
      VALUES ($1, $2, $3, $4, $5, $6, $7)
      `,
      [
        params.recipientAddress,
        params.type,
        params.actorAddress,
        params.title,
        params.body,
        params.targetId,
        params.targetType,
      ]
    );

    // Emit for real-time delivery
    this.events.emit('notification:created', {
      recipientAddress: params.recipientAddress,
      type: params.type,
    });
  }

  /**
   * Get notifications
   */
  async getNotifications(
    recipientAddress: string,
    options: { limit?: number; unreadOnly?: boolean } = {}
  ): Promise<Notification[]> {
    const { limit = 50, unreadOnly = false } = options;

    let query = `
      SELECT n.*, u.display_name as actor_name, u.avatar_url as actor_avatar
      FROM notifications n
      LEFT JOIN user_profiles u ON n.actor_address = u.wallet_address
      WHERE n.recipient_address = $1
    `;

    if (unreadOnly) {
      query += ` AND n.read = false`;
    }

    query += ` ORDER BY n.created_at DESC LIMIT $2`;

    const result = await this.pool.query(query, [recipientAddress, limit]);

    return result.rows.map(this.mapNotification);
  }

  /**
   * Mark notifications as read
   */
  async markNotificationsRead(
    recipientAddress: string,
    notificationIds?: string[]
  ): Promise<void> {
    if (notificationIds && notificationIds.length > 0) {
      await this.pool.query(
        `UPDATE notifications SET read = true WHERE id = ANY($1) AND recipient_address = $2`,
        [notificationIds, recipientAddress]
      );
    } else {
      await this.pool.query(
        `UPDATE notifications SET read = true WHERE recipient_address = $1`,
        [recipientAddress]
      );
    }
  }

  /**
   * Get unread count
   */
  async getUnreadCount(recipientAddress: string): Promise<number> {
    const result = await this.pool.query(
      `SELECT COUNT(*) FROM notifications WHERE recipient_address = $1 AND read = false`,
      [recipientAddress]
    );
    return parseInt(result.rows[0].count);
  }

  // ==================== Mappers ====================

  private mapFollow(row: any): Follow {
    return {
      id: row.id,
      followerAddress: row.follower_address,
      followedAddress: row.followed_address,
      followedType: row.followed_type,
      createdAt: row.created_at,
    };
  }

  private mapActivity(row: any): ActivityItem {
    return {
      id: row.id,
      type: row.type,
      actorAddress: row.actor_address,
      actorName: row.actor_name,
      actorAvatar: row.actor_avatar,
      targetId: row.target_id,
      targetType: row.target_type,
      targetTitle: row.target_title || '',
      targetImage: row.target_image,
      metadata: row.metadata,
      createdAt: row.created_at,
    };
  }

  private mapPlaylist(row: any): Playlist {
    return {
      id: row.id,
      ownerAddress: row.owner_address,
      ownerName: row.owner_name,
      name: row.name,
      description: row.description,
      coverImage: row.cover_image,
      isPublic: row.is_public,
      isCollaborative: row.is_collaborative,
      songCount: row.song_count || 0,
      duration: row.duration || 0,
      followers: row.followers || 0,
      createdAt: row.created_at,
      updatedAt: row.updated_at,
    };
  }

  private mapPlaylistSong(row: any): PlaylistSong {
    return {
      id: row.id,
      playlistId: row.playlist_id,
      songId: row.song_id,
      addedBy: row.added_by,
      position: row.position,
      addedAt: row.added_at,
      song: row.title
        ? {
            id: row.song_id,
            title: row.title,
            artist: row.artist_name,
            artistAddress: row.artist_address,
            duration: row.duration,
            coverArt: row.cover_art_url,
          }
        : undefined,
    };
  }

  private mapLike(row: any): Like {
    return {
      id: row.id,
      userAddress: row.user_address,
      targetId: row.target_id,
      targetType: row.target_type,
      createdAt: row.created_at,
    };
  }

  private mapComment(row: any): Comment {
    return {
      id: row.id,
      authorAddress: row.author_address,
      authorName: row.author_name,
      authorAvatar: row.author_avatar,
      targetId: row.target_id,
      targetType: row.target_type,
      parentId: row.parent_id,
      content: row.content,
      likes: row.likes || 0,
      replyCount: row.reply_count || 0,
      createdAt: row.created_at,
      updatedAt: row.updated_at,
    };
  }

  private mapNotification(row: any): Notification {
    return {
      id: row.id,
      recipientAddress: row.recipient_address,
      type: row.type,
      actorAddress: row.actor_address,
      actorName: row.actor_name,
      actorAvatar: row.actor_avatar,
      title: row.title,
      body: row.body,
      targetId: row.target_id,
      targetType: row.target_type,
      read: row.read,
      createdAt: row.created_at,
    };
  }
}

/**
 * Migration SQL
 */
export const SOCIAL_MIGRATION_SQL = `
-- Follows
CREATE TABLE IF NOT EXISTS follows (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  follower_address VARCHAR(42) NOT NULL,
  followed_address VARCHAR(42) NOT NULL,
  followed_type VARCHAR(20) DEFAULT 'user',
  created_at TIMESTAMPTZ DEFAULT NOW(),
  UNIQUE(follower_address, followed_address)
);

CREATE INDEX idx_follows_follower ON follows(follower_address);
CREATE INDEX idx_follows_followed ON follows(followed_address);

-- Activities
CREATE TABLE IF NOT EXISTS activities (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  type VARCHAR(50) NOT NULL,
  actor_address VARCHAR(42) NOT NULL,
  target_id VARCHAR(100),
  target_type VARCHAR(50),
  target_title VARCHAR(500),
  target_image VARCHAR(500),
  metadata JSONB DEFAULT '{}',
  created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_activities_actor ON activities(actor_address);
CREATE INDEX idx_activities_created ON activities(created_at);

-- Playlists
CREATE TABLE IF NOT EXISTS playlists (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  owner_address VARCHAR(42) NOT NULL,
  name VARCHAR(200) NOT NULL,
  description TEXT,
  cover_image VARCHAR(500),
  is_public BOOLEAN DEFAULT true,
  is_collaborative BOOLEAN DEFAULT false,
  song_count INTEGER DEFAULT 0,
  duration INTEGER DEFAULT 0,
  followers INTEGER DEFAULT 0,
  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_playlists_owner ON playlists(owner_address);

-- Playlist songs
CREATE TABLE IF NOT EXISTS playlist_songs (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  playlist_id UUID REFERENCES playlists(id) ON DELETE CASCADE,
  song_id UUID REFERENCES songs(id) ON DELETE CASCADE,
  added_by VARCHAR(42) NOT NULL,
  position INTEGER NOT NULL,
  added_at TIMESTAMPTZ DEFAULT NOW(),
  UNIQUE(playlist_id, song_id)
);

CREATE INDEX idx_playlist_songs_playlist ON playlist_songs(playlist_id);

-- Playlist followers
CREATE TABLE IF NOT EXISTS playlist_followers (
  playlist_id UUID REFERENCES playlists(id) ON DELETE CASCADE,
  follower_address VARCHAR(42) NOT NULL,
  created_at TIMESTAMPTZ DEFAULT NOW(),
  PRIMARY KEY(playlist_id, follower_address)
);

-- Likes
CREATE TABLE IF NOT EXISTS likes (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_address VARCHAR(42) NOT NULL,
  target_id VARCHAR(100) NOT NULL,
  target_type VARCHAR(50) NOT NULL,
  created_at TIMESTAMPTZ DEFAULT NOW(),
  UNIQUE(user_address, target_id, target_type)
);

CREATE INDEX idx_likes_user ON likes(user_address);
CREATE INDEX idx_likes_target ON likes(target_id, target_type);

-- Comments
CREATE TABLE IF NOT EXISTS comments (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  author_address VARCHAR(42) NOT NULL,
  target_id VARCHAR(100) NOT NULL,
  target_type VARCHAR(50) NOT NULL,
  parent_id UUID REFERENCES comments(id) ON DELETE CASCADE,
  content TEXT NOT NULL,
  likes INTEGER DEFAULT 0,
  reply_count INTEGER DEFAULT 0,
  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_comments_target ON comments(target_id, target_type);
CREATE INDEX idx_comments_author ON comments(author_address);

-- Notifications
CREATE TABLE IF NOT EXISTS notifications (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  recipient_address VARCHAR(42) NOT NULL,
  type VARCHAR(50) NOT NULL,
  actor_address VARCHAR(42),
  title VARCHAR(200) NOT NULL,
  body TEXT,
  target_id VARCHAR(100),
  target_type VARCHAR(50),
  read BOOLEAN DEFAULT false,
  created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_notifications_recipient ON notifications(recipient_address);
CREATE INDEX idx_notifications_unread ON notifications(recipient_address, read) WHERE read = false;
`;

export default SocialService;
