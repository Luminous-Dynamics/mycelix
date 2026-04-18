// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Artist Analytics Service
 *
 * Comprehensive analytics for artists: streams, revenue,
 * audience demographics, engagement metrics, and trends.
 */

import { Pool } from 'pg';
import { getLogger } from '../logging';
import { getMetrics } from '../metrics';

const logger = getLogger();

/**
 * Time period for analytics
 */
export type TimePeriod = '24h' | '7d' | '30d' | '90d' | '1y' | 'all';

/**
 * Stream analytics
 */
export interface StreamAnalytics {
  totalStreams: number;
  uniqueListeners: number;
  avgListenDuration: number;
  completionRate: number;
  skipRate: number;
  repeatRate: number;
  peakHour: number;
  peakDay: string;
}

/**
 * Revenue analytics
 */
export interface RevenueAnalytics {
  totalRevenue: number;
  streamingRevenue: number;
  nftRevenue: number;
  tipsReceived: number;
  avgRevenuePerStream: number;
  pendingPayout: number;
  lastPayout: {
    amount: number;
    date: Date;
    txHash: string;
  } | null;
}

/**
 * Audience demographics
 */
export interface AudienceDemographics {
  countries: { country: string; listeners: number; percentage: number }[];
  cities: { city: string; listeners: number }[];
  ageGroups: { range: string; percentage: number }[];
  platforms: { platform: string; percentage: number }[];
  sources: { source: string; streams: number }[];
}

/**
 * Song performance
 */
export interface SongPerformance {
  songId: string;
  title: string;
  streams: number;
  uniqueListeners: number;
  saves: number;
  shares: number;
  playlistAdds: number;
  avgPosition: number;
  trend: 'up' | 'down' | 'stable';
  trendPercent: number;
}

/**
 * Engagement metrics
 */
export interface EngagementMetrics {
  followers: number;
  followerGrowth: number;
  followerGrowthPercent: number;
  playlistReach: number;
  socialShares: number;
  profileViews: number;
  savesToLibrary: number;
  avgEngagementRate: number;
}

/**
 * Time series data point
 */
export interface TimeSeriesPoint {
  date: Date;
  value: number;
}

/**
 * Artist dashboard data
 */
export interface ArtistDashboard {
  period: TimePeriod;
  streams: StreamAnalytics;
  revenue: RevenueAnalytics;
  engagement: EngagementMetrics;
  topSongs: SongPerformance[];
  streamHistory: TimeSeriesPoint[];
  revenueHistory: TimeSeriesPoint[];
  followerHistory: TimeSeriesPoint[];
}

/**
 * Real-time stats
 */
export interface RealTimeStats {
  activeListeners: number;
  streamsToday: number;
  revenueToday: number;
  newFollowersToday: number;
  currentlyPlaying: {
    songId: string;
    title: string;
    listeners: number;
  }[];
}

/**
 * Artist Analytics Service
 */
export class ArtistAnalyticsService {
  constructor(private pool: Pool) {
    const metrics = getMetrics();
    metrics.createCounter('artist_analytics_queries_total', 'Analytics queries', ['type']);
  }

  /**
   * Get full dashboard data
   */
  async getDashboard(
    artistAddress: string,
    period: TimePeriod = '30d'
  ): Promise<ArtistDashboard> {
    const [streams, revenue, engagement, topSongs, streamHistory, revenueHistory, followerHistory] =
      await Promise.all([
        this.getStreamAnalytics(artistAddress, period),
        this.getRevenueAnalytics(artistAddress, period),
        this.getEngagementMetrics(artistAddress, period),
        this.getTopSongs(artistAddress, period, 10),
        this.getStreamHistory(artistAddress, period),
        this.getRevenueHistory(artistAddress, period),
        this.getFollowerHistory(artistAddress, period),
      ]);

    getMetrics().incCounter('artist_analytics_queries_total', { type: 'dashboard' });

    return {
      period,
      streams,
      revenue,
      engagement,
      topSongs,
      streamHistory,
      revenueHistory,
      followerHistory,
    };
  }

  /**
   * Get stream analytics
   */
  async getStreamAnalytics(
    artistAddress: string,
    period: TimePeriod
  ): Promise<StreamAnalytics> {
    const interval = this.periodToInterval(period);

    const result = await this.pool.query(
      `
      WITH period_plays AS (
        SELECT
          p.*,
          s.duration as song_duration
        FROM plays p
        JOIN songs s ON p.song_id = s.id
        WHERE s.artist_address = $1
          AND p.played_at >= NOW() - $2::interval
      )
      SELECT
        COUNT(*) as total_streams,
        COUNT(DISTINCT wallet_address) as unique_listeners,
        AVG(duration) as avg_listen_duration,
        AVG(CASE WHEN duration >= song_duration * 0.9 THEN 1 ELSE 0 END) as completion_rate,
        AVG(CASE WHEN duration < 30 THEN 1 ELSE 0 END) as skip_rate,
        COUNT(*) FILTER (WHERE wallet_address IN (
          SELECT wallet_address FROM period_plays GROUP BY wallet_address HAVING COUNT(*) > 1
        ))::float / NULLIF(COUNT(*), 0) as repeat_rate,
        EXTRACT(HOUR FROM MODE() WITHIN GROUP (ORDER BY played_at)) as peak_hour,
        TO_CHAR(MODE() WITHIN GROUP (ORDER BY played_at), 'Day') as peak_day
      FROM period_plays
      `,
      [artistAddress, interval]
    );

    const row = result.rows[0];

    return {
      totalStreams: parseInt(row.total_streams) || 0,
      uniqueListeners: parseInt(row.unique_listeners) || 0,
      avgListenDuration: parseFloat(row.avg_listen_duration) || 0,
      completionRate: parseFloat(row.completion_rate) || 0,
      skipRate: parseFloat(row.skip_rate) || 0,
      repeatRate: parseFloat(row.repeat_rate) || 0,
      peakHour: parseInt(row.peak_hour) || 12,
      peakDay: row.peak_day?.trim() || 'Saturday',
    };
  }

  /**
   * Get revenue analytics
   */
  async getRevenueAnalytics(
    artistAddress: string,
    period: TimePeriod
  ): Promise<RevenueAnalytics> {
    const interval = this.periodToInterval(period);

    const [revenueResult, payoutResult] = await Promise.all([
      this.pool.query(
        `
        SELECT
          COALESCE(SUM(amount), 0) as total_revenue,
          COALESCE(SUM(CASE WHEN type = 'streaming' THEN amount ELSE 0 END), 0) as streaming_revenue,
          COALESCE(SUM(CASE WHEN type = 'nft' THEN amount ELSE 0 END), 0) as nft_revenue,
          COALESCE(SUM(CASE WHEN type = 'tip' THEN amount ELSE 0 END), 0) as tips,
          COUNT(*) FILTER (WHERE type = 'streaming') as stream_count
        FROM artist_earnings
        WHERE artist_address = $1
          AND created_at >= NOW() - $2::interval
        `,
        [artistAddress, interval]
      ),
      this.pool.query(
        `
        SELECT amount, created_at, tx_hash
        FROM artist_payouts
        WHERE artist_address = $1
        ORDER BY created_at DESC
        LIMIT 1
        `,
        [artistAddress]
      ),
    ]);

    const rev = revenueResult.rows[0];
    const payout = payoutResult.rows[0];
    const totalRevenue = parseFloat(rev.total_revenue) || 0;
    const streamCount = parseInt(rev.stream_count) || 1;

    // Get pending payout
    const pendingResult = await this.pool.query(
      `
      SELECT COALESCE(SUM(amount), 0) as pending
      FROM artist_earnings
      WHERE artist_address = $1 AND paid_out = false
      `,
      [artistAddress]
    );

    return {
      totalRevenue,
      streamingRevenue: parseFloat(rev.streaming_revenue) || 0,
      nftRevenue: parseFloat(rev.nft_revenue) || 0,
      tipsReceived: parseFloat(rev.tips) || 0,
      avgRevenuePerStream: totalRevenue / streamCount,
      pendingPayout: parseFloat(pendingResult.rows[0].pending) || 0,
      lastPayout: payout
        ? {
            amount: parseFloat(payout.amount),
            date: payout.created_at,
            txHash: payout.tx_hash,
          }
        : null,
    };
  }

  /**
   * Get engagement metrics
   */
  async getEngagementMetrics(
    artistAddress: string,
    period: TimePeriod
  ): Promise<EngagementMetrics> {
    const interval = this.periodToInterval(period);

    const result = await this.pool.query(
      `
      WITH current_period AS (
        SELECT COUNT(*) as followers
        FROM follows
        WHERE followed_address = $1
      ),
      period_growth AS (
        SELECT COUNT(*) as new_followers
        FROM follows
        WHERE followed_address = $1
          AND created_at >= NOW() - $2::interval
      ),
      previous_period AS (
        SELECT COUNT(*) as old_followers
        FROM follows
        WHERE followed_address = $1
          AND created_at < NOW() - $2::interval
          AND created_at >= NOW() - ($2::interval * 2)
      ),
      engagement AS (
        SELECT
          COUNT(DISTINCT pl.id) as playlist_adds,
          COUNT(DISTINCT sh.id) as shares,
          COUNT(DISTINCT sv.id) as saves
        FROM songs s
        LEFT JOIN playlist_songs pl ON s.id = pl.song_id AND pl.added_at >= NOW() - $2::interval
        LEFT JOIN social_shares sh ON s.id = sh.song_id AND sh.created_at >= NOW() - $2::interval
        LEFT JOIN library_saves sv ON s.id = sv.song_id AND sv.created_at >= NOW() - $2::interval
        WHERE s.artist_address = $1
      )
      SELECT
        cp.followers,
        pg.new_followers as follower_growth,
        CASE WHEN pp.old_followers > 0
          THEN ((pg.new_followers - pp.old_followers)::float / pp.old_followers * 100)
          ELSE 0
        END as follower_growth_percent,
        e.playlist_adds,
        e.shares,
        e.saves
      FROM current_period cp, period_growth pg, previous_period pp, engagement e
      `,
      [artistAddress, interval]
    );

    const row = result.rows[0];

    // Get profile views
    const viewsResult = await this.pool.query(
      `
      SELECT COUNT(*) as views
      FROM profile_views
      WHERE artist_address = $1
        AND viewed_at >= NOW() - $2::interval
      `,
      [artistAddress, interval]
    );

    // Calculate playlist reach (unique listeners from playlists)
    const reachResult = await this.pool.query(
      `
      SELECT COUNT(DISTINCT p.wallet_address) as reach
      FROM plays p
      JOIN playlist_songs ps ON p.song_id = ps.song_id
      JOIN songs s ON p.song_id = s.id
      WHERE s.artist_address = $1
        AND p.played_at >= NOW() - $2::interval
        AND p.source = 'playlist'
      `,
      [artistAddress, interval]
    );

    const followers = parseInt(row.followers) || 0;
    const shares = parseInt(row.shares) || 0;
    const saves = parseInt(row.saves) || 0;

    return {
      followers,
      followerGrowth: parseInt(row.follower_growth) || 0,
      followerGrowthPercent: parseFloat(row.follower_growth_percent) || 0,
      playlistReach: parseInt(reachResult.rows[0].reach) || 0,
      socialShares: shares,
      profileViews: parseInt(viewsResult.rows[0].views) || 0,
      savesToLibrary: saves,
      avgEngagementRate: followers > 0 ? (shares + saves) / followers : 0,
    };
  }

  /**
   * Get top performing songs
   */
  async getTopSongs(
    artistAddress: string,
    period: TimePeriod,
    limit = 10
  ): Promise<SongPerformance[]> {
    const interval = this.periodToInterval(period);

    const result = await this.pool.query(
      `
      WITH current_period AS (
        SELECT
          s.id,
          s.title,
          COUNT(p.id) as streams,
          COUNT(DISTINCT p.wallet_address) as unique_listeners,
          COUNT(DISTINCT ls.id) as saves,
          COUNT(DISTINCT sh.id) as shares,
          COUNT(DISTINCT ps.id) as playlist_adds,
          AVG(p.queue_position) as avg_position
        FROM songs s
        LEFT JOIN plays p ON s.id = p.song_id AND p.played_at >= NOW() - $2::interval
        LEFT JOIN library_saves ls ON s.id = ls.song_id AND ls.created_at >= NOW() - $2::interval
        LEFT JOIN social_shares sh ON s.id = sh.song_id AND sh.created_at >= NOW() - $2::interval
        LEFT JOIN playlist_songs ps ON s.id = ps.song_id AND ps.added_at >= NOW() - $2::interval
        WHERE s.artist_address = $1
        GROUP BY s.id, s.title
      ),
      previous_period AS (
        SELECT
          s.id,
          COUNT(p.id) as streams
        FROM songs s
        LEFT JOIN plays p ON s.id = p.song_id
          AND p.played_at >= NOW() - ($2::interval * 2)
          AND p.played_at < NOW() - $2::interval
        WHERE s.artist_address = $1
        GROUP BY s.id
      )
      SELECT
        cp.*,
        COALESCE(pp.streams, 0) as prev_streams,
        CASE
          WHEN COALESCE(pp.streams, 0) = 0 THEN 'stable'
          WHEN cp.streams > pp.streams THEN 'up'
          WHEN cp.streams < pp.streams THEN 'down'
          ELSE 'stable'
        END as trend,
        CASE
          WHEN COALESCE(pp.streams, 0) = 0 THEN 0
          ELSE ((cp.streams - pp.streams)::float / pp.streams * 100)
        END as trend_percent
      FROM current_period cp
      LEFT JOIN previous_period pp ON cp.id = pp.id
      ORDER BY cp.streams DESC
      LIMIT $3
      `,
      [artistAddress, interval, limit]
    );

    return result.rows.map((row) => ({
      songId: row.id,
      title: row.title,
      streams: parseInt(row.streams) || 0,
      uniqueListeners: parseInt(row.unique_listeners) || 0,
      saves: parseInt(row.saves) || 0,
      shares: parseInt(row.shares) || 0,
      playlistAdds: parseInt(row.playlist_adds) || 0,
      avgPosition: parseFloat(row.avg_position) || 0,
      trend: row.trend as 'up' | 'down' | 'stable',
      trendPercent: parseFloat(row.trend_percent) || 0,
    }));
  }

  /**
   * Get stream history time series
   */
  async getStreamHistory(
    artistAddress: string,
    period: TimePeriod
  ): Promise<TimeSeriesPoint[]> {
    const { interval, granularity } = this.getTimeSeriesParams(period);

    const result = await this.pool.query(
      `
      SELECT
        date_trunc($3, p.played_at) as date,
        COUNT(*) as value
      FROM plays p
      JOIN songs s ON p.song_id = s.id
      WHERE s.artist_address = $1
        AND p.played_at >= NOW() - $2::interval
      GROUP BY date_trunc($3, p.played_at)
      ORDER BY date
      `,
      [artistAddress, interval, granularity]
    );

    return result.rows.map((row) => ({
      date: row.date,
      value: parseInt(row.value),
    }));
  }

  /**
   * Get revenue history time series
   */
  async getRevenueHistory(
    artistAddress: string,
    period: TimePeriod
  ): Promise<TimeSeriesPoint[]> {
    const { interval, granularity } = this.getTimeSeriesParams(period);

    const result = await this.pool.query(
      `
      SELECT
        date_trunc($3, created_at) as date,
        SUM(amount) as value
      FROM artist_earnings
      WHERE artist_address = $1
        AND created_at >= NOW() - $2::interval
      GROUP BY date_trunc($3, created_at)
      ORDER BY date
      `,
      [artistAddress, interval, granularity]
    );

    return result.rows.map((row) => ({
      date: row.date,
      value: parseFloat(row.value),
    }));
  }

  /**
   * Get follower history time series
   */
  async getFollowerHistory(
    artistAddress: string,
    period: TimePeriod
  ): Promise<TimeSeriesPoint[]> {
    const { interval, granularity } = this.getTimeSeriesParams(period);

    const result = await this.pool.query(
      `
      SELECT
        date_trunc($3, created_at) as date,
        COUNT(*) as value
      FROM follows
      WHERE followed_address = $1
        AND created_at >= NOW() - $2::interval
      GROUP BY date_trunc($3, created_at)
      ORDER BY date
      `,
      [artistAddress, interval, granularity]
    );

    return result.rows.map((row) => ({
      date: row.date,
      value: parseInt(row.value),
    }));
  }

  /**
   * Get audience demographics
   */
  async getAudienceDemographics(
    artistAddress: string,
    period: TimePeriod
  ): Promise<AudienceDemographics> {
    const interval = this.periodToInterval(period);

    const [countries, cities, platforms, sources] = await Promise.all([
      this.pool.query(
        `
        SELECT
          country,
          COUNT(DISTINCT wallet_address) as listeners,
          COUNT(DISTINCT wallet_address)::float / SUM(COUNT(DISTINCT wallet_address)) OVER () * 100 as percentage
        FROM plays p
        JOIN songs s ON p.song_id = s.id
        JOIN user_profiles up ON p.wallet_address = up.wallet_address
        WHERE s.artist_address = $1
          AND p.played_at >= NOW() - $2::interval
        GROUP BY country
        ORDER BY listeners DESC
        LIMIT 10
        `,
        [artistAddress, interval]
      ),
      this.pool.query(
        `
        SELECT
          city,
          COUNT(DISTINCT wallet_address) as listeners
        FROM plays p
        JOIN songs s ON p.song_id = s.id
        JOIN user_profiles up ON p.wallet_address = up.wallet_address
        WHERE s.artist_address = $1
          AND p.played_at >= NOW() - $2::interval
        GROUP BY city
        ORDER BY listeners DESC
        LIMIT 10
        `,
        [artistAddress, interval]
      ),
      this.pool.query(
        `
        SELECT
          platform,
          COUNT(*)::float / SUM(COUNT(*)) OVER () * 100 as percentage
        FROM plays p
        JOIN songs s ON p.song_id = s.id
        WHERE s.artist_address = $1
          AND p.played_at >= NOW() - $2::interval
        GROUP BY platform
        ORDER BY percentage DESC
        `,
        [artistAddress, interval]
      ),
      this.pool.query(
        `
        SELECT
          source,
          COUNT(*) as streams
        FROM plays p
        JOIN songs s ON p.song_id = s.id
        WHERE s.artist_address = $1
          AND p.played_at >= NOW() - $2::interval
        GROUP BY source
        ORDER BY streams DESC
        `,
        [artistAddress, interval]
      ),
    ]);

    return {
      countries: countries.rows.map((r) => ({
        country: r.country,
        listeners: parseInt(r.listeners),
        percentage: parseFloat(r.percentage),
      })),
      cities: cities.rows.map((r) => ({
        city: r.city,
        listeners: parseInt(r.listeners),
      })),
      ageGroups: [], // Would require age data
      platforms: platforms.rows.map((r) => ({
        platform: r.platform,
        percentage: parseFloat(r.percentage),
      })),
      sources: sources.rows.map((r) => ({
        source: r.source,
        streams: parseInt(r.streams),
      })),
    };
  }

  /**
   * Get real-time stats
   */
  async getRealTimeStats(artistAddress: string): Promise<RealTimeStats> {
    const [listeners, today, playing] = await Promise.all([
      this.pool.query(
        `
        SELECT COUNT(DISTINCT wallet_address) as count
        FROM plays p
        JOIN songs s ON p.song_id = s.id
        WHERE s.artist_address = $1
          AND p.played_at >= NOW() - INTERVAL '5 minutes'
        `,
        [artistAddress]
      ),
      this.pool.query(
        `
        SELECT
          COUNT(DISTINCT p.id) as streams,
          COALESCE(SUM(e.amount), 0) as revenue,
          COUNT(DISTINCT f.id) as followers
        FROM songs s
        LEFT JOIN plays p ON s.id = p.song_id AND p.played_at >= CURRENT_DATE
        LEFT JOIN artist_earnings e ON s.artist_address = e.artist_address AND e.created_at >= CURRENT_DATE
        LEFT JOIN follows f ON s.artist_address = f.followed_address AND f.created_at >= CURRENT_DATE
        WHERE s.artist_address = $1
        `,
        [artistAddress]
      ),
      this.pool.query(
        `
        SELECT
          s.id as song_id,
          s.title,
          COUNT(DISTINCT p.wallet_address) as listeners
        FROM plays p
        JOIN songs s ON p.song_id = s.id
        WHERE s.artist_address = $1
          AND p.played_at >= NOW() - INTERVAL '5 minutes'
        GROUP BY s.id, s.title
        ORDER BY listeners DESC
        LIMIT 5
        `,
        [artistAddress]
      ),
    ]);

    const todayRow = today.rows[0];

    return {
      activeListeners: parseInt(listeners.rows[0].count) || 0,
      streamsToday: parseInt(todayRow.streams) || 0,
      revenueToday: parseFloat(todayRow.revenue) || 0,
      newFollowersToday: parseInt(todayRow.followers) || 0,
      currentlyPlaying: playing.rows.map((r) => ({
        songId: r.song_id,
        title: r.title,
        listeners: parseInt(r.listeners),
      })),
    };
  }

  /**
   * Convert period to PostgreSQL interval
   */
  private periodToInterval(period: TimePeriod): string {
    switch (period) {
      case '24h':
        return '24 hours';
      case '7d':
        return '7 days';
      case '30d':
        return '30 days';
      case '90d':
        return '90 days';
      case '1y':
        return '1 year';
      case 'all':
        return '100 years';
      default:
        return '30 days';
    }
  }

  /**
   * Get time series parameters
   */
  private getTimeSeriesParams(period: TimePeriod): {
    interval: string;
    granularity: string;
  } {
    switch (period) {
      case '24h':
        return { interval: '24 hours', granularity: 'hour' };
      case '7d':
        return { interval: '7 days', granularity: 'day' };
      case '30d':
        return { interval: '30 days', granularity: 'day' };
      case '90d':
        return { interval: '90 days', granularity: 'week' };
      case '1y':
        return { interval: '1 year', granularity: 'month' };
      case 'all':
        return { interval: '100 years', granularity: 'month' };
      default:
        return { interval: '30 days', granularity: 'day' };
    }
  }
}

export default ArtistAnalyticsService;
