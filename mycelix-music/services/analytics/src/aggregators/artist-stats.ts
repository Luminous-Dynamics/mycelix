// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Artist Stats Aggregator
 *
 * Aggregates and serves comprehensive artist analytics including:
 * - Revenue breakdown and projections
 * - Audience demographics and geography
 * - Engagement patterns and trends
 * - Playlist placements and discovery metrics
 */

import { Redis } from 'ioredis';
import { Pool } from 'pg';

// ============================================================================
// Types
// ============================================================================

interface ArtistOverview {
  artistId: string;
  period: DateRange;
  summary: {
    totalPlays: number;
    uniqueListeners: number;
    totalListenTime: number;  // seconds
    avgListenDuration: number;
    completionRate: number;   // percentage
    saveRate: number;         // percentage of listeners who saved
    shareRate: number;        // percentage who shared
  };
  revenue: {
    total: number;
    streaming: number;
    patronage: number;
    nftSales: number;
    licensing: number;
    tips: number;
  };
  growth: {
    playsChange: number;      // percentage vs previous period
    listenersChange: number;
    revenueChange: number;
    followersChange: number;
  };
}

interface AudienceDemographics {
  ageRanges: Record<string, number>;      // "18-24": 1500
  genders: Record<string, number>;
  topCountries: { country: string; listeners: number; percentage: number }[];
  topCities: { city: string; country: string; listeners: number }[];
  listeningTimes: {
    byHour: number[];         // 24 hours
    byDayOfWeek: number[];    // 7 days
  };
  deviceTypes: Record<string, number>;
  premiumVsFree: { premium: number; free: number };
}

interface TrackPerformance {
  trackId: string;
  title: string;
  plays: number;
  uniqueListeners: number;
  saves: number;
  shares: number;
  completionRate: number;
  avgListenDuration: number;
  revenue: number;
  playlistPlacements: number;
  trend: 'rising' | 'stable' | 'declining';
  peakPosition: number;       // highest chart position
}

interface PlaylistPlacement {
  playlistId: string;
  playlistName: string;
  curator: string;
  followers: number;
  trackId: string;
  addedAt: Date;
  position: number;
  playsFromPlaylist: number;
  isEditorial: boolean;
}

interface RevenueBreakdown {
  period: DateRange;
  total: number;
  bySource: {
    streaming: { amount: number; plays: number; ratePerPlay: number };
    patronage: { amount: number; patrons: number; avgPerPatron: number };
    nftSales: { amount: number; sales: number; avgPrice: number };
    licensing: { amount: number; licenses: number };
    tips: { amount: number; tippers: number };
  };
  byTrack: { trackId: string; title: string; amount: number }[];
  projectedMonthly: number;
  payoutSchedule: { date: Date; amount: number; status: string }[];
}

interface EngagementMetrics {
  followers: {
    total: number;
    gained: number;
    lost: number;
    netChange: number;
  };
  engagement: {
    likes: number;
    comments: number;
    shares: number;
    saves: number;
    rate: number;  // engagement rate percentage
  };
  circles: {
    totalCircles: number;       // listening circles playing artist
    totalCircleListeners: number;
    avgCircleSize: number;
  };
  social: {
    mentions: number;
    sentiment: number;  // -1 to 1
  };
}

interface DateRange {
  start: Date;
  end: Date;
  period: 'day' | 'week' | 'month' | 'year' | 'all_time';
}

// ============================================================================
// Artist Stats Aggregator
// ============================================================================

export class ArtistStatsAggregator {
  private redis: Redis;
  private db: Pool;

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
  // Overview
  // ============================================================================

  async getOverview(artistId: string, range: DateRange): Promise<ArtistOverview> {
    const [currentStats, previousStats, revenue] = await Promise.all([
      this.getStatsForPeriod(artistId, range),
      this.getStatsForPeriod(artistId, this.getPreviousPeriod(range)),
      this.getRevenueForPeriod(artistId, range),
    ]);

    return {
      artistId,
      period: range,
      summary: {
        totalPlays: currentStats.plays,
        uniqueListeners: currentStats.uniqueListeners,
        totalListenTime: currentStats.listenTime,
        avgListenDuration: currentStats.plays > 0 ? currentStats.listenTime / currentStats.plays : 0,
        completionRate: currentStats.plays > 0
          ? (currentStats.completions / currentStats.plays) * 100
          : 0,
        saveRate: currentStats.plays > 0
          ? (currentStats.saves / currentStats.plays) * 100
          : 0,
        shareRate: currentStats.plays > 0
          ? (currentStats.shares / currentStats.plays) * 100
          : 0,
      },
      revenue: {
        total: revenue.total,
        streaming: revenue.streaming,
        patronage: revenue.patronage,
        nftSales: revenue.nftSales,
        licensing: revenue.licensing,
        tips: revenue.tips,
      },
      growth: {
        playsChange: this.calculateGrowth(currentStats.plays, previousStats.plays),
        listenersChange: this.calculateGrowth(currentStats.uniqueListeners, previousStats.uniqueListeners),
        revenueChange: this.calculateGrowth(revenue.total, previousStats.revenue),
        followersChange: this.calculateGrowth(currentStats.followers, previousStats.followers),
      },
    };
  }

  // ============================================================================
  // Audience
  // ============================================================================

  async getAudienceDemographics(artistId: string, range: DateRange): Promise<AudienceDemographics> {
    const result = await this.db.query(`
      SELECT
        listener_age_range,
        listener_gender,
        listener_country,
        listener_city,
        listen_hour,
        listen_day_of_week,
        device_type,
        is_premium,
        COUNT(DISTINCT listener_id) as listeners
      FROM stream_events se
      JOIN listener_profiles lp ON se.listener_id = lp.id
      WHERE se.artist_id = $1
        AND se.timestamp BETWEEN $2 AND $3
      GROUP BY
        listener_age_range,
        listener_gender,
        listener_country,
        listener_city,
        listen_hour,
        listen_day_of_week,
        device_type,
        is_premium
    `, [artistId, range.start, range.end]);

    // Process and aggregate results
    const ageRanges: Record<string, number> = {};
    const genders: Record<string, number> = {};
    const countries: Record<string, number> = {};
    const cities: Record<string, { city: string; country: string; count: number }> = {};
    const hourly = new Array(24).fill(0);
    const daily = new Array(7).fill(0);
    const devices: Record<string, number> = {};
    let premium = 0;
    let free = 0;

    for (const row of result.rows) {
      const listeners = parseInt(row.listeners);

      if (row.listener_age_range) {
        ageRanges[row.listener_age_range] = (ageRanges[row.listener_age_range] || 0) + listeners;
      }
      if (row.listener_gender) {
        genders[row.listener_gender] = (genders[row.listener_gender] || 0) + listeners;
      }
      if (row.listener_country) {
        countries[row.listener_country] = (countries[row.listener_country] || 0) + listeners;
      }
      if (row.listener_city) {
        const cityKey = `${row.listener_city}:${row.listener_country}`;
        if (!cities[cityKey]) {
          cities[cityKey] = { city: row.listener_city, country: row.listener_country, count: 0 };
        }
        cities[cityKey].count += listeners;
      }
      if (row.listen_hour !== null) {
        hourly[row.listen_hour] += listeners;
      }
      if (row.listen_day_of_week !== null) {
        daily[row.listen_day_of_week] += listeners;
      }
      if (row.device_type) {
        devices[row.device_type] = (devices[row.device_type] || 0) + listeners;
      }
      if (row.is_premium) {
        premium += listeners;
      } else {
        free += listeners;
      }
    }

    const totalListeners = Object.values(countries).reduce((a, b) => a + b, 0);

    return {
      ageRanges,
      genders,
      topCountries: Object.entries(countries)
        .sort((a, b) => b[1] - a[1])
        .slice(0, 10)
        .map(([country, listeners]) => ({
          country,
          listeners,
          percentage: totalListeners > 0 ? (listeners / totalListeners) * 100 : 0,
        })),
      topCities: Object.values(cities)
        .sort((a, b) => b.count - a.count)
        .slice(0, 10)
        .map(c => ({ city: c.city, country: c.country, listeners: c.count })),
      listeningTimes: {
        byHour: hourly,
        byDayOfWeek: daily,
      },
      deviceTypes: devices,
      premiumVsFree: { premium, free },
    };
  }

  // ============================================================================
  // Track Performance
  // ============================================================================

  async getTrackPerformance(artistId: string, range: DateRange): Promise<TrackPerformance[]> {
    const result = await this.db.query(`
      WITH track_stats AS (
        SELECT
          se.track_id,
          t.title,
          COUNT(*) as plays,
          COUNT(DISTINCT se.listener_id) as unique_listeners,
          SUM(CASE WHEN se.event_type = 'save' THEN 1 ELSE 0 END) as saves,
          SUM(CASE WHEN se.event_type = 'share' THEN 1 ELSE 0 END) as shares,
          SUM(CASE WHEN se.event_type = 'play_complete' THEN 1 ELSE 0 END) as completions,
          AVG(se.listen_duration) as avg_duration
        FROM stream_events se
        JOIN tracks t ON se.track_id = t.id
        WHERE se.artist_id = $1
          AND se.timestamp BETWEEN $2 AND $3
        GROUP BY se.track_id, t.title
      ),
      playlist_counts AS (
        SELECT
          track_id,
          COUNT(*) as playlist_placements
        FROM playlist_tracks pt
        JOIN playlists p ON pt.playlist_id = p.id
        WHERE pt.artist_id = $1
        GROUP BY track_id
      ),
      revenue AS (
        SELECT
          track_id,
          SUM(amount) as revenue
        FROM earnings
        WHERE artist_id = $1
          AND timestamp BETWEEN $2 AND $3
        GROUP BY track_id
      )
      SELECT
        ts.*,
        COALESCE(pc.playlist_placements, 0) as playlist_placements,
        COALESCE(r.revenue, 0) as revenue
      FROM track_stats ts
      LEFT JOIN playlist_counts pc ON ts.track_id = pc.track_id
      LEFT JOIN revenue r ON ts.track_id = r.track_id
      ORDER BY ts.plays DESC
    `, [artistId, range.start, range.end]);

    return result.rows.map(row => ({
      trackId: row.track_id,
      title: row.title,
      plays: parseInt(row.plays),
      uniqueListeners: parseInt(row.unique_listeners),
      saves: parseInt(row.saves),
      shares: parseInt(row.shares),
      completionRate: row.plays > 0 ? (parseInt(row.completions) / parseInt(row.plays)) * 100 : 0,
      avgListenDuration: parseFloat(row.avg_duration) || 0,
      revenue: parseFloat(row.revenue),
      playlistPlacements: parseInt(row.playlist_placements),
      trend: this.calculateTrend(row.track_id),
      peakPosition: 0, // Would come from chart data
    }));
  }

  // ============================================================================
  // Playlist Placements
  // ============================================================================

  async getPlaylistPlacements(artistId: string): Promise<PlaylistPlacement[]> {
    const result = await this.db.query(`
      SELECT
        p.id as playlist_id,
        p.name as playlist_name,
        p.curator_name as curator,
        p.follower_count as followers,
        pt.track_id,
        pt.added_at,
        pt.position,
        p.is_editorial,
        (
          SELECT COUNT(*)
          FROM stream_events se
          WHERE se.track_id = pt.track_id
            AND se.source_type = 'playlist'
            AND se.source_id = p.id
        ) as plays_from_playlist
      FROM playlist_tracks pt
      JOIN playlists p ON pt.playlist_id = p.id
      JOIN tracks t ON pt.track_id = t.id
      WHERE t.artist_id = $1
      ORDER BY p.follower_count DESC
    `, [artistId]);

    return result.rows.map(row => ({
      playlistId: row.playlist_id,
      playlistName: row.playlist_name,
      curator: row.curator,
      followers: parseInt(row.followers),
      trackId: row.track_id,
      addedAt: new Date(row.added_at),
      position: parseInt(row.position),
      playsFromPlaylist: parseInt(row.plays_from_playlist),
      isEditorial: row.is_editorial,
    }));
  }

  // ============================================================================
  // Revenue
  // ============================================================================

  async getRevenueBreakdown(artistId: string, range: DateRange): Promise<RevenueBreakdown> {
    const [earnings, payouts] = await Promise.all([
      this.db.query(`
        SELECT
          source_type,
          SUM(amount) as total,
          COUNT(*) as count,
          track_id
        FROM earnings
        WHERE artist_id = $1
          AND timestamp BETWEEN $2 AND $3
        GROUP BY source_type, track_id
      `, [artistId, range.start, range.end]),

      this.db.query(`
        SELECT scheduled_date, amount, status
        FROM payouts
        WHERE artist_id = $1
        ORDER BY scheduled_date DESC
        LIMIT 10
      `, [artistId]),
    ]);

    // Aggregate by source
    const bySource = {
      streaming: { amount: 0, plays: 0, ratePerPlay: 0 },
      patronage: { amount: 0, patrons: 0, avgPerPatron: 0 },
      nftSales: { amount: 0, sales: 0, avgPrice: 0 },
      licensing: { amount: 0, licenses: 0 },
      tips: { amount: 0, tippers: 0 },
    };

    const byTrack: Record<string, { trackId: string; title: string; amount: number }> = {};

    for (const row of earnings.rows) {
      const amount = parseFloat(row.total);
      const count = parseInt(row.count);

      switch (row.source_type) {
        case 'streaming':
          bySource.streaming.amount += amount;
          bySource.streaming.plays += count;
          break;
        case 'patronage':
          bySource.patronage.amount += amount;
          bySource.patronage.patrons = count;
          break;
        case 'nft':
          bySource.nftSales.amount += amount;
          bySource.nftSales.sales += count;
          break;
        case 'licensing':
          bySource.licensing.amount += amount;
          bySource.licensing.licenses += count;
          break;
        case 'tip':
          bySource.tips.amount += amount;
          bySource.tips.tippers += count;
          break;
      }

      if (row.track_id) {
        if (!byTrack[row.track_id]) {
          byTrack[row.track_id] = { trackId: row.track_id, title: '', amount: 0 };
        }
        byTrack[row.track_id].amount += amount;
      }
    }

    // Calculate rates
    if (bySource.streaming.plays > 0) {
      bySource.streaming.ratePerPlay = bySource.streaming.amount / bySource.streaming.plays;
    }
    if (bySource.patronage.patrons > 0) {
      bySource.patronage.avgPerPatron = bySource.patronage.amount / bySource.patronage.patrons;
    }
    if (bySource.nftSales.sales > 0) {
      bySource.nftSales.avgPrice = bySource.nftSales.amount / bySource.nftSales.sales;
    }

    const total = Object.values(bySource).reduce((sum, s) => sum + s.amount, 0);

    return {
      period: range,
      total,
      bySource,
      byTrack: Object.values(byTrack).sort((a, b) => b.amount - a.amount),
      projectedMonthly: this.projectMonthlyRevenue(total, range),
      payoutSchedule: payouts.rows.map(row => ({
        date: new Date(row.scheduled_date),
        amount: parseFloat(row.amount),
        status: row.status,
      })),
    };
  }

  // ============================================================================
  // Engagement
  // ============================================================================

  async getEngagementMetrics(artistId: string, range: DateRange): Promise<EngagementMetrics> {
    const [followers, engagement, circles] = await Promise.all([
      this.db.query(`
        SELECT
          (SELECT COUNT(*) FROM follows WHERE artist_id = $1 AND created_at <= $3) as total,
          (SELECT COUNT(*) FROM follows WHERE artist_id = $1 AND created_at BETWEEN $2 AND $3) as gained,
          (SELECT COUNT(*) FROM unfollows WHERE artist_id = $1 AND created_at BETWEEN $2 AND $3) as lost
      `, [artistId, range.start, range.end]),

      this.db.query(`
        SELECT
          SUM(CASE WHEN event_type = 'like' THEN 1 ELSE 0 END) as likes,
          SUM(CASE WHEN event_type = 'comment' THEN 1 ELSE 0 END) as comments,
          SUM(CASE WHEN event_type = 'share' THEN 1 ELSE 0 END) as shares,
          SUM(CASE WHEN event_type = 'save' THEN 1 ELSE 0 END) as saves,
          COUNT(DISTINCT listener_id) as unique_listeners
        FROM engagement_events
        WHERE artist_id = $1
          AND timestamp BETWEEN $2 AND $3
      `, [artistId, range.start, range.end]),

      this.db.query(`
        SELECT
          COUNT(DISTINCT c.id) as total_circles,
          SUM(c.listener_count) as total_listeners,
          AVG(c.listener_count) as avg_size
        FROM circles c
        JOIN circle_tracks ct ON c.id = ct.circle_id
        JOIN tracks t ON ct.track_id = t.id
        WHERE t.artist_id = $1
          AND c.is_active = true
      `, [artistId]),
    ]);

    const followerData = followers.rows[0];
    const engagementData = engagement.rows[0];
    const circleData = circles.rows[0];

    const totalEngagements =
      parseInt(engagementData.likes || 0) +
      parseInt(engagementData.comments || 0) +
      parseInt(engagementData.shares || 0) +
      parseInt(engagementData.saves || 0);

    const uniqueListeners = parseInt(engagementData.unique_listeners || 0);

    return {
      followers: {
        total: parseInt(followerData.total || 0),
        gained: parseInt(followerData.gained || 0),
        lost: parseInt(followerData.lost || 0),
        netChange: parseInt(followerData.gained || 0) - parseInt(followerData.lost || 0),
      },
      engagement: {
        likes: parseInt(engagementData.likes || 0),
        comments: parseInt(engagementData.comments || 0),
        shares: parseInt(engagementData.shares || 0),
        saves: parseInt(engagementData.saves || 0),
        rate: uniqueListeners > 0 ? (totalEngagements / uniqueListeners) * 100 : 0,
      },
      circles: {
        totalCircles: parseInt(circleData.total_circles || 0),
        totalCircleListeners: parseInt(circleData.total_listeners || 0),
        avgCircleSize: parseFloat(circleData.avg_size || 0),
      },
      social: {
        mentions: 0,  // Would come from social media integration
        sentiment: 0,
      },
    };
  }

  // ============================================================================
  // Real-Time Dashboard Data
  // ============================================================================

  async getRealTimeDashboard(artistId: string): Promise<{
    activeListeners: number;
    playsLast24h: number;
    topTrackNow: { trackId: string; title: string; listeners: number } | null;
    recentActivity: { type: string; data: any; timestamp: Date }[];
  }> {
    // Get active listeners from Redis
    const activeListeners = await this.redis.zcard(`artist:${artistId}:active`);

    // Get 24h plays
    const now = new Date();
    const yesterday = new Date(now.getTime() - 24 * 60 * 60 * 1000);
    const playsResult = await this.redis.get(`artist:${artistId}:plays:24h`);
    const playsLast24h = parseInt(playsResult || '0');

    // Get currently most-played track
    const topTracks = await this.redis.zrevrange(`artist:${artistId}:tracks:active`, 0, 0, 'WITHSCORES');
    let topTrackNow = null;
    if (topTracks.length >= 2) {
      topTrackNow = {
        trackId: topTracks[0],
        title: '', // Would fetch from DB
        listeners: parseInt(topTracks[1]),
      };
    }

    // Get recent activity stream
    const activityResult = await this.redis.lrange(`artist:${artistId}:activity`, 0, 20);
    const recentActivity = activityResult.map(item => JSON.parse(item));

    return {
      activeListeners,
      playsLast24h,
      topTrackNow,
      recentActivity,
    };
  }

  // ============================================================================
  // Helpers
  // ============================================================================

  private async getStatsForPeriod(artistId: string, range: DateRange): Promise<{
    plays: number;
    uniqueListeners: number;
    listenTime: number;
    completions: number;
    saves: number;
    shares: number;
    revenue: number;
    followers: number;
  }> {
    const result = await this.db.query(`
      SELECT
        COUNT(*) as plays,
        COUNT(DISTINCT listener_id) as unique_listeners,
        SUM(listen_duration) as listen_time,
        SUM(CASE WHEN event_type = 'play_complete' THEN 1 ELSE 0 END) as completions,
        SUM(CASE WHEN event_type = 'save' THEN 1 ELSE 0 END) as saves,
        SUM(CASE WHEN event_type = 'share' THEN 1 ELSE 0 END) as shares
      FROM stream_events
      WHERE artist_id = $1
        AND timestamp BETWEEN $2 AND $3
    `, [artistId, range.start, range.end]);

    const row = result.rows[0];
    return {
      plays: parseInt(row.plays || 0),
      uniqueListeners: parseInt(row.unique_listeners || 0),
      listenTime: parseFloat(row.listen_time || 0),
      completions: parseInt(row.completions || 0),
      saves: parseInt(row.saves || 0),
      shares: parseInt(row.shares || 0),
      revenue: 0,
      followers: 0,
    };
  }

  private async getRevenueForPeriod(artistId: string, range: DateRange): Promise<{
    total: number;
    streaming: number;
    patronage: number;
    nftSales: number;
    licensing: number;
    tips: number;
  }> {
    const result = await this.db.query(`
      SELECT
        source_type,
        SUM(amount) as total
      FROM earnings
      WHERE artist_id = $1
        AND timestamp BETWEEN $2 AND $3
      GROUP BY source_type
    `, [artistId, range.start, range.end]);

    const revenue = {
      total: 0,
      streaming: 0,
      patronage: 0,
      nftSales: 0,
      licensing: 0,
      tips: 0,
    };

    for (const row of result.rows) {
      const amount = parseFloat(row.total);
      revenue.total += amount;

      switch (row.source_type) {
        case 'streaming': revenue.streaming = amount; break;
        case 'patronage': revenue.patronage = amount; break;
        case 'nft': revenue.nftSales = amount; break;
        case 'licensing': revenue.licensing = amount; break;
        case 'tip': revenue.tips = amount; break;
      }
    }

    return revenue;
  }

  private getPreviousPeriod(range: DateRange): DateRange {
    const duration = range.end.getTime() - range.start.getTime();
    return {
      start: new Date(range.start.getTime() - duration),
      end: new Date(range.start.getTime()),
      period: range.period,
    };
  }

  private calculateGrowth(current: number, previous: number): number {
    if (previous === 0) return current > 0 ? 100 : 0;
    return ((current - previous) / previous) * 100;
  }

  private calculateTrend(trackId: string): 'rising' | 'stable' | 'declining' {
    // Would analyze recent play velocity
    return 'stable';
  }

  private projectMonthlyRevenue(periodRevenue: number, range: DateRange): number {
    const days = (range.end.getTime() - range.start.getTime()) / (1000 * 60 * 60 * 24);
    return (periodRevenue / days) * 30;
  }
}

export default ArtistStatsAggregator;
