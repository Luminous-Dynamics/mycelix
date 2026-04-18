// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Stream Collector
 *
 * Collects and processes streaming events in real-time for analytics.
 * Handles play events, skip events, completion rates, and listening patterns.
 */

import { Redis } from 'ioredis';
import { Kafka, Producer, Consumer } from 'kafkajs';

// ============================================================================
// Types
// ============================================================================

interface StreamEvent {
  eventId: string;
  eventType: 'play_start' | 'play_pause' | 'play_resume' | 'play_complete' | 'skip' | 'seek';
  trackId: string;
  releaseId: string;
  artistId: string;
  listenerId: string;
  timestamp: Date;
  position: number;        // Current position in seconds
  duration: number;        // Total track duration
  source: StreamSource;
  context: StreamContext;
  device: DeviceInfo;
  quality: AudioQuality;
}

interface StreamSource {
  type: 'direct' | 'playlist' | 'album' | 'radio' | 'search' | 'recommendation' | 'circle';
  id?: string;             // Playlist/album/circle ID if applicable
  position?: number;       // Position in queue
}

interface StreamContext {
  circleId?: string;       // If listening in a circle
  sessionId: string;       // Listening session ID
  isOffline: boolean;
  isPremium: boolean;
}

interface DeviceInfo {
  type: 'web' | 'mobile' | 'desktop' | 'smart_speaker' | 'tv';
  os: string;
  browser?: string;
  app_version?: string;
}

interface AudioQuality {
  bitrate: number;
  format: 'mp3' | 'aac' | 'flac' | 'opus';
  spatial: boolean;
}

interface AggregatedMetrics {
  trackId: string;
  period: 'minute' | 'hour' | 'day';
  timestamp: Date;
  plays: number;
  uniqueListeners: number;
  completions: number;
  skips: number;
  totalListenTime: number;
  avgListenPercentage: number;
  sources: Record<string, number>;
  devices: Record<string, number>;
  regions: Record<string, number>;
}

// ============================================================================
// Stream Collector
// ============================================================================

export class StreamCollector {
  private redis: Redis;
  private kafka: Kafka;
  private producer: Producer;
  private consumer: Consumer;
  private isRunning = false;

  // Batch settings for efficient writes
  private eventBuffer: StreamEvent[] = [];
  private readonly BATCH_SIZE = 100;
  private readonly FLUSH_INTERVAL = 5000; // 5 seconds

  constructor(
    private readonly config: {
      redisUrl: string;
      kafkaBrokers: string[];
      consumerGroup: string;
    }
  ) {
    this.redis = new Redis(config.redisUrl);
    this.kafka = new Kafka({
      clientId: 'stream-collector',
      brokers: config.kafkaBrokers,
    });
    this.producer = this.kafka.producer();
    this.consumer = this.kafka.consumer({ groupId: config.consumerGroup });
  }

  // ============================================================================
  // Lifecycle
  // ============================================================================

  async start(): Promise<void> {
    await this.producer.connect();
    await this.consumer.connect();

    // Subscribe to stream events topic
    await this.consumer.subscribe({ topic: 'stream-events', fromBeginning: false });

    // Start consuming
    this.isRunning = true;
    await this.consumer.run({
      eachMessage: async ({ message }) => {
        if (message.value) {
          const event = JSON.parse(message.value.toString()) as StreamEvent;
          await this.processEvent(event);
        }
      },
    });

    // Start periodic flush
    this.startFlushInterval();

    console.log('Stream collector started');
  }

  async stop(): Promise<void> {
    this.isRunning = false;
    await this.flushBuffer();
    await this.consumer.disconnect();
    await this.producer.disconnect();
    await this.redis.quit();
  }

  // ============================================================================
  // Event Collection
  // ============================================================================

  async collectEvent(event: StreamEvent): Promise<void> {
    // Publish to Kafka for distributed processing
    await this.producer.send({
      topic: 'stream-events',
      messages: [{ value: JSON.stringify(event) }],
    });
  }

  private async processEvent(event: StreamEvent): Promise<void> {
    // Add to buffer for batch processing
    this.eventBuffer.push(event);

    // Update real-time counters immediately
    await this.updateRealTimeCounters(event);

    // Flush if buffer is full
    if (this.eventBuffer.length >= this.BATCH_SIZE) {
      await this.flushBuffer();
    }
  }

  // ============================================================================
  // Real-Time Counters
  // ============================================================================

  private async updateRealTimeCounters(event: StreamEvent): Promise<void> {
    const now = new Date();
    const hourKey = this.getHourKey(now);
    const dayKey = this.getDayKey(now);
    const pipeline = this.redis.pipeline();

    // Track-level counters
    if (event.eventType === 'play_start') {
      // Increment play count
      pipeline.hincrby(`track:${event.trackId}:stats:${dayKey}`, 'plays', 1);
      pipeline.hincrby(`artist:${event.artistId}:stats:${dayKey}`, 'plays', 1);

      // Add to unique listeners set (HyperLogLog for memory efficiency)
      pipeline.pfadd(`track:${event.trackId}:listeners:${dayKey}`, event.listenerId);
      pipeline.pfadd(`artist:${event.artistId}:listeners:${dayKey}`, event.listenerId);

      // Track source distribution
      pipeline.hincrby(`track:${event.trackId}:sources:${dayKey}`, event.source.type, 1);

      // Track device distribution
      pipeline.hincrby(`track:${event.trackId}:devices:${dayKey}`, event.device.type, 1);

      // Real-time active listeners (sorted set with expiry)
      pipeline.zadd(`track:${event.trackId}:active`, Date.now(), event.listenerId);
      pipeline.zadd(`artist:${event.artistId}:active`, Date.now(), event.listenerId);

      // Global trending score (time-decayed)
      pipeline.zincrby('trending:tracks:hourly', 1, event.trackId);
      pipeline.zincrby('trending:artists:hourly', 1, event.artistId);
    }

    if (event.eventType === 'play_complete') {
      // Track completion
      pipeline.hincrby(`track:${event.trackId}:stats:${dayKey}`, 'completions', 1);

      // Listen time
      pipeline.hincrbyfloat(
        `track:${event.trackId}:stats:${dayKey}`,
        'listen_time',
        event.duration
      );
    }

    if (event.eventType === 'skip') {
      // Track skips with position data
      pipeline.hincrby(`track:${event.trackId}:stats:${dayKey}`, 'skips', 1);

      // Record skip position for heatmap (bucketed)
      const skipBucket = Math.floor((event.position / event.duration) * 10);
      pipeline.hincrby(`track:${event.trackId}:skips:${dayKey}`, `bucket_${skipBucket}`, 1);
    }

    // Clean up old active listeners (older than 5 minutes)
    const fiveMinutesAgo = Date.now() - 5 * 60 * 1000;
    pipeline.zremrangebyscore(`track:${event.trackId}:active`, 0, fiveMinutesAgo);
    pipeline.zremrangebyscore(`artist:${event.artistId}:active`, 0, fiveMinutesAgo);

    await pipeline.exec();
  }

  // ============================================================================
  // Batch Processing
  // ============================================================================

  private startFlushInterval(): void {
    setInterval(async () => {
      if (this.isRunning && this.eventBuffer.length > 0) {
        await this.flushBuffer();
      }
    }, this.FLUSH_INTERVAL);
  }

  private async flushBuffer(): Promise<void> {
    if (this.eventBuffer.length === 0) return;

    const events = [...this.eventBuffer];
    this.eventBuffer = [];

    // Aggregate events by track and time period
    const aggregated = this.aggregateEvents(events);

    // Write aggregated metrics to TimescaleDB (via message queue)
    await this.producer.send({
      topic: 'aggregated-metrics',
      messages: aggregated.map(m => ({ value: JSON.stringify(m) })),
    });

    // Update engagement heatmaps
    await this.updateHeatmaps(events);
  }

  private aggregateEvents(events: StreamEvent[]): AggregatedMetrics[] {
    const aggregated = new Map<string, AggregatedMetrics>();

    for (const event of events) {
      const key = `${event.trackId}:${this.getMinuteKey(event.timestamp)}`;

      if (!aggregated.has(key)) {
        aggregated.set(key, {
          trackId: event.trackId,
          period: 'minute',
          timestamp: new Date(event.timestamp),
          plays: 0,
          uniqueListeners: 0,
          completions: 0,
          skips: 0,
          totalListenTime: 0,
          avgListenPercentage: 0,
          sources: {},
          devices: {},
          regions: {},
        });
      }

      const metrics = aggregated.get(key)!;

      if (event.eventType === 'play_start') {
        metrics.plays++;
        metrics.sources[event.source.type] = (metrics.sources[event.source.type] || 0) + 1;
        metrics.devices[event.device.type] = (metrics.devices[event.device.type] || 0) + 1;
      }

      if (event.eventType === 'play_complete') {
        metrics.completions++;
        metrics.totalListenTime += event.duration;
      }

      if (event.eventType === 'skip') {
        metrics.skips++;
        metrics.totalListenTime += event.position;
      }
    }

    // Calculate averages
    for (const metrics of aggregated.values()) {
      if (metrics.plays > 0) {
        metrics.avgListenPercentage =
          ((metrics.completions * 100) + (metrics.skips * 50)) / metrics.plays;
      }
    }

    return Array.from(aggregated.values());
  }

  // ============================================================================
  // Engagement Heatmaps
  // ============================================================================

  private async updateHeatmaps(events: StreamEvent[]): Promise<void> {
    const pipeline = this.redis.pipeline();

    // Group events by track
    const byTrack = new Map<string, StreamEvent[]>();
    for (const event of events) {
      if (!byTrack.has(event.trackId)) {
        byTrack.set(event.trackId, []);
      }
      byTrack.get(event.trackId)!.push(event);
    }

    // Update heatmaps for each track
    for (const [trackId, trackEvents] of byTrack) {
      // Listening heatmap (where people listen most)
      for (const event of trackEvents) {
        if (event.eventType === 'play_start' || event.eventType === 'play_resume') {
          const bucket = Math.floor((event.position / event.duration) * 100);
          pipeline.hincrby(`track:${trackId}:heatmap:listen`, `${bucket}`, 1);
        }
      }

      // Skip heatmap (where people skip)
      for (const event of trackEvents) {
        if (event.eventType === 'skip') {
          const bucket = Math.floor((event.position / event.duration) * 100);
          pipeline.hincrby(`track:${trackId}:heatmap:skip`, `${bucket}`, 1);
        }
      }
    }

    await pipeline.exec();
  }

  // ============================================================================
  // Query Methods
  // ============================================================================

  async getTrackStats(trackId: string, days: number = 7): Promise<Record<string, any>> {
    const stats: Record<string, any> = {
      daily: [],
      totals: { plays: 0, completions: 0, skips: 0, listenTime: 0 },
      sources: {},
      devices: {},
    };

    for (let i = 0; i < days; i++) {
      const date = new Date();
      date.setDate(date.getDate() - i);
      const dayKey = this.getDayKey(date);

      const [dailyStats, sources, devices, uniqueListeners] = await Promise.all([
        this.redis.hgetall(`track:${trackId}:stats:${dayKey}`),
        this.redis.hgetall(`track:${trackId}:sources:${dayKey}`),
        this.redis.hgetall(`track:${trackId}:devices:${dayKey}`),
        this.redis.pfcount(`track:${trackId}:listeners:${dayKey}`),
      ]);

      const dayData = {
        date: dayKey,
        plays: parseInt(dailyStats.plays || '0'),
        completions: parseInt(dailyStats.completions || '0'),
        skips: parseInt(dailyStats.skips || '0'),
        listenTime: parseFloat(dailyStats.listen_time || '0'),
        uniqueListeners,
      };

      stats.daily.push(dayData);
      stats.totals.plays += dayData.plays;
      stats.totals.completions += dayData.completions;
      stats.totals.skips += dayData.skips;
      stats.totals.listenTime += dayData.listenTime;

      // Aggregate sources and devices
      for (const [source, count] of Object.entries(sources)) {
        stats.sources[source] = (stats.sources[source] || 0) + parseInt(count);
      }
      for (const [device, count] of Object.entries(devices)) {
        stats.devices[device] = (stats.devices[device] || 0) + parseInt(count);
      }
    }

    return stats;
  }

  async getActiveListeners(entityType: 'track' | 'artist', entityId: string): Promise<number> {
    return this.redis.zcard(`${entityType}:${entityId}:active`);
  }

  async getEngagementHeatmap(trackId: string): Promise<{
    listen: number[];
    skip: number[];
  }> {
    const [listen, skip] = await Promise.all([
      this.redis.hgetall(`track:${trackId}:heatmap:listen`),
      this.redis.hgetall(`track:${trackId}:heatmap:skip`),
    ]);

    const listenHeatmap = new Array(100).fill(0);
    const skipHeatmap = new Array(100).fill(0);

    for (const [bucket, count] of Object.entries(listen)) {
      listenHeatmap[parseInt(bucket)] = parseInt(count);
    }
    for (const [bucket, count] of Object.entries(skip)) {
      skipHeatmap[parseInt(bucket)] = parseInt(count);
    }

    return { listen: listenHeatmap, skip: skipHeatmap };
  }

  async getTrendingTracks(limit: number = 50): Promise<string[]> {
    return this.redis.zrevrange('trending:tracks:hourly', 0, limit - 1);
  }

  // ============================================================================
  // Helpers
  // ============================================================================

  private getMinuteKey(date: Date): string {
    return `${date.getFullYear()}-${String(date.getMonth() + 1).padStart(2, '0')}-${String(date.getDate()).padStart(2, '0')}-${String(date.getHours()).padStart(2, '0')}-${String(date.getMinutes()).padStart(2, '0')}`;
  }

  private getHourKey(date: Date): string {
    return `${date.getFullYear()}-${String(date.getMonth() + 1).padStart(2, '0')}-${String(date.getDate()).padStart(2, '0')}-${String(date.getHours()).padStart(2, '0')}`;
  }

  private getDayKey(date: Date): string {
    return `${date.getFullYear()}-${String(date.getMonth() + 1).padStart(2, '0')}-${String(date.getDate()).padStart(2, '0')}`;
  }
}

export default StreamCollector;
