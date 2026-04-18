// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Contextual Experience Engine
 *
 * Activity detection, location-aware recommendations, social listening,
 * and cross-platform continuity for personalized music experiences.
 */

import { EventEmitter } from 'events';

// ============================================================================
// Types - Activity Detection
// ============================================================================

export interface UserContext {
  userId: string;
  activity: ActivityState;
  location: LocationContext;
  time: TimeContext;
  device: DeviceContext;
  social: SocialContext;
  preferences: ContextualPreferences;
  lastUpdated: Date;
}

export interface ActivityState {
  current: ActivityType;
  confidence: number;
  duration: number;
  startedAt: Date;
  metrics?: ActivityMetrics;
  detectionSource: 'sensor' | 'manual' | 'schedule' | 'inferred';
}

export type ActivityType =
  | 'working'
  | 'exercising'
  | 'commuting'
  | 'relaxing'
  | 'sleeping'
  | 'studying'
  | 'cooking'
  | 'gaming'
  | 'socializing'
  | 'meditating'
  | 'driving'
  | 'walking'
  | 'running'
  | 'cycling'
  | 'unknown';

export interface ActivityMetrics {
  heartRate?: number;
  steps?: number;
  speed?: number;
  elevation?: number;
  cadence?: number;
}

export interface LocationContext {
  type: LocationType;
  coordinates?: { lat: number; lng: number };
  city?: string;
  country?: string;
  venue?: VenueInfo;
  isMoving: boolean;
  timezone: string;
}

export type LocationType =
  | 'home'
  | 'work'
  | 'gym'
  | 'transit'
  | 'cafe'
  | 'restaurant'
  | 'venue'
  | 'outdoor'
  | 'travel'
  | 'unknown';

export interface VenueInfo {
  id: string;
  name: string;
  type: string;
  hasLiveMusic: boolean;
  localArtists?: string[];
}

export interface TimeContext {
  hour: number;
  dayOfWeek: string;
  isWeekend: boolean;
  season: 'spring' | 'summer' | 'autumn' | 'winter';
  timeOfDay: 'morning' | 'afternoon' | 'evening' | 'night' | 'late_night';
  isHoliday: boolean;
  holidayName?: string;
}

export interface DeviceContext {
  type: 'phone' | 'tablet' | 'desktop' | 'tv' | 'speaker' | 'car' | 'wearable';
  platform: 'ios' | 'android' | 'web' | 'macos' | 'windows';
  audioOutput: AudioOutputType;
  batteryLevel?: number;
  isCharging?: boolean;
  networkType?: 'wifi' | 'cellular' | 'offline';
  screenOn: boolean;
}

export type AudioOutputType =
  | 'speaker'
  | 'headphones'
  | 'bluetooth_speaker'
  | 'bluetooth_headphones'
  | 'airpods'
  | 'car_audio'
  | 'tv'
  | 'casting';

export interface SocialContext {
  nearbyFriends: NearbyFriend[];
  activeCircles: string[];
  friendsListening: FriendListening[];
  trendingInNetwork: TrendingItem[];
  sharedPlaylists: string[];
}

export interface NearbyFriend {
  userId: string;
  username: string;
  distance: number;
  isListening: boolean;
  currentTrack?: string;
}

export interface FriendListening {
  userId: string;
  username: string;
  avatar: string;
  trackId: string;
  trackTitle: string;
  artist: string;
  startedAt: Date;
}

export interface TrendingItem {
  type: 'track' | 'artist' | 'playlist' | 'album';
  id: string;
  name: string;
  playCount: number;
  uniqueListeners: number;
}

export interface ContextualPreferences {
  activityPresets: ActivityPreset[];
  locationPresets: LocationPreset[];
  timePresets: TimePreset[];
  autoSwitch: boolean;
  notifyOnSwitch: boolean;
}

export interface ActivityPreset {
  activity: ActivityType;
  enabled: boolean;
  playlist?: string;
  genres?: string[];
  energy?: { min: number; max: number };
  tempo?: { min: number; max: number };
  instrumental?: boolean;
  customRules?: ContextRule[];
}

export interface LocationPreset {
  locationType: LocationType;
  enabled: boolean;
  playlist?: string;
  localArtistsBoost: number;
  genres?: string[];
}

export interface TimePreset {
  timeOfDay: TimeContext['timeOfDay'];
  dayType: 'weekday' | 'weekend' | 'any';
  enabled: boolean;
  playlist?: string;
  genres?: string[];
  energy?: { min: number; max: number };
}

export interface ContextRule {
  condition: {
    field: string;
    operator: 'equals' | 'greater' | 'less' | 'contains' | 'between';
    value: any;
  };
  action: {
    type: 'boost_genre' | 'filter_tempo' | 'prefer_instrumental' | 'prefer_local';
    parameters: any;
  };
}

// ============================================================================
// Types - Recommendations
// ============================================================================

export interface ContextualRecommendation {
  tracks: RecommendedTrack[];
  playlists: RecommendedPlaylist[];
  reasoning: RecommendationReasoning;
  alternatives: AlternativeRecommendation[];
}

export interface RecommendedTrack {
  trackId: string;
  title: string;
  artist: string;
  score: number;
  contextFactors: ContextFactor[];
}

export interface RecommendedPlaylist {
  playlistId: string;
  name: string;
  description: string;
  trackCount: number;
  score: number;
  contextMatch: string;
}

export interface ContextFactor {
  type: 'activity' | 'location' | 'time' | 'social' | 'mood' | 'history';
  weight: number;
  description: string;
}

export interface RecommendationReasoning {
  primaryFactor: string;
  explanation: string;
  confidence: number;
  alternativeExplanations: string[];
}

export interface AlternativeRecommendation {
  name: string;
  description: string;
  playlist?: string;
  genre?: string;
  reason: string;
}

// ============================================================================
// Types - Cross-Platform Continuity
// ============================================================================

export interface PlaybackSession {
  id: string;
  userId: string;
  deviceId: string;
  deviceType: DeviceContext['type'];
  trackId: string;
  position: number;
  isPlaying: boolean;
  queue: string[];
  queuePosition: number;
  context: {
    type: 'album' | 'playlist' | 'artist' | 'radio' | 'search';
    id: string;
    name: string;
  };
  startedAt: Date;
  lastUpdated: Date;
}

export interface HandoffRequest {
  fromDeviceId: string;
  toDeviceId: string;
  session: PlaybackSession;
  transferQueue: boolean;
  transferContext: boolean;
  seamless: boolean;
}

export interface DeviceCapabilities {
  deviceId: string;
  type: DeviceContext['type'];
  name: string;
  supportsPlayback: boolean;
  supportsControl: boolean;
  supportsHandoff: boolean;
  maxQuality: 'low' | 'normal' | 'high' | 'lossless';
  supportsGapless: boolean;
  supportsCrossfade: boolean;
  isOnline: boolean;
  lastSeen: Date;
}

// ============================================================================
// Contextual Engine Service
// ============================================================================

class ContextualEngineService extends EventEmitter {
  private userContexts: Map<string, UserContext> = new Map();
  private playbackSessions: Map<string, PlaybackSession> = new Map();
  private deviceRegistry: Map<string, DeviceCapabilities[]> = new Map();

  // ============================================================================
  // Context Detection
  // ============================================================================

  async updateContext(
    userId: string,
    updates: Partial<{
      activity: Partial<ActivityState>;
      location: Partial<LocationContext>;
      device: Partial<DeviceContext>;
      social: Partial<SocialContext>;
    }>
  ): Promise<UserContext> {
    let context = this.userContexts.get(userId);

    if (!context) {
      context = this.createDefaultContext(userId);
    }

    // Update activity
    if (updates.activity) {
      context.activity = {
        ...context.activity,
        ...updates.activity,
      };
    }

    // Update location
    if (updates.location) {
      context.location = {
        ...context.location,
        ...updates.location,
      };

      // Enrich with venue info if at a venue
      if (updates.location.coordinates) {
        context.location.venue = await this.detectVenue(updates.location.coordinates);
      }
    }

    // Update time context
    context.time = this.computeTimeContext();

    // Update device
    if (updates.device) {
      context.device = {
        ...context.device,
        ...updates.device,
      };
    }

    // Update social
    if (updates.social) {
      context.social = {
        ...context.social,
        ...updates.social,
      };
    } else {
      // Refresh social context
      context.social = await this.refreshSocialContext(userId);
    }

    context.lastUpdated = new Date();
    this.userContexts.set(userId, context);

    // Check if context change should trigger recommendation update
    this.emit('context_updated', { userId, context });

    return context;
  }

  async detectActivity(
    userId: string,
    sensorData: {
      accelerometer?: { x: number; y: number; z: number };
      heartRate?: number;
      location?: { lat: number; lng: number; speed: number };
      audioLevel?: number;
    }
  ): Promise<ActivityState> {
    let activity: ActivityType = 'unknown';
    let confidence = 0.5;
    const metrics: ActivityMetrics = {};

    // Heart rate based detection
    if (sensorData.heartRate) {
      metrics.heartRate = sensorData.heartRate;

      if (sensorData.heartRate > 140) {
        activity = 'exercising';
        confidence = 0.9;
      } else if (sensorData.heartRate < 60) {
        activity = 'sleeping';
        confidence = 0.7;
      }
    }

    // Motion based detection
    if (sensorData.accelerometer) {
      const magnitude = Math.sqrt(
        sensorData.accelerometer.x ** 2 +
        sensorData.accelerometer.y ** 2 +
        sensorData.accelerometer.z ** 2
      );

      if (magnitude > 15) {
        activity = 'running';
        confidence = 0.85;
      } else if (magnitude > 8) {
        activity = 'walking';
        confidence = 0.8;
      } else if (magnitude < 2) {
        if (activity !== 'sleeping') {
          activity = 'relaxing';
          confidence = 0.6;
        }
      }
    }

    // Speed based detection
    if (sensorData.location?.speed) {
      metrics.speed = sensorData.location.speed;

      if (sensorData.location.speed > 20) {
        activity = 'driving';
        confidence = 0.95;
      } else if (sensorData.location.speed > 15) {
        activity = 'cycling';
        confidence = 0.8;
      } else if (sensorData.location.speed > 2) {
        activity = 'commuting';
        confidence = 0.7;
      }
    }

    const context = this.userContexts.get(userId);
    const previousActivity = context?.activity;

    const activityState: ActivityState = {
      current: activity,
      confidence,
      duration: previousActivity?.current === activity
        ? previousActivity.duration + 1
        : 0,
      startedAt: previousActivity?.current === activity
        ? previousActivity.startedAt
        : new Date(),
      metrics,
      detectionSource: 'sensor',
    };

    // Update context with new activity
    await this.updateContext(userId, { activity: activityState });

    return activityState;
  }

  async setActivityManually(
    userId: string,
    activity: ActivityType,
    duration?: number
  ): Promise<void> {
    await this.updateContext(userId, {
      activity: {
        current: activity,
        confidence: 1,
        duration: duration || 0,
        startedAt: new Date(),
        detectionSource: 'manual',
      },
    });
  }

  private createDefaultContext(userId: string): UserContext {
    return {
      userId,
      activity: {
        current: 'unknown',
        confidence: 0,
        duration: 0,
        startedAt: new Date(),
        detectionSource: 'inferred',
      },
      location: {
        type: 'unknown',
        isMoving: false,
        timezone: Intl.DateTimeFormat().resolvedOptions().timeZone,
      },
      time: this.computeTimeContext(),
      device: {
        type: 'phone',
        platform: 'ios',
        audioOutput: 'speaker',
        screenOn: true,
      },
      social: {
        nearbyFriends: [],
        activeCircles: [],
        friendsListening: [],
        trendingInNetwork: [],
        sharedPlaylists: [],
      },
      preferences: {
        activityPresets: [],
        locationPresets: [],
        timePresets: [],
        autoSwitch: true,
        notifyOnSwitch: true,
      },
      lastUpdated: new Date(),
    };
  }

  private computeTimeContext(): TimeContext {
    const now = new Date();
    const hour = now.getHours();
    const days = ['Sunday', 'Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday'];
    const dayOfWeek = days[now.getDay()];
    const isWeekend = now.getDay() === 0 || now.getDay() === 6;

    let timeOfDay: TimeContext['timeOfDay'];
    if (hour >= 5 && hour < 12) timeOfDay = 'morning';
    else if (hour >= 12 && hour < 17) timeOfDay = 'afternoon';
    else if (hour >= 17 && hour < 21) timeOfDay = 'evening';
    else if (hour >= 21 || hour < 1) timeOfDay = 'night';
    else timeOfDay = 'late_night';

    const month = now.getMonth();
    let season: TimeContext['season'];
    if (month >= 2 && month <= 4) season = 'spring';
    else if (month >= 5 && month <= 7) season = 'summer';
    else if (month >= 8 && month <= 10) season = 'autumn';
    else season = 'winter';

    return {
      hour,
      dayOfWeek,
      isWeekend,
      season,
      timeOfDay,
      isHoliday: false, // Would check holiday calendar
    };
  }

  private async detectVenue(
    coordinates: { lat: number; lng: number }
  ): Promise<VenueInfo | undefined> {
    // Would query venue database / places API
    return undefined;
  }

  private async refreshSocialContext(userId: string): Promise<SocialContext> {
    // Would fetch from social service
    return {
      nearbyFriends: [],
      activeCircles: [],
      friendsListening: await this.getFriendsListening(userId),
      trendingInNetwork: await this.getTrendingInNetwork(userId),
      sharedPlaylists: [],
    };
  }

  private async getFriendsListening(userId: string): Promise<FriendListening[]> {
    // Would query active listening sessions of friends
    return [];
  }

  private async getTrendingInNetwork(userId: string): Promise<TrendingItem[]> {
    // Would analyze friend listening patterns
    return [];
  }

  // ============================================================================
  // Contextual Recommendations
  // ============================================================================

  async getContextualRecommendations(
    userId: string
  ): Promise<ContextualRecommendation> {
    const context = this.userContexts.get(userId);
    if (!context) {
      throw new Error('No context available for user');
    }

    const factors = this.computeContextFactors(context);
    const tracks = await this.recommendTracks(userId, context, factors);
    const playlists = await this.recommendPlaylists(userId, context);
    const reasoning = this.generateReasoning(context, factors);
    const alternatives = this.generateAlternatives(context);

    return {
      tracks,
      playlists,
      reasoning,
      alternatives,
    };
  }

  private computeContextFactors(context: UserContext): ContextFactor[] {
    const factors: ContextFactor[] = [];

    // Activity factor
    if (context.activity.current !== 'unknown' && context.activity.confidence > 0.5) {
      factors.push({
        type: 'activity',
        weight: context.activity.confidence * 0.4,
        description: `Currently ${context.activity.current}`,
      });
    }

    // Time factor
    factors.push({
      type: 'time',
      weight: 0.2,
      description: `${context.time.timeOfDay} on ${context.time.dayOfWeek}`,
    });

    // Location factor
    if (context.location.type !== 'unknown') {
      factors.push({
        type: 'location',
        weight: 0.2,
        description: `At ${context.location.type}`,
      });
    }

    // Social factor
    if (context.social.friendsListening.length > 0) {
      factors.push({
        type: 'social',
        weight: 0.15,
        description: `${context.social.friendsListening.length} friends listening`,
      });
    }

    // Normalize weights
    const totalWeight = factors.reduce((sum, f) => sum + f.weight, 0);
    factors.forEach(f => (f.weight = f.weight / totalWeight));

    return factors;
  }

  private async recommendTracks(
    userId: string,
    context: UserContext,
    factors: ContextFactor[]
  ): Promise<RecommendedTrack[]> {
    // Build query parameters based on context
    const queryParams = this.buildQueryParams(context);

    // Would query recommendation service
    const tracks: RecommendedTrack[] = [];

    // Check user's activity presets
    const preset = context.preferences.activityPresets.find(
      p => p.activity === context.activity.current && p.enabled
    );

    if (preset) {
      // Use preset preferences
      if (preset.playlist) {
        // Load tracks from preset playlist
      }
    }

    return tracks;
  }

  private buildQueryParams(context: UserContext): Record<string, any> {
    const params: Record<string, any> = {};

    // Activity-based params
    switch (context.activity.current) {
      case 'exercising':
      case 'running':
        params.minTempo = 120;
        params.minEnergy = 0.7;
        break;
      case 'working':
      case 'studying':
        params.maxEnergy = 0.6;
        params.instrumental = true;
        break;
      case 'sleeping':
      case 'meditating':
        params.maxTempo = 80;
        params.maxEnergy = 0.3;
        break;
      case 'driving':
        params.minEnergy = 0.5;
        break;
      case 'relaxing':
        params.maxEnergy = 0.5;
        params.maxTempo = 100;
        break;
    }

    // Time-based params
    switch (context.time.timeOfDay) {
      case 'morning':
        params.minValence = 0.5; // Upbeat
        break;
      case 'late_night':
        params.maxEnergy = 0.4;
        break;
    }

    // Location-based params
    if (context.location.type === 'gym') {
      params.minEnergy = 0.7;
      params.minTempo = 120;
    }

    return params;
  }

  private async recommendPlaylists(
    userId: string,
    context: UserContext
  ): Promise<RecommendedPlaylist[]> {
    const playlists: RecommendedPlaylist[] = [];

    // Activity-based playlist
    const activityPlaylistMap: Record<ActivityType, string> = {
      exercising: 'Workout Beats',
      running: 'Running Mix',
      studying: 'Focus Flow',
      working: 'Deep Work',
      sleeping: 'Sleep Sounds',
      meditating: 'Meditation',
      cooking: 'Kitchen Vibes',
      driving: 'Road Trip',
      commuting: 'Commute Mix',
      relaxing: 'Chill Out',
      gaming: 'Gaming Mode',
      socializing: 'Party Mix',
      walking: 'Walking Playlist',
      cycling: 'Cycling Beats',
      unknown: 'Discover Weekly',
    };

    const activityPlaylist = activityPlaylistMap[context.activity.current];
    if (activityPlaylist) {
      playlists.push({
        playlistId: `activity_${context.activity.current}`,
        name: activityPlaylist,
        description: `Perfect for ${context.activity.current}`,
        trackCount: 50,
        score: 0.9,
        contextMatch: 'activity',
      });
    }

    // Time-based playlist
    const timePlaylistMap: Record<TimeContext['timeOfDay'], string> = {
      morning: 'Morning Energy',
      afternoon: 'Afternoon Groove',
      evening: 'Evening Unwind',
      night: 'Night Vibes',
      late_night: 'Late Night',
    };

    playlists.push({
      playlistId: `time_${context.time.timeOfDay}`,
      name: timePlaylistMap[context.time.timeOfDay],
      description: `${context.time.dayOfWeek} ${context.time.timeOfDay} picks`,
      trackCount: 40,
      score: 0.7,
      contextMatch: 'time',
    });

    return playlists;
  }

  private generateReasoning(
    context: UserContext,
    factors: ContextFactor[]
  ): RecommendationReasoning {
    const primaryFactor = factors.sort((a, b) => b.weight - a.weight)[0];

    let explanation: string;

    switch (primaryFactor.type) {
      case 'activity':
        explanation = `Based on your ${context.activity.current} activity, we've selected music to match your energy.`;
        break;
      case 'time':
        explanation = `It's ${context.time.timeOfDay} on ${context.time.dayOfWeek} - here's music that fits the moment.`;
        break;
      case 'location':
        explanation = `Since you're at ${context.location.type}, we've curated music for the setting.`;
        break;
      case 'social':
        explanation = `Your friends are listening to similar vibes - join the wave!`;
        break;
      default:
        explanation = `Personalized picks based on your listening history.`;
    }

    return {
      primaryFactor: primaryFactor.type,
      explanation,
      confidence: primaryFactor.weight,
      alternativeExplanations: factors.slice(1).map(f => f.description),
    };
  }

  private generateAlternatives(context: UserContext): AlternativeRecommendation[] {
    const alternatives: AlternativeRecommendation[] = [];

    // Suggest mood alternatives
    alternatives.push({
      name: 'Energize',
      description: 'High-energy tracks to boost your mood',
      genre: 'electronic',
      reason: 'Sometimes you need an energy boost',
    });

    alternatives.push({
      name: 'Chill',
      description: 'Relaxing music to unwind',
      genre: 'ambient',
      reason: 'Take it easy with mellow sounds',
    });

    // Social alternative
    if (context.social.friendsListening.length > 0) {
      alternatives.push({
        name: 'Friend Vibes',
        description: "What your friends are listening to",
        reason: `${context.social.friendsListening[0].username} is listening now`,
      });
    }

    return alternatives;
  }

  // ============================================================================
  // Local Discovery
  // ============================================================================

  async discoverLocalArtists(
    userId: string
  ): Promise<{
    artists: { artistId: string; name: string; distance: number; genres: string[] }[];
    venues: VenueInfo[];
    events: { name: string; venue: string; date: Date; artists: string[] }[];
  }> {
    const context = this.userContexts.get(userId);
    if (!context?.location.coordinates) {
      throw new Error('Location not available');
    }

    // Would query local artists based on location
    return {
      artists: [],
      venues: [],
      events: [],
    };
  }

  // ============================================================================
  // Cross-Platform Continuity
  // ============================================================================

  async registerDevice(
    userId: string,
    device: DeviceCapabilities
  ): Promise<void> {
    let devices = this.deviceRegistry.get(userId);
    if (!devices) {
      devices = [];
      this.deviceRegistry.set(userId, devices);
    }

    const existingIndex = devices.findIndex(d => d.deviceId === device.deviceId);
    if (existingIndex >= 0) {
      devices[existingIndex] = device;
    } else {
      devices.push(device);
    }
  }

  async getAvailableDevices(userId: string): Promise<DeviceCapabilities[]> {
    return this.deviceRegistry.get(userId) || [];
  }

  async updatePlaybackSession(
    userId: string,
    session: Partial<PlaybackSession> & { deviceId: string }
  ): Promise<PlaybackSession> {
    const sessionKey = `${userId}_${session.deviceId}`;
    let existingSession = this.playbackSessions.get(sessionKey);

    if (!existingSession) {
      existingSession = {
        id: `session_${Date.now()}`,
        userId,
        deviceId: session.deviceId,
        deviceType: session.deviceType || 'phone',
        trackId: session.trackId || '',
        position: 0,
        isPlaying: false,
        queue: [],
        queuePosition: 0,
        context: session.context || { type: 'search', id: '', name: '' },
        startedAt: new Date(),
        lastUpdated: new Date(),
      };
    }

    Object.assign(existingSession, session, { lastUpdated: new Date() });
    this.playbackSessions.set(sessionKey, existingSession);

    this.emit('session_updated', { userId, session: existingSession });

    return existingSession;
  }

  async getActiveSession(userId: string): Promise<PlaybackSession | null> {
    const sessions = Array.from(this.playbackSessions.values())
      .filter(s => s.userId === userId && s.isPlaying);

    if (sessions.length === 0) {
      // Return most recent session
      const allSessions = Array.from(this.playbackSessions.values())
        .filter(s => s.userId === userId)
        .sort((a, b) => b.lastUpdated.getTime() - a.lastUpdated.getTime());

      return allSessions[0] || null;
    }

    return sessions[0];
  }

  async handoffPlayback(request: HandoffRequest): Promise<void> {
    const { fromDeviceId, toDeviceId, session, transferQueue, transferContext, seamless } = request;

    // Pause on source device
    await this.updatePlaybackSession(session.userId, {
      deviceId: fromDeviceId,
      isPlaying: false,
    });

    // Build new session for target device
    const newSession: Partial<PlaybackSession> = {
      deviceId: toDeviceId,
      trackId: session.trackId,
      position: seamless ? session.position : 0,
      isPlaying: true,
    };

    if (transferQueue) {
      newSession.queue = session.queue;
      newSession.queuePosition = session.queuePosition;
    }

    if (transferContext) {
      newSession.context = session.context;
    }

    await this.updatePlaybackSession(session.userId, newSession as any);

    this.emit('handoff_completed', {
      userId: session.userId,
      fromDevice: fromDeviceId,
      toDevice: toDeviceId,
    });
  }

  async syncPosition(
    userId: string,
    deviceId: string,
    position: number
  ): Promise<void> {
    await this.updatePlaybackSession(userId, {
      deviceId,
      position,
    });
  }

  // ============================================================================
  // Preferences Management
  // ============================================================================

  async setActivityPreset(
    userId: string,
    preset: ActivityPreset
  ): Promise<void> {
    const context = this.userContexts.get(userId);
    if (!context) return;

    const existingIndex = context.preferences.activityPresets.findIndex(
      p => p.activity === preset.activity
    );

    if (existingIndex >= 0) {
      context.preferences.activityPresets[existingIndex] = preset;
    } else {
      context.preferences.activityPresets.push(preset);
    }
  }

  async setLocationPreset(
    userId: string,
    preset: LocationPreset
  ): Promise<void> {
    const context = this.userContexts.get(userId);
    if (!context) return;

    const existingIndex = context.preferences.locationPresets.findIndex(
      p => p.locationType === preset.locationType
    );

    if (existingIndex >= 0) {
      context.preferences.locationPresets[existingIndex] = preset;
    } else {
      context.preferences.locationPresets.push(preset);
    }
  }

  async setTimePreset(
    userId: string,
    preset: TimePreset
  ): Promise<void> {
    const context = this.userContexts.get(userId);
    if (!context) return;

    const existingIndex = context.preferences.timePresets.findIndex(
      p => p.timeOfDay === preset.timeOfDay && p.dayType === preset.dayType
    );

    if (existingIndex >= 0) {
      context.preferences.timePresets[existingIndex] = preset;
    } else {
      context.preferences.timePresets.push(preset);
    }
  }

  getContext(userId: string): UserContext | undefined {
    return this.userContexts.get(userId);
  }
}

export const contextualEngine = new ContextualEngineService();
export default contextualEngine;
