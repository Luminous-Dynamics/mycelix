// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Fan Intelligence & Advanced Analytics
 *
 * Deep analytics for artists including fan demographics, geographic insights,
 * engagement funnels, predictive analytics, and market trends.
 */

// ============================================================================
// Types
// ============================================================================

export interface FanProfile {
  id: string;
  demographicSegment: DemographicSegment;
  engagementScore: number;
  lifetimeValue: number;
  acquisitionSource: string;
  firstInteraction: Date;
  lastInteraction: Date;
  totalStreams: number;
  totalPurchases: number;
  preferredGenres: string[];
  listeningPatterns: ListeningPattern;
  socialConnections: number;
  influenceScore: number;
}

export interface DemographicSegment {
  ageRange: '13-17' | '18-24' | '25-34' | '35-44' | '45-54' | '55+';
  gender?: 'male' | 'female' | 'non-binary' | 'unknown';
  location: {
    country: string;
    region?: string;
    city?: string;
    timezone: string;
  };
  language: string;
  platform: 'ios' | 'android' | 'web' | 'desktop';
}

export interface ListeningPattern {
  peakHours: number[];
  peakDays: string[];
  averageSessionLength: number;
  skipRate: number;
  repeatRate: number;
  playlistAddRate: number;
  shareRate: number;
}

export interface GeographicHeatmap {
  regions: GeographicRegion[];
  hotspots: Hotspot[];
  growthAreas: GrowthArea[];
}

export interface GeographicRegion {
  id: string;
  name: string;
  country: string;
  listeners: number;
  streams: number;
  revenue: number;
  growth: number;
  coordinates: { lat: number; lng: number };
  density: number;
}

export interface Hotspot {
  city: string;
  country: string;
  listeners: number;
  growthRate: number;
  coordinates: { lat: number; lng: number };
  topTracks: string[];
}

export interface GrowthArea {
  region: string;
  currentListeners: number;
  projectedListeners: number;
  growthRate: number;
  recommendedActions: string[];
}

export interface EngagementFunnel {
  stages: FunnelStage[];
  conversionRates: ConversionRate[];
  dropoffPoints: DropoffPoint[];
  recommendations: FunnelRecommendation[];
}

export interface FunnelStage {
  name: string;
  users: number;
  percentage: number;
  averageTimeInStage: number;
}

export interface ConversionRate {
  from: string;
  to: string;
  rate: number;
  trend: 'up' | 'down' | 'stable';
  benchmark: number;
}

export interface DropoffPoint {
  stage: string;
  dropoffRate: number;
  commonReasons: string[];
  suggestedFixes: string[];
}

export interface FunnelRecommendation {
  priority: 'high' | 'medium' | 'low';
  stage: string;
  recommendation: string;
  expectedImpact: number;
  effort: 'low' | 'medium' | 'high';
}

export interface PredictiveInsights {
  releaseTimingOptimization: ReleaseTimingPrediction;
  viralPotentialScores: ViralPotentialScore[];
  churnRisk: ChurnRiskAnalysis;
  revenueForecasts: RevenueForecast[];
  audienceGrowthPrediction: GrowthPrediction;
}

export interface ReleaseTimingPrediction {
  optimalDay: string;
  optimalHour: number;
  optimalTimezone: string;
  competitionAnalysis: {
    lowCompetitionWindows: { start: Date; end: Date }[];
    highRiskPeriods: { start: Date; end: Date; reason: string }[];
  };
  audienceReadiness: number;
  predictedFirstWeekStreams: {
    low: number;
    expected: number;
    high: number;
  };
}

export interface ViralPotentialScore {
  trackId: string;
  title: string;
  score: number;
  factors: {
    hookStrength: number;
    shareability: number;
    trendAlignment: number;
    audienceMatch: number;
    productionQuality: number;
  };
  recommendations: string[];
  similarViralTracks: string[];
}

export interface ChurnRiskAnalysis {
  atRiskFans: number;
  totalFans: number;
  riskSegments: {
    segment: string;
    count: number;
    riskLevel: 'high' | 'medium' | 'low';
    interventionSuggestion: string;
  }[];
  predictedChurn30Days: number;
  retentionOpportunities: string[];
}

export interface RevenueForecast {
  period: string;
  streamingRevenue: { low: number; expected: number; high: number };
  merchandiseRevenue: { low: number; expected: number; high: number };
  patronRevenue: { low: number; expected: number; high: number };
  totalRevenue: { low: number; expected: number; high: number };
  confidence: number;
}

export interface GrowthPrediction {
  currentFollowers: number;
  predicted30Days: number;
  predicted90Days: number;
  predicted1Year: number;
  growthDrivers: string[];
  growthBlockers: string[];
  recommendedActions: string[];
}

export interface MarketTrends {
  genreMomentum: GenreTrend[];
  emergingArtists: EmergingArtist[];
  soundTrends: SoundTrend[];
  platformTrends: PlatformTrend[];
  seasonalPatterns: SeasonalPattern[];
}

export interface GenreTrend {
  genre: string;
  momentum: number;
  growth7Day: number;
  growth30Day: number;
  growth90Day: number;
  topArtists: string[];
  subgenreBreakdown: { name: string; share: number }[];
  predictedPeak: Date | null;
}

export interface EmergingArtist {
  artistId: string;
  name: string;
  genre: string;
  growthRate: number;
  currentMonthlyListeners: number;
  predictedMonthlyListeners: number;
  breakoutProbability: number;
  similarTo: string[];
  keyMetrics: {
    engagementRate: number;
    saveRate: number;
    playlistAdditions: number;
  };
}

export interface SoundTrend {
  name: string;
  description: string;
  examples: string[];
  popularity: number;
  growthRate: number;
  relatedGenres: string[];
  audioCharacteristics: {
    tempo: { min: number; max: number };
    energy: number;
    valence: number;
    danceability: number;
  };
}

export interface PlatformTrend {
  platform: string;
  trend: string;
  impact: 'high' | 'medium' | 'low';
  actionableInsight: string;
  relevantFor: string[];
}

export interface SeasonalPattern {
  period: string;
  genres: { genre: string; multiplier: number }[];
  listeningBehavior: string;
  recommendations: string[];
}

export interface ABTest {
  id: string;
  name: string;
  type: 'artwork' | 'title' | 'release_time' | 'description' | 'pricing';
  status: 'draft' | 'running' | 'completed' | 'cancelled';
  variants: ABTestVariant[];
  targetAudience: string;
  sampleSize: number;
  currentParticipants: number;
  startDate: Date;
  endDate?: Date;
  results?: ABTestResults;
}

export interface ABTestVariant {
  id: string;
  name: string;
  content: any;
  participants: number;
  conversions: number;
  conversionRate: number;
}

export interface ABTestResults {
  winner: string;
  confidence: number;
  improvement: number;
  statisticalSignificance: boolean;
  insights: string[];
}

// ============================================================================
// Fan Intelligence Service
// ============================================================================

class FanIntelligenceService {
  private fanProfiles: Map<string, Map<string, FanProfile>> = new Map();
  private marketTrendsCache: MarketTrends | null = null;
  private trendsCacheExpiry: Date | null = null;

  // ============================================================================
  // Fan Demographics & Profiles
  // ============================================================================

  async getFanDemographics(artistId: string): Promise<{
    totalFans: number;
    segments: {
      byAge: { range: string; count: number; percentage: number }[];
      byGender: { gender: string; count: number; percentage: number }[];
      byCountry: { country: string; count: number; percentage: number }[];
      byPlatform: { platform: string; count: number; percentage: number }[];
    };
    trends: {
      fastestGrowingSegment: string;
      highestValueSegment: string;
      mostEngagedSegment: string;
    };
  }> {
    const fans = await this.getArtistFans(artistId);
    const totalFans = fans.length;

    // Aggregate by age
    const byAge = this.aggregateByField(fans, f => f.demographicSegment.ageRange);

    // Aggregate by gender
    const byGender = this.aggregateByField(
      fans,
      f => f.demographicSegment.gender || 'unknown'
    );

    // Aggregate by country
    const byCountry = this.aggregateByField(
      fans,
      f => f.demographicSegment.location.country
    );

    // Aggregate by platform
    const byPlatform = this.aggregateByField(
      fans,
      f => f.demographicSegment.platform
    );

    // Find trends
    const fastestGrowingSegment = await this.findFastestGrowingSegment(artistId);
    const highestValueSegment = this.findHighestValueSegment(fans);
    const mostEngagedSegment = this.findMostEngagedSegment(fans);

    return {
      totalFans,
      segments: {
        byAge: byAge.map(([range, count]) => ({
          range,
          count,
          percentage: (count / totalFans) * 100,
        })),
        byGender: byGender.map(([gender, count]) => ({
          gender,
          count,
          percentage: (count / totalFans) * 100,
        })),
        byCountry: byCountry.map(([country, count]) => ({
          country,
          count,
          percentage: (count / totalFans) * 100,
        })),
        byPlatform: byPlatform.map(([platform, count]) => ({
          platform,
          count,
          percentage: (count / totalFans) * 100,
        })),
      },
      trends: {
        fastestGrowingSegment,
        highestValueSegment,
        mostEngagedSegment,
      },
    };
  }

  async getGeographicHeatmap(artistId: string): Promise<GeographicHeatmap> {
    const fans = await this.getArtistFans(artistId);

    // Group by region
    const regionMap = new Map<string, FanProfile[]>();
    for (const fan of fans) {
      const region = `${fan.demographicSegment.location.country}-${fan.demographicSegment.location.region || 'unknown'}`;
      if (!regionMap.has(region)) {
        regionMap.set(region, []);
      }
      regionMap.get(region)!.push(fan);
    }

    const regions: GeographicRegion[] = [];
    for (const [regionId, regionFans] of regionMap) {
      const [country, region] = regionId.split('-');
      const streams = regionFans.reduce((sum, f) => sum + f.totalStreams, 0);
      const revenue = regionFans.reduce((sum, f) => sum + f.lifetimeValue, 0);

      regions.push({
        id: regionId,
        name: region,
        country,
        listeners: regionFans.length,
        streams,
        revenue,
        growth: await this.calculateRegionGrowth(artistId, regionId),
        coordinates: await this.getRegionCoordinates(country, region),
        density: regionFans.length / 1000, // Normalized density
      });
    }

    // Find hotspots (cities with high concentration)
    const cityMap = new Map<string, FanProfile[]>();
    for (const fan of fans) {
      const city = fan.demographicSegment.location.city;
      if (city) {
        if (!cityMap.has(city)) {
          cityMap.set(city, []);
        }
        cityMap.get(city)!.push(fan);
      }
    }

    const hotspots: Hotspot[] = Array.from(cityMap.entries())
      .filter(([_, cityFans]) => cityFans.length >= 100)
      .map(([city, cityFans]) => ({
        city,
        country: cityFans[0].demographicSegment.location.country,
        listeners: cityFans.length,
        growthRate: 0.15, // Would calculate actual growth
        coordinates: { lat: 0, lng: 0 }, // Would geocode
        topTracks: [], // Would analyze
      }))
      .sort((a, b) => b.listeners - a.listeners)
      .slice(0, 20);

    // Identify growth areas
    const growthAreas: GrowthArea[] = regions
      .filter(r => r.growth > 0.1)
      .map(r => ({
        region: `${r.name}, ${r.country}`,
        currentListeners: r.listeners,
        projectedListeners: Math.round(r.listeners * (1 + r.growth)),
        growthRate: r.growth,
        recommendedActions: this.getGrowthRecommendations(r),
      }))
      .sort((a, b) => b.growthRate - a.growthRate)
      .slice(0, 10);

    return { regions, hotspots, growthAreas };
  }

  // ============================================================================
  // Engagement Funnel Analysis
  // ============================================================================

  async getEngagementFunnel(artistId: string): Promise<EngagementFunnel> {
    const stats = await this.getArtistFunnelStats(artistId);

    const stages: FunnelStage[] = [
      {
        name: 'Discovery',
        users: stats.discovered,
        percentage: 100,
        averageTimeInStage: 0,
      },
      {
        name: 'First Listen',
        users: stats.firstListen,
        percentage: (stats.firstListen / stats.discovered) * 100,
        averageTimeInStage: 2.5,
      },
      {
        name: 'Repeat Listen',
        users: stats.repeatListen,
        percentage: (stats.repeatListen / stats.discovered) * 100,
        averageTimeInStage: 7,
      },
      {
        name: 'Save/Follow',
        users: stats.saved,
        percentage: (stats.saved / stats.discovered) * 100,
        averageTimeInStage: 14,
      },
      {
        name: 'Engaged Fan',
        users: stats.engaged,
        percentage: (stats.engaged / stats.discovered) * 100,
        averageTimeInStage: 30,
      },
      {
        name: 'Super Fan',
        users: stats.superFan,
        percentage: (stats.superFan / stats.discovered) * 100,
        averageTimeInStage: 90,
      },
    ];

    const conversionRates: ConversionRate[] = [];
    for (let i = 0; i < stages.length - 1; i++) {
      const rate = stages[i + 1].users / stages[i].users;
      conversionRates.push({
        from: stages[i].name,
        to: stages[i + 1].name,
        rate,
        trend: rate > 0.5 ? 'up' : rate < 0.3 ? 'down' : 'stable',
        benchmark: this.getBenchmarkConversion(stages[i].name, stages[i + 1].name),
      });
    }

    const dropoffPoints: DropoffPoint[] = conversionRates
      .filter(cr => cr.rate < cr.benchmark * 0.8)
      .map(cr => ({
        stage: cr.from,
        dropoffRate: 1 - cr.rate,
        commonReasons: this.getDropoffReasons(cr.from),
        suggestedFixes: this.getDropoffFixes(cr.from),
      }));

    const recommendations = this.generateFunnelRecommendations(
      stages,
      conversionRates,
      dropoffPoints
    );

    return { stages, conversionRates, dropoffPoints, recommendations };
  }

  // ============================================================================
  // Predictive Analytics
  // ============================================================================

  async getPredictiveInsights(artistId: string): Promise<PredictiveInsights> {
    const [
      releaseTimingOptimization,
      viralPotentialScores,
      churnRisk,
      revenueForecasts,
      audienceGrowthPrediction,
    ] = await Promise.all([
      this.predictOptimalReleaseTime(artistId),
      this.analyzeViralPotential(artistId),
      this.analyzeChurnRisk(artistId),
      this.forecastRevenue(artistId),
      this.predictAudienceGrowth(artistId),
    ]);

    return {
      releaseTimingOptimization,
      viralPotentialScores,
      churnRisk,
      revenueForecasts,
      audienceGrowthPrediction,
    };
  }

  private async predictOptimalReleaseTime(
    artistId: string
  ): Promise<ReleaseTimingPrediction> {
    const fans = await this.getArtistFans(artistId);

    // Analyze fan listening patterns
    const hourCounts = new Array(24).fill(0);
    const dayCounts = new Array(7).fill(0);

    for (const fan of fans) {
      for (const hour of fan.listeningPatterns.peakHours) {
        hourCounts[hour]++;
      }
      for (const day of fan.listeningPatterns.peakDays) {
        const dayIndex = ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat'].indexOf(day);
        if (dayIndex >= 0) dayCounts[dayIndex]++;
      }
    }

    const optimalHour = hourCounts.indexOf(Math.max(...hourCounts));
    const optimalDayIndex = dayCounts.indexOf(Math.max(...dayCounts));
    const days = ['Sunday', 'Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday'];

    // Find low competition windows (would analyze market data)
    const lowCompetitionWindows = [
      {
        start: new Date(Date.now() + 7 * 86400000),
        end: new Date(Date.now() + 14 * 86400000),
      },
    ];

    // Predict first week streams based on historical data
    const avgFirstWeekStreams = 10000; // Would calculate from history
    const audienceMultiplier = fans.length / 1000;

    return {
      optimalDay: days[optimalDayIndex],
      optimalHour,
      optimalTimezone: this.getMostCommonTimezone(fans),
      competitionAnalysis: {
        lowCompetitionWindows,
        highRiskPeriods: [],
      },
      audienceReadiness: 0.85,
      predictedFirstWeekStreams: {
        low: Math.round(avgFirstWeekStreams * audienceMultiplier * 0.7),
        expected: Math.round(avgFirstWeekStreams * audienceMultiplier),
        high: Math.round(avgFirstWeekStreams * audienceMultiplier * 1.5),
      },
    };
  }

  private async analyzeViralPotential(
    artistId: string
  ): Promise<ViralPotentialScore[]> {
    // Would analyze unreleased tracks
    // For now, return analysis of recent tracks
    const tracks = await this.getArtistRecentTracks(artistId);

    return tracks.map(track => ({
      trackId: track.id,
      title: track.title,
      score: this.calculateViralScore(track),
      factors: {
        hookStrength: 0.8,
        shareability: 0.75,
        trendAlignment: 0.7,
        audienceMatch: 0.85,
        productionQuality: 0.9,
      },
      recommendations: [
        'Consider creating a TikTok-friendly 15-second clip',
        'Hook at 0:15 is strong - feature in previews',
        'Similar to trending sounds in your genre',
      ],
      similarViralTracks: [],
    }));
  }

  private async analyzeChurnRisk(artistId: string): Promise<ChurnRiskAnalysis> {
    const fans = await this.getArtistFans(artistId);
    const now = new Date();

    const atRiskFans = fans.filter(fan => {
      const daysSinceActive = (now.getTime() - fan.lastInteraction.getTime()) / 86400000;
      return daysSinceActive > 30 && fan.engagementScore < 0.3;
    });

    const riskSegments = [
      {
        segment: 'Inactive 30+ days',
        count: atRiskFans.filter(f => {
          const days = (now.getTime() - f.lastInteraction.getTime()) / 86400000;
          return days > 30 && days <= 60;
        }).length,
        riskLevel: 'medium' as const,
        interventionSuggestion: 'Send re-engagement notification with new content',
      },
      {
        segment: 'Inactive 60+ days',
        count: atRiskFans.filter(f => {
          const days = (now.getTime() - f.lastInteraction.getTime()) / 86400000;
          return days > 60;
        }).length,
        riskLevel: 'high' as const,
        interventionSuggestion: 'Exclusive content offer or personalized playlist',
      },
      {
        segment: 'Declining engagement',
        count: fans.filter(f => f.engagementScore < 0.2).length,
        riskLevel: 'medium' as const,
        interventionSuggestion: 'Analyze content preferences and adjust',
      },
    ];

    return {
      atRiskFans: atRiskFans.length,
      totalFans: fans.length,
      riskSegments,
      predictedChurn30Days: Math.round(atRiskFans.length * 0.3),
      retentionOpportunities: [
        'Release exclusive content for loyal fans',
        'Host a live Q&A session',
        'Create personalized year-in-review for top fans',
      ],
    };
  }

  private async forecastRevenue(artistId: string): Promise<RevenueForecast[]> {
    const months = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun'];
    const baseStreaming = 5000;
    const baseMerch = 2000;
    const basePatron = 3000;

    return months.map((month, i) => {
      const growthFactor = 1 + i * 0.05;

      return {
        period: month,
        streamingRevenue: {
          low: Math.round(baseStreaming * growthFactor * 0.8),
          expected: Math.round(baseStreaming * growthFactor),
          high: Math.round(baseStreaming * growthFactor * 1.3),
        },
        merchandiseRevenue: {
          low: Math.round(baseMerch * growthFactor * 0.7),
          expected: Math.round(baseMerch * growthFactor),
          high: Math.round(baseMerch * growthFactor * 1.5),
        },
        patronRevenue: {
          low: Math.round(basePatron * growthFactor * 0.9),
          expected: Math.round(basePatron * growthFactor),
          high: Math.round(basePatron * growthFactor * 1.2),
        },
        totalRevenue: {
          low: Math.round((baseStreaming + baseMerch + basePatron) * growthFactor * 0.8),
          expected: Math.round((baseStreaming + baseMerch + basePatron) * growthFactor),
          high: Math.round((baseStreaming + baseMerch + basePatron) * growthFactor * 1.4),
        },
        confidence: 0.85 - i * 0.05,
      };
    });
  }

  private async predictAudienceGrowth(artistId: string): Promise<GrowthPrediction> {
    const fans = await this.getArtistFans(artistId);
    const currentFollowers = fans.length;

    // Calculate growth rate from historical data
    const monthlyGrowthRate = 0.08; // 8% monthly growth

    return {
      currentFollowers,
      predicted30Days: Math.round(currentFollowers * (1 + monthlyGrowthRate)),
      predicted90Days: Math.round(currentFollowers * Math.pow(1 + monthlyGrowthRate, 3)),
      predicted1Year: Math.round(currentFollowers * Math.pow(1 + monthlyGrowthRate, 12)),
      growthDrivers: [
        'Strong playlist placements',
        'Growing social media presence',
        'Consistent release schedule',
        'High engagement rate',
      ],
      growthBlockers: [
        'Limited international reach',
        'Low discovery rate outside genre',
      ],
      recommendedActions: [
        'Collaborate with artists in adjacent genres',
        'Increase presence on TikTok',
        'Consider remix releases for international markets',
      ],
    };
  }

  // ============================================================================
  // Market Trends
  // ============================================================================

  async getMarketTrends(): Promise<MarketTrends> {
    if (this.marketTrendsCache && this.trendsCacheExpiry && new Date() < this.trendsCacheExpiry) {
      return this.marketTrendsCache;
    }

    const [genreMomentum, emergingArtists, soundTrends, platformTrends, seasonalPatterns] =
      await Promise.all([
        this.analyzeGenreMomentum(),
        this.findEmergingArtists(),
        this.analyzeSoundTrends(),
        this.analyzePlatformTrends(),
        this.analyzeSeasonalPatterns(),
      ]);

    this.marketTrendsCache = {
      genreMomentum,
      emergingArtists,
      soundTrends,
      platformTrends,
      seasonalPatterns,
    };
    this.trendsCacheExpiry = new Date(Date.now() + 3600000); // 1 hour cache

    return this.marketTrendsCache;
  }

  private async analyzeGenreMomentum(): Promise<GenreTrend[]> {
    // Would analyze platform-wide data
    return [
      {
        genre: 'Hyperpop',
        momentum: 0.92,
        growth7Day: 0.15,
        growth30Day: 0.45,
        growth90Day: 1.2,
        topArtists: [],
        subgenreBreakdown: [
          { name: 'Glitchcore', share: 0.35 },
          { name: 'Bubblegum Bass', share: 0.25 },
          { name: 'Deconstructed Club', share: 0.2 },
        ],
        predictedPeak: new Date(Date.now() + 180 * 86400000),
      },
      {
        genre: 'Amapiano',
        momentum: 0.88,
        growth7Day: 0.12,
        growth30Day: 0.38,
        growth90Day: 0.95,
        topArtists: [],
        subgenreBreakdown: [],
        predictedPeak: null,
      },
      {
        genre: 'Indie Folk',
        momentum: 0.65,
        growth7Day: 0.02,
        growth30Day: 0.05,
        growth90Day: 0.08,
        topArtists: [],
        subgenreBreakdown: [],
        predictedPeak: null,
      },
    ];
  }

  private async findEmergingArtists(): Promise<EmergingArtist[]> {
    // Would analyze rising artists
    return [];
  }

  private async analyzeSoundTrends(): Promise<SoundTrend[]> {
    return [
      {
        name: 'Pitched Vocals',
        description: 'Heavily pitch-shifted vocal processing',
        examples: [],
        popularity: 0.78,
        growthRate: 0.25,
        relatedGenres: ['Hyperpop', 'Cloud Rap', 'Experimental'],
        audioCharacteristics: {
          tempo: { min: 120, max: 160 },
          energy: 0.8,
          valence: 0.6,
          danceability: 0.7,
        },
      },
      {
        name: 'Lo-Fi Textures',
        description: 'Vinyl crackle, tape saturation, degraded audio',
        examples: [],
        popularity: 0.72,
        growthRate: 0.08,
        relatedGenres: ['Lo-Fi Hip Hop', 'Bedroom Pop', 'Chillwave'],
        audioCharacteristics: {
          tempo: { min: 70, max: 95 },
          energy: 0.3,
          valence: 0.5,
          danceability: 0.4,
        },
      },
    ];
  }

  private async analyzePlatformTrends(): Promise<PlatformTrend[]> {
    return [
      {
        platform: 'TikTok',
        trend: '15-second hooks driving discovery',
        impact: 'high',
        actionableInsight: 'Create shareable 15-second clips of your strongest hooks',
        relevantFor: ['Pop', 'Hip Hop', 'Electronic'],
      },
      {
        platform: 'Spotify',
        trend: 'Playlist pitching success increasing for independent artists',
        impact: 'medium',
        actionableInsight: 'Submit to editorial playlists 4 weeks before release',
        relevantFor: ['All genres'],
      },
    ];
  }

  private async analyzeSeasonalPatterns(): Promise<SeasonalPattern[]> {
    return [
      {
        period: 'Summer',
        genres: [
          { genre: 'Dance/Electronic', multiplier: 1.4 },
          { genre: 'Reggaeton', multiplier: 1.3 },
          { genre: 'Pop', multiplier: 1.2 },
        ],
        listeningBehavior: 'Upbeat, outdoor-friendly tracks preferred',
        recommendations: ['Release high-energy tracks', 'Focus on festival-ready mixes'],
      },
      {
        period: 'Winter',
        genres: [
          { genre: 'Indie', multiplier: 1.3 },
          { genre: 'R&B', multiplier: 1.2 },
          { genre: 'Classical', multiplier: 1.15 },
        ],
        listeningBehavior: 'Introspective, cozy listening sessions',
        recommendations: ['Release emotional ballads', 'Acoustic versions perform well'],
      },
    ];
  }

  // ============================================================================
  // A/B Testing
  // ============================================================================

  async createABTest(
    artistId: string,
    data: {
      name: string;
      type: ABTest['type'];
      variants: { name: string; content: any }[];
      targetAudience: string;
      sampleSize: number;
      durationDays: number;
    }
  ): Promise<ABTest> {
    const testId = `test_${Date.now()}`;

    const test: ABTest = {
      id: testId,
      name: data.name,
      type: data.type,
      status: 'draft',
      variants: data.variants.map((v, i) => ({
        id: `variant_${i}`,
        name: v.name,
        content: v.content,
        participants: 0,
        conversions: 0,
        conversionRate: 0,
      })),
      targetAudience: data.targetAudience,
      sampleSize: data.sampleSize,
      currentParticipants: 0,
      startDate: new Date(),
    };

    // Would store in database

    return test;
  }

  async startABTest(testId: string): Promise<void> {
    // Would update test status and begin variant assignment
  }

  async getABTestResults(testId: string): Promise<ABTestResults | null> {
    // Would calculate statistical significance
    return {
      winner: 'variant_0',
      confidence: 0.95,
      improvement: 0.23,
      statisticalSignificance: true,
      insights: [
        'Variant A showed 23% higher click-through rate',
        'Best performing in 18-24 age segment',
        'Mobile users responded best to Variant A',
      ],
    };
  }

  // ============================================================================
  // Helper Methods
  // ============================================================================

  private async getArtistFans(artistId: string): Promise<FanProfile[]> {
    return Array.from(this.fanProfiles.get(artistId)?.values() || []);
  }

  private async getArtistFunnelStats(artistId: string): Promise<{
    discovered: number;
    firstListen: number;
    repeatListen: number;
    saved: number;
    engaged: number;
    superFan: number;
  }> {
    // Would query analytics database
    return {
      discovered: 100000,
      firstListen: 75000,
      repeatListen: 30000,
      saved: 15000,
      engaged: 5000,
      superFan: 1000,
    };
  }

  private async getArtistRecentTracks(artistId: string): Promise<any[]> {
    // Would fetch from database
    return [];
  }

  private aggregateByField(
    fans: FanProfile[],
    getter: (fan: FanProfile) => string
  ): [string, number][] {
    const counts = new Map<string, number>();
    for (const fan of fans) {
      const value = getter(fan);
      counts.set(value, (counts.get(value) || 0) + 1);
    }
    return Array.from(counts.entries()).sort((a, b) => b[1] - a[1]);
  }

  private async findFastestGrowingSegment(artistId: string): Promise<string> {
    return '18-24 age group';
  }

  private findHighestValueSegment(fans: FanProfile[]): string {
    const segmentValues = new Map<string, number>();
    for (const fan of fans) {
      const segment = fan.demographicSegment.ageRange;
      segmentValues.set(
        segment,
        (segmentValues.get(segment) || 0) + fan.lifetimeValue
      );
    }
    let highest = '';
    let highestValue = 0;
    for (const [segment, value] of segmentValues) {
      if (value > highestValue) {
        highest = segment;
        highestValue = value;
      }
    }
    return highest;
  }

  private findMostEngagedSegment(fans: FanProfile[]): string {
    const segmentEngagement = new Map<string, { total: number; count: number }>();
    for (const fan of fans) {
      const segment = fan.demographicSegment.ageRange;
      const current = segmentEngagement.get(segment) || { total: 0, count: 0 };
      segmentEngagement.set(segment, {
        total: current.total + fan.engagementScore,
        count: current.count + 1,
      });
    }
    let highest = '';
    let highestAvg = 0;
    for (const [segment, data] of segmentEngagement) {
      const avg = data.total / data.count;
      if (avg > highestAvg) {
        highest = segment;
        highestAvg = avg;
      }
    }
    return highest;
  }

  private async calculateRegionGrowth(artistId: string, regionId: string): Promise<number> {
    return 0.12; // Would calculate from historical data
  }

  private async getRegionCoordinates(
    country: string,
    region: string
  ): Promise<{ lat: number; lng: number }> {
    return { lat: 0, lng: 0 }; // Would geocode
  }

  private getGrowthRecommendations(region: GeographicRegion): string[] {
    return [
      `Consider local marketing in ${region.name}`,
      'Collaborate with local artists',
      'Translate content for local audience',
    ];
  }

  private getMostCommonTimezone(fans: FanProfile[]): string {
    const tzCounts = new Map<string, number>();
    for (const fan of fans) {
      const tz = fan.demographicSegment.location.timezone;
      tzCounts.set(tz, (tzCounts.get(tz) || 0) + 1);
    }
    let mostCommon = 'UTC';
    let highestCount = 0;
    for (const [tz, count] of tzCounts) {
      if (count > highestCount) {
        mostCommon = tz;
        highestCount = count;
      }
    }
    return mostCommon;
  }

  private calculateViralScore(track: any): number {
    return 0.75; // Would analyze audio features and metadata
  }

  private getBenchmarkConversion(from: string, to: string): number {
    const benchmarks: Record<string, number> = {
      'Discovery-First Listen': 0.7,
      'First Listen-Repeat Listen': 0.4,
      'Repeat Listen-Save/Follow': 0.5,
      'Save/Follow-Engaged Fan': 0.35,
      'Engaged Fan-Super Fan': 0.2,
    };
    return benchmarks[`${from}-${to}`] || 0.5;
  }

  private getDropoffReasons(stage: string): string[] {
    const reasons: Record<string, string[]> = {
      Discovery: ['Low visibility in recommendations', 'Weak metadata'],
      'First Listen': ['Opening not engaging', 'Production quality'],
      'Repeat Listen': ['Song not memorable', 'Too similar to first impression'],
      'Save/Follow': ['Not enough content', 'Inconsistent style'],
      'Engaged Fan': ['Lack of interaction', 'Long gaps between releases'],
    };
    return reasons[stage] || [];
  }

  private getDropoffFixes(stage: string): string[] {
    const fixes: Record<string, string[]> = {
      Discovery: ['Improve SEO', 'Playlist pitching', 'Social promotion'],
      'First Listen': ['Optimize song structure', 'Stronger intro'],
      'Repeat Listen': ['Add memorable hooks', 'Improve production'],
      'Save/Follow': ['Regular content schedule', 'Artist storytelling'],
      'Engaged Fan': ['Fan engagement activities', 'Exclusive content'],
    };
    return fixes[stage] || [];
  }

  private generateFunnelRecommendations(
    stages: FunnelStage[],
    conversionRates: ConversionRate[],
    dropoffPoints: DropoffPoint[]
  ): FunnelRecommendation[] {
    const recommendations: FunnelRecommendation[] = [];

    for (const dropoff of dropoffPoints) {
      recommendations.push({
        priority: dropoff.dropoffRate > 0.5 ? 'high' : 'medium',
        stage: dropoff.stage,
        recommendation: dropoff.suggestedFixes[0] || 'Analyze user behavior at this stage',
        expectedImpact: dropoff.dropoffRate * 0.3,
        effort: 'medium',
      });
    }

    return recommendations.sort((a, b) => {
      const priorityOrder = { high: 0, medium: 1, low: 2 };
      return priorityOrder[a.priority] - priorityOrder[b.priority];
    });
  }
}

export const fanIntelligence = new FanIntelligenceService();
export default fanIntelligence;
