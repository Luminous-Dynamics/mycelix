// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Achievements & Gamification System
 *
 * Comprehensive gamification with achievements, XP, levels, streaks,
 * challenges, leaderboards, and seasonal events for Mycelix.
 */

import { EventEmitter } from 'events';

// ============================================================================
// Types
// ============================================================================

export interface Achievement {
  id: string;
  name: string;
  description: string;
  category: AchievementCategory;
  tier: AchievementTier;
  icon: string;
  xpReward: number;
  requirement: AchievementRequirement;
  isSecret: boolean;
  unlockedBy?: number; // percentage of users who have it
}

export interface AchievementRequirement {
  type: RequirementType;
  target: number;
  metadata?: Record<string, any>;
}

export type RequirementType =
  | 'tracks_played'
  | 'unique_artists'
  | 'hours_listened'
  | 'tracks_liked'
  | 'playlists_created'
  | 'circles_joined'
  | 'live_sessions_attended'
  | 'comments_posted'
  | 'followers_gained'
  | 'artists_followed'
  | 'genres_explored'
  | 'daily_streak'
  | 'samples_purchased'
  | 'tracks_uploaded'
  | 'collaborations_completed'
  | 'patron_months'
  | 'nft_collected'
  | 'challenges_completed'
  | 'perfect_week'
  | 'social_shares';

export type AchievementCategory =
  | 'listening'
  | 'discovery'
  | 'social'
  | 'creator'
  | 'collector'
  | 'community'
  | 'seasonal'
  | 'special';

export type AchievementTier = 'bronze' | 'silver' | 'gold' | 'platinum' | 'diamond';

export interface UserProgress {
  userId: string;
  xp: number;
  level: number;
  achievements: UnlockedAchievement[];
  streaks: StreakData;
  stats: UserStats;
  badges: Badge[];
  seasonalProgress: SeasonalProgress;
}

export interface UnlockedAchievement {
  achievementId: string;
  unlockedAt: Date;
  progress: number;
  isNew: boolean;
}

export interface StreakData {
  currentDaily: number;
  longestDaily: number;
  currentWeekly: number;
  longestWeekly: number;
  lastActiveDate: string;
  freezesRemaining: number;
  freezesUsed: number;
}

export interface UserStats {
  totalTracksPlayed: number;
  totalHoursListened: number;
  uniqueArtistsPlayed: number;
  uniqueGenresExplored: number;
  tracksLiked: number;
  playlistsCreated: number;
  circlesJoined: number;
  liveSessionsAttended: number;
  commentsPosted: number;
  followersGained: number;
  artistsFollowed: number;
  socialShares: number;
  tracksUploaded: number;
  collaborationsCompleted: number;
  challengesCompleted: number;
  samplesPurchased: number;
  nftsCollected: number;
  patronMonths: number;
}

export interface Badge {
  id: string;
  name: string;
  icon: string;
  earnedAt: Date;
  isDisplayed: boolean;
}

export interface SeasonalProgress {
  seasonId: string;
  seasonName: string;
  startDate: Date;
  endDate: Date;
  tier: number;
  xpEarned: number;
  rewards: SeasonalReward[];
  challengesCompleted: string[];
}

export interface SeasonalReward {
  tier: number;
  name: string;
  type: 'badge' | 'avatar_frame' | 'theme' | 'exclusive_content' | 'discount';
  claimed: boolean;
}

export interface Challenge {
  id: string;
  name: string;
  description: string;
  type: ChallengeType;
  requirement: AchievementRequirement;
  xpReward: number;
  startDate: Date;
  endDate: Date;
  participants: number;
  completedBy: number;
  rewards?: ChallengeReward[];
}

export type ChallengeType = 'daily' | 'weekly' | 'monthly' | 'seasonal' | 'community';

export interface ChallengeReward {
  type: 'badge' | 'xp_boost' | 'exclusive_track' | 'discount' | 'feature_unlock';
  value: any;
}

export interface LeaderboardEntry {
  rank: number;
  userId: string;
  username: string;
  avatar?: string;
  score: number;
  level: number;
  change: number; // rank change from last period
}

export interface Leaderboard {
  id: string;
  name: string;
  type: LeaderboardType;
  period: 'daily' | 'weekly' | 'monthly' | 'all_time';
  entries: LeaderboardEntry[];
  userRank?: LeaderboardEntry;
  lastUpdated: Date;
}

export type LeaderboardType =
  | 'xp'
  | 'listening_time'
  | 'discoveries'
  | 'social'
  | 'creator'
  | 'streak'
  | 'challenges';

// ============================================================================
// XP & Level System
// ============================================================================

const XP_ACTIONS = {
  track_played: 5,
  track_completed: 10,
  new_artist_discovered: 25,
  new_genre_explored: 50,
  track_liked: 5,
  playlist_created: 25,
  playlist_shared: 15,
  circle_joined: 50,
  live_session_attended: 75,
  comment_posted: 10,
  track_shared: 20,
  friend_invited: 100,
  daily_login: 25,
  streak_maintained: 15,
  challenge_completed: 100,
  achievement_unlocked: 50,
  profile_completed: 100,
  first_follow: 25,
  first_patron: 200,
};

function calculateLevelFromXP(xp: number): number {
  // Level formula: XP required = 100 * level^1.5
  let level = 1;
  let xpRequired = 0;

  while (xpRequired <= xp) {
    level++;
    xpRequired += Math.floor(100 * Math.pow(level, 1.5));
  }

  return level - 1;
}

function getXPForLevel(level: number): number {
  let total = 0;
  for (let l = 1; l <= level; l++) {
    total += Math.floor(100 * Math.pow(l, 1.5));
  }
  return total;
}

function getXPProgress(xp: number): { current: number; needed: number; percentage: number } {
  const level = calculateLevelFromXP(xp);
  const currentLevelXP = getXPForLevel(level);
  const nextLevelXP = getXPForLevel(level + 1);

  const current = xp - currentLevelXP;
  const needed = nextLevelXP - currentLevelXP;

  return {
    current,
    needed,
    percentage: Math.round((current / needed) * 100),
  };
}

// ============================================================================
// Achievement Definitions
// ============================================================================

const ACHIEVEMENTS: Achievement[] = [
  // Listening Achievements
  {
    id: 'first_track',
    name: 'First Notes',
    description: 'Play your first track on Mycelix',
    category: 'listening',
    tier: 'bronze',
    icon: 'music-note',
    xpReward: 50,
    requirement: { type: 'tracks_played', target: 1 },
    isSecret: false,
  },
  {
    id: 'track_enthusiast',
    name: 'Track Enthusiast',
    description: 'Play 100 tracks',
    category: 'listening',
    tier: 'bronze',
    icon: 'headphones',
    xpReward: 100,
    requirement: { type: 'tracks_played', target: 100 },
    isSecret: false,
  },
  {
    id: 'music_devotee',
    name: 'Music Devotee',
    description: 'Play 1,000 tracks',
    category: 'listening',
    tier: 'silver',
    icon: 'fire',
    xpReward: 250,
    requirement: { type: 'tracks_played', target: 1000 },
    isSecret: false,
  },
  {
    id: 'sonic_master',
    name: 'Sonic Master',
    description: 'Play 10,000 tracks',
    category: 'listening',
    tier: 'gold',
    icon: 'crown',
    xpReward: 1000,
    requirement: { type: 'tracks_played', target: 10000 },
    isSecret: false,
  },
  {
    id: 'hour_listener',
    name: 'Hour Power',
    description: 'Listen for 10 hours total',
    category: 'listening',
    tier: 'bronze',
    icon: 'clock',
    xpReward: 100,
    requirement: { type: 'hours_listened', target: 10 },
    isSecret: false,
  },
  {
    id: 'marathon_listener',
    name: 'Marathon Listener',
    description: 'Listen for 100 hours total',
    category: 'listening',
    tier: 'silver',
    icon: 'timer',
    xpReward: 500,
    requirement: { type: 'hours_listened', target: 100 },
    isSecret: false,
  },
  {
    id: 'audiophile',
    name: 'True Audiophile',
    description: 'Listen for 1,000 hours total',
    category: 'listening',
    tier: 'platinum',
    icon: 'diamond',
    xpReward: 2500,
    requirement: { type: 'hours_listened', target: 1000 },
    isSecret: false,
  },

  // Discovery Achievements
  {
    id: 'explorer',
    name: 'Explorer',
    description: 'Listen to 10 different artists',
    category: 'discovery',
    tier: 'bronze',
    icon: 'compass',
    xpReward: 75,
    requirement: { type: 'unique_artists', target: 10 },
    isSecret: false,
  },
  {
    id: 'globetrotter',
    name: 'Globetrotter',
    description: 'Listen to 100 different artists',
    category: 'discovery',
    tier: 'silver',
    icon: 'globe',
    xpReward: 300,
    requirement: { type: 'unique_artists', target: 100 },
    isSecret: false,
  },
  {
    id: 'genre_explorer',
    name: 'Genre Explorer',
    description: 'Explore 5 different genres',
    category: 'discovery',
    tier: 'bronze',
    icon: 'map',
    xpReward: 100,
    requirement: { type: 'genres_explored', target: 5 },
    isSecret: false,
  },
  {
    id: 'genre_master',
    name: 'Genre Master',
    description: 'Explore 20 different genres',
    category: 'discovery',
    tier: 'gold',
    icon: 'stars',
    xpReward: 500,
    requirement: { type: 'genres_explored', target: 20 },
    isSecret: false,
  },

  // Social Achievements
  {
    id: 'social_butterfly',
    name: 'Social Butterfly',
    description: 'Join your first listening circle',
    category: 'social',
    tier: 'bronze',
    icon: 'users',
    xpReward: 75,
    requirement: { type: 'circles_joined', target: 1 },
    isSecret: false,
  },
  {
    id: 'circle_regular',
    name: 'Circle Regular',
    description: 'Join 10 listening circles',
    category: 'social',
    tier: 'silver',
    icon: 'circles',
    xpReward: 250,
    requirement: { type: 'circles_joined', target: 10 },
    isSecret: false,
  },
  {
    id: 'live_fan',
    name: 'Live Fan',
    description: 'Attend 5 live sessions',
    category: 'social',
    tier: 'bronze',
    icon: 'broadcast',
    xpReward: 150,
    requirement: { type: 'live_sessions_attended', target: 5 },
    isSecret: false,
  },
  {
    id: 'live_devotee',
    name: 'Live Devotee',
    description: 'Attend 50 live sessions',
    category: 'social',
    tier: 'gold',
    icon: 'star-live',
    xpReward: 750,
    requirement: { type: 'live_sessions_attended', target: 50 },
    isSecret: false,
  },
  {
    id: 'rising_star',
    name: 'Rising Star',
    description: 'Gain 100 followers',
    category: 'social',
    tier: 'silver',
    icon: 'trending-up',
    xpReward: 300,
    requirement: { type: 'followers_gained', target: 100 },
    isSecret: false,
  },
  {
    id: 'influencer',
    name: 'Influencer',
    description: 'Gain 1,000 followers',
    category: 'social',
    tier: 'gold',
    icon: 'verified',
    xpReward: 1000,
    requirement: { type: 'followers_gained', target: 1000 },
    isSecret: false,
  },

  // Creator Achievements
  {
    id: 'first_upload',
    name: 'First Upload',
    description: 'Upload your first track',
    category: 'creator',
    tier: 'bronze',
    icon: 'upload',
    xpReward: 100,
    requirement: { type: 'tracks_uploaded', target: 1 },
    isSecret: false,
  },
  {
    id: 'prolific_creator',
    name: 'Prolific Creator',
    description: 'Upload 50 tracks',
    category: 'creator',
    tier: 'gold',
    icon: 'library-music',
    xpReward: 750,
    requirement: { type: 'tracks_uploaded', target: 50 },
    isSecret: false,
  },
  {
    id: 'collaborator',
    name: 'Collaborator',
    description: 'Complete your first collaboration',
    category: 'creator',
    tier: 'silver',
    icon: 'handshake',
    xpReward: 200,
    requirement: { type: 'collaborations_completed', target: 1 },
    isSecret: false,
  },
  {
    id: 'collab_master',
    name: 'Collab Master',
    description: 'Complete 25 collaborations',
    category: 'creator',
    tier: 'platinum',
    icon: 'network',
    xpReward: 1500,
    requirement: { type: 'collaborations_completed', target: 25 },
    isSecret: false,
  },

  // Streak Achievements
  {
    id: 'week_streak',
    name: 'Week Warrior',
    description: 'Maintain a 7-day listening streak',
    category: 'listening',
    tier: 'bronze',
    icon: 'flame',
    xpReward: 150,
    requirement: { type: 'daily_streak', target: 7 },
    isSecret: false,
  },
  {
    id: 'month_streak',
    name: 'Monthly Maestro',
    description: 'Maintain a 30-day listening streak',
    category: 'listening',
    tier: 'silver',
    icon: 'fire-flame',
    xpReward: 500,
    requirement: { type: 'daily_streak', target: 30 },
    isSecret: false,
  },
  {
    id: 'century_streak',
    name: 'Century Streak',
    description: 'Maintain a 100-day listening streak',
    category: 'listening',
    tier: 'gold',
    icon: 'meteor',
    xpReward: 1500,
    requirement: { type: 'daily_streak', target: 100 },
    isSecret: false,
  },
  {
    id: 'year_streak',
    name: 'Year of Music',
    description: 'Maintain a 365-day listening streak',
    category: 'listening',
    tier: 'diamond',
    icon: 'infinity',
    xpReward: 5000,
    requirement: { type: 'daily_streak', target: 365 },
    isSecret: false,
  },

  // Collector Achievements
  {
    id: 'first_nft',
    name: 'Digital Collector',
    description: 'Collect your first music NFT',
    category: 'collector',
    tier: 'bronze',
    icon: 'gem',
    xpReward: 100,
    requirement: { type: 'nft_collected', target: 1 },
    isSecret: false,
  },
  {
    id: 'nft_enthusiast',
    name: 'NFT Enthusiast',
    description: 'Collect 10 music NFTs',
    category: 'collector',
    tier: 'silver',
    icon: 'treasure-chest',
    xpReward: 400,
    requirement: { type: 'nft_collected', target: 10 },
    isSecret: false,
  },
  {
    id: 'sample_hunter',
    name: 'Sample Hunter',
    description: 'Purchase 5 sample packs',
    category: 'collector',
    tier: 'bronze',
    icon: 'shopping-cart',
    xpReward: 150,
    requirement: { type: 'samples_purchased', target: 5 },
    isSecret: false,
  },

  // Secret Achievements
  {
    id: 'night_owl',
    name: 'Night Owl',
    description: 'Listen for 2 hours between midnight and 5 AM',
    category: 'special',
    tier: 'silver',
    icon: 'moon',
    xpReward: 200,
    requirement: { type: 'hours_listened', target: 2, metadata: { timeRange: '00:00-05:00' } },
    isSecret: true,
  },
  {
    id: 'early_bird',
    name: 'Early Bird',
    description: 'Listen at 5 AM for 7 days straight',
    category: 'special',
    tier: 'gold',
    icon: 'sunrise',
    xpReward: 350,
    requirement: { type: 'daily_streak', target: 7, metadata: { mustBeAt: '05:00' } },
    isSecret: true,
  },
  {
    id: 'perfect_taste',
    name: 'Perfect Taste',
    description: 'Get 100 likes on your playlists',
    category: 'special',
    tier: 'gold',
    icon: 'award',
    xpReward: 500,
    requirement: { type: 'tracks_liked', target: 100, metadata: { on: 'playlists' } },
    isSecret: true,
  },
];

// ============================================================================
// Gamification Service
// ============================================================================

class GamificationService extends EventEmitter {
  private userProgressCache: Map<string, UserProgress> = new Map();
  private activeChallenges: Challenge[] = [];
  private leaderboardCache: Map<string, Leaderboard> = new Map();

  // ============================================================================
  // XP & Levels
  // ============================================================================

  async awardXP(
    userId: string,
    action: keyof typeof XP_ACTIONS,
    multiplier: number = 1,
    metadata?: Record<string, any>
  ): Promise<{ xpAwarded: number; newTotal: number; levelUp: boolean; newLevel?: number }> {
    const baseXP = XP_ACTIONS[action] || 0;
    const xpAwarded = Math.floor(baseXP * multiplier);

    const progress = await this.getUserProgress(userId);
    const oldLevel = progress.level;

    progress.xp += xpAwarded;
    progress.level = calculateLevelFromXP(progress.xp);

    const levelUp = progress.level > oldLevel;

    // Save progress
    await this.saveUserProgress(userId, progress);

    // Check for achievements
    await this.checkAchievements(userId, progress);

    // Emit events
    this.emit('xp_awarded', { userId, xpAwarded, action, metadata });

    if (levelUp) {
      this.emit('level_up', { userId, newLevel: progress.level, oldLevel });
    }

    return {
      xpAwarded,
      newTotal: progress.xp,
      levelUp,
      newLevel: levelUp ? progress.level : undefined,
    };
  }

  async getUserLevel(userId: string): Promise<{
    level: number;
    xp: number;
    progress: { current: number; needed: number; percentage: number };
    title: string;
  }> {
    const progress = await this.getUserProgress(userId);

    return {
      level: progress.level,
      xp: progress.xp,
      progress: getXPProgress(progress.xp),
      title: this.getLevelTitle(progress.level),
    };
  }

  private getLevelTitle(level: number): string {
    if (level < 5) return 'Listener';
    if (level < 10) return 'Music Fan';
    if (level < 20) return 'Audio Explorer';
    if (level < 35) return 'Sound Seeker';
    if (level < 50) return 'Beat Master';
    if (level < 75) return 'Rhythm Sage';
    if (level < 100) return 'Audio Legend';
    return 'Sound Deity';
  }

  // ============================================================================
  // Achievements
  // ============================================================================

  async getAchievements(userId: string): Promise<{
    unlocked: (Achievement & { unlockedAt: Date })[];
    inProgress: (Achievement & { progress: number; target: number })[];
    locked: Achievement[];
  }> {
    const progress = await this.getUserProgress(userId);
    const unlockedIds = new Set(progress.achievements.map(a => a.achievementId));

    const unlocked: (Achievement & { unlockedAt: Date })[] = [];
    const inProgress: (Achievement & { progress: number; target: number })[] = [];
    const locked: Achievement[] = [];

    for (const achievement of ACHIEVEMENTS) {
      if (unlockedIds.has(achievement.id)) {
        const unlockedAchievement = progress.achievements.find(
          a => a.achievementId === achievement.id
        );
        unlocked.push({
          ...achievement,
          unlockedAt: unlockedAchievement!.unlockedAt,
        });
      } else {
        const currentProgress = this.getAchievementProgress(achievement, progress.stats);

        if (currentProgress > 0 && !achievement.isSecret) {
          inProgress.push({
            ...achievement,
            progress: currentProgress,
            target: achievement.requirement.target,
          });
        } else if (!achievement.isSecret) {
          locked.push(achievement);
        }
      }
    }

    return { unlocked, inProgress, locked };
  }

  private async checkAchievements(userId: string, progress: UserProgress): Promise<void> {
    const unlockedIds = new Set(progress.achievements.map(a => a.achievementId));

    for (const achievement of ACHIEVEMENTS) {
      if (unlockedIds.has(achievement.id)) continue;

      const currentProgress = this.getAchievementProgress(achievement, progress.stats);

      if (currentProgress >= achievement.requirement.target) {
        // Unlock achievement
        progress.achievements.push({
          achievementId: achievement.id,
          unlockedAt: new Date(),
          progress: currentProgress,
          isNew: true,
        });

        // Award XP
        progress.xp += achievement.xpReward;

        this.emit('achievement_unlocked', {
          userId,
          achievement,
          xpAwarded: achievement.xpReward,
        });
      }
    }
  }

  private getAchievementProgress(achievement: Achievement, stats: UserStats): number {
    const { type, target } = achievement.requirement;

    switch (type) {
      case 'tracks_played':
        return stats.totalTracksPlayed;
      case 'hours_listened':
        return stats.totalHoursListened;
      case 'unique_artists':
        return stats.uniqueArtistsPlayed;
      case 'genres_explored':
        return stats.uniqueGenresExplored;
      case 'tracks_liked':
        return stats.tracksLiked;
      case 'playlists_created':
        return stats.playlistsCreated;
      case 'circles_joined':
        return stats.circlesJoined;
      case 'live_sessions_attended':
        return stats.liveSessionsAttended;
      case 'followers_gained':
        return stats.followersGained;
      case 'tracks_uploaded':
        return stats.tracksUploaded;
      case 'collaborations_completed':
        return stats.collaborationsCompleted;
      case 'nft_collected':
        return stats.nftsCollected;
      case 'samples_purchased':
        return stats.samplesPurchased;
      case 'challenges_completed':
        return stats.challengesCompleted;
      default:
        return 0;
    }
  }

  // ============================================================================
  // Streaks
  // ============================================================================

  async recordActivity(userId: string): Promise<StreakData> {
    const progress = await this.getUserProgress(userId);
    const today = new Date().toISOString().split('T')[0];

    if (progress.streaks.lastActiveDate === today) {
      // Already active today
      return progress.streaks;
    }

    const yesterday = new Date(Date.now() - 86400000).toISOString().split('T')[0];

    if (progress.streaks.lastActiveDate === yesterday) {
      // Continue streak
      progress.streaks.currentDaily++;
      progress.streaks.longestDaily = Math.max(
        progress.streaks.longestDaily,
        progress.streaks.currentDaily
      );
    } else if (progress.streaks.lastActiveDate) {
      // Check for streak freeze
      const daysMissed = Math.floor(
        (Date.now() - new Date(progress.streaks.lastActiveDate).getTime()) / 86400000
      );

      if (daysMissed <= progress.streaks.freezesRemaining) {
        progress.streaks.freezesRemaining -= daysMissed;
        progress.streaks.freezesUsed += daysMissed;
        progress.streaks.currentDaily++;
      } else {
        // Streak broken
        progress.streaks.currentDaily = 1;
      }
    } else {
      // First activity
      progress.streaks.currentDaily = 1;
    }

    progress.streaks.lastActiveDate = today;

    // Update weekly streak
    const weekNumber = this.getWeekNumber(new Date());
    const lastWeek = progress.streaks.lastActiveDate
      ? this.getWeekNumber(new Date(progress.streaks.lastActiveDate))
      : 0;

    if (weekNumber === lastWeek) {
      // Same week, continue
    } else if (weekNumber === lastWeek + 1) {
      progress.streaks.currentWeekly++;
      progress.streaks.longestWeekly = Math.max(
        progress.streaks.longestWeekly,
        progress.streaks.currentWeekly
      );
    } else {
      progress.streaks.currentWeekly = 1;
    }

    await this.saveUserProgress(userId, progress);

    // Check streak achievements
    await this.checkStreakAchievements(userId, progress);

    return progress.streaks;
  }

  async useStreakFreeze(userId: string): Promise<boolean> {
    const progress = await this.getUserProgress(userId);

    if (progress.streaks.freezesRemaining <= 0) {
      return false;
    }

    progress.streaks.freezesRemaining--;
    progress.streaks.freezesUsed++;

    await this.saveUserProgress(userId, progress);

    return true;
  }

  private async checkStreakAchievements(userId: string, progress: UserProgress): Promise<void> {
    const streakAchievements = ACHIEVEMENTS.filter(
      a => a.requirement.type === 'daily_streak'
    );

    for (const achievement of streakAchievements) {
      if (progress.streaks.currentDaily >= achievement.requirement.target) {
        const alreadyUnlocked = progress.achievements.some(
          a => a.achievementId === achievement.id
        );

        if (!alreadyUnlocked) {
          progress.achievements.push({
            achievementId: achievement.id,
            unlockedAt: new Date(),
            progress: progress.streaks.currentDaily,
            isNew: true,
          });

          progress.xp += achievement.xpReward;

          this.emit('achievement_unlocked', {
            userId,
            achievement,
            xpAwarded: achievement.xpReward,
          });
        }
      }
    }
  }

  private getWeekNumber(date: Date): number {
    const startOfYear = new Date(date.getFullYear(), 0, 1);
    const days = Math.floor((date.getTime() - startOfYear.getTime()) / 86400000);
    return Math.ceil((days + startOfYear.getDay() + 1) / 7);
  }

  // ============================================================================
  // Challenges
  // ============================================================================

  async getActiveChallenges(): Promise<Challenge[]> {
    return this.activeChallenges.filter(c => new Date() < c.endDate);
  }

  async joinChallenge(userId: string, challengeId: string): Promise<boolean> {
    const challenge = this.activeChallenges.find(c => c.id === challengeId);
    if (!challenge) return false;

    // Would store in database
    challenge.participants++;

    this.emit('challenge_joined', { userId, challengeId });

    return true;
  }

  async getChallengeProgress(
    userId: string,
    challengeId: string
  ): Promise<{ progress: number; target: number; completed: boolean }> {
    const challenge = this.activeChallenges.find(c => c.id === challengeId);
    if (!challenge) {
      throw new Error('Challenge not found');
    }

    const progress = await this.getUserProgress(userId);
    const currentProgress = this.getAchievementProgress(
      { requirement: challenge.requirement } as Achievement,
      progress.stats
    );

    return {
      progress: currentProgress,
      target: challenge.requirement.target,
      completed: currentProgress >= challenge.requirement.target,
    };
  }

  async createChallenge(data: Omit<Challenge, 'id' | 'participants' | 'completedBy'>): Promise<string> {
    const challengeId = `challenge_${Date.now()}`;

    const challenge: Challenge = {
      ...data,
      id: challengeId,
      participants: 0,
      completedBy: 0,
    };

    this.activeChallenges.push(challenge);

    return challengeId;
  }

  // ============================================================================
  // Leaderboards
  // ============================================================================

  async getLeaderboard(
    type: LeaderboardType,
    period: 'daily' | 'weekly' | 'monthly' | 'all_time',
    limit: number = 100
  ): Promise<Leaderboard> {
    const cacheKey = `${type}_${period}`;
    const cached = this.leaderboardCache.get(cacheKey);

    if (cached && Date.now() - cached.lastUpdated.getTime() < 300000) {
      return cached;
    }

    // Would fetch from database
    // For now, return mock data
    const leaderboard: Leaderboard = {
      id: cacheKey,
      name: this.getLeaderboardName(type),
      type,
      period,
      entries: [],
      lastUpdated: new Date(),
    };

    this.leaderboardCache.set(cacheKey, leaderboard);

    return leaderboard;
  }

  async getUserRank(
    userId: string,
    type: LeaderboardType,
    period: 'daily' | 'weekly' | 'monthly' | 'all_time'
  ): Promise<LeaderboardEntry | null> {
    // Would query database for user's rank
    return null;
  }

  private getLeaderboardName(type: LeaderboardType): string {
    const names: Record<LeaderboardType, string> = {
      xp: 'Top XP Earners',
      listening_time: 'Most Hours Listened',
      discoveries: 'Top Explorers',
      social: 'Most Social',
      creator: 'Top Creators',
      streak: 'Longest Streaks',
      challenges: 'Challenge Champions',
    };

    return names[type];
  }

  // ============================================================================
  // Seasonal Events
  // ============================================================================

  async getSeasonalProgress(userId: string): Promise<SeasonalProgress | null> {
    const progress = await this.getUserProgress(userId);
    return progress.seasonalProgress;
  }

  async claimSeasonalReward(userId: string, tier: number): Promise<boolean> {
    const progress = await this.getUserProgress(userId);

    if (!progress.seasonalProgress) return false;
    if (progress.seasonalProgress.tier < tier) return false;

    const reward = progress.seasonalProgress.rewards.find(
      r => r.tier === tier && !r.claimed
    );

    if (!reward) return false;

    reward.claimed = true;

    // Apply reward based on type
    switch (reward.type) {
      case 'badge':
        progress.badges.push({
          id: `seasonal_${progress.seasonalProgress.seasonId}_${tier}`,
          name: reward.name,
          icon: 'seasonal-badge',
          earnedAt: new Date(),
          isDisplayed: false,
        });
        break;
      // Handle other reward types
    }

    await this.saveUserProgress(userId, progress);

    return true;
  }

  // ============================================================================
  // Stats Tracking
  // ============================================================================

  async incrementStat(
    userId: string,
    stat: keyof UserStats,
    amount: number = 1
  ): Promise<void> {
    const progress = await this.getUserProgress(userId);
    progress.stats[stat] = (progress.stats[stat] || 0) + amount;

    await this.saveUserProgress(userId, progress);
    await this.checkAchievements(userId, progress);
  }

  // ============================================================================
  // Progress Management
  // ============================================================================

  async getUserProgress(userId: string): Promise<UserProgress> {
    const cached = this.userProgressCache.get(userId);
    if (cached) return cached;

    // Would load from database
    const progress: UserProgress = {
      userId,
      xp: 0,
      level: 1,
      achievements: [],
      streaks: {
        currentDaily: 0,
        longestDaily: 0,
        currentWeekly: 0,
        longestWeekly: 0,
        lastActiveDate: '',
        freezesRemaining: 2,
        freezesUsed: 0,
      },
      stats: {
        totalTracksPlayed: 0,
        totalHoursListened: 0,
        uniqueArtistsPlayed: 0,
        uniqueGenresExplored: 0,
        tracksLiked: 0,
        playlistsCreated: 0,
        circlesJoined: 0,
        liveSessionsAttended: 0,
        commentsPosted: 0,
        followersGained: 0,
        artistsFollowed: 0,
        socialShares: 0,
        tracksUploaded: 0,
        collaborationsCompleted: 0,
        challengesCompleted: 0,
        samplesPurchased: 0,
        nftsCollected: 0,
        patronMonths: 0,
      },
      badges: [],
      seasonalProgress: {
        seasonId: 'winter_2026',
        seasonName: 'Winter Beats',
        startDate: new Date('2026-01-01'),
        endDate: new Date('2026-03-31'),
        tier: 0,
        xpEarned: 0,
        rewards: [
          { tier: 1, name: 'Snowflake Badge', type: 'badge', claimed: false },
          { tier: 5, name: 'Winter Frame', type: 'avatar_frame', claimed: false },
          { tier: 10, name: 'Frost Theme', type: 'theme', claimed: false },
        ],
        challengesCompleted: [],
      },
    };

    this.userProgressCache.set(userId, progress);
    return progress;
  }

  private async saveUserProgress(userId: string, progress: UserProgress): Promise<void> {
    this.userProgressCache.set(userId, progress);
    // Would persist to database
  }
}

export const gamification = new GamificationService();
export default gamification;
