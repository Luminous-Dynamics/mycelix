// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Smart Spam Filter Service for Mycelix Mail
 *
 * Multi-layered spam detection:
 * - Trust-based filtering (MATL scores)
 * - Content analysis (patterns, keywords)
 * - Behavioral signals (sending patterns)
 * - ML-ready feature extraction
 * - Adaptive learning from user actions
 */

import type { AgentPubKey, ActionHash } from '@holochain/client';
import type { MycelixMailClient } from '../index';
import type { DecryptedEmail, TrustScore } from '../types';

// ==================== TYPES ====================

export interface SpamConfig {
  /** Trust threshold below which emails are suspicious */
  trustThreshold: number;
  /** Trust threshold below which emails are likely spam */
  spamTrustThreshold: number;
  /** Enable content analysis */
  enableContentAnalysis: boolean;
  /** Enable behavioral analysis */
  enableBehavioralAnalysis: boolean;
  /** Custom spam keywords */
  customSpamKeywords: string[];
  /** Whitelisted senders (always allow) */
  whitelist: AgentPubKey[];
  /** Blacklisted senders (always block) */
  blacklist: AgentPubKey[];
  /** Auto-learn from user actions */
  enableAutoLearn: boolean;
}

export interface SpamVerdict {
  /** Is this email spam? */
  isSpam: boolean;
  /** Spam probability (0-1) */
  spamProbability: number;
  /** Confidence in the verdict (0-1) */
  confidence: number;
  /** Category of spam/ham */
  category: SpamCategory;
  /** Reasons for the verdict */
  reasons: SpamReason[];
  /** Recommended action */
  action: SpamAction;
  /** Trust score of sender */
  senderTrustScore: number | null;
  /** Feature vector (for ML) */
  features: SpamFeatures;
}

export type SpamCategory =
  | 'clean'
  | 'suspicious'
  | 'likely_spam'
  | 'definite_spam'
  | 'phishing'
  | 'scam'
  | 'malware_link'
  | 'unknown_sender';

export type SpamAction =
  | 'deliver'
  | 'deliver_with_warning'
  | 'quarantine'
  | 'reject'
  | 'require_verification';

export interface SpamReason {
  type: string;
  description: string;
  weight: number;
  details?: unknown;
}

export interface SpamFeatures {
  // Trust features
  senderTrustScore: number;
  senderAttestationCount: number;
  senderByzantineFlags: number;
  hasMutualContacts: boolean;

  // Content features
  subjectLength: number;
  bodyLength: number;
  linkCount: number;
  imageCount: number;
  attachmentCount: number;
  hasHtmlContent: boolean;
  capsRatio: number;
  exclamationCount: number;
  questionCount: number;
  dollarSignCount: number;
  spamKeywordCount: number;
  urgencyWordCount: number;
  suspiciousPatternCount: number;

  // Behavioral features
  isFirstContact: boolean;
  senderEmailCount: number;
  senderRecentActivity: number;
  timeSinceLastEmail: number;
  isReplyToOurEmail: boolean;

  // Network features
  senderNetworkSize: number;
  sharedConnections: number;
}

export interface SpamStats {
  totalAnalyzed: number;
  spamDetected: number;
  falsePositives: number;
  falseNegatives: number;
  accuracy: number;
  lastUpdated: Date;
}

// ==================== SPAM FILTER SERVICE ====================

export class SpamFilterService {
  private config: SpamConfig;
  private spamKeywords: Set<string>;
  private urgencyWords: Set<string>;
  private suspiciousPatterns: RegExp[];
  private learnedSpamPatterns: Map<string, number> = new Map();
  private learnedHamPatterns: Map<string, number> = new Map();
  private senderHistory: Map<string, { count: number; lastSeen: number; marked: 'spam' | 'ham' | null }> = new Map();
  private stats: SpamStats;

  constructor(
    private client: MycelixMailClient,
    config?: Partial<SpamConfig>
  ) {
    this.config = {
      trustThreshold: 0.3,
      spamTrustThreshold: 0.1,
      enableContentAnalysis: true,
      enableBehavioralAnalysis: true,
      customSpamKeywords: [],
      whitelist: [],
      blacklist: [],
      enableAutoLearn: true,
      ...config,
    };

    this.spamKeywords = new Set([
      'viagra', 'cialis', 'lottery', 'winner', 'prize', 'casino',
      'bitcoin', 'crypto', 'investment', 'guaranteed', 'free money',
      'nigerian', 'prince', 'inheritance', 'million dollars',
      'click here', 'act now', 'limited time', 'expires',
      'unsubscribe', 'opt-out', 'remove me',
      ...this.config.customSpamKeywords,
    ]);

    this.urgencyWords = new Set([
      'urgent', 'immediately', 'asap', 'hurry', 'quick', 'fast',
      'now', 'today', 'deadline', 'expires', 'limited',
      'last chance', 'final notice', 'action required',
    ]);

    this.suspiciousPatterns = [
      /\b[A-Z]{5,}\b/g, // All caps words
      /[$€£]\d+[,.]?\d*/g, // Money amounts
      /\d{1,2}[%]/g, // Percentages
      /https?:\/\/[^\s]+\.(tk|ml|ga|cf|gq)/gi, // Suspicious TLDs
      /bit\.ly|tinyurl|goo\.gl/gi, // URL shorteners
      /(password|account|verify|confirm|update|secure)/gi, // Phishing keywords
    ];

    this.stats = {
      totalAnalyzed: 0,
      spamDetected: 0,
      falsePositives: 0,
      falseNegatives: 0,
      accuracy: 1.0,
      lastUpdated: new Date(),
    };
  }

  // ==================== MAIN ANALYSIS ====================

  /**
   * Analyze an email for spam
   */
  async analyzeEmail(email: DecryptedEmail): Promise<SpamVerdict> {
    this.stats.totalAnalyzed++;

    // Check whitelist/blacklist first
    if (this.isWhitelisted(email.sender)) {
      return this.createVerdict('clean', 0, 1.0, 'deliver', [
        { type: 'whitelist', description: 'Sender is whitelisted', weight: -1 },
      ], null, this.extractFeatures(email, null));
    }

    if (this.isBlacklisted(email.sender)) {
      this.stats.spamDetected++;
      return this.createVerdict('definite_spam', 1.0, 1.0, 'reject', [
        { type: 'blacklist', description: 'Sender is blacklisted', weight: 1 },
      ], null, this.extractFeatures(email, null));
    }

    // Get trust score
    let trustScore: TrustScore | null = null;
    try {
      trustScore = await this.client.trust.getTrustScore({
        subject: email.sender,
        category: 'Communication',
        include_transitive: true,
      });
    } catch {
      // Trust service unavailable
    }

    // Extract features
    const features = this.extractFeatures(email, trustScore);

    // Calculate spam probability from multiple signals
    const reasons: SpamReason[] = [];
    let spamScore = 0;
    let totalWeight = 0;

    // Trust-based scoring
    const trustResult = this.analyzeTrust(features, trustScore);
    reasons.push(...trustResult.reasons);
    spamScore += trustResult.score * trustResult.weight;
    totalWeight += trustResult.weight;

    // Content analysis
    if (this.config.enableContentAnalysis) {
      const contentResult = this.analyzeContent(email, features);
      reasons.push(...contentResult.reasons);
      spamScore += contentResult.score * contentResult.weight;
      totalWeight += contentResult.weight;
    }

    // Behavioral analysis
    if (this.config.enableBehavioralAnalysis) {
      const behaviorResult = this.analyzeBehavior(email, features);
      reasons.push(...behaviorResult.reasons);
      spamScore += behaviorResult.score * behaviorResult.weight;
      totalWeight += behaviorResult.weight;
    }

    // Normalize spam probability
    const spamProbability = Math.min(1, Math.max(0, spamScore / totalWeight));

    // Determine category and action
    const { category, action, confidence } = this.categorize(spamProbability, reasons);

    if (category !== 'clean') {
      this.stats.spamDetected++;
    }

    return this.createVerdict(
      category,
      spamProbability,
      confidence,
      action,
      reasons,
      trustScore?.combined_score ?? null,
      features
    );
  }

  /**
   * Batch analyze multiple emails
   */
  async analyzeEmails(emails: DecryptedEmail[]): Promise<Map<ActionHash, SpamVerdict>> {
    const results = new Map<ActionHash, SpamVerdict>();

    for (const email of emails) {
      const verdict = await this.analyzeEmail(email);
      results.set(email.hash, verdict);
    }

    return results;
  }

  // ==================== TRUST ANALYSIS ====================

  private analyzeTrust(
    features: SpamFeatures,
    trustScore: TrustScore | null
  ): { score: number; weight: number; reasons: SpamReason[] } {
    const reasons: SpamReason[] = [];
    let score = 0.5; // Neutral baseline

    if (!trustScore) {
      score = 0.7; // Unknown sender is suspicious
      reasons.push({
        type: 'no_trust_data',
        description: 'No trust information available for sender',
        weight: 0.3,
      });
    } else {
      // Low trust score
      if (trustScore.combined_score < this.config.spamTrustThreshold) {
        score = 0.9;
        reasons.push({
          type: 'very_low_trust',
          description: `Sender trust score very low: ${trustScore.combined_score.toFixed(2)}`,
          weight: 0.5,
        });
      } else if (trustScore.combined_score < this.config.trustThreshold) {
        score = 0.6;
        reasons.push({
          type: 'low_trust',
          description: `Sender trust score below threshold: ${trustScore.combined_score.toFixed(2)}`,
          weight: 0.3,
        });
      } else {
        score = 0.2;
        reasons.push({
          type: 'trusted_sender',
          description: `Sender has good trust score: ${trustScore.combined_score.toFixed(2)}`,
          weight: -0.3,
        });
      }

      // Byzantine flags
      if (features.senderByzantineFlags > 0) {
        score += 0.2;
        reasons.push({
          type: 'byzantine_flags',
          description: `Sender has ${features.senderByzantineFlags} Byzantine flags`,
          weight: 0.3,
        });
      }
    }

    // First contact
    if (features.isFirstContact) {
      score += 0.1;
      reasons.push({
        type: 'first_contact',
        description: 'First message from this sender',
        weight: 0.1,
      });
    }

    // Shared connections reduce suspicion
    if (features.sharedConnections > 0) {
      score -= 0.1;
      reasons.push({
        type: 'shared_connections',
        description: `You share ${features.sharedConnections} connections`,
        weight: -0.1,
      });
    }

    return { score: Math.min(1, Math.max(0, score)), weight: 0.4, reasons };
  }

  // ==================== CONTENT ANALYSIS ====================

  private analyzeContent(
    email: DecryptedEmail,
    features: SpamFeatures
  ): { score: number; weight: number; reasons: SpamReason[] } {
    const reasons: SpamReason[] = [];
    let score = 0;

    const subject = email.subject.toLowerCase();
    const body = email.body.toLowerCase();
    const text = `${subject} ${body}`;

    // Spam keywords
    let keywordCount = 0;
    for (const keyword of this.spamKeywords) {
      if (text.includes(keyword)) {
        keywordCount++;
      }
    }
    if (keywordCount > 0) {
      score += Math.min(0.5, keywordCount * 0.1);
      reasons.push({
        type: 'spam_keywords',
        description: `Contains ${keywordCount} spam keywords`,
        weight: Math.min(0.5, keywordCount * 0.1),
      });
    }

    // Urgency words
    let urgencyCount = 0;
    for (const word of this.urgencyWords) {
      if (text.includes(word)) {
        urgencyCount++;
      }
    }
    if (urgencyCount > 2) {
      score += 0.2;
      reasons.push({
        type: 'urgency_language',
        description: `Excessive urgency words: ${urgencyCount}`,
        weight: 0.2,
      });
    }

    // Suspicious patterns
    let patternMatches = 0;
    for (const pattern of this.suspiciousPatterns) {
      const matches = text.match(pattern);
      if (matches) {
        patternMatches += matches.length;
      }
    }
    if (patternMatches > 0) {
      score += Math.min(0.4, patternMatches * 0.05);
      reasons.push({
        type: 'suspicious_patterns',
        description: `Found ${patternMatches} suspicious patterns`,
        weight: Math.min(0.4, patternMatches * 0.05),
      });
    }

    // All caps ratio
    if (features.capsRatio > 0.3) {
      score += 0.15;
      reasons.push({
        type: 'excessive_caps',
        description: `High caps ratio: ${(features.capsRatio * 100).toFixed(0)}%`,
        weight: 0.15,
      });
    }

    // Many exclamation marks
    if (features.exclamationCount > 5) {
      score += 0.1;
      reasons.push({
        type: 'excessive_punctuation',
        description: `Many exclamation marks: ${features.exclamationCount}`,
        weight: 0.1,
      });
    }

    // Money mentions
    if (features.dollarSignCount > 2) {
      score += 0.15;
      reasons.push({
        type: 'money_mentions',
        description: `Multiple money references: ${features.dollarSignCount}`,
        weight: 0.15,
      });
    }

    // Too many links
    if (features.linkCount > 5) {
      score += 0.2;
      reasons.push({
        type: 'many_links',
        description: `Many links in email: ${features.linkCount}`,
        weight: 0.2,
      });
    }

    // Reply to our email (strong ham signal)
    if (features.isReplyToOurEmail) {
      score -= 0.3;
      reasons.push({
        type: 'is_reply',
        description: 'Email is a reply to our message',
        weight: -0.3,
      });
    }

    return { score: Math.min(1, Math.max(0, score)), weight: 0.35, reasons };
  }

  // ==================== BEHAVIORAL ANALYSIS ====================

  private analyzeBehavior(
    email: DecryptedEmail,
    features: SpamFeatures
  ): { score: number; weight: number; reasons: SpamReason[] } {
    const reasons: SpamReason[] = [];
    let score = 0;

    const senderKey = email.sender.toString();
    const history = this.senderHistory.get(senderKey);

    if (history) {
      // Previously marked as spam
      if (history.marked === 'spam') {
        score += 0.5;
        reasons.push({
          type: 'previously_spam',
          description: 'Previous emails from sender marked as spam',
          weight: 0.5,
        });
      }

      // Previously marked as ham (good)
      if (history.marked === 'ham') {
        score -= 0.3;
        reasons.push({
          type: 'previously_ham',
          description: 'Previous emails from sender marked as legitimate',
          weight: -0.3,
        });
      }

      // High volume sender
      if (history.count > 10) {
        const hoursSinceFirst = (Date.now() - history.lastSeen) / (1000 * 60 * 60);
        const emailsPerHour = history.count / Math.max(1, hoursSinceFirst);
        if (emailsPerHour > 2) {
          score += 0.3;
          reasons.push({
            type: 'high_volume',
            description: `Sender sending many emails: ${emailsPerHour.toFixed(1)}/hour`,
            weight: 0.3,
          });
        }
      }
    }

    // Update history
    this.senderHistory.set(senderKey, {
      count: (history?.count ?? 0) + 1,
      lastSeen: Date.now(),
      marked: history?.marked ?? null,
    });

    return { score: Math.min(1, Math.max(0, score)), weight: 0.25, reasons };
  }

  // ==================== FEATURE EXTRACTION ====================

  private extractFeatures(email: DecryptedEmail, trustScore: TrustScore | null): SpamFeatures {
    const body = email.body;
    const subject = email.subject;
    const text = `${subject} ${body}`;

    // Count links
    const linkMatches = text.match(/https?:\/\/[^\s]+/g);
    const linkCount = linkMatches?.length ?? 0;

    // Count images (markdown or HTML)
    const imageMatches = text.match(/!\[.*?\]\(.*?\)|<img[^>]+>/g);
    const imageCount = imageMatches?.length ?? 0;

    // Caps ratio
    const letters = text.replace(/[^a-zA-Z]/g, '');
    const capsCount = (text.match(/[A-Z]/g) || []).length;
    const capsRatio = letters.length > 0 ? capsCount / letters.length : 0;

    // Punctuation counts
    const exclamationCount = (text.match(/!/g) || []).length;
    const questionCount = (text.match(/\?/g) || []).length;
    const dollarSignCount = (text.match(/[$€£]/g) || []).length;

    // Spam keywords
    let spamKeywordCount = 0;
    const lowerText = text.toLowerCase();
    for (const keyword of this.spamKeywords) {
      if (lowerText.includes(keyword)) spamKeywordCount++;
    }

    // Urgency words
    let urgencyWordCount = 0;
    for (const word of this.urgencyWords) {
      if (lowerText.includes(word)) urgencyWordCount++;
    }

    // Suspicious patterns
    let suspiciousPatternCount = 0;
    for (const pattern of this.suspiciousPatterns) {
      const matches = text.match(pattern);
      if (matches) suspiciousPatternCount += matches.length;
    }

    return {
      // Trust features
      senderTrustScore: trustScore?.combined_score ?? 0,
      senderAttestationCount: trustScore?.attestation_count ?? 0,
      senderByzantineFlags: trustScore?.byzantine_flags?.length ?? 0,
      hasMutualContacts: false, // Would need to check contacts

      // Content features
      subjectLength: subject.length,
      bodyLength: body.length,
      linkCount,
      imageCount,
      attachmentCount: email.attachments?.length ?? 0,
      hasHtmlContent: /<[^>]+>/.test(body),
      capsRatio,
      exclamationCount,
      questionCount,
      dollarSignCount,
      spamKeywordCount,
      urgencyWordCount,
      suspiciousPatternCount,

      // Behavioral features
      isFirstContact: !this.senderHistory.has(email.sender.toString()),
      senderEmailCount: this.senderHistory.get(email.sender.toString())?.count ?? 0,
      senderRecentActivity: 0,
      timeSinceLastEmail: 0,
      isReplyToOurEmail: !!email.in_reply_to,

      // Network features
      senderNetworkSize: 0,
      sharedConnections: 0,
    };
  }

  // ==================== CATEGORIZATION ====================

  private categorize(
    spamProbability: number,
    reasons: SpamReason[]
  ): { category: SpamCategory; action: SpamAction; confidence: number } {
    // Check for phishing indicators
    const hasPhishingIndicators = reasons.some(
      (r) => r.type === 'suspicious_patterns' && r.weight > 0.3
    );
    if (hasPhishingIndicators && spamProbability > 0.6) {
      return { category: 'phishing', action: 'quarantine', confidence: 0.8 };
    }

    // Categorize by probability
    if (spamProbability < 0.2) {
      return { category: 'clean', action: 'deliver', confidence: 1 - spamProbability };
    }
    if (spamProbability < 0.4) {
      return { category: 'suspicious', action: 'deliver_with_warning', confidence: 0.6 };
    }
    if (spamProbability < 0.7) {
      return { category: 'likely_spam', action: 'quarantine', confidence: 0.7 };
    }
    return { category: 'definite_spam', action: 'reject', confidence: spamProbability };
  }

  // ==================== LEARNING ====================

  /**
   * Report email as spam (user feedback)
   */
  reportSpam(email: DecryptedEmail): void {
    if (!this.config.enableAutoLearn) return;

    const senderKey = email.sender.toString();
    const history = this.senderHistory.get(senderKey);
    this.senderHistory.set(senderKey, {
      count: history?.count ?? 1,
      lastSeen: history?.lastSeen ?? Date.now(),
      marked: 'spam',
    });

    // Add to blacklist if reported multiple times
    if (history && history.marked === 'spam') {
      this.config.blacklist.push(email.sender);
    }

    // Update stats
    this.stats.lastUpdated = new Date();
  }

  /**
   * Report email as not spam (user feedback)
   */
  reportNotSpam(email: DecryptedEmail): void {
    if (!this.config.enableAutoLearn) return;

    const senderKey = email.sender.toString();
    const history = this.senderHistory.get(senderKey);
    this.senderHistory.set(senderKey, {
      count: history?.count ?? 1,
      lastSeen: history?.lastSeen ?? Date.now(),
      marked: 'ham',
    });

    // Update stats
    this.stats.falsePositives++;
    this.stats.lastUpdated = new Date();
  }

  // ==================== UTILITIES ====================

  private isWhitelisted(agent: AgentPubKey): boolean {
    return this.config.whitelist.some((w) => w.toString() === agent.toString());
  }

  private isBlacklisted(agent: AgentPubKey): boolean {
    return this.config.blacklist.some((b) => b.toString() === agent.toString());
  }

  private createVerdict(
    category: SpamCategory,
    spamProbability: number,
    confidence: number,
    action: SpamAction,
    reasons: SpamReason[],
    senderTrustScore: number | null,
    features: SpamFeatures
  ): SpamVerdict {
    return {
      isSpam: category !== 'clean' && category !== 'suspicious',
      spamProbability,
      confidence,
      category,
      reasons,
      action,
      senderTrustScore,
      features,
    };
  }

  /**
   * Add sender to whitelist
   */
  addToWhitelist(agent: AgentPubKey): void {
    if (!this.isWhitelisted(agent)) {
      this.config.whitelist.push(agent);
    }
    // Remove from blacklist
    this.config.blacklist = this.config.blacklist.filter(
      (b) => b.toString() !== agent.toString()
    );
  }

  /**
   * Add sender to blacklist
   */
  addToBlacklist(agent: AgentPubKey): void {
    if (!this.isBlacklisted(agent)) {
      this.config.blacklist.push(agent);
    }
    // Remove from whitelist
    this.config.whitelist = this.config.whitelist.filter(
      (w) => w.toString() !== agent.toString()
    );
  }

  /**
   * Get spam statistics
   */
  getStats(): SpamStats {
    const total = this.stats.totalAnalyzed;
    const fp = this.stats.falsePositives;
    const fn = this.stats.falseNegatives;
    const correct = total - fp - fn;

    return {
      ...this.stats,
      accuracy: total > 0 ? correct / total : 1.0,
    };
  }

  /**
   * Update configuration
   */
  updateConfig(config: Partial<SpamConfig>): void {
    this.config = { ...this.config, ...config };
    if (config.customSpamKeywords) {
      for (const keyword of config.customSpamKeywords) {
        this.spamKeywords.add(keyword);
      }
    }
  }
}

export default SpamFilterService;
