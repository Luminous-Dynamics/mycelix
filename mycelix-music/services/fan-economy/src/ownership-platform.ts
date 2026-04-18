// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
/**
 * Fan Economy & Ownership Platform
 *
 * Fractional ownership, fan investments, dynamic pricing, micro-licensing
 */

import { EventEmitter } from 'events';
import { ethers } from 'ethers';

// ============================================================================
// Fractional Track Ownership
// ============================================================================

interface FractionalOwnership {
  trackId: string;
  tokenAddress: string;
  totalShares: number;
  availableShares: number;
  pricePerShare: number;
  currency: 'ETH' | 'USDC' | 'MATIC';
  royaltyPool: number;
  shareholders: Shareholder[];
  vestingSchedule?: VestingSchedule;
}

interface Shareholder {
  userId: string;
  walletAddress: string;
  shares: number;
  purchasePrice: number;
  purchaseDate: Date;
  earnings: number;
  pendingWithdrawal: number;
}

interface VestingSchedule {
  cliffMonths: number;
  vestingMonths: number;
  releasePercentage: number;
}

interface TrackRoyaltyDistribution {
  trackId: string;
  period: { start: Date; end: Date };
  totalRoyalties: number;
  distributions: RoyaltyDistribution[];
  artistShare: number;
  platformFee: number;
  shareholderPool: number;
}

interface RoyaltyDistribution {
  shareholderId: string;
  shares: number;
  percentage: number;
  amount: number;
  status: 'pending' | 'distributed' | 'claimed';
}

export class FractionalOwnershipService extends EventEmitter {
  private provider: ethers.Provider;
  private factoryContract: ethers.Contract;
  private ownerships: Map<string, FractionalOwnership> = new Map();

  constructor(providerUrl: string, factoryAddress: string) {
    super();
    this.provider = new ethers.JsonRpcProvider(providerUrl);
    this.initializeFactory(factoryAddress);
  }

  private async initializeFactory(address: string): Promise<void> {
    const abi = [
      'function createFractionalToken(string trackId, uint256 totalShares, uint256 pricePerShare) returns (address)',
      'function getTokenAddress(string trackId) view returns (address)',
      'event TokenCreated(string trackId, address tokenAddress)',
    ];
    this.factoryContract = new ethers.Contract(address, abi, this.provider);
  }

  async createFractionalOffering(params: {
    trackId: string;
    artistId: string;
    totalShares: number;
    pricePerShare: number;
    artistRetainedShares: number;
    vestingSchedule?: VestingSchedule;
    currency: 'ETH' | 'USDC' | 'MATIC';
  }): Promise<FractionalOwnership> {
    // Deploy new fractional token contract
    const tokenAddress = await this.deployFractionalToken(params);

    const ownership: FractionalOwnership = {
      trackId: params.trackId,
      tokenAddress,
      totalShares: params.totalShares,
      availableShares: params.totalShares - params.artistRetainedShares,
      pricePerShare: params.pricePerShare,
      currency: params.currency,
      royaltyPool: 0,
      shareholders: [
        {
          userId: params.artistId,
          walletAddress: '', // Will be set from artist profile
          shares: params.artistRetainedShares,
          purchasePrice: 0,
          purchaseDate: new Date(),
          earnings: 0,
          pendingWithdrawal: 0,
        },
      ],
      vestingSchedule: params.vestingSchedule,
    };

    this.ownerships.set(params.trackId, ownership);
    this.emit('offeringCreated', ownership);
    return ownership;
  }

  private async deployFractionalToken(params: any): Promise<string> {
    // Deploy ERC-1155 fractional token
    const tx = await this.factoryContract.createFractionalToken(
      params.trackId,
      params.totalShares,
      ethers.parseEther(params.pricePerShare.toString())
    );
    const receipt = await tx.wait();

    // Extract token address from event
    const event = receipt.logs.find((log: any) => log.fragment?.name === 'TokenCreated');
    return event?.args?.tokenAddress || '';
  }

  async purchaseShares(params: {
    trackId: string;
    buyerId: string;
    walletAddress: string;
    shares: number;
  }): Promise<PurchaseResult> {
    const ownership = this.ownerships.get(params.trackId);
    if (!ownership) throw new Error('Track not found');
    if (ownership.availableShares < params.shares) {
      throw new Error('Insufficient shares available');
    }

    const totalCost = params.shares * ownership.pricePerShare;

    // Execute blockchain transaction
    const txHash = await this.executeSharePurchase(
      ownership.tokenAddress,
      params.walletAddress,
      params.shares,
      totalCost
    );

    // Update ownership records
    ownership.availableShares -= params.shares;
    ownership.shareholders.push({
      userId: params.buyerId,
      walletAddress: params.walletAddress,
      shares: params.shares,
      purchasePrice: totalCost,
      purchaseDate: new Date(),
      earnings: 0,
      pendingWithdrawal: 0,
    });

    this.emit('sharesPurchased', {
      trackId: params.trackId,
      buyerId: params.buyerId,
      shares: params.shares,
      cost: totalCost,
      txHash,
    });

    return {
      success: true,
      shares: params.shares,
      totalCost,
      txHash,
      newOwnershipPercentage: (params.shares / ownership.totalShares) * 100,
    };
  }

  private async executeSharePurchase(
    tokenAddress: string,
    buyer: string,
    shares: number,
    amount: number
  ): Promise<string> {
    // Execute on-chain purchase
    return '0x...';
  }

  async distributeRoyalties(trackId: string, royaltyAmount: number): Promise<TrackRoyaltyDistribution> {
    const ownership = this.ownerships.get(trackId);
    if (!ownership) throw new Error('Track not found');

    const platformFee = royaltyAmount * 0.05; // 5% platform fee
    const artistShare = royaltyAmount * 0.50; // 50% to artist
    const shareholderPool = royaltyAmount - platformFee - artistShare;

    const distributions: RoyaltyDistribution[] = ownership.shareholders
      .filter(s => s.userId !== ownership.shareholders[0].userId) // Exclude artist
      .map(shareholder => ({
        shareholderId: shareholder.userId,
        shares: shareholder.shares,
        percentage: (shareholder.shares / ownership.totalShares) * 100,
        amount: (shareholder.shares / ownership.totalShares) * shareholderPool,
        status: 'pending' as const,
      }));

    // Update pending withdrawals
    for (const dist of distributions) {
      const shareholder = ownership.shareholders.find(s => s.userId === dist.shareholderId);
      if (shareholder) {
        shareholder.pendingWithdrawal += dist.amount;
        shareholder.earnings += dist.amount;
      }
    }

    ownership.royaltyPool += shareholderPool;

    const distribution: TrackRoyaltyDistribution = {
      trackId,
      period: { start: new Date(), end: new Date() },
      totalRoyalties: royaltyAmount,
      distributions,
      artistShare,
      platformFee,
      shareholderPool,
    };

    this.emit('royaltiesDistributed', distribution);
    return distribution;
  }

  async claimRoyalties(trackId: string, userId: string): Promise<ClaimResult> {
    const ownership = this.ownerships.get(trackId);
    if (!ownership) throw new Error('Track not found');

    const shareholder = ownership.shareholders.find(s => s.userId === userId);
    if (!shareholder) throw new Error('Not a shareholder');
    if (shareholder.pendingWithdrawal <= 0) {
      throw new Error('No pending royalties');
    }

    const amount = shareholder.pendingWithdrawal;

    // Execute on-chain withdrawal
    const txHash = await this.executeRoyaltyClaim(
      ownership.tokenAddress,
      shareholder.walletAddress,
      amount
    );

    shareholder.pendingWithdrawal = 0;

    return {
      success: true,
      amount,
      txHash,
    };
  }

  private async executeRoyaltyClaim(
    tokenAddress: string,
    recipient: string,
    amount: number
  ): Promise<string> {
    return '0x...';
  }

  async getShareholderDashboard(userId: string): Promise<ShareholderDashboard> {
    const holdings: TrackHolding[] = [];

    for (const [trackId, ownership] of this.ownerships) {
      const shareholder = ownership.shareholders.find(s => s.userId === userId);
      if (shareholder) {
        holdings.push({
          trackId,
          shares: shareholder.shares,
          percentage: (shareholder.shares / ownership.totalShares) * 100,
          currentValue: shareholder.shares * ownership.pricePerShare,
          purchaseValue: shareholder.purchasePrice,
          totalEarnings: shareholder.earnings,
          pendingRoyalties: shareholder.pendingWithdrawal,
        });
      }
    }

    return {
      userId,
      totalHoldings: holdings.length,
      totalValue: holdings.reduce((sum, h) => sum + h.currentValue, 0),
      totalEarnings: holdings.reduce((sum, h) => sum + h.totalEarnings, 0),
      pendingRoyalties: holdings.reduce((sum, h) => sum + h.pendingRoyalties, 0),
      holdings,
    };
  }
}

// ============================================================================
// Fan-Funded Campaigns
// ============================================================================

interface FundingCampaign {
  id: string;
  artistId: string;
  title: string;
  description: string;
  type: 'album' | 'tour' | 'music_video' | 'single' | 'equipment';
  goal: number;
  raised: number;
  currency: string;
  startDate: Date;
  endDate: Date;
  status: 'draft' | 'active' | 'funded' | 'failed' | 'completed';
  tiers: FundingTier[];
  backers: Backer[];
  updates: CampaignUpdate[];
  revenueShare?: RevenueShareTerms;
}

interface FundingTier {
  id: string;
  name: string;
  amount: number;
  description: string;
  rewards: string[];
  limit?: number;
  claimed: number;
  deliveryDate?: Date;
}

interface Backer {
  userId: string;
  tierId: string;
  amount: number;
  pledgeDate: Date;
  rewardStatus: 'pending' | 'processing' | 'delivered';
  revenueSharePercentage?: number;
}

interface RevenueShareTerms {
  enabled: boolean;
  minimumTier: string;
  percentagePerDollar: number;
  maxPercentage: number;
  duration: 'perpetual' | 'years';
  years?: number;
}

interface CampaignUpdate {
  id: string;
  date: Date;
  title: string;
  content: string;
  mediaUrls: string[];
  backersOnly: boolean;
}

export class FanFundingPlatform extends EventEmitter {
  private campaigns: Map<string, FundingCampaign> = new Map();
  private escrowService: EscrowService;

  constructor() {
    super();
    this.escrowService = new EscrowService();
  }

  async createCampaign(params: {
    artistId: string;
    title: string;
    description: string;
    type: FundingCampaign['type'];
    goal: number;
    duration: number; // days
    tiers: Omit<FundingTier, 'id' | 'claimed'>[];
    revenueShare?: RevenueShareTerms;
  }): Promise<FundingCampaign> {
    const campaign: FundingCampaign = {
      id: generateId(),
      artistId: params.artistId,
      title: params.title,
      description: params.description,
      type: params.type,
      goal: params.goal,
      raised: 0,
      currency: 'USD',
      startDate: new Date(),
      endDate: new Date(Date.now() + params.duration * 24 * 60 * 60 * 1000),
      status: 'active',
      tiers: params.tiers.map((t, i) => ({ ...t, id: `tier-${i}`, claimed: 0 })),
      backers: [],
      updates: [],
      revenueShare: params.revenueShare,
    };

    this.campaigns.set(campaign.id, campaign);
    this.emit('campaignCreated', campaign);
    return campaign;
  }

  async backCampaign(params: {
    campaignId: string;
    userId: string;
    tierId: string;
    amount: number;
    paymentMethodId: string;
  }): Promise<BackingResult> {
    const campaign = this.campaigns.get(params.campaignId);
    if (!campaign) throw new Error('Campaign not found');
    if (campaign.status !== 'active') throw new Error('Campaign not active');

    const tier = campaign.tiers.find(t => t.id === params.tierId);
    if (!tier) throw new Error('Tier not found');
    if (tier.limit && tier.claimed >= tier.limit) {
      throw new Error('Tier sold out');
    }
    if (params.amount < tier.amount) {
      throw new Error('Amount below tier minimum');
    }

    // Process payment into escrow
    const escrowId = await this.escrowService.holdFunds({
      amount: params.amount,
      paymentMethodId: params.paymentMethodId,
      releaseCondition: 'campaign_funded',
      campaignId: params.campaignId,
    });

    // Calculate revenue share if applicable
    let revenueSharePercentage = 0;
    if (campaign.revenueShare?.enabled && params.amount >= tier.amount) {
      revenueSharePercentage = Math.min(
        params.amount * campaign.revenueShare.percentagePerDollar,
        campaign.revenueShare.maxPercentage
      );
    }

    const backer: Backer = {
      userId: params.userId,
      tierId: params.tierId,
      amount: params.amount,
      pledgeDate: new Date(),
      rewardStatus: 'pending',
      revenueSharePercentage,
    };

    campaign.backers.push(backer);
    campaign.raised += params.amount;
    tier.claimed++;

    // Check if goal reached
    if (campaign.raised >= campaign.goal && campaign.status === 'active') {
      campaign.status = 'funded';
      await this.escrowService.releaseFunds(params.campaignId);
      this.emit('campaignFunded', campaign);
    }

    this.emit('campaignBacked', { campaign, backer });

    return {
      success: true,
      escrowId,
      revenueSharePercentage,
      estimatedDelivery: tier.deliveryDate,
    };
  }

  async postUpdate(campaignId: string, update: Omit<CampaignUpdate, 'id' | 'date'>): Promise<void> {
    const campaign = this.campaigns.get(campaignId);
    if (!campaign) throw new Error('Campaign not found');

    const newUpdate: CampaignUpdate = {
      id: generateId(),
      date: new Date(),
      ...update,
    };

    campaign.updates.push(newUpdate);

    // Notify backers
    const backerIds = update.backersOnly
      ? campaign.backers.map(b => b.userId)
      : []; // All followers if public

    this.emit('updatePosted', { campaignId, update: newUpdate, notifyUsers: backerIds });
  }

  async processRevenueShare(campaignId: string, revenue: number): Promise<RevenueShareDistribution[]> {
    const campaign = this.campaigns.get(campaignId);
    if (!campaign || !campaign.revenueShare?.enabled) {
      throw new Error('Revenue sharing not enabled');
    }

    const distributions: RevenueShareDistribution[] = [];

    for (const backer of campaign.backers) {
      if (backer.revenueSharePercentage && backer.revenueSharePercentage > 0) {
        const share = revenue * (backer.revenueSharePercentage / 100);
        distributions.push({
          userId: backer.userId,
          percentage: backer.revenueSharePercentage,
          amount: share,
        });
      }
    }

    this.emit('revenueShared', { campaignId, revenue, distributions });
    return distributions;
  }

  async getCampaignAnalytics(campaignId: string): Promise<CampaignAnalytics> {
    const campaign = this.campaigns.get(campaignId);
    if (!campaign) throw new Error('Campaign not found');

    const tierBreakdown = campaign.tiers.map(tier => ({
      tierId: tier.id,
      name: tier.name,
      backers: campaign.backers.filter(b => b.tierId === tier.id).length,
      revenue: campaign.backers
        .filter(b => b.tierId === tier.id)
        .reduce((sum, b) => sum + b.amount, 0),
      conversionRate: tier.limit ? (tier.claimed / tier.limit) * 100 : 0,
    }));

    return {
      campaignId,
      totalBackers: campaign.backers.length,
      totalRaised: campaign.raised,
      percentFunded: (campaign.raised / campaign.goal) * 100,
      averagePledge: campaign.raised / campaign.backers.length || 0,
      tierBreakdown,
      dailyPledges: this.calculateDailyPledges(campaign),
      referralSources: this.calculateReferralSources(campaign),
    };
  }

  private calculateDailyPledges(campaign: FundingCampaign): Array<{ date: string; amount: number }> {
    // Group pledges by day
    return [];
  }

  private calculateReferralSources(campaign: FundingCampaign): Array<{ source: string; count: number }> {
    return [];
  }
}

// ============================================================================
// Dynamic Pricing Engine
// ============================================================================

interface PricingModel {
  basePrice: number;
  demandMultiplier: number;
  supplyFactor: number;
  timeDecay: number;
  competitionAdjustment: number;
  loyaltyDiscount: number;
}

interface DemandSignal {
  trackId: string;
  timestamp: Date;
  type: 'play' | 'save' | 'share' | 'purchase' | 'playlist_add';
  weight: number;
}

export class DynamicPricingEngine extends EventEmitter {
  private demandHistory: Map<string, DemandSignal[]> = new Map();
  private pricingModels: Map<string, PricingModel> = new Map();
  private currentPrices: Map<string, number> = new Map();

  async calculatePrice(trackId: string, context: PricingContext): Promise<PriceResult> {
    const model = this.pricingModels.get(trackId) || this.getDefaultModel();
    const signals = this.demandHistory.get(trackId) || [];

    // Calculate demand score
    const demandScore = this.calculateDemandScore(signals);

    // Calculate supply factor (scarcity)
    const supplyFactor = await this.calculateSupplyFactor(trackId);

    // Time-based adjustments
    const timeMultiplier = this.calculateTimeMultiplier(trackId, context.timestamp);

    // User-specific adjustments
    const userDiscount = await this.calculateUserDiscount(context.userId);

    // Competition analysis
    const competitionFactor = await this.analyzeCompetition(trackId);

    // Calculate final price
    const price = model.basePrice
      * (1 + demandScore * model.demandMultiplier)
      * supplyFactor
      * timeMultiplier
      * (1 - userDiscount)
      * competitionFactor;

    const result: PriceResult = {
      trackId,
      price: Math.round(price * 100) / 100, // Round to cents
      basePrice: model.basePrice,
      adjustments: {
        demand: demandScore * model.demandMultiplier,
        supply: supplyFactor,
        time: timeMultiplier,
        loyalty: -userDiscount,
        competition: competitionFactor,
      },
      validUntil: new Date(Date.now() + 5 * 60 * 1000), // 5 minute validity
    };

    this.currentPrices.set(trackId, result.price);
    this.emit('priceCalculated', result);
    return result;
  }

  private calculateDemandScore(signals: DemandSignal[]): number {
    const now = Date.now();
    const hourAgo = now - 60 * 60 * 1000;
    const dayAgo = now - 24 * 60 * 60 * 1000;

    // Weight recent signals more heavily
    let score = 0;
    for (const signal of signals) {
      const age = now - signal.timestamp.getTime();
      const recency = age < hourAgo ? 2 : age < dayAgo ? 1 : 0.5;
      score += signal.weight * recency;
    }

    // Normalize to 0-1 range
    return Math.min(score / 1000, 1);
  }

  private async calculateSupplyFactor(trackId: string): Promise<number> {
    // Check if track has limited editions
    const limitedEdition = await this.checkLimitedEdition(trackId);
    if (limitedEdition) {
      const remaining = limitedEdition.remaining / limitedEdition.total;
      return 1 + (1 - remaining) * 0.5; // Up to 50% premium for scarcity
    }
    return 1;
  }

  private calculateTimeMultiplier(trackId: string, timestamp: Date): number {
    // Consider release date, time of day, day of week
    const hour = timestamp.getHours();
    const dayOfWeek = timestamp.getDay();

    // Peak hours (evening) get slight premium
    const hourMultiplier = hour >= 18 && hour <= 22 ? 1.05 : 1;

    // Weekends slightly higher
    const weekendMultiplier = dayOfWeek === 0 || dayOfWeek === 6 ? 1.02 : 1;

    return hourMultiplier * weekendMultiplier;
  }

  private async calculateUserDiscount(userId?: string): Promise<number> {
    if (!userId) return 0;

    // Check user loyalty tier
    const loyalty = await this.getUserLoyalty(userId);

    const discounts: Record<string, number> = {
      bronze: 0.02,
      silver: 0.05,
      gold: 0.10,
      platinum: 0.15,
    };

    return discounts[loyalty.tier] || 0;
  }

  private async analyzeCompetition(trackId: string): Promise<number> {
    // Compare with similar tracks' prices
    const similarTracks = await this.getSimilarTracks(trackId);
    const avgPrice = similarTracks.reduce((sum, t) => sum + t.price, 0) / similarTracks.length;

    const currentPrice = this.currentPrices.get(trackId) || 0;

    // Adjust if significantly different from market
    if (currentPrice > avgPrice * 1.2) {
      return 0.95; // Reduce by 5%
    }
    if (currentPrice < avgPrice * 0.8) {
      return 1.05; // Increase by 5%
    }
    return 1;
  }

  recordDemandSignal(signal: DemandSignal): void {
    const signals = this.demandHistory.get(signal.trackId) || [];
    signals.push(signal);

    // Keep only last 1000 signals
    if (signals.length > 1000) {
      signals.shift();
    }

    this.demandHistory.set(signal.trackId, signals);
    this.emit('demandSignal', signal);
  }

  async getBulkPricing(trackIds: string[], context: PricingContext): Promise<Map<string, PriceResult>> {
    const results = new Map<string, PriceResult>();

    await Promise.all(
      trackIds.map(async (trackId) => {
        const price = await this.calculatePrice(trackId, context);
        results.set(trackId, price);
      })
    );

    return results;
  }

  private getDefaultModel(): PricingModel {
    return {
      basePrice: 0.99,
      demandMultiplier: 0.5,
      supplyFactor: 1,
      timeDecay: 0.1,
      competitionAdjustment: 0.1,
      loyaltyDiscount: 0.15,
    };
  }

  private async checkLimitedEdition(trackId: string): Promise<{ remaining: number; total: number } | null> {
    return null;
  }

  private async getUserLoyalty(userId: string): Promise<{ tier: string }> {
    return { tier: 'bronze' };
  }

  private async getSimilarTracks(trackId: string): Promise<Array<{ price: number }>> {
    return [];
  }
}

// ============================================================================
// Micro-Licensing Marketplace
// ============================================================================

interface License {
  id: string;
  trackId: string;
  type: LicenseType;
  terms: LicenseTerms;
  price: number;
  currency: string;
  artistId: string;
  status: 'available' | 'sold' | 'expired';
}

type LicenseType =
  | 'sync_social'      // Social media (TikTok, Reels, etc.)
  | 'sync_youtube'     // YouTube videos
  | 'sync_podcast'     // Podcast background
  | 'sync_commercial'  // Commercial use
  | 'sync_film'        // Film/TV
  | 'sample'           // Sampling rights
  | 'cover'            // Cover recording
  | 'remix'            // Official remix
  | 'stems'            // Access to stems
  | 'exclusive';       // Exclusive rights

interface LicenseTerms {
  duration: 'perpetual' | 'limited';
  durationMonths?: number;
  territory: 'worldwide' | 'region' | 'country';
  territories?: string[];
  platforms: string[];
  exclusivity: 'non-exclusive' | 'exclusive' | 'semi-exclusive';
  attribution: boolean;
  modifications: boolean;
  commercialUse: boolean;
  maxViews?: number;
  maxRevenue?: number;
}

interface LicensePurchase {
  id: string;
  licenseId: string;
  buyerId: string;
  buyerCompany?: string;
  projectDescription: string;
  purchaseDate: Date;
  expirationDate?: Date;
  price: number;
  agreement: string; // URL to signed agreement
  usageReports: UsageReport[];
}

interface UsageReport {
  date: Date;
  platform: string;
  views: number;
  revenue?: number;
}

export class MicroLicensingMarketplace extends EventEmitter {
  private licenses: Map<string, License> = new Map();
  private purchases: Map<string, LicensePurchase> = new Map();
  private priceCalculator: LicensePriceCalculator;

  constructor() {
    super();
    this.priceCalculator = new LicensePriceCalculator();
  }

  async createLicense(params: {
    trackId: string;
    artistId: string;
    type: LicenseType;
    terms: LicenseTerms;
    customPrice?: number;
  }): Promise<License> {
    // Calculate price if not custom
    const price = params.customPrice ||
      await this.priceCalculator.calculate(params.type, params.terms, params.trackId);

    const license: License = {
      id: generateId(),
      trackId: params.trackId,
      type: params.type,
      terms: params.terms,
      price,
      currency: 'USD',
      artistId: params.artistId,
      status: 'available',
    };

    this.licenses.set(license.id, license);
    this.emit('licenseCreated', license);
    return license;
  }

  async searchLicenses(query: {
    type?: LicenseType[];
    priceRange?: { min: number; max: number };
    duration?: string;
    territory?: string;
    genre?: string;
    mood?: string;
    tempo?: { min: number; max: number };
  }): Promise<LicenseSearchResult[]> {
    const results: LicenseSearchResult[] = [];

    for (const [, license] of this.licenses) {
      if (license.status !== 'available') continue;

      // Apply filters
      if (query.type && !query.type.includes(license.type)) continue;
      if (query.priceRange) {
        if (license.price < query.priceRange.min || license.price > query.priceRange.max) continue;
      }

      // Get track details
      const track = await this.getTrackDetails(license.trackId);

      results.push({
        license,
        track,
        matchScore: this.calculateMatchScore(license, query),
      });
    }

    return results.sort((a, b) => b.matchScore - a.matchScore);
  }

  async purchaseLicense(params: {
    licenseId: string;
    buyerId: string;
    buyerCompany?: string;
    projectDescription: string;
    paymentMethodId: string;
  }): Promise<LicensePurchase> {
    const license = this.licenses.get(params.licenseId);
    if (!license) throw new Error('License not found');
    if (license.status !== 'available') throw new Error('License not available');

    // Process payment
    await this.processPayment(params.paymentMethodId, license.price);

    // Generate agreement
    const agreement = await this.generateAgreement(license, params);

    const purchase: LicensePurchase = {
      id: generateId(),
      licenseId: params.licenseId,
      buyerId: params.buyerId,
      buyerCompany: params.buyerCompany,
      projectDescription: params.projectDescription,
      purchaseDate: new Date(),
      expirationDate: license.terms.duration === 'limited'
        ? new Date(Date.now() + (license.terms.durationMonths || 12) * 30 * 24 * 60 * 60 * 1000)
        : undefined,
      price: license.price,
      agreement,
      usageReports: [],
    };

    // Update license status if exclusive
    if (license.terms.exclusivity === 'exclusive') {
      license.status = 'sold';
    }

    this.purchases.set(purchase.id, purchase);

    // Distribute payment to artist
    await this.distributePayment(license.artistId, license.price);

    this.emit('licensePurchased', { purchase, license });
    return purchase;
  }

  async reportUsage(purchaseId: string, report: Omit<UsageReport, 'date'>): Promise<void> {
    const purchase = this.purchases.get(purchaseId);
    if (!purchase) throw new Error('Purchase not found');

    purchase.usageReports.push({
      date: new Date(),
      ...report,
    });

    // Check if usage limits exceeded
    const license = this.licenses.get(purchase.licenseId);
    if (license?.terms.maxViews) {
      const totalViews = purchase.usageReports.reduce((sum, r) => sum + r.views, 0);
      if (totalViews > license.terms.maxViews) {
        this.emit('usageLimitExceeded', { purchaseId, totalViews, limit: license.terms.maxViews });
      }
    }

    this.emit('usageReported', { purchaseId, report });
  }

  async getArtistLicensingDashboard(artistId: string): Promise<LicensingDashboard> {
    const artistLicenses = Array.from(this.licenses.values())
      .filter(l => l.artistId === artistId);

    const artistPurchases = Array.from(this.purchases.values())
      .filter(p => {
        const license = this.licenses.get(p.licenseId);
        return license?.artistId === artistId;
      });

    return {
      totalLicenses: artistLicenses.length,
      activeLicenses: artistLicenses.filter(l => l.status === 'available').length,
      soldLicenses: artistPurchases.length,
      totalRevenue: artistPurchases.reduce((sum, p) => sum + p.price, 0),
      revenueByType: this.groupRevenueByType(artistPurchases),
      recentSales: artistPurchases.slice(-10),
      topLicensedTracks: this.getTopLicensedTracks(artistPurchases),
    };
  }

  private async getTrackDetails(trackId: string): Promise<any> {
    return {};
  }

  private calculateMatchScore(license: License, query: any): number {
    return Math.random(); // Placeholder
  }

  private async processPayment(methodId: string, amount: number): Promise<void> {
    // Process payment
  }

  private async generateAgreement(license: License, buyer: any): Promise<string> {
    return 'https://agreements.mycelix.io/...';
  }

  private async distributePayment(artistId: string, amount: number): Promise<void> {
    const platformFee = amount * 0.15; // 15% platform fee
    const artistPayout = amount - platformFee;
    // Transfer to artist
  }

  private groupRevenueByType(purchases: LicensePurchase[]): Record<string, number> {
    const result: Record<string, number> = {};
    for (const purchase of purchases) {
      const license = this.licenses.get(purchase.licenseId);
      if (license) {
        result[license.type] = (result[license.type] || 0) + purchase.price;
      }
    }
    return result;
  }

  private getTopLicensedTracks(purchases: LicensePurchase[]): Array<{ trackId: string; count: number; revenue: number }> {
    const trackStats = new Map<string, { count: number; revenue: number }>();

    for (const purchase of purchases) {
      const license = this.licenses.get(purchase.licenseId);
      if (license) {
        const stats = trackStats.get(license.trackId) || { count: 0, revenue: 0 };
        stats.count++;
        stats.revenue += purchase.price;
        trackStats.set(license.trackId, stats);
      }
    }

    return Array.from(trackStats.entries())
      .map(([trackId, stats]) => ({ trackId, ...stats }))
      .sort((a, b) => b.revenue - a.revenue)
      .slice(0, 10);
  }
}

// ============================================================================
// Type Definitions & Helpers
// ============================================================================

interface PurchaseResult {
  success: boolean;
  shares: number;
  totalCost: number;
  txHash: string;
  newOwnershipPercentage: number;
}

interface ClaimResult {
  success: boolean;
  amount: number;
  txHash: string;
}

interface ShareholderDashboard {
  userId: string;
  totalHoldings: number;
  totalValue: number;
  totalEarnings: number;
  pendingRoyalties: number;
  holdings: TrackHolding[];
}

interface TrackHolding {
  trackId: string;
  shares: number;
  percentage: number;
  currentValue: number;
  purchaseValue: number;
  totalEarnings: number;
  pendingRoyalties: number;
}

interface BackingResult {
  success: boolean;
  escrowId: string;
  revenueSharePercentage: number;
  estimatedDelivery?: Date;
}

interface RevenueShareDistribution {
  userId: string;
  percentage: number;
  amount: number;
}

interface CampaignAnalytics {
  campaignId: string;
  totalBackers: number;
  totalRaised: number;
  percentFunded: number;
  averagePledge: number;
  tierBreakdown: any[];
  dailyPledges: any[];
  referralSources: any[];
}

interface PricingContext {
  userId?: string;
  timestamp: Date;
  platform?: string;
  country?: string;
}

interface PriceResult {
  trackId: string;
  price: number;
  basePrice: number;
  adjustments: Record<string, number>;
  validUntil: Date;
}

interface LicenseSearchResult {
  license: License;
  track: any;
  matchScore: number;
}

interface LicensingDashboard {
  totalLicenses: number;
  activeLicenses: number;
  soldLicenses: number;
  totalRevenue: number;
  revenueByType: Record<string, number>;
  recentSales: LicensePurchase[];
  topLicensedTracks: any[];
}

function generateId(): string {
  return Math.random().toString(36).substring(2, 15);
}

class EscrowService {
  async holdFunds(params: any): Promise<string> { return 'escrow-id'; }
  async releaseFunds(campaignId: string): Promise<void> {}
}

class LicensePriceCalculator {
  async calculate(type: LicenseType, terms: LicenseTerms, trackId: string): Promise<number> {
    const basePrices: Record<LicenseType, number> = {
      sync_social: 15,
      sync_youtube: 25,
      sync_podcast: 20,
      sync_commercial: 500,
      sync_film: 2000,
      sample: 100,
      cover: 50,
      remix: 200,
      stems: 100,
      exclusive: 10000,
    };
    return basePrices[type] || 100;
  }
}
