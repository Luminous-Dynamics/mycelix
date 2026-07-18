import type { AppClient } from '@holochain/client';

export interface EpistemicPosition {
  empirical: number;
  normative: number;
  mythic: number;
}

export interface Claim {
  id: string;
  content: string;
  classification: EpistemicPosition;
  [key: string]: unknown;
}

export interface EnhancedCredibilityScore {
  overallScore: number;
  [key: string]: unknown;
}

export interface InformationValue {
  id: string;
  claimId: string;
  expectedValue: number;
  dependentCount: number;
  averageDependencyWeight: number;
  uncertainty: number;
  impactScore: number;
  recommendedForVerification: boolean;
  assessedAt: number;
  reasoning: string;
}

export interface AuthorReputation {
  overallScore: number;
  claimsAuthored: number;
  claimsVerifiedTrue: number;
  claimsVerifiedFalse: number;
  historicalAccuracy: number;
  matlTrust: number;
}

export interface CreateClaimInput {
  content: string;
  classification: EpistemicPosition;
  domain: string;
  topics: string[];
  sources: Array<{
    uri: string;
    title: string;
    author?: string;
    publishedAt?: number;
    reliability?: number;
  }>;
  evidence: unknown[];
}

export interface VerificationRecommendation {
  recommend: boolean;
  reason: string;
  suggestedTargetE: number;
}

export class KnowledgeClientUnavailableError extends Error {}

export class KnowledgeClient {
  constructor(appClient: AppClient, roleName?: string);
  query: {
    search(uri: string, options?: { minE?: number; limit?: number }): Promise<Claim[]>;
  };
  graph: {
    getDependencyTree(
      claimId: string,
      depth: number
    ): Promise<{ totalDependencies: number; aggregateWeight: number }>;
  };
  inference: {
    calculateEnhancedCredibility(
      subjectId: string,
      subjectType: string
    ): Promise<EnhancedCredibilityScore>;
    getAuthorReputation(authorDid: string): Promise<AuthorReputation>;
  };
  marketsIntegration: {
    requestVerificationMarket(
      claimId: string,
      targetE: number,
      minConfidence: number,
      closesAt: number,
      tags?: string[]
    ): Promise<string | unknown>;
  };
}

export class KnowledgeService {
  constructor(appClient: AppClient, roleName?: string);
  submitAndAnalyzeClaim(input: CreateClaimInput): Promise<unknown>;
}

export function calculateInformationValue(
  uncertainty: number,
  dependentCount: number,
  averageDependencyWeight: number
): number;

export function recommendVerification(
  claim: Claim,
  informationValue: InformationValue
): VerificationRecommendation;

export function toDiscreteEpistemic(position: EpistemicPosition): {
  empirical: 'low' | 'medium' | 'high';
  normative: 'low' | 'medium' | 'high';
  mythic: 'low' | 'medium' | 'high';
};
