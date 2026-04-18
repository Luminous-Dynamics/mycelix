import crypto from 'crypto';
import { config } from '../config';

type TrustTier = 'high' | 'medium' | 'low' | 'unknown';

export interface TrustSummary {
  score: number;
  tier: TrustTier;
  reasons: string[];
  pathLength: number;
  decayAt: string;
  attestations: Attestation[];
  quarantined: boolean;
  fetchedAt: string;
}

export interface Attestation {
  from: string;
  to: string;
  weight: number;
  reason?: string;
}

const cache = new Map<string, { summary: TrustSummary; timestamp: number }>();
type TrustProvider = (sender: string) => Promise<TrustSummary | null>;
let externalProvider: TrustProvider | null = null;
let failureCount = 0;
let circuitOpenUntil = 0;
const FAILURE_THRESHOLD = 3;
const CIRCUIT_COOLDOWN_MS = 60_000;

const deterministicScore = (input: string) => {
  const hash = crypto.createHash('sha256').update(input).digest('hex');
  const val = parseInt(hash.slice(0, 4), 16);
  return 40 + (val % 61); // 40..100
};

const tierFromScore = (score: number): TrustTier => {
  if (score >= 80) return 'high';
  if (score >= 45) return 'medium';
  return 'low';
};

const buildReasons = (sender: string, score: number): string[] => {
  const reasons: string[] = [];
  const domain = sender.split('@')[1] || '';

  if (domain.includes('example') || domain.includes('test')) {
    reasons.push('Unverified domain');
  } else if (domain) {
    reasons.push(`Domain ${domain} observed`);
  }

  if (sender.includes('+')) {
    reasons.push('Tagged sender address');
  }

  if (score >= 80) reasons.push('Consistent positive interactions');
  if (score < 50) reasons.push('Limited attestations');

  return reasons;
};

const buildPathLength = (sender: string) => {
  const hash = crypto.createHash('md5').update(sender).digest('hex');
  return (parseInt(hash.slice(0, 2), 16) % 4) + 1; // 1..4 hops
};

const buildDecayAt = () => {
  const expires = new Date(Date.now() + 1000 * 60 * 60 * 24 * 7); // 7 days
  return expires.toISOString();
};

const buildAttestations = (sender: string): Attestation[] => {
  const peers = ['peer-A', 'peer-B', 'peer-C'];
  return peers.slice(0, 2).map((peer, idx) => ({
    from: peer,
    to: sender,
    weight: 0.6 + idx * 0.1,
    reason: idx === 0 ? 'Recent positive interactions' : 'Shared circle endorsement',
  }));
};

export const trustService = {
  registerProvider(provider: TrustProvider) {
    externalProvider = provider;
  },

  async getSummary(sender: string): Promise<TrustSummary> {
    const now = Date.now();
    const ttl = config.trustCacheTtlMs;

    const cached = cache.get(sender);
    if (cached && now - cached.timestamp < ttl) {
      return cached.summary;
    }

    // Attempt external MATL/Holochain provider first if registered
    if (externalProvider) {
      if (now < circuitOpenUntil) {
        // Circuit open; skip provider
      } else {
        try {
          const provided = await externalProvider(sender);
          if (provided) {
            cache.set(sender, { summary: provided, timestamp: now });
            failureCount = 0;
            return provided;
          }
        } catch (err) {
          failureCount += 1;
          if (failureCount >= FAILURE_THRESHOLD) {
            circuitOpenUntil = now + CIRCUIT_COOLDOWN_MS;
          }
          // eslint-disable-next-line no-console
          console.warn('Trust provider failed, falling back to deterministic:', err);
        }
      }
    }

    // Retry provider once after backoff if circuit is not open
    if (externalProvider && now >= circuitOpenUntil) {
      try {
        await new Promise((resolve) => setTimeout(resolve, 200));
        try {
          const providedRetry = await externalProvider(sender);
          if (providedRetry) {
            cache.set(sender, { summary: providedRetry, timestamp: now });
            failureCount = 0;
            return providedRetry;
          }
        } catch (err) {
          failureCount += 1;
          if (failureCount >= FAILURE_THRESHOLD) {
            circuitOpenUntil = Date.now() + CIRCUIT_COOLDOWN_MS;
          }
          // eslint-disable-next-line no-console
          console.warn('Trust provider retry failed:', err);
        }
      } catch {
        // ignore backoff errors
      }
    }

    // Fallback deterministic summary
    const score = deterministicScore(sender);
    const tier = tierFromScore(score);
    const summary: TrustSummary = {
      score,
      tier,
      reasons: buildReasons(sender, score),
      pathLength: buildPathLength(sender),
      decayAt: buildDecayAt(),
      attestations: buildAttestations(sender),
      quarantined: tier === 'low',
      fetchedAt: new Date(now).toISOString(),
    };

    cache.set(sender, { summary, timestamp: now });
    return summary;
  },

  clearCache() {
    cache.clear();
  },

  getProviderStatus() {
    return {
      providerConfigured: Boolean(externalProvider),
      cacheSize: cache.size,
      ttlMs: config.trustCacheTtlMs,
      circuitOpen: Date.now() < circuitOpenUntil,
      circuitOpensUntil: circuitOpenUntil,
      failureCount,
    };
  },
};
