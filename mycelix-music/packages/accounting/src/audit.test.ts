import { describe, expect, it } from 'vitest';
import { createEconomicAuditCapsule, economicAuditCapsuleDigest } from './audit.js';
import { money } from './money.js';

function capsule() {
  return {
    protocolVersion: 1 as const,
    usageCommitment: 'usage-root',
    repertoireSnapshot: 'repertoire-root',
    rightsPolicy: 'rights-policy-root',
    economicTerms: 'terms-root',
    rightsResolutionRoot: 'rights-resolution-root',
    obligationRoot: 'obligation-root',
    adjustmentRoot: 'adjustment-root',
    nettingRoot: 'netting-root',
    settlementPlanRoot: 'settlement-plan-root',
    paymentReceiptRoot: 'payment-root',
    conservationProof: {
      inputTotal: money(1000n),
      distributedTotal: money(800n),
      heldTotal: money(100n),
      deductionTotal: money(50n),
      residualTotal: money(50n),
    },
    generatedBy: {
      id: 'mycelix-music-royalty-compiler',
      version: '1.0.0',
      buildDigest: 'compiler-build-root',
    },
  };
}

describe('EconomicAuditCapsule v1', () => {
  it('binds the full epoch lineage to one reproducible digest', () => {
    const first = createEconomicAuditCapsule(capsule());
    const second = createEconomicAuditCapsule(capsule());
    expect(economicAuditCapsuleDigest(second)).toBe(economicAuditCapsuleDigest(first));
  });

  it('changes identity when any authoritative root changes', () => {
    const first = createEconomicAuditCapsule(capsule());
    const second = createEconomicAuditCapsule({ ...capsule(), obligationRoot: 'different-obligation-root' });
    expect(economicAuditCapsuleDigest(second)).not.toBe(economicAuditCapsuleDigest(first));
  });

  it('refuses a capsule whose conservation proof loses one unit', () => {
    expect(() => createEconomicAuditCapsule({
      ...capsule(),
      conservationProof: { ...capsule().conservationProof, residualTotal: money(49n) },
    })).toThrow(/does not conserve/);
  });
});
