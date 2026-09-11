import { generateKeyPairSync, sign as signEd25519 } from 'node:crypto';
import { describe, expect, it } from 'vitest';
import {
  assertEconomicAuditCapsule,
  createEconomicAuditCapsule,
  economicAuditCapsuleDigest,
  type EconomicAuditCapsuleV1,
} from './audit.js';
import { money } from './money.js';
import { buildDeterministicNettingBatches } from './netting.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { reconstructSettlementRecovery, SettlementAttemptState } from './recovery.js';
import {
  createSettlementAllocationLineageCheckpoint,
  resolveCheckpointBackedSettlementAllocationLineage,
  signSettlementAllocationLineageCheckpoint,
  verifySettlementAllocationLineageCheckpoint,
} from './settlement-allocation-lineage-checkpoint.js';
import { createSettlementAllocationLineageCheckpointSignerTrustBundle } from './settlement-allocation-lineage-checkpoint-trust.js';
import {
  attachSettlementAllocationLineageTrustBundleDetachedSignature,
  createSettlementAllocationLineageTrustBundleAttestationRequest,
  settlementAllocationLineageTrustBundleAttestationPayloadBase64,
  verifySettlementAllocationLineageCheckpointWithAnchoredTrustBundle,
} from './settlement-allocation-lineage-checkpoint-trust-anchor.js';
import { createSettlementAllocationAuthority } from './settlement-allocation.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';
import { StatementKind } from './statements.js';

const checkpointKey = generateKeyPairSync('ed25519');
const rootKey = generateKeyPairSync('ed25519');
const checkpointPrivateKeyPem = checkpointKey.privateKey.export({ format: 'pem', type: 'pkcs8' }).toString();
const checkpointPublicKeyPem = checkpointKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const rootPublicKeyPem = rootKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();

const asOf = '2026-10-02T00:00:00Z';
const obligation = createRoyaltyObligationAuthority({
  id: 'obl:audit-v2',
  beneficiaryId: 'artist:audit-v2',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:audit-v2',
    rightsResolutionRef: 'rights:audit-v2',
    economicTermsRef: 'terms:audit-v2',
  },
});
const epoch: SettlementEpoch = {
  id: 'epoch:audit-v2',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const eligibility = [{
  id: 'elig:audit-v2',
  obligationId: obligation.id,
  code: SettlementEligibilityCode.Eligible,
  sourceRef: 'eligibility:audit-v2',
  observedAt: '2026-09-30T00:00:00Z',
}] as const;
const batch = buildDeterministicNettingBatches([obligation], epoch, eligibility)[0]!;
const recovery = reconstructSettlementRecovery(batch, [{
  attemptId: 'attempt:audit-v2',
  batchId: batch.batchId,
  obligationSetRoot: batch.obligationSetRoot,
  eligibilityAsOf: batch.eligibilityAsOf,
  eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot,
  state: SettlementAttemptState.Finalized,
  observedAt: '2026-10-01T00:01:00Z',
  railReceiptRef: 'rail:audit-v2',
  settledAmount: money(500n, 'USD'),
}]);
const allocation = createSettlementAllocationAuthority({
  allocationId: 'allocation:audit-v2',
  batch,
  recovery,
  deductions: [],
  residualHeld: money(0n, 'USD'),
  allocatedAt: '2026-10-01T00:02:00Z',
});
const unsignedCheckpoint = createSettlementAllocationLineageCheckpoint({
  checkpointId: 'checkpoint:audit-v2',
  sourceRef: 'postgres:audit-v2',
  sourceInstanceId: 'cluster:audit-v2',
  batchId: batch.batchId,
  asOf,
  observedThrough: asOf,
  highWaterMark: '7',
  allocations: [allocation],
  links: [],
});
const signedCheckpoint = signSettlementAllocationLineageCheckpoint(
  unsignedCheckpoint,
  'checkpoint-key:audit-v2',
  checkpointPrivateKeyPem,
);
const checkpointTrust = {
  sourceRef: unsignedCheckpoint.sourceRef,
  sourceInstanceId: unsignedCheckpoint.sourceInstanceId,
  signerKeyId: signedCheckpoint.signerKeyId,
  publicKeyPem: checkpointPublicKeyPem,
} as const;
const verifiedCheckpoint = verifySettlementAllocationLineageCheckpoint(signedCheckpoint, checkpointTrust);
const lineage = resolveCheckpointBackedSettlementAllocationLineage([allocation], [], verifiedCheckpoint);

function trustedStatementInput(policySequence = '1') {
  const trustBundle = createSettlementAllocationLineageCheckpointSignerTrustBundle({
    bundleId: `trust-bundle:audit-v2:${policySequence}`,
    policyAsOf: asOf,
    entries: [{
      entryId: 'checkpoint-signer:audit-v2',
      sourceRef: unsignedCheckpoint.sourceRef,
      sourceInstanceId: unsignedCheckpoint.sourceInstanceId,
      capabilityId: signedCheckpoint.signingRequest.capabilityId,
      signerKeyId: signedCheckpoint.signerKeyId,
      publicKeyPem: checkpointPublicKeyPem,
      validFrom: '2026-09-01T00:00:00Z',
      validUntil: '2027-01-01T00:00:00Z',
    }],
  });
  const request = createSettlementAllocationLineageTrustBundleAttestationRequest(trustBundle, {
    requestId: `trust-attestation:audit-v2:${policySequence}`,
    anchorId: 'root-anchor:audit-v2',
    signerKeyId: 'root-anchor-key:audit-v2',
    policySequence,
    ...(policySequence === '1' ? {} : { predecessorBundleRoot: 'a'.repeat(64) }),
    issuedAt: asOf,
    expiresAt: '2026-12-31T00:00:00Z',
    nonce: `nonce:audit-v2:${policySequence}`,
  });
  const signatureBase64 = signEd25519(
    null,
    Buffer.from(settlementAllocationLineageTrustBundleAttestationPayloadBase64(request, trustBundle, asOf), 'base64'),
    rootKey.privateKey,
  ).toString('base64');
  const attestation = attachSettlementAllocationLineageTrustBundleDetachedSignature(trustBundle, request, {
    signerKeyId: request.signerKeyId,
    signedAt: asOf,
    signatureBase64,
  });
  const anchoredTrust = verifySettlementAllocationLineageCheckpointWithAnchoredTrustBundle(
    signedCheckpoint,
    trustBundle,
    attestation,
    {
      anchorId: request.anchorId,
      signerKeyId: request.signerKeyId,
      publicKeyPem: rootPublicKeyPem,
      validFrom: '2026-09-01T00:00:00Z',
      validUntil: '2027-01-01T00:00:00Z',
      minimumPolicySequence: policySequence,
    },
    asOf,
  );
  return {
    statementId: 'statement:audit-v2',
    kind: StatementKind.Periodic,
    beneficiaryId: obligation.beneficiaryId,
    period: { startInclusive: '2026-09-01T00:00:00Z', endExclusive: '2026-10-01T00:00:00Z' },
    asOf,
    completeness: {
      kind: 'complete' as const,
      through: {
        usageObservedThrough: '2026-10-01T00:00:00Z',
        rightsResolvedThrough: '2026-10-01T00:00:00Z',
        settlementsObservedThrough: asOf,
      },
    },
    settlementEpoch: epoch,
    obligations: [obligation],
    eligibilityObservations: eligibility,
    settlements: [{ batch, recovery, allocationLineage: lineage, anchoredTrust }],
  };
}

function digest(seed: string): string {
  if (!/^[0-9a-f]$/.test(seed)) throw new Error('test digest seed must be one hex character');
  return seed.repeat(64);
}

function input(policySequence = '1') {
  return {
    statementInput: trustedStatementInput(policySequence),
    usageCommitment: digest('1'),
    repertoireSnapshot: digest('2'),
    rightsPolicy: digest('3'),
    economicTerms: digest('4'),
    rightsResolutionRoot: digest('5'),
    nettingRoot: digest('6'),
    settlementPlanRoot: digest('7'),
    paymentReceiptRoot: digest('8'),
    generatedBy: {
      id: 'mycelix-music-royalty-compiler',
      version: '2.0.0',
      buildDigest: digest('9'),
    },
  };
}

describe('EconomicAuditCapsule v2', () => {
  it('compiles the exact statement input and derives a complete per-batch anchored-trust set', () => {
    const capsule = createEconomicAuditCapsule(input());
    expect(capsule.protocolVersion).toBe(2);
    expect(capsule.statement.statementId).toBe('statement:audit-v2');
    expect(capsule.statementSnapshotRoot).toMatch(/^[0-9a-f]{64}$/);
    expect(capsule.anchoredSettlementTrust).toHaveLength(1);
    expect(capsule.anchoredSettlementTrust[0]!.batchId).toBe(batch.batchId);
    expect(capsule.anchoredSettlementTrust[0]!.trustPolicySequence).toBe('1');
    expect(capsule.anchoredSettlementTrust[0]!.verifiedAt).toBe('2026-10-02T00:00:00.000Z');
    expect(capsule.conservationProof.inputTotal.amountMinor).toBe(500n);
    expect(capsule.conservationProof.distributedTotal.amountMinor).toBe(500n);
    expect(capsule.conservationProof.residualTotal.amountMinor).toBe(0n);
  });

  it('is reproducible for identical authoritative statement and trust inputs', () => {
    const first = createEconomicAuditCapsule(input());
    const second = createEconomicAuditCapsule(input());
    expect(economicAuditCapsuleDigest(second)).toBe(economicAuditCapsuleDigest(first));
  });

  it('changes identity when root trust policy evidence changes', () => {
    const first = createEconomicAuditCapsule(input('1'));
    const second = createEconomicAuditCapsule(input('2'));
    expect(second.statement.obligationRoot).toBe(first.statement.obligationRoot);
    expect(second.statement.adjustmentRoot).toBe(first.statement.adjustmentRoot);
    expect(second.anchoredSettlementTrustRoot).not.toBe(first.anchoredSettlementTrustRoot);
    expect(second.statement.settlementRoot).not.toBe(first.statement.settlementRoot);
    expect(second.statementSnapshotRoot).not.toBe(first.statementSnapshotRoot);
    expect(economicAuditCapsuleDigest(second)).not.toBe(economicAuditCapsuleDigest(first));
  });

  it('detects omission or mutation inside the committed anchored-trust set', () => {
    const capsule = createEconomicAuditCapsule(input());
    expect(() => assertEconomicAuditCapsule({ ...capsule, anchoredSettlementTrust: [] }))
      .toThrow(/trust root does not match canonical evidence set/);
    expect(() => assertEconomicAuditCapsule({
      ...capsule,
      anchoredSettlementTrust: [{ ...capsule.anchoredSettlementTrust[0]!, trustPolicySequence: '02' }],
    })).toThrow(/canonical positive-integer text/);
  });

  it('binds the full statement snapshot and recomputes conservation from embedded economics', () => {
    const capsule = createEconomicAuditCapsule(input());
    expect(() => assertEconomicAuditCapsule({
      ...capsule,
      statement: { ...capsule.statement, beneficiaryId: 'artist:forged' },
    })).toThrow(/statementSnapshotRoot does not match embedded immutable statement/);
    expect(() => assertEconomicAuditCapsule({
      ...capsule,
      conservationProof: {
        ...capsule.conservationProof,
        distributedTotal: money(499n, 'USD'),
        residualTotal: money(1n, 'USD'),
      },
    })).toThrow(/does not match embedded statement economics/);
  });

  it('requires v2 external roots and compiler build identity to be canonical SHA-256 digests', () => {
    expect(() => createEconomicAuditCapsule({ ...input(), usageCommitment: 'usage-root' }))
      .toThrow(/lowercase SHA-256 digest/);
    expect(() => createEconomicAuditCapsule({
      ...input(),
      generatedBy: { ...input().generatedBy, buildDigest: 'compiler-build-root' },
    })).toThrow(/lowercase SHA-256 digest/);
  });

  it('keeps historical v1 capsules verifiable without allowing v1 creation through the v2 constructor', () => {
    const historical: EconomicAuditCapsuleV1 = {
      protocolVersion: 1,
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
      generatedBy: { id: 'legacy-compiler', version: '1.0.0', buildDigest: 'legacy-build-root' },
    };
    expect(() => assertEconomicAuditCapsule(historical)).not.toThrow();
    expect(economicAuditCapsuleDigest(historical)).toMatch(/^[0-9a-f]{64}$/);
  });
});
