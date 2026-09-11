import { generateKeyPairSync, sign as signEd25519 } from 'node:crypto';
import { describe, expect, it } from 'vitest';
import { createRoyaltyDeductionAuthority } from './deduction-authority.js';
import { money } from './money.js';
import { buildDeterministicNettingBatches } from './netting.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { compileRoyaltyStatement } from './projection.js';
import { reconstructSettlementRecovery, SettlementAttemptState } from './recovery.js';
import {
  createSettlementAllocationLineageCheckpoint,
  resolveCheckpointBackedSettlementAllocationLineage,
  signSettlementAllocationLineageCheckpoint,
  verifySettlementAllocationLineageCheckpoint,
} from './settlement-allocation-lineage-checkpoint.js';
import {
  createSettlementAllocationLineageCheckpointSignerTrustBundle,
} from './settlement-allocation-lineage-checkpoint-trust.js';
import {
  attachSettlementAllocationLineageTrustBundleDetachedSignature,
  createSettlementAllocationLineageTrustBundleAttestationRequest,
  settlementAllocationLineageTrustBundleAttestationPayloadBase64,
  verifySettlementAllocationLineageCheckpointWithAnchoredTrustBundle,
  type AnchoredSettlementAllocationLineageCheckpointAuthority,
} from './settlement-allocation-lineage-checkpoint-trust-anchor.js';
import {
  createSettlementAllocationLineageBoundary,
  createSettlementAllocationSuccessorLink,
  resolveSettlementAllocationLineage,
  type SettlementAllocationLineageResolution,
  type SettlementAllocationSuccessorLink,
} from './settlement-allocation-lineage.js';
import { createSettlementAllocationAuthority, type SettlementAllocationAuthority } from './settlement-allocation.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';
import { StatementKind } from './statements.js';

const keyPair = generateKeyPairSync('ed25519');
const rootKeyPair = generateKeyPairSync('ed25519');
const privateKeyPem = keyPair.privateKey.export({ format: 'pem', type: 'pkcs8' }).toString();
const publicKeyPem = keyPair.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const rootPublicKeyPem = rootKeyPair.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const checkpointTrust = {
  sourceRef: 'postgres:statement-lineage',
  sourceInstanceId: 'cluster:test:statement-lineage',
  signerKeyId: 'checkpoint-key:test',
  publicKeyPem,
} as const;

const obligation = createRoyaltyObligationAuthority({
  id: 'obl:statement-lineage',
  beneficiaryId: 'artist:statement-lineage',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:statement-lineage',
    rightsResolutionRef: 'rights:statement-lineage',
    economicTermsRef: 'terms:statement-lineage',
  },
});
const epoch: SettlementEpoch = {
  id: 'epoch:statement-lineage',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const eligibility = [{
  id: 'elig:statement-lineage',
  obligationId: obligation.id,
  code: SettlementEligibilityCode.Eligible,
  sourceRef: 'eligibility:statement-lineage',
  observedAt: '2026-09-30T00:00:00Z',
}] as const;
const batch = buildDeterministicNettingBatches([obligation], epoch, eligibility)[0]!;
const recovery = reconstructSettlementRecovery(batch, [{
  attemptId: 'attempt:statement-lineage',
  batchId: batch.batchId,
  obligationSetRoot: batch.obligationSetRoot,
  eligibilityAsOf: batch.eligibilityAsOf,
  eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot,
  state: SettlementAttemptState.Finalized,
  observedAt: '2026-10-01T00:01:00Z',
  railReceiptRef: 'rail:receipt:statement-lineage',
  settledAmount: money(425n, 'USD'),
}]);
const tax50 = createRoyaltyDeductionAuthority({
  id: 'deduction:tax-50', beneficiaryId: batch.beneficiaryId, amount: money(50n, 'USD'),
  basis: 'tax:withholding:v1', authorityRef: 'tax:notice:50', observedAt: '2026-10-01T00:01:30Z',
});
const tax25 = createRoyaltyDeductionAuthority({
  id: 'deduction:tax-25', beneficiaryId: batch.beneficiaryId, amount: money(25n, 'USD'),
  basis: 'tax:withholding:supplement:v1', authorityRef: 'tax:notice:25', observedAt: '2026-10-01T00:02:30Z',
});
const initial = createSettlementAllocationAuthority({
  allocationId: 'allocation:statement-lineage:initial',
  batch,
  recovery,
  deductions: [tax50],
  residualHeld: money(25n, 'USD'),
  residualAuthorityRef: 'residual:statement-lineage',
  allocatedAt: '2026-10-01T00:02:00Z',
});
const successor = createSettlementAllocationAuthority({
  allocationId: 'allocation:statement-lineage:successor',
  batch,
  recovery,
  deductions: [tax50, tax25],
  residualHeld: money(0n, 'USD'),
  allocatedAt: '2026-10-01T00:03:00Z',
});
const link = createSettlementAllocationSuccessorLink({
  linkId: 'allocation-link:statement-lineage',
  predecessor: initial,
  successor,
  supersessionEvidenceRef: 'reconciliation:statement-lineage',
  linkedAt: '2026-10-01T00:03:30Z',
});
const asOf = '2026-10-02T00:00:00Z';

type TrustedLineage = Readonly<{
  lineage: SettlementAllocationLineageResolution;
  anchoredTrust: AnchoredSettlementAllocationLineageCheckpointAuthority;
}>;

function checkpointBackedLineage(
  allocations: readonly SettlementAllocationAuthority[],
  links: readonly SettlementAllocationSuccessorLink[],
  options: {
    readonly asOf?: string;
    readonly checkpointId?: string;
    readonly highWaterMark?: string;
    readonly trustPolicySequence?: string;
    readonly predecessorBundleRoot?: string;
  } = {},
): TrustedLineage {
  const checkpointAsOf = options.asOf ?? asOf;
  const unsigned = createSettlementAllocationLineageCheckpoint({
    checkpointId: options.checkpointId ?? 'checkpoint:statement-lineage',
    sourceRef: checkpointTrust.sourceRef,
    sourceInstanceId: checkpointTrust.sourceInstanceId,
    batchId: batch.batchId,
    asOf: checkpointAsOf,
    observedThrough: checkpointAsOf,
    highWaterMark: options.highWaterMark ?? '100',
    allocations,
    links,
  });
  const signed = signSettlementAllocationLineageCheckpoint(unsigned, checkpointTrust.signerKeyId, privateKeyPem);
  const verified = verifySettlementAllocationLineageCheckpoint(signed, checkpointTrust);
  const lineage = resolveCheckpointBackedSettlementAllocationLineage(allocations, links, verified);

  const trustBundle = createSettlementAllocationLineageCheckpointSignerTrustBundle({
    bundleId: `trust-bundle:${options.checkpointId ?? 'statement-lineage'}:${options.trustPolicySequence ?? '1'}`,
    policyAsOf: checkpointAsOf,
    entries: [{
      entryId: 'checkpoint-signer:statement-lineage',
      sourceRef: checkpointTrust.sourceRef,
      sourceInstanceId: checkpointTrust.sourceInstanceId,
      capabilityId: signed.signingRequest.capabilityId,
      signerKeyId: signed.signerKeyId,
      publicKeyPem,
      validFrom: '2026-09-01T00:00:00Z',
      validUntil: '2027-01-01T00:00:00Z',
    }],
  });
  const policySequence = options.trustPolicySequence ?? '1';
  const request = createSettlementAllocationLineageTrustBundleAttestationRequest(trustBundle, {
    requestId: `trust-attestation:${unsigned.checkpointId}:${policySequence}`,
    anchorId: 'root-anchor:statement-lineage',
    signerKeyId: 'root-anchor-key:statement-lineage',
    policySequence,
    ...(options.predecessorBundleRoot === undefined ? {} : { predecessorBundleRoot: options.predecessorBundleRoot }),
    issuedAt: checkpointAsOf,
    expiresAt: '2026-12-31T00:00:00Z',
    nonce: `nonce:${unsigned.checkpointId}:${policySequence}`,
  });
  const signatureBase64 = signEd25519(
    null,
    Buffer.from(
      settlementAllocationLineageTrustBundleAttestationPayloadBase64(request, trustBundle, checkpointAsOf),
      'base64',
    ),
    rootKeyPair.privateKey,
  ).toString('base64');
  const attestation = attachSettlementAllocationLineageTrustBundleDetachedSignature(trustBundle, request, {
    signerKeyId: request.signerKeyId,
    signedAt: checkpointAsOf,
    signatureBase64,
  });
  const anchoredTrust = verifySettlementAllocationLineageCheckpointWithAnchoredTrustBundle(
    signed,
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
    checkpointAsOf,
  );
  return Object.freeze({ lineage, anchoredTrust });
}

const trustedLineage = checkpointBackedLineage([successor, initial], [link]);

function compile(trusted = trustedLineage) {
  return compileRoyaltyStatement({
    statementId: 'statement:statement-lineage',
    kind: StatementKind.Periodic,
    beneficiaryId: batch.beneficiaryId,
    period: { startInclusive: '2026-09-01T00:00:00Z', endExclusive: '2026-10-01T00:00:00Z' },
    asOf,
    completeness: {
      kind: 'complete',
      through: {
        usageObservedThrough: '2026-10-01T00:00:00Z',
        rightsResolvedThrough: '2026-10-01T00:00:00Z',
        settlementsObservedThrough: asOf,
      },
    },
    settlementEpoch: epoch,
    obligations: [obligation],
    eligibilityObservations: eligibility,
    deductions: [tax50, tax25],
    settlements: [{ batch, recovery, allocationLineage: trusted.lineage, anchoredTrust: trusted.anchoredTrust }],
  });
}

describe('statement allocation-lineage authority', () => {
  it('uses the root-anchored successor head and commits the lineage history', () => {
    expect(trustedLineage.lineage.headAllocationRoot).toBe(successor.allocationRoot);
    expect(trustedLineage.lineage.headAllocation.obligationSetDischarged).toBe(true);
    const statement = compile();
    expect(statement.paid.amountMinor).toBe(425n);
    expect(statement.deductions.amountMinor).toBe(75n);
    expect(statement.netPayable.amountMinor).toBe(425n);
  });

  it('changes settlement commitment when authenticated observed allocation history changes', () => {
    const predecessorOnly = checkpointBackedLineage([initial], [], {
      checkpointId: 'checkpoint:statement-lineage:predecessor-only',
      highWaterMark: '99',
    });
    const predecessorStatement = compile(predecessorOnly);
    const successorStatement = compile(trustedLineage);
    expect(successorStatement.obligationRoot).toBe(predecessorStatement.obligationRoot);
    expect(successorStatement.adjustmentRoot).toBe(predecessorStatement.adjustmentRoot);
    expect(successorStatement.settlementRoot).not.toBe(predecessorStatement.settlementRoot);
  });

  it('changes settlement commitment when root trust-policy evidence changes', () => {
    const policy2 = checkpointBackedLineage([successor, initial], [link], {
      checkpointId: 'checkpoint:statement-lineage:policy-2',
      highWaterMark: '101',
      trustPolicySequence: '2',
      predecessorBundleRoot: trustedLineage.anchoredTrust.receipt.trustBundleRoot,
    });
    const first = compile(trustedLineage);
    const second = compile(policy2);
    expect(second.obligationRoot).toBe(first.obligationRoot);
    expect(second.adjustmentRoot).toBe(first.adjustmentRoot);
    expect(second.settlementRoot).not.toBe(first.settlementRoot);
  });

  it('rejects checkpoint-backed lineage without root-anchored trust authority', () => {
    expect(() => compileRoyaltyStatement({
      statementId: 'statement:unanchored',
      kind: StatementKind.Periodic,
      beneficiaryId: batch.beneficiaryId,
      period: { startInclusive: '2026-09-01T00:00:00Z', endExclusive: '2026-10-01T00:00:00Z' },
      asOf,
      completeness: { kind: 'complete', through: { usageObservedThrough: asOf, rightsResolvedThrough: asOf, settlementsObservedThrough: asOf } },
      settlementEpoch: epoch,
      obligations: [obligation],
      eligibilityObservations: eligibility,
      deductions: [tax50, tax25],
      settlements: [{ batch, recovery, allocationLineage: trustedLineage.lineage }],
    })).toThrow(/requires anchored trust authority/);
  });

  it('rejects a checkpoint-backed lineage resolved for a different statement instant', () => {
    const wrong = checkpointBackedLineage([initial, successor], [link], {
      asOf: '2026-10-01T23:59:59Z',
      checkpointId: 'checkpoint:statement-lineage:wrong-asof',
      highWaterMark: '98',
    });
    expect(() => compile(wrong)).toThrow(/boundary asOf must equal statement asOf/);
  });

  it('rejects a structural clone of a genuine checkpoint-backed lineage resolution', () => {
    expect(() => compile({ ...trustedLineage, lineage: { ...trustedLineage.lineage } })).toThrow(/verified source checkpoint/);
  });

  it('rejects a structural clone of a genuine anchored trust authority', () => {
    expect(() => compile({ ...trustedLineage, anchoredTrust: { ...trustedLineage.anchoredTrust } }))
      .toThrow(/requires verified anchored trust authority/);
  });

  it('rejects a direct caller-asserted complete lineage even when structurally valid', () => {
    const directComplete = resolveSettlementAllocationLineage(
      [initial, successor],
      [link],
      createSettlementAllocationLineageBoundary({
        asOf,
        observedThrough: asOf,
        coverage: 'complete',
        sourceRef: 'caller-asserted-complete',
      }),
    );
    expect(() => compile({ ...trustedLineage, lineage: directComplete })).toThrow(/verified source checkpoint/);
  });
});
