import { generateKeyPairSync, sign as signEd25519 } from 'node:crypto';
import { describe, expect, it } from 'vitest';
import { money } from './money.js';
import { buildDeterministicNettingBatches } from './netting.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { reconstructSettlementRecovery, SettlementAttemptState } from './recovery.js';
import {
  SETTLEMENT_ALLOCATION_LINEAGE_CHECKPOINT_SIGNING_SCOPE,
  createSettlementAllocationLineageCheckpoint,
  createSettlementAllocationLineageCheckpointSigningRequest,
  settlementAllocationLineageCheckpointSigningPayloadBase64,
  type SignedSettlementAllocationLineageCheckpoint,
} from './settlement-allocation-lineage-checkpoint.js';
import {
  createSettlementAllocationLineageCheckpointSignerTrustBundle,
  verifySettlementAllocationLineageCheckpointWithTrustBundle,
} from './settlement-allocation-lineage-checkpoint-trust.js';
import { createSettlementAllocationAuthority } from './settlement-allocation.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';

const obligation = createRoyaltyObligationAuthority({
  id: 'obl:trust-rotation',
  beneficiaryId: 'creator:trust-rotation',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:trust-rotation',
    rightsResolutionRef: 'rights:trust-rotation',
    economicTermsRef: 'terms:trust-rotation',
  },
});
const epoch: SettlementEpoch = {
  id: 'epoch:trust-rotation',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const eligibility = [{
  id: 'elig:trust-rotation',
  obligationId: obligation.id,
  code: SettlementEligibilityCode.Eligible,
  sourceRef: 'eligibility:trust-rotation',
  observedAt: '2026-09-30T00:00:00Z',
}] as const;
const batch = buildDeterministicNettingBatches([obligation], epoch, eligibility)[0]!;
const recovery = reconstructSettlementRecovery(batch, [{
  attemptId: 'attempt:trust-rotation',
  batchId: batch.batchId,
  obligationSetRoot: batch.obligationSetRoot,
  eligibilityAsOf: batch.eligibilityAsOf,
  eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot,
  state: SettlementAttemptState.Finalized,
  observedAt: '2026-10-01T00:01:00Z',
  railReceiptRef: 'rail:trust-rotation',
  settledAmount: money(500n, 'USD'),
}]);
const allocation = createSettlementAllocationAuthority({
  allocationId: 'allocation:trust-rotation',
  batch,
  recovery,
  deductions: [],
  residualHeld: money(0n, 'USD'),
  allocatedAt: '2026-10-01T00:02:00Z',
});
const checkpoint = createSettlementAllocationLineageCheckpoint({
  checkpointId: 'checkpoint:trust-rotation',
  sourceRef: 'postgres:creator-accounting:allocation-lineage',
  sourceInstanceId: 'cluster:trust-rotation',
  batchId: batch.batchId,
  asOf: '2026-10-02T00:00:00Z',
  observedThrough: '2026-10-02T00:00:01Z',
  highWaterMark: '12',
  allocations: [allocation],
  links: [],
});

const oldKey = generateKeyPairSync('ed25519');
const newKey = generateKeyPairSync('ed25519');
const oldPublicKeyPem = oldKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const newPublicKeyPem = newKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();

function signedCheckpoint(input: {
  readonly signer: typeof oldKey;
  readonly signerKeyId: string;
  readonly capabilityId: string;
  readonly signedAt: string;
  readonly capabilityValidFrom?: string;
  readonly capabilityValidUntil?: string;
}): SignedSettlementAllocationLineageCheckpoint {
  const validFrom = input.capabilityValidFrom ?? '2026-10-01T00:00:00Z';
  const validUntil = input.capabilityValidUntil ?? '2026-10-10T00:00:00Z';
  const issuedAt = new Date(Date.parse(input.signedAt) - 1_000).toISOString();
  const expiresAt = new Date(Date.parse(input.signedAt) + 60_000).toISOString();
  const request = createSettlementAllocationLineageCheckpointSigningRequest(
    checkpoint,
    {
      capabilityId: input.capabilityId,
      scope: SETTLEMENT_ALLOCATION_LINEAGE_CHECKPOINT_SIGNING_SCOPE,
      sourceRef: checkpoint.sourceRef,
      sourceInstanceId: checkpoint.sourceInstanceId,
      signerKeyId: input.signerKeyId,
      validFrom,
      validUntil,
    },
    {
      requestId: `request:${input.signerKeyId}:${input.signedAt}`,
      issuedAt,
      expiresAt,
      nonce: `nonce:${input.signerKeyId}:${input.signedAt}`,
    },
  );
  const payload = Buffer.from(
    settlementAllocationLineageCheckpointSigningPayloadBase64(request, input.signedAt),
    'base64',
  );
  return Object.freeze({
    ...checkpoint,
    signerKeyId: input.signerKeyId,
    signatureBase64: signEd25519(null, payload, input.signer.privateKey).toString('base64'),
    signedAt: input.signedAt,
    signingRequest: request,
  });
}

function trustEntry(input: {
  readonly entryId: string;
  readonly signerKeyId: string;
  readonly capabilityId: string;
  readonly publicKeyPem: string;
  readonly validFrom: string;
  readonly validUntil: string;
  readonly revokedAt?: string;
}) {
  return {
    entryId: input.entryId,
    sourceRef: checkpoint.sourceRef,
    sourceInstanceId: checkpoint.sourceInstanceId,
    capabilityId: input.capabilityId,
    signerKeyId: input.signerKeyId,
    publicKeyPem: input.publicKeyPem,
    validFrom: input.validFrom,
    validUntil: input.validUntil,
    ...(input.revokedAt === undefined ? {} : { revokedAt: input.revokedAt }),
  };
}

const oldEntry = trustEntry({
  entryId: 'trust:old',
  signerKeyId: 'key:old',
  capabilityId: 'capability:old',
  publicKeyPem: oldPublicKeyPem,
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2026-10-05T00:00:00Z',
});
const newEntry = trustEntry({
  entryId: 'trust:new',
  signerKeyId: 'key:new',
  capabilityId: 'capability:new',
  publicKeyPem: newPublicKeyPem,
  validFrom: '2026-10-05T00:00:00Z',
  validUntil: '2026-11-01T00:00:00Z',
});

function rotationBundle(policyAsOf = '2026-10-06T00:00:00Z') {
  return createSettlementAllocationLineageCheckpointSignerTrustBundle({
    bundleId: 'trust-bundle:rotation-v1',
    policyAsOf,
    entries: [newEntry, oldEntry],
  });
}

describe('settlement allocation checkpoint signer trust rotation', () => {
  it('preserves historical old-key verification after rotation and emits an auditable receipt', () => {
    const signed = signedCheckpoint({
      signer: oldKey,
      signerKeyId: 'key:old',
      capabilityId: 'capability:old',
      signedAt: '2026-10-04T12:00:00Z',
      capabilityValidUntil: '2026-10-05T00:00:00Z',
    });
    const verified = verifySettlementAllocationLineageCheckpointWithTrustBundle(
      signed,
      rotationBundle(),
      '2026-10-06T00:01:00Z',
    );
    expect(verified.checkpoint.checkpointRoot).toBe(checkpoint.checkpointRoot);
    expect(verified.receipt.trustEntryId).toBe('trust:old');
    expect(verified.receipt.trustBundleRoot).toBe(rotationBundle().bundleRoot);
    expect(verified.receipt.signingRequestRoot).toBe(signed.signingRequest.requestRoot);
    expect(verified.receipt.receiptRoot).toMatch(/^[0-9a-f]{64}$/);
  });

  it('accepts the rotated key after its authority window begins', () => {
    const signed = signedCheckpoint({
      signer: newKey,
      signerKeyId: 'key:new',
      capabilityId: 'capability:new',
      signedAt: '2026-10-05T12:00:00Z',
      capabilityValidFrom: '2026-10-05T00:00:00Z',
    });
    const verified = verifySettlementAllocationLineageCheckpointWithTrustBundle(
      signed,
      rotationBundle(),
      '2026-10-06T00:01:00Z',
    );
    expect(verified.receipt.trustEntryId).toBe('trust:new');
  });

  it('rejects an old-key signature after the trust window expires', () => {
    const signed = signedCheckpoint({
      signer: oldKey,
      signerKeyId: 'key:old',
      capabilityId: 'capability:old',
      signedAt: '2026-10-06T00:00:00Z',
    });
    expect(() => verifySettlementAllocationLineageCheckpointWithTrustBundle(
      signed,
      rotationBundle(),
      '2026-10-06T00:01:00Z',
    )).toThrow(/no authorized signer trust entry/);
  });

  it('treats revocation prospectively: pre-cutoff signatures survive, post-cutoff signatures fail', () => {
    const revokedEntry = trustEntry({
      entryId: 'trust:revoked',
      signerKeyId: 'key:old',
      capabilityId: 'capability:old',
      publicKeyPem: oldPublicKeyPem,
      validFrom: '2026-10-01T00:00:00Z',
      validUntil: '2026-10-10T00:00:00Z',
      revokedAt: '2026-10-04T18:00:00Z',
    });
    const bundle = createSettlementAllocationLineageCheckpointSignerTrustBundle({
      bundleId: 'trust-bundle:revocation',
      policyAsOf: '2026-10-05T00:00:00Z',
      entries: [revokedEntry],
    });
    const before = signedCheckpoint({
      signer: oldKey,
      signerKeyId: 'key:old',
      capabilityId: 'capability:old',
      signedAt: '2026-10-04T17:59:00Z',
    });
    const after = signedCheckpoint({
      signer: oldKey,
      signerKeyId: 'key:old',
      capabilityId: 'capability:old',
      signedAt: '2026-10-04T18:00:00Z',
    });
    expect(verifySettlementAllocationLineageCheckpointWithTrustBundle(
      before,
      bundle,
      '2026-10-05T00:01:00Z',
    ).receipt.trustEntryId).toBe('trust:revoked');
    expect(() => verifySettlementAllocationLineageCheckpointWithTrustBundle(
      after,
      bundle,
      '2026-10-05T00:01:00Z',
    )).toThrow(/no authorized signer trust entry/);
  });

  it('rejects stale trust policy snapshots that do not reach checkpoint signedAt', () => {
    const signed = signedCheckpoint({
      signer: oldKey,
      signerKeyId: 'key:old',
      capabilityId: 'capability:old',
      signedAt: '2026-10-04T12:00:00Z',
    });
    expect(() => verifySettlementAllocationLineageCheckpointWithTrustBundle(
      signed,
      rotationBundle('2026-10-04T11:59:59Z'),
      '2026-10-06T00:01:00Z',
    )).toThrow(/must be observed through checkpoint signedAt/);
  });

  it('rejects overlapping authority windows for the same signer capability identity', () => {
    expect(() => createSettlementAllocationLineageCheckpointSignerTrustBundle({
      bundleId: 'trust-bundle:overlap',
      policyAsOf: '2026-10-06T00:00:00Z',
      entries: [
        oldEntry,
        trustEntry({
          entryId: 'trust:old:overlap',
          signerKeyId: 'key:old',
          capabilityId: 'capability:old',
          publicKeyPem: oldPublicKeyPem,
          validFrom: '2026-10-04T00:00:00Z',
          validUntil: '2026-10-07T00:00:00Z',
        }),
      ],
    })).toThrow(/overlapping authority windows/);
  });

  it('rejects trust-bundle tampering and binds receipt identity to the exact bundle root', () => {
    const signed = signedCheckpoint({
      signer: oldKey,
      signerKeyId: 'key:old',
      capabilityId: 'capability:old',
      signedAt: '2026-10-04T12:00:00Z',
    });
    const bundle = rotationBundle();
    expect(() => verifySettlementAllocationLineageCheckpointWithTrustBundle(
      signed,
      { ...bundle, bundleRoot: 'f'.repeat(64) },
      '2026-10-06T00:01:00Z',
    )).toThrow(/does not match canonical reconstruction/);

    const alternate = createSettlementAllocationLineageCheckpointSignerTrustBundle({
      bundleId: 'trust-bundle:rotation-v2',
      policyAsOf: '2026-10-06T00:00:00Z',
      entries: [newEntry, oldEntry],
    });
    const firstReceipt = verifySettlementAllocationLineageCheckpointWithTrustBundle(
      signed,
      bundle,
      '2026-10-06T00:01:00Z',
    ).receipt;
    const secondReceipt = verifySettlementAllocationLineageCheckpointWithTrustBundle(
      signed,
      alternate,
      '2026-10-06T00:01:00Z',
    ).receipt;
    expect(firstReceipt.trustBundleRoot).not.toBe(secondReceipt.trustBundleRoot);
    expect(firstReceipt.receiptRoot).not.toBe(secondReceipt.receiptRoot);
  });
});
