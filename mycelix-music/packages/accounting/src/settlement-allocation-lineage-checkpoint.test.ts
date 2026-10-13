import { generateKeyPairSync } from 'node:crypto';
import { describe, expect, it } from 'vitest';
import { createRoyaltyDeductionAuthority } from './deduction-authority.js';
import { money } from './money.js';
import { buildDeterministicNettingBatches } from './netting.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { reconstructSettlementRecovery, SettlementAttemptState } from './recovery.js';
import {
  assertSettlementAllocationLineageCheckpointAdvances,
  createSettlementAllocationLineageCheckpoint,
  requireCheckpointBackedSettlementAllocationLineage,
  resolveCheckpointBackedSettlementAllocationLineage,
  signSettlementAllocationLineageCheckpoint,
  verifySettlementAllocationLineageCheckpoint,
} from './settlement-allocation-lineage-checkpoint.js';
import {
  createSettlementAllocationLineageBoundary,
  createSettlementAllocationSuccessorLink,
  resolveSettlementAllocationLineage,
} from './settlement-allocation-lineage.js';
import { createSettlementAllocationAuthority } from './settlement-allocation.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';

const sourceKeyPair = generateKeyPairSync('ed25519');
const otherKeyPair = generateKeyPairSync('ed25519');
const privateKeyPem = sourceKeyPair.privateKey.export({ format: 'pem', type: 'pkcs8' }).toString();
const publicKeyPem = sourceKeyPair.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const otherPublicKeyPem = otherKeyPair.publicKey.export({ format: 'pem', type: 'spki' }).toString();

const trust = {
  sourceRef: 'postgres:music-accounting',
  sourceInstanceId: 'cluster:prod:1',
  signerKeyId: 'checkpoint-key:2026-09',
  publicKeyPem,
} as const;

const obligation = createRoyaltyObligationAuthority({
  id: 'obl:checkpoint',
  beneficiaryId: 'artist:checkpoint',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:checkpoint',
    rightsResolutionRef: 'rights:checkpoint',
    economicTermsRef: 'terms:checkpoint',
  },
});
const epoch: SettlementEpoch = {
  id: 'epoch:checkpoint',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const eligibility = [{
  id: 'elig:checkpoint',
  obligationId: obligation.id,
  code: SettlementEligibilityCode.Eligible,
  sourceRef: 'eligibility:checkpoint',
  observedAt: '2026-09-30T00:00:00Z',
}] as const;
const batch = buildDeterministicNettingBatches([obligation], epoch, eligibility)[0]!;

function recovery(amountMinor: bigint, receipt: string) {
  return reconstructSettlementRecovery(batch, [{
    attemptId: `attempt:${receipt}`,
    batchId: batch.batchId,
    obligationSetRoot: batch.obligationSetRoot,
    eligibilityAsOf: batch.eligibilityAsOf,
    eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot,
    state: SettlementAttemptState.Finalized,
    observedAt: '2026-10-01T00:01:00Z',
    railReceiptRef: receipt,
    settledAmount: money(amountMinor, 'USD'),
  }]);
}

const deduction50 = createRoyaltyDeductionAuthority({
  id: 'deduction:50',
  beneficiaryId: batch.beneficiaryId,
  amount: money(50n, 'USD'),
  basis: 'tax:withholding:v1',
  authorityRef: 'tax-authority:50',
  observedAt: '2026-10-01T00:01:30Z',
});
const deduction25 = createRoyaltyDeductionAuthority({
  id: 'deduction:25',
  beneficiaryId: batch.beneficiaryId,
  amount: money(25n, 'USD'),
  basis: 'tax:withholding:v1',
  authorityRef: 'tax-authority:25',
  observedAt: '2026-10-01T00:02:30Z',
});

const initial = createSettlementAllocationAuthority({
  allocationId: 'allocation:checkpoint:initial',
  batch,
  recovery: recovery(425n, 'rail:receipt:425'),
  deductions: [deduction50],
  residualHeld: money(25n, 'USD'),
  residualAuthorityRef: 'residual:checkpoint:25',
  allocatedAt: '2026-10-01T00:02:00Z',
});
const successor = createSettlementAllocationAuthority({
  allocationId: 'allocation:checkpoint:successor',
  batch,
  recovery: recovery(425n, 'rail:receipt:425'),
  deductions: [deduction50, deduction25],
  residualHeld: money(0n, 'USD'),
  allocatedAt: '2026-10-01T00:03:00Z',
});
const link = createSettlementAllocationSuccessorLink({
  linkId: 'allocation-link:checkpoint',
  predecessor: initial,
  successor,
  supersessionEvidenceRef: 'reconciliation:checkpoint',
  linkedAt: '2026-10-01T00:04:00Z',
});

function unsignedCheckpoint(allocations = [initial, successor] as const, links = [link] as const) {
  return createSettlementAllocationLineageCheckpoint({
    checkpointId: 'checkpoint:allocation-lineage:1',
    sourceRef: trust.sourceRef,
    sourceInstanceId: trust.sourceInstanceId,
    batchId: batch.batchId,
    asOf: '2026-10-02T00:00:00Z',
    observedThrough: '2026-10-02T00:00:05Z',
    highWaterMark: '42',
    allocations,
    links,
  });
}

function signedCheckpoint() {
  return signSettlementAllocationLineageCheckpoint(
    unsignedCheckpoint(),
    trust.signerKeyId,
    privateKeyPem,
  );
}

function verifiedCheckpoint() {
  return verifySettlementAllocationLineageCheckpoint(signedCheckpoint(), trust);
}

function signedCheckpointState(options: {
  readonly checkpointId?: string;
  readonly asOf?: string;
  readonly observedThrough?: string;
  readonly highWaterMark?: string;
  readonly allocations?: readonly SettlementAllocationAuthority[];
  readonly links?: readonly SettlementAllocationSuccessorLink[];
} = {}) {
  const unsigned = createSettlementAllocationLineageCheckpoint({
    checkpointId: options.checkpointId ?? 'checkpoint:allocation-lineage:next',
    sourceRef: trust.sourceRef,
    sourceInstanceId: trust.sourceInstanceId,
    batchId: batch.batchId,
    asOf: options.asOf ?? '2026-10-03T00:00:00Z',
    observedThrough: options.observedThrough ?? '2026-10-03T00:00:05Z',
    highWaterMark: options.highWaterMark ?? '43',
    allocations: options.allocations ?? [initial, successor],
    links: options.links ?? [link],
  });
  return signSettlementAllocationLineageCheckpoint(unsigned, trust.signerKeyId, privateKeyPem);
}

describe('authenticated settlement allocation lineage checkpoint', () => {
  it('authenticates exact source snapshot and resolves its canonical successor head', () => {
    const verified = verifiedCheckpoint();
    const resolution = resolveCheckpointBackedSettlementAllocationLineage([successor, initial], [link], verified);
    const authority = requireCheckpointBackedSettlementAllocationLineage(resolution);
    expect(authority.headAllocation.allocationRoot).toBe(successor.allocationRoot);
    expect(authority.checkpoint.checkpointRoot).toBe(verified.checkpointRoot);
    expect(authority.checkpoint.highWaterMark).toBe('42');
  });

  it('rejects the omission attack even when the omitted set is locally a valid lineage', () => {
    const verified = verifiedCheckpoint();
    expect(() => resolveCheckpointBackedSettlementAllocationLineage([initial], [], verified))
      .toThrow(/checkpoint allocation roots do not match/);
  });

  it('rejects an uncheckpointed extra successor', () => {
    const initialOnly = createSettlementAllocationLineageCheckpoint({
      checkpointId: 'checkpoint:allocation-lineage:initial-only',
      sourceRef: trust.sourceRef,
      sourceInstanceId: trust.sourceInstanceId,
      batchId: batch.batchId,
      asOf: '2026-10-02T00:00:00Z',
      observedThrough: '2026-10-02T00:00:05Z',
      highWaterMark: '41',
      allocations: [initial],
      links: [],
    });
    const verified = verifySettlementAllocationLineageCheckpoint(
      signSettlementAllocationLineageCheckpoint(initialOnly, trust.signerKeyId, privateKeyPem),
      trust,
    );
    expect(() => resolveCheckpointBackedSettlementAllocationLineage([initial, successor], [link], verified))
      .toThrow(/checkpoint allocation roots do not match/);
  });

  it('rejects checkpoint content tampering behind an unchanged signature', () => {
    const signed = signedCheckpoint();
    expect(() => verifySettlementAllocationLineageCheckpoint({ ...signed, highWaterMark: '43' }, trust))
      .toThrow(/root does not match canonical contents/);
    expect(() => verifySettlementAllocationLineageCheckpoint({ ...signed, checkpointRoot: 'f'.repeat(64) }, trust))
      .toThrow(/root does not match canonical contents/);
  });

  it('rejects the wrong trusted signing key or source identity', () => {
    const signed = signedCheckpoint();
    expect(() => verifySettlementAllocationLineageCheckpoint(signed, { ...trust, publicKeyPem: otherPublicKeyPem }))
      .toThrow(/signature verification failed/);
    expect(() => verifySettlementAllocationLineageCheckpoint(signed, { ...trust, sourceInstanceId: 'cluster:other' }))
      .toThrow(/source identity is not trusted/);
  });

  it('requires canonical monotonic source high-water marks', () => {
    expect(() => createSettlementAllocationLineageCheckpoint({
      checkpointId: 'checkpoint:bad-cursor',
      sourceRef: trust.sourceRef,
      sourceInstanceId: trust.sourceInstanceId,
      batchId: batch.batchId,
      asOf: '2026-10-02T00:00:00Z',
      observedThrough: '2026-10-02T00:00:05Z',
      highWaterMark: '0042',
      allocations: [initial, successor],
      links: [link],
    })).toThrow(/canonical unsigned integer/);
  });

  it('accepts a signature-valid checkpoint that advances the trusted source cursor', () => {
    const previous = signedCheckpoint();
    const candidate = signedCheckpointState();
    const advanced = assertSettlementAllocationLineageCheckpointAdvances(previous, candidate, trust);
    expect(advanced.highWaterMark).toBe('43');
    expect(Date.parse(advanced.observedThrough)).toBeGreaterThan(Date.parse(previous.observedThrough));
  });

  it('rejects a signed checkpoint whose high-water mark rolls back', () => {
    const previous = signedCheckpoint();
    const candidate = signedCheckpointState({ highWaterMark: '41' });
    expect(() => assertSettlementAllocationLineageCheckpointAdvances(previous, candidate, trust))
      .toThrow(/high-water mark cannot move backwards/);
  });

  it('rejects a newer checkpoint that omits accepted allocation history', () => {
    const previous = signedCheckpoint();
    const candidate = signedCheckpointState({
      allocations: [initial],
      links: [],
    });
    expect(() => assertSettlementAllocationLineageCheckpointAdvances(previous, candidate, trust))
      .toThrow(/allocation roots cannot shrink after acceptance/);
  });

  it('rejects a cursor advance that changes the source batch identity', () => {
    const previous = signedCheckpoint();
    const otherBatch = 'batch:other';
    const alternateAllocation = {
      ...initial,
      batchId: otherBatch,
    };
    const unsigned = createSettlementAllocationLineageCheckpoint({
      checkpointId: 'checkpoint:other-batch',
      sourceRef: trust.sourceRef,
      sourceInstanceId: trust.sourceInstanceId,
      batchId: otherBatch,
      asOf: '2026-10-03T00:00:00Z',
      observedThrough: '2026-10-03T00:00:05Z',
      highWaterMark: '43',
      allocations: [alternateAllocation],
      links: [],
    });
    const candidate = signSettlementAllocationLineageCheckpoint(unsigned, trust.signerKeyId, privateKeyPem);
    expect(() => assertSettlementAllocationLineageCheckpointAdvances(previous, candidate, trust))
      .toThrow(/source and batch identity cannot change/);
  });

  it('rejects a signed checkpoint whose as-of time rolls back even when its cursor grows', () => {
    const previous = signedCheckpoint();
    const candidate = signedCheckpointState({
      asOf: '2026-10-01T00:00:00Z',
      observedThrough: '2026-10-02T00:00:05Z',
      highWaterMark: '43',
    });
    expect(() => assertSettlementAllocationLineageCheckpointAdvances(previous, candidate, trust))
      .toThrow(/asOf cannot move backwards/);
  });

  it('rejects checkpoint identity reuse even when the new signature and cursor are valid', () => {
    const previous = signedCheckpoint();
    const candidate = signedCheckpointState({
      checkpointId: previous.checkpointId,
      asOf: '2026-10-03T00:00:00Z',
      observedThrough: '2026-10-03T00:00:05Z',
      highWaterMark: '43',
    });
    expect(() => assertSettlementAllocationLineageCheckpointAdvances(previous, candidate, trust))
      .toThrow(/checkpointId cannot be reused/);
  });

  it('rejects checkpoint-id changes that do not advance any source cursor', () => {
    const previous = signedCheckpoint();
    const replayWithNewId = signedCheckpointState({
      checkpointId: 'checkpoint:allocation-lineage:replay',
      asOf: previous.asOf,
      observedThrough: previous.observedThrough,
      highWaterMark: previous.highWaterMark,
    });
    expect(() => assertSettlementAllocationLineageCheckpointAdvances(previous, replayWithNewId, trust))
      .toThrow(/does not advance the accepted source cursor/);
  });

  it('permits an idle-source checkpoint only when observation coverage advances', () => {
    const previous = signedCheckpoint();
    const idle = signedCheckpointState({
      checkpointId: 'checkpoint:allocation-lineage:idle',
      asOf: previous.asOf,
      observedThrough: '2026-10-02T00:00:06Z',
      highWaterMark: previous.highWaterMark,
    });
    const advanced = assertSettlementAllocationLineageCheckpointAdvances(previous, idle, trust);
    expect(advanced.highWaterMark).toBe(previous.highWaterMark);
    expect(advanced.observedThrough).toBe('2026-10-02T00:00:06.000Z');
  });

  it('does not let a spread clone of a verified checkpoint retain runtime trust', () => {
    const verified = verifiedCheckpoint();
    expect(() => resolveCheckpointBackedSettlementAllocationLineage([initial, successor], [link], { ...verified }))
      .toThrow(/cryptographically verified/);
  });

  it('allows serialized signed evidence only after cryptographic reverification', () => {
    const transported = JSON.parse(JSON.stringify(signedCheckpoint())) as ReturnType<typeof signedCheckpoint>;
    expect(() => resolveCheckpointBackedSettlementAllocationLineage([initial, successor], [link], transported))
      .toThrow(/cryptographically verified/);
    const reverified = verifySettlementAllocationLineageCheckpoint(transported, trust);
    expect(requireCheckpointBackedSettlementAllocationLineage(
      resolveCheckpointBackedSettlementAllocationLineage([initial, successor], [link], reverified),
    ).headAllocation.allocationRoot).toBe(successor.allocationRoot);
  });

  it('does not promote a direct complete-boundary resolution to checkpoint-backed authority', () => {
    const direct = resolveSettlementAllocationLineage(
      [initial, successor],
      [link],
      createSettlementAllocationLineageBoundary({
        asOf: '2026-10-02T00:00:00Z',
        observedThrough: '2026-10-02T00:00:05Z',
        coverage: 'complete',
        sourceRef: 'caller-asserted-complete',
      }),
    );
    expect(() => requireCheckpointBackedSettlementAllocationLineage(direct))
      .toThrow(/verified source checkpoint/);
  });
});
