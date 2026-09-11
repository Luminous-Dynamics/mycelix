import { generateKeyPairSync, sign as signEd25519 } from 'node:crypto';
import { describe, expect, it } from 'vitest';
import { createRoyaltyDeductionAuthority } from './deduction-authority.js';
import { money } from './money.js';
import { buildDeterministicNettingBatches } from './netting.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { projectSettlementAllocationRecord } from './persistence-projection.js';
import { reconstructSettlementRecovery, SettlementAttemptState } from './recovery.js';
import {
  compileSettlementAllocationLineageCheckpoint,
  createCompiledSettlementAllocationLineageCheckpointSigningRequest,
  type PersistedSettlementAllocationLineageSourceSnapshot,
  type SettlementAllocationReplayContext,
} from './settlement-allocation-lineage-checkpoint-compiler.js';
import {
  SETTLEMENT_ALLOCATION_LINEAGE_CHECKPOINT_SIGNING_SCOPE,
  attachSettlementAllocationLineageCheckpointDetachedSignature,
  settlementAllocationLineageCheckpointSigningPayloadBase64,
  verifySettlementAllocationLineageCheckpoint,
  type SettlementAllocationLineageCheckpointSigningRequest,
} from './settlement-allocation-lineage-checkpoint.js';
import {
  createSettlementAllocationSuccessorLink,
} from './settlement-allocation-lineage.js';
import {
  projectSettlementAllocationSuccessorLinkRecord,
} from './settlement-allocation-lineage-persistence.js';
import { createSettlementAllocationAuthority } from './settlement-allocation.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';

const obligation = createRoyaltyObligationAuthority({
  id: 'obl:checkpoint-compiler',
  beneficiaryId: 'creator:checkpoint-compiler',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:checkpoint-compiler',
    rightsResolutionRef: 'rights:checkpoint-compiler',
    economicTermsRef: 'terms:checkpoint-compiler',
  },
});
const epoch: SettlementEpoch = {
  id: 'epoch:checkpoint-compiler',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const eligibility = [{
  id: 'elig:checkpoint-compiler',
  obligationId: obligation.id,
  code: SettlementEligibilityCode.Eligible,
  sourceRef: 'eligibility:checkpoint-compiler',
  observedAt: '2026-09-30T00:00:00Z',
}] as const;
const batch = buildDeterministicNettingBatches([obligation], epoch, eligibility)[0]!;
const recovery = reconstructSettlementRecovery(batch, [{
  attemptId: 'attempt:checkpoint-compiler',
  batchId: batch.batchId,
  obligationSetRoot: batch.obligationSetRoot,
  eligibilityAsOf: batch.eligibilityAsOf,
  eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot,
  state: SettlementAttemptState.Finalized,
  observedAt: '2026-10-01T00:01:00Z',
  railReceiptRef: 'rail:receipt:checkpoint-compiler',
  settledAmount: money(425n, 'USD'),
}]);
const deduction50 = createRoyaltyDeductionAuthority({
  id: 'deduction:checkpoint-compiler:50',
  beneficiaryId: batch.beneficiaryId,
  amount: money(50n, 'USD'),
  basis: 'tax:withholding:v1',
  authorityRef: 'tax:checkpoint-compiler:50',
  observedAt: '2026-10-01T00:01:30Z',
});
const deduction25 = createRoyaltyDeductionAuthority({
  id: 'deduction:checkpoint-compiler:25',
  beneficiaryId: batch.beneficiaryId,
  amount: money(25n, 'USD'),
  basis: 'tax:withholding:supplement:v1',
  authorityRef: 'tax:checkpoint-compiler:25',
  observedAt: '2026-10-01T00:02:30Z',
});
const initial = createSettlementAllocationAuthority({
  allocationId: 'allocation:checkpoint-compiler:initial',
  batch,
  recovery,
  deductions: [deduction50],
  residualHeld: money(25n, 'USD'),
  residualAuthorityRef: 'reconciliation:checkpoint-compiler:25',
  allocatedAt: '2026-10-01T00:02:00Z',
});
const successor = createSettlementAllocationAuthority({
  allocationId: 'allocation:checkpoint-compiler:successor',
  batch,
  recovery,
  deductions: [deduction50, deduction25],
  residualHeld: money(0n, 'USD'),
  allocatedAt: '2026-10-01T00:03:00Z',
});
const link = createSettlementAllocationSuccessorLink({
  linkId: 'allocation-link:checkpoint-compiler',
  predecessor: initial,
  successor,
  supersessionEvidenceRef: 'reconciliation:checkpoint-compiler:complete',
  linkedAt: '2026-10-01T00:03:30Z',
});

const initialRecord = projectSettlementAllocationRecord(initial, batch, recovery, [deduction50]);
const successorRecord = projectSettlementAllocationRecord(successor, batch, recovery, [deduction50, deduction25]);
const linkRecord = projectSettlementAllocationSuccessorLinkRecord(link, initial, successor);

const snapshot: PersistedSettlementAllocationLineageSourceSnapshot = {
  sourceRef: 'postgres:creator-accounting:allocation-lineage',
  sourceInstanceId: 'postgres:checkpoint-compiler:test',
  batchId: batch.batchId,
  asOf: '2026-10-02T00:00:00Z',
  observedThrough: '2026-10-02T00:00:01Z',
  highWaterMark: '11',
  allocations: [
    { ingestSeq: '3', ...initialRecord, residualAuthorityRef: initialRecord.residualAuthorityRef ?? null },
    { ingestSeq: '7', ...successorRecord, residualAuthorityRef: successorRecord.residualAuthorityRef ?? null },
  ],
  links: [{ ingestSeq: '9', ...linkRecord }],
};
const contexts: Readonly<Record<string, SettlementAllocationReplayContext>> = {
  [initial.allocationId]: { recovery, deductions: [deduction50] },
  [successor.allocationId]: { recovery, deductions: [deduction50, deduction25] },
};

const signer = generateKeyPairSync('ed25519');
const publicKeyPem = signer.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const capability = {
  capabilityId: 'capability:checkpoint-compiler:sign',
  scope: SETTLEMENT_ALLOCATION_LINEAGE_CHECKPOINT_SIGNING_SCOPE,
  sourceRef: snapshot.sourceRef,
  sourceInstanceId: snapshot.sourceInstanceId,
  signerKeyId: 'key:checkpoint-compiler',
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2026-10-03T00:00:00Z',
} as const;
const trust = {
  sourceRef: snapshot.sourceRef,
  sourceInstanceId: snapshot.sourceInstanceId,
  signerKeyId: capability.signerKeyId,
  capabilityId: capability.capabilityId,
  publicKeyPem,
} as const;

function compile(
  source: PersistedSettlementAllocationLineageSourceSnapshot = snapshot,
  allocationContexts: Readonly<Record<string, SettlementAllocationReplayContext>> = contexts,
) {
  return compileSettlementAllocationLineageCheckpoint({
    checkpointId: 'checkpoint:compiled:1',
    snapshot: source,
    batch,
    allocationContexts,
  });
}

function createSigningRequest(compiled = compile()) {
  return createCompiledSettlementAllocationLineageCheckpointSigningRequest(
    compiled,
    capability,
    {
      requestId: 'sign-request:checkpoint-compiler:1',
      issuedAt: '2026-10-02T00:00:02Z',
      expiresAt: '2026-10-02T00:05:00Z',
      nonce: 'nonce:checkpoint-compiler:1',
    },
  );
}

function detachedSignature(request: SettlementAllocationLineageCheckpointSigningRequest) {
  const signedAt = '2026-10-02T00:00:03Z';
  const payload = Buffer.from(
    settlementAllocationLineageCheckpointSigningPayloadBase64(request, signedAt),
    'base64',
  );
  return {
    requestRoot: request.requestRoot,
    signerKeyId: capability.signerKeyId,
    signedAt,
    signatureBase64: signEd25519(null, payload, signer.privateKey).toString('base64'),
  } as const;
}

describe('serialized allocation-lineage checkpoint compiler', () => {
  it('replays source rows before constructing the exact checkpoint root set', () => {
    const compiled = compile();
    expect(compiled.allocations.map(item => item.allocationRoot)).toEqual([
      initial.allocationRoot,
      successor.allocationRoot,
    ]);
    expect(compiled.links.map(item => item.linkRoot)).toEqual([link.linkRoot]);
    expect(compiled.checkpoint.allocationRoots).toEqual([initial.allocationRoot, successor.allocationRoot]);
    expect(compiled.checkpoint.linkRoots).toEqual([link.linkRoot]);
    expect(compiled.checkpoint.highWaterMark).toBe('11');
  });

  it('hands a compiler-minted request to an external signer without private-key custody', () => {
    const compiled = compile();
    const request = createSigningRequest(compiled);
    const signed = attachSettlementAllocationLineageCheckpointDetachedSignature(
      compiled.checkpoint,
      request,
      detachedSignature(request),
      trust,
    );
    const verified = verifySettlementAllocationLineageCheckpoint(signed, trust);
    expect(verified.checkpointRoot).toBe(compiled.checkpoint.checkpointRoot);
    expect(verified.signingRequest.requestRoot).toBe(request.requestRoot);
    expect(verified.signingRequest.capabilityId).toBe(capability.capabilityId);
    expect(() => createCompiledSettlementAllocationLineageCheckpointSigningRequest(
      { ...compiled },
      capability,
      {
        requestId: 'sign-request:clone',
        issuedAt: '2026-10-02T00:00:02Z',
        expiresAt: '2026-10-02T00:05:00Z',
        nonce: 'nonce:clone',
      },
    )).toThrow(/must be produced by canonical source compiler/);
  });

  it('rejects capabilities that do not authorize the exact source or validity window', () => {
    const compiled = compile();
    expect(() => createCompiledSettlementAllocationLineageCheckpointSigningRequest(
      compiled,
      { ...capability, sourceInstanceId: 'postgres:other-instance' },
      {
        requestId: 'sign-request:wrong-source',
        issuedAt: '2026-10-02T00:00:02Z',
        expiresAt: '2026-10-02T00:05:00Z',
        nonce: 'nonce:wrong-source',
      },
    )).toThrow(/does not authorize checkpoint source identity/);

    expect(() => createCompiledSettlementAllocationLineageCheckpointSigningRequest(
      compiled,
      capability,
      {
        requestId: 'sign-request:expired',
        issuedAt: '2026-10-02T23:59:59Z',
        expiresAt: '2026-10-03T00:00:01Z',
        nonce: 'nonce:expired',
      },
    )).toThrow(/inside capability validity window/);
  });

  it('rejects detached signatures rebound to another request or capability', () => {
    const compiled = compile();
    const request = createSigningRequest(compiled);
    const signature = detachedSignature(request);
    expect(() => attachSettlementAllocationLineageCheckpointDetachedSignature(
      compiled.checkpoint,
      request,
      { ...signature, requestRoot: 'f'.repeat(64) },
      trust,
    )).toThrow(/does not bind the canonical signing request/);
    expect(() => attachSettlementAllocationLineageCheckpointDetachedSignature(
      compiled.checkpoint,
      request,
      signature,
      { ...trust, capabilityId: 'capability:other' },
    )).toThrow(/signing capability is not trusted/);
  });

  it('rejects persisted allocation content changed behind its authority root', () => {
    const tampered: PersistedSettlementAllocationLineageSourceSnapshot = {
      ...snapshot,
      allocations: [
        { ...snapshot.allocations[0]!, creatorPaidMinor: '424' },
        snapshot.allocations[1]!,
      ],
    };
    expect(() => compile(tampered)).toThrow(/canonical projection at creatorPaidMinor/);
  });

  it('requires the exact semantic replay context for every allocation', () => {
    const missing: Readonly<Record<string, SettlementAllocationReplayContext>> = {
      [initial.allocationId]: contexts[initial.allocationId]!,
    };
    expect(() => compile(snapshot, missing)).toThrow(/missing replay context for allocation/);
  });

  it('rejects duplicate, reordered and over-high-water ingestion cursors', () => {
    const duplicate: PersistedSettlementAllocationLineageSourceSnapshot = {
      ...snapshot,
      links: [{ ...snapshot.links[0]!, ingestSeq: '7' }],
    };
    expect(() => compile(duplicate)).toThrow(/duplicate allocation-lineage ingestion cursor/);

    const reordered: PersistedSettlementAllocationLineageSourceSnapshot = {
      ...snapshot,
      allocations: [snapshot.allocations[1]!, snapshot.allocations[0]!],
    };
    expect(() => compile(reordered)).toThrow(/strictly increasing ingestion cursor/);

    const beyond: PersistedSettlementAllocationLineageSourceSnapshot = {
      ...snapshot,
      highWaterMark: '8',
    };
    expect(() => compile(beyond)).toThrow(/exceeds snapshot highWaterMark/);

    const noncanonical: PersistedSettlementAllocationLineageSourceSnapshot = {
      ...snapshot,
      allocations: [{ ...snapshot.allocations[0]!, ingestSeq: '03' }, snapshot.allocations[1]!],
    };
    expect(() => compile(noncanonical)).toThrow(/canonical unsigned-integer text/);
  });

  it('requires a successor link cursor to follow both endpoint allocations', () => {
    const earlyLink: PersistedSettlementAllocationLineageSourceSnapshot = {
      ...snapshot,
      links: [{ ...snapshot.links[0]!, ingestSeq: '5' }],
    };
    expect(() => compile(earlyLink)).toThrow(/must follow both endpoint allocations/);
  });

  it('rejects source evidence outside the snapshot time boundary', () => {
    const future: PersistedSettlementAllocationLineageSourceSnapshot = {
      ...snapshot,
      asOf: '2026-10-01T00:02:30Z',
    };
    expect(() => compile(future)).toThrow(/allocation is later than snapshot asOf/);
  });

  it('rejects successor links whose endpoint allocation is omitted from transport', () => {
    const omitted: PersistedSettlementAllocationLineageSourceSnapshot = {
      ...snapshot,
      allocations: [snapshot.allocations[0]!],
    };
    expect(() => compile(omitted, { [initial.allocationId]: contexts[initial.allocationId]! }))
      .toThrow(/references allocation omitted from source snapshot/);
  });
});
