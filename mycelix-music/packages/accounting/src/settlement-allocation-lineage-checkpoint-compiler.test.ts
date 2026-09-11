import { generateKeyPairSync } from 'node:crypto';
import { describe, expect, it } from 'vitest';
import { createRoyaltyDeductionAuthority } from './deduction-authority.js';
import { money } from './money.js';
import { buildDeterministicNettingBatches } from './netting.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { projectSettlementAllocationRecord } from './persistence-projection.js';
import { reconstructSettlementRecovery, SettlementAttemptState } from './recovery.js';
import {
  compileSettlementAllocationLineageCheckpoint,
  signCompiledSettlementAllocationLineageCheckpoint,
  type PersistedSettlementAllocationLineageSourceSnapshot,
} from './settlement-allocation-lineage-checkpoint-compiler.js';
import {
  verifySettlementAllocationLineageCheckpoint,
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
const contexts = {
  [initial.allocationId]: { recovery, deductions: [deduction50] },
  [successor.allocationId]: { recovery, deductions: [deduction50, deduction25] },
};

function compile(source = snapshot, allocationContexts = contexts) {
  return compileSettlementAllocationLineageCheckpoint({
    checkpointId: 'checkpoint:compiled:1',
    snapshot: source,
    batch,
    allocationContexts,
  });
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

  it('signs only a compiler-minted candidate through the strict signing surface', () => {
    const { privateKey, publicKey } = generateKeyPairSync('ed25519');
    const privateKeyPem = privateKey.export({ format: 'pem', type: 'pkcs8' }).toString();
    const publicKeyPem = publicKey.export({ format: 'pem', type: 'spki' }).toString();
    const compiled = compile();
    const signed = signCompiledSettlementAllocationLineageCheckpoint(compiled, 'key:checkpoint-compiler', privateKeyPem);
    const verified = verifySettlementAllocationLineageCheckpoint(signed, {
      sourceRef: snapshot.sourceRef,
      sourceInstanceId: snapshot.sourceInstanceId,
      signerKeyId: 'key:checkpoint-compiler',
      publicKeyPem,
    });
    expect(verified.checkpointRoot).toBe(compiled.checkpoint.checkpointRoot);
    expect(() => signCompiledSettlementAllocationLineageCheckpoint({ ...compiled }, 'key:checkpoint-compiler', privateKeyPem))
      .toThrow(/must be produced by canonical source compiler/);
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
    const missing = { [initial.allocationId]: contexts[initial.allocationId] };
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
    expect(() => compile(omitted, { [initial.allocationId]: contexts[initial.allocationId] }))
      .toThrow(/references allocation omitted from source snapshot/);
  });
});
