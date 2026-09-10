import assert from 'node:assert/strict';
import test from 'node:test';
import {
  AccountingAuthorityStore,
  type AccountingAuthorityPrismaClient,
  type SettlementAllocationRecordInput,
} from './accounting-authority';

const root = (character: string) => character.repeat(64);

class FakeAllocationDelegate {
  readonly rows = new Map<string, Record<string, unknown>>();
  createCalls = 0;

  async findUnique(args: { where: { allocationId: string } }): Promise<Record<string, unknown> | null> {
    return this.rows.get(args.where.allocationId) ?? null;
  }

  async create(args: { data: Record<string, unknown> }): Promise<Record<string, unknown>> {
    this.createCalls += 1;
    const allocationId = String(args.data.allocationId);
    if (this.rows.has(allocationId)) throw new Error('unique collision');
    const row = { ...args.data, createdAt: new Date('2026-09-10T00:00:00Z') };
    this.rows.set(allocationId, row);
    return row;
  }
}

const unavailableDelegate = {
  async findUnique() { return null; },
  async create() { return {}; },
};

function client() {
  const allocation = new FakeAllocationDelegate();
  const prisma: AccountingAuthorityPrismaClient = {
    royaltyObligationRecord: unavailableDelegate,
    royaltyEligibilityObservation: unavailableDelegate,
    royaltyDeductionRecord: unavailableDelegate,
    settlementAttemptObservationRecord: unavailableDelegate,
    settlementAllocationRecord: allocation,
    royaltyStatementSnapshotRecord: unavailableDelegate,
  };
  return { prisma, allocation };
}

const valid: SettlementAllocationRecordInput = {
  allocationId: 'allocation:1',
  batchId: 'batch:1',
  obligationSetRoot: root('1'),
  eligibilityAsOf: '2026-09-10T00:00:00Z',
  eligibilityEvidenceRoot: root('2'),
  beneficiaryId: 'creator:alice',
  currency: 'usd',
  railReceiptRef: 'rail:receipt:1',
  creatorPaidMinor: '450',
  deductionRoots: [root('3')],
  deductionTotalMinor: '50',
  residualHeldMinor: '0',
  allocatedAt: '2026-09-10T00:02:00Z',
  obligationSetDischarged: true,
  allocationRoot: root('4'),
};

test('same settlement allocation replays idempotently after normalization', async () => {
  const fake = client();
  const store = new AccountingAuthorityStore(fake.prisma);
  const first = await store.appendSettlementAllocation(valid);
  const second = await store.appendSettlementAllocation({ ...valid, currency: ' USD ' });
  assert.deepEqual(second, first);
  assert.equal(first.currency, 'USD');
  assert.deepEqual(first.eligibilityAsOf, new Date('2026-09-10T00:00:00Z'));
  assert.equal(fake.allocation.createCalls, 1);
});

test('same allocation identity cannot be rebound to changed economic content', async () => {
  const fake = client();
  const store = new AccountingAuthorityStore(fake.prisma);
  await store.appendSettlementAllocation(valid);
  await assert.rejects(
    store.appendSettlementAllocation({ ...valid, creatorPaidMinor: '449' }),
    /different immutable content at creatorPaidMinor/,
  );
});

test('allocation deduction roots must be canonical sorted unique digests', async () => {
  const fake = client();
  const store = new AccountingAuthorityStore(fake.prisma);
  await assert.rejects(
    store.appendSettlementAllocation({ ...valid, deductionRoots: [root('4'), root('3')] }),
    /canonically sorted/,
  );
  await assert.rejects(
    store.appendSettlementAllocation({ ...valid, deductionRoots: [root('3'), root('3')] }),
    /must not contain duplicates/,
  );
  await assert.rejects(
    store.appendSettlementAllocation({ ...valid, deductionRoots: ['not-a-digest'] }),
    /lowercase SHA-256 digest/,
  );
  assert.equal(fake.allocation.createCalls, 0);
});

test('deduction total cannot appear without deduction authority roots', async () => {
  const fake = client();
  const store = new AccountingAuthorityStore(fake.prisma);
  await assert.rejects(
    store.appendSettlementAllocation({ ...valid, deductionRoots: [], deductionTotalMinor: '50' }),
    /without deduction roots must have zero deductionTotalMinor/,
  );
});

test('residual provenance and discharge state are exact opposites', async () => {
  const fake = client();
  const store = new AccountingAuthorityStore(fake.prisma);
  await assert.rejects(
    store.appendSettlementAllocation({ ...valid, residualHeldMinor: '25', obligationSetDischarged: false }),
    /requires residualAuthorityRef/,
  );
  await assert.rejects(
    store.appendSettlementAllocation({ ...valid, residualAuthorityRef: 'case:7' }),
    /zero settlement residual must not claim residualAuthorityRef/,
  );
  await assert.rejects(
    store.appendSettlementAllocation({ ...valid, obligationSetDischarged: false }),
    /discharge state must equal residualHeldMinor == 0/,
  );
  const held = await store.appendSettlementAllocation({
    ...valid,
    allocationId: 'allocation:held',
    creatorPaidMinor: '425',
    residualHeldMinor: '25',
    residualAuthorityRef: 'reconciliation:case:7',
    obligationSetDischarged: false,
    allocationRoot: root('5'),
  });
  assert.equal(held.obligationSetDischarged, false);
  assert.equal(held.residualAuthorityRef, 'reconciliation:case:7');
});

test('allocation rejects zero creator payment and reversed eligibility chronology', async () => {
  const fake = client();
  const store = new AccountingAuthorityStore(fake.prisma);
  await assert.rejects(
    store.appendSettlementAllocation({ ...valid, creatorPaidMinor: '0' }),
    /creatorPaidMinor must be positive/,
  );
  await assert.rejects(
    store.appendSettlementAllocation({
      ...valid,
      eligibilityAsOf: '2026-09-10T00:03:00Z',
      allocatedAt: '2026-09-10T00:02:59Z',
    }),
    /cannot predate its eligibility snapshot/,
  );
  assert.equal(fake.allocation.createCalls, 0);
});
