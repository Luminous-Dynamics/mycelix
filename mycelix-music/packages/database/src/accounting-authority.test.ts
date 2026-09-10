import assert from 'node:assert/strict';
import test from 'node:test';
import { AccountingAuthorityStore, type AccountingAuthorityPrismaClient } from './accounting-authority';

const root = (character: string) => character.repeat(64);

class FakeDelegate {
  readonly rows = new Map<string, Record<string, unknown>>();
  createCalls = 0;

  async findUnique(args: { where: { id: string } }): Promise<Record<string, unknown> | null> {
    return this.rows.get(args.where.id) ?? null;
  }

  async create(args: { data: Record<string, unknown> }): Promise<Record<string, unknown>> {
    this.createCalls += 1;
    const id = String(args.data.id);
    if (this.rows.has(id)) throw new Error('unique collision');
    const row = { ...args.data, createdAt: new Date('2026-09-10T00:00:00Z') };
    this.rows.set(id, row);
    return row;
  }
}

class FakeStatementDelegate {
  readonly rows = new Map<string, Record<string, unknown>>();

  async findUnique(args: { where: { statementId: string } }): Promise<Record<string, unknown> | null> {
    return this.rows.get(args.where.statementId) ?? null;
  }

  async create(args: { data: Record<string, unknown> }): Promise<Record<string, unknown>> {
    const id = String(args.data.statementId);
    if (this.rows.has(id)) throw new Error('unique collision');
    const row = { ...args.data, createdAt: new Date('2026-09-10T00:00:00Z') };
    this.rows.set(id, row);
    return row;
  }
}

function client() {
  const obligation = new FakeDelegate();
  const eligibility = new FakeDelegate();
  const deduction = new FakeDelegate();
  const settlement = new FakeDelegate();
  const statement = new FakeStatementDelegate();
  const prisma: AccountingAuthorityPrismaClient = {
    royaltyObligationRecord: obligation,
    royaltyEligibilityObservation: eligibility,
    royaltyDeductionRecord: deduction,
    settlementAttemptObservationRecord: settlement,
    royaltyStatementSnapshotRecord: statement,
  };
  return { prisma, obligation };
}

const obligationInput = {
  id: 'obl:1',
  beneficiaryId: 'creator:alice',
  amountMinor: '500',
  currency: 'usd',
  observedAt: '2026-09-10T00:00:00Z',
  usageEvidenceRef: 'usage:epoch:1',
  rightsResolutionRef: 'rights:resolution:1',
  economicTermsRef: 'terms:1',
  obligationRoot: root('a'),
} as const;

test('same immutable authority record replays idempotently', async () => {
  const fake = client();
  const store = new AccountingAuthorityStore(fake.prisma);
  const first = await store.appendObligation(obligationInput);
  const second = await store.appendObligation({ ...obligationInput, currency: ' USD ' });
  assert.deepEqual(second, first);
  assert.equal(fake.obligation.createCalls, 1);
});

test('same authority id with different content fails closed', async () => {
  const fake = client();
  const store = new AccountingAuthorityStore(fake.prisma);
  await store.appendObligation(obligationInput);
  await assert.rejects(
    store.appendObligation({ ...obligationInput, amountMinor: '501' }),
    /different immutable content at amountMinor/,
  );
});

test('finalized settlement observation requires durable receipt identity', async () => {
  const fake = client();
  const store = new AccountingAuthorityStore(fake.prisma);
  await assert.rejects(store.appendSettlementObservation({
    id: 'obs:1',
    attemptId: 'attempt:1',
    batchId: 'batch:1',
    obligationSetRoot: root('b'),
    state: 'finalized',
    observedAt: '2026-09-10T00:00:00Z',
    observationRoot: root('c'),
  }), /requires railReceiptRef/);
});

test('statement persistence re-checks conservation before insert', async () => {
  const fake = client();
  const store = new AccountingAuthorityStore(fake.prisma);
  await assert.rejects(store.appendStatementSnapshot({
    statementId: 'statement:1',
    kind: 'periodic',
    beneficiaryId: 'creator:alice',
    periodStart: '2026-09-01T00:00:00Z',
    periodEnd: '2026-10-01T00:00:00Z',
    asOf: '2026-10-02T00:00:00Z',
    obligationRoot: root('d'),
    adjustmentRoot: root('e'),
    settlementRoot: root('f'),
    grossMinor: '500',
    heldMinor: '100',
    deductionMinor: '50',
    netPayableMinor: '349',
    paidMinor: '0',
    currency: 'USD',
    completenessKind: 'complete',
    completenessData: { through: '2026-10-01T00:00:00Z' },
    snapshotRoot: root('1'),
  }), /does not conserve gross value/);
});
