import assert from 'node:assert/strict';
import test from 'node:test';
import {
  AccountingAuthorityStore,
  type AccountingAuthorityPrismaClient,
  type RoyaltyEligibilityObservationInput,
  type SettlementAttemptObservationRecordInput,
} from './accounting-authority';

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
  const allocation = new FakeDelegate();
  const statement = new FakeStatementDelegate();
  const prisma: AccountingAuthorityPrismaClient = {
    royaltyObligationRecord: obligation,
    royaltyEligibilityObservation: eligibility,
    royaltyDeductionRecord: deduction,
    settlementAttemptObservationRecord: settlement,
    settlementAllocationRecord: allocation,
    royaltyStatementSnapshotRecord: statement,
  };
  return { prisma, obligation, eligibility, settlement };
}

const obligationInput = {
  id: 'obl:1', beneficiaryId: 'creator:alice', amountMinor: '500', currency: 'usd',
  observedAt: '2026-09-10T00:00:00Z', usageEvidenceRef: 'usage:epoch:1',
  rightsResolutionRef: 'rights:resolution:1', economicTermsRef: 'terms:1', obligationRoot: root('a'),
} as const;

const settlementInput = {
  id: 'obs:submitted', attemptId: 'attempt:1', batchId: 'batch:1', obligationSetRoot: root('b'),
  eligibilityAsOf: '2026-09-10T00:00:00Z', eligibilityEvidenceRoot: root('c'),
  state: 'submitted', observedAt: '2026-09-10T00:01:00Z', observationRoot: root('d'),
} as const;

const finalizedInput = {
  ...settlementInput,
  id: 'obs:final',
  state: 'finalized',
  observedAt: '2026-09-10T00:02:00Z',
  railReceiptRef: 'rail:receipt:1',
  settledAmountMinor: '500',
  settledCurrency: 'usd',
  observationRoot: root('e'),
} as const;

test('same immutable authority record replays idempotently', async () => {
  const fake = client(); const store = new AccountingAuthorityStore(fake.prisma);
  const first = await store.appendObligation(obligationInput);
  const second = await store.appendObligation({ ...obligationInput, currency: ' USD ' });
  assert.deepEqual(second, first); assert.equal(fake.obligation.createCalls, 1);
});

test('same authority id with different content fails closed', async () => {
  const fake = client(); const store = new AccountingAuthorityStore(fake.prisma);
  await store.appendObligation(obligationInput);
  await assert.rejects(store.appendObligation({ ...obligationInput, amountMinor: '501' }), /different immutable content at amountMinor/);
});

test('compiler-only eligibility states cannot be persisted as source evidence', async () => {
  const fake = client(); const store = new AccountingAuthorityStore(fake.prisma);
  const forged = {
    id: 'eligibility:forged', obligationId: 'obl:1', code: 'below_threshold',
    sourceRef: 'source:forged', observedAt: '2026-09-10T00:00:00Z', observationRoot: root('b'),
  } as unknown as RoyaltyEligibilityObservationInput;
  await assert.rejects(store.appendEligibilityObservation(forged), /unsupported source eligibility code/);
  assert.equal(fake.eligibility.createCalls, 0);
});

test('settlement persistence retains the exact eligibility snapshot identity', async () => {
  const fake = client(); const store = new AccountingAuthorityStore(fake.prisma);
  const row = await store.appendSettlementObservation(settlementInput);
  assert.equal(row.eligibilityEvidenceRoot, root('c'));
  assert.deepEqual(row.eligibilityAsOf, new Date('2026-09-10T00:00:00Z'));
  assert.equal(fake.settlement.createCalls, 1);
});

test('application store rejects settlement evidence before its eligibility snapshot', async () => {
  const fake = client(); const store = new AccountingAuthorityStore(fake.prisma);
  await assert.rejects(
    store.appendSettlementObservation({
      ...settlementInput,
      id: 'obs:early',
      eligibilityAsOf: '2026-09-10T00:02:00Z',
      observedAt: '2026-09-10T00:01:59Z',
    }),
    /predate its eligibility snapshot/,
  );
  assert.equal(fake.settlement.createCalls, 0);
});

test('runtime validation rejects settlement states outside the closed vocabulary', async () => {
  const fake = client(); const store = new AccountingAuthorityStore(fake.prisma);
  const forged = { ...settlementInput, state: 'teleported' } as unknown as SettlementAttemptObservationRecordInput;
  await assert.rejects(store.appendSettlementObservation(forged), /unsupported settlement state/);
  assert.equal(fake.settlement.createCalls, 0);
});

test('same settlement observation id cannot be rebound to another eligibility snapshot', async () => {
  const fake = client(); const store = new AccountingAuthorityStore(fake.prisma);
  await store.appendSettlementObservation(settlementInput);
  await assert.rejects(
    store.appendSettlementObservation({ ...settlementInput, eligibilityEvidenceRoot: root('e') }),
    /different immutable content at eligibilityEvidenceRoot/,
  );
});

test('finalized settlement observation requires durable receipt identity', async () => {
  const fake = client(); const store = new AccountingAuthorityStore(fake.prisma);
  const { railReceiptRef: _omit, ...withoutReceipt } = finalizedInput;
  await assert.rejects(store.appendSettlementObservation(withoutReceipt), /requires railReceiptRef/);
  assert.equal(fake.settlement.createCalls, 0);
});

test('finalized settlement observation requires exact amount and currency', async () => {
  const fake = client(); const store = new AccountingAuthorityStore(fake.prisma);
  const { settledAmountMinor: _minor, settledCurrency: _currency, ...withoutAmount } = finalizedInput;
  await assert.rejects(store.appendSettlementObservation(withoutAmount), /requires exact settled amount and currency/);
  assert.equal(fake.settlement.createCalls, 0);
});

test('finalized settlement observation requires a positive amount before Prisma', async () => {
  const fake = client(); const store = new AccountingAuthorityStore(fake.prisma);
  await assert.rejects(
    store.appendSettlementObservation({ ...finalizedInput, settledAmountMinor: '0' }),
    /settled amount must be positive/,
  );
  assert.equal(fake.settlement.createCalls, 0);
});

test('non-final settlement evidence cannot carry a settled amount', async () => {
  const fake = client(); const store = new AccountingAuthorityStore(fake.prisma);
  await assert.rejects(store.appendSettlementObservation({
    ...settlementInput,
    settledAmountMinor: '500',
    settledCurrency: 'USD',
  }), /only on finalized/);
  assert.equal(fake.settlement.createCalls, 0);
});

test('valid finalized settlement normalizes and retains receipt amount evidence', async () => {
  const fake = client(); const store = new AccountingAuthorityStore(fake.prisma);
  const row = await store.appendSettlementObservation(finalizedInput);
  assert.equal(row.railReceiptRef, 'rail:receipt:1');
  assert.equal(row.settledAmountMinor, '500');
  assert.equal(row.settledCurrency, 'USD');
  assert.equal(fake.settlement.createCalls, 1);
});

test('same finalized observation id cannot be rebound to another settled amount', async () => {
  const fake = client(); const store = new AccountingAuthorityStore(fake.prisma);
  await store.appendSettlementObservation(finalizedInput);
  await assert.rejects(
    store.appendSettlementObservation({ ...finalizedInput, settledAmountMinor: '499' }),
    /different immutable content at settledAmountMinor/,
  );
});

test('statement persistence re-checks conservation before insert', async () => {
  const fake = client(); const store = new AccountingAuthorityStore(fake.prisma);
  await assert.rejects(store.appendStatementSnapshot({
    statementId: 'statement:1', kind: 'periodic', beneficiaryId: 'creator:alice',
    periodStart: '2026-09-01T00:00:00Z', periodEnd: '2026-10-01T00:00:00Z', asOf: '2026-10-02T00:00:00Z',
    obligationRoot: root('d'), adjustmentRoot: root('e'), settlementRoot: root('f'),
    grossMinor: '500', heldMinor: '100', deductionMinor: '50', netPayableMinor: '349', paidMinor: '0',
    currency: 'USD', completenessKind: 'complete', completenessData: { through: '2026-10-01T00:00:00Z' }, snapshotRoot: root('1'),
  }), /does not conserve gross value/);
});
