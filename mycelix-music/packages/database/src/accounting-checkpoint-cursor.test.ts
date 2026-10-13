import assert from 'node:assert/strict';
import { generateKeyPairSync } from 'node:crypto';
import test from 'node:test';
import {
  createSettlementAllocationLineageCheckpoint,
  signSettlementAllocationLineageCheckpoint,
  type SettlementAllocationLineageCheckpointTrustPolicy,
  type SignedSettlementAllocationLineageCheckpoint,
} from '../../accounting/src/settlement-allocation-lineage-checkpoint.js';
import type { SettlementAllocationAuthority } from '../../accounting/src/settlement-allocation.js';
import {
  AccountingAuthorityStore,
  type AccountingAuthorityPrismaClient,
} from './accounting-authority';

const root = (character: string) => character.repeat(64);
const keys = generateKeyPairSync('ed25519');
const privateKeyPem = keys.privateKey.export({ format: 'pem', type: 'pkcs8' }).toString();
const trust: SettlementAllocationLineageCheckpointTrustPolicy = {
  sourceRef: 'postgres:music-accounting',
  sourceInstanceId: 'cluster:test:music-accounting',
  signerKeyId: 'checkpoint-key:test',
  publicKeyPem: keys.publicKey.export({ format: 'pem', type: 'spki' }).toString(),
};

const allocation = {
  allocationId: 'allocation:test:1',
  batchId: 'batch:test:1',
  obligationSetRoot: root('1'),
  eligibilityAsOf: '2026-09-10T00:00:00.000Z',
  eligibilityEvidenceRoot: root('2'),
  beneficiaryId: 'creator:test',
  currency: 'USD',
  allocationRoot: root('3'),
  allocatedAt: '2026-09-10T00:02:00.000Z',
  obligationSetDischarged: true,
} as unknown as SettlementAllocationAuthority;

function signedCheckpoint(options: {
  id: string;
  highWaterMark: string;
  asOf?: string;
  observedThrough?: string;
}): Readonly<SignedSettlementAllocationLineageCheckpoint> {
  const asOf = options.asOf ?? '2026-10-02T00:00:00.000Z';
  const unsigned = createSettlementAllocationLineageCheckpoint({
    checkpointId: options.id,
    sourceRef: trust.sourceRef,
    sourceInstanceId: trust.sourceInstanceId,
    batchId: allocation.batchId,
    asOf,
    observedThrough: options.observedThrough ?? '2026-10-02T00:00:05.000Z',
    highWaterMark: options.highWaterMark,
    allocations: [allocation],
    links: [],
  });
  return signSettlementAllocationLineageCheckpoint(unsigned, trust.signerKeyId, privateKeyPem);
}

function uniqueError(): Error & { code: string } {
  return Object.assign(new Error('unique collision'), { code: 'P2002' });
}

const compoundId = (key: { sourceRef: string; sourceInstanceId: string; batchId: string }, id: string) =>
  [key.sourceRef, key.sourceInstanceId, key.batchId, id].join('\u0000');
const cursorId = (key: { sourceRef: string; sourceInstanceId: string; batchId: string }) =>
  [key.sourceRef, key.sourceInstanceId, key.batchId].join('\u0000');

class FakeCheckpointEvidence {
  readonly byRoot = new Map<string, Record<string, unknown>>();
  readonly byIdentity = new Map<string, Record<string, unknown>>();

  async findUnique(args: { where: Record<string, unknown> }): Promise<Record<string, unknown> | null> {
    if (typeof args.where.checkpointRoot === 'string') {
      return this.byRoot.get(args.where.checkpointRoot) ?? null;
    }
    const composite = args.where.sourceRef_sourceInstanceId_batchId_checkpointId as
      | ({ sourceRef: string; sourceInstanceId: string; batchId: string; checkpointId: string })
      | undefined;
    if (!composite) return null;
    return this.byIdentity.get(compoundId(composite, composite.checkpointId)) ?? null;
  }

  async create(args: { data: Record<string, unknown> }): Promise<Record<string, unknown>> {
    const row = {
      ...args.data,
      acceptedAt: new Date('2026-10-02T00:01:00.000Z'),
    };
    const key = {
      sourceRef: String(row.sourceRef),
      sourceInstanceId: String(row.sourceInstanceId),
      batchId: String(row.batchId),
    };
    const id = compoundId(key, String(row.checkpointId));
    const rootValue = String(row.checkpointRoot);
    if (this.byRoot.has(rootValue) || this.byIdentity.has(id)) throw uniqueError();
    this.byRoot.set(rootValue, row);
    this.byIdentity.set(id, row);
    return row;
  }

  snapshot() {
    return { byRoot: new Map(this.byRoot), byIdentity: new Map(this.byIdentity) };
  }
  restore(snapshot: ReturnType<FakeCheckpointEvidence['snapshot']>) {
    this.byRoot.clear();
    this.byIdentity.clear();
    for (const [key, value] of snapshot.byRoot) this.byRoot.set(key, value);
    for (const [key, value] of snapshot.byIdentity) this.byIdentity.set(key, value);
  }
}

class FakeCheckpointCursor {
  readonly rows = new Map<string, Record<string, unknown>>();

  async findUnique(args: { where: { sourceRef_sourceInstanceId_batchId: {
    sourceRef: string; sourceInstanceId: string; batchId: string;
  } } }): Promise<Record<string, unknown> | null> {
    return this.rows.get(cursorId(args.where.sourceRef_sourceInstanceId_batchId)) ?? null;
  }

  async create(args: { data: Record<string, unknown> }): Promise<Record<string, unknown>> {
    const key = {
      sourceRef: String(args.data.sourceRef),
      sourceInstanceId: String(args.data.sourceInstanceId),
      batchId: String(args.data.batchId),
    };
    const id = cursorId(key);
    if (this.rows.has(id)) throw uniqueError();
    const row = { ...args.data, updatedAt: new Date('2026-10-02T00:01:00.000Z') };
    this.rows.set(id, row);
    return row;
  }

  async updateMany(args: {
    where: { sourceRef: string; sourceInstanceId: string; batchId: string; revision: number; checkpointRoot: string };
    data: Record<string, unknown>;
  }): Promise<{ count: number }> {
    const id = cursorId(args.where);
    const current = this.rows.get(id);
    if (!current || current.revision !== args.where.revision || current.checkpointRoot !== args.where.checkpointRoot) {
      return { count: 0 };
    }
    const row = { ...current, ...args.data, updatedAt: new Date('2026-10-03T00:00:00.000Z') };
    this.rows.set(id, row);
    return { count: 1 };
  }

  snapshot() { return new Map(this.rows); }
  restore(snapshot: Map<string, Record<string, unknown>>) {
    this.rows.clear();
    for (const [key, value] of snapshot) this.rows.set(key, value);
  }
}

function storeFixture() {
  const evidence = new FakeCheckpointEvidence();
  const cursor = new FakeCheckpointCursor();
  const prisma = {
    settlementAllocationLineageCheckpointRecord: evidence,
    settlementAllocationLineageCheckpointCursor: cursor,
    $transaction: async (
      callback: (transaction: unknown) => Promise<unknown>,
      _options: { isolationLevel: 'Serializable'; maxWait: number; timeout: number },
    ) => {
      const evidenceSnapshot = evidence.snapshot();
      const cursorSnapshot = cursor.snapshot();
      try {
        return await callback(prisma);
      } catch (error) {
        evidence.restore(evidenceSnapshot);
        cursor.restore(cursorSnapshot);
        throw error;
      }
    },
  };
  const store = new AccountingAuthorityStore(prisma as unknown as AccountingAuthorityPrismaClient);
  return { store, evidence, cursor };
}

test('checkpoint acceptance creates immutable evidence and revision-one cursor atomically', async () => {
  const fixture = storeFixture();
  const checkpoint = signedCheckpoint({ id: 'checkpoint:1', highWaterMark: '10' });
  const result = await fixture.store.acceptSettlementAllocationLineageCheckpoint({ checkpoint, trustPolicy: trust });
  assert.equal(result.disposition, 'accepted');
  assert.equal(result.revision, 1);
  assert.equal(fixture.evidence.byRoot.size, 1);
  assert.equal(fixture.cursor.rows.size, 1);
});

test('replaying the current signed checkpoint is idempotent and does not advance the cursor', async () => {
  const fixture = storeFixture();
  const checkpoint = signedCheckpoint({ id: 'checkpoint:1', highWaterMark: '10' });
  const first = await fixture.store.acceptSettlementAllocationLineageCheckpoint({ checkpoint, trustPolicy: trust });
  const replay = await fixture.store.acceptSettlementAllocationLineageCheckpoint({ checkpoint, trustPolicy: trust });
  assert.equal(first.disposition, 'accepted');
  assert.equal(replay.disposition, 'replayed');
  assert.equal(replay.revision, 1);
  assert.equal(fixture.evidence.byRoot.size, 1);
});

test('a valid advancing checkpoint appends evidence and compare-and-swaps the cursor', async () => {
  const fixture = storeFixture();
  const first = signedCheckpoint({ id: 'checkpoint:1', highWaterMark: '10' });
  const second = signedCheckpoint({
    id: 'checkpoint:2',
    highWaterMark: '11',
    asOf: '2026-10-03T00:00:00.000Z',
    observedThrough: '2026-10-03T00:00:05.000Z',
  });
  await fixture.store.acceptSettlementAllocationLineageCheckpoint({ checkpoint: first, trustPolicy: trust });
  const accepted = await fixture.store.acceptSettlementAllocationLineageCheckpoint({ checkpoint: second, trustPolicy: trust });
  assert.equal(accepted.disposition, 'accepted');
  assert.equal(accepted.revision, 2);
  assert.equal(fixture.evidence.byRoot.size, 2);
  const cursorRow = [...fixture.cursor.rows.values()][0]!;
  assert.equal(cursorRow.checkpointRoot, second.checkpointRoot);
  assert.equal(cursorRow.highWaterMark, '11');
  assert.equal(cursorRow.revision, 2);
});

test('an older already-accepted checkpoint cannot roll the cursor back', async () => {
  const fixture = storeFixture();
  const first = signedCheckpoint({ id: 'checkpoint:1', highWaterMark: '10' });
  const second = signedCheckpoint({
    id: 'checkpoint:2',
    highWaterMark: '11',
    asOf: '2026-10-03T00:00:00.000Z',
    observedThrough: '2026-10-03T00:00:05.000Z',
  });
  await fixture.store.acceptSettlementAllocationLineageCheckpoint({ checkpoint: first, trustPolicy: trust });
  await fixture.store.acceptSettlementAllocationLineageCheckpoint({ checkpoint: second, trustPolicy: trust });
  await assert.rejects(
    fixture.store.acceptSettlementAllocationLineageCheckpoint({ checkpoint: first, trustPolicy: trust }),
    /stale and cannot replace the current cursor/,
  );
  assert.equal([...fixture.cursor.rows.values()][0]!.checkpointRoot, second.checkpointRoot);
});

test('checkpoint identity cannot be rebound to a different signed checkpoint root', async () => {
  const fixture = storeFixture();
  const first = signedCheckpoint({ id: 'checkpoint:1', highWaterMark: '10' });
  const reused = signedCheckpoint({
    id: 'checkpoint:1',
    highWaterMark: '11',
    asOf: '2026-10-03T00:00:00.000Z',
    observedThrough: '2026-10-03T00:00:05.000Z',
  });
  await fixture.store.acceptSettlementAllocationLineageCheckpoint({ checkpoint: first, trustPolicy: trust });
  await assert.rejects(
    fixture.store.acceptSettlementAllocationLineageCheckpoint({ checkpoint: reused, trustPolicy: trust }),
    /checkpointId cannot be reused/,
  );
  assert.equal(fixture.evidence.byRoot.size, 1);
  assert.equal([...fixture.cursor.rows.values()][0]!.checkpointRoot, first.checkpointRoot);
});

test('checkpoint signature and source trust are verified before durable acceptance', async () => {
  const fixture = storeFixture();
  const checkpoint = signedCheckpoint({ id: 'checkpoint:1', highWaterMark: '10' });
  await assert.rejects(
    fixture.store.acceptSettlementAllocationLineageCheckpoint({
      checkpoint: { ...checkpoint, highWaterMark: '999' },
      trustPolicy: trust,
    }),
    /root does not match canonical contents/,
  );
  await assert.rejects(
    fixture.store.acceptSettlementAllocationLineageCheckpoint({
      checkpoint,
      trustPolicy: { ...trust, sourceInstanceId: 'cluster:other' },
    }),
    /source identity is not trusted/,
  );
  assert.equal(fixture.evidence.byRoot.size, 0);
  assert.equal(fixture.cursor.rows.size, 0);
});
