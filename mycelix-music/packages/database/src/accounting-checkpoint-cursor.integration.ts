import assert from 'node:assert/strict';
import test from 'node:test';
import { PrismaClient } from '@prisma/client';

const root = (character: string) => character.repeat(64);
const signatureShapeOnly = 'A'.repeat(86) + '==';
const sourceRef = 'postgres:checkpoint-cursor-integration';
const sourceInstanceId = 'cluster:checkpoint-cursor-integration';
const batchId = 'batch:checkpoint-cursor-integration';

async function insertCheckpoint(
  prisma: PrismaClient,
  input: { id: string; root: string; asOf: string; observedThrough: string; highWaterMark: string },
): Promise<void> {
  await prisma.$executeRawUnsafe(
    'INSERT INTO "SettlementAllocationLineageCheckpointRecord" ' +
    '("checkpointId", "sourceRef", "sourceInstanceId", "batchId", "asOf", "observedThrough", ' +
    '"highWaterMark", "allocationRoots", "linkRoots", "checkpointRoot", "signerKeyId", "signatureBase64") ' +
    'VALUES ($1, $2, $3, $4, $5, $6, $7, $8::jsonb, $9::jsonb, $10, $11, $12)',
    input.id,
    sourceRef,
    sourceInstanceId,
    batchId,
    input.asOf,
    input.observedThrough,
    input.highWaterMark,
    JSON.stringify([root('2')]),
    JSON.stringify([]),
    input.root,
    'checkpoint-key:integration',
    signatureShapeOnly,
  );
}

async function insertCursor(
  prisma: PrismaClient,
  input: {
    id: string;
    root: string;
    asOf: string;
    observedThrough: string;
    highWaterMark: string;
    revision: number;
  },
): Promise<void> {
  await prisma.$executeRawUnsafe(
    'INSERT INTO "SettlementAllocationLineageCheckpointCursor" ' +
    '("sourceRef", "sourceInstanceId", "batchId", "checkpointId", "checkpointRoot", "asOf", ' +
    '"observedThrough", "highWaterMark", "revision", "updatedAt") ' +
    'VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, NOW())',
    sourceRef,
    sourceInstanceId,
    batchId,
    input.id,
    input.root,
    input.asOf,
    input.observedThrough,
    input.highWaterMark,
    input.revision,
  );
}

test('PostgreSQL keeps checkpoint evidence append-only and rejects cursor rollback', async () => {
  assert.ok(process.env.DATABASE_URL, 'DATABASE_URL is required for the PostgreSQL checkpoint cursor integration test');
  const prisma = new PrismaClient();
  try {
    const evidenceTrigger = await prisma.$queryRawUnsafe<Array<{ count: number }>>(
      'SELECT COUNT(*)::int AS "count" FROM pg_trigger ' +
      'WHERE tgname = \'accounting_append_only_guard\' AND NOT tgisinternal ' +
      'AND tgrelid = \'"SettlementAllocationLineageCheckpointRecord"\'::regclass'
    );
    assert.equal(evidenceTrigger[0]?.count, 1);

    const cursorTrigger = await prisma.$queryRawUnsafe<Array<{ count: number }>>(
      'SELECT COUNT(*)::int AS "count" FROM pg_trigger ' +
      'WHERE tgname = \'settlement_allocation_checkpoint_cursor_guard\' AND NOT tgisinternal ' +
      'AND tgrelid = \'"SettlementAllocationLineageCheckpointCursor"\'::regclass'
    );
    assert.equal(cursorTrigger[0]?.count, 1);

    const first = {
      id: 'checkpoint:sql:1',
      root: root('1'),
      asOf: '2026-10-02T00:00:00.000Z',
      observedThrough: '2026-10-02T00:00:05.000Z',
      highWaterMark: '10',
    };
    await insertCheckpoint(prisma, first);

    await assert.rejects(
      insertCursor(prisma, { ...first, revision: 2 }),
      /initial settlement allocation checkpoint cursor revision must equal 1/,
    );
    await assert.rejects(
      insertCursor(prisma, {
        ...first,
        id: 'checkpoint:sql:missing-evidence',
        root: root('9'),
        revision: 1,
      }),
      /cursor must reference exact immutable evidence/,
    );
    await assert.rejects(
      insertCursor(prisma, {
        ...first,
        id: 'checkpoint:sql:forged-identity',
        revision: 1,
      }),
      /cursor must reference exact immutable evidence/,
    );

    await insertCursor(prisma, { ...first, revision: 1 });
    await assert.rejects(
      prisma.$executeRawUnsafe(
        'UPDATE "SettlementAllocationLineageCheckpointCursor" SET "revision" = 2 ' +
        'WHERE "sourceRef" = $1 AND "sourceInstanceId" = $2 AND "batchId" = $3',
        sourceRef,
        sourceInstanceId,
        batchId,
      ),
      /requires a new root and revision \+ 1/,
    );

    const backwards = {
      id: 'checkpoint:sql:rollback',
      root: root('3'),
      asOf: '2026-10-03T00:00:00.000Z',
      observedThrough: '2026-10-03T00:00:05.000Z',
      highWaterMark: '9',
    };
    await insertCheckpoint(prisma, backwards);
    await assert.rejects(
      prisma.$executeRawUnsafe(
        'UPDATE "SettlementAllocationLineageCheckpointCursor" SET ' +
        '"checkpointId" = $1, "checkpointRoot" = $2, "asOf" = $3, "observedThrough" = $4, ' +
        '"highWaterMark" = $5, "revision" = 2 ' +
        'WHERE "sourceRef" = $6 AND "sourceInstanceId" = $7 AND "batchId" = $8',
        backwards.id,
        backwards.root,
        backwards.asOf,
        backwards.observedThrough,
        backwards.highWaterMark,
        sourceRef,
        sourceInstanceId,
        batchId,
      ),
      /highWaterMark cannot move backwards/,
    );

    const next = {
      id: 'checkpoint:sql:2',
      root: root('4'),
      asOf: '2026-10-03T00:00:00.000Z',
      observedThrough: '2026-10-03T00:00:05.000Z',
      highWaterMark: '11',
    };
    await insertCheckpoint(prisma, next);
    await prisma.$executeRawUnsafe(
      'UPDATE "SettlementAllocationLineageCheckpointCursor" SET ' +
      '"checkpointId" = $1, "checkpointRoot" = $2, "asOf" = $3, "observedThrough" = $4, ' +
      '"highWaterMark" = $5, "revision" = 2 ' +
      'WHERE "sourceRef" = $6 AND "sourceInstanceId" = $7 AND "batchId" = $8',
      next.id,
      next.root,
      next.asOf,
      next.observedThrough,
      next.highWaterMark,
      sourceRef,
      sourceInstanceId,
      batchId,
    );

    const accepted = await prisma.$queryRawUnsafe<Array<{ checkpointRoot: string; highWaterMark: string; revision: number }>>(
      'SELECT "checkpointRoot", "highWaterMark", "revision" FROM "SettlementAllocationLineageCheckpointCursor" ' +
      'WHERE "sourceRef" = $1 AND "sourceInstanceId" = $2 AND "batchId" = $3',
      sourceRef,
      sourceInstanceId,
      batchId,
    );
    assert.equal(accepted[0]?.checkpointRoot, next.root);
    assert.equal(accepted[0]?.highWaterMark, '11');
    assert.equal(accepted[0]?.revision, 2);

    // An idle source may advance observation coverage without changing its data HWM.
    const idle = {
      id: 'checkpoint:sql:idle-observation',
      root: root('7'),
      asOf: next.asOf,
      observedThrough: '2026-10-03T00:00:10.000Z',
      highWaterMark: next.highWaterMark,
    };
    await insertCheckpoint(prisma, idle);
    await prisma.$executeRawUnsafe(
      'UPDATE "SettlementAllocationLineageCheckpointCursor" SET ' +
      '"checkpointId" = $1, "checkpointRoot" = $2, "asOf" = $3, "observedThrough" = $4, ' +
      '"highWaterMark" = $5, "revision" = 3 ' +
      'WHERE "sourceRef" = $6 AND "sourceInstanceId" = $7 AND "batchId" = $8',
      idle.id,
      idle.root,
      idle.asOf,
      idle.observedThrough,
      idle.highWaterMark,
      sourceRef,
      sourceInstanceId,
      batchId,
    );
    const idleAccepted = await prisma.$queryRawUnsafe<Array<{
      checkpointRoot: string; highWaterMark: string; observedThrough: Date; revision: number;
    }>>(
      'SELECT "checkpointRoot", "highWaterMark", "observedThrough", "revision" ' +
      'FROM "SettlementAllocationLineageCheckpointCursor" ' +
      'WHERE "sourceRef" = $1 AND "sourceInstanceId" = $2 AND "batchId" = $3',
      sourceRef,
      sourceInstanceId,
      batchId,
    );
    assert.equal(idleAccepted[0]?.checkpointRoot, idle.root);
    assert.equal(idleAccepted[0]?.highWaterMark, '11');
    assert.equal(new Date(idleAccepted[0]!.observedThrough).toISOString(), idle.observedThrough);
    assert.equal(idleAccepted[0]?.revision, 3);

    const timestampRollback = {
      id: 'checkpoint:sql:timestamp-rollback',
      root: root('5'),
      asOf: '2026-10-01T00:00:00.000Z',
      observedThrough: '2026-10-01T00:00:05.000Z',
      highWaterMark: '12',
    };
    await insertCheckpoint(prisma, timestampRollback);
    await assert.rejects(
      prisma.$executeRawUnsafe(
        'UPDATE "SettlementAllocationLineageCheckpointCursor" SET ' +
        '"checkpointId" = $1, "checkpointRoot" = $2, "asOf" = $3, "observedThrough" = $4, ' +
        '"highWaterMark" = $5, "revision" = 4 ' +
        'WHERE "sourceRef" = $6 AND "sourceInstanceId" = $7 AND "batchId" = $8',
        timestampRollback.id,
        timestampRollback.root,
        timestampRollback.asOf,
        timestampRollback.observedThrough,
        timestampRollback.highWaterMark,
        sourceRef,
        sourceInstanceId,
        batchId,
      ),
      /timestamps cannot move backwards/,
    );

    const noProgress = {
      id: 'checkpoint:sql:no-progress',
      root: root('6'),
      asOf: idle.asOf,
      observedThrough: idle.observedThrough,
      highWaterMark: idle.highWaterMark,
    };
    await insertCheckpoint(prisma, noProgress);
    await assert.rejects(
      prisma.$executeRawUnsafe(
        'UPDATE "SettlementAllocationLineageCheckpointCursor" SET ' +
        '"checkpointId" = $1, "checkpointRoot" = $2, "asOf" = $3, "observedThrough" = $4, ' +
        '"highWaterMark" = $5, "revision" = 4 ' +
        'WHERE "sourceRef" = $6 AND "sourceInstanceId" = $7 AND "batchId" = $8',
        noProgress.id,
        noProgress.root,
        noProgress.asOf,
        noProgress.observedThrough,
        noProgress.highWaterMark,
        sourceRef,
        sourceInstanceId,
        batchId,
      ),
      /must advance a source cursor or observation coverage/,
    );

    // Decimal high-water marks must stay exact beyond JavaScript's safe-integer range.
    const largeCursor = {
      id: 'checkpoint:sql:large-hwm',
      root: root('8'),
      asOf: '2026-10-04T00:00:00.000Z',
      observedThrough: '2026-10-04T00:00:05.000Z',
      highWaterMark: '9007199254740993',
    };
    await insertCheckpoint(prisma, largeCursor);
    await prisma.$executeRawUnsafe(
      'UPDATE "SettlementAllocationLineageCheckpointCursor" SET ' +
      '"checkpointId" = $1, "checkpointRoot" = $2, "asOf" = $3, "observedThrough" = $4, ' +
      '"highWaterMark" = $5, "revision" = 4 ' +
      'WHERE "sourceRef" = $6 AND "sourceInstanceId" = $7 AND "batchId" = $8',
      largeCursor.id,
      largeCursor.root,
      largeCursor.asOf,
      largeCursor.observedThrough,
      largeCursor.highWaterMark,
      sourceRef,
      sourceInstanceId,
      batchId,
    );
    const largeAccepted = await prisma.$queryRawUnsafe<Array<{ highWaterMark: string; revision: number }>>(
      'SELECT "highWaterMark", "revision" FROM "SettlementAllocationLineageCheckpointCursor" ' +
      'WHERE "sourceRef" = $1 AND "sourceInstanceId" = $2 AND "batchId" = $3',
      sourceRef,
      sourceInstanceId,
      batchId,
    );
    assert.equal(largeAccepted[0]?.highWaterMark, '9007199254740993');
    assert.equal(largeAccepted[0]?.revision, 4);

    const largeRollback = {
      id: 'checkpoint:sql:large-hwm-rollback',
      root: root('9'),
      asOf: '2026-10-05T00:00:00.000Z',
      observedThrough: '2026-10-05T00:00:05.000Z',
      highWaterMark: '9007199254740992',
    };
    await insertCheckpoint(prisma, largeRollback);
    await assert.rejects(
      prisma.$executeRawUnsafe(
        'UPDATE "SettlementAllocationLineageCheckpointCursor" SET ' +
        '"checkpointId" = $1, "checkpointRoot" = $2, "asOf" = $3, "observedThrough" = $4, ' +
        '"highWaterMark" = $5, "revision" = 5 ' +
        'WHERE "sourceRef" = $6 AND "sourceInstanceId" = $7 AND "batchId" = $8',
        largeRollback.id,
        largeRollback.root,
        largeRollback.asOf,
        largeRollback.observedThrough,
        largeRollback.highWaterMark,
        sourceRef,
        sourceInstanceId,
        batchId,
      ),
      /highWaterMark cannot move backwards/,
    );

    await assert.rejects(
      prisma.$executeRawUnsafe(
        'DELETE FROM "SettlementAllocationLineageCheckpointCursor" WHERE "sourceRef" = $1 ' +
        'AND "sourceInstanceId" = $2 AND "batchId" = $3',
        sourceRef,
        sourceInstanceId,
        batchId,
      ),
      /cannot be deleted/,
    );
    await assert.rejects(
      prisma.$executeRawUnsafe('TRUNCATE TABLE "SettlementAllocationLineageCheckpointCursor"'),
      /TRUNCATE is forbidden/,
    );
    await assert.rejects(
      prisma.$executeRawUnsafe('TRUNCATE TABLE "SettlementAllocationLineageCheckpointRecord"'),
      /TRUNCATE is forbidden/,
    );
    await assert.rejects(
      prisma.$executeRawUnsafe(
        'UPDATE "SettlementAllocationLineageCheckpointRecord" SET "highWaterMark" = $1 WHERE "checkpointRoot" = $2',
        '12',
        next.root,
      ),
      /append-only/,
    );
  } finally {
    await prisma.$disconnect();
  }
});
