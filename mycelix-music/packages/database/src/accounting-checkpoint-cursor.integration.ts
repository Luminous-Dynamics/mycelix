import assert from 'node:assert/strict';
import test from 'node:test';
import { PrismaClient } from '@prisma/client';

const root = (character: string) => character.repeat(64);
const signatureShapeOnly = 'A'.repeat(86) + '==';
const sourceRef = 'postgres:checkpoint-cursor-integration';
const sourceInstanceId = 'cluster:checkpoint-cursor-integration';
const batchId = 'batch:checkpoint-cursor-integration';

function insertCheckpoint(input: {
  id: string;
  root: string;
  asOf: string;
  observedThrough: string;
  highWaterMark: string;
}): string {
  return 'INSERT INTO "SettlementAllocationLineageCheckpointRecord" ' +
    '("checkpointId", "sourceRef", "sourceInstanceId", "batchId", "asOf", "observedThrough", ' +
    '"highWaterMark", "allocationRoots", "linkRoots", "checkpointRoot", "signerKeyId", "signatureBase64") ' +
    "VALUES ('" + input.id + "', '" + sourceRef + "', '" + sourceInstanceId + "', '" + batchId + "', '" +
    input.asOf + "', '" + input.observedThrough + "', '" + input.highWaterMark + "', " +
    '\'["' + root('2') + '"]\' + '::jsonb, \'[]\'::jsonb, \'' + input.root + "', " +
    "'checkpoint-key:integration', '" + signatureShapeOnly + "')";
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
    await prisma.$executeRawUnsafe(insertCheckpoint(first));
    await prisma.$executeRawUnsafe(
      'INSERT INTO "SettlementAllocationLineageCheckpointCursor" ' +
      '("sourceRef", "sourceInstanceId", "batchId", "checkpointId", "checkpointRoot", "asOf", ' +
      '"observedThrough", "highWaterMark", "revision", "updatedAt") VALUES (' +
      "'" + sourceRef + "', '" + sourceInstanceId + "', '" + batchId + "', '" + first.id + "', '" + first.root + "', " +
      "'" + first.asOf + "', '" + first.observedThrough + "', '" + first.highWaterMark + "', 1, NOW())"
    );

    const backwards = {
      id: 'checkpoint:sql:rollback',
      root: root('3'),
      asOf: '2026-10-03T00:00:00.000Z',
      observedThrough: '2026-10-03T00:00:05.000Z',
      highWaterMark: '9',
    };
    await prisma.$executeRawUnsafe(insertCheckpoint(backwards));
    await assert.rejects(
      prisma.$executeRawUnsafe(
        'UPDATE "SettlementAllocationLineageCheckpointCursor" SET ' +
        '"checkpointId" = \'' + backwards.id + '\', "checkpointRoot" = \'' + backwards.root + '\', ' +
        '"asOf" = \'' + backwards.asOf + '\', "observedThrough" = \'' + backwards.observedThrough + '\', ' +
        '"highWaterMark" = \'' + backwards.highWaterMark + '\', "revision" = 2 ' +
        'WHERE "sourceRef" = \'' + sourceRef + '\' AND "sourceInstanceId" = \'' + sourceInstanceId + '\' ' +
        'AND "batchId" = \'' + batchId + '\''
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
    await prisma.$executeRawUnsafe(insertCheckpoint(next));
    await prisma.$executeRawUnsafe(
      'UPDATE "SettlementAllocationLineageCheckpointCursor" SET ' +
      '"checkpointId" = \'' + next.id + '\', "checkpointRoot" = \'' + next.root + '\', ' +
      '"asOf" = \'' + next.asOf + '\', "observedThrough" = \'' + next.observedThrough + '\', ' +
      '"highWaterMark" = \'' + next.highWaterMark + '\', "revision" = 2 ' +
      'WHERE "sourceRef" = \'' + sourceRef + '\' AND "sourceInstanceId" = \'' + sourceInstanceId + '\' ' +
      'AND "batchId" = \'' + batchId + '\''
    );

    const accepted = await prisma.$queryRawUnsafe<Array<{ checkpointRoot: string; highWaterMark: string; revision: number }>>(
      'SELECT "checkpointRoot", "highWaterMark", "revision" FROM "SettlementAllocationLineageCheckpointCursor" ' +
      'WHERE "sourceRef" = \'' + sourceRef + '\' AND "sourceInstanceId" = \'' + sourceInstanceId + '\' ' +
      'AND "batchId" = \'' + batchId + '\''
    );
    assert.equal(accepted[0]?.checkpointRoot, next.root);
    assert.equal(accepted[0]?.highWaterMark, '11');
    assert.equal(accepted[0]?.revision, 2);

    await assert.rejects(
      prisma.$executeRawUnsafe(
        'DELETE FROM "SettlementAllocationLineageCheckpointCursor" WHERE "sourceRef" = \'' + sourceRef + '\' ' +
        'AND "sourceInstanceId" = \'' + sourceInstanceId + '\' AND "batchId" = \'' + batchId + '\''
      ),
      /cannot be deleted/,
    );
    await assert.rejects(
      prisma.$executeRawUnsafe(
        'UPDATE "SettlementAllocationLineageCheckpointRecord" SET "highWaterMark" = \'12\' ' +
        'WHERE "checkpointRoot" = \'' + next.root + '\''
      ),
      /append-only/,
    );
  } finally {
    await prisma.$disconnect();
  }
});
