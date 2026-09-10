import assert from 'node:assert/strict';
import test from 'node:test';
import { PrismaClient } from '@prisma/client';
import { AccountingAllocationLineageSource } from './accounting-lineage-source.js';

const root = (pair: string) => pair.repeat(32);
const R = {
  obligation: root('a1'),
  eligibility: root('b2'),
  deductionA: root('c3'),
  deductionB: root('d4'),
  initialAllocation: root('e5'),
  successorAllocation: root('f6'),
  link: root('07'),
  badLink: root('18'),
  bypassAllocation: root('29'),
} as const;

const batchId = 'batch:lineage-source:integration';

function allocationInsert(input: {
  allocationId: string;
  allocationRoot: string;
  creatorPaidMinor: string;
  deductionRoots: readonly string[];
  deductionTotalMinor: string;
  residualHeldMinor: string;
  residualAuthorityRef?: string;
  allocatedAt: string;
  obligationSetDischarged: boolean;
  batch?: string;
}): string {
  const roots = JSON.stringify(input.deductionRoots).replaceAll("'", "''");
  const residualAuthority = input.residualAuthorityRef === undefined
    ? 'NULL'
    : `'${input.residualAuthorityRef.replaceAll("'", "''")}'`;
  return `
    INSERT INTO "SettlementAllocationRecord"
      ("allocationId", "batchId", "obligationSetRoot", "eligibilityAsOf",
       "eligibilityEvidenceRoot", "beneficiaryId", "currency", "railReceiptRef",
       "creatorPaidMinor", "deductionRoots", "deductionTotalMinor", "residualHeldMinor",
       "residualAuthorityRef", "allocatedAt", "obligationSetDischarged", "allocationRoot")
    VALUES
      ('${input.allocationId}', '${input.batch ?? batchId}', '${R.obligation}',
       '2026-09-10T00:03:00Z', '${R.eligibility}', 'creator:lineage-source', 'USD',
       'rail:receipt:lineage-source', '${input.creatorPaidMinor}', '${roots}'::jsonb,
       '${input.deductionTotalMinor}', '${input.residualHeldMinor}', ${residualAuthority},
       '${input.allocatedAt}', ${input.obligationSetDischarged}, '${input.allocationRoot}')
  `;
}

test('PostgreSQL serializes allocation-lineage checkpoint source evidence', async () => {
  assert.ok(process.env.DATABASE_URL, 'DATABASE_URL is required for the lineage-source integration test');
  const prisma = new PrismaClient();
  const source = new AccountingAllocationLineageSource(prisma, {
    sourceRef: 'postgres:creator-accounting:allocation-lineage',
    sourceInstanceId: 'ci:ephemeral-postgres',
  });

  try {
    const tables = await prisma.$queryRawUnsafe<Array<{ tableName: string }>>(`
      SELECT table_name AS "tableName"
        FROM information_schema.tables
       WHERE table_schema = 'public'
         AND table_name IN (
           'SettlementAllocationSuccessorLinkRecord',
           'SettlementAllocationLineageIngestRecord'
         )
       ORDER BY table_name
    `);
    assert.deepEqual(tables.map(row => row.tableName), [
      'SettlementAllocationLineageIngestRecord',
      'SettlementAllocationSuccessorLinkRecord',
    ]);

    const triggerNames = await prisma.$queryRawUnsafe<Array<{ name: string }>>(`
      SELECT tgname AS "name"
        FROM pg_trigger
       WHERE NOT tgisinternal
         AND tgname IN (
           'settlement_allocation_lineage_writer_lock',
           'settlement_allocation_ingest_register',
           'settlement_allocation_successor_link_writer_lock',
           'settlement_allocation_successor_link_semantic_guard',
           'settlement_allocation_successor_link_ingest_register',
           'settlement_allocation_successor_link_append_only_guard',
           'settlement_allocation_lineage_ingest_assign_guard',
           'settlement_allocation_lineage_ingest_append_only_guard'
         )
       ORDER BY tgname
    `);
    assert.equal(triggerNames.length, 8, 'serialized lineage source must install all writer/snapshot guards');

    await prisma.$executeRawUnsafe(allocationInsert({
      allocationId: 'allocation:lineage-source:initial',
      allocationRoot: R.initialAllocation,
      creatorPaidMinor: '425',
      deductionRoots: [R.deductionA],
      deductionTotalMinor: '50',
      residualHeldMinor: '25',
      residualAuthorityRef: 'reconciliation:lineage-source:25',
      allocatedAt: '2026-09-10T00:04:00Z',
      obligationSetDischarged: false,
    }));

    await prisma.$executeRawUnsafe(allocationInsert({
      allocationId: 'allocation:lineage-source:successor',
      allocationRoot: R.successorAllocation,
      creatorPaidMinor: '425',
      deductionRoots: [R.deductionA, R.deductionB],
      deductionTotalMinor: '75',
      residualHeldMinor: '0',
      allocatedAt: '2026-09-10T00:05:00Z',
      obligationSetDischarged: true,
    }));

    await source.appendSuccessorLink({
      linkId: 'allocation-link:lineage-source',
      batchId,
      predecessorAllocationId: 'allocation:lineage-source:initial',
      predecessorAllocationRoot: R.initialAllocation,
      successorAllocationId: 'allocation:lineage-source:successor',
      successorAllocationRoot: R.successorAllocation,
      supersessionEvidenceRef: 'reconciliation:lineage-source:complete',
      linkedAt: '2026-09-10T00:06:00Z',
      linkRoot: R.link,
    });

    // Exact-content replay is idempotent at the operational source boundary.
    await source.appendSuccessorLink({
      linkId: 'allocation-link:lineage-source',
      batchId,
      predecessorAllocationId: 'allocation:lineage-source:initial',
      predecessorAllocationRoot: R.initialAllocation,
      successorAllocationId: 'allocation:lineage-source:successor',
      successorAllocationRoot: R.successorAllocation,
      supersessionEvidenceRef: 'reconciliation:lineage-source:complete',
      linkedAt: '2026-09-10T00:06:00Z',
      linkRoot: R.link,
    });

    const ingest = await prisma.$queryRawUnsafe<Array<{
      ingestSeq: string;
      recordKind: string;
      recordId: string;
    }>>(`
      SELECT "ingestSeq"::text AS "ingestSeq", "recordKind", "recordId"
        FROM "SettlementAllocationLineageIngestRecord"
       WHERE "batchId" = '${batchId}'
       ORDER BY "ingestSeq" ASC
    `);
    assert.deepEqual(ingest.map(row => [row.recordKind, row.recordId]), [
      ['allocation', 'allocation:lineage-source:initial'],
      ['allocation', 'allocation:lineage-source:successor'],
      ['successor_link', 'allocation-link:lineage-source'],
    ]);
    const seq = ingest.map(row => BigInt(row.ingestSeq));
    assert.ok(seq[0]! < seq[1]! && seq[1]! < seq[2]!, 'ingestion cursor must preserve serialized source order');

    await assert.rejects(
      prisma.$executeRawUnsafe(`
        INSERT INTO "SettlementAllocationLineageIngestRecord"
          ("ingestSeq", "recordKind", "recordId", "batchId", "recordRoot", "evidenceAt")
        VALUES
          (999999, 'allocation', 'allocation:lineage-source:initial', '${batchId}',
           '${R.initialAllocation}', '2026-09-10T00:04:00Z')
      `),
      /cursor_is_database_assigned/,
    );

    await assert.rejects(
      source.appendSuccessorLink({
        linkId: 'allocation-link:lineage-source:bad-root',
        batchId,
        predecessorAllocationId: 'allocation:lineage-source:initial',
        predecessorAllocationRoot: R.badLink,
        successorAllocationId: 'allocation:lineage-source:successor',
        successorAllocationRoot: R.successorAllocation,
        supersessionEvidenceRef: 'reconciliation:lineage-source:bad-root',
        linkedAt: '2026-09-10T00:06:30Z',
        linkRoot: R.badLink,
      }),
      /root_mismatch/,
    );

    const snapshot = await source.captureSnapshot({
      batchId,
      asOf: '2026-09-10T00:10:00Z',
    });
    assert.equal(snapshot.sourceRef, 'postgres:creator-accounting:allocation-lineage');
    assert.equal(snapshot.sourceInstanceId, 'ci:ephemeral-postgres');
    assert.equal(snapshot.batchId, batchId);
    assert.equal(snapshot.asOf, '2026-09-10T00:10:00.000Z');
    assert.ok(Date.parse(snapshot.observedThrough) >= Date.parse(snapshot.asOf));
    assert.ok(BigInt(snapshot.highWaterMark) >= seq[2]!);
    assert.deepEqual(snapshot.allocations.map(row => row.allocationRoot), [
      R.initialAllocation,
      R.successorAllocation,
    ]);
    assert.deepEqual(snapshot.links.map(row => row.linkRoot), [R.link]);
    assert.deepEqual(snapshot.allocations.map(row => row.ingestSeq), [ingest[0]!.ingestSeq, ingest[1]!.ingestSeq]);
    assert.deepEqual(snapshot.links.map(row => row.ingestSeq), [ingest[2]!.ingestSeq]);

    await assert.rejects(
      prisma.$executeRawUnsafe(`
        UPDATE "SettlementAllocationSuccessorLinkRecord"
           SET "supersessionEvidenceRef" = 'tampered'
         WHERE "linkId" = 'allocation-link:lineage-source'
      `),
      /append-only/,
    );
    await assert.rejects(
      prisma.$executeRawUnsafe(`
        DELETE FROM "SettlementAllocationLineageIngestRecord"
         WHERE "recordKind" = 'successor_link'
           AND "recordId" = 'allocation-link:lineage-source'
      `),
      /append-only/,
    );

    // Simulate a privileged operational bypass that suppresses user triggers.
    // The checkpoint reader must detect this row as missing from the ingestion
    // registry instead of silently producing a truncated "complete" snapshot.
    const bypassBatch = 'batch:lineage-source:bypass';
    await prisma.$transaction(async tx => {
      await tx.$executeRawUnsafe('SET LOCAL session_replication_role = replica');
      await tx.$executeRawUnsafe(allocationInsert({
        allocationId: 'allocation:lineage-source:bypass',
        allocationRoot: R.bypassAllocation,
        creatorPaidMinor: '500',
        deductionRoots: [],
        deductionTotalMinor: '0',
        residualHeldMinor: '0',
        allocatedAt: '2026-09-10T00:07:00Z',
        obligationSetDischarged: true,
        batch: bypassBatch,
      }));
    });
    await assert.rejects(
      source.captureSnapshot({ batchId: bypassBatch, asOf: '2026-09-10T00:10:00Z' }),
      /unregistered eligible evidence/,
    );
  } finally {
    await prisma.$disconnect();
  }
});
