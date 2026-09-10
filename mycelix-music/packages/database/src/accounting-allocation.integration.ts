import assert from 'node:assert/strict';
import test from 'node:test';
import { PrismaClient } from '@prisma/client';

const root = (character: string) => character.repeat(64);

const R = {
  obligation: root('1'),
  eligibility: root('2'),
  deductionA: root('3'),
  deductionB: root('4'),
  allocation: root('5'),
  heldAllocation: root('6'),
  bad: root('7'),
} as const;

function allocationInsert(overrides: {
  allocationId?: string;
  obligationSetRoot?: string;
  eligibilityAsOf?: string;
  eligibilityEvidenceRoot?: string;
  railReceiptRef?: string;
  creatorPaidMinor?: string;
  deductionRoots?: string;
  deductionTotalMinor?: string;
  residualHeldMinor?: string;
  residualAuthorityRefSql?: string;
  allocatedAt?: string;
  obligationSetDischarged?: boolean;
  allocationRoot?: string;
} = {}): string {
  const residualAuthorityRefSql = overrides.residualAuthorityRefSql ?? 'NULL';
  return `
    INSERT INTO "SettlementAllocationRecord"
      ("allocationId", "batchId", "obligationSetRoot", "eligibilityAsOf",
       "eligibilityEvidenceRoot", "beneficiaryId", "currency", "railReceiptRef",
       "creatorPaidMinor", "deductionRoots", "deductionTotalMinor", "residualHeldMinor",
       "residualAuthorityRef", "allocatedAt", "obligationSetDischarged", "allocationRoot")
    VALUES
      ('${overrides.allocationId ?? 'guard:allocation:1'}', 'batch:guard:1',
       '${overrides.obligationSetRoot ?? R.obligation}',
       '${overrides.eligibilityAsOf ?? '2026-09-10T00:03:00Z'}',
       '${overrides.eligibilityEvidenceRoot ?? R.eligibility}', 'creator:alice', 'USD',
       '${overrides.railReceiptRef ?? 'rail:receipt:guard:1'}',
       '${overrides.creatorPaidMinor ?? '450'}',
       '${overrides.deductionRoots ?? `["${R.deductionA}"]`}'::jsonb,
       '${overrides.deductionTotalMinor ?? '50'}', '${overrides.residualHeldMinor ?? '0'}',
       ${residualAuthorityRefSql}, '${overrides.allocatedAt ?? '2026-09-10T00:04:00Z'}',
       ${overrides.obligationSetDischarged ?? true}, '${overrides.allocationRoot ?? R.allocation}')
  `;
}

test('PostgreSQL protects persisted settlement allocation authority', async () => {
  assert.ok(process.env.DATABASE_URL, 'DATABASE_URL is required for the PostgreSQL allocation integration test');
  const prisma = new PrismaClient();

  try {
    const trigger = await prisma.$queryRawUnsafe<Array<{ count: number }>>(`
      SELECT COUNT(*)::int AS "count"
      FROM pg_trigger
      WHERE tgname = 'settlement_allocation_append_only_guard'
        AND NOT tgisinternal
    `);
    assert.equal(trigger[0]?.count, 1);

    const constraint = await prisma.$queryRawUnsafe<Array<{ count: number }>>(`
      SELECT COUNT(*)::int AS "count"
      FROM pg_constraint
      WHERE conname = 'settlement_allocation_canonical_shape'
    `);
    assert.equal(constraint[0]?.count, 1);

    const duplicateGrossColumn = await prisma.$queryRawUnsafe<Array<{ count: number }>>(`
      SELECT COUNT(*)::int AS "count"
      FROM information_schema.columns
      WHERE table_name = 'SettlementAllocationRecord'
        AND column_name = 'batchGrossMinor'
    `);
    assert.equal(duplicateGrossColumn[0]?.count, 0, 'allocation persistence must not duplicate batch gross authority');

    await prisma.$executeRawUnsafe(allocationInsert());

    await assert.rejects(
      prisma.$executeRawUnsafe(allocationInsert({
        allocationId: 'guard:allocation:bad-root',
        obligationSetRoot: 'not-a-digest',
        allocationRoot: R.bad,
      })),
      /settlement_allocation_canonical_shape/,
    );

    await assert.rejects(
      prisma.$executeRawUnsafe(allocationInsert({
        allocationId: 'guard:allocation:unsorted',
        deductionRoots: `["${R.deductionB}","${R.deductionA}"]`,
        deductionTotalMinor: '100',
        creatorPaidMinor: '400',
        allocationRoot: root('8'),
      })),
      /settlement_allocation_canonical_shape/,
    );

    await assert.rejects(
      prisma.$executeRawUnsafe(allocationInsert({
        allocationId: 'guard:allocation:duplicate',
        deductionRoots: `["${R.deductionA}","${R.deductionA}"]`,
        deductionTotalMinor: '100',
        creatorPaidMinor: '400',
        allocationRoot: root('9'),
      })),
      /settlement_allocation_canonical_shape/,
    );

    await assert.rejects(
      prisma.$executeRawUnsafe(allocationInsert({
        allocationId: 'guard:allocation:unexplained-deduction-total',
        deductionRoots: '[]',
        deductionTotalMinor: '50',
        allocationRoot: root('e'),
      })),
      /settlement_allocation_canonical_shape/,
    );

    await assert.rejects(
      prisma.$executeRawUnsafe(allocationInsert({
        allocationId: 'guard:allocation:zero-payment',
        creatorPaidMinor: '0',
        allocationRoot: root('a'),
      })),
      /settlement_allocation_canonical_shape/,
    );

    await assert.rejects(
      prisma.$executeRawUnsafe(allocationInsert({
        allocationId: 'guard:allocation:residual-no-authority',
        creatorPaidMinor: '425',
        residualHeldMinor: '25',
        obligationSetDischarged: false,
        allocationRoot: root('b'),
      })),
      /settlement_allocation_canonical_shape/,
    );

    await assert.rejects(
      prisma.$executeRawUnsafe(allocationInsert({
        allocationId: 'guard:allocation:false-discharge',
        obligationSetDischarged: false,
        allocationRoot: root('c'),
      })),
      /settlement_allocation_canonical_shape/,
    );

    await assert.rejects(
      prisma.$executeRawUnsafe(allocationInsert({
        allocationId: 'guard:allocation:predates',
        eligibilityAsOf: '2026-09-10T00:05:00Z',
        allocatedAt: '2026-09-10T00:04:59Z',
        allocationRoot: root('d'),
      })),
      /settlement_allocation_canonical_shape/,
    );

    await prisma.$executeRawUnsafe(allocationInsert({
      allocationId: 'guard:allocation:held',
      creatorPaidMinor: '425',
      residualHeldMinor: '25',
      residualAuthorityRefSql: "'reconciliation:case:7'",
      obligationSetDischarged: false,
      allocationRoot: R.heldAllocation,
    }));

    const held = await prisma.$queryRawUnsafe<Array<{
      residualHeldMinor: string;
      residualAuthorityRef: string;
      obligationSetDischarged: boolean;
    }>>(`
      SELECT "residualHeldMinor", "residualAuthorityRef", "obligationSetDischarged"
      FROM "SettlementAllocationRecord"
      WHERE "allocationId" = 'guard:allocation:held'
    `);
    assert.equal(held[0]?.residualHeldMinor, '25');
    assert.equal(held[0]?.residualAuthorityRef, 'reconciliation:case:7');
    assert.equal(held[0]?.obligationSetDischarged, false);

    await assert.rejects(
      prisma.$executeRawUnsafe(`
        UPDATE "SettlementAllocationRecord"
        SET "creatorPaidMinor" = '449'
        WHERE "allocationId" = 'guard:allocation:1'
      `),
      /append-only/,
    );

    await assert.rejects(
      prisma.$executeRawUnsafe(`
        DELETE FROM "SettlementAllocationRecord"
        WHERE "allocationId" = 'guard:allocation:1'
      `),
      /append-only/,
    );
  } finally {
    await prisma.$disconnect();
  }
});
