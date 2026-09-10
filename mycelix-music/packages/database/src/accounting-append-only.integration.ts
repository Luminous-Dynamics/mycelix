import assert from 'node:assert/strict';
import test from 'node:test';
import { PrismaClient } from '@prisma/client';

const GUARDED_TABLES = [
  'RoyaltyDeductionRecord',
  'RoyaltyEligibilityObservation',
  'RoyaltyObligationRecord',
  'RoyaltyStatementSnapshotRecord',
  'SettlementAttemptObservationRecord',
] as const;

const ROOT = {
  obligation: '1'.repeat(64), eligibility: '2'.repeat(64), deduction: '3'.repeat(64),
  settlement: '4'.repeat(64), settlementEligibility: '9'.repeat(64),
  invalidEligibility: 'a'.repeat(64), invalidSettlement: 'b'.repeat(64),
  statementObligation: '5'.repeat(64), statementAdjustment: '6'.repeat(64),
  statementSettlement: '7'.repeat(64), statementSnapshot: '8'.repeat(64),
} as const;

test('PostgreSQL enforces creator accounting authority as append-only and causally valid', async () => {
  assert.ok(process.env.DATABASE_URL, 'DATABASE_URL is required for the PostgreSQL authority integration test');
  const prisma = new PrismaClient();

  try {
    const triggers = await prisma.$queryRawUnsafe<Array<{ tableName: string }>>(`
      SELECT c.relname AS "tableName"
      FROM pg_trigger AS t
      JOIN pg_class AS c ON c.oid = t.tgrelid
      WHERE t.tgname = 'accounting_append_only_guard'
        AND NOT t.tgisinternal
      ORDER BY c.relname
    `);
    assert.deepEqual(triggers.map(row => row.tableName), [...GUARDED_TABLES]);

    const constraints = await prisma.$queryRawUnsafe<Array<{ constraintName: string }>>(`
      SELECT conname AS "constraintName"
      FROM pg_constraint
      WHERE conname IN (
        'royalty_eligibility_observable_code',
        'settlement_observation_after_eligibility'
      )
      ORDER BY conname
    `);
    assert.deepEqual(constraints.map(row => row.constraintName), [
      'royalty_eligibility_observable_code',
      'settlement_observation_after_eligibility',
    ]);

    await prisma.$executeRawUnsafe(`
      INSERT INTO "RoyaltyObligationRecord"
        ("id", "beneficiaryId", "amountMinor", "currency", "observedAt",
         "usageEvidenceRef", "rightsResolutionRef", "economicTermsRef", "obligationRoot")
      VALUES
        ('guard:obligation:1', 'creator:alice', '500', 'USD', '2026-09-10T00:00:00Z',
         'usage:epoch:1', 'rights:resolution:1', 'terms:v1', '${ROOT.obligation}')
    `);

    await prisma.$executeRawUnsafe(`
      INSERT INTO "RoyaltyEligibilityObservation"
        ("id", "obligationId", "code", "reason", "sourceRef", "observedAt", "observationRoot")
      VALUES
        ('guard:eligibility:1', 'guard:obligation:1', 'awaiting_payee_route',
         'route unavailable', 'route-registry:v1', '2026-09-10T00:01:00Z', '${ROOT.eligibility}')
    `);

    await assert.rejects(
      prisma.$executeRawUnsafe(`
        INSERT INTO "RoyaltyEligibilityObservation"
          ("id", "obligationId", "code", "sourceRef", "observedAt", "observationRoot")
        VALUES
          ('guard:eligibility:compiler-only', 'guard:obligation:1', 'below_threshold',
           'compiler:forged', '2026-09-10T00:01:30Z', '${ROOT.invalidEligibility}')
      `),
      /royalty_eligibility_observable_code/,
      'direct SQL must not persist compiler-only eligibility state',
    );

    await prisma.$executeRawUnsafe(`
      INSERT INTO "RoyaltyDeductionRecord"
        ("id", "beneficiaryId", "amountMinor", "currency", "basis", "authorityRef", "observedAt", "deductionRoot")
      VALUES
        ('guard:deduction:1', 'creator:alice', '50', 'USD', 'tax:withholding:v1',
         'tax-authority:notice:1', '2026-09-10T00:02:00Z', '${ROOT.deduction}')
    `);

    await prisma.$executeRawUnsafe(`
      INSERT INTO "SettlementAttemptObservationRecord"
        ("id", "attemptId", "batchId", "obligationSetRoot", "eligibilityAsOf",
         "eligibilityEvidenceRoot", "state", "observedAt", "observationRoot")
      VALUES
        ('guard:settlement:1', 'attempt:1', 'batch:1', '${ROOT.obligation}',
         '2026-09-10T00:02:30Z', '${ROOT.settlementEligibility}', 'submitted',
         '2026-09-10T00:03:00Z', '${ROOT.settlement}')
    `);

    await assert.rejects(
      prisma.$executeRawUnsafe(`
        INSERT INTO "SettlementAttemptObservationRecord"
          ("id", "attemptId", "batchId", "obligationSetRoot", "eligibilityAsOf",
           "eligibilityEvidenceRoot", "state", "observedAt", "observationRoot")
        VALUES
          ('guard:settlement:pre-snapshot', 'attempt:2', 'batch:2', '${ROOT.obligation}',
           '2026-09-10T00:05:00Z', '${ROOT.settlementEligibility}', 'submitted',
           '2026-09-10T00:04:59Z', '${ROOT.invalidSettlement}')
      `),
      /settlement_observation_after_eligibility/,
      'direct SQL must not persist settlement evidence before its eligibility snapshot',
    );

    const settlementSnapshot = await prisma.$queryRawUnsafe<Array<{ eligibilityAsOf: Date; eligibilityEvidenceRoot: string }>>(`
      SELECT "eligibilityAsOf", "eligibilityEvidenceRoot"
      FROM "SettlementAttemptObservationRecord"
      WHERE "id" = 'guard:settlement:1'
    `);
    assert.equal(settlementSnapshot[0]?.eligibilityAsOf.toISOString(), '2026-09-10T00:02:30.000Z');
    assert.equal(settlementSnapshot[0]?.eligibilityEvidenceRoot, ROOT.settlementEligibility);

    await prisma.$executeRawUnsafe(`
      INSERT INTO "RoyaltyStatementSnapshotRecord"
        ("statementId", "kind", "beneficiaryId", "periodStart", "periodEnd", "asOf",
         "obligationRoot", "adjustmentRoot", "settlementRoot", "grossMinor", "heldMinor",
         "deductionMinor", "netPayableMinor", "paidMinor", "currency", "completenessKind",
         "completenessData", "snapshotRoot")
      VALUES
        ('guard:statement:1', 'periodic', 'creator:alice', '2026-09-01T00:00:00Z',
         '2026-10-01T00:00:00Z', '2026-10-02T00:00:00Z', '${ROOT.statementObligation}',
         '${ROOT.statementAdjustment}', '${ROOT.statementSettlement}', '500', '100', '50',
         '350', '300', 'USD', 'complete', '{"kind":"complete"}'::jsonb, '${ROOT.statementSnapshot}')
    `);

    const mutationTargets = [
      { table: 'RoyaltyObligationRecord', key: 'id', value: 'guard:obligation:1' },
      { table: 'RoyaltyEligibilityObservation', key: 'id', value: 'guard:eligibility:1' },
      { table: 'RoyaltyDeductionRecord', key: 'id', value: 'guard:deduction:1' },
      { table: 'SettlementAttemptObservationRecord', key: 'id', value: 'guard:settlement:1' },
      { table: 'RoyaltyStatementSnapshotRecord', key: 'statementId', value: 'guard:statement:1' },
    ] as const;

    for (const target of mutationTargets) {
      await assert.rejects(
        prisma.$executeRawUnsafe(`UPDATE "${target.table}" SET "createdAt" = "createdAt" WHERE "${target.key}" = '${target.value}'`),
        /append-only.*UPDATE is forbidden/s,
        `${target.table} UPDATE must be rejected by the PostgreSQL trigger`,
      );
      await assert.rejects(
        prisma.$executeRawUnsafe(`DELETE FROM "${target.table}" WHERE "${target.key}" = '${target.value}'`),
        /append-only.*DELETE is forbidden/s,
        `${target.table} DELETE must be rejected by the PostgreSQL trigger`,
      );
    }

    for (const target of mutationTargets) {
      const rows = await prisma.$queryRawUnsafe<Array<{ count: number }>>(
        `SELECT COUNT(*)::int AS "count" FROM "${target.table}" WHERE "${target.key}" = '${target.value}'`,
      );
      assert.equal(rows[0]?.count, 1, `${target.table} append must remain present after rejected mutations`);
    }
  } finally {
    await prisma.$disconnect();
  }
});
