// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

/**
 * Operational source adapter for authenticated allocation-lineage checkpoints.
 *
 * This module does not create economic authority. It appends already-projected
 * successor-link evidence and captures a transactionally serialized raw source
 * view. Callers must canonically verify allocations and links in
 * @mycelix/accounting before creating/signing a checkpoint.
 */

const DIGEST_RE = /^[0-9a-f]{64}$/;
const UINT_RE = /^(0|[1-9][0-9]*)$/;
const LINEAGE_WRITER_LOCK_SQL = "SELECT lock_music_accounting_allocation_lineage_source()";
const LINEAGE_SNAPSHOT_TABLE_LOCK_SQL = `LOCK TABLE
  "SettlementAllocationRecord",
  "SettlementAllocationSuccessorLinkRecord",
  "SettlementAllocationLineageIngestRecord"
IN SHARE MODE`;

export interface SettlementAllocationSuccessorLinkRecordInput {
  readonly linkId: string;
  readonly batchId: string;
  readonly predecessorAllocationId: string;
  readonly predecessorAllocationRoot: string;
  readonly successorAllocationId: string;
  readonly successorAllocationRoot: string;
  readonly supersessionEvidenceRef: string;
  readonly linkedAt: Date | string;
  readonly linkRoot: string;
}

export interface SettlementAllocationLineageSnapshotAllocationRecord {
  readonly ingestSeq: string;
  readonly allocationId: string;
  readonly batchId: string;
  readonly obligationSetRoot: string;
  readonly eligibilityAsOf: Date;
  readonly eligibilityEvidenceRoot: string;
  readonly beneficiaryId: string;
  readonly currency: string;
  readonly railReceiptRef: string;
  readonly creatorPaidMinor: string;
  readonly deductionRoots: unknown;
  readonly deductionTotalMinor: string;
  readonly residualHeldMinor: string;
  readonly residualAuthorityRef: string | null;
  readonly allocatedAt: Date;
  readonly obligationSetDischarged: boolean;
  readonly allocationRoot: string;
}

export interface SettlementAllocationLineageSnapshotLinkRecord {
  readonly ingestSeq: string;
  readonly linkId: string;
  readonly batchId: string;
  readonly predecessorAllocationId: string;
  readonly predecessorAllocationRoot: string;
  readonly successorAllocationId: string;
  readonly successorAllocationRoot: string;
  readonly supersessionEvidenceRef: string;
  readonly linkedAt: Date;
  readonly linkRoot: string;
}

export interface SettlementAllocationLineageSourceSnapshot {
  readonly sourceRef: string;
  readonly sourceInstanceId: string;
  readonly batchId: string;
  readonly asOf: string;
  readonly observedThrough: string;
  readonly highWaterMark: string;
  readonly allocations: readonly SettlementAllocationLineageSnapshotAllocationRecord[];
  readonly links: readonly SettlementAllocationLineageSnapshotLinkRecord[];
}

interface RawTransactionClient {
  $queryRawUnsafe<T = unknown>(query: string, ...values: unknown[]): Promise<T>;
  $executeRawUnsafe(query: string, ...values: unknown[]): Promise<number>;
}

export interface AccountingLineageSourcePrismaClient {
  $transaction<T>(fn: (tx: RawTransactionClient) => Promise<T>): Promise<T>;
}

function required(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

function digest(label: string, value: string): string {
  const normalized = required(label, value);
  if (!DIGEST_RE.test(normalized)) throw new Error(`${label} must be a lowercase SHA-256 digest`);
  return normalized;
}

function timestamp(label: string, value: Date | string): Date {
  const date = value instanceof Date ? new Date(value.getTime()) : new Date(value);
  if (!Number.isFinite(date.getTime())) throw new Error(`${label} must be a valid timestamp`);
  return date;
}

function canonicalUint(label: string, value: string): string {
  const normalized = value.trim();
  if (!UINT_RE.test(normalized)) throw new Error(`${label} must be a canonical unsigned integer`);
  return normalized;
}

function canonical(value: unknown): string {
  if (value === null) return 'null';
  if (value instanceof Date) return `date:${value.toISOString()}`;
  if (typeof value === 'string') return `string:${JSON.stringify(value)}`;
  if (typeof value === 'boolean') return value ? 'true' : 'false';
  if (typeof value === 'number') {
    if (!Number.isFinite(value)) throw new Error('lineage source comparison requires finite numbers');
    return `number:${value}`;
  }
  if (typeof value === 'bigint') return `bigint:${value.toString(10)}`;
  if (Array.isArray(value)) return `[${value.map(canonical).join(',')}]`;
  if (typeof value === 'object') {
    const record = value as Record<string, unknown>;
    return `{${Object.keys(record).sort().map(key => {
      if (record[key] === undefined) throw new Error('lineage source comparison forbids undefined');
      return `${JSON.stringify(key)}:${canonical(record[key])}`;
    }).join(',')}}`;
  }
  throw new Error(`unsupported lineage source comparison value: ${typeof value}`);
}

function normalizeLink(input: SettlementAllocationSuccessorLinkRecordInput): Readonly<Record<string, unknown>> {
  return Object.freeze({
    linkId: required('allocation successor linkId', input.linkId),
    batchId: required('allocation successor batchId', input.batchId),
    predecessorAllocationId: required('allocation successor predecessorAllocationId', input.predecessorAllocationId),
    predecessorAllocationRoot: digest('allocation successor predecessorAllocationRoot', input.predecessorAllocationRoot),
    successorAllocationId: required('allocation successor successorAllocationId', input.successorAllocationId),
    successorAllocationRoot: digest('allocation successor successorAllocationRoot', input.successorAllocationRoot),
    supersessionEvidenceRef: required('allocation successor supersessionEvidenceRef', input.supersessionEvidenceRef),
    linkedAt: timestamp('allocation successor linkedAt', input.linkedAt),
    linkRoot: digest('allocation successor linkRoot', input.linkRoot),
  });
}

function assertExactLink(existing: Readonly<Record<string, unknown>>, expected: Readonly<Record<string, unknown>>): void {
  for (const [field, expectedValue] of Object.entries(expected)) {
    if (canonical(existing[field]) !== canonical(expectedValue)) {
      throw new Error(`settlement allocation successor link ${String(expected.linkId)} already exists with different immutable content at ${field}`);
    }
  }
}

export class AccountingAllocationLineageSource {
  private readonly sourceRef: string;
  private readonly sourceInstanceId: string;

  constructor(
    private readonly client: AccountingLineageSourcePrismaClient,
    sourceIdentity: { readonly sourceRef: string; readonly sourceInstanceId: string },
  ) {
    this.sourceRef = required('allocation lineage sourceRef', sourceIdentity.sourceRef);
    this.sourceInstanceId = required('allocation lineage sourceInstanceId', sourceIdentity.sourceInstanceId);
  }

  async appendSuccessorLink(input: SettlementAllocationSuccessorLinkRecordInput): Promise<void> {
    const expected = normalizeLink(input);
    await this.client.$transaction(async tx => {
      await tx.$queryRawUnsafe(LINEAGE_WRITER_LOCK_SQL);
      await tx.$executeRawUnsafe(
        `INSERT INTO "SettlementAllocationSuccessorLinkRecord"
          ("linkId", "batchId", "predecessorAllocationId", "predecessorAllocationRoot",
           "successorAllocationId", "successorAllocationRoot", "supersessionEvidenceRef", "linkedAt", "linkRoot")
         VALUES ($1,$2,$3,$4,$5,$6,$7,$8,$9)
         ON CONFLICT ("linkId") DO NOTHING`,
        expected.linkId, expected.batchId, expected.predecessorAllocationId, expected.predecessorAllocationRoot,
        expected.successorAllocationId, expected.successorAllocationRoot, expected.supersessionEvidenceRef,
        expected.linkedAt, expected.linkRoot,
      );
      const rows = await tx.$queryRawUnsafe<Array<Record<string, unknown>>>(
        `SELECT "linkId", "batchId", "predecessorAllocationId", "predecessorAllocationRoot",
                "successorAllocationId", "successorAllocationRoot", "supersessionEvidenceRef", "linkedAt", "linkRoot"
           FROM "SettlementAllocationSuccessorLinkRecord"
          WHERE "linkId" = $1`,
        expected.linkId,
      );
      if (rows.length !== 1) throw new Error('settlement allocation successor link insert did not produce exactly one durable row');
      const row = rows[0]!;
      const normalized = {
        ...row,
        linkedAt: timestamp('persisted allocation successor linkedAt', row.linkedAt as Date | string),
      };
      assertExactLink(normalized, expected);
    });
  }

  async captureSnapshot(input: {
    readonly batchId: string;
    readonly asOf: Date | string;
  }): Promise<Readonly<SettlementAllocationLineageSourceSnapshot>> {
    const batchId = required('allocation lineage snapshot batchId', input.batchId);
    const asOf = timestamp('allocation lineage snapshot asOf', input.asOf);

    return this.client.$transaction(async tx => {
      // PostgreSQL acquires ROW EXCLUSIVE before firing INSERT triggers. Snapshot
      // capture therefore freezes all source tables *before* waiting on the
      // advisory writer lock; reversing this order can deadlock with a writer
      // that already owns ROW EXCLUSIVE and is waiting inside its trigger.
      await tx.$executeRawUnsafe(LINEAGE_SNAPSHOT_TABLE_LOCK_SQL);
      await tx.$queryRawUnsafe(LINEAGE_WRITER_LOCK_SQL);

      const clockRows = await tx.$queryRawUnsafe<Array<{ observedThrough: Date | string }>>(
        `SELECT clock_timestamp()::text AS "observedThrough"`,
      );
      const observedThrough = timestamp('allocation lineage snapshot source clock', clockRows[0]?.observedThrough ?? '');
      if (asOf.getTime() > observedThrough.getTime()) {
        throw new Error('allocation lineage snapshot asOf cannot be later than serialized source observation time');
      }

      const highWaterRows = await tx.$queryRawUnsafe<Array<{ highWaterMark: string }>>(
        `SELECT COALESCE(MAX("ingestSeq"), 0)::text AS "highWaterMark"
           FROM "SettlementAllocationLineageIngestRecord"`,
      );
      const highWaterMark = canonicalUint('allocation lineage snapshot highWaterMark', highWaterRows[0]?.highWaterMark ?? '');

      const missingRows = await tx.$queryRawUnsafe<Array<{ missing: number }>>(
        `SELECT (
           SELECT COUNT(*)::int
             FROM "SettlementAllocationRecord" a
             LEFT JOIN "SettlementAllocationLineageIngestRecord" r
               ON r."recordKind" = 'allocation'
              AND r."recordId" = a."allocationId"
              AND r."batchId" = a."batchId"
              AND r."recordRoot" = a."allocationRoot"
              AND r."evidenceAt" = a."allocatedAt"
            WHERE a."batchId" = $1
              AND a."allocatedAt" <= $2::timestamp
              AND r."ingestSeq" IS NULL
         ) + (
           SELECT COUNT(*)::int
             FROM "SettlementAllocationSuccessorLinkRecord" l
             LEFT JOIN "SettlementAllocationLineageIngestRecord" r
               ON r."recordKind" = 'successor_link'
              AND r."recordId" = l."linkId"
              AND r."batchId" = l."batchId"
              AND r."recordRoot" = l."linkRoot"
              AND r."evidenceAt" = l."linkedAt"
            WHERE l."batchId" = $1
              AND l."linkedAt" <= $2::timestamp
              AND r."ingestSeq" IS NULL
         ) AS "missing"`,
        batchId,
        asOf,
      );
      if ((missingRows[0]?.missing ?? -1) !== 0) {
        throw new Error('allocation lineage source snapshot contains unregistered eligible evidence');
      }

      const allocations = await tx.$queryRawUnsafe<SettlementAllocationLineageSnapshotAllocationRecord[]>(
        `SELECT r."ingestSeq"::text AS "ingestSeq",
                a."allocationId", a."batchId", a."obligationSetRoot", a."eligibilityAsOf",
                a."eligibilityEvidenceRoot", a."beneficiaryId", a."currency", a."railReceiptRef",
                a."creatorPaidMinor", a."deductionRoots", a."deductionTotalMinor", a."residualHeldMinor",
                a."residualAuthorityRef", a."allocatedAt", a."obligationSetDischarged", a."allocationRoot"
           FROM "SettlementAllocationRecord" a
           JOIN "SettlementAllocationLineageIngestRecord" r
             ON r."recordKind" = 'allocation'
            AND r."recordId" = a."allocationId"
            AND r."batchId" = a."batchId"
            AND r."recordRoot" = a."allocationRoot"
            AND r."evidenceAt" = a."allocatedAt"
          WHERE a."batchId" = $1
            AND a."allocatedAt" <= $2::timestamp
            AND r."ingestSeq" <= $3::bigint
          ORDER BY r."ingestSeq" ASC`,
        batchId,
        asOf,
        highWaterMark,
      );
      const links = await tx.$queryRawUnsafe<SettlementAllocationLineageSnapshotLinkRecord[]>(
        `SELECT r."ingestSeq"::text AS "ingestSeq",
                l."linkId", l."batchId", l."predecessorAllocationId", l."predecessorAllocationRoot",
                l."successorAllocationId", l."successorAllocationRoot", l."supersessionEvidenceRef",
                l."linkedAt", l."linkRoot"
           FROM "SettlementAllocationSuccessorLinkRecord" l
           JOIN "SettlementAllocationLineageIngestRecord" r
             ON r."recordKind" = 'successor_link'
            AND r."recordId" = l."linkId"
            AND r."batchId" = l."batchId"
            AND r."recordRoot" = l."linkRoot"
            AND r."evidenceAt" = l."linkedAt"
          WHERE l."batchId" = $1
            AND l."linkedAt" <= $2::timestamp
            AND r."ingestSeq" <= $3::bigint
          ORDER BY r."ingestSeq" ASC`,
        batchId,
        asOf,
        highWaterMark,
      );

      const normalizedAllocations = allocations.map(record => Object.freeze({
        ...record,
        eligibilityAsOf: timestamp('allocation lineage snapshot allocation eligibilityAsOf', record.eligibilityAsOf),
        allocatedAt: timestamp('allocation lineage snapshot allocation allocatedAt', record.allocatedAt),
      }));
      const normalizedLinks = links.map(record => Object.freeze({
        ...record,
        linkedAt: timestamp('allocation lineage snapshot successor linkedAt', record.linkedAt),
      }));
      for (const record of [...normalizedAllocations, ...normalizedLinks]) {
        canonicalUint('allocation lineage snapshot ingestSeq', record.ingestSeq);
      }
      return Object.freeze({
        sourceRef: this.sourceRef,
        sourceInstanceId: this.sourceInstanceId,
        batchId,
        asOf: asOf.toISOString(),
        observedThrough: observedThrough.toISOString(),
        highWaterMark,
        allocations: Object.freeze(normalizedAllocations),
        links: Object.freeze(normalizedLinks),
      });
    });
  }
}
