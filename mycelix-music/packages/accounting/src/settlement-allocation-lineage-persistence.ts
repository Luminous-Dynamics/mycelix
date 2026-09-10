import { canonicalAccountingValue } from './merkle.js';
import {
  assertSettlementAllocationSuccessorLink,
  createSettlementAllocationSuccessorLink,
  type SettlementAllocationSuccessorLink,
} from './settlement-allocation-lineage.js';
import type { SettlementAllocationAuthority } from './settlement-allocation.js';

export interface PersistedSettlementAllocationSuccessorLinkRecord {
  readonly linkId: string;
  readonly batchId: string;
  readonly predecessorAllocationId: string;
  readonly predecessorAllocationRoot: string;
  readonly successorAllocationId: string;
  readonly successorAllocationRoot: string;
  readonly supersessionEvidenceRef: string;
  readonly linkedAt: string;
  readonly linkRoot: string;
}

type DateLike = string | Date;

export type VerifiablePersistedSettlementAllocationSuccessorLinkRecord =
  Omit<PersistedSettlementAllocationSuccessorLinkRecord, 'linkedAt'> & {
    readonly linkedAt: DateLike;
  };

function timestamp(label: string, value: DateLike): string {
  const date = value instanceof Date ? new Date(value.getTime()) : new Date(value);
  if (!Number.isFinite(date.getTime())) throw new Error(`${label} must be a valid timestamp`);
  return date.toISOString();
}

function sameCanonicalValue(left: unknown, right: unknown): boolean {
  if (left === undefined || right === undefined) return left === right;
  return canonicalAccountingValue(left) === canonicalAccountingValue(right);
}

function assertProjectedFields(
  actual: Readonly<Record<string, unknown>>,
  expected: Readonly<Record<string, unknown>>,
): void {
  for (const [field, expectedValue] of Object.entries(expected)) {
    if (!sameCanonicalValue(actual[field], expectedValue)) {
      throw new Error(`persisted settlement allocation successor link does not match canonical projection at ${field}`);
    }
  }
}

export function projectSettlementAllocationSuccessorLinkRecord(
  link: SettlementAllocationSuccessorLink,
  predecessor: SettlementAllocationAuthority,
  successor: SettlementAllocationAuthority,
): Readonly<PersistedSettlementAllocationSuccessorLinkRecord> {
  assertSettlementAllocationSuccessorLink(link, predecessor, successor);
  return Object.freeze({
    linkId: link.linkId,
    batchId: link.batchId,
    predecessorAllocationId: link.predecessorAllocationId,
    predecessorAllocationRoot: link.predecessorAllocationRoot,
    successorAllocationId: link.successorAllocationId,
    successorAllocationRoot: link.successorAllocationRoot,
    supersessionEvidenceRef: link.supersessionEvidenceRef,
    linkedAt: link.linkedAt,
    linkRoot: link.linkRoot,
  });
}

export function verifyPersistedSettlementAllocationSuccessorLink(
  record: VerifiablePersistedSettlementAllocationSuccessorLinkRecord,
  predecessor: SettlementAllocationAuthority,
  successor: SettlementAllocationAuthority,
): Readonly<SettlementAllocationSuccessorLink> {
  const linkedAt = timestamp('persisted settlement allocation successor linkedAt', record.linkedAt);
  const link = createSettlementAllocationSuccessorLink({
    linkId: record.linkId,
    predecessor,
    successor,
    supersessionEvidenceRef: record.supersessionEvidenceRef,
    linkedAt,
  });
  const expected = projectSettlementAllocationSuccessorLinkRecord(link, predecessor, successor);
  const normalized = { ...record, linkedAt } as Readonly<Record<string, unknown>>;
  assertProjectedFields(normalized, expected as unknown as Readonly<Record<string, unknown>>);
  return link;
}
