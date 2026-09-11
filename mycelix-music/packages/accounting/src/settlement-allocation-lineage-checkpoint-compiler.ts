import type { RoyaltyDeductionAuthority } from './deduction-authority.js';
import type { DeterministicNettingBatch } from './netting.js';
import {
  verifyPersistedSettlementAllocation,
  type VerifiablePersistedSettlementAllocationRecord,
} from './persistence-verification.js';
import type { SettlementRecoveryState } from './recovery.js';
import {
  createSettlementAllocationLineageCheckpoint,
  createSettlementAllocationLineageCheckpointSigningRequest,
  type CreateSettlementAllocationLineageCheckpointSigningRequestInput,
  type SettlementAllocationLineageCheckpoint,
  type SettlementAllocationLineageCheckpointSigningCapability,
  type SettlementAllocationLineageCheckpointSigningRequest,
} from './settlement-allocation-lineage-checkpoint.js';
import {
  verifyPersistedSettlementAllocationSuccessorLink,
  type VerifiablePersistedSettlementAllocationSuccessorLinkRecord,
} from './settlement-allocation-lineage-persistence.js';
import type { SettlementAllocationSuccessorLink } from './settlement-allocation-lineage.js';
import type { SettlementAllocationAuthority } from './settlement-allocation.js';

const CANONICAL_UINT = /^(0|[1-9][0-9]*)$/;
type DateLike = string | Date;

export type PersistedSettlementAllocationLineageSourceAllocationRecord =
  Omit<VerifiablePersistedSettlementAllocationRecord, 'residualAuthorityRef'> & {
    readonly ingestSeq: string;
    readonly residualAuthorityRef?: string | null;
  };

export type PersistedSettlementAllocationLineageSourceLinkRecord =
  VerifiablePersistedSettlementAllocationSuccessorLinkRecord & {
    readonly ingestSeq: string;
  };

/** Portable shape emitted by the serialized database source adapter. */
export interface PersistedSettlementAllocationLineageSourceSnapshot {
  readonly sourceRef: string;
  readonly sourceInstanceId: string;
  readonly batchId: string;
  readonly asOf: DateLike;
  readonly observedThrough: DateLike;
  readonly highWaterMark: string;
  readonly allocations: readonly PersistedSettlementAllocationLineageSourceAllocationRecord[];
  readonly links: readonly PersistedSettlementAllocationLineageSourceLinkRecord[];
}

/** Exact semantic inputs needed to replay one persisted allocation authority. */
export interface SettlementAllocationReplayContext {
  readonly recovery: SettlementRecoveryState;
  readonly deductions: readonly RoyaltyDeductionAuthority[];
}

export interface CompileSettlementAllocationLineageCheckpointInput {
  readonly checkpointId: string;
  readonly snapshot: PersistedSettlementAllocationLineageSourceSnapshot;
  readonly batch: DeterministicNettingBatch;
  readonly allocationContexts: Readonly<Record<string, SettlementAllocationReplayContext>>;
}

export interface CompiledSettlementAllocationLineageCheckpoint {
  readonly checkpoint: SettlementAllocationLineageCheckpoint;
  readonly allocations: readonly SettlementAllocationAuthority[];
  readonly links: readonly SettlementAllocationSuccessorLink[];
}

const COMPILED_CHECKPOINTS = new WeakSet<object>();

function required(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

function canonicalTimestamp(label: string, value: DateLike): string {
  const date = value instanceof Date ? new Date(value.getTime()) : new Date(value);
  if (!Number.isFinite(date.getTime())) throw new Error(`${label} must be a valid timestamp`);
  return date.toISOString();
}

function canonicalUint(label: string, value: string): bigint {
  if (value !== value.trim() || !CANONICAL_UINT.test(value)) {
    throw new Error(`${label} must be canonical unsigned-integer text`);
  }
  return BigInt(value);
}

function validateSourceOrder(
  label: string,
  records: readonly { readonly ingestSeq: string }[],
  highWater: bigint,
  seen: Set<string>,
): void {
  let previous: bigint | undefined;
  for (const [index, record] of records.entries()) {
    const raw = record.ingestSeq;
    const sequence = canonicalUint(`${label}[${index}] ingestSeq`, raw);
    if (sequence <= 0n) throw new Error(`${label} ingestion cursor must be positive`);
    if (sequence > highWater) throw new Error(`${label} ingestion cursor exceeds snapshot highWaterMark`);
    if (previous !== undefined && sequence <= previous) {
      throw new Error(`${label} must be ordered by strictly increasing ingestion cursor`);
    }
    if (seen.has(raw)) throw new Error(`duplicate allocation-lineage ingestion cursor: ${raw}`);
    seen.add(raw);
    previous = sequence;
  }
}

function withoutAllocationIngest(
  record: PersistedSettlementAllocationLineageSourceAllocationRecord,
): VerifiablePersistedSettlementAllocationRecord {
  const { ingestSeq: _ingestSeq, residualAuthorityRef, ...rest } = record;
  return {
    ...rest,
    ...(residualAuthorityRef == null ? {} : { residualAuthorityRef }),
  };
}

function withoutLinkIngest(
  record: PersistedSettlementAllocationLineageSourceLinkRecord,
): VerifiablePersistedSettlementAllocationSuccessorLinkRecord {
  const { ingestSeq: _ingestSeq, ...rest } = record;
  return rest;
}

/**
 * Replay a serialized source snapshot into canonical accounting authorities and
 * only then construct a checkpoint candidate. This function proves content and
 * cursor coherence; source origin remains the responsibility of the serialized
 * source capture boundary that supplied the snapshot.
 */
export function compileSettlementAllocationLineageCheckpoint(
  input: CompileSettlementAllocationLineageCheckpointInput,
): Readonly<CompiledSettlementAllocationLineageCheckpoint> {
  const checkpointId = required('compiled checkpointId', input.checkpointId);
  const sourceRef = required('lineage sourceRef', input.snapshot.sourceRef);
  const sourceInstanceId = required('lineage sourceInstanceId', input.snapshot.sourceInstanceId);
  const snapshotBatchId = required('lineage snapshot batchId', input.snapshot.batchId);
  if (snapshotBatchId !== input.batch.batchId) throw new Error('lineage source snapshot batch does not match deterministic batch');

  const asOf = canonicalTimestamp('lineage source snapshot asOf', input.snapshot.asOf);
  const observedThrough = canonicalTimestamp('lineage source snapshot observedThrough', input.snapshot.observedThrough);
  if (Date.parse(observedThrough) < Date.parse(asOf)) {
    throw new Error('lineage source snapshot must be observed through asOf');
  }
  const highWater = canonicalUint('lineage source snapshot highWaterMark', input.snapshot.highWaterMark);
  if (input.snapshot.allocations.length === 0) throw new Error('lineage source snapshot requires at least one allocation');
  if (highWater <= 0n) throw new Error('non-empty lineage source snapshot requires a positive highWaterMark');

  const seenSequences = new Set<string>();
  validateSourceOrder('lineage source allocations', input.snapshot.allocations, highWater, seenSequences);
  validateSourceOrder('lineage source successor links', input.snapshot.links, highWater, seenSequences);

  const verifiedAllocations: SettlementAllocationAuthority[] = [];
  const allocationsById = new Map<string, SettlementAllocationAuthority>();
  const allocationSequenceById = new Map<string, bigint>();
  for (const record of input.snapshot.allocations) {
    if (record.batchId !== snapshotBatchId) throw new Error('persisted allocation belongs to a different source batch');
    const allocatedAt = canonicalTimestamp('lineage source allocation allocatedAt', record.allocatedAt);
    if (Date.parse(allocatedAt) > Date.parse(asOf)) throw new Error('lineage source allocation is later than snapshot asOf');
    if (allocationsById.has(record.allocationId)) throw new Error(`duplicate lineage source allocation id: ${record.allocationId}`);

    const context = input.allocationContexts[record.allocationId];
    if (context === undefined) throw new Error(`missing replay context for allocation: ${record.allocationId}`);
    const allocation = verifyPersistedSettlementAllocation(
      withoutAllocationIngest(record),
      input.batch,
      context.recovery,
      context.deductions,
    );
    if (allocation.batchId !== snapshotBatchId) throw new Error('verified allocation belongs to a different source batch');
    verifiedAllocations.push(allocation);
    allocationsById.set(allocation.allocationId, allocation);
    allocationSequenceById.set(allocation.allocationId, canonicalUint('allocation ingestSeq', record.ingestSeq));
  }

  const verifiedLinks: SettlementAllocationSuccessorLink[] = [];
  const linkIds = new Set<string>();
  for (const record of input.snapshot.links) {
    if (record.batchId !== snapshotBatchId) throw new Error('persisted successor link belongs to a different source batch');
    if (linkIds.has(record.linkId)) throw new Error(`duplicate lineage source successor link id: ${record.linkId}`);
    linkIds.add(record.linkId);
    const linkedAt = canonicalTimestamp('lineage source successor linkedAt', record.linkedAt);
    if (Date.parse(linkedAt) > Date.parse(asOf)) throw new Error('lineage source successor link is later than snapshot asOf');

    const predecessor = allocationsById.get(record.predecessorAllocationId);
    const successor = allocationsById.get(record.successorAllocationId);
    if (!predecessor || !successor) throw new Error('persisted successor link references allocation omitted from source snapshot');
    const linkSequence = canonicalUint('successor link ingestSeq', record.ingestSeq);
    const predecessorSequence = allocationSequenceById.get(predecessor.allocationId)!;
    const successorSequence = allocationSequenceById.get(successor.allocationId)!;
    if (linkSequence <= predecessorSequence || linkSequence <= successorSequence) {
      throw new Error('successor link ingestion cursor must follow both endpoint allocations');
    }

    verifiedLinks.push(verifyPersistedSettlementAllocationSuccessorLink(
      withoutLinkIngest(record),
      predecessor,
      successor,
    ));
  }

  const checkpoint = createSettlementAllocationLineageCheckpoint({
    checkpointId,
    sourceRef,
    sourceInstanceId,
    batchId: snapshotBatchId,
    asOf,
    observedThrough,
    highWaterMark: highWater.toString(10),
    allocations: verifiedAllocations,
    links: verifiedLinks,
  });

  const compiled = Object.freeze({
    checkpoint,
    allocations: Object.freeze([...verifiedAllocations]),
    links: Object.freeze([...verifiedLinks]),
  });
  COMPILED_CHECKPOINTS.add(compiled);
  return compiled;
}

/**
 * Operational handoff boundary. The compiler may mint a scoped signing request
 * for a canonical candidate, but it never receives or invokes a private key.
 */
export function createCompiledSettlementAllocationLineageCheckpointSigningRequest(
  compiled: CompiledSettlementAllocationLineageCheckpoint,
  capability: SettlementAllocationLineageCheckpointSigningCapability,
  input: CreateSettlementAllocationLineageCheckpointSigningRequestInput,
): Readonly<SettlementAllocationLineageCheckpointSigningRequest> {
  if (!COMPILED_CHECKPOINTS.has(compiled as object)) {
    throw new Error('settlement allocation checkpoint must be produced by canonical source compiler before requesting signature');
  }
  return createSettlementAllocationLineageCheckpointSigningRequest(compiled.checkpoint, capability, input);
}
