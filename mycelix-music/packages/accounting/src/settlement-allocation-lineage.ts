import { buildMerkleCommitment, type Digest } from './merkle.js';
import type { SettlementAllocationAuthority } from './settlement-allocation.js';

export interface SettlementAllocationSuccessorLink {
  readonly linkId: string;
  readonly batchId: string;
  readonly predecessorAllocationId: string;
  readonly predecessorAllocationRoot: Digest;
  readonly successorAllocationId: string;
  readonly successorAllocationRoot: Digest;
  /** Evidence explaining why the successor allocation is authoritative. */
  readonly supersessionEvidenceRef: string;
  readonly linkedAt: string;
  readonly linkRoot: Digest;
}

export interface CreateSettlementAllocationSuccessorLinkInput {
  readonly linkId: string;
  readonly predecessor: SettlementAllocationAuthority;
  readonly successor: SettlementAllocationAuthority;
  readonly supersessionEvidenceRef: string;
  readonly linkedAt: string;
}

export interface SettlementAllocationLineageResolution {
  readonly batchId: string;
  readonly initialAllocationRoot: Digest;
  readonly headAllocationRoot: Digest;
  readonly headAllocation: SettlementAllocationAuthority;
  readonly allocationRoots: readonly Digest[];
  readonly linkRoots: readonly Digest[];
  readonly depth: number;
  readonly lineageRoot: Digest;
}

function required(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

function timestamp(label: string, value: string): string {
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error(`${label} must be a valid timestamp`);
  return new Date(parsed).toISOString();
}

function sameBatchIdentity(
  predecessor: SettlementAllocationAuthority,
  successor: SettlementAllocationAuthority,
): boolean {
  return predecessor.batchId === successor.batchId
    && predecessor.obligationSetRoot === successor.obligationSetRoot
    && predecessor.eligibilityAsOf === successor.eligibilityAsOf
    && predecessor.eligibilityEvidenceRoot === successor.eligibilityEvidenceRoot
    && predecessor.beneficiaryId === successor.beneficiaryId
    && predecessor.currency === successor.currency;
}

function rootsAreSubset(previous: readonly Digest[], next: readonly Digest[]): boolean {
  const nextRoots = new Set(next);
  return previous.every(root => nextRoots.has(root));
}

/**
 * Prove that a successor allocation is monotonic economic progress for exactly
 * the same deterministic settlement batch. This is deliberately stricter than
 * "newer timestamp wins": previously-accounted value may not disappear or be
 * rebound to different evidence.
 */
export function assertSettlementAllocationLineageStep(
  predecessor: SettlementAllocationAuthority,
  successor: SettlementAllocationAuthority,
): void {
  if (predecessor.allocationRoot === successor.allocationRoot) {
    throw new Error('settlement allocation successor must have a distinct allocation root');
  }
  if (predecessor.allocationId === successor.allocationId) {
    throw new Error('settlement allocation successor must have a distinct allocation id');
  }
  if (!sameBatchIdentity(predecessor, successor)) {
    throw new Error('settlement allocation successor must preserve exact batch and eligibility identity');
  }
  if (predecessor.obligationSetDischarged) {
    throw new Error('a discharged settlement allocation cannot be superseded');
  }

  const predecessorTime = Date.parse(predecessor.allocatedAt);
  const successorTime = Date.parse(successor.allocatedAt);
  if (!Number.isFinite(predecessorTime) || !Number.isFinite(successorTime)) {
    throw new Error('settlement allocation lineage requires valid allocation timestamps');
  }
  if (successorTime <= predecessorTime) {
    throw new Error('settlement allocation successor must be allocated strictly after its predecessor');
  }

  if (successor.residualHeld.amountMinor >= predecessor.residualHeld.amountMinor) {
    throw new Error('settlement allocation successor must strictly reduce residual held value');
  }
  if (successor.creatorPaid.amountMinor < predecessor.creatorPaid.amountMinor) {
    throw new Error('settlement allocation successor cannot reduce creator-paid value');
  }
  if (successor.deductionTotal.amountMinor < predecessor.deductionTotal.amountMinor) {
    throw new Error('settlement allocation successor cannot reduce authority-bound deductions');
  }
  if (!rootsAreSubset(predecessor.deductionRoots, successor.deductionRoots)) {
    throw new Error('settlement allocation successor cannot remove a prior deduction authority root');
  }
  if (
    successor.deductionTotal.amountMinor > predecessor.deductionTotal.amountMinor
    && successor.deductionRoots.length <= predecessor.deductionRoots.length
  ) {
    throw new Error('increased settlement deductions require additional authority roots');
  }

  const creatorAmountChanged = successor.creatorPaid.amountMinor !== predecessor.creatorPaid.amountMinor;
  const receiptChanged = successor.railReceiptRef !== predecessor.railReceiptRef;
  if (creatorAmountChanged !== receiptChanged) {
    throw new Error('creator-paid amount and durable rail receipt identity must advance together');
  }
}

export function createSettlementAllocationSuccessorLink(
  input: CreateSettlementAllocationSuccessorLinkInput,
): Readonly<SettlementAllocationSuccessorLink> {
  assertSettlementAllocationLineageStep(input.predecessor, input.successor);
  const linkId = required('settlement allocation successor linkId', input.linkId);
  const supersessionEvidenceRef = required('settlement allocation supersessionEvidenceRef', input.supersessionEvidenceRef);
  const linkedAt = timestamp('settlement allocation successor linkedAt', input.linkedAt);
  if (Date.parse(linkedAt) < Date.parse(input.successor.allocatedAt)) {
    throw new Error('settlement allocation successor link cannot predate the successor allocation');
  }

  const committed = Object.freeze({
    linkId,
    batchId: input.predecessor.batchId,
    predecessorAllocationId: input.predecessor.allocationId,
    predecessorAllocationRoot: input.predecessor.allocationRoot,
    successorAllocationId: input.successor.allocationId,
    successorAllocationRoot: input.successor.allocationRoot,
    supersessionEvidenceRef,
    linkedAt,
  });
  const linkRoot = buildMerkleCommitment([{
    recordType: 'settlement_allocation_successor_link_v1',
    ...committed,
  }]).root;

  return Object.freeze({ ...committed, linkRoot });
}

export function assertSettlementAllocationSuccessorLink(
  link: SettlementAllocationSuccessorLink,
  predecessor: SettlementAllocationAuthority,
  successor: SettlementAllocationAuthority,
): void {
  const expected = createSettlementAllocationSuccessorLink({
    linkId: link.linkId,
    predecessor,
    successor,
    supersessionEvidenceRef: link.supersessionEvidenceRef,
    linkedAt: link.linkedAt,
  });
  const exact = link.batchId === expected.batchId
    && link.predecessorAllocationId === expected.predecessorAllocationId
    && link.predecessorAllocationRoot === expected.predecessorAllocationRoot
    && link.successorAllocationId === expected.successorAllocationId
    && link.successorAllocationRoot === expected.successorAllocationRoot
    && link.linkRoot === expected.linkRoot;
  if (!exact) throw new Error('settlement allocation successor link does not match authoritative reconstruction');
}

/**
 * Resolve one batch's complete immutable allocation lineage. The resolver never
 * uses insertion order or "latest wins". Missing links, forks, joins, cycles,
 * foreign endpoints and disconnected components all fail closed.
 */
export function resolveSettlementAllocationLineage(
  allocations: readonly SettlementAllocationAuthority[],
  links: readonly SettlementAllocationSuccessorLink[],
): Readonly<SettlementAllocationLineageResolution> {
  if (allocations.length === 0) throw new Error('settlement allocation lineage requires at least one allocation');

  const byRoot = new Map<Digest, SettlementAllocationAuthority>();
  const allocationIds = new Set<string>();
  const first = allocations[0]!;
  for (const allocation of allocations) {
    if (!sameBatchIdentity(first, allocation)) {
      throw new Error('settlement allocation lineage cannot mix batch or eligibility identities');
    }
    if (byRoot.has(allocation.allocationRoot)) throw new Error(`duplicate allocation root in lineage: ${allocation.allocationRoot}`);
    if (allocationIds.has(allocation.allocationId)) throw new Error(`duplicate allocation id in lineage: ${allocation.allocationId}`);
    byRoot.set(allocation.allocationRoot, allocation);
    allocationIds.add(allocation.allocationId);
  }

  if (links.length !== allocations.length - 1) {
    throw new Error('settlement allocation lineage must contain exactly one successor link per non-initial allocation');
  }

  const outgoing = new Map<Digest, SettlementAllocationSuccessorLink>();
  const incoming = new Map<Digest, SettlementAllocationSuccessorLink>();
  const linkIds = new Set<string>();
  const linkRoots = new Set<Digest>();
  for (const link of links) {
    if (linkIds.has(link.linkId)) throw new Error(`duplicate settlement allocation successor link id: ${link.linkId}`);
    if (linkRoots.has(link.linkRoot)) throw new Error(`duplicate settlement allocation successor link root: ${link.linkRoot}`);
    linkIds.add(link.linkId);
    linkRoots.add(link.linkRoot);

    const predecessor = byRoot.get(link.predecessorAllocationRoot);
    const successor = byRoot.get(link.successorAllocationRoot);
    if (!predecessor || !successor) throw new Error('settlement allocation successor link references an unknown allocation root');
    if (outgoing.has(predecessor.allocationRoot)) throw new Error('settlement allocation lineage fork detected');
    if (incoming.has(successor.allocationRoot)) throw new Error('settlement allocation lineage join detected');
    assertSettlementAllocationSuccessorLink(link, predecessor, successor);
    outgoing.set(predecessor.allocationRoot, link);
    incoming.set(successor.allocationRoot, link);
  }

  const initial = allocations.filter(allocation => !incoming.has(allocation.allocationRoot));
  const heads = allocations.filter(allocation => !outgoing.has(allocation.allocationRoot));
  if (initial.length !== 1) throw new Error('settlement allocation lineage must have exactly one initial allocation');
  if (heads.length !== 1) throw new Error('settlement allocation lineage must have exactly one canonical head');

  const orderedAllocations: SettlementAllocationAuthority[] = [];
  const orderedLinks: SettlementAllocationSuccessorLink[] = [];
  const visited = new Set<Digest>();
  let current = initial[0]!;
  while (true) {
    if (visited.has(current.allocationRoot)) throw new Error('settlement allocation lineage cycle detected');
    visited.add(current.allocationRoot);
    orderedAllocations.push(current);
    const link = outgoing.get(current.allocationRoot);
    if (!link) break;
    orderedLinks.push(link);
    current = byRoot.get(link.successorAllocationRoot)!;
  }
  if (visited.size !== allocations.length) {
    throw new Error('settlement allocation lineage is disconnected or cyclic');
  }
  if (current.allocationRoot !== heads[0]!.allocationRoot) {
    throw new Error('settlement allocation lineage head is not reachable from its initial allocation');
  }

  const allocationRoots = Object.freeze(orderedAllocations.map(allocation => allocation.allocationRoot));
  const orderedLinkRoots = Object.freeze(orderedLinks.map(link => link.linkRoot));
  const lineageRoot = buildMerkleCommitment([{
    recordType: 'settlement_allocation_lineage_v1',
    batchId: first.batchId,
    allocationRoots,
    linkRoots: orderedLinkRoots,
  }]).root;

  return Object.freeze({
    batchId: first.batchId,
    initialAllocationRoot: allocationRoots[0]!,
    headAllocationRoot: current.allocationRoot,
    headAllocation: current,
    allocationRoots,
    linkRoots: orderedLinkRoots,
    depth: orderedLinks.length,
    lineageRoot,
  });
}
