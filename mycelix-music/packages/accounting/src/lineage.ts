import { buildMerkleCommitment, type Digest } from './merkle.js';
import { type Money } from './money.js';

export type MoneyLineageNodeKind =
  | 'payment'
  | 'settlement_line'
  | 'netting_batch'
  | 'royalty_obligation'
  | 'economic_terms'
  | 'rights_agreement'
  | 'repertoire_snapshot'
  | 'usage_evidence_epoch';

export interface MoneyLineageNode {
  readonly kind: MoneyLineageNodeKind;
  readonly ref: string;
}

export interface MoneyLineage {
  readonly beneficiaryId: string;
  readonly received: Money;
  readonly nodes: readonly MoneyLineageNode[];
  readonly lineageRoot: Digest;
}

export interface MoneyLineageInput {
  readonly beneficiaryId: string;
  readonly received: Money;
  readonly paymentRef: string;
  readonly settlementLineRef: string;
  readonly nettingBatchRef: string;
  readonly obligationRefs: readonly string[];
  readonly economicTermsRef: string;
  readonly rightsAgreementRef: string;
  readonly repertoireSnapshotRef: string;
  readonly usageEvidenceEpochRef: string;
}

function requireRef(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

export function buildMoneyLineage(input: MoneyLineageInput): Readonly<MoneyLineage> {
  const beneficiaryId = requireRef('beneficiaryId', input.beneficiaryId);
  if (input.received.amountMinor < 0n) throw new Error('received amount must be non-negative');
  if (input.obligationRefs.length === 0) throw new Error('money lineage requires at least one royalty obligation');

  const obligationRefs = input.obligationRefs.map(ref => requireRef('obligationRef', ref));
  if (new Set(obligationRefs).size !== obligationRefs.length) {
    throw new Error('money lineage obligation refs must be unique');
  }

  const nodes: MoneyLineageNode[] = [
    { kind: 'payment', ref: requireRef('paymentRef', input.paymentRef) },
    { kind: 'settlement_line', ref: requireRef('settlementLineRef', input.settlementLineRef) },
    { kind: 'netting_batch', ref: requireRef('nettingBatchRef', input.nettingBatchRef) },
    ...obligationRefs.sort().map(ref => ({ kind: 'royalty_obligation' as const, ref })),
    { kind: 'economic_terms', ref: requireRef('economicTermsRef', input.economicTermsRef) },
    { kind: 'rights_agreement', ref: requireRef('rightsAgreementRef', input.rightsAgreementRef) },
    { kind: 'repertoire_snapshot', ref: requireRef('repertoireSnapshotRef', input.repertoireSnapshotRef) },
    { kind: 'usage_evidence_epoch', ref: requireRef('usageEvidenceEpochRef', input.usageEvidenceEpochRef) },
  ];
  const frozenNodes = Object.freeze(nodes.map(node => Object.freeze({ ...node })));
  return Object.freeze({
    beneficiaryId,
    received: Object.freeze({ ...input.received }),
    nodes: frozenNodes,
    lineageRoot: buildMerkleCommitment(frozenNodes).root,
  });
}

export function lineageRefs(lineage: MoneyLineage, kind: MoneyLineageNodeKind): readonly string[] {
  return Object.freeze(lineage.nodes.filter(node => node.kind === kind).map(node => node.ref));
}
