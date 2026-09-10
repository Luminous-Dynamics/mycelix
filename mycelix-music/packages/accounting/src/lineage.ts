import { buildMerkleCommitment, type Digest } from './merkle.js';
import { assertRoyaltyObligationAuthority, type RoyaltyObligationAuthority } from './obligation-authority.js';
import { type Money } from './money.js';

export type MoneyLineageNodeKind =
  | 'payment'
  | 'settlement_line'
  | 'netting_batch'
  | 'royalty_obligation'
  | 'economic_terms'
  | 'rights_resolution'
  | 'rights_agreement'
  | 'repertoire_snapshot'
  | 'usage_evidence_epoch';

export interface MoneyLineageNode {
  readonly kind: MoneyLineageNodeKind;
  readonly ref: string;
  /** Present when the node names immutable royalty-debt authority. */
  readonly authorityRoot?: Digest;
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
  /** Exact immutable debts represented by this payment lineage. */
  readonly obligations: readonly RoyaltyObligationAuthority[];
  /** Broader rights/repertoire context not encoded directly in an obligation principal. */
  readonly rightsAgreementRef: string;
  readonly repertoireSnapshotRef: string;
}

function requireRef(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

function uniqueSorted(values: readonly string[], label: string): readonly string[] {
  const normalized = values.map(value => requireRef(label, value));
  return Object.freeze([...new Set(normalized)].sort());
}

export function buildMoneyLineage(input: MoneyLineageInput): Readonly<MoneyLineage> {
  const beneficiaryId = requireRef('beneficiaryId', input.beneficiaryId);
  if (input.received.amountMinor < 0n) throw new Error('received amount must be non-negative');
  if (input.obligations.length === 0) throw new Error('money lineage requires at least one royalty obligation');

  const obligations = [...input.obligations].sort((a, b) => a.id.localeCompare(b.id));
  const seenIds = new Set<string>();
  for (const obligation of obligations) {
    assertRoyaltyObligationAuthority(obligation);
    if (obligation.beneficiaryId !== beneficiaryId) {
      throw new Error('money lineage obligation beneficiary must match payment beneficiary');
    }
    if (obligation.amount.currency !== input.received.currency) {
      throw new Error('money lineage obligation currency must match received currency');
    }
    if (seenIds.has(obligation.id)) throw new Error('money lineage obligation ids must be unique');
    seenIds.add(obligation.id);
  }

  const economicTermsRefs = uniqueSorted(obligations.map(item => item.provenance.economicTermsRef), 'economicTermsRef');
  const rightsResolutionRefs = uniqueSorted(obligations.map(item => item.provenance.rightsResolutionRef), 'rightsResolutionRef');
  const usageEvidenceRefs = uniqueSorted(obligations.map(item => item.provenance.usageEvidenceRef), 'usageEvidenceRef');

  const nodes: MoneyLineageNode[] = [
    { kind: 'payment', ref: requireRef('paymentRef', input.paymentRef) },
    { kind: 'settlement_line', ref: requireRef('settlementLineRef', input.settlementLineRef) },
    { kind: 'netting_batch', ref: requireRef('nettingBatchRef', input.nettingBatchRef) },
    ...obligations.map(obligation => ({
      kind: 'royalty_obligation' as const,
      ref: obligation.id,
      authorityRoot: obligation.authorityRoot,
    })),
    ...economicTermsRefs.map(ref => ({ kind: 'economic_terms' as const, ref })),
    ...rightsResolutionRefs.map(ref => ({ kind: 'rights_resolution' as const, ref })),
    { kind: 'rights_agreement', ref: requireRef('rightsAgreementRef', input.rightsAgreementRef) },
    { kind: 'repertoire_snapshot', ref: requireRef('repertoireSnapshotRef', input.repertoireSnapshotRef) },
    ...usageEvidenceRefs.map(ref => ({ kind: 'usage_evidence_epoch' as const, ref })),
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

export function lineageObligationAuthorities(
  lineage: MoneyLineage,
): readonly Readonly<{ ref: string; authorityRoot: Digest }>[] {
  return Object.freeze(lineage.nodes
    .filter((node): node is MoneyLineageNode & { readonly authorityRoot: Digest } =>
      node.kind === 'royalty_obligation' && node.authorityRoot !== undefined)
    .map(node => Object.freeze({ ref: node.ref, authorityRoot: node.authorityRoot })));
}
