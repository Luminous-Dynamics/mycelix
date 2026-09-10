import { buildMerkleCommitment, type Digest } from './merkle.js';
import { money, type Money } from './money.js';

export interface RoyaltyObligationProvenance {
  readonly usageEvidenceRef: string;
  readonly rightsResolutionRef: string;
  readonly economicTermsRef: string;
}

export interface RoyaltyObligationPrincipal {
  readonly id: string;
  readonly beneficiaryId: string;
  readonly amount: Money;
  readonly observedAt: string;
  readonly provenance: RoyaltyObligationProvenance;
}

export interface RoyaltyObligationAuthority extends RoyaltyObligationPrincipal {
  readonly authorityRoot: Digest;
}

function normalizeText(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

function normalizeTimestamp(value: string): string {
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error('obligation observedAt must be valid');
  return new Date(parsed).toISOString();
}

export function committedRoyaltyObligationPrincipal(
  obligation: RoyaltyObligationPrincipal,
): Readonly<Record<string, unknown>> {
  const normalizedAmount = money(obligation.amount.amountMinor, obligation.amount.currency);
  if (normalizedAmount.amountMinor < 0n) throw new Error('royalty obligation amount must be non-negative');

  return Object.freeze({
    id: normalizeText('royalty obligation id', obligation.id),
    beneficiaryId: normalizeText('royalty obligation beneficiaryId', obligation.beneficiaryId),
    amountMinor: normalizedAmount.amountMinor,
    currency: normalizedAmount.currency,
    observedAt: normalizeTimestamp(obligation.observedAt),
    usageEvidenceRef: normalizeText('usageEvidenceRef', obligation.provenance.usageEvidenceRef),
    rightsResolutionRef: normalizeText('rightsResolutionRef', obligation.provenance.rightsResolutionRef),
    economicTermsRef: normalizeText('economicTermsRef', obligation.provenance.economicTermsRef),
  });
}

export function createRoyaltyObligationAuthority(
  input: RoyaltyObligationPrincipal,
): Readonly<RoyaltyObligationAuthority> {
  const committed = committedRoyaltyObligationPrincipal(input);
  const amount = money(committed.amountMinor as bigint, committed.currency as string);
  const provenance = Object.freeze({
    usageEvidenceRef: committed.usageEvidenceRef as string,
    rightsResolutionRef: committed.rightsResolutionRef as string,
    economicTermsRef: committed.economicTermsRef as string,
  });

  return Object.freeze({
    id: committed.id as string,
    beneficiaryId: committed.beneficiaryId as string,
    amount,
    observedAt: committed.observedAt as string,
    provenance,
    authorityRoot: buildMerkleCommitment([committed]).root,
  });
}

export function assertRoyaltyObligationAuthority(
  obligation: RoyaltyObligationAuthority,
): void {
  const expected = buildMerkleCommitment([committedRoyaltyObligationPrincipal(obligation)]).root;
  if (obligation.authorityRoot !== expected) {
    throw new Error(`royalty obligation ${obligation.id} authorityRoot does not match immutable provenance`);
  }
}
