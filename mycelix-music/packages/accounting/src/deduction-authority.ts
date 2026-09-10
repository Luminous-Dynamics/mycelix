import { buildMerkleCommitment, type Digest } from './merkle.js';
import { money, type Money } from './money.js';

export interface RoyaltyDeductionPrincipal {
  readonly id: string;
  readonly beneficiaryId: string;
  readonly amount: Money;
  readonly basis: string;
  readonly authorityRef: string;
  readonly observedAt: string;
}

export interface RoyaltyDeductionAuthority extends RoyaltyDeductionPrincipal {
  readonly deductionRoot: Digest;
}

function required(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

function canonicalTimestamp(value: string): string {
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error('deduction observedAt must be valid');
  return new Date(parsed).toISOString();
}

export function committedRoyaltyDeductionPrincipal(
  deduction: RoyaltyDeductionPrincipal,
): Readonly<Record<string, unknown>> {
  const amount = money(deduction.amount.amountMinor, deduction.amount.currency);
  if (amount.amountMinor < 0n) throw new Error('deduction amount must be non-negative');
  return Object.freeze({
    id: required('deduction id', deduction.id),
    beneficiaryId: required('deduction beneficiaryId', deduction.beneficiaryId),
    amountMinor: amount.amountMinor,
    currency: amount.currency,
    basis: required('deduction basis', deduction.basis),
    authorityRef: required('deduction authorityRef', deduction.authorityRef),
    observedAt: canonicalTimestamp(deduction.observedAt),
  });
}

function deductionRoot(committed: Readonly<Record<string, unknown>>): Digest {
  return buildMerkleCommitment([{ recordType: 'royalty_deduction_v1', ...committed }]).root;
}

export function createRoyaltyDeductionAuthority(
  input: RoyaltyDeductionPrincipal,
): Readonly<RoyaltyDeductionAuthority> {
  const committed = committedRoyaltyDeductionPrincipal(input);
  return Object.freeze({
    id: committed.id as string,
    beneficiaryId: committed.beneficiaryId as string,
    amount: money(committed.amountMinor as bigint, committed.currency as string),
    basis: committed.basis as string,
    authorityRef: committed.authorityRef as string,
    observedAt: committed.observedAt as string,
    deductionRoot: deductionRoot(committed),
  });
}

export function assertRoyaltyDeductionAuthority(
  deduction: RoyaltyDeductionAuthority,
): void {
  const expected = deductionRoot(committedRoyaltyDeductionPrincipal(deduction));
  if (deduction.deductionRoot !== expected) {
    throw new Error(`royalty deduction ${deduction.id} deductionRoot does not match immutable authority evidence`);
  }
}
