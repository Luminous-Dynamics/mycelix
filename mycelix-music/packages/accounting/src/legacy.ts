import { buildMerkleCommitment, type Digest } from './merkle.js';
import { money, type Money } from './money.js';

/** Shape of the historical Prisma `RoyaltyPayment` row. */
export interface LegacyRoyaltyPaymentRow {
  readonly id: string;
  readonly songId: string;
  readonly recipientAddress: string;
  readonly amount: string;
  readonly currency: string;
  readonly txHash: string;
  readonly period: string;
  readonly playCount: number;
  readonly paidAt: Date | string;
}

/**
 * Evidence imported from the historical payment table.
 *
 * This is deliberately not assignable to RoyaltyObligation. A legacy payment
 * can corroborate that a rail transfer was observed, but cannot establish the
 * debt, rights basis, obligation set, or whether any other value remains owed.
 */
export interface LegacyRoyaltyPaymentObservation {
  readonly authority: 'receipt_projection_only';
  readonly legacyPaymentId: string;
  readonly songId: string;
  readonly recipientAddress: string;
  readonly amount: Money;
  readonly railReference: string;
  readonly periodLabel: string;
  readonly playCount: number;
  readonly observedPaidAt: string;
  readonly observationRoot: Digest;
  readonly migrationWarning: 'does_not_create_or_extinguish_obligation';
}

function normalizedTimestamp(value: Date | string): string {
  const date = value instanceof Date ? value : new Date(value);
  if (!Number.isFinite(date.getTime())) throw new Error('legacy royalty payment paidAt must be valid');
  return date.toISOString();
}

function parseMinorAmount(value: string): bigint {
  if (!/^[0-9]+$/.test(value)) {
    throw new Error('legacy royalty payment amount must be a non-negative integer string');
  }
  return BigInt(value);
}

export function observeLegacyRoyaltyPayment(
  row: LegacyRoyaltyPaymentRow,
): Readonly<LegacyRoyaltyPaymentObservation> {
  const legacyPaymentId = row.id.trim();
  const songId = row.songId.trim();
  const recipientAddress = row.recipientAddress.trim();
  const railReference = row.txHash.trim();
  const periodLabel = row.period.trim();
  if (!legacyPaymentId || !songId || !recipientAddress) {
    throw new Error('legacy royalty payment requires id, songId and recipientAddress');
  }
  if (!railReference) throw new Error('legacy royalty payment requires a rail reference');
  if (!periodLabel) throw new Error('legacy royalty payment requires a period label');
  if (!Number.isSafeInteger(row.playCount) || row.playCount < 0) {
    throw new Error('legacy royalty payment playCount must be a non-negative safe integer');
  }

  const amount = money(parseMinorAmount(row.amount), row.currency);
  const observedPaidAt = normalizedTimestamp(row.paidAt);
  const committed = {
    legacyPaymentId,
    songId,
    recipientAddress,
    amountMinor: amount.amountMinor,
    currency: amount.currency,
    railReference,
    periodLabel,
    playCount: row.playCount,
    observedPaidAt,
  };

  return Object.freeze({
    authority: 'receipt_projection_only',
    legacyPaymentId,
    songId,
    recipientAddress,
    amount,
    railReference,
    periodLabel,
    playCount: row.playCount,
    observedPaidAt,
    observationRoot: buildMerkleCommitment([committed]).root,
    migrationWarning: 'does_not_create_or_extinguish_obligation',
  });
}
