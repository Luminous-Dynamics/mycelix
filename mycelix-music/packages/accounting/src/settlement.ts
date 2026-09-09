import { addMoney, assertSameCurrency, money, type Money } from './money.js';

export interface RoyaltyObligation {
  readonly id: string;
  readonly beneficiaryId: string;
  readonly amount: Money;
  readonly observedAt: string;
  readonly routeAvailable?: boolean;
  readonly rightsConflict?: boolean;
  readonly legalHold?: boolean;
  readonly taxDocumentationRequired?: boolean;
  readonly taxDocumentationPresent?: boolean;
  readonly fxQuoteRequired?: boolean;
  readonly fxQuotePresent?: boolean;
  readonly dormantBeneficiary?: boolean;
}

export enum SettlementEligibilityCode {
  Eligible = 'eligible',
  BelowThreshold = 'below_threshold',
  AwaitingPayeeRoute = 'awaiting_payee_route',
  RightsConflict = 'rights_conflict',
  LegalHold = 'legal_hold',
  TaxDocumentationRequired = 'tax_documentation_required',
  AwaitingFxQuote = 'awaiting_fx_quote',
  DormantBeneficiary = 'dormant_beneficiary',
}

export interface SettlementEligibility {
  readonly code: SettlementEligibilityCode;
  readonly reason?: string;
}

export interface SettlementEpoch {
  readonly id: string;
  readonly cutoff: string;
  readonly minimumPayout: Money;
}

export interface CutoffPartition {
  readonly inEpoch: readonly RoyaltyObligation[];
  readonly nextEpoch: readonly RoyaltyObligation[];
}

export interface CarryForwardAssessment {
  readonly beneficiaryId: string;
  readonly payableNow: readonly RoyaltyObligation[];
  readonly held: readonly Readonly<{ obligation: RoyaltyObligation; eligibility: SettlementEligibility }>[];
  readonly carriedForward: Money;
  readonly eligibility: SettlementEligibility;
}

function validateObligation(obligation: RoyaltyObligation): void {
  if (!obligation.id.trim() || !obligation.beneficiaryId.trim()) {
    throw new Error('royalty obligation id and beneficiaryId must be non-empty');
  }
  if (obligation.amount.amountMinor < 0n) throw new Error('royalty obligation amount must be non-negative');
  if (!Number.isFinite(Date.parse(obligation.observedAt))) throw new Error('obligation observedAt must be valid');
}

export function classifySettlementEligibility(
  obligation: RoyaltyObligation,
): SettlementEligibility {
  validateObligation(obligation);
  if (obligation.rightsConflict) return Object.freeze({ code: SettlementEligibilityCode.RightsConflict });
  if (obligation.legalHold) return Object.freeze({ code: SettlementEligibilityCode.LegalHold });
  if (obligation.taxDocumentationRequired && !obligation.taxDocumentationPresent) {
    return Object.freeze({ code: SettlementEligibilityCode.TaxDocumentationRequired });
  }
  if (obligation.fxQuoteRequired && !obligation.fxQuotePresent) {
    return Object.freeze({ code: SettlementEligibilityCode.AwaitingFxQuote });
  }
  if (obligation.dormantBeneficiary) {
    return Object.freeze({ code: SettlementEligibilityCode.DormantBeneficiary });
  }
  if (obligation.routeAvailable === false) {
    return Object.freeze({ code: SettlementEligibilityCode.AwaitingPayeeRoute });
  }
  return Object.freeze({ code: SettlementEligibilityCode.Eligible });
}

export function partitionObligationsAtCutoff(
  obligations: readonly RoyaltyObligation[],
  epoch: SettlementEpoch,
): CutoffPartition {
  if (!epoch.id.trim()) throw new Error('settlement epoch id must be non-empty');
  const cutoff = Date.parse(epoch.cutoff);
  if (!Number.isFinite(cutoff)) throw new Error('settlement epoch cutoff must be valid');
  const inEpoch: RoyaltyObligation[] = [];
  const nextEpoch: RoyaltyObligation[] = [];
  for (const obligation of obligations) {
    validateObligation(obligation);
    (Date.parse(obligation.observedAt) <= cutoff ? inEpoch : nextEpoch).push(obligation);
  }
  const byId = (a: RoyaltyObligation, b: RoyaltyObligation) => a.id.localeCompare(b.id);
  return Object.freeze({
    inEpoch: Object.freeze([...inEpoch].sort(byId)),
    nextEpoch: Object.freeze([...nextEpoch].sort(byId)),
  });
}

export function assessCarryForward(
  obligations: readonly RoyaltyObligation[],
  epoch: SettlementEpoch,
): CarryForwardAssessment {
  if (obligations.length === 0) throw new Error('carry-forward assessment requires obligations');
  if (epoch.minimumPayout.amountMinor < 0n) throw new Error('minimum payout must be non-negative');
  const { inEpoch } = partitionObligationsAtCutoff(obligations, epoch);
  if (inEpoch.length === 0) throw new Error('no obligations fall within this settlement epoch');

  const beneficiaryId = inEpoch[0]!.beneficiaryId;
  const currency = inEpoch[0]!.amount.currency;
  const payableNow: RoyaltyObligation[] = [];
  const held: Array<Readonly<{ obligation: RoyaltyObligation; eligibility: SettlementEligibility }>> = [];
  let carriedForward = money(0n, currency);

  assertSameCurrency(carriedForward, epoch.minimumPayout);
  for (const obligation of inEpoch) {
    if (obligation.beneficiaryId !== beneficiaryId) {
      throw new Error('carry-forward assessment must cover exactly one beneficiary');
    }
    assertSameCurrency(carriedForward, obligation.amount);
    const eligibility = classifySettlementEligibility(obligation);
    if (eligibility.code === SettlementEligibilityCode.Eligible) {
      payableNow.push(obligation);
      carriedForward = addMoney(carriedForward, obligation.amount);
    } else {
      held.push(Object.freeze({ obligation, eligibility }));
    }
  }

  const eligibility = carriedForward.amountMinor < epoch.minimumPayout.amountMinor
    ? Object.freeze({
        code: SettlementEligibilityCode.BelowThreshold,
        reason: `carried value ${carriedForward.amountMinor} is below minimum ${epoch.minimumPayout.amountMinor}`,
      })
    : Object.freeze({ code: SettlementEligibilityCode.Eligible });

  return Object.freeze({
    beneficiaryId,
    payableNow: Object.freeze(payableNow),
    held: Object.freeze(held),
    carriedForward,
    eligibility,
  });
}
