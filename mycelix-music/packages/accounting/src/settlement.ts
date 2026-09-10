import { buildMerkleCommitment, type Digest } from './merkle.js';
import { addMoney, assertSameCurrency, money, type Money } from './money.js';
import {
  assertRoyaltyObligationAuthority,
  type RoyaltyObligationAuthority,
} from './obligation-authority.js';

/** Immutable debt principal. Payability state lives in append-only observations. */
export type RoyaltyObligation = RoyaltyObligationAuthority;

export enum SettlementEligibilityCode {
  Eligible = 'eligible',
  BelowThreshold = 'below_threshold',
  AwaitingEligibilityEvidence = 'awaiting_eligibility_evidence',
  AwaitingPayeeRoute = 'awaiting_payee_route',
  RightsConflict = 'rights_conflict',
  LegalHold = 'legal_hold',
  TaxDocumentationRequired = 'tax_documentation_required',
  AwaitingFxQuote = 'awaiting_fx_quote',
  DormantBeneficiary = 'dormant_beneficiary',
}

/**
 * Codes that may be asserted by an external eligibility authority.
 * BelowThreshold and AwaitingEligibilityEvidence are deterministic compiler
 * outcomes and therefore can never be persisted as source observations.
 */
export type ObservableSettlementEligibilityCode =
  | SettlementEligibilityCode.Eligible
  | SettlementEligibilityCode.AwaitingPayeeRoute
  | SettlementEligibilityCode.RightsConflict
  | SettlementEligibilityCode.LegalHold
  | SettlementEligibilityCode.TaxDocumentationRequired
  | SettlementEligibilityCode.AwaitingFxQuote
  | SettlementEligibilityCode.DormantBeneficiary;

const OBSERVABLE_ELIGIBILITY_CODES: ReadonlySet<SettlementEligibilityCode> = new Set([
  SettlementEligibilityCode.Eligible,
  SettlementEligibilityCode.AwaitingPayeeRoute,
  SettlementEligibilityCode.RightsConflict,
  SettlementEligibilityCode.LegalHold,
  SettlementEligibilityCode.TaxDocumentationRequired,
  SettlementEligibilityCode.AwaitingFxQuote,
  SettlementEligibilityCode.DormantBeneficiary,
]);

export interface SettlementEligibility {
  readonly code: SettlementEligibilityCode;
  readonly reason?: string;
}

export interface SettlementEligibilityObservation {
  readonly id: string;
  readonly obligationId: string;
  readonly code: ObservableSettlementEligibilityCode;
  readonly reason?: string;
  readonly sourceRef: string;
  readonly observedAt: string;
}

export interface ReconstructedSettlementEligibility {
  readonly obligationId: string;
  readonly asOf: string;
  readonly eligibility: SettlementEligibility;
  readonly observation?: Readonly<SettlementEligibilityObservation>;
}

export interface SettlementEligibilityEvidenceCommitment {
  readonly asOf: string;
  readonly root: Digest;
  readonly selected: readonly ReconstructedSettlementEligibility[];
}

export interface SettlementEpoch {
  readonly id: string;
  /** Which obligations are eligible to enter this settlement epoch. */
  readonly cutoff: string;
  /** Which append-only payability evidence may influence this plan. */
  readonly eligibilityAsOf: string;
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

function required(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

function canonicalTimestamp(label: string, value: string): string {
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error(`${label} must be valid`);
  return new Date(parsed).toISOString();
}

function validateObligation(obligation: RoyaltyObligation): void {
  assertRoyaltyObligationAuthority(obligation);
  if (obligation.amount.amountMinor < 0n) throw new Error('royalty obligation amount must be non-negative');
  canonicalTimestamp('obligation observedAt', obligation.observedAt);
}

function validateEpoch(epoch: SettlementEpoch): void {
  required('settlement epoch id', epoch.id);
  canonicalTimestamp('settlement epoch cutoff', epoch.cutoff);
  canonicalTimestamp('settlement epoch eligibilityAsOf', epoch.eligibilityAsOf);
  if (epoch.minimumPayout.amountMinor < 0n) throw new Error('minimum payout must be non-negative');
}

export function isObservableSettlementEligibilityCode(
  code: SettlementEligibilityCode,
): code is ObservableSettlementEligibilityCode {
  return OBSERVABLE_ELIGIBILITY_CODES.has(code);
}

function normalizeEligibilityObservation(
  observation: SettlementEligibilityObservation,
): Readonly<SettlementEligibilityObservation> {
  const code = observation.code as SettlementEligibilityCode;
  if (!isObservableSettlementEligibilityCode(code)) {
    throw new Error(`eligibility observation cannot assert compiler-only code: ${code}`);
  }
  const reason = observation.reason === undefined
    ? undefined
    : required('eligibility reason', observation.reason);
  return Object.freeze({
    id: required('eligibility observation id', observation.id),
    obligationId: required('eligibility obligationId', observation.obligationId),
    code,
    ...(reason === undefined ? {} : { reason }),
    sourceRef: required('eligibility sourceRef', observation.sourceRef),
    observedAt: canonicalTimestamp('eligibility observedAt', observation.observedAt),
  });
}

function sameObservation(
  left: SettlementEligibilityObservation,
  right: SettlementEligibilityObservation,
): boolean {
  return left.id === right.id
    && left.obligationId === right.obligationId
    && left.code === right.code
    && (left.reason ?? '') === (right.reason ?? '')
    && left.sourceRef === right.sourceRef
    && left.observedAt === right.observedAt;
}

function normalizedObservationSet(
  observations: readonly SettlementEligibilityObservation[],
): readonly Readonly<SettlementEligibilityObservation>[] {
  const byId = new Map<string, Readonly<SettlementEligibilityObservation>>();
  for (const raw of observations) {
    const observation = normalizeEligibilityObservation(raw);
    const existing = byId.get(observation.id);
    if (existing) {
      if (!sameObservation(existing, observation)) {
        throw new Error(`eligibility observation id reused with different evidence: ${observation.id}`);
      }
      continue;
    }
    byId.set(observation.id, observation);
  }
  return Object.freeze([...byId.values()]);
}

export function assertEligibilityObservationReferences(
  obligations: readonly RoyaltyObligation[],
  observations: readonly SettlementEligibilityObservation[],
): void {
  const obligationIds = new Set(obligations.map(obligation => obligation.id));
  for (const observation of normalizedObservationSet(observations)) {
    if (!obligationIds.has(observation.obligationId)) {
      throw new Error(`eligibility observation references unknown obligation: ${observation.obligationId}`);
    }
  }
}

export function reconstructSettlementEligibility(
  obligation: RoyaltyObligation,
  observations: readonly SettlementEligibilityObservation[],
  asOfInput: string,
): Readonly<ReconstructedSettlementEligibility> {
  validateObligation(obligation);
  const asOf = canonicalTimestamp('eligibility asOf', asOfInput);
  const asOfMs = Date.parse(asOf);
  const relevant = normalizedObservationSet(observations)
    .filter(observation => observation.obligationId === obligation.id && Date.parse(observation.observedAt) <= asOfMs)
    .sort((left, right) => Date.parse(left.observedAt) - Date.parse(right.observedAt) || left.id.localeCompare(right.id));

  if (relevant.length === 0) {
    return Object.freeze({
      obligationId: obligation.id,
      asOf,
      eligibility: Object.freeze({
        code: SettlementEligibilityCode.AwaitingEligibilityEvidence,
        reason: `no eligibility evidence observable by ${asOf}`,
      }),
    });
  }

  const latest = relevant[relevant.length - 1]!;
  const latestTimestamp = latest.observedAt;
  const tied = relevant.filter(observation => observation.observedAt === latestTimestamp);
  if (tied.length > 1) {
    throw new Error(`ambiguous eligibility evidence at identical timestamp for obligation ${obligation.id}`);
  }

  return Object.freeze({
    obligationId: obligation.id,
    asOf,
    eligibility: Object.freeze({
      code: latest.code,
      ...(latest.reason === undefined ? {} : { reason: latest.reason }),
    }),
    observation: latest,
  });
}

export function buildSettlementEligibilityEvidenceCommitment(
  obligations: readonly RoyaltyObligation[],
  observations: readonly SettlementEligibilityObservation[],
  asOfInput: string,
): Readonly<SettlementEligibilityEvidenceCommitment> {
  const asOf = canonicalTimestamp('eligibility asOf', asOfInput);
  const selected = [...obligations]
    .sort((left, right) => left.id.localeCompare(right.id))
    .map(obligation => reconstructSettlementEligibility(obligation, observations, asOf));

  const committed = selected.map(selection => selection.observation
    ? {
        obligationId: selection.obligationId,
        observationId: selection.observation.id,
        code: selection.observation.code,
        reason: selection.observation.reason ?? null,
        sourceRef: selection.observation.sourceRef,
        observedAt: selection.observation.observedAt,
        eligibilityAsOf: asOf,
      }
    : {
        obligationId: selection.obligationId,
        code: SettlementEligibilityCode.AwaitingEligibilityEvidence,
        eligibilityAsOf: asOf,
      });

  const root = buildMerkleCommitment(
    committed.length > 0 ? committed : [{ eligibilityAsOf: asOf, obligationCount: 0 }],
  ).root;
  return Object.freeze({ asOf, root, selected: Object.freeze(selected) });
}

export function partitionObligationsAtCutoff(
  obligations: readonly RoyaltyObligation[],
  epoch: SettlementEpoch,
): CutoffPartition {
  validateEpoch(epoch);
  const cutoff = Date.parse(epoch.cutoff);
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
  eligibilityObservations: readonly SettlementEligibilityObservation[],
): CarryForwardAssessment {
  if (obligations.length === 0) throw new Error('carry-forward assessment requires obligations');
  validateEpoch(epoch);
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
    const reconstructed = reconstructSettlementEligibility(
      obligation,
      eligibilityObservations,
      epoch.eligibilityAsOf,
    );
    if (reconstructed.eligibility.code === SettlementEligibilityCode.Eligible) {
      payableNow.push(obligation);
      carriedForward = addMoney(carriedForward, obligation.amount);
    } else {
      held.push(Object.freeze({ obligation, eligibility: reconstructed.eligibility }));
    }
  }

  let eligibility: SettlementEligibility;
  if (payableNow.length === 0) {
    eligibility = held[0]?.eligibility ?? Object.freeze({
      code: SettlementEligibilityCode.AwaitingEligibilityEvidence,
      reason: 'no payable obligations have eligibility evidence',
    });
  } else if (carriedForward.amountMinor < epoch.minimumPayout.amountMinor) {
    eligibility = Object.freeze({
      code: SettlementEligibilityCode.BelowThreshold,
      reason: `carried value ${carriedForward.amountMinor} is below minimum ${epoch.minimumPayout.amountMinor}`,
    });
  } else {
    eligibility = Object.freeze({ code: SettlementEligibilityCode.Eligible });
  }

  return Object.freeze({
    beneficiaryId,
    payableNow: Object.freeze(payableNow),
    held: Object.freeze(held),
    carriedForward,
    eligibility,
  });
}
