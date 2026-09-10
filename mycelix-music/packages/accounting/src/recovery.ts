import type { DeterministicNettingBatch } from './netting.js';
import { assertSameCurrency, money, type Money } from './money.js';

export enum SettlementAttemptState {
  Authorized = 'authorized',
  Submitted = 'submitted',
  Accepted = 'accepted',
  Confirmed = 'confirmed',
  Finalized = 'finalized',
  Failed = 'failed',
  Rejected = 'rejected',
  Reversed = 'reversed',
  Disputed = 'disputed',
}

export interface SettlementAttemptObservation {
  readonly attemptId: string;
  readonly batchId: string;
  readonly obligationSetRoot: string;
  readonly eligibilityAsOf: string;
  readonly eligibilityEvidenceRoot: string;
  readonly state: SettlementAttemptState;
  readonly observedAt: string;
  readonly railReceiptRef?: string;
  /** Exact rail-attested amount. Required only when state is Finalized. */
  readonly settledAmount?: Money;
  /** Required on the first observation of a retry attempt. Forbidden on the initial attempt. */
  readonly supersedesAttemptId?: string;
}

export type SettlementRecoveryStatus =
  | 'never_attempted'
  | 'in_flight'
  | 'retryable'
  | 'partial_finality'
  | 'finalized'
  | 'blocked_ambiguous';

export interface SettlementRecoveryState {
  readonly batchId: string;
  readonly obligationSetRoot: string;
  readonly eligibilityAsOf: string;
  readonly eligibilityEvidenceRoot: string;
  readonly status: SettlementRecoveryStatus;
  readonly activeAttemptId?: string;
  readonly lastObservedAt?: string;
  /** Durable receipt from a finalized rail observation, retained through later reversal/dispute. */
  readonly finalReceiptRef?: string;
  /** Exact amount attested by that finalized rail receipt, retained for audit after reversal/dispute. */
  readonly finalSettledAmount?: Money;
  readonly reason?: string;
  /** Rail evidence only: true when the finalized rail amount covers the batch gross. Never debt-discharge authority. */
  readonly railCoversBatchGross: boolean;
}

const ALLOWED_TRANSITIONS: Readonly<Record<SettlementAttemptState, readonly SettlementAttemptState[]>> = {
  [SettlementAttemptState.Authorized]: [SettlementAttemptState.Submitted, SettlementAttemptState.Accepted, SettlementAttemptState.Confirmed, SettlementAttemptState.Finalized, SettlementAttemptState.Failed, SettlementAttemptState.Rejected, SettlementAttemptState.Disputed],
  [SettlementAttemptState.Submitted]: [SettlementAttemptState.Accepted, SettlementAttemptState.Confirmed, SettlementAttemptState.Finalized, SettlementAttemptState.Failed, SettlementAttemptState.Rejected, SettlementAttemptState.Disputed],
  [SettlementAttemptState.Accepted]: [SettlementAttemptState.Confirmed, SettlementAttemptState.Finalized, SettlementAttemptState.Failed, SettlementAttemptState.Rejected, SettlementAttemptState.Disputed],
  [SettlementAttemptState.Confirmed]: [SettlementAttemptState.Finalized, SettlementAttemptState.Reversed, SettlementAttemptState.Disputed],
  [SettlementAttemptState.Finalized]: [SettlementAttemptState.Reversed, SettlementAttemptState.Disputed],
  [SettlementAttemptState.Failed]: [],
  [SettlementAttemptState.Rejected]: [],
  [SettlementAttemptState.Reversed]: [],
  [SettlementAttemptState.Disputed]: [],
};

function parseTimestamp(value: string): number {
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error(`invalid settlement observation timestamp: ${value}`);
  return parsed;
}

function validateObservation(batch: DeterministicNettingBatch, observation: SettlementAttemptObservation): void {
  if (!observation.attemptId.trim()) throw new Error('settlement attemptId must be non-empty');
  if (observation.batchId !== batch.batchId) throw new Error('settlement observation batchId mismatch');
  if (observation.obligationSetRoot !== batch.obligationSetRoot) throw new Error('settlement observation obligationSetRoot mismatch');
  if (observation.eligibilityAsOf !== batch.eligibilityAsOf) throw new Error('settlement observation eligibilityAsOf mismatch');
  if (observation.eligibilityEvidenceRoot !== batch.eligibilityEvidenceRoot) throw new Error('settlement observation eligibilityEvidenceRoot mismatch');
  const observedAt = parseTimestamp(observation.observedAt);
  const eligibilityAsOf = Date.parse(batch.eligibilityAsOf);
  if (observedAt < eligibilityAsOf) {
    throw new Error('settlement observation cannot precede the eligibility snapshot it executes against');
  }
  if (observation.railReceiptRef !== undefined && !observation.railReceiptRef.trim()) throw new Error('railReceiptRef must be non-empty when present');

  if (observation.state === SettlementAttemptState.Finalized) {
    if (!observation.railReceiptRef?.trim()) {
      throw new Error('every finalized settlement observation requires a rail receipt reference');
    }
    if (observation.settledAmount === undefined) {
      throw new Error('every finalized settlement observation requires an exact settled amount');
    }
    if (observation.settledAmount.amountMinor <= 0n) {
      throw new Error('finalized settlement amount must be positive');
    }
    assertSameCurrency(observation.settledAmount, batch.grossAmount);
    if (observation.settledAmount.amountMinor > batch.grossAmount.amountMinor) {
      throw new Error('finalized settlement amount cannot exceed batch gross amount');
    }
  } else if (observation.settledAmount !== undefined) {
    throw new Error('settledAmount is permitted only on finalized settlement evidence');
  }
}

function observationIdentity(observation: SettlementAttemptObservation): string {
  return [
    observation.observedAt,
    observation.state,
    observation.eligibilityAsOf,
    observation.eligibilityEvidenceRoot,
    observation.railReceiptRef ?? '',
    observation.settledAmount?.amountMinor.toString(10) ?? '',
    observation.settledAmount?.currency ?? '',
    observation.supersedesAttemptId ?? '',
  ].join('\u0000');
}

interface AttemptHistory {
  readonly attemptId: string;
  readonly observations: readonly SettlementAttemptObservation[];
  readonly firstObservedAt: number;
  readonly lastObservedAt: number;
  readonly finalState: SettlementAttemptState;
  readonly supersedesAttemptId?: string;
  readonly finalizedReceiptRef?: string;
  readonly finalizedSettledAmount?: Money;
}

function buildAttemptHistories(batch: DeterministicNettingBatch, observations: readonly SettlementAttemptObservation[]): readonly AttemptHistory[] {
  const byAttempt = new Map<string, SettlementAttemptObservation[]>();
  for (const observation of observations) {
    validateObservation(batch, observation);
    const group = byAttempt.get(observation.attemptId) ?? [];
    group.push(observation);
    byAttempt.set(observation.attemptId, group);
  }

  const histories: AttemptHistory[] = [];
  for (const [attemptId, raw] of byAttempt) {
    const seenExact = new Set<string>();
    const unique = raw.filter(observation => {
      const identity = observationIdentity(observation);
      if (seenExact.has(identity)) return false;
      seenExact.add(identity);
      return true;
    });
    const sorted = [...unique].sort((a, b) => {
      const time = parseTimestamp(a.observedAt) - parseTimestamp(b.observedAt);
      return time || a.state.localeCompare(b.state);
    });

    const first = sorted[0]!;
    for (let index = 1; index < sorted.length; index += 1) {
      const previous = sorted[index - 1]!;
      const current = sorted[index]!;
      if (current.supersedesAttemptId !== undefined) throw new Error('supersedesAttemptId is permitted only on the first observation of an attempt');
      if (previous.observedAt === current.observedAt) throw new Error(`conflicting settlement evidence at identical timestamp for attempt ${attemptId}`);
      if (previous.state === current.state) continue;
      if (!ALLOWED_TRANSITIONS[previous.state].includes(current.state)) throw new Error(`invalid settlement transition ${previous.state} -> ${current.state}`);
    }

    const finalized = sorted.filter(observation => observation.state === SettlementAttemptState.Finalized);
    const finalizedRefs = new Set(finalized.map(observation => observation.railReceiptRef!));
    if (finalizedRefs.size > 1) throw new Error(`finalized rail receipt reference changed within attempt ${attemptId}`);
    const finalizedAmounts = new Set(finalized.map(observation => `${observation.settledAmount!.currency}\u0000${observation.settledAmount!.amountMinor.toString(10)}`));
    if (finalizedAmounts.size > 1) throw new Error(`finalized settled amount changed within attempt ${attemptId}`);
    const finalizedReceiptRef = finalizedRefs.values().next().value as string | undefined;
    const finalizedAmount = finalized[0]?.settledAmount;
    const finalizedSettledAmount = finalizedAmount === undefined
      ? undefined
      : money(finalizedAmount.amountMinor, finalizedAmount.currency);
    const final = sorted[sorted.length - 1]!;
    histories.push(Object.freeze({
      attemptId,
      observations: Object.freeze(sorted),
      firstObservedAt: parseTimestamp(first.observedAt),
      lastObservedAt: parseTimestamp(final.observedAt),
      finalState: final.state,
      ...(first.supersedesAttemptId === undefined ? {} : { supersedesAttemptId: first.supersedesAttemptId }),
      ...(finalizedReceiptRef === undefined ? {} : { finalizedReceiptRef }),
      ...(finalizedSettledAmount === undefined ? {} : { finalizedSettledAmount }),
    }));
  }

  histories.sort((a, b) => a.firstObservedAt - b.firstObservedAt || a.attemptId.localeCompare(b.attemptId));
  const initial = histories[0]!;
  if (initial.supersedesAttemptId !== undefined) throw new Error('initial settlement attempt cannot supersede another attempt');
  for (let index = 1; index < histories.length; index += 1) {
    const prior = histories[index - 1]!;
    const current = histories[index]!;
    if (current.supersedesAttemptId !== prior.attemptId) throw new Error('retry attempt must explicitly supersede the immediately prior attempt');
    if (![SettlementAttemptState.Failed, SettlementAttemptState.Rejected].includes(prior.finalState)) throw new Error('new settlement attempt is forbidden until the prior attempt is retryable');
    if (current.firstObservedAt <= prior.lastObservedAt) throw new Error('retry attempt must begin after the prior terminal observation');
  }
  return Object.freeze(histories);
}

export function reconstructSettlementRecovery(batch: DeterministicNettingBatch, observations: readonly SettlementAttemptObservation[]): Readonly<SettlementRecoveryState> {
  if (!batch.batchId.trim() || !batch.obligationSetRoot.trim() || !batch.eligibilityEvidenceRoot.trim() || !Number.isFinite(Date.parse(batch.eligibilityAsOf))) {
    throw new Error('settlement batch identity must be complete');
  }
  const identity = { batchId: batch.batchId, obligationSetRoot: batch.obligationSetRoot, eligibilityAsOf: batch.eligibilityAsOf, eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot } as const;
  if (observations.length === 0) return Object.freeze({ ...identity, status: 'never_attempted', railCoversBatchGross: false });

  const histories = buildAttemptHistories(batch, observations);
  const latest = histories[histories.length - 1]!;
  const finalObservation = latest.observations[latest.observations.length - 1]!;
  const base = { ...identity, activeAttemptId: latest.attemptId, lastObservedAt: finalObservation.observedAt } as const;

  switch (latest.finalState) {
    case SettlementAttemptState.Finalized: {
      const finalSettledAmount = latest.finalizedSettledAmount!;
      const coversGross = finalSettledAmount.amountMinor === batch.grossAmount.amountMinor;
      return Object.freeze({
        ...base,
        status: coversGross ? 'finalized' : 'partial_finality',
        finalReceiptRef: latest.finalizedReceiptRef!,
        finalSettledAmount,
        ...(coversGross ? {} : { reason: 'rail finality covers less than the deterministic batch; allocation or reconciliation is required before economic discharge' }),
        railCoversBatchGross: coversGross,
      });
    }
    case SettlementAttemptState.Failed:
    case SettlementAttemptState.Rejected:
      return Object.freeze({ ...base, status: 'retryable', railCoversBatchGross: false });
    case SettlementAttemptState.Reversed:
    case SettlementAttemptState.Disputed:
      return Object.freeze({
        ...base,
        status: 'blocked_ambiguous',
        ...(latest.finalizedReceiptRef === undefined ? {} : { finalReceiptRef: latest.finalizedReceiptRef }),
        ...(latest.finalizedSettledAmount === undefined ? {} : { finalSettledAmount: latest.finalizedSettledAmount }),
        reason: `rail state ${latest.finalState} requires reconciliation before any retry`,
        railCoversBatchGross: false,
      });
    default:
      return Object.freeze({ ...base, status: 'in_flight', railCoversBatchGross: false });
  }
}

export function mayStartSettlementAttempt(state: SettlementRecoveryState): boolean {
  return state.status === 'never_attempted' || state.status === 'retryable';
}
