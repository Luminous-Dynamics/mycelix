import type { DeterministicNettingBatch } from './netting.js';

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
  readonly state: SettlementAttemptState;
  readonly observedAt: string;
  readonly railReceiptRef?: string;
  /** Required on the first observation of a retry attempt. */
  readonly supersedesAttemptId?: string;
}

export type SettlementRecoveryStatus =
  | 'never_attempted'
  | 'in_flight'
  | 'retryable'
  | 'finalized'
  | 'blocked_ambiguous';

export interface SettlementRecoveryState {
  readonly batchId: string;
  readonly obligationSetRoot: string;
  readonly status: SettlementRecoveryStatus;
  readonly activeAttemptId?: string;
  readonly lastObservedAt?: string;
  readonly finalReceiptRef?: string;
  readonly reason?: string;
  /** True only after a terminal Finalized observation with no later contradiction. */
  readonly obligationSetSettled: boolean;
}

/**
 * Observers may legitimately miss intermediate provider states after restart,
 * so forward monotonic jumps are accepted. Rollbacks/retries are never inferred.
 */
const ALLOWED_TRANSITIONS: Readonly<Record<SettlementAttemptState, readonly SettlementAttemptState[]>> = {
  [SettlementAttemptState.Authorized]: [
    SettlementAttemptState.Submitted,
    SettlementAttemptState.Accepted,
    SettlementAttemptState.Confirmed,
    SettlementAttemptState.Finalized,
    SettlementAttemptState.Failed,
    SettlementAttemptState.Rejected,
    SettlementAttemptState.Disputed,
  ],
  [SettlementAttemptState.Submitted]: [
    SettlementAttemptState.Accepted,
    SettlementAttemptState.Confirmed,
    SettlementAttemptState.Finalized,
    SettlementAttemptState.Failed,
    SettlementAttemptState.Rejected,
    SettlementAttemptState.Disputed,
  ],
  [SettlementAttemptState.Accepted]: [
    SettlementAttemptState.Confirmed,
    SettlementAttemptState.Finalized,
    SettlementAttemptState.Failed,
    SettlementAttemptState.Rejected,
    SettlementAttemptState.Disputed,
  ],
  [SettlementAttemptState.Confirmed]: [
    SettlementAttemptState.Finalized,
    SettlementAttemptState.Reversed,
    SettlementAttemptState.Disputed,
  ],
  [SettlementAttemptState.Finalized]: [
    SettlementAttemptState.Reversed,
    SettlementAttemptState.Disputed,
  ],
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

function validateObservation(
  batch: DeterministicNettingBatch,
  observation: SettlementAttemptObservation,
): void {
  if (!observation.attemptId.trim()) throw new Error('settlement attemptId must be non-empty');
  if (observation.batchId !== batch.batchId) throw new Error('settlement observation batchId mismatch');
  if (observation.obligationSetRoot !== batch.obligationSetRoot) {
    throw new Error('settlement observation obligationSetRoot mismatch');
  }
  parseTimestamp(observation.observedAt);
  if (observation.railReceiptRef !== undefined && !observation.railReceiptRef.trim()) {
    throw new Error('railReceiptRef must be non-empty when present');
  }
  if (observation.state === SettlementAttemptState.Finalized && !observation.railReceiptRef?.trim()) {
    throw new Error('every finalized settlement observation requires a rail receipt reference');
  }
}

function observationIdentity(observation: SettlementAttemptObservation): string {
  return [
    observation.observedAt,
    observation.state,
    observation.railReceiptRef ?? '',
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
}

function buildAttemptHistories(
  batch: DeterministicNettingBatch,
  observations: readonly SettlementAttemptObservation[],
): readonly AttemptHistory[] {
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
      if (current.supersedesAttemptId !== undefined) {
        throw new Error('supersedesAttemptId is permitted only on the first observation of an attempt');
      }
      if (previous.observedAt === current.observedAt) {
        throw new Error(`conflicting settlement evidence at identical timestamp for attempt ${attemptId}`);
      }
      if (previous.state === current.state) continue; // later observation of the same provider state
      if (!ALLOWED_TRANSITIONS[previous.state].includes(current.state)) {
        throw new Error(`invalid settlement transition ${previous.state} -> ${current.state}`);
      }
    }

    const finalizedRefs = new Set(
      sorted
        .filter(observation => observation.state === SettlementAttemptState.Finalized)
        .map(observation => observation.railReceiptRef!),
    );
    if (finalizedRefs.size > 1) {
      throw new Error(`finalized rail receipt reference changed within attempt ${attemptId}`);
    }

    const final = sorted[sorted.length - 1]!;
    histories.push(Object.freeze({
      attemptId,
      observations: Object.freeze(sorted),
      firstObservedAt: parseTimestamp(first.observedAt),
      lastObservedAt: parseTimestamp(final.observedAt),
      finalState: final.state,
      ...(first.supersedesAttemptId === undefined ? {} : { supersedesAttemptId: first.supersedesAttemptId }),
    }));
  }

  histories.sort((a, b) => a.firstObservedAt - b.firstObservedAt || a.attemptId.localeCompare(b.attemptId));
  for (let index = 1; index < histories.length; index += 1) {
    const prior = histories[index - 1]!;
    const current = histories[index]!;
    if (current.supersedesAttemptId !== prior.attemptId) {
      throw new Error('retry attempt must explicitly supersede the immediately prior attempt');
    }
    if (![SettlementAttemptState.Failed, SettlementAttemptState.Rejected].includes(prior.finalState)) {
      throw new Error('new settlement attempt is forbidden until the prior attempt is retryable');
    }
    if (current.firstObservedAt <= prior.lastObservedAt) {
      throw new Error('retry attempt must begin after the prior terminal observation');
    }
  }

  return Object.freeze(histories);
}

export function reconstructSettlementRecovery(
  batch: DeterministicNettingBatch,
  observations: readonly SettlementAttemptObservation[],
): Readonly<SettlementRecoveryState> {
  if (!batch.batchId.trim() || !batch.obligationSetRoot.trim()) {
    throw new Error('settlement batch identity must be complete');
  }
  if (observations.length === 0) {
    return Object.freeze({
      batchId: batch.batchId,
      obligationSetRoot: batch.obligationSetRoot,
      status: 'never_attempted',
      obligationSetSettled: false,
    });
  }

  const histories = buildAttemptHistories(batch, observations);
  const latest = histories[histories.length - 1]!;
  const finalObservation = latest.observations[latest.observations.length - 1]!;
  const base = {
    batchId: batch.batchId,
    obligationSetRoot: batch.obligationSetRoot,
    activeAttemptId: latest.attemptId,
    lastObservedAt: finalObservation.observedAt,
  } as const;

  switch (latest.finalState) {
    case SettlementAttemptState.Finalized:
      return Object.freeze({
        ...base,
        status: 'finalized',
        finalReceiptRef: finalObservation.railReceiptRef!,
        obligationSetSettled: true,
      });

    case SettlementAttemptState.Failed:
    case SettlementAttemptState.Rejected:
      return Object.freeze({ ...base, status: 'retryable', obligationSetSettled: false });

    case SettlementAttemptState.Reversed:
    case SettlementAttemptState.Disputed:
      return Object.freeze({
        ...base,
        status: 'blocked_ambiguous',
        reason: `rail state ${latest.finalState} requires reconciliation before any retry`,
        obligationSetSettled: false,
      });

    default:
      return Object.freeze({ ...base, status: 'in_flight', obligationSetSettled: false });
  }
}

export function mayStartSettlementAttempt(state: SettlementRecoveryState): boolean {
  return state.status === 'never_attempted' || state.status === 'retryable';
}
