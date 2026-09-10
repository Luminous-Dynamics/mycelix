import { buildMerkleCommitment, type Digest } from './merkle.js';
import { money } from './money.js';
import { assertRoyaltyObligationAuthority, type RoyaltyObligationAuthority } from './obligation-authority.js';
import type { StatementDeduction } from './projection.js';
import { SettlementAttemptState, type SettlementAttemptObservation } from './recovery.js';
import {
  isObservableSettlementEligibilityCode,
  type ObservableSettlementEligibilityCode,
  type SettlementEligibilityObservation,
} from './settlement.js';
import { assertStatementArithmetic, type RoyaltyStatementSnapshot } from './statements.js';

const DIGEST_RE = /^[0-9a-f]{64}$/;
const SETTLEMENT_STATES: ReadonlySet<string> = new Set(Object.values(SettlementAttemptState));

export interface PersistedRoyaltyObligationRecord {
  readonly id: string;
  readonly beneficiaryId: string;
  readonly amountMinor: string;
  readonly currency: string;
  readonly observedAt: string;
  readonly usageEvidenceRef: string;
  readonly rightsResolutionRef: string;
  readonly economicTermsRef: string;
  readonly obligationRoot: Digest;
}

export interface PersistedEligibilityObservationRecord {
  readonly id: string;
  readonly obligationId: string;
  readonly code: ObservableSettlementEligibilityCode;
  readonly reason?: string;
  readonly sourceRef: string;
  readonly observedAt: string;
  readonly observationRoot: Digest;
}

export interface PersistedDeductionRecord {
  readonly id: string;
  readonly beneficiaryId: string;
  readonly amountMinor: string;
  readonly currency: string;
  readonly basis: string;
  readonly authorityRef: string;
  readonly observedAt: string;
  readonly deductionRoot: Digest;
}

export interface PersistedSettlementObservationRecord {
  readonly id: string;
  readonly attemptId: string;
  readonly batchId: string;
  readonly obligationSetRoot: Digest;
  readonly eligibilityAsOf: string;
  readonly eligibilityEvidenceRoot: Digest;
  readonly state: SettlementAttemptState;
  readonly observedAt: string;
  readonly railReceiptRef?: string;
  readonly settledAmountMinor?: string;
  readonly settledCurrency?: string;
  readonly supersedesAttemptId?: string;
  readonly observationRoot: Digest;
}

export interface PersistedStatementSnapshotRecord {
  readonly statementId: string;
  readonly kind: RoyaltyStatementSnapshot['kind'];
  readonly predecessorStatementId?: string;
  readonly beneficiaryId: string;
  readonly periodStart: string;
  readonly periodEnd: string;
  readonly asOf: string;
  readonly obligationRoot: string;
  readonly adjustmentRoot: string;
  readonly settlementRoot: string;
  readonly grossMinor: string;
  readonly heldMinor: string;
  readonly deductionMinor: string;
  readonly netPayableMinor: string;
  readonly paidMinor: string;
  readonly currency: string;
  readonly completenessKind: RoyaltyStatementSnapshot['completeness']['kind'];
  readonly completenessData: RoyaltyStatementSnapshot['completeness'];
  readonly snapshotRoot: Digest;
}

function required(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

function digest(label: string, value: string): Digest {
  const normalized = required(label, value);
  if (!DIGEST_RE.test(normalized)) throw new Error(`${label} must be a lowercase SHA-256 digest`);
  return normalized;
}

function timestamp(label: string, value: string): string {
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error(`${label} must be a valid timestamp`);
  return new Date(parsed).toISOString();
}

function singleRecordRoot(recordType: string, value: Readonly<Record<string, unknown>>): Digest {
  return buildMerkleCommitment([{ recordType, ...value }]).root;
}

export function projectRoyaltyObligationRecord(
  obligation: RoyaltyObligationAuthority,
): Readonly<PersistedRoyaltyObligationRecord> {
  assertRoyaltyObligationAuthority(obligation);
  return Object.freeze({
    id: obligation.id,
    beneficiaryId: obligation.beneficiaryId,
    amountMinor: obligation.amount.amountMinor.toString(10),
    currency: obligation.amount.currency,
    observedAt: obligation.observedAt,
    usageEvidenceRef: obligation.provenance.usageEvidenceRef,
    rightsResolutionRef: obligation.provenance.rightsResolutionRef,
    economicTermsRef: obligation.provenance.economicTermsRef,
    obligationRoot: obligation.authorityRoot,
  });
}

export function projectEligibilityObservationRecord(
  input: SettlementEligibilityObservation,
): Readonly<PersistedEligibilityObservationRecord> {
  if (!isObservableSettlementEligibilityCode(input.code)) {
    throw new Error(`eligibility observation cannot persist compiler-only code: ${String(input.code)}`);
  }
  const id = required('eligibility observation id', input.id);
  const obligationId = required('eligibility obligationId', input.obligationId);
  const sourceRef = required('eligibility sourceRef', input.sourceRef);
  const observedAt = timestamp('eligibility observedAt', input.observedAt);
  const reason = input.reason === undefined ? undefined : required('eligibility reason', input.reason);
  const committed = Object.freeze({
    id,
    obligationId,
    code: input.code,
    ...(reason === undefined ? {} : { reason }),
    sourceRef,
    observedAt,
  });
  return Object.freeze({
    ...committed,
    observationRoot: singleRecordRoot('royalty_eligibility_observation_v1', committed),
  });
}

export function projectDeductionRecord(
  deduction: StatementDeduction,
  authorityRef: string,
): Readonly<PersistedDeductionRecord> {
  const id = required('deduction id', deduction.id);
  const beneficiaryId = required('deduction beneficiaryId', deduction.beneficiaryId);
  const basis = required('deduction basis', deduction.basis);
  const normalizedAuthorityRef = required('deduction authorityRef', authorityRef);
  const observedAt = timestamp('deduction observedAt', deduction.observedAt);
  if (deduction.amount.amountMinor < 0n) throw new Error('deduction amount must be non-negative');
  const committed = Object.freeze({
    id,
    beneficiaryId,
    amountMinor: deduction.amount.amountMinor,
    currency: deduction.amount.currency,
    basis,
    authorityRef: normalizedAuthorityRef,
    observedAt,
  });
  return Object.freeze({
    id,
    beneficiaryId,
    amountMinor: deduction.amount.amountMinor.toString(10),
    currency: deduction.amount.currency,
    basis,
    authorityRef: normalizedAuthorityRef,
    observedAt,
    deductionRoot: singleRecordRoot('royalty_deduction_v1', committed),
  });
}

export function projectSettlementObservationRecord(
  idInput: string,
  observation: SettlementAttemptObservation,
): Readonly<PersistedSettlementObservationRecord> {
  const id = required('settlement observation id', idInput);
  const attemptId = required('settlement attemptId', observation.attemptId);
  const batchId = required('settlement batchId', observation.batchId);
  const obligationSetRoot = digest('settlement obligationSetRoot', observation.obligationSetRoot);
  const eligibilityAsOf = timestamp('settlement eligibilityAsOf', observation.eligibilityAsOf);
  const eligibilityEvidenceRoot = digest('settlement eligibilityEvidenceRoot', observation.eligibilityEvidenceRoot);
  const observedAt = timestamp('settlement observedAt', observation.observedAt);
  if (!SETTLEMENT_STATES.has(observation.state)) {
    throw new Error(`unsupported settlement state: ${String(observation.state)}`);
  }
  if (Date.parse(observedAt) < Date.parse(eligibilityAsOf)) {
    throw new Error('settlement observation cannot predate its eligibility snapshot');
  }
  const railReceiptRef = observation.railReceiptRef === undefined
    ? undefined
    : required('railReceiptRef', observation.railReceiptRef);
  const supersedesAttemptId = observation.supersedesAttemptId === undefined
    ? undefined
    : required('supersedesAttemptId', observation.supersedesAttemptId);
  const settledAmount = observation.settledAmount === undefined
    ? undefined
    : money(observation.settledAmount.amountMinor, observation.settledAmount.currency);
  if (settledAmount && settledAmount.amountMinor < 0n) throw new Error('settled amount must be non-negative');
  if (observation.state === SettlementAttemptState.Finalized) {
    if (railReceiptRef === undefined) throw new Error('finalized settlement observation requires railReceiptRef');
    if (settledAmount === undefined) throw new Error('finalized settlement observation requires settledAmount');
  } else if (settledAmount !== undefined) {
    throw new Error('settledAmount is permitted only on finalized settlement evidence');
  }
  const committed = Object.freeze({
    id,
    attemptId,
    batchId,
    obligationSetRoot,
    eligibilityAsOf,
    eligibilityEvidenceRoot,
    state: observation.state,
    observedAt,
    ...(railReceiptRef === undefined ? {} : { railReceiptRef }),
    ...(settledAmount === undefined ? {} : {
      settledAmountMinor: settledAmount.amountMinor,
      settledCurrency: settledAmount.currency,
    }),
    ...(supersedesAttemptId === undefined ? {} : { supersedesAttemptId }),
  });
  return Object.freeze({
    ...committed,
    ...(settledAmount === undefined ? {} : { settledAmountMinor: settledAmount.amountMinor.toString(10) }),
    observationRoot: singleRecordRoot('settlement_attempt_observation_v3', committed),
  });
}

export function projectStatementSnapshotRecord(
  statement: RoyaltyStatementSnapshot,
): Readonly<PersistedStatementSnapshotRecord> {
  assertStatementArithmetic(statement);
  const committed = Object.freeze({
    statementId: required('statementId', statement.statementId),
    kind: statement.kind,
    ...(statement.predecessorStatementId === undefined
      ? {}
      : { predecessorStatementId: required('predecessorStatementId', statement.predecessorStatementId) }),
    beneficiaryId: required('statement beneficiaryId', statement.beneficiaryId),
    periodStart: timestamp('statement periodStart', statement.period.startInclusive),
    periodEnd: timestamp('statement periodEnd', statement.period.endExclusive),
    asOf: timestamp('statement asOf', statement.asOf),
    obligationRoot: required('statement obligationRoot', statement.obligationRoot),
    adjustmentRoot: required('statement adjustmentRoot', statement.adjustmentRoot),
    settlementRoot: required('statement settlementRoot', statement.settlementRoot),
    grossMinor: statement.gross.amountMinor,
    heldMinor: statement.held.amountMinor,
    deductionMinor: statement.deductions.amountMinor,
    netPayableMinor: statement.netPayable.amountMinor,
    paidMinor: statement.paid.amountMinor,
    currency: statement.gross.currency,
    completenessKind: statement.completeness.kind,
    completenessData: statement.completeness,
  });
  return Object.freeze({
    ...committed,
    grossMinor: statement.gross.amountMinor.toString(10),
    heldMinor: statement.held.amountMinor.toString(10),
    deductionMinor: statement.deductions.amountMinor.toString(10),
    netPayableMinor: statement.netPayable.amountMinor.toString(10),
    paidMinor: statement.paid.amountMinor.toString(10),
    snapshotRoot: singleRecordRoot('royalty_statement_snapshot_v1', committed),
  });
}
