import { describe, expect, it } from 'vitest';
import { buildDeterministicNettingBatches, type DeterministicNettingBatch } from './netting.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { money } from './money.js';
import { compileRoyaltyStatement } from './projection.js';
import { SettlementAttemptState, reconstructSettlementRecovery, type SettlementAttemptObservation } from './recovery.js';
import { SettlementEligibilityCode, type RoyaltyObligation, type SettlementEligibilityObservation, type SettlementEpoch } from './settlement.js';
import { StatementKind } from './statements.js';

const period = { startInclusive: '2026-09-01T00:00:00Z', endExclusive: '2026-10-01T00:00:00Z' };
const complete = { kind: 'complete' as const, through: { usageObservedThrough: '2026-10-01T00:00:00Z', rightsResolvedThrough: '2026-10-01T00:00:00Z', settlementsObservedThrough: '2026-10-01T00:00:00Z' } };
const epoch: SettlementEpoch = { id: 'epoch:2026-09', cutoff: '2026-09-30T23:59:59Z', eligibilityAsOf: '2026-10-01T00:00:00Z', minimumPayout: money(100n, 'USD') };

type ObligationOverride = { readonly observedAt?: string; readonly usageEvidenceRef?: string };
function obligation(id: string, amountMinor: bigint, extra: ObligationOverride = {}): RoyaltyObligation {
  return createRoyaltyObligationAuthority({ id, beneficiaryId: 'artist:1', amount: money(amountMinor, 'USD'), observedAt: extra.observedAt ?? '2026-09-10T12:00:00Z', provenance: { usageEvidenceRef: extra.usageEvidenceRef ?? `usage:${id}`, rightsResolutionRef: `rights:${id}`, economicTermsRef: 'terms:v1' } });
}
function eligibility(obligationId: string, code: SettlementEligibilityObservation['code'] = SettlementEligibilityCode.Eligible, extra: Partial<Omit<SettlementEligibilityObservation, 'obligationId' | 'code'>> = {}): SettlementEligibilityObservation {
  return { id: `elig:${obligationId}`, obligationId, code, sourceRef: 'eligibility-authority:v1', observedAt: '2026-09-30T00:00:00Z', ...extra };
}
function defaultEligibility(obligations: readonly RoyaltyObligation[]): SettlementEligibilityObservation[] { return obligations.map(item => eligibility(item.id)); }
function compile(obligations: readonly RoyaltyObligation[], extra: Partial<Parameters<typeof compileRoyaltyStatement>[0]> = {}) {
  const { eligibilityObservations = defaultEligibility(obligations), ...rest } = extra;
  return compileRoyaltyStatement({ statementId: 'statement:2026-09:artist:1', kind: StatementKind.Periodic, beneficiaryId: 'artist:1', period, asOf: '2026-10-02T00:00:00Z', completeness: complete, settlementEpoch: epoch, obligations, eligibilityObservations, ...rest });
}
function settlementObservation(batch: DeterministicNettingBatch, state: SettlementAttemptState, observedAt: string, extra: Partial<SettlementAttemptObservation> = {}): SettlementAttemptObservation {
  return { attemptId: 'attempt:1', batchId: batch.batchId, obligationSetRoot: batch.obligationSetRoot, eligibilityAsOf: batch.eligibilityAsOf, eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot, state, observedAt, ...extra };
}

describe('royalty statement compiler', () => {
  it('keeps an unavailable payout route as held debt rather than erasing it', () => {
    const debt = obligation('obl:1', 500n);
    const statement = compile([debt], { eligibilityObservations: [eligibility('obl:1', SettlementEligibilityCode.AwaitingPayeeRoute)] });
    expect(statement.gross.amountMinor).toBe(500n); expect(statement.held.amountMinor).toBe(500n); expect(statement.netPayable.amountMinor).toBe(0n); expect(statement.paid.amountMinor).toBe(0n);
  });
  it('fails closed when an in-epoch obligation has no eligibility evidence', () => {
    const statement = compile([obligation('obl:1', 500n)], { eligibilityObservations: [] });
    expect(statement.held.amountMinor).toBe(500n); expect(statement.netPayable.amountMinor).toBe(0n);
  });
  it('carries sub-threshold royalties forward without creating a zero-value batch', () => {
    const statement = compile([obligation('obl:1', 99n)]); expect(statement.held.amountMinor).toBe(99n); expect(statement.netPayable.amountMinor).toBe(0n);
  });
  it('holds obligations observed after the settlement cutoff for the next epoch', () => {
    const obligations = [obligation('obl:early', 300n, { observedAt: '2026-09-10T00:00:00Z' }), obligation('obl:late', 200n, { observedAt: '2026-09-20T00:00:00Z' })];
    const statement = compile(obligations, { settlementEpoch: { ...epoch, cutoff: '2026-09-15T23:59:59Z' } });
    expect(statement.gross.amountMinor).toBe(500n); expect(statement.held.amountMinor).toBe(200n); expect(statement.netPayable.amountMinor).toBe(300n);
  });
  it('handles an all-post-cutoff statement without promoting value to payable', () => {
    const statement = compile([obligation('obl:late', 500n, { observedAt: '2026-09-20T00:00:00Z' })], { settlementEpoch: { ...epoch, cutoff: '2026-09-15T23:59:59Z' } });
    expect(statement.held.amountMinor).toBe(500n); expect(statement.netPayable.amountMinor).toBe(0n);
  });
  it('rejects obligations observed after immutable statement asOf', () => {
    expect(() => compile([obligation('obl:future', 500n, { observedAt: '2026-09-20T00:00:00Z' })], { asOf: '2026-09-15T00:00:00Z', settlementEpoch: { ...epoch, eligibilityAsOf: '2026-09-15T00:00:00Z' }, completeness: { kind: 'partial', through: { usageObservedThrough: '2026-09-15T00:00:00Z', rightsResolvedThrough: '2026-09-15T00:00:00Z', settlementsObservedThrough: '2026-09-15T00:00:00Z' }, missingSources: ['usage:future'] } })).toThrow(/observed after the statement asOf/);
  });
  it('rejects eligibilityAsOf later than statement asOf', () => {
    expect(() => compile([obligation('obl:1', 500n)], { asOf: '2026-10-01T00:00:00Z', settlementEpoch: { ...epoch, eligibilityAsOf: '2026-10-01T00:00:01Z' } })).toThrow(/eligibilityAsOf cannot be later/);
  });
  it('does not let future eligibility evidence leak backward', () => {
    const statement = compile([obligation('obl:1', 500n)], { settlementEpoch: { ...epoch, eligibilityAsOf: '2026-09-20T00:00:00Z' }, eligibilityObservations: [eligibility('obl:1', SettlementEligibilityCode.Eligible, { id: 'elig:future', observedAt: '2026-09-21T00:00:00Z' })] });
    expect(statement.held.amountMinor).toBe(500n); expect(statement.netPayable.amountMinor).toBe(0n);
  });
  it('applies only deductions observable by statement asOf', () => {
    const statement = compile([obligation('obl:1', 500n)], { deductions: [{ id: 'deduction:1', beneficiaryId: 'artist:1', amount: money(50n, 'USD'), basis: 'tax:withholding:v1', observedAt: '2026-10-01T12:00:00Z' }] });
    expect(statement.deductions.amountMinor).toBe(50n); expect(statement.netPayable.amountMinor).toBe(450n);
  });
  it('rejects deduction evidence observed after immutable statement asOf', () => {
    expect(() => compile([obligation('obl:1', 500n)], { deductions: [{ id: 'deduction:future', beneficiaryId: 'artist:1', amount: money(50n, 'USD'), basis: 'tax:withholding:v1', observedAt: '2026-10-03T00:00:00Z' }] })).toThrow(/deduction was observed after/);
  });
  it('counts value as paid only after settlement finality', () => {
    const obligations = [obligation('obl:1', 500n)]; const eligibilityObservations = defaultEligibility(obligations); const batch = buildDeterministicNettingBatches(obligations, epoch, eligibilityObservations)[0]!;
    const inFlight = reconstructSettlementRecovery(batch, [settlementObservation(batch, SettlementAttemptState.Submitted, '2026-10-01T00:01:00Z')]);
    expect(compile(obligations, { eligibilityObservations, settlements: [{ batch, recovery: inFlight, settledAmount: money(500n, 'USD') }] }).paid.amountMinor).toBe(0n);
    const finalized = reconstructSettlementRecovery(batch, [settlementObservation(batch, SettlementAttemptState.Submitted, '2026-10-01T00:01:00Z'), settlementObservation(batch, SettlementAttemptState.Finalized, '2026-10-01T00:02:00Z', { railReceiptRef: 'rail:receipt:1' })]);
    expect(compile(obligations, { eligibilityObservations, settlements: [{ batch, recovery: finalized, settledAmount: money(500n, 'USD') }] }).paid.amountMinor).toBe(500n);
  });
  it('rejects recovery bound to a different eligibility snapshot', () => {
    const obligations = [obligation('obl:1', 500n)]; const eligibilityObservations = defaultEligibility(obligations); const batch = buildDeterministicNettingBatches(obligations, epoch, eligibilityObservations)[0]!; const recovery = reconstructSettlementRecovery(batch, []);
    expect(() => compile(obligations, { eligibilityObservations, settlements: [{ batch, recovery: { ...recovery, eligibilityEvidenceRoot: 'f'.repeat(64) }, settledAmount: money(500n, 'USD') }] })).toThrow(/eligibility snapshot/);
  });
  it('rejects forged batch economic fields', () => {
    const obligations = [obligation('obl:1', 500n)]; const eligibilityObservations = defaultEligibility(obligations); const batch = buildDeterministicNettingBatches(obligations, epoch, eligibilityObservations)[0]!; const forgedBatch = { ...batch, grossAmount: money(501n, 'USD') };
    const recovery = reconstructSettlementRecovery(forgedBatch, [{ attemptId: 'attempt:forged', batchId: forgedBatch.batchId, obligationSetRoot: forgedBatch.obligationSetRoot, eligibilityAsOf: forgedBatch.eligibilityAsOf, eligibilityEvidenceRoot: forgedBatch.eligibilityEvidenceRoot, state: SettlementAttemptState.Finalized, observedAt: '2026-10-01T00:02:00Z', railReceiptRef: 'rail:receipt:forged' }]);
    expect(() => compile(obligations, { eligibilityObservations, settlements: [{ batch: forgedBatch, recovery, settledAmount: money(500n, 'USD') }] })).toThrow(/deterministic authoritative reconstruction/);
  });
  it('rejects settlement observations after statement asOf', () => {
    const obligations = [obligation('obl:1', 500n)]; const eligibilityObservations = defaultEligibility(obligations); const batch = buildDeterministicNettingBatches(obligations, epoch, eligibilityObservations)[0]!; const recovery = reconstructSettlementRecovery(batch, [settlementObservation(batch, SettlementAttemptState.Finalized, '2026-10-03T00:00:00Z', { railReceiptRef: 'rail:receipt:future' })]);
    expect(() => compile(obligations, { eligibilityObservations, settlements: [{ batch, recovery, settledAmount: money(500n, 'USD') }] })).toThrow(/later than statement asOf/);
  });
  it('produces obligation roots independent of input order', () => {
    expect(compile([obligation('obl:b', 200n), obligation('obl:a', 300n)]).obligationRoot).toBe(compile([obligation('obl:a', 300n), obligation('obl:b', 200n)]).obligationRoot);
  });
  it('changes obligation root when immutable provenance changes', () => {
    expect(compile([obligation('obl:1', 500n, { usageEvidenceRef: 'usage:a' })]).obligationRoot).not.toBe(compile([obligation('obl:1', 500n, { usageEvidenceRef: 'usage:b' })]).obligationRoot);
  });
  it('changes settlement root when eligibility provenance changes without changing debt root', () => {
    const debt = obligation('obl:1', 500n); const first = compile([debt], { eligibilityObservations: [eligibility('obl:1', SettlementEligibilityCode.Eligible, { sourceRef: 'route-registry:v1' })] }); const second = compile([debt], { eligibilityObservations: [eligibility('obl:1', SettlementEligibilityCode.Eligible, { sourceRef: 'route-registry:v2' })] });
    expect(second.obligationRoot).toBe(first.obligationRoot); expect(second.settlementRoot).not.toBe(first.settlementRoot);
  });
  it('rejects obligations outside statement period', () => {
    expect(() => compile([obligation('obl:late', 500n, { observedAt: '2026-10-01T00:00:00Z' })])).toThrow(/outside the statement period/);
  });
});
