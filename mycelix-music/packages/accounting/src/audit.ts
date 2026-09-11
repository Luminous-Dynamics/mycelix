import { buildMerkleCommitment, type Digest } from './merkle.js';
import { subtractMoney, type Money } from './money.js';
import { compileRoyaltyStatement, type CompileRoyaltyStatementInput } from './projection.js';
import { assertConservation, type ConservationSummary } from './reconciliation.js';
import {
  requireAnchoredSettlementAllocationLineageCheckpointAuthority,
  type AnchoredSettlementAllocationLineageCheckpointAuthority,
} from './settlement-allocation-lineage-checkpoint-trust-anchor.js';
import { assertStatementArithmetic, type RoyaltyStatementSnapshot } from './statements.js';

export interface CompilerIdentity {
  readonly id: string;
  readonly version: string;
  readonly buildDigest: string;
}

/** Historical v1 capsules remain verifiable, but new capsule creation is v2 only. */
export interface EconomicAuditCapsuleV1 {
  readonly protocolVersion: 1;
  readonly usageCommitment: Digest;
  readonly repertoireSnapshot: Digest;
  readonly rightsPolicy: Digest;
  readonly economicTerms: Digest;
  readonly rightsResolutionRoot: Digest;
  readonly obligationRoot: Digest;
  readonly adjustmentRoot: Digest;
  readonly nettingRoot: Digest;
  readonly settlementPlanRoot: Digest;
  readonly paymentReceiptRoot: Digest;
  readonly conservationProof: ConservationSummary;
  readonly generatedBy: CompilerIdentity;
}

export interface AnchoredSettlementTrustAuditEvidence {
  readonly batchId: string;
  readonly checkpointRoot: Digest;
  readonly checkpointVerificationReceiptRoot: Digest;
  readonly anchoredVerificationReceiptRoot: Digest;
  readonly trustBundleRoot: Digest;
  readonly trustBundleAttestationRoot: Digest;
  readonly trustAnchorId: string;
  readonly trustPolicySequence: string;
  readonly verifiedAt: string;
}

export interface EconomicAuditCapsuleV2 {
  readonly protocolVersion: 2;
  /** Complete immutable statement snapshot, not only selected roots. */
  readonly statement: RoyaltyStatementSnapshot;
  readonly statementSnapshotRoot: Digest;
  readonly usageCommitment: Digest;
  readonly repertoireSnapshot: Digest;
  readonly rightsPolicy: Digest;
  readonly economicTerms: Digest;
  readonly rightsResolutionRoot: Digest;
  readonly nettingRoot: Digest;
  readonly settlementPlanRoot: Digest;
  readonly paymentReceiptRoot: Digest;
  /** Complete anchored-trust set derived from the exact statement compiler input. */
  readonly anchoredSettlementTrust: readonly AnchoredSettlementTrustAuditEvidence[];
  readonly anchoredSettlementTrustRoot: Digest;
  /** Derived from the embedded immutable statement; never caller supplied in v2 creation. */
  readonly conservationProof: ConservationSummary;
  readonly generatedBy: CompilerIdentity;
}

export type EconomicAuditCapsule = EconomicAuditCapsuleV1 | EconomicAuditCapsuleV2;

export interface CreateEconomicAuditCapsuleV2Input {
  /**
   * The capsule compiles this exact input itself. This prevents callers from
   * pairing one statement with an omitted or unrelated trust-evidence set.
   */
  readonly statementInput: CompileRoyaltyStatementInput;
  readonly usageCommitment: Digest;
  readonly repertoireSnapshot: Digest;
  readonly rightsPolicy: Digest;
  readonly economicTerms: Digest;
  readonly rightsResolutionRoot: Digest;
  readonly nettingRoot: Digest;
  readonly settlementPlanRoot: Digest;
  readonly paymentReceiptRoot: Digest;
  readonly generatedBy: CompilerIdentity;
}

const SHA256_HEX = /^[0-9a-f]{64}$/;
const CANONICAL_POSITIVE_UINT = /^[1-9][0-9]*$/;

const V1_DIGEST_FIELDS: ReadonlyArray<keyof Pick<
  EconomicAuditCapsuleV1,
  | 'usageCommitment'
  | 'repertoireSnapshot'
  | 'rightsPolicy'
  | 'economicTerms'
  | 'rightsResolutionRoot'
  | 'obligationRoot'
  | 'adjustmentRoot'
  | 'nettingRoot'
  | 'settlementPlanRoot'
  | 'paymentReceiptRoot'
>> = [
  'usageCommitment',
  'repertoireSnapshot',
  'rightsPolicy',
  'economicTerms',
  'rightsResolutionRoot',
  'obligationRoot',
  'adjustmentRoot',
  'nettingRoot',
  'settlementPlanRoot',
  'paymentReceiptRoot',
];

const V2_DIGEST_FIELDS: ReadonlyArray<keyof Pick<
  EconomicAuditCapsuleV2,
  | 'statementSnapshotRoot'
  | 'usageCommitment'
  | 'repertoireSnapshot'
  | 'rightsPolicy'
  | 'economicTerms'
  | 'rightsResolutionRoot'
  | 'nettingRoot'
  | 'settlementPlanRoot'
  | 'paymentReceiptRoot'
  | 'anchoredSettlementTrustRoot'
>> = [
  'statementSnapshotRoot',
  'usageCommitment',
  'repertoireSnapshot',
  'rightsPolicy',
  'economicTerms',
  'rightsResolutionRoot',
  'nettingRoot',
  'settlementPlanRoot',
  'paymentReceiptRoot',
  'anchoredSettlementTrustRoot',
];

function required(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

function timestamp(label: string, value: string): string {
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error(`${label} must be a valid timestamp`);
  return new Date(parsed).toISOString();
}

function sha256(label: string, value: string): Digest {
  if (!SHA256_HEX.test(value)) throw new Error(`${label} must be a lowercase SHA-256 digest`);
  return value;
}

function assertCompilerIdentity(identity: CompilerIdentity, requireDigest: boolean): void {
  required('compiler id', identity.id);
  required('compiler version', identity.version);
  if (requireDigest) sha256('compiler buildDigest', identity.buildDigest);
  else required('compiler buildDigest', identity.buildDigest);
}

function statementSnapshotRoot(statement: RoyaltyStatementSnapshot): Digest {
  return buildMerkleCommitment([{
    recordType: 'economic_audit_royalty_statement_snapshot_v1',
    statement,
  }]).root;
}

function trustSetRoot(entries: readonly AnchoredSettlementTrustAuditEvidence[]): Digest {
  return buildMerkleCommitment([{
    recordType: 'economic_audit_anchored_settlement_trust_set_v1',
    entries,
  }]).root;
}

function assertAnchoredTrustEvidence(
  evidence: AnchoredSettlementTrustAuditEvidence,
  statementAsOf: string,
): void {
  required('anchored settlement trust batchId', evidence.batchId);
  sha256('anchored settlement trust checkpointRoot', evidence.checkpointRoot);
  sha256('anchored settlement trust checkpointVerificationReceiptRoot', evidence.checkpointVerificationReceiptRoot);
  sha256('anchored settlement trust anchoredVerificationReceiptRoot', evidence.anchoredVerificationReceiptRoot);
  sha256('anchored settlement trust trustBundleRoot', evidence.trustBundleRoot);
  sha256('anchored settlement trust trustBundleAttestationRoot', evidence.trustBundleAttestationRoot);
  required('anchored settlement trust trustAnchorId', evidence.trustAnchorId);
  if (!CANONICAL_POSITIVE_UINT.test(evidence.trustPolicySequence)) {
    throw new Error('anchored settlement trust policy sequence must be canonical positive-integer text');
  }
  const verifiedAt = timestamp('anchored settlement trust verifiedAt', evidence.verifiedAt);
  if (Date.parse(verifiedAt) !== Date.parse(statementAsOf)) {
    throw new Error('anchored settlement trust verifiedAt must equal statementAsOf');
  }
}

function derivedConservation(statement: RoyaltyStatementSnapshot): Readonly<ConservationSummary> {
  const proof = Object.freeze({
    inputTotal: Object.freeze({ ...statement.gross }),
    distributedTotal: Object.freeze({ ...statement.paid }),
    heldTotal: Object.freeze({ ...statement.held }),
    deductionTotal: Object.freeze({ ...statement.deductions }),
    residualTotal: Object.freeze({ ...subtractMoney(statement.netPayable, statement.paid) }),
  });
  assertConservation(proof);
  return proof;
}

function sameMoney(left: Money, right: Money): boolean {
  return left.currency === right.currency && left.amountMinor === right.amountMinor;
}

function assertConservationMatchesStatement(
  proof: ConservationSummary,
  statement: RoyaltyStatementSnapshot,
): void {
  assertConservation(proof);
  const expected = derivedConservation(statement);
  if (
    !sameMoney(proof.inputTotal, expected.inputTotal)
    || !sameMoney(proof.distributedTotal, expected.distributedTotal)
    || !sameMoney(proof.heldTotal, expected.heldTotal)
    || !sameMoney(proof.deductionTotal, expected.deductionTotal)
    || !sameMoney(proof.residualTotal, expected.residualTotal)
  ) {
    throw new Error('audit capsule conservation proof does not match embedded statement economics');
  }
}

function assertV1(capsule: EconomicAuditCapsuleV1): void {
  for (const field of V1_DIGEST_FIELDS) {
    if (!capsule[field].trim()) throw new Error(`${field} must be non-empty`);
  }
  assertCompilerIdentity(capsule.generatedBy, false);
  assertConservation(capsule.conservationProof);
}

function assertV2(capsule: EconomicAuditCapsuleV2): void {
  assertStatementArithmetic(capsule.statement);
  required('statementId', capsule.statement.statementId);
  const statementAsOf = timestamp('statement asOf', capsule.statement.asOf);
  for (const field of V2_DIGEST_FIELDS) sha256(field, capsule[field]);
  assertCompilerIdentity(capsule.generatedBy, true);
  assertConservationMatchesStatement(capsule.conservationProof, capsule.statement);

  const expectedStatementRoot = statementSnapshotRoot(capsule.statement);
  if (capsule.statementSnapshotRoot !== expectedStatementRoot) {
    throw new Error('statementSnapshotRoot does not match embedded immutable statement');
  }

  let previousBatchId: string | undefined;
  const seenBatches = new Set<string>();
  for (const evidence of capsule.anchoredSettlementTrust) {
    assertAnchoredTrustEvidence(evidence, statementAsOf);
    if (seenBatches.has(evidence.batchId)) {
      throw new Error(`duplicate anchored settlement trust batchId: ${evidence.batchId}`);
    }
    seenBatches.add(evidence.batchId);
    if (previousBatchId !== undefined && previousBatchId.localeCompare(evidence.batchId) >= 0) {
      throw new Error('anchored settlement trust evidence must be strictly sorted by batchId');
    }
    previousBatchId = evidence.batchId;
  }
  const expectedTrustRoot = trustSetRoot(capsule.anchoredSettlementTrust);
  if (capsule.anchoredSettlementTrustRoot !== expectedTrustRoot) {
    throw new Error('anchored settlement trust root does not match canonical evidence set');
  }
}

export function assertEconomicAuditCapsule(capsule: EconomicAuditCapsule): void {
  if (capsule.protocolVersion === 1) {
    assertV1(capsule);
    return;
  }
  if (capsule.protocolVersion === 2) {
    assertV2(capsule);
    return;
  }
  throw new Error('unsupported economic audit capsule version');
}

function projectAnchoredTrustEvidence(
  batchId: string,
  authorityInput: AnchoredSettlementAllocationLineageCheckpointAuthority,
  statementAsOf: string,
): Readonly<AnchoredSettlementTrustAuditEvidence> {
  const authority = requireAnchoredSettlementAllocationLineageCheckpointAuthority(authorityInput);
  const checkpoint = authority.checkpointAuthority.checkpoint;
  if (checkpoint.batchId !== batchId) {
    throw new Error('audit trust authority checkpoint batch does not match statement settlement batch');
  }
  if (Date.parse(authority.receipt.verifiedAt) !== Date.parse(statementAsOf)) {
    throw new Error('audit trust authority verification time must equal statementAsOf');
  }
  return Object.freeze({
    batchId,
    checkpointRoot: checkpoint.checkpointRoot,
    checkpointVerificationReceiptRoot: authority.receipt.checkpointVerificationReceiptRoot,
    anchoredVerificationReceiptRoot: authority.receipt.receiptRoot,
    trustBundleRoot: authority.receipt.trustBundleRoot,
    trustBundleAttestationRoot: authority.receipt.trustBundleAttestationRoot,
    trustAnchorId: authority.receipt.trustAnchorId,
    trustPolicySequence: authority.receipt.trustPolicySequence,
    verifiedAt: timestamp('audit trust authority verifiedAt', authority.receipt.verifiedAt),
  });
}

/**
 * New audit capsules are compiled from the exact statement input and are always
 * protocol v2. Historical v1 capsules remain verifiable through
 * assertEconomicAuditCapsule/economicAuditCapsuleDigest.
 */
export function createEconomicAuditCapsule(
  input: CreateEconomicAuditCapsuleV2Input,
): Readonly<EconomicAuditCapsuleV2> {
  const statement = compileRoyaltyStatement(input.statementInput);
  const statementAsOf = timestamp('statement asOf', statement.asOf);
  const anchoredSettlementTrust = Object.freeze(
    [...(input.statementInput.settlements ?? [])]
      .filter(settlement => settlement.anchoredTrust !== undefined)
      .map(settlement => projectAnchoredTrustEvidence(
        settlement.batch.batchId,
        settlement.anchoredTrust!,
        statementAsOf,
      ))
      .sort((left, right) => left.batchId.localeCompare(right.batchId)),
  );

  const conservationProof = derivedConservation(statement);
  const capsule: EconomicAuditCapsuleV2 = Object.freeze({
    protocolVersion: 2,
    statement,
    statementSnapshotRoot: statementSnapshotRoot(statement),
    usageCommitment: input.usageCommitment,
    repertoireSnapshot: input.repertoireSnapshot,
    rightsPolicy: input.rightsPolicy,
    economicTerms: input.economicTerms,
    rightsResolutionRoot: input.rightsResolutionRoot,
    nettingRoot: input.nettingRoot,
    settlementPlanRoot: input.settlementPlanRoot,
    paymentReceiptRoot: input.paymentReceiptRoot,
    anchoredSettlementTrust,
    anchoredSettlementTrustRoot: trustSetRoot(anchoredSettlementTrust),
    conservationProof,
    generatedBy: Object.freeze({ ...input.generatedBy }),
  });
  assertV2(capsule);
  return capsule;
}

export function economicAuditCapsuleDigest(capsule: EconomicAuditCapsule): Digest {
  assertEconomicAuditCapsule(capsule);
  return buildMerkleCommitment([capsule]).root;
}
