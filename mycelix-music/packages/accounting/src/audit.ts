import { buildMerkleCommitment, type Digest } from './merkle.js';
import { assertConservation, type ConservationSummary } from './reconciliation.js';

export interface CompilerIdentity {
  readonly id: string;
  readonly version: string;
  readonly buildDigest: string;
}

export interface EconomicAuditCapsule {
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

const DIGEST_FIELDS: ReadonlyArray<keyof Pick<
  EconomicAuditCapsule,
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

export function assertEconomicAuditCapsule(capsule: EconomicAuditCapsule): void {
  if (capsule.protocolVersion !== 1) throw new Error('unsupported economic audit capsule version');
  for (const field of DIGEST_FIELDS) {
    if (!capsule[field].trim()) throw new Error(`${field} must be non-empty`);
  }
  if (!capsule.generatedBy.id.trim() || !capsule.generatedBy.version.trim() || !capsule.generatedBy.buildDigest.trim()) {
    throw new Error('compiler identity must include id, version and buildDigest');
  }
  assertConservation(capsule.conservationProof);
}

export function createEconomicAuditCapsule(
  capsule: EconomicAuditCapsule,
): Readonly<EconomicAuditCapsule> {
  assertEconomicAuditCapsule(capsule);
  return Object.freeze({
    ...capsule,
    generatedBy: Object.freeze({ ...capsule.generatedBy }),
    conservationProof: Object.freeze({
      ...capsule.conservationProof,
      inputTotal: Object.freeze({ ...capsule.conservationProof.inputTotal }),
      distributedTotal: Object.freeze({ ...capsule.conservationProof.distributedTotal }),
      heldTotal: Object.freeze({ ...capsule.conservationProof.heldTotal }),
      deductionTotal: Object.freeze({ ...capsule.conservationProof.deductionTotal }),
      residualTotal: Object.freeze({ ...capsule.conservationProof.residualTotal }),
    }),
  });
}

export function economicAuditCapsuleDigest(capsule: EconomicAuditCapsule): Digest {
  assertEconomicAuditCapsule(capsule);
  return buildMerkleCommitment([capsule]).root;
}
