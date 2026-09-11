import { generateKeyPairSync, sign as signEd25519 } from 'node:crypto';
import { describe, expect, it } from 'vitest';
import { createEconomicAuditCapsule, economicAuditCapsuleDigest } from './audit.js';
import {
  attachEconomicAuditCapsuleDetachedSignature,
  createEconomicAuditCapsuleAttestationCapability,
  createEconomicAuditCapsuleAttestationRequest,
  economicAuditCapsuleAttestationPayloadBase64,
} from './audit-capsule-attestation.js';
import { createEconomicAuditCapsuleIssuerTrustBundle } from './audit-capsule-attestation-trust.js';
import {
  attachEconomicAuditIssuerTrustBundleDetachedSignature,
  createEconomicAuditIssuerTrustBundleAttestationRequest,
  economicAuditIssuerTrustBundleAttestationPayloadBase64,
  verifyEconomicAuditCapsuleAttestationWithAnchoredIssuerTrust,
} from './audit-capsule-attestation-trust-anchor.js';
import {
  exportAttestedCisacCrdReportingProjection,
  exportAttestedDdexRoyaltyReportingProjection,
  type RoyaltyReportingLine,
} from './interop.js';
import { money } from './money.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';
import { StatementKind } from './statements.js';

function digest(seed: string): string {
  if (!/^[0-9a-f]$/.test(seed)) throw new Error('test digest seed must be one hex character');
  return seed.repeat(64);
}

const issuerKey = generateKeyPairSync('ed25519');
const rootKey = generateKeyPairSync('ed25519');
const issuerPublicKeyPem = issuerKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const rootPublicKeyPem = rootKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const asOf = '2026-10-02T00:00:00Z';

const obligation = createRoyaltyObligationAuthority({
  id: 'obligation:interop-attested',
  beneficiaryId: 'creator:interop-attested',
  amount: money(327n, 'USD'),
  observedAt: '2026-09-15T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:interop-attested',
    rightsResolutionRef: 'rights:interop-attested',
    economicTermsRef: 'terms:interop-attested',
  },
});
const epoch: SettlementEpoch = {
  id: 'epoch:interop-attested',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const eligibility = [{
  id: 'eligibility:interop-attested',
  obligationId: obligation.id,
  code: SettlementEligibilityCode.Eligible,
  sourceRef: 'eligibility-source:interop-attested',
  observedAt: '2026-09-30T00:00:00Z',
}] as const;

const capsule = createEconomicAuditCapsule({
  statementInput: {
    statementId: 'statement:interop-attested',
    kind: StatementKind.Periodic,
    beneficiaryId: obligation.beneficiaryId,
    period: { startInclusive: '2026-09-01T00:00:00Z', endExclusive: '2026-10-01T00:00:00Z' },
    asOf,
    completeness: {
      kind: 'complete',
      through: {
        usageObservedThrough: '2026-10-01T00:00:00Z',
        rightsResolvedThrough: '2026-10-01T00:00:00Z',
        settlementsObservedThrough: asOf,
      },
    },
    settlementEpoch: epoch,
    obligations: [obligation],
    eligibilityObservations: eligibility,
  },
  usageCommitment: digest('1'),
  repertoireSnapshot: digest('2'),
  rightsPolicy: digest('3'),
  economicTerms: digest('4'),
  rightsResolutionRoot: digest('5'),
  nettingRoot: digest('6'),
  settlementPlanRoot: digest('7'),
  paymentReceiptRoot: digest('8'),
  generatedBy: {
    id: 'mycelix-music-royalty-compiler',
    version: '2.0.0',
    buildDigest: digest('9'),
  },
});

const capability = createEconomicAuditCapsuleAttestationCapability({
  capabilityId: 'audit-capability:interop-attested',
  issuerId: 'audit-issuer:interop-attested',
  signerKeyId: 'audit-key:interop-attested',
  compilerId: capsule.generatedBy.id,
  compilerBuildDigest: capsule.generatedBy.buildDigest,
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2027-01-01T00:00:00Z',
});
const issuerRequest = createEconomicAuditCapsuleAttestationRequest(capsule, capability, {
  requestId: 'audit-request:interop-attested',
  issuedAt: '2026-10-03T11:59:59Z',
  expiresAt: '2026-10-03T12:01:00Z',
  nonce: 'nonce:interop-attested',
});
const issuerSignedAt = '2026-10-03T12:00:00Z';
const issuerAttestation = attachEconomicAuditCapsuleDetachedSignature(capsule, capability, issuerRequest, {
  requestRoot: issuerRequest.requestRoot,
  signerKeyId: issuerRequest.signerKeyId,
  signedAt: issuerSignedAt,
  signatureBase64: signEd25519(
    null,
    Buffer.from(economicAuditCapsuleAttestationPayloadBase64(issuerRequest, issuerSignedAt), 'base64'),
    issuerKey.privateKey,
  ).toString('base64'),
});
const trustBundle = createEconomicAuditCapsuleIssuerTrustBundle({
  bundleId: 'audit-issuer-bundle:interop-attested',
  policyAsOf: '2026-10-03T13:00:00Z',
  entries: [{
    entryId: 'audit-issuer-entry:interop-attested',
    issuerId: capability.issuerId,
    signerKeyId: capability.signerKeyId,
    capabilityRoot: capability.capabilityRoot,
    publicKeyPem: issuerPublicKeyPem,
    validFrom: '2026-10-01T00:00:00Z',
    validUntil: '2027-01-01T00:00:00Z',
  }],
});
const rootRequest = createEconomicAuditIssuerTrustBundleAttestationRequest(trustBundle, {
  requestId: 'audit-root-request:interop-attested',
  anchorId: 'audit-root-anchor:interop-attested',
  signerKeyId: 'audit-root-key:interop-attested',
  policySequence: '1',
  issuedAt: '2026-10-03T13:00:00Z',
  expiresAt: '2026-12-31T00:00:00Z',
  nonce: 'nonce:audit-root:interop-attested',
});
const rootSignedAt = '2026-10-03T13:00:01Z';
const rootAttestation = attachEconomicAuditIssuerTrustBundleDetachedSignature(trustBundle, rootRequest, {
  signerKeyId: rootRequest.signerKeyId,
  signedAt: rootSignedAt,
  signatureBase64: signEd25519(
    null,
    Buffer.from(economicAuditIssuerTrustBundleAttestationPayloadBase64(rootRequest, trustBundle, rootSignedAt), 'base64'),
    rootKey.privateKey,
  ).toString('base64'),
});
const anchoredIssuer = verifyEconomicAuditCapsuleAttestationWithAnchoredIssuerTrust(
  capsule,
  capability,
  issuerAttestation,
  trustBundle,
  rootAttestation,
  {
    anchorId: rootRequest.anchorId,
    signerKeyId: rootRequest.signerKeyId,
    publicKeyPem: rootPublicKeyPem,
    validFrom: '2026-10-01T00:00:00Z',
    validUntil: '2027-01-01T00:00:00Z',
    minimumPolicySequence: '1',
  },
  '2026-10-03T14:00:00Z',
);

const lines: RoyaltyReportingLine[] = [{
  obligationId: obligation.id,
  workReference: 'isrc:interop-attested',
  beneficiaryReference: obligation.beneficiaryId,
  amount: money(327n, 'USD'),
  usageReference: 'usage:interop-attested',
  territory: 'ZA',
}];

function attestedInput() {
  return {
    profileVersion: 'configured-profile-v1',
    capsule,
    anchoredIssuer,
    generatedAt: '2026-10-03T15:00:00Z',
    lines,
  } as const;
}

describe('root-attested reporting projections', () => {
  it('derives DDEX statement provenance from the sealed attested audit chain', () => {
    const first = exportAttestedDdexRoyaltyReportingProjection(attestedInput());
    const second = exportAttestedDdexRoyaltyReportingProjection(attestedInput());
    expect(first.authority).toBe('reporting_projection_only');
    expect(first.source.statementId).toBe(capsule.statement.statementId);
    expect(first.source.statementCommitmentRoot).toBe(capsule.statementSnapshotRoot);
    expect(first.audit.capsuleDigest).toBe(economicAuditCapsuleDigest(capsule));
    expect(first.audit.anchoredIssuerVerificationReceiptRoot).toBe(anchoredIssuer.receipt.receiptRoot);
    expect(first.audit.issuerTrustBundleAttestationRoot).toBe(anchoredIssuer.receipt.trustBundleAttestationRoot);
    expect(first.audit.lineEvidenceScope).toBe('statement_level_reference_only');
    expect(first.audit.provenanceRoot).toMatch(/^[0-9a-f]{64}$/);
    expect(first.projectionRoot).toMatch(/^[0-9a-f]{64}$/);
    expect(second.projectionRoot).toBe(first.projectionRoot);
  });

  it('derives CRD provenance from the same attested statement without granting ledger authority', () => {
    const projection = exportAttestedCisacCrdReportingProjection({
      ...attestedInput(),
      profileVersion: 'configured-crd-profile-v1',
    });
    expect(projection.authority).toBe('reporting_projection_only');
    expect(projection.audit.provenanceKind).toBe('root_attested_economic_audit_capsule_v1');
    expect(projection.audit.issuerTrustAnchorId).toBe('audit-root-anchor:interop-attested');
    expect(projection.lines[0]?.distributionReference).toBe(obligation.id);
  });

  it('rejects structural clones of the anchored issuer authority', () => {
    expect(() => exportAttestedDdexRoyaltyReportingProjection({
      ...attestedInput(),
      anchoredIssuer: { ...anchoredIssuer },
    })).toThrow(/must be produced by root trust verification/);
  });

  it('rejects reports generated before anchored issuer verification', () => {
    expect(() => exportAttestedDdexRoyaltyReportingProjection({
      ...attestedInput(),
      generatedAt: '2026-10-03T13:30:00Z',
    })).toThrow(/cannot predate anchored issuer verification/);
  });

  it('rejects lines that cross the audited beneficiary or currency boundary', () => {
    expect(() => exportAttestedDdexRoyaltyReportingProjection({
      ...attestedInput(),
      lines: [{ ...lines[0]!, beneficiaryReference: 'creator:other' }],
    })).toThrow(/beneficiary does not match audited statement beneficiary/);
    expect(() => exportAttestedDdexRoyaltyReportingProjection({
      ...attestedInput(),
      lines: [{ ...lines[0]!, amount: money(327n, 'EUR') }],
    })).toThrow(/currency does not match audited statement currency/);
  });

  it('keeps projection content commitments sensitive to reporting profile and line content', () => {
    const first = exportAttestedDdexRoyaltyReportingProjection(attestedInput());
    const profileChanged = exportAttestedDdexRoyaltyReportingProjection({
      ...attestedInput(),
      profileVersion: 'configured-profile-v2',
    });
    const lineChanged = exportAttestedDdexRoyaltyReportingProjection({
      ...attestedInput(),
      lines: [{ ...lines[0]!, amount: money(326n, 'USD') }],
    });
    expect(profileChanged.projectionRoot).not.toBe(first.projectionRoot);
    expect(lineChanged.projectionRoot).not.toBe(first.projectionRoot);
    expect(lineChanged.audit.provenanceRoot).toBe(first.audit.provenanceRoot);
  });
});
