import { generateKeyPairSync, sign as signEd25519 } from 'node:crypto';
import { describe, expect, it } from 'vitest';
import { createEconomicAuditCapsule, economicAuditCapsuleDigest } from './audit.js';
import {
  attachReportingEvidencePackageDetachedSignature,
  createReportingEvidencePackageAttestationCapability,
  createReportingEvidencePackageAttestationRequest,
  reportingEvidencePackageAttestationPayloadBase64,
} from './interop-line-proof-attestation.js';
import {
  createReportingEvidencePackageIssuerTrustBundle,
  requireVerifiedReportingEvidencePackageIssuerAuthority,
  verifyReportingEvidencePackageAttestationWithTrustBundle,
} from './interop-line-proof-attestation-trust.js';
import {
  createReportingSafeDisclosureCommitment,
  createReportingSafeObligationEvidence,
  type AttestedReportingEvidencePackage,
} from './interop-line-proofs.js';
import type { AttestedDdexRoyaltyReportingProjection, AttestedAuditReportingReference } from './interop.js';
import { buildMerkleCommitment } from './merkle.js';
import { money } from './money.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';
import { StatementKind } from './statements.js';

function digest(seed: string): string {
  if (!/^[0-9a-f]$/.test(seed)) throw new Error('test digest seed must be one hex character');
  return seed.repeat(64);
}

const oldKey = generateKeyPairSync('ed25519');
const newKey = generateKeyPairSync('ed25519');
const oldPublicKeyPem = oldKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const newPublicKeyPem = newKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();

const obligation = createRoyaltyObligationAuthority({
  id: 'obligation:reporting-trust:1',
  beneficiaryId: 'creator:reporting-trust',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:private:reporting-trust',
    rightsResolutionRef: 'rights:private:reporting-trust',
    economicTermsRef: 'terms:private:reporting-trust',
  },
});
const obligations = [obligation] as const;
const epoch: SettlementEpoch = {
  id: 'epoch:reporting-trust',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const capsule = createEconomicAuditCapsule({
  statementInput: {
    statementId: 'statement:reporting-trust',
    kind: StatementKind.Periodic,
    beneficiaryId: obligation.beneficiaryId,
    period: { startInclusive: '2026-09-01T00:00:00Z', endExclusive: '2026-10-01T00:00:00Z' },
    asOf: '2026-10-02T00:00:00Z',
    completeness: {
      kind: 'complete',
      through: {
        usageObservedThrough: '2026-10-01T00:00:00Z',
        rightsResolvedThrough: '2026-10-01T00:00:00Z',
        settlementsObservedThrough: '2026-10-02T00:00:00Z',
      },
    },
    settlementEpoch: epoch,
    obligations,
    eligibilityObservations: [{
      id: 'eligibility:reporting-trust:1',
      obligationId: obligation.id,
      code: SettlementEligibilityCode.Eligible,
      sourceRef: 'eligibility-source:reporting-trust:1',
      observedAt: '2026-09-30T00:00:00Z',
    }],
  },
  usageCommitment: digest('1'),
  repertoireSnapshot: digest('2'),
  rightsPolicy: digest('3'),
  economicTerms: digest('4'),
  rightsResolutionRoot: digest('5'),
  nettingRoot: digest('6'),
  settlementPlanRoot: digest('7'),
  paymentReceiptRoot: digest('8'),
  generatedBy: { id: 'mycelix-music-royalty-compiler', version: '2.0.0', buildDigest: digest('9') },
});

const disclosure = createReportingSafeDisclosureCommitment({ capsule, obligations });
const lineEvidence = [createReportingSafeObligationEvidence({ capsule, obligations, obligationId: obligation.id })] as const;
const source = Object.freeze({
  statementId: capsule.statement.statementId,
  statementCommitmentRoot: capsule.statementSnapshotRoot,
  generatedAt: '2026-10-03T15:00:00.000Z',
});
const auditFields = Object.freeze({
  provenanceKind: 'root_attested_economic_audit_capsule_v1' as const,
  capsuleProtocolVersion: 2 as const,
  capsuleDigest: economicAuditCapsuleDigest(capsule),
  statementSnapshotRoot: capsule.statementSnapshotRoot,
  obligationRoot: capsule.statement.obligationRoot,
  adjustmentRoot: capsule.statement.adjustmentRoot,
  settlementRoot: capsule.statement.settlementRoot,
  compilerId: capsule.generatedBy.id,
  compilerBuildDigest: capsule.generatedBy.buildDigest,
  issuerVerificationReceiptRoot: digest('a'),
  anchoredIssuerVerificationReceiptRoot: digest('b'),
  issuerTrustBundleRoot: digest('c'),
  issuerTrustBundleAttestationRoot: digest('d'),
  issuerTrustAnchorId: 'audit-root:reporting-trust',
  issuerTrustPolicySequence: '1',
  issuerVerifiedAt: '2026-10-03T14:00:00.000Z',
  lineEvidenceScope: 'statement_level_reference_only' as const,
});
const audit: AttestedAuditReportingReference = Object.freeze({
  ...auditFields,
  provenanceRoot: buildMerkleCommitment([{ recordType: 'interop_root_attested_audit_reference_v1', ...auditFields }]).root,
});
const projectionLines = Object.freeze([Object.freeze({
  obligationReference: obligation.id,
  workReference: 'isrc:reporting-trust:1',
  payeeReference: obligation.beneficiaryId,
  royaltyAmount: { amountMinor: obligation.amount.amountMinor.toString(10), currency: obligation.amount.currency },
})]);
const projectionBase = Object.freeze({
  format: 'DDEX' as const,
  authority: 'reporting_projection_only' as const,
  profileVersion: 'ddex-profile-v1',
  source,
  lines: projectionLines,
  audit,
});
const projection: AttestedDdexRoyaltyReportingProjection = Object.freeze({
  ...projectionBase,
  projectionRoot: buildMerkleCommitment([{
    recordType: 'ddex_royalty_reporting_projection_attested_audit_v1',
    ...projectionBase,
  }]).root,
});
const packageBase = Object.freeze({
  authority: 'reporting_projection_only' as const,
  evidenceStatus: 'compiler_derived_line_evidence_not_issuer_attested' as const,
  projection,
  disclosure,
  lineEvidence,
});
const pkg: AttestedReportingEvidencePackage<AttestedDdexRoyaltyReportingProjection> = Object.freeze({
  ...packageBase,
  packageRoot: buildMerkleCommitment([{
    recordType: 'attested_reporting_evidence_package_v1',
    authority: packageBase.authority,
    evidenceStatus: packageBase.evidenceStatus,
    projectionRoot: projection.projectionRoot,
    sidecarRoot: disclosure.sidecarRoot,
    lineEvidence,
  }]).root,
});

const oldCapability = createReportingEvidencePackageAttestationCapability({
  capabilityId: 'reporting-trust-capability:old',
  issuerId: 'reporting-trust-issuer',
  signerKeyId: 'reporting-trust-key:old',
  compilerId: capsule.generatedBy.id,
  compilerBuildDigest: capsule.generatedBy.buildDigest,
  allowedProfiles: [{ format: 'DDEX', profileVersion: projection.profileVersion }],
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2026-11-01T00:00:00Z',
});
const oldRequest = createReportingEvidencePackageAttestationRequest(pkg, oldCapability, {
  requestId: 'reporting-trust-request:old',
  issuedAt: '2026-10-04T11:59:00Z',
  expiresAt: '2026-10-04T12:01:00Z',
  nonce: 'nonce:reporting-trust:old',
});
const oldSignedAt = '2026-10-04T12:00:00Z';
const oldAttestation = attachReportingEvidencePackageDetachedSignature(pkg, oldCapability, oldRequest, {
  requestRoot: oldRequest.requestRoot,
  signerKeyId: oldRequest.signerKeyId,
  signedAt: oldSignedAt,
  signatureBase64: signEd25519(
    null,
    Buffer.from(reportingEvidencePackageAttestationPayloadBase64(pkg, oldCapability, oldRequest, oldSignedAt), 'base64'),
    oldKey.privateKey,
  ).toString('base64'),
});

const newCapability = createReportingEvidencePackageAttestationCapability({
  capabilityId: 'reporting-trust-capability:new',
  issuerId: oldCapability.issuerId,
  signerKeyId: 'reporting-trust-key:new',
  compilerId: capsule.generatedBy.id,
  compilerBuildDigest: capsule.generatedBy.buildDigest,
  allowedProfiles: [{ format: 'DDEX', profileVersion: projection.profileVersion }],
  validFrom: '2026-10-10T00:00:00Z',
  validUntil: '2026-12-01T00:00:00Z',
});
const newRequest = createReportingEvidencePackageAttestationRequest(pkg, newCapability, {
  requestId: 'reporting-trust-request:new',
  issuedAt: '2026-10-12T11:59:00Z',
  expiresAt: '2026-10-12T12:01:00Z',
  nonce: 'nonce:reporting-trust:new',
});
const newSignedAt = '2026-10-12T12:00:00Z';
const newAttestation = attachReportingEvidencePackageDetachedSignature(pkg, newCapability, newRequest, {
  requestRoot: newRequest.requestRoot,
  signerKeyId: newRequest.signerKeyId,
  signedAt: newSignedAt,
  signatureBase64: signEd25519(
    null,
    Buffer.from(reportingEvidencePackageAttestationPayloadBase64(pkg, newCapability, newRequest, newSignedAt), 'base64'),
    newKey.privateKey,
  ).toString('base64'),
});

function rotatedBundle(policyAsOf = '2026-10-20T00:00:00Z') {
  return createReportingEvidencePackageIssuerTrustBundle({
    bundleId: 'reporting-evidence-issuer-trust:rotation',
    policyAsOf,
    entries: [
      {
        entryId: 'reporting-evidence-issuer-entry:old',
        issuerId: oldCapability.issuerId,
        signerKeyId: oldCapability.signerKeyId,
        capabilityRoot: oldCapability.capabilityRoot,
        publicKeyPem: oldPublicKeyPem,
        validFrom: '2026-10-01T00:00:00Z',
        validUntil: '2026-10-10T00:00:00Z',
      },
      {
        entryId: 'reporting-evidence-issuer-entry:new',
        issuerId: newCapability.issuerId,
        signerKeyId: newCapability.signerKeyId,
        capabilityRoot: newCapability.capabilityRoot,
        publicKeyPem: newPublicKeyPem,
        validFrom: '2026-10-10T00:00:00Z',
        validUntil: '2026-12-01T00:00:00Z',
      },
    ],
  });
}

describe('reporting evidence issuer trust rotation', () => {
  it('keeps a pre-retirement signature verifiable after rotation', () => {
    const authority = verifyReportingEvidencePackageAttestationWithTrustBundle(
      pkg,
      oldCapability,
      oldAttestation,
      rotatedBundle(),
      '2026-10-20T01:00:00Z',
    );
    expect(authority.receipt.trustEntryId).toBe('reporting-evidence-issuer-entry:old');
    expect(authority.receipt.trustBundleRoot).toMatch(/^[0-9a-f]{64}$/);
    expect(requireVerifiedReportingEvidencePackageIssuerAuthority(authority)).toBe(authority);
  });

  it('authorizes the rotated signer in its own window', () => {
    const authority = verifyReportingEvidencePackageAttestationWithTrustBundle(
      pkg,
      newCapability,
      newAttestation,
      rotatedBundle(),
      '2026-10-20T01:00:00Z',
    );
    expect(authority.receipt.trustEntryId).toBe('reporting-evidence-issuer-entry:new');
  });

  it('uses half-open retirement semantics at validUntil', () => {
    const boundaryBundle = createReportingEvidencePackageIssuerTrustBundle({
      bundleId: 'reporting-evidence-issuer-trust:boundary',
      policyAsOf: '2026-10-05T00:00:00Z',
      entries: [{
        entryId: 'reporting-evidence-issuer-entry:boundary',
        issuerId: oldCapability.issuerId,
        signerKeyId: oldCapability.signerKeyId,
        capabilityRoot: oldCapability.capabilityRoot,
        publicKeyPem: oldPublicKeyPem,
        validFrom: '2026-10-01T00:00:00Z',
        validUntil: oldSignedAt,
      }],
    });
    expect(() => verifyReportingEvidencePackageAttestationWithTrustBundle(
      pkg,
      oldCapability,
      oldAttestation,
      boundaryBundle,
      '2026-10-05T01:00:00Z',
    )).toThrow(/no authorized issuer trust entry/);
  });

  it('treats revocation as prospective and rejects signatures at revokedAt', () => {
    const revokedBundle = createReportingEvidencePackageIssuerTrustBundle({
      bundleId: 'reporting-evidence-issuer-trust:revoked',
      policyAsOf: '2026-10-05T00:00:00Z',
      entries: [{
        entryId: 'reporting-evidence-issuer-entry:revoked',
        issuerId: oldCapability.issuerId,
        signerKeyId: oldCapability.signerKeyId,
        capabilityRoot: oldCapability.capabilityRoot,
        publicKeyPem: oldPublicKeyPem,
        validFrom: '2026-10-01T00:00:00Z',
        validUntil: '2026-11-01T00:00:00Z',
        revokedAt: oldSignedAt,
      }],
    });
    expect(() => verifyReportingEvidencePackageAttestationWithTrustBundle(
      pkg,
      oldCapability,
      oldAttestation,
      revokedBundle,
      '2026-10-05T01:00:00Z',
    )).toThrow(/no authorized issuer trust entry/);
  });

  it('rejects a stale policy snapshot predating the signature', () => {
    expect(() => verifyReportingEvidencePackageAttestationWithTrustBundle(
      pkg,
      oldCapability,
      oldAttestation,
      rotatedBundle('2026-10-04T11:59:59Z'),
      '2026-10-20T01:00:00Z',
    )).toThrow(/policy must be observed through attestation signedAt/);
  });

  it('rejects overlapping windows for the same exact authority identity', () => {
    expect(() => createReportingEvidencePackageIssuerTrustBundle({
      bundleId: 'reporting-evidence-issuer-trust:overlap',
      policyAsOf: '2026-10-20T00:00:00Z',
      entries: [
        {
          entryId: 'reporting-evidence-issuer-entry:overlap:a',
          issuerId: oldCapability.issuerId,
          signerKeyId: oldCapability.signerKeyId,
          capabilityRoot: oldCapability.capabilityRoot,
          publicKeyPem: oldPublicKeyPem,
          validFrom: '2026-10-01T00:00:00Z',
          validUntil: '2026-10-15T00:00:00Z',
        },
        {
          entryId: 'reporting-evidence-issuer-entry:overlap:b',
          issuerId: oldCapability.issuerId,
          signerKeyId: oldCapability.signerKeyId,
          capabilityRoot: oldCapability.capabilityRoot,
          publicKeyPem: oldPublicKeyPem,
          validFrom: '2026-10-10T00:00:00Z',
          validUntil: '2026-10-20T00:00:00Z',
        },
      ],
    })).toThrow(/overlapping authority windows/);
  });

  it('does not let structural clones regain issuer trust authority', () => {
    const authority = verifyReportingEvidencePackageAttestationWithTrustBundle(
      pkg,
      oldCapability,
      oldAttestation,
      rotatedBundle(),
      '2026-10-20T01:00:00Z',
    );
    expect(() => requireVerifiedReportingEvidencePackageIssuerAuthority({ ...authority }))
      .toThrow(/must be produced by trust-bundle verification/);
  });
});
