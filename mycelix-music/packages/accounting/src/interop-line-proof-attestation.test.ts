import { generateKeyPairSync, sign as signEd25519 } from 'node:crypto';
import { describe, expect, it } from 'vitest';
import { createEconomicAuditCapsule } from './audit.js';
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
  attachReportingEvidencePackageDetachedSignature,
  createReportingEvidencePackageAttestationCapability,
  createReportingEvidencePackageAttestationRequest,
  reportingEvidencePackageAttestationPayloadBase64,
  requireVerifiedReportingEvidencePackageAuthority,
  verifyReportingEvidencePackageAttestation,
} from './interop-line-proof-attestation.js';
import { exportDdexReportingEvidencePackage } from './interop-line-proofs.js';
import { money } from './money.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';
import { StatementKind } from './statements.js';

function digest(seed: string): string {
  if (!/^[0-9a-f]$/.test(seed)) throw new Error('test digest seed must be one hex character');
  return seed.repeat(64);
}

const auditIssuerKey = generateKeyPairSync('ed25519');
const auditRootKey = generateKeyPairSync('ed25519');
const reportingKey = generateKeyPairSync('ed25519');
const wrongReportingKey = generateKeyPairSync('ed25519');
const auditIssuerPublicKeyPem = auditIssuerKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const auditRootPublicKeyPem = auditRootKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const reportingPublicKeyPem = reportingKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();

const obligationA = createRoyaltyObligationAuthority({
  id: 'obligation:attest:a',
  beneficiaryId: 'creator:attest',
  amount: money(200n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:private:attest:a',
    rightsResolutionRef: 'rights:private:attest:a',
    economicTermsRef: 'terms:private:attest:a',
  },
});
const obligationB = createRoyaltyObligationAuthority({
  id: 'obligation:attest:b',
  beneficiaryId: 'creator:attest',
  amount: money(300n, 'USD'),
  observedAt: '2026-09-11T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:private:attest:b',
    rightsResolutionRef: 'rights:private:attest:b',
    economicTermsRef: 'terms:private:attest:b',
  },
});
const obligations = [obligationA, obligationB] as const;
const epoch: SettlementEpoch = {
  id: 'epoch:attest',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const eligibility = [
  {
    id: 'eligibility:attest:a',
    obligationId: obligationA.id,
    code: SettlementEligibilityCode.Eligible,
    sourceRef: 'eligibility-source:attest:a',
    observedAt: '2026-09-30T00:00:00Z',
  },
  {
    id: 'eligibility:attest:b',
    obligationId: obligationB.id,
    code: SettlementEligibilityCode.Eligible,
    sourceRef: 'eligibility-source:attest:b',
    observedAt: '2026-09-30T00:00:00Z',
  },
] as const;

const capsule = createEconomicAuditCapsule({
  statementInput: {
    statementId: 'statement:attest',
    kind: StatementKind.Periodic,
    beneficiaryId: 'creator:attest',
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
  generatedBy: { id: 'mycelix-music-royalty-compiler', version: '2.0.0', buildDigest: digest('9') },
});

const auditCapability = createEconomicAuditCapsuleAttestationCapability({
  capabilityId: 'audit-capability:reporting-attest',
  issuerId: 'audit-issuer:reporting-attest',
  signerKeyId: 'audit-key:reporting-attest',
  compilerId: capsule.generatedBy.id,
  compilerBuildDigest: capsule.generatedBy.buildDigest,
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2027-01-01T00:00:00Z',
});
const auditRequest = createEconomicAuditCapsuleAttestationRequest(capsule, auditCapability, {
  requestId: 'audit-request:reporting-attest',
  issuedAt: '2026-10-03T11:59:59Z',
  expiresAt: '2026-10-03T12:01:00Z',
  nonce: 'nonce:audit-request:reporting-attest',
});
const auditSignedAt = '2026-10-03T12:00:00Z';
const auditAttestation = attachEconomicAuditCapsuleDetachedSignature(capsule, auditCapability, auditRequest, {
  requestRoot: auditRequest.requestRoot,
  signerKeyId: auditRequest.signerKeyId,
  signedAt: auditSignedAt,
  signatureBase64: signEd25519(
    null,
    Buffer.from(economicAuditCapsuleAttestationPayloadBase64(auditRequest, auditSignedAt), 'base64'),
    auditIssuerKey.privateKey,
  ).toString('base64'),
});
const auditTrustBundle = createEconomicAuditCapsuleIssuerTrustBundle({
  bundleId: 'audit-issuer-bundle:reporting-attest',
  policyAsOf: '2026-10-03T13:00:00Z',
  entries: [{
    entryId: 'audit-issuer-entry:reporting-attest',
    issuerId: auditCapability.issuerId,
    signerKeyId: auditCapability.signerKeyId,
    capabilityRoot: auditCapability.capabilityRoot,
    publicKeyPem: auditIssuerPublicKeyPem,
    validFrom: '2026-10-01T00:00:00Z',
    validUntil: '2027-01-01T00:00:00Z',
  }],
});
const auditRootRequest = createEconomicAuditIssuerTrustBundleAttestationRequest(auditTrustBundle, {
  requestId: 'audit-root-request:reporting-attest',
  anchorId: 'audit-root-anchor:reporting-attest',
  signerKeyId: 'audit-root-key:reporting-attest',
  policySequence: '1',
  issuedAt: '2026-10-03T13:00:00Z',
  expiresAt: '2026-12-31T00:00:00Z',
  nonce: 'nonce:audit-root:reporting-attest',
});
const auditRootSignedAt = '2026-10-03T13:00:01Z';
const auditRootAttestation = attachEconomicAuditIssuerTrustBundleDetachedSignature(auditTrustBundle, auditRootRequest, {
  signerKeyId: auditRootRequest.signerKeyId,
  signedAt: auditRootSignedAt,
  signatureBase64: signEd25519(
    null,
    Buffer.from(economicAuditIssuerTrustBundleAttestationPayloadBase64(auditRootRequest, auditTrustBundle, auditRootSignedAt), 'base64'),
    auditRootKey.privateKey,
  ).toString('base64'),
});
const anchoredAuditIssuer = verifyEconomicAuditCapsuleAttestationWithAnchoredIssuerTrust(
  capsule,
  auditCapability,
  auditAttestation,
  auditTrustBundle,
  auditRootAttestation,
  {
    anchorId: auditRootRequest.anchorId,
    signerKeyId: auditRootRequest.signerKeyId,
    publicKeyPem: auditRootPublicKeyPem,
    validFrom: '2026-10-01T00:00:00Z',
    validUntil: '2027-01-01T00:00:00Z',
    minimumPolicySequence: '1',
  },
  '2026-10-03T14:00:00Z',
);

const unsortedLines = [
  {
    obligationId: obligationB.id,
    workReference: 'isrc:attest:b',
    beneficiaryReference: obligationB.beneficiaryId,
    amount: obligationB.amount,
    usageReference: 'usage:public:attest:b',
    territory: 'US',
  },
  {
    obligationId: obligationA.id,
    workReference: 'isrc:attest:a',
    beneficiaryReference: obligationA.beneficiaryId,
    amount: obligationA.amount,
    usageReference: 'usage:public:attest:a',
    territory: 'ZA',
  },
] as const;

const pkg = exportDdexReportingEvidencePackage({
  profileVersion: 'ddex-profile-v1',
  capsule,
  anchoredIssuer: anchoredAuditIssuer,
  generatedAt: '2026-10-03T15:00:00Z',
  lines: unsortedLines,
  obligations,
});

const capability = createReportingEvidencePackageAttestationCapability({
  capabilityId: 'reporting-evidence-capability:1',
  issuerId: 'reporting-evidence-issuer:1',
  signerKeyId: 'reporting-evidence-key:1',
  compilerId: capsule.generatedBy.id,
  compilerBuildDigest: capsule.generatedBy.buildDigest,
  allowedProfiles: [
    { format: 'DDEX', profileVersion: 'ddex-profile-v1' },
    { format: 'CISAC_CRD', profileVersion: 'crd-profile-v1' },
  ],
  validFrom: '2026-10-03T00:00:00Z',
  validUntil: '2026-11-01T00:00:00Z',
});
const request = createReportingEvidencePackageAttestationRequest(pkg, capability, {
  requestId: 'reporting-evidence-request:1',
  issuedAt: '2026-10-04T11:59:00Z',
  expiresAt: '2026-10-04T12:01:00Z',
  nonce: 'nonce:reporting-evidence:1',
});
const signedAt = '2026-10-04T12:00:00Z';

function signedAttestation(signer = reportingKey) {
  return attachReportingEvidencePackageDetachedSignature(pkg, capability, request, {
    requestRoot: request.requestRoot,
    signerKeyId: request.signerKeyId,
    signedAt,
    signatureBase64: signEd25519(
      null,
      Buffer.from(reportingEvidencePackageAttestationPayloadBase64(pkg, capability, request, signedAt), 'base64'),
      signer.privateKey,
    ).toString('base64'),
  });
}

const trustPolicy = {
  issuerId: capability.issuerId,
  signerKeyId: capability.signerKeyId,
  capabilityRoot: capability.capabilityRoot,
  publicKeyPem: reportingPublicKeyPem,
} as const;

describe('detached reporting evidence package attestation', () => {
  it('verifies an externally signed package and emits a sealed receipt', () => {
    const authority = verifyReportingEvidencePackageAttestation(
      pkg,
      capability,
      signedAttestation(),
      trustPolicy,
      '2026-10-04T13:00:00Z',
    );
    expect(authority.receipt.packageRoot).toBe(pkg.packageRoot);
    expect(authority.receipt.projectionRoot).toBe(pkg.projection.projectionRoot);
    expect(authority.receipt.sidecarRoot).toBe(pkg.disclosure.sidecarRoot);
    expect(authority.receipt.capsuleDigest).toBe(pkg.projection.audit.capsuleDigest);
    expect(authority.receipt.receiptRoot).toMatch(/^[0-9a-f]{64}$/);
    expect(requireVerifiedReportingEvidencePackageAuthority(authority)).toBe(authority);
  });

  it('accepts a package built from unsorted caller lines because proof matching is obligation-id based', () => {
    expect(pkg.projection.lines.map(line => line.obligationReference)).toEqual([
      obligationA.id,
      obligationB.id,
    ]);
    expect(pkg.lineEvidence.map(evidence => evidence.leaf.obligationId)).toEqual([
      obligationB.id,
      obligationA.id,
    ]);
    expect(() => createReportingEvidencePackageAttestationRequest(pkg, capability, {
      requestId: 'reporting-evidence-request:unsorted',
      issuedAt: '2026-10-04T11:59:00Z',
      expiresAt: '2026-10-04T12:01:00Z',
      nonce: 'nonce:reporting-evidence:unsorted',
    })).not.toThrow();
  });

  it('rejects projection content tampering behind an unchanged projection root', () => {
    const tampered = {
      ...pkg,
      projection: {
        ...pkg.projection,
        lines: pkg.projection.lines.map((line, index) => index === 0
          ? { ...line, workReference: 'isrc:tampered' }
          : line),
      },
    };
    expect(() => createReportingEvidencePackageAttestationRequest(tampered, capability, {
      requestId: 'reporting-evidence-request:tampered',
      issuedAt: '2026-10-04T11:59:00Z',
      expiresAt: '2026-10-04T12:01:00Z',
      nonce: 'nonce:reporting-evidence:tampered',
    })).toThrow(/projection root does not match canonical projection contents/);
  });

  it('rejects capabilities that do not authorize the exact format/profile pair', () => {
    const wrongProfile = createReportingEvidencePackageAttestationCapability({
      capabilityId: 'reporting-evidence-capability:wrong-profile',
      issuerId: capability.issuerId,
      signerKeyId: capability.signerKeyId,
      compilerId: capsule.generatedBy.id,
      compilerBuildDigest: capsule.generatedBy.buildDigest,
      allowedProfiles: [{ format: 'DDEX', profileVersion: 'different-profile' }],
      validFrom: capability.validFrom,
      validUntil: capability.validUntil,
    });
    expect(() => createReportingEvidencePackageAttestationRequest(pkg, wrongProfile, {
      requestId: 'reporting-evidence-request:wrong-profile',
      issuedAt: '2026-10-04T11:59:00Z',
      expiresAt: '2026-10-04T12:01:00Z',
      nonce: 'nonce:reporting-evidence:wrong-profile',
    })).toThrow(/profile is outside capability/);
  });

  it('uses a half-open signature request window', () => {
    expect(() => reportingEvidencePackageAttestationPayloadBase64(
      pkg,
      capability,
      request,
      request.expiresAt,
    )).toThrow(/outside request window/);
  });

  it('rejects a detached signature produced by the wrong Ed25519 key', () => {
    expect(() => verifyReportingEvidencePackageAttestation(
      pkg,
      capability,
      signedAttestation(wrongReportingKey),
      trustPolicy,
      '2026-10-04T13:00:00Z',
    )).toThrow(/signature verification failed/);
  });

  it('requires trust policy to pin the exact capability root', () => {
    expect(() => verifyReportingEvidencePackageAttestation(
      pkg,
      capability,
      signedAttestation(),
      { ...trustPolicy, capabilityRoot: digest('a') },
      '2026-10-04T13:00:00Z',
    )).toThrow(/does not pin exact capability root/);
  });

  it('does not let structural clones regain verified package authority', () => {
    const authority = verifyReportingEvidencePackageAttestation(
      pkg,
      capability,
      signedAttestation(),
      trustPolicy,
      '2026-10-04T13:00:00Z',
    );
    expect(() => requireVerifiedReportingEvidencePackageAuthority({ ...authority }))
      .toThrow(/must be produced by detached signature verification/);
  });
});
