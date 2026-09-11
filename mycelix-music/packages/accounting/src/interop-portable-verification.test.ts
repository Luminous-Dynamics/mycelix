import { generateKeyPairSync, sign as signEd25519 } from 'node:crypto';
import { describe, expect, it } from 'vitest';
import { serializeAccountingWire, deserializeAccountingWire } from './accounting-wire.js';
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
} from './interop-line-proof-attestation.js';
import { createReportingEvidencePackageIssuerTrustBundle } from './interop-line-proof-attestation-trust.js';
import {
  attachReportingEvidenceIssuerTrustBundleDetachedSignature,
  createReportingEvidenceIssuerTrustBundleAttestationRequest,
  reportingEvidenceIssuerTrustBundleAttestationPayloadBase64,
} from './interop-line-proof-attestation-trust-anchor.js';
import { exportDdexReportingEvidencePackage } from './interop-line-proofs.js';
import {
  createPortableReportingEvidenceBundle,
  requireVerifiedPortableReportingEvidence,
  serializePortableReportingEvidenceBundle,
  verifyPortableReportingEvidenceWire,
} from './interop-portable-verification.js';
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
const reportingIssuerKey = generateKeyPairSync('ed25519');
const reportingRootKey = generateKeyPairSync('ed25519');
const wrongRootKey = generateKeyPairSync('ed25519');
const auditIssuerPublicKeyPem = auditIssuerKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const auditRootPublicKeyPem = auditRootKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const reportingIssuerPublicKeyPem = reportingIssuerKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const reportingRootPublicKeyPem = reportingRootKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const wrongRootPublicKeyPem = wrongRootKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();

const obligation = createRoyaltyObligationAuthority({
  id: 'obligation:portable:1',
  beneficiaryId: 'creator:portable',
  amount: money(12345678901234567890n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:private:portable:1',
    rightsResolutionRef: 'rights:private:portable:1',
    economicTermsRef: 'terms:private:portable:1',
  },
});
const epoch: SettlementEpoch = {
  id: 'epoch:portable',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const capsule = createEconomicAuditCapsule({
  statementInput: {
    statementId: 'statement:portable',
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
    obligations: [obligation],
    eligibilityObservations: [{
      id: 'eligibility:portable:1',
      obligationId: obligation.id,
      code: SettlementEligibilityCode.Eligible,
      sourceRef: 'eligibility-source:portable:1',
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

const auditCapability = createEconomicAuditCapsuleAttestationCapability({
  capabilityId: 'audit-capability:portable',
  issuerId: 'audit-issuer:portable',
  signerKeyId: 'audit-issuer-key:portable',
  compilerId: capsule.generatedBy.id,
  compilerBuildDigest: capsule.generatedBy.buildDigest,
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2027-01-01T00:00:00Z',
});
const auditRequest = createEconomicAuditCapsuleAttestationRequest(capsule, auditCapability, {
  requestId: 'audit-request:portable',
  issuedAt: '2026-10-03T11:59:00Z',
  expiresAt: '2026-10-03T12:01:00Z',
  nonce: 'nonce:audit:portable',
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
  bundleId: 'audit-trust:portable',
  policyAsOf: '2026-10-03T12:30:00Z',
  entries: [{
    entryId: 'audit-trust-entry:portable',
    issuerId: auditCapability.issuerId,
    signerKeyId: auditCapability.signerKeyId,
    capabilityRoot: auditCapability.capabilityRoot,
    publicKeyPem: auditIssuerPublicKeyPem,
    validFrom: '2026-10-01T00:00:00Z',
    validUntil: '2027-01-01T00:00:00Z',
  }],
});
const auditRootRequest = createEconomicAuditIssuerTrustBundleAttestationRequest(auditTrustBundle, {
  requestId: 'audit-root-request:portable',
  anchorId: 'audit-root:portable',
  signerKeyId: 'audit-root-key:portable',
  policySequence: '1',
  issuedAt: '2026-10-03T12:30:00Z',
  expiresAt: '2026-12-31T00:00:00Z',
  nonce: 'nonce:audit-root:portable',
});
const auditRootSignedAt = '2026-10-03T12:30:01Z';
const auditRootAttestation = attachEconomicAuditIssuerTrustBundleDetachedSignature(auditTrustBundle, auditRootRequest, {
  signerKeyId: auditRootRequest.signerKeyId,
  signedAt: auditRootSignedAt,
  signatureBase64: signEd25519(
    null,
    Buffer.from(
      economicAuditIssuerTrustBundleAttestationPayloadBase64(auditRootRequest, auditTrustBundle, auditRootSignedAt),
      'base64',
    ),
    auditRootKey.privateKey,
  ).toString('base64'),
});
const auditRootPolicy = {
  anchorId: auditRootRequest.anchorId,
  signerKeyId: auditRootRequest.signerKeyId,
  publicKeyPem: auditRootPublicKeyPem,
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2027-01-01T00:00:00Z',
  minimumPolicySequence: '1',
} as const;
const auditVerifiedAt = '2026-10-03T13:00:00Z';
const anchoredAudit = verifyEconomicAuditCapsuleAttestationWithAnchoredIssuerTrust(
  capsule,
  auditCapability,
  auditAttestation,
  auditTrustBundle,
  auditRootAttestation,
  auditRootPolicy,
  auditVerifiedAt,
);

const pkg = exportDdexReportingEvidencePackage({
  profileVersion: 'ddex-profile-v1',
  capsule,
  anchoredIssuer: anchoredAudit,
  generatedAt: '2026-10-03T14:00:00Z',
  obligations: [obligation],
  lines: [{
    obligationId: obligation.id,
    workReference: 'isrc:portable:1',
    beneficiaryReference: obligation.beneficiaryId,
    amount: obligation.amount,
    usageReference: 'usage:public:portable:1',
    territory: 'ZA',
  }],
});
const reportingCapability = createReportingEvidencePackageAttestationCapability({
  capabilityId: 'reporting-capability:portable',
  issuerId: 'reporting-issuer:portable',
  signerKeyId: 'reporting-issuer-key:portable',
  compilerId: capsule.generatedBy.id,
  compilerBuildDigest: capsule.generatedBy.buildDigest,
  allowedProfiles: [{ format: 'DDEX', profileVersion: 'ddex-profile-v1' }],
  validFrom: '2026-10-03T00:00:00Z',
  validUntil: '2027-01-01T00:00:00Z',
});
const reportingRequest = createReportingEvidencePackageAttestationRequest(pkg, reportingCapability, {
  requestId: 'reporting-request:portable',
  issuedAt: '2026-10-04T11:59:00Z',
  expiresAt: '2026-10-04T12:01:00Z',
  nonce: 'nonce:reporting:portable',
});
const reportingSignedAt = '2026-10-04T12:00:00Z';
const reportingAttestation = attachReportingEvidencePackageDetachedSignature(pkg, reportingCapability, reportingRequest, {
  requestRoot: reportingRequest.requestRoot,
  signerKeyId: reportingRequest.signerKeyId,
  signedAt: reportingSignedAt,
  signatureBase64: signEd25519(
    null,
    Buffer.from(
      reportingEvidencePackageAttestationPayloadBase64(pkg, reportingCapability, reportingRequest, reportingSignedAt),
      'base64',
    ),
    reportingIssuerKey.privateKey,
  ).toString('base64'),
});
const reportingTrustBundle = createReportingEvidencePackageIssuerTrustBundle({
  bundleId: 'reporting-trust:portable',
  policyAsOf: '2026-10-04T12:30:00Z',
  entries: [{
    entryId: 'reporting-trust-entry:portable',
    issuerId: reportingCapability.issuerId,
    signerKeyId: reportingCapability.signerKeyId,
    capabilityRoot: reportingCapability.capabilityRoot,
    publicKeyPem: reportingIssuerPublicKeyPem,
    validFrom: '2026-10-03T00:00:00Z',
    validUntil: '2027-01-01T00:00:00Z',
  }],
});
const reportingRootRequest = createReportingEvidenceIssuerTrustBundleAttestationRequest(reportingTrustBundle, {
  requestId: 'reporting-root-request:portable',
  anchorId: 'reporting-root:portable',
  signerKeyId: 'reporting-root-key:portable',
  policySequence: '1',
  issuedAt: '2026-10-04T12:30:00Z',
  expiresAt: '2026-12-31T00:00:00Z',
  nonce: 'nonce:reporting-root:portable',
});
const reportingRootSignedAt = '2026-10-04T12:30:01Z';
const reportingRootAttestation = attachReportingEvidenceIssuerTrustBundleDetachedSignature(
  reportingTrustBundle,
  reportingRootRequest,
  {
    signerKeyId: reportingRootRequest.signerKeyId,
    signedAt: reportingRootSignedAt,
    signatureBase64: signEd25519(
      null,
      Buffer.from(
        reportingEvidenceIssuerTrustBundleAttestationPayloadBase64(
          reportingRootRequest,
          reportingTrustBundle,
          reportingRootSignedAt,
        ),
        'base64',
      ),
      reportingRootKey.privateKey,
    ).toString('base64'),
  },
);
const reportingRootPolicy = {
  anchorId: reportingRootRequest.anchorId,
  signerKeyId: reportingRootRequest.signerKeyId,
  publicKeyPem: reportingRootPublicKeyPem,
  validFrom: '2026-10-03T00:00:00Z',
  validUntil: '2027-01-01T00:00:00Z',
  minimumPolicySequence: '1',
} as const;

const portableBundle = createPortableReportingEvidenceBundle({
  capsule,
  auditCapability,
  auditAttestation,
  auditIssuerTrustBundle: auditTrustBundle,
  auditIssuerTrustAttestation: auditRootAttestation,
  package: pkg,
  reportingCapability,
  reportingAttestation,
  reportingIssuerTrustBundle: reportingTrustBundle,
  reportingIssuerTrustAttestation: reportingRootAttestation,
});
const wireText = serializePortableReportingEvidenceBundle(portableBundle);
const verifiedAt = '2026-10-04T13:00:00Z';

describe('portable reporting evidence verification', () => {
  it('replays both root-attested trust chains from canonical wire bytes', () => {
    expect(() => JSON.stringify(portableBundle)).toThrow(/BigInt|bigint/);
    expect(() => JSON.parse(wireText)).not.toThrow();
    expect(wireText).not.toContain(auditRootPublicKeyPem);
    expect(wireText).not.toContain(reportingRootPublicKeyPem);

    const authority = verifyPortableReportingEvidenceWire(wireText, {
      auditIssuerRoot: auditRootPolicy,
      reportingIssuerRoot: reportingRootPolicy,
      verifiedAt,
    });
    expect(authority.receipt.bundleRoot).toBe(portableBundle.bundleRoot);
    expect(authority.receipt.packageRoot).toBe(pkg.packageRoot);
    expect(authority.receipt.auditTrustAnchorId).toBe(auditRootPolicy.anchorId);
    expect(authority.receipt.reportingTrustAnchorId).toBe(reportingRootPolicy.anchorId);
    expect(authority.receipt.lineEvidenceScope).toBe('issuer_attested_compiler_derived_merkle_not_zero_knowledge');
    expect(authority.receipt.receiptRoot).toMatch(/^[0-9a-f]{64}$/);
    expect(requireVerifiedPortableReportingEvidence(authority)).toBe(authority);
  });

  it('rejects transport-level top-level field injection', () => {
    const decoded = deserializeAccountingWire(wireText) as Record<string, unknown>;
    const injected = serializeAccountingWire({ ...decoded, unexpected: 'field' });
    expect(() => verifyPortableReportingEvidenceWire(injected, {
      auditIssuerRoot: auditRootPolicy,
      reportingIssuerRoot: reportingRootPolicy,
      verifiedAt,
    })).toThrow(/unexpected top-level fields/);
  });

  it('rejects package content tampering even when the old packageRoot is retained', () => {
    const decoded = deserializeAccountingWire(wireText) as typeof portableBundle;
    const projection = decoded.package.projection;
    if (projection.format !== 'DDEX') throw new Error('portable fixture must be DDEX');
    const tampered = {
      ...decoded,
      package: {
        ...decoded.package,
        projection: {
          ...projection,
          lines: projection.lines.map((line, index) => index === 0
            ? { ...line, workReference: 'isrc:tampered' }
            : line),
        },
      },
    };
    const tamperedWire = serializeAccountingWire(tampered);
    expect(() => verifyPortableReportingEvidenceWire(tamperedWire, {
      auditIssuerRoot: auditRootPolicy,
      reportingIssuerRoot: reportingRootPolicy,
      verifiedAt,
    })).toThrow(/projection root does not match canonical projection contents/);
  });

  it('requires the separately pinned audit root key', () => {
    expect(() => verifyPortableReportingEvidenceWire(wireText, {
      auditIssuerRoot: { ...auditRootPolicy, publicKeyPem: wrongRootPublicKeyPem },
      reportingIssuerRoot: reportingRootPolicy,
      verifiedAt,
    })).toThrow(/signature verification failed/);
  });

  it('enforces reporting anti-rollback floor outside the wire artifact', () => {
    expect(() => verifyPortableReportingEvidenceWire(wireText, {
      auditIssuerRoot: auditRootPolicy,
      reportingIssuerRoot: { ...reportingRootPolicy, minimumPolicySequence: '2' },
      verifiedAt,
    })).toThrow(/below anti-rollback floor/);
  });

  it('does not let structural clones regain portable verification authority', () => {
    const authority = verifyPortableReportingEvidenceWire(wireText, {
      auditIssuerRoot: auditRootPolicy,
      reportingIssuerRoot: reportingRootPolicy,
      verifiedAt,
    });
    expect(() => requireVerifiedPortableReportingEvidence({ ...authority }))
      .toThrow(/must be produced by canonical wire verification/);
  });
});
