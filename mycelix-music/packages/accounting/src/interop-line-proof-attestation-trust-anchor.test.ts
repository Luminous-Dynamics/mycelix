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
} from './interop-line-proof-attestation-trust.js';
import {
  assertReportingEvidenceIssuerTrustBundleAttestationSuccessor,
  attachReportingEvidenceIssuerTrustBundleDetachedSignature,
  createReportingEvidenceIssuerTrustBundleAttestationRequest,
  reportingEvidenceIssuerTrustBundleAttestationPayloadBase64,
  requireAnchoredReportingEvidencePackageIssuerAuthority,
  verifyReportingEvidenceIssuerTrustBundleAttestation,
  verifyReportingEvidencePackageAttestationWithAnchoredIssuerTrust,
} from './interop-line-proof-attestation-trust-anchor.js';
import {
  createReportingSafeDisclosureCommitment,
  createReportingSafeObligationEvidence,
  type AttestedReportingEvidencePackage,
} from './interop-line-proofs.js';
import type { AttestedAuditReportingReference, AttestedDdexRoyaltyReportingProjection } from './interop.js';
import { buildMerkleCommitment } from './merkle.js';
import { money } from './money.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';
import { StatementKind } from './statements.js';

function digest(seed: string): string {
  if (!/^[0-9a-f]$/.test(seed)) throw new Error('test digest seed must be one hex character');
  return seed.repeat(64);
}

const packageKey = generateKeyPairSync('ed25519');
const rootKey = generateKeyPairSync('ed25519');
const wrongRootKey = generateKeyPairSync('ed25519');
const packagePublicKeyPem = packageKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const rootPublicKeyPem = rootKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();

const obligation = createRoyaltyObligationAuthority({
  id: 'obligation:reporting-anchor:1',
  beneficiaryId: 'creator:reporting-anchor',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:private:reporting-anchor',
    rightsResolutionRef: 'rights:private:reporting-anchor',
    economicTermsRef: 'terms:private:reporting-anchor',
  },
});
const obligations = [obligation] as const;
const epoch: SettlementEpoch = {
  id: 'epoch:reporting-anchor',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const capsule = createEconomicAuditCapsule({
  statementInput: {
    statementId: 'statement:reporting-anchor',
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
      id: 'eligibility:reporting-anchor:1',
      obligationId: obligation.id,
      code: SettlementEligibilityCode.Eligible,
      sourceRef: 'eligibility-source:reporting-anchor:1',
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
  issuerTrustAnchorId: 'audit-root:reporting-anchor',
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
  workReference: 'isrc:reporting-anchor:1',
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

const capability = createReportingEvidencePackageAttestationCapability({
  capabilityId: 'reporting-anchor-capability:1',
  issuerId: 'reporting-anchor-issuer',
  signerKeyId: 'reporting-anchor-package-key',
  compilerId: capsule.generatedBy.id,
  compilerBuildDigest: capsule.generatedBy.buildDigest,
  allowedProfiles: [{ format: 'DDEX', profileVersion: projection.profileVersion }],
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2026-11-01T00:00:00Z',
});
const request = createReportingEvidencePackageAttestationRequest(pkg, capability, {
  requestId: 'reporting-anchor-package-request:1',
  issuedAt: '2026-10-04T11:59:00Z',
  expiresAt: '2026-10-04T12:01:00Z',
  nonce: 'nonce:reporting-anchor-package',
});
const packageSignedAt = '2026-10-04T12:00:00Z';
const packageAttestation = attachReportingEvidencePackageDetachedSignature(pkg, capability, request, {
  requestRoot: request.requestRoot,
  signerKeyId: request.signerKeyId,
  signedAt: packageSignedAt,
  signatureBase64: signEd25519(
    null,
    Buffer.from(reportingEvidencePackageAttestationPayloadBase64(pkg, capability, request, packageSignedAt), 'base64'),
    packageKey.privateKey,
  ).toString('base64'),
});

const bundle1 = createReportingEvidencePackageIssuerTrustBundle({
  bundleId: 'reporting-anchor-bundle:1',
  policyAsOf: '2026-10-04T12:30:00Z',
  entries: [{
    entryId: 'reporting-anchor-entry:1',
    issuerId: capability.issuerId,
    signerKeyId: capability.signerKeyId,
    capabilityRoot: capability.capabilityRoot,
    publicKeyPem: packagePublicKeyPem,
    validFrom: '2026-10-01T00:00:00Z',
    validUntil: '2026-11-01T00:00:00Z',
  }],
});

const anchorPolicy = {
  anchorId: 'reporting-anchor-root',
  signerKeyId: 'reporting-anchor-root-key',
  publicKeyPem: rootPublicKeyPem,
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2027-01-01T00:00:00Z',
  minimumPolicySequence: '1',
} as const;

function signedRootAttestation(
  bundle = bundle1,
  policySequence = '1',
  predecessorBundleRoot?: string,
  signedAt = '2026-10-04T12:40:00Z',
) {
  const rootRequest = createReportingEvidenceIssuerTrustBundleAttestationRequest(bundle, {
    requestId: `reporting-anchor-root-request:${policySequence}`,
    anchorId: anchorPolicy.anchorId,
    signerKeyId: anchorPolicy.signerKeyId,
    policySequence,
    ...(predecessorBundleRoot === undefined ? {} : { predecessorBundleRoot }),
    issuedAt: bundle.policyAsOf,
    expiresAt: '2026-12-31T00:00:00Z',
    nonce: `nonce:reporting-anchor-root:${policySequence}`,
  });
  return attachReportingEvidenceIssuerTrustBundleDetachedSignature(bundle, rootRequest, {
    signerKeyId: rootRequest.signerKeyId,
    signedAt,
    signatureBase64: signEd25519(
      null,
      Buffer.from(reportingEvidenceIssuerTrustBundleAttestationPayloadBase64(rootRequest, bundle, signedAt), 'base64'),
      rootKey.privateKey,
    ).toString('base64'),
  });
}

describe('root-attested reporting evidence issuer trust', () => {
  it('verifies the complete package -> issuer trust -> root anchor chain', () => {
    const authority = verifyReportingEvidencePackageAttestationWithAnchoredIssuerTrust(
      pkg,
      capability,
      packageAttestation,
      bundle1,
      signedRootAttestation(),
      anchorPolicy,
      '2026-10-04T13:00:00Z',
    );
    expect(authority.receipt.packageRoot).toBe(pkg.packageRoot);
    expect(authority.receipt.trustBundleRoot).toBe(bundle1.bundleRoot);
    expect(authority.receipt.trustPolicySequence).toBe('1');
    expect(authority.receipt.receiptRoot).toMatch(/^[0-9a-f]{64}$/);
    expect(requireAnchoredReportingEvidencePackageIssuerAuthority(authority)).toBe(authority);
  });

  it('rejects a cryptographically valid trust bundle below the pinned anti-rollback floor', () => {
    expect(() => verifyReportingEvidenceIssuerTrustBundleAttestation(
      bundle1,
      signedRootAttestation(),
      { ...anchorPolicy, minimumPolicySequence: '2' },
    )).toThrow(/below anti-rollback floor/);
  });

  it('uses a half-open root-signature request window', () => {
    const rootRequest = createReportingEvidenceIssuerTrustBundleAttestationRequest(bundle1, {
      requestId: 'reporting-anchor-expiry-request',
      anchorId: anchorPolicy.anchorId,
      signerKeyId: anchorPolicy.signerKeyId,
      policySequence: '1',
      issuedAt: bundle1.policyAsOf,
      expiresAt: '2026-10-04T12:40:00Z',
      nonce: 'nonce:reporting-anchor-expiry',
    });
    expect(() => reportingEvidenceIssuerTrustBundleAttestationPayloadBase64(
      rootRequest,
      bundle1,
      rootRequest.expiresAt,
    )).toThrow(/outside request window/);
  });

  it('rejects a root signature from the wrong Ed25519 key', () => {
    const rootRequest = createReportingEvidenceIssuerTrustBundleAttestationRequest(bundle1, {
      requestId: 'reporting-anchor-wrong-key-request',
      anchorId: anchorPolicy.anchorId,
      signerKeyId: anchorPolicy.signerKeyId,
      policySequence: '1',
      issuedAt: bundle1.policyAsOf,
      expiresAt: '2026-12-31T00:00:00Z',
      nonce: 'nonce:reporting-anchor-wrong-key',
    });
    const signedAt = '2026-10-04T12:40:00Z';
    const wrong = attachReportingEvidenceIssuerTrustBundleDetachedSignature(bundle1, rootRequest, {
      signerKeyId: rootRequest.signerKeyId,
      signedAt,
      signatureBase64: signEd25519(
        null,
        Buffer.from(reportingEvidenceIssuerTrustBundleAttestationPayloadBase64(rootRequest, bundle1, signedAt), 'base64'),
        wrongRootKey.privateKey,
      ).toString('base64'),
    });
    expect(() => verifyReportingEvidenceIssuerTrustBundleAttestation(bundle1, wrong, anchorPolicy))
      .toThrow(/signature verification failed/);
  });

  it('rejects root attestations signed after the requested package verification instant', () => {
    expect(() => verifyReportingEvidencePackageAttestationWithAnchoredIssuerTrust(
      pkg,
      capability,
      packageAttestation,
      bundle1,
      signedRootAttestation(bundle1, '1', undefined, '2026-10-04T14:00:00Z'),
      anchorPolicy,
      '2026-10-04T13:00:00Z',
    )).toThrow(/cannot authorize an earlier verification view/);
  });

  it('requires exact predecessor continuity and one-step policy sequence advancement', () => {
    const predecessor = verifyReportingEvidenceIssuerTrustBundleAttestation(bundle1, signedRootAttestation(), anchorPolicy);
    const bundle2 = createReportingEvidencePackageIssuerTrustBundle({
      bundleId: 'reporting-anchor-bundle:2',
      policyAsOf: '2026-10-05T12:30:00Z',
      entries: bundle1.entries,
    });
    const successor = verifyReportingEvidenceIssuerTrustBundleAttestation(
      bundle2,
      signedRootAttestation(bundle2, '2', bundle1.bundleRoot, '2026-10-05T12:40:00Z'),
      anchorPolicy,
    );
    expect(() => assertReportingEvidenceIssuerTrustBundleAttestationSuccessor(predecessor, successor)).not.toThrow();

    const skipped = verifyReportingEvidenceIssuerTrustBundleAttestation(
      bundle2,
      signedRootAttestation(bundle2, '3', bundle1.bundleRoot, '2026-10-05T12:41:00Z'),
      anchorPolicy,
    );
    expect(() => assertReportingEvidenceIssuerTrustBundleAttestationSuccessor(predecessor, skipped))
      .toThrow(/advance policy sequence exactly once/);
  });

  it('does not let structural clones regain anchored package issuer authority', () => {
    const authority = verifyReportingEvidencePackageAttestationWithAnchoredIssuerTrust(
      pkg,
      capability,
      packageAttestation,
      bundle1,
      signedRootAttestation(),
      anchorPolicy,
      '2026-10-04T13:00:00Z',
    );
    expect(() => requireAnchoredReportingEvidencePackageIssuerAuthority({ ...authority }))
      .toThrow(/must be produced by root-anchored verification/);
  });
});
