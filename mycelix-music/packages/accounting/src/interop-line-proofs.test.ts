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
  createReportingSafeDisclosureCommitment,
  createReportingSafeObligationEvidence,
  exportCisacCrdReportingEvidencePackage,
  exportDdexReportingEvidencePackage,
  verifyReportingSafeObligationEvidence,
} from './interop-line-proofs.js';
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

const obligationA = createRoyaltyObligationAuthority({
  id: 'obligation:proof:a',
  beneficiaryId: 'creator:proof',
  amount: money(200n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:private:a',
    rightsResolutionRef: 'rights:private:a',
    economicTermsRef: 'terms:private:a',
  },
});
const obligationB = createRoyaltyObligationAuthority({
  id: 'obligation:proof:b',
  beneficiaryId: 'creator:proof',
  amount: money(300n, 'USD'),
  observedAt: '2026-09-11T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:private:b',
    rightsResolutionRef: 'rights:private:b',
    economicTermsRef: 'terms:private:b',
  },
});
const obligations = [obligationA, obligationB] as const;
const epoch: SettlementEpoch = {
  id: 'epoch:proof',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const eligibility = [
  {
    id: 'eligibility:proof:a',
    obligationId: obligationA.id,
    code: SettlementEligibilityCode.Eligible,
    sourceRef: 'eligibility-source:proof:a',
    observedAt: '2026-09-30T00:00:00Z',
  },
  {
    id: 'eligibility:proof:b',
    obligationId: obligationB.id,
    code: SettlementEligibilityCode.Eligible,
    sourceRef: 'eligibility-source:proof:b',
    observedAt: '2026-09-30T00:00:00Z',
  },
] as const;

const capsule = createEconomicAuditCapsule({
  statementInput: {
    statementId: 'statement:proof',
    kind: StatementKind.Periodic,
    beneficiaryId: 'creator:proof',
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

const capability = createEconomicAuditCapsuleAttestationCapability({
  capabilityId: 'audit-capability:proof',
  issuerId: 'audit-issuer:proof',
  signerKeyId: 'audit-key:proof',
  compilerId: capsule.generatedBy.id,
  compilerBuildDigest: capsule.generatedBy.buildDigest,
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2027-01-01T00:00:00Z',
});
const issuerRequest = createEconomicAuditCapsuleAttestationRequest(capsule, capability, {
  requestId: 'audit-request:proof',
  issuedAt: '2026-10-03T11:59:59Z',
  expiresAt: '2026-10-03T12:01:00Z',
  nonce: 'nonce:audit-request:proof',
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
  bundleId: 'audit-issuer-bundle:proof',
  policyAsOf: '2026-10-03T13:00:00Z',
  entries: [{
    entryId: 'audit-issuer-entry:proof',
    issuerId: capability.issuerId,
    signerKeyId: capability.signerKeyId,
    capabilityRoot: capability.capabilityRoot,
    publicKeyPem: issuerPublicKeyPem,
    validFrom: '2026-10-01T00:00:00Z',
    validUntil: '2027-01-01T00:00:00Z',
  }],
});
const rootRequest = createEconomicAuditIssuerTrustBundleAttestationRequest(trustBundle, {
  requestId: 'audit-root-request:proof',
  anchorId: 'audit-root-anchor:proof',
  signerKeyId: 'audit-root-key:proof',
  policySequence: '1',
  issuedAt: '2026-10-03T13:00:00Z',
  expiresAt: '2026-12-31T00:00:00Z',
  nonce: 'nonce:audit-root:proof',
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

const lines = [
  {
    obligationId: obligationA.id,
    workReference: 'isrc:proof:a',
    beneficiaryReference: obligationA.beneficiaryId,
    amount: obligationA.amount,
    usageReference: 'usage:public:a',
    territory: 'ZA',
  },
  {
    obligationId: obligationB.id,
    workReference: 'isrc:proof:b',
    beneficiaryReference: obligationB.beneficiaryId,
    amount: obligationB.amount,
    usageReference: 'usage:public:b',
    territory: 'US',
  },
] as const;

function packageInput() {
  return {
    profileVersion: 'configured-profile-v1',
    capsule,
    anchoredIssuer,
    generatedAt: '2026-10-03T15:00:00Z',
    lines,
    obligations,
  } as const;
}

describe('reporting-safe obligation line proofs', () => {
  it('reconstructs the private obligation root before committing public-safe leaves', () => {
    const commitment = createReportingSafeDisclosureCommitment({ capsule, obligations });
    expect(commitment.internalObligationRoot).toBe(capsule.statement.obligationRoot);
    expect(commitment.disclosureCount).toBe(2);
    expect(commitment.authority).toBe('compiler_derived_sidecar_not_issuer_attested');
    expect(commitment.sidecarRoot).toMatch(/^[0-9a-f]{64}$/);
  });

  it('proves a safe obligation leaf without disclosing internal provenance references', () => {
    const evidence = createReportingSafeObligationEvidence({
      capsule,
      obligations,
      obligationId: obligationA.id,
    });
    expect(verifyReportingSafeObligationEvidence(evidence)).toBe(true);
    expect(evidence.leaf.obligationId).toBe(obligationA.id);
    expect(evidence.leaf.authorityRoot).toBe(obligationA.authorityRoot);
    expect(evidence.fieldEvidenceScope).toBe('obligation_id_authority_root_beneficiary_amount_currency_only');
    expect(Object.keys(evidence.leaf)).not.toContain('usageEvidenceRef');
    expect(Object.keys(evidence.leaf)).not.toContain('rightsResolutionRef');
    expect(Object.keys(evidence.leaf)).not.toContain('economicTermsRef');
  });

  it('rejects tampered public-safe Merkle leaves', () => {
    const evidence = createReportingSafeObligationEvidence({ capsule, obligations, obligationId: obligationA.id });
    expect(verifyReportingSafeObligationEvidence({
      ...evidence,
      leaf: { ...evidence.leaf, amountMinor: '201' },
    })).toBe(false);
  });

  it('fails closed when the supplied obligation set omits audited debt', () => {
    expect(() => createReportingSafeDisclosureCommitment({ capsule, obligations: [obligationA] }))
      .toThrow(/do not reconstruct audited internal obligation root/);
  });

  it('packages DDEX projection with one safe proof per exported obligation line', () => {
    const pkg = exportDdexReportingEvidencePackage(packageInput());
    expect(pkg.authority).toBe('reporting_projection_only');
    expect(pkg.evidenceStatus).toBe('compiler_derived_line_evidence_not_issuer_attested');
    expect(pkg.lineEvidence).toHaveLength(2);
    expect(pkg.lineEvidence.every(verifyReportingSafeObligationEvidence)).toBe(true);
    expect(pkg.packageRoot).toMatch(/^[0-9a-f]{64}$/);
  });

  it('packages CRD projection without turning line proofs into ledger authority', () => {
    const pkg = exportCisacCrdReportingEvidencePackage({
      ...packageInput(),
      profileVersion: 'configured-crd-profile-v1',
    });
    expect(pkg.projection.format).toBe('CISAC_CRD');
    expect(pkg.projection.authority).toBe('reporting_projection_only');
    expect(pkg.disclosure.authority).toBe('compiler_derived_sidecar_not_issuer_attested');
  });

  it('rejects a reporting line amount that does not match its proved obligation', () => {
    expect(() => exportDdexReportingEvidencePackage({
      ...packageInput(),
      lines: [{ ...lines[0], amount: money(199n, 'USD') }, lines[1]],
    })).toThrow(/amount does not match line evidence/);
  });
});
