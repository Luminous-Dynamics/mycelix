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
  assertEconomicAuditIssuerTrustBundleAttestationSuccessor,
  attachEconomicAuditIssuerTrustBundleDetachedSignature,
  createEconomicAuditIssuerTrustBundleAttestationRequest,
  economicAuditIssuerTrustBundleAttestationPayloadBase64,
  requireAnchoredEconomicAuditCapsuleIssuerAuthority,
  verifyEconomicAuditCapsuleAttestationWithAnchoredIssuerTrust,
  verifyEconomicAuditIssuerTrustBundleAttestation,
  type EconomicAuditIssuerTrustAnchorPolicy,
} from './audit-capsule-attestation-trust-anchor.js';
import { money } from './money.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';
import { StatementKind } from './statements.js';

const issuerKey = generateKeyPairSync('ed25519');
const rootKey = generateKeyPairSync('ed25519');
const wrongRootKey = generateKeyPairSync('ed25519');
const issuerPublicKeyPem = issuerKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const rootPublicKeyPem = rootKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();

function digest(seed: string): string {
  if (!/^[0-9a-f]$/.test(seed)) throw new Error('test digest seed must be one hex character');
  return seed.repeat(64);
}

const obligation = createRoyaltyObligationAuthority({
  id: 'obl:audit-issuer-anchor',
  beneficiaryId: 'artist:audit-issuer-anchor',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:audit-issuer-anchor',
    rightsResolutionRef: 'rights:audit-issuer-anchor',
    economicTermsRef: 'terms:audit-issuer-anchor',
  },
});
const epoch: SettlementEpoch = {
  id: 'epoch:audit-issuer-anchor',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const eligibility = [{
  id: 'elig:audit-issuer-anchor',
  obligationId: obligation.id,
  code: SettlementEligibilityCode.Eligible,
  sourceRef: 'eligibility:audit-issuer-anchor',
  observedAt: '2026-09-30T00:00:00Z',
}] as const;
const capsule = createEconomicAuditCapsule({
  statementInput: {
    statementId: 'statement:audit-issuer-anchor',
    kind: StatementKind.Periodic,
    beneficiaryId: obligation.beneficiaryId,
    period: { startInclusive: '2026-09-01T00:00:00Z', endExclusive: '2026-10-01T00:00:00Z' },
    asOf: '2026-10-02T00:00:00Z',
    completeness: {
      kind: 'complete',
      through: {
        usageObservedThrough: '2026-10-01T00:00:00Z',
        rightsResolvedThrough: '2026-10-01T00:00:00Z',
        settlementsObservedThrough: '2026-10-01T00:00:00Z',
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
  generatedBy: { id: 'mycelix-music-royalty-compiler', version: '2.0.0', buildDigest: digest('9') },
});
const capability = createEconomicAuditCapsuleAttestationCapability({
  capabilityId: 'audit-capability:anchored',
  issuerId: 'audit-issuer:anchored',
  signerKeyId: 'audit-key:anchored',
  compilerId: capsule.generatedBy.id,
  compilerBuildDigest: capsule.generatedBy.buildDigest,
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2027-01-01T00:00:00Z',
});
const issuerSignedAt = '2026-10-04T12:00:00Z';
const issuerRequest = createEconomicAuditCapsuleAttestationRequest(capsule, capability, {
  requestId: 'audit-request:anchored',
  issuedAt: '2026-10-04T11:59:59Z',
  expiresAt: '2026-10-04T12:01:00Z',
  nonce: 'nonce:audit-request:anchored',
});
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

function trustBundle(bundleId: string, policyAsOf: string) {
  return createEconomicAuditCapsuleIssuerTrustBundle({
    bundleId,
    policyAsOf,
    entries: [{
      entryId: `${bundleId}:entry`,
      issuerId: capability.issuerId,
      signerKeyId: capability.signerKeyId,
      capabilityRoot: capability.capabilityRoot,
      publicKeyPem: issuerPublicKeyPem,
      validFrom: '2026-10-01T00:00:00Z',
      validUntil: '2027-01-01T00:00:00Z',
    }],
  });
}

const bundle1 = trustBundle('audit-issuer-bundle:1', '2026-10-04T13:00:00Z');
const bundle2 = trustBundle('audit-issuer-bundle:2', '2026-10-05T13:00:00Z');

function anchor(minimumPolicySequence = '1'): EconomicAuditIssuerTrustAnchorPolicy {
  return {
    anchorId: 'audit-issuer-root-anchor:1',
    signerKeyId: 'audit-issuer-root-key:1',
    publicKeyPem: rootPublicKeyPem,
    validFrom: '2026-10-01T00:00:00Z',
    validUntil: '2027-01-01T00:00:00Z',
    minimumPolicySequence,
  };
}

function bundleAttestation(
  bundle: ReturnType<typeof trustBundle>,
  policySequence: string,
  options: {
    readonly predecessorBundleRoot?: string;
    readonly signedAt?: string;
    readonly expiresAt?: string;
    readonly signer?: typeof rootKey;
  } = {},
) {
  const issuedAt = bundle.policyAsOf;
  const signedAt = options.signedAt ?? new Date(Date.parse(issuedAt) + 1_000).toISOString();
  const request = createEconomicAuditIssuerTrustBundleAttestationRequest(bundle, {
    requestId: `issuer-trust-attestation:${bundle.bundleId}:${policySequence}`,
    anchorId: 'audit-issuer-root-anchor:1',
    signerKeyId: 'audit-issuer-root-key:1',
    policySequence,
    ...(options.predecessorBundleRoot === undefined ? {} : { predecessorBundleRoot: options.predecessorBundleRoot }),
    issuedAt,
    expiresAt: options.expiresAt ?? '2026-12-31T00:00:00Z',
    nonce: `nonce:${bundle.bundleId}:${policySequence}`,
  });
  const signatureBase64 = signEd25519(
    null,
    Buffer.from(economicAuditIssuerTrustBundleAttestationPayloadBase64(request, bundle, signedAt), 'base64'),
    (options.signer ?? rootKey).privateKey,
  ).toString('base64');
  return attachEconomicAuditIssuerTrustBundleDetachedSignature(bundle, request, {
    signerKeyId: request.signerKeyId,
    signedAt,
    signatureBase64,
  });
}

describe('EconomicAuditCapsule issuer trust root anchor', () => {
  it('rejects a cryptographically valid older issuer trust bundle below the pinned rollback floor', () => {
    expect(() => verifyEconomicAuditIssuerTrustBundleAttestation(
      bundle1,
      bundleAttestation(bundle1, '1'),
      anchor('2'),
    )).toThrow(/below anti-rollback floor/);
  });

  it('requires exact predecessor continuity and one-step issuer trust policy advancement', () => {
    const first = verifyEconomicAuditIssuerTrustBundleAttestation(bundle1, bundleAttestation(bundle1, '1'), anchor());
    const second = verifyEconomicAuditIssuerTrustBundleAttestation(
      bundle2,
      bundleAttestation(bundle2, '2', { predecessorBundleRoot: bundle1.bundleRoot }),
      anchor(),
    );
    expect(() => assertEconomicAuditIssuerTrustBundleAttestationSuccessor(first, second)).not.toThrow();

    const bundle3 = trustBundle('audit-issuer-bundle:3', '2026-10-06T13:00:00Z');
    const skipped = verifyEconomicAuditIssuerTrustBundleAttestation(
      bundle3,
      bundleAttestation(bundle3, '3', { predecessorBundleRoot: bundle2.bundleRoot }),
      anchor(),
    );
    expect(() => assertEconomicAuditIssuerTrustBundleAttestationSuccessor(first, skipped))
      .toThrow(/advance policy sequence exactly once/);
  });

  it('rejects a successor that names the wrong predecessor issuer trust bundle root', () => {
    const first = verifyEconomicAuditIssuerTrustBundleAttestation(bundle1, bundleAttestation(bundle1, '1'), anchor());
    const second = verifyEconomicAuditIssuerTrustBundleAttestation(
      bundle2,
      bundleAttestation(bundle2, '2', { predecessorBundleRoot: digest('a') }),
      anchor(),
    );
    expect(() => assertEconomicAuditIssuerTrustBundleAttestationSuccessor(first, second))
      .toThrow(/bind predecessor bundle root/);
  });

  it('uses a half-open root-attestation request window', () => {
    const expiresAt = '2026-10-04T13:01:00Z';
    const request = createEconomicAuditIssuerTrustBundleAttestationRequest(bundle1, {
      requestId: 'issuer-trust-attestation:expiry',
      anchorId: 'audit-issuer-root-anchor:1',
      signerKeyId: 'audit-issuer-root-key:1',
      policySequence: '1',
      issuedAt: bundle1.policyAsOf,
      expiresAt,
      nonce: 'nonce:issuer-trust-expiry',
    });
    expect(() => economicAuditIssuerTrustBundleAttestationPayloadBase64(request, bundle1, expiresAt))
      .toThrow(/outside request window/);
  });

  it('rejects root signatures from a key not pinned by the audit issuer trust anchor', () => {
    expect(() => verifyEconomicAuditIssuerTrustBundleAttestation(
      bundle1,
      bundleAttestation(bundle1, '1', { signer: wrongRootKey }),
      anchor(),
    )).toThrow(/signature verification failed/);
  });

  it('verifies capsule issuer authority through the root-attested trust bundle and emits an anchored receipt', () => {
    const authority = verifyEconomicAuditCapsuleAttestationWithAnchoredIssuerTrust(
      capsule,
      capability,
      issuerAttestation,
      bundle1,
      bundleAttestation(bundle1, '1'),
      anchor(),
      '2026-10-04T14:00:00Z',
    );
    expect(authority.receipt.trustBundleRoot).toBe(bundle1.bundleRoot);
    expect(authority.receipt.trustAnchorId).toBe('audit-issuer-root-anchor:1');
    expect(authority.receipt.trustPolicySequence).toBe('1');
    expect(authority.receipt.capsuleDigest).toBe(authority.issuerAuthority.receipt.capsuleDigest);
    expect(authority.receipt.receiptRoot).toMatch(/^[0-9a-f]{64}$/);
    expect(requireAnchoredEconomicAuditCapsuleIssuerAuthority(authority)).toBe(authority);
  });

  it('rejects a root trust attestation newer than the requested anchored verification instant', () => {
    const futureAttestation = bundleAttestation(bundle1, '1', { signedAt: '2026-10-04T15:00:00Z' });
    expect(() => verifyEconomicAuditCapsuleAttestationWithAnchoredIssuerTrust(
      capsule,
      capability,
      issuerAttestation,
      bundle1,
      futureAttestation,
      anchor(),
      '2026-10-04T14:00:00Z',
    )).toThrow(/cannot be newer than anchored verification time/);
  });

  it('does not let structural clones participate in verified issuer trust succession or anchored authority', () => {
    const first = verifyEconomicAuditIssuerTrustBundleAttestation(bundle1, bundleAttestation(bundle1, '1'), anchor());
    const second = verifyEconomicAuditIssuerTrustBundleAttestation(
      bundle2,
      bundleAttestation(bundle2, '2', { predecessorBundleRoot: bundle1.bundleRoot }),
      anchor(),
    );
    expect(() => assertEconomicAuditIssuerTrustBundleAttestationSuccessor({ ...first }, second))
      .toThrow(/requires verified authorities/);

    const authority = verifyEconomicAuditCapsuleAttestationWithAnchoredIssuerTrust(
      capsule,
      capability,
      issuerAttestation,
      bundle1,
      bundleAttestation(bundle1, '1'),
      anchor(),
      '2026-10-04T14:00:00Z',
    );
    expect(() => requireAnchoredEconomicAuditCapsuleIssuerAuthority({ ...authority }))
      .toThrow(/must be produced by root trust verification/);
  });
});
