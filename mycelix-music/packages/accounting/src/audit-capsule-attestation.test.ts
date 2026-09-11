import { generateKeyPairSync, sign as signEd25519 } from 'node:crypto';
import { describe, expect, it } from 'vitest';
import { createEconomicAuditCapsule, economicAuditCapsuleDigest } from './audit.js';
import {
  ECONOMIC_AUDIT_CAPSULE_ATTESTATION_SCOPE,
  attachEconomicAuditCapsuleDetachedSignature,
  createEconomicAuditCapsuleAttestationCapability,
  createEconomicAuditCapsuleAttestationRequest,
  economicAuditCapsuleAttestationPayloadBase64,
  requireVerifiedEconomicAuditCapsuleAttestation,
  verifyEconomicAuditCapsuleAttestation,
} from './audit-capsule-attestation.js';
import { money } from './money.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';
import { StatementKind } from './statements.js';

const issuerKey = generateKeyPairSync('ed25519');
const wrongKey = generateKeyPairSync('ed25519');
const issuerPublicKeyPem = issuerKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const asOf = '2026-10-02T00:00:00Z';

function digest(seed: string): string {
  if (!/^[0-9a-f]$/.test(seed)) throw new Error('test digest seed must be one hex character');
  return seed.repeat(64);
}

const obligation = createRoyaltyObligationAuthority({
  id: 'obl:audit-attestation',
  beneficiaryId: 'artist:audit-attestation',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:audit-attestation',
    rightsResolutionRef: 'rights:audit-attestation',
    economicTermsRef: 'terms:audit-attestation',
  },
});
const epoch: SettlementEpoch = {
  id: 'epoch:audit-attestation',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const eligibility = [{
  id: 'elig:audit-attestation',
  obligationId: obligation.id,
  code: SettlementEligibilityCode.Eligible,
  sourceRef: 'eligibility:audit-attestation',
  observedAt: '2026-09-30T00:00:00Z',
}] as const;

function capsule(usageCommitment = digest('1')) {
  return createEconomicAuditCapsule({
    statementInput: {
      statementId: 'statement:audit-attestation',
      kind: StatementKind.Periodic,
      beneficiaryId: obligation.beneficiaryId,
      period: { startInclusive: '2026-09-01T00:00:00Z', endExclusive: '2026-10-01T00:00:00Z' },
      asOf,
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
    usageCommitment,
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
}

const canonicalCapsule = capsule();
const capability = createEconomicAuditCapsuleAttestationCapability({
  capabilityId: 'audit-attestation-capability:prod',
  issuerId: 'audit-issuer:prod',
  signerKeyId: 'audit-issuer-key:1',
  compilerId: canonicalCapsule.generatedBy.id,
  compilerBuildDigest: canonicalCapsule.generatedBy.buildDigest,
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2027-01-01T00:00:00Z',
});
const issuedAt = '2026-10-02T00:01:00Z';
const expiresAt = '2026-10-02T00:02:00Z';
const signedAt = '2026-10-02T00:01:30Z';

function requestFor(target = canonicalCapsule) {
  return createEconomicAuditCapsuleAttestationRequest(target, capability, {
    requestId: `audit-request:${target.statement.statementId}`,
    issuedAt,
    expiresAt,
    nonce: `nonce:${economicAuditCapsuleDigest(target)}`,
  });
}

function attestationFor(
  target = canonicalCapsule,
  signer: typeof issuerKey = issuerKey,
) {
  const request = requestFor(target);
  const payload = Buffer.from(economicAuditCapsuleAttestationPayloadBase64(request, signedAt), 'base64');
  const signatureBase64 = signEd25519(null, payload, signer.privateKey).toString('base64');
  return attachEconomicAuditCapsuleDetachedSignature(target, capability, request, {
    requestRoot: request.requestRoot,
    signerKeyId: request.signerKeyId,
    signedAt,
    signatureBase64,
  });
}

function policy(capabilityRoot = capability.capabilityRoot) {
  return {
    issuerId: capability.issuerId,
    signerKeyId: capability.signerKeyId,
    publicKeyPem: issuerPublicKeyPem,
    capabilityRoot,
    validFrom: '2026-10-01T00:00:00Z',
    validUntil: '2027-01-01T00:00:00Z',
  };
}

describe('EconomicAuditCapsule detached issuer attestation', () => {
  it('verifies a capability-scoped detached signature and emits a root-bearing receipt', () => {
    expect(capability.scope).toBe(ECONOMIC_AUDIT_CAPSULE_ATTESTATION_SCOPE);
    const verified = verifyEconomicAuditCapsuleAttestation(
      canonicalCapsule,
      capability,
      attestationFor(),
      policy(),
      '2026-10-02T00:03:00Z',
    );
    expect(verified.receipt.capsuleDigest).toBe(economicAuditCapsuleDigest(canonicalCapsule));
    expect(verified.receipt.statementSnapshotRoot).toBe(canonicalCapsule.statementSnapshotRoot);
    expect(verified.receipt.capabilityRoot).toBe(capability.capabilityRoot);
    expect(verified.receipt.receiptRoot).toMatch(/^[0-9a-f]{64}$/);
    expect(requireVerifiedEconomicAuditCapsuleAttestation(verified)).toBe(verified);
  });

  it('rejects a capability that does not authorize the exact compiler build', () => {
    const wrongCapability = createEconomicAuditCapsuleAttestationCapability({
      capabilityId: 'audit-attestation-capability:wrong-build',
      issuerId: capability.issuerId,
      signerKeyId: capability.signerKeyId,
      compilerId: canonicalCapsule.generatedBy.id,
      compilerBuildDigest: digest('a'),
      validFrom: capability.validFrom,
      validUntil: capability.validUntil,
    });
    expect(() => createEconomicAuditCapsuleAttestationRequest(canonicalCapsule, wrongCapability, {
      requestId: 'request:wrong-build', issuedAt, expiresAt, nonce: 'nonce:wrong-build',
    })).toThrow(/compiler identity is not authorized/);
  });

  it('rejects capability content changed behind an unchanged capability root', () => {
    const forged = { ...capability, validUntil: '2028-01-01T00:00:00Z' };
    expect(() => createEconomicAuditCapsuleAttestationRequest(canonicalCapsule, forged, {
      requestId: 'request:forged-capability', issuedAt, expiresAt, nonce: 'nonce:forged-capability',
    })).toThrow(/capability root does not match canonical contents/);
  });

  it('rejects rebinding a request or signature to another capsule', () => {
    const otherCapsule = capsule(digest('a'));
    const request = requestFor();
    const payload = Buffer.from(economicAuditCapsuleAttestationPayloadBase64(request, signedAt), 'base64');
    const signatureBase64 = signEd25519(null, payload, issuerKey.privateKey).toString('base64');
    expect(() => attachEconomicAuditCapsuleDetachedSignature(otherCapsule, capability, request, {
      requestRoot: request.requestRoot,
      signerKeyId: request.signerKeyId,
      signedAt,
      signatureBase64,
    })).toThrow(/does not match exact capsule identity/);
    expect(() => attachEconomicAuditCapsuleDetachedSignature(canonicalCapsule, capability, request, {
      requestRoot: digest('b'),
      signerKeyId: request.signerKeyId,
      signedAt,
      signatureBase64,
    })).toThrow(/does not bind canonical request/);
  });

  it('uses a half-open request signing window', () => {
    const request = requestFor();
    expect(() => economicAuditCapsuleAttestationPayloadBase64(request, expiresAt))
      .toThrow(/outside request window/);
  });

  it('rejects signatures from an untrusted key', () => {
    expect(() => verifyEconomicAuditCapsuleAttestation(
      canonicalCapsule,
      capability,
      attestationFor(canonicalCapsule, wrongKey),
      policy(),
      '2026-10-02T00:03:00Z',
    )).toThrow(/signature verification failed/);
  });

  it('requires issuer trust to pin the exact capability root', () => {
    expect(() => verifyEconomicAuditCapsuleAttestation(
      canonicalCapsule,
      capability,
      attestationFor(),
      policy(digest('c')),
      '2026-10-02T00:03:00Z',
    )).toThrow(/not authorized by pinned issuer trust policy/);
  });

  it('rejects structural clones of verified attestation authority', () => {
    const verified = verifyEconomicAuditCapsuleAttestation(
      canonicalCapsule,
      capability,
      attestationFor(),
      policy(),
      '2026-10-02T00:03:00Z',
    );
    expect(() => requireVerifiedEconomicAuditCapsuleAttestation({ ...verified }))
      .toThrow(/must be produced by cryptographic verification/);
  });
});
