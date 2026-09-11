import { generateKeyPairSync, sign as signEd25519 } from 'node:crypto';
import { describe, expect, it } from 'vitest';
import { createEconomicAuditCapsule } from './audit.js';
import {
  attachEconomicAuditCapsuleDetachedSignature,
  createEconomicAuditCapsuleAttestationCapability,
  createEconomicAuditCapsuleAttestationRequest,
  economicAuditCapsuleAttestationPayloadBase64,
} from './audit-capsule-attestation.js';
import {
  createEconomicAuditCapsuleIssuerTrustBundle,
  requireVerifiedEconomicAuditCapsuleIssuerAuthority,
  verifyEconomicAuditCapsuleAttestationWithTrustBundle,
} from './audit-capsule-attestation-trust.js';
import { money } from './money.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';
import { StatementKind } from './statements.js';

const oldKey = generateKeyPairSync('ed25519');
const newKey = generateKeyPairSync('ed25519');
const oldPublicKeyPem = oldKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const newPublicKeyPem = newKey.publicKey.export({ format: 'pem', type: 'spki' }).toString();

function digest(seed: string): string {
  if (!/^[0-9a-f]$/.test(seed)) throw new Error('test digest seed must be one hex character');
  return seed.repeat(64);
}

const obligation = createRoyaltyObligationAuthority({
  id: 'obl:audit-issuer-trust',
  beneficiaryId: 'artist:audit-issuer-trust',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:audit-issuer-trust',
    rightsResolutionRef: 'rights:audit-issuer-trust',
    economicTermsRef: 'terms:audit-issuer-trust',
  },
});
const epoch: SettlementEpoch = {
  id: 'epoch:audit-issuer-trust',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const eligibility = [{
  id: 'elig:audit-issuer-trust',
  obligationId: obligation.id,
  code: SettlementEligibilityCode.Eligible,
  sourceRef: 'eligibility:audit-issuer-trust',
  observedAt: '2026-09-30T00:00:00Z',
}] as const;
const capsule = createEconomicAuditCapsule({
  statementInput: {
    statementId: 'statement:audit-issuer-trust',
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
  generatedBy: {
    id: 'mycelix-music-royalty-compiler',
    version: '2.0.0',
    buildDigest: digest('9'),
  },
});

const oldCapability = createEconomicAuditCapsuleAttestationCapability({
  capabilityId: 'audit-capability:old',
  issuerId: 'audit-issuer:prod',
  signerKeyId: 'audit-key:old',
  compilerId: capsule.generatedBy.id,
  compilerBuildDigest: capsule.generatedBy.buildDigest,
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2027-01-01T00:00:00Z',
});
const newCapability = createEconomicAuditCapsuleAttestationCapability({
  capabilityId: 'audit-capability:new',
  issuerId: 'audit-issuer:prod',
  signerKeyId: 'audit-key:new',
  compilerId: capsule.generatedBy.id,
  compilerBuildDigest: capsule.generatedBy.buildDigest,
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2027-01-01T00:00:00Z',
});

function attestationFor(
  capability: typeof oldCapability,
  signer: typeof oldKey,
  signedAt: string,
) {
  const issuedAt = new Date(Date.parse(signedAt) - 1_000).toISOString();
  const expiresAt = new Date(Date.parse(signedAt) + 60_000).toISOString();
  const request = createEconomicAuditCapsuleAttestationRequest(capsule, capability, {
    requestId: `request:${capability.signerKeyId}:${signedAt}`,
    issuedAt,
    expiresAt,
    nonce: `nonce:${capability.capabilityRoot}:${signedAt}`,
  });
  const payload = Buffer.from(economicAuditCapsuleAttestationPayloadBase64(request, signedAt), 'base64');
  const signatureBase64 = signEd25519(null, payload, signer.privateKey).toString('base64');
  return attachEconomicAuditCapsuleDetachedSignature(capsule, capability, request, {
    requestRoot: request.requestRoot,
    signerKeyId: request.signerKeyId,
    signedAt,
    signatureBase64,
  });
}

function entry(input: {
  readonly entryId: string;
  readonly capability: typeof oldCapability;
  readonly publicKeyPem: string;
  readonly validFrom: string;
  readonly validUntil: string;
  readonly revokedAt?: string;
}) {
  return {
    entryId: input.entryId,
    issuerId: input.capability.issuerId,
    signerKeyId: input.capability.signerKeyId,
    capabilityRoot: input.capability.capabilityRoot,
    publicKeyPem: input.publicKeyPem,
    validFrom: input.validFrom,
    validUntil: input.validUntil,
    ...(input.revokedAt === undefined ? {} : { revokedAt: input.revokedAt }),
  };
}

const oldEntry = entry({
  entryId: 'issuer-trust:old',
  capability: oldCapability,
  publicKeyPem: oldPublicKeyPem,
  validFrom: '2026-10-01T00:00:00Z',
  validUntil: '2026-10-05T00:00:00Z',
});
const newEntry = entry({
  entryId: 'issuer-trust:new',
  capability: newCapability,
  publicKeyPem: newPublicKeyPem,
  validFrom: '2026-10-05T00:00:00Z',
  validUntil: '2027-01-01T00:00:00Z',
});

function bundle(
  policyAsOf: string,
  entries = [oldEntry, newEntry],
  bundleId = `audit-issuer-trust:${policyAsOf}`,
) {
  return createEconomicAuditCapsuleIssuerTrustBundle({ bundleId, policyAsOf, entries });
}

describe('EconomicAuditCapsule issuer trust rotation', () => {
  it('preserves historical old-key verification after rotation and emits an auditable receipt', () => {
    const signedAt = '2026-10-04T12:00:00Z';
    const trust = bundle('2026-10-06T00:00:00Z');
    const authority = verifyEconomicAuditCapsuleAttestationWithTrustBundle(
      capsule,
      oldCapability,
      attestationFor(oldCapability, oldKey, signedAt),
      trust,
      '2026-10-06T00:01:00Z',
    );
    expect(authority.receipt.trustBundleRoot).toBe(trust.bundleRoot);
    expect(authority.receipt.trustEntryId).toBe(oldEntry.entryId);
    expect(authority.receipt.signedAt).toBe('2026-10-04T12:00:00.000Z');
    expect(authority.receipt.receiptRoot).toMatch(/^[0-9a-f]{64}$/);
    expect(requireVerifiedEconomicAuditCapsuleIssuerAuthority(authority)).toBe(authority);
  });

  it('accepts the rotated audit issuer key after its trust window begins', () => {
    const signedAt = '2026-10-06T12:00:00Z';
    const trust = bundle('2026-10-07T00:00:00Z');
    const authority = verifyEconomicAuditCapsuleAttestationWithTrustBundle(
      capsule,
      newCapability,
      attestationFor(newCapability, newKey, signedAt),
      trust,
      '2026-10-07T00:01:00Z',
    );
    expect(authority.receipt.trustEntryId).toBe(newEntry.entryId);
    expect(authority.receipt.signerKeyId).toBe(newCapability.signerKeyId);
  });

  it('rejects an old issuer signature at and after the trust window expires', () => {
    const trust = bundle('2026-10-06T00:00:00Z');
    expect(() => verifyEconomicAuditCapsuleAttestationWithTrustBundle(
      capsule,
      oldCapability,
      attestationFor(oldCapability, oldKey, '2026-10-05T00:00:00Z'),
      trust,
      '2026-10-06T00:01:00Z',
    )).toThrow(/no authorized issuer trust entry/);
  });

  it('treats audit issuer revocation prospectively: pre-cutoff signatures survive, post-cutoff signatures fail', () => {
    const revokedEntry = entry({
      entryId: 'issuer-trust:revoked',
      capability: oldCapability,
      publicKeyPem: oldPublicKeyPem,
      validFrom: '2026-10-01T00:00:00Z',
      validUntil: '2027-01-01T00:00:00Z',
      revokedAt: '2026-10-05T00:00:00Z',
    });
    const trust = bundle('2026-10-06T00:00:00Z', [revokedEntry], 'audit-issuer-trust:revoked');
    expect(() => verifyEconomicAuditCapsuleAttestationWithTrustBundle(
      capsule,
      oldCapability,
      attestationFor(oldCapability, oldKey, '2026-10-04T23:59:59Z'),
      trust,
      '2026-10-06T00:01:00Z',
    )).not.toThrow();
    expect(() => verifyEconomicAuditCapsuleAttestationWithTrustBundle(
      capsule,
      oldCapability,
      attestationFor(oldCapability, oldKey, '2026-10-05T00:00:00Z'),
      trust,
      '2026-10-06T00:01:00Z',
    )).toThrow(/no authorized issuer trust entry/);
  });

  it('rejects stale audit issuer trust policy snapshots predating the attestation signature', () => {
    const signedAt = '2026-10-04T12:00:00Z';
    const stale = bundle('2026-10-03T00:00:00Z');
    expect(() => verifyEconomicAuditCapsuleAttestationWithTrustBundle(
      capsule,
      oldCapability,
      attestationFor(oldCapability, oldKey, signedAt),
      stale,
      '2026-10-06T00:01:00Z',
    )).toThrow(/policy must be observed through attestation signedAt/);
  });

  it('rejects audit issuer trust-bundle tampering behind an unchanged root', () => {
    const canonical = bundle('2026-10-06T00:00:00Z');
    const forged = {
      ...canonical,
      entries: canonical.entries.map(item => item.entryId === oldEntry.entryId
        ? { ...item, validUntil: '2026-10-10T00:00:00Z' }
        : item),
    };
    expect(() => verifyEconomicAuditCapsuleAttestationWithTrustBundle(
      capsule,
      oldCapability,
      attestationFor(oldCapability, oldKey, '2026-10-04T12:00:00Z'),
      forged,
      '2026-10-06T00:01:00Z',
    )).toThrow(/does not match canonical reconstruction/);
  });

  it('rejects overlapping authority windows for the same audit issuer capability identity', () => {
    const overlap = entry({
      entryId: 'issuer-trust:old-overlap',
      capability: oldCapability,
      publicKeyPem: oldPublicKeyPem,
      validFrom: '2026-10-04T00:00:00Z',
      validUntil: '2026-10-06T00:00:00Z',
    });
    expect(() => bundle('2026-10-06T00:00:00Z', [oldEntry, overlap], 'audit-issuer-trust:overlap'))
      .toThrow(/overlapping authority windows/);
  });

  it('rejects structural clones of verified audit issuer authority', () => {
    const trust = bundle('2026-10-06T00:00:00Z');
    const authority = verifyEconomicAuditCapsuleAttestationWithTrustBundle(
      capsule,
      oldCapability,
      attestationFor(oldCapability, oldKey, '2026-10-04T12:00:00Z'),
      trust,
      '2026-10-06T00:01:00Z',
    );
    expect(() => requireVerifiedEconomicAuditCapsuleIssuerAuthority({ ...authority }))
      .toThrow(/must be produced by trust-bundle verification/);
  });
});
