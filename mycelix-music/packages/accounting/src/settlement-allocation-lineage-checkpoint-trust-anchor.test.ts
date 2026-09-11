import { generateKeyPairSync, sign as signEd25519 } from 'node:crypto';
import { describe, expect, it } from 'vitest';
import {
  createSettlementAllocationLineageCheckpointSignerTrustBundle,
} from './settlement-allocation-lineage-checkpoint-trust.js';
import {
  assertSettlementAllocationLineageTrustBundleAttestationSuccessor,
  attachSettlementAllocationLineageTrustBundleDetachedSignature,
  createSettlementAllocationLineageTrustBundleAttestationRequest,
  requireAnchoredSettlementAllocationLineageCheckpointAuthority,
  settlementAllocationLineageTrustBundleAttestationPayloadBase64,
  verifySettlementAllocationLineageTrustBundleAttestation,
  type AnchoredSettlementAllocationLineageCheckpointAuthority,
  type SettlementAllocationLineageTrustAnchorPolicy,
} from './settlement-allocation-lineage-checkpoint-trust-anchor.js';

const rootKeyPair = generateKeyPairSync('ed25519');
const wrongRootKeyPair = generateKeyPairSync('ed25519');
const checkpointKeyPair = generateKeyPairSync('ed25519');
const rootPublicKeyPem = rootKeyPair.publicKey.export({ format: 'pem', type: 'spki' }).toString();
const checkpointPublicKeyPem = checkpointKeyPair.publicKey.export({ format: 'pem', type: 'spki' }).toString();

function trustBundle(bundleId: string, policyAsOf: string, signerKeyId: string) {
  return createSettlementAllocationLineageCheckpointSignerTrustBundle({
    bundleId,
    policyAsOf,
    entries: [{
      entryId: `${bundleId}:entry`,
      sourceRef: 'postgres:creator-accounting:allocation-lineage',
      sourceInstanceId: 'postgres:prod:1',
      capabilityId: 'checkpoint-capability:prod',
      signerKeyId,
      publicKeyPem: checkpointPublicKeyPem,
      validFrom: '2026-09-01T00:00:00Z',
      validUntil: '2027-01-01T00:00:00Z',
    }],
  });
}

const bundle1 = trustBundle('trust-bundle:1', '2026-10-01T00:00:00Z', 'checkpoint-key:1');
const bundle2 = trustBundle('trust-bundle:2', '2026-10-02T00:00:00Z', 'checkpoint-key:2');
const bundle3 = trustBundle('trust-bundle:3', '2026-10-03T00:00:00Z', 'checkpoint-key:3');

function anchor(minimumPolicySequence = '1'): SettlementAllocationLineageTrustAnchorPolicy {
  return {
    anchorId: 'trust-anchor:root:1',
    signerKeyId: 'trust-anchor-key:1',
    publicKeyPem: rootPublicKeyPem,
    validFrom: '2026-09-01T00:00:00Z',
    validUntil: '2027-01-01T00:00:00Z',
    minimumPolicySequence,
  };
}

function attestationFor(
  bundle: ReturnType<typeof trustBundle>,
  sequence: string,
  options: {
    predecessorBundleRoot?: string;
    issuedAt?: string;
    expiresAt?: string;
    signedAt?: string;
    requestId?: string;
    privateKey?: typeof rootKeyPair.privateKey;
  } = {},
) {
  const issuedAt = options.issuedAt ?? bundle.policyAsOf;
  const expiresAt = options.expiresAt ?? '2026-12-31T00:00:00Z';
  const signedAt = options.signedAt ?? new Date(Date.parse(issuedAt) + 1_000).toISOString();
  const request = createSettlementAllocationLineageTrustBundleAttestationRequest(bundle, {
    requestId: options.requestId ?? `attest:${bundle.bundleId}`,
    anchorId: 'trust-anchor:root:1',
    signerKeyId: 'trust-anchor-key:1',
    policySequence: sequence,
    ...(options.predecessorBundleRoot === undefined ? {} : { predecessorBundleRoot: options.predecessorBundleRoot }),
    issuedAt,
    expiresAt,
    nonce: `nonce:${bundle.bundleId}:${sequence}`,
  });
  const payload = Buffer.from(
    settlementAllocationLineageTrustBundleAttestationPayloadBase64(request, bundle, signedAt),
    'base64',
  );
  const signatureBase64 = signEd25519(null, payload, options.privateKey ?? rootKeyPair.privateKey).toString('base64');
  return attachSettlementAllocationLineageTrustBundleDetachedSignature(bundle, request, {
    signerKeyId: request.signerKeyId,
    signedAt,
    signatureBase64,
  });
}

describe('allocation-lineage trust anchor', () => {
  it('authenticates the initial trust bundle and binds the exact policy sequence', () => {
    const attestation = attestationFor(bundle1, '1');
    const verified = verifySettlementAllocationLineageTrustBundleAttestation(bundle1, attestation, anchor());
    expect(verified.bundle.bundleRoot).toBe(bundle1.bundleRoot);
    expect(verified.policySequence).toBe('1');
    expect(verified.anchorId).toBe('trust-anchor:root:1');
  });

  it('rejects a cryptographically valid older trust bundle below the pinned rollback floor', () => {
    const attestation = attestationFor(bundle1, '1');
    expect(() => verifySettlementAllocationLineageTrustBundleAttestation(bundle1, attestation, anchor('2')))
      .toThrow(/below anti-rollback floor/);
  });

  it('requires exact predecessor continuity and one-step policy advancement', () => {
    const first = verifySettlementAllocationLineageTrustBundleAttestation(
      bundle1,
      attestationFor(bundle1, '1'),
      anchor(),
    );
    const second = verifySettlementAllocationLineageTrustBundleAttestation(
      bundle2,
      attestationFor(bundle2, '2', { predecessorBundleRoot: bundle1.bundleRoot }),
      anchor(),
    );
    expect(() => assertSettlementAllocationLineageTrustBundleAttestationSuccessor(first, second)).not.toThrow();

    const skipped = verifySettlementAllocationLineageTrustBundleAttestation(
      bundle3,
      attestationFor(bundle3, '3', { predecessorBundleRoot: bundle2.bundleRoot }),
      anchor(),
    );
    expect(() => assertSettlementAllocationLineageTrustBundleAttestationSuccessor(first, skipped))
      .toThrow(/advance policy sequence exactly once/);
  });

  it('rejects a successor that names the wrong predecessor bundle root', () => {
    const first = verifySettlementAllocationLineageTrustBundleAttestation(
      bundle1,
      attestationFor(bundle1, '1'),
      anchor(),
    );
    const second = verifySettlementAllocationLineageTrustBundleAttestation(
      bundle2,
      attestationFor(bundle2, '2', { predecessorBundleRoot: 'f'.repeat(64) }),
      anchor(),
    );
    expect(() => assertSettlementAllocationLineageTrustBundleAttestationSuccessor(first, second))
      .toThrow(/bind predecessor bundle root/);
  });

  it('rejects root signatures from a key not pinned by the trust anchor', () => {
    const attestation = attestationFor(bundle1, '1', { privateKey: wrongRootKeyPair.privateKey });
    expect(() => verifySettlementAllocationLineageTrustBundleAttestation(bundle1, attestation, anchor()))
      .toThrow(/signature verification failed/);
  });

  it('uses half-open attestation request windows', () => {
    const issuedAt = '2026-10-01T00:00:00Z';
    const expiresAt = '2026-10-01T00:01:00Z';
    const request = createSettlementAllocationLineageTrustBundleAttestationRequest(bundle1, {
      requestId: 'attest:expiry',
      anchorId: 'trust-anchor:root:1',
      signerKeyId: 'trust-anchor-key:1',
      policySequence: '1',
      issuedAt,
      expiresAt,
      nonce: 'nonce:expiry',
    });
    expect(() => settlementAllocationLineageTrustBundleAttestationPayloadBase64(request, bundle1, expiresAt))
      .toThrow(/outside request window/);
  });

  it('rejects request or attestation content changed behind unchanged roots', () => {
    const attestation = attestationFor(bundle1, '1');
    expect(() => verifySettlementAllocationLineageTrustBundleAttestation(
      bundle1,
      { ...attestation, request: { ...attestation.request, policySequence: '2' } },
      anchor(),
    )).toThrow(/request root does not match canonical contents/);
    expect(() => verifySettlementAllocationLineageTrustBundleAttestation(
      bundle1,
      { ...attestation, attestationRoot: 'e'.repeat(64) },
      anchor(),
    )).toThrow(/attestation root does not match canonical contents/);
  });

  it('does not let structural clones participate in verified policy succession', () => {
    const first = verifySettlementAllocationLineageTrustBundleAttestation(
      bundle1,
      attestationFor(bundle1, '1'),
      anchor(),
    );
    const second = verifySettlementAllocationLineageTrustBundleAttestation(
      bundle2,
      attestationFor(bundle2, '2', { predecessorBundleRoot: bundle1.bundleRoot }),
      anchor(),
    );
    expect(() => assertSettlementAllocationLineageTrustBundleAttestationSuccessor({ ...first }, second))
      .toThrow(/requires verified attestation authorities/);
  });

  it('rejects fabricated anchored checkpoint authority objects', () => {
    const fabricated = {} as AnchoredSettlementAllocationLineageCheckpointAuthority;
    expect(() => requireAnchoredSettlementAllocationLineageCheckpointAuthority(fabricated))
      .toThrow(/requires verified anchored trust authority/);
  });

  it('requires non-initial policies to carry predecessor evidence', () => {
    expect(() => createSettlementAllocationLineageTrustBundleAttestationRequest(bundle2, {
      requestId: 'attest:no-predecessor',
      anchorId: 'trust-anchor:root:1',
      signerKeyId: 'trust-anchor-key:1',
      policySequence: '2',
      issuedAt: bundle2.policyAsOf,
      expiresAt: '2026-12-31T00:00:00Z',
      nonce: 'nonce:no-predecessor',
    })).toThrow(/requires predecessor bundle root/);
  });
});
