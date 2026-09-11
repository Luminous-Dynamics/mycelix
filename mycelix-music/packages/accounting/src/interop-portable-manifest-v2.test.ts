import { describe, expect, it } from 'vitest';
import { serializeAccountingWire } from './accounting-wire.js';
import {
  assertPortableReportingEvidenceManifestV2,
  createPortableReportingEvidenceManifestV2,
} from './interop-portable-manifest-v2.js';

const syntheticPortableV1 = Object.freeze({
  artifactKind: 'mycelix-reporting-evidence-portable-v1' as const,
  auditAttestation: Object.freeze({ id: 'audit-attestation' }),
  auditCapability: Object.freeze({ id: 'audit-capability' }),
  auditIssuerTrustAttestation: Object.freeze({ id: 'audit-root-attestation' }),
  auditIssuerTrustBundle: Object.freeze({ id: 'audit-trust' }),
  bundleRoot: 'a'.repeat(64),
  capsule: Object.freeze({ amountMinor: 12345678901234567890n, id: 'capsule' }),
  package: Object.freeze({ id: 'package' }),
  protocolVersion: 1 as const,
  reportingAttestation: Object.freeze({ id: 'reporting-attestation' }),
  reportingCapability: Object.freeze({ id: 'reporting-capability' }),
  reportingIssuerTrustAttestation: Object.freeze({ id: 'reporting-root-attestation' }),
  reportingIssuerTrustBundle: Object.freeze({ id: 'reporting-trust' }),
});

const wireText = serializeAccountingWire(syntheticPortableV1);

describe('portable reporting dual-commitment manifest v2', () => {
  it('binds fixed cross-language wire and Merkle-v2 roots without rewriting the v1 bundle root', () => {
    const manifest = createPortableReportingEvidenceManifestV2(wireText);
    expect(manifest.legacyBundleRoot).toBe('a'.repeat(64));
    expect(manifest.legacyBundleRootScheme).toBe('accounting_merkle_v1_legacy');
    expect(manifest.wireDigest).toBe('a4623f5c498ad7b5e753f0cb086f162a25a26b80c8558b4c7572c8cea5bf9f98');
    expect(manifest.portablePayloadRootV2).toBe('c6a04ad0b236d62e925170e2564a974a577aa33d2884a6c128504b25a0a6d9a4');
    expect(manifest.manifestRootV2).toBe('e0c209f261a098dc764fe7fb40b4e8bd3f4cd3d39e61b3d5bbb4dd6f2f3aefb1');
    expect(manifest.nestedAuthorityStatus).toBe('legacy_v1_domain_verification_required');
    expect(() => assertPortableReportingEvidenceManifestV2(wireText, manifest)).not.toThrow();
  });

  it('changes the portable v2 commitment when payload contents change even if the legacy root string is retained', () => {
    const original = createPortableReportingEvidenceManifestV2(wireText);
    const changedWire = serializeAccountingWire({
      ...syntheticPortableV1,
      package: { id: 'package:changed' },
    });
    const changed = createPortableReportingEvidenceManifestV2(changedWire);
    expect(changed.legacyBundleRoot).toBe(original.legacyBundleRoot);
    expect(changed.wireDigest).not.toBe(original.wireDigest);
    expect(changed.portablePayloadRootV2).not.toBe(original.portablePayloadRootV2);
    expect(changed.manifestRootV2).not.toBe(original.manifestRootV2);
  });

  it('rejects manifest tampering and extra fields', () => {
    const manifest = createPortableReportingEvidenceManifestV2(wireText);
    expect(() => assertPortableReportingEvidenceManifestV2(wireText, {
      ...manifest,
      portablePayloadRootV2: '0'.repeat(64),
    })).toThrow(/portablePayloadRootV2/);

    expect(() => assertPortableReportingEvidenceManifestV2(wireText, {
      ...manifest,
      extra: 'field',
    } as typeof manifest)).toThrow(/unexpected fields/);
  });

  it('rejects a transport shape that is not portable reporting evidence v1', () => {
    const wrong = serializeAccountingWire({ ...syntheticPortableV1, artifactKind: 'other-protocol' });
    expect(() => createPortableReportingEvidenceManifestV2(wrong)).toThrow(/portable reporting evidence v1/);
  });
});
