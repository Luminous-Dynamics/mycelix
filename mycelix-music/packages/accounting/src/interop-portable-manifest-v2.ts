import {
  ACCOUNTING_WIRE_FORMAT,
  ACCOUNTING_WIRE_VERSION,
  accountingWireDigestFromText,
  deserializeAccountingWire,
} from './accounting-wire.js';
import { buildMerkleCommitmentV2 } from './merkle-v2.js';
import type { Digest } from './merkle.js';
import { PORTABLE_REPORTING_EVIDENCE_PROTOCOL } from './interop-portable-verification.js';

export const PORTABLE_REPORTING_MANIFEST_V2_PROTOCOL =
  'mycelix-reporting-evidence-manifest-v2' as const;

const SHA256_HEX = /^[0-9a-f]{64}$/;

export interface PortableReportingEvidenceTransportV1 {
  readonly artifactKind: typeof PORTABLE_REPORTING_EVIDENCE_PROTOCOL;
  readonly protocolVersion: 1;
  readonly auditAttestation: unknown;
  readonly auditCapability: unknown;
  readonly auditIssuerTrustAttestation: unknown;
  readonly auditIssuerTrustBundle: unknown;
  readonly bundleRoot: Digest;
  readonly capsule: unknown;
  readonly package: unknown;
  readonly reportingAttestation: unknown;
  readonly reportingCapability: unknown;
  readonly reportingIssuerTrustAttestation: unknown;
  readonly reportingIssuerTrustBundle: unknown;
}

export interface PortableReportingEvidenceManifestV2 {
  readonly protocol: typeof PORTABLE_REPORTING_MANIFEST_V2_PROTOCOL;
  readonly protocolVersion: 2;
  readonly wireFormat: typeof ACCOUNTING_WIRE_FORMAT;
  readonly wireVersion: typeof ACCOUNTING_WIRE_VERSION;
  readonly wireDigest: Digest;
  readonly legacyPortableProtocol: typeof PORTABLE_REPORTING_EVIDENCE_PROTOCOL;
  readonly legacyBundleRoot: Digest;
  readonly legacyBundleRootScheme: 'accounting_merkle_v1_legacy';
  readonly portablePayloadRootV2: Digest;
  readonly portablePayloadRootScheme: 'accounting_merkle_v2_wire';
  readonly nestedAuthorityStatus: 'legacy_v1_domain_verification_required';
  readonly manifestRootV2: Digest;
}

function digest(label: string, value: string): Digest {
  if (!SHA256_HEX.test(value)) throw new Error(`${label} must be a lowercase SHA-256 digest`);
  return value;
}

function exactPortableTransport(value: unknown): Readonly<PortableReportingEvidenceTransportV1> {
  if (value === null || typeof value !== 'object' || Array.isArray(value)) {
    throw new Error('portable reporting manifest requires an object payload');
  }
  const record = value as Record<string, unknown>;
  const expected = [
    'artifactKind',
    'auditAttestation',
    'auditCapability',
    'auditIssuerTrustAttestation',
    'auditIssuerTrustBundle',
    'bundleRoot',
    'capsule',
    'package',
    'protocolVersion',
    'reportingAttestation',
    'reportingCapability',
    'reportingIssuerTrustAttestation',
    'reportingIssuerTrustBundle',
  ] as const;
  const keys = Object.keys(record);
  if (keys.length !== expected.length || keys.some((key, index) => key !== expected[index])) {
    throw new Error('portable reporting manifest payload has unexpected top-level fields');
  }
  if (record.artifactKind !== PORTABLE_REPORTING_EVIDENCE_PROTOCOL || record.protocolVersion !== 1) {
    throw new Error('portable reporting manifest requires portable reporting evidence v1 payload');
  }
  if (typeof record.bundleRoot !== 'string') {
    throw new Error('portable reporting manifest requires a legacy bundle root');
  }
  digest('portable reporting legacy bundleRoot', record.bundleRoot);
  return value as PortableReportingEvidenceTransportV1;
}

function manifestRootV2(
  manifest: Omit<PortableReportingEvidenceManifestV2, 'manifestRootV2'>,
): Digest {
  return buildMerkleCommitmentV2([{
    recordType: 'portable_reporting_evidence_manifest_v2',
    ...manifest,
  }]).root;
}

export function createPortableReportingEvidenceManifestV2(
  wireText: string,
): Readonly<PortableReportingEvidenceManifestV2> {
  const wireDigest = accountingWireDigestFromText(wireText);
  const payload = exactPortableTransport(deserializeAccountingWire(wireText));
  const portablePayloadRootV2 = buildMerkleCommitmentV2([payload]).root;
  const committed = Object.freeze({
    protocol: PORTABLE_REPORTING_MANIFEST_V2_PROTOCOL,
    protocolVersion: 2 as const,
    wireFormat: ACCOUNTING_WIRE_FORMAT,
    wireVersion: ACCOUNTING_WIRE_VERSION,
    wireDigest,
    legacyPortableProtocol: PORTABLE_REPORTING_EVIDENCE_PROTOCOL,
    legacyBundleRoot: payload.bundleRoot,
    legacyBundleRootScheme: 'accounting_merkle_v1_legacy' as const,
    portablePayloadRootV2,
    portablePayloadRootScheme: 'accounting_merkle_v2_wire' as const,
    nestedAuthorityStatus: 'legacy_v1_domain_verification_required' as const,
  });
  return Object.freeze({ ...committed, manifestRootV2: manifestRootV2(committed) });
}

export function assertPortableReportingEvidenceManifestV2(
  wireText: string,
  manifest: PortableReportingEvidenceManifestV2,
): void {
  const expected = createPortableReportingEvidenceManifestV2(wireText);
  for (const [key, value] of Object.entries(expected)) {
    if ((manifest as unknown as Record<string, unknown>)[key] !== value) {
      throw new Error(`portable reporting manifest mismatch: ${key}`);
    }
  }
  if (Object.keys(manifest).length !== Object.keys(expected).length) {
    throw new Error('portable reporting manifest has unexpected fields');
  }
}
