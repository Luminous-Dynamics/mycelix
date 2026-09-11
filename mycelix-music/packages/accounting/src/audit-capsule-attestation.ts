import { createPublicKey, verify as verifyEd25519 } from 'node:crypto';
import {
  assertEconomicAuditCapsule,
  economicAuditCapsuleDigest,
  type EconomicAuditCapsuleV2,
} from './audit.js';
import { buildMerkleCommitment, type Digest } from './merkle.js';

export const ECONOMIC_AUDIT_CAPSULE_ATTESTATION_SCOPE = 'accounting.economic-audit-capsule.attest' as const;
const AUDIT_CAPSULE_ATTESTATION_SIGNATURE_DOMAIN = 'mycelix-accounting-economic-audit-capsule-attestation-signature-v1\0';
const SHA256_HEX = /^[0-9a-f]{64}$/;

export interface EconomicAuditCapsuleAttestationCapability {
  readonly capabilityId: string;
  readonly scope: typeof ECONOMIC_AUDIT_CAPSULE_ATTESTATION_SCOPE;
  readonly issuerId: string;
  readonly signerKeyId: string;
  readonly compilerId: string;
  readonly compilerBuildDigest: Digest;
  readonly validFrom: string;
  readonly validUntil: string;
  readonly capabilityRoot: Digest;
}

export interface CreateEconomicAuditCapsuleAttestationCapabilityInput {
  readonly capabilityId: string;
  readonly issuerId: string;
  readonly signerKeyId: string;
  readonly compilerId: string;
  readonly compilerBuildDigest: Digest;
  readonly validFrom: string;
  readonly validUntil: string;
}

export interface EconomicAuditCapsuleAttestationRequest {
  readonly requestId: string;
  readonly capabilityId: string;
  readonly capabilityRoot: Digest;
  readonly scope: typeof ECONOMIC_AUDIT_CAPSULE_ATTESTATION_SCOPE;
  readonly issuerId: string;
  readonly signerKeyId: string;
  readonly capsuleDigest: Digest;
  readonly protocolVersion: 2;
  readonly statementSnapshotRoot: Digest;
  readonly compilerId: string;
  readonly compilerVersion: string;
  readonly compilerBuildDigest: Digest;
  readonly issuedAt: string;
  readonly expiresAt: string;
  readonly nonce: string;
  readonly requestRoot: Digest;
}

export interface CreateEconomicAuditCapsuleAttestationRequestInput {
  readonly requestId: string;
  readonly issuedAt: string;
  readonly expiresAt: string;
  readonly nonce: string;
}

export interface EconomicAuditCapsuleDetachedSignature {
  readonly requestRoot: Digest;
  readonly signerKeyId: string;
  readonly signedAt: string;
  readonly signatureBase64: string;
}

export interface EconomicAuditCapsuleAttestation {
  readonly request: EconomicAuditCapsuleAttestationRequest;
  readonly signerKeyId: string;
  readonly signedAt: string;
  readonly signatureBase64: string;
  readonly attestationRoot: Digest;
}

export interface EconomicAuditCapsuleIssuerTrustPolicy {
  readonly issuerId: string;
  readonly signerKeyId: string;
  readonly publicKeyPem: string;
  readonly capabilityRoot: Digest;
  readonly validFrom: string;
  readonly validUntil: string;
}

export interface EconomicAuditCapsuleAttestationVerificationReceipt {
  readonly capsuleDigest: Digest;
  readonly statementSnapshotRoot: Digest;
  readonly attestationRoot: Digest;
  readonly capabilityRoot: Digest;
  readonly issuerId: string;
  readonly signerKeyId: string;
  readonly signedAt: string;
  readonly verifiedAt: string;
  readonly receiptRoot: Digest;
}

export interface VerifiedEconomicAuditCapsuleAttestation {
  readonly capsule: EconomicAuditCapsuleV2;
  readonly attestation: EconomicAuditCapsuleAttestation;
  readonly receipt: EconomicAuditCapsuleAttestationVerificationReceipt;
}

const VERIFIED_AUDIT_CAPSULE_ATTESTATIONS = new WeakSet<object>();

function required(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

function timestamp(label: string, value: string): string {
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error(`${label} must be a valid timestamp`);
  return new Date(parsed).toISOString();
}

function digest(label: string, value: string): Digest {
  if (!SHA256_HEX.test(value)) throw new Error(`${label} must be a lowercase SHA-256 digest`);
  return value;
}

function canonicalPublicKeyPem(value: string): string {
  const key = createPublicKey(value);
  if (key.asymmetricKeyType !== 'ed25519') throw new Error('audit capsule issuer public key must be Ed25519');
  return key.export({ format: 'pem', type: 'spki' }).toString();
}

function canonicalSignature(value: string): Buffer {
  if (!value.trim()) throw new Error('audit capsule attestation signature must be non-empty');
  const decoded = Buffer.from(value, 'base64');
  if (decoded.length !== 64 || decoded.toString('base64') !== value) {
    throw new Error('audit capsule attestation signature must be canonical Ed25519 base64');
  }
  return decoded;
}

function capabilityCommitment(
  input: Omit<EconomicAuditCapsuleAttestationCapability, 'capabilityRoot'>,
): Digest {
  return buildMerkleCommitment([{
    recordType: 'economic_audit_capsule_attestation_capability_v1',
    ...input,
  }]).root;
}

function requestCommitment(
  input: Omit<EconomicAuditCapsuleAttestationRequest, 'requestRoot'>,
): Digest {
  return buildMerkleCommitment([{
    recordType: 'economic_audit_capsule_attestation_request_v1',
    ...input,
  }]).root;
}

function attestationCommitment(
  input: Omit<EconomicAuditCapsuleAttestation, 'attestationRoot'>,
): Digest {
  return buildMerkleCommitment([{
    recordType: 'economic_audit_capsule_attestation_v1',
    requestRoot: input.request.requestRoot,
    signerKeyId: input.signerKeyId,
    signedAt: input.signedAt,
    signatureBase64: input.signatureBase64,
  }]).root;
}

function signingMessage(requestRoot: Digest, signedAt: string): Buffer {
  return Buffer.from(`${AUDIT_CAPSULE_ATTESTATION_SIGNATURE_DOMAIN}${requestRoot}\0${signedAt}`, 'utf8');
}

export function createEconomicAuditCapsuleAttestationCapability(
  input: CreateEconomicAuditCapsuleAttestationCapabilityInput,
): Readonly<EconomicAuditCapsuleAttestationCapability> {
  const committed = Object.freeze({
    capabilityId: required('audit capsule attestation capabilityId', input.capabilityId),
    scope: ECONOMIC_AUDIT_CAPSULE_ATTESTATION_SCOPE,
    issuerId: required('audit capsule attestation issuerId', input.issuerId),
    signerKeyId: required('audit capsule attestation signerKeyId', input.signerKeyId),
    compilerId: required('audit capsule attestation compilerId', input.compilerId),
    compilerBuildDigest: digest('audit capsule attestation compilerBuildDigest', input.compilerBuildDigest),
    validFrom: timestamp('audit capsule attestation validFrom', input.validFrom),
    validUntil: timestamp('audit capsule attestation validUntil', input.validUntil),
  });
  if (Date.parse(committed.validUntil) <= Date.parse(committed.validFrom)) {
    throw new Error('audit capsule attestation capability validity window must be strictly increasing');
  }
  return Object.freeze({ ...committed, capabilityRoot: capabilityCommitment(committed) });
}

export function assertEconomicAuditCapsuleAttestationCapability(
  capability: EconomicAuditCapsuleAttestationCapability,
): void {
  const reconstructed = createEconomicAuditCapsuleAttestationCapability(capability);
  if (capability.scope !== ECONOMIC_AUDIT_CAPSULE_ATTESTATION_SCOPE) {
    throw new Error('audit capsule attestation capability has unsupported scope');
  }
  if (capability.capabilityRoot !== reconstructed.capabilityRoot) {
    throw new Error('audit capsule attestation capability root does not match canonical contents');
  }
}

function canonicalRequest(
  request: EconomicAuditCapsuleAttestationRequest,
  capsule?: EconomicAuditCapsuleV2,
  capability?: EconomicAuditCapsuleAttestationCapability,
): Readonly<EconomicAuditCapsuleAttestationRequest> {
  const committed = Object.freeze({
    requestId: required('audit capsule attestation requestId', request.requestId),
    capabilityId: required('audit capsule attestation request capabilityId', request.capabilityId),
    capabilityRoot: digest('audit capsule attestation request capabilityRoot', request.capabilityRoot),
    scope: request.scope,
    issuerId: required('audit capsule attestation request issuerId', request.issuerId),
    signerKeyId: required('audit capsule attestation request signerKeyId', request.signerKeyId),
    capsuleDigest: digest('audit capsule attestation request capsuleDigest', request.capsuleDigest),
    protocolVersion: request.protocolVersion,
    statementSnapshotRoot: digest('audit capsule attestation request statementSnapshotRoot', request.statementSnapshotRoot),
    compilerId: required('audit capsule attestation request compilerId', request.compilerId),
    compilerVersion: required('audit capsule attestation request compilerVersion', request.compilerVersion),
    compilerBuildDigest: digest('audit capsule attestation request compilerBuildDigest', request.compilerBuildDigest),
    issuedAt: timestamp('audit capsule attestation issuedAt', request.issuedAt),
    expiresAt: timestamp('audit capsule attestation expiresAt', request.expiresAt),
    nonce: required('audit capsule attestation nonce', request.nonce),
  });
  if (committed.scope !== ECONOMIC_AUDIT_CAPSULE_ATTESTATION_SCOPE || committed.protocolVersion !== 2) {
    throw new Error('audit capsule attestation request has unsupported scope or protocol');
  }
  if (Date.parse(committed.expiresAt) <= Date.parse(committed.issuedAt)) {
    throw new Error('audit capsule attestation request expiry must be after issuance');
  }
  if (capsule !== undefined) {
    assertEconomicAuditCapsule(capsule);
    if (capsule.protocolVersion !== 2) throw new Error('audit capsule attestation supports protocol v2 capsules only');
    if (
      committed.capsuleDigest !== economicAuditCapsuleDigest(capsule)
      || committed.statementSnapshotRoot !== capsule.statementSnapshotRoot
      || committed.compilerId !== capsule.generatedBy.id
      || committed.compilerVersion !== capsule.generatedBy.version
      || committed.compilerBuildDigest !== capsule.generatedBy.buildDigest
    ) {
      throw new Error('audit capsule attestation request does not match exact capsule identity');
    }
  }
  if (capability !== undefined) {
    assertEconomicAuditCapsuleAttestationCapability(capability);
    if (
      committed.capabilityId !== capability.capabilityId
      || committed.capabilityRoot !== capability.capabilityRoot
      || committed.issuerId !== capability.issuerId
      || committed.signerKeyId !== capability.signerKeyId
      || committed.compilerId !== capability.compilerId
      || committed.compilerBuildDigest !== capability.compilerBuildDigest
    ) {
      throw new Error('audit capsule attestation request is outside exact capability authority');
    }
    if (
      Date.parse(committed.issuedAt) < Date.parse(capability.validFrom)
      || Date.parse(committed.expiresAt) > Date.parse(capability.validUntil)
    ) {
      throw new Error('audit capsule attestation request must remain inside capability validity window');
    }
  }
  const expectedRoot = requestCommitment(committed);
  if (request.requestRoot !== expectedRoot) {
    throw new Error('audit capsule attestation request root does not match canonical contents');
  }
  return Object.freeze({ ...committed, requestRoot: expectedRoot });
}

export function createEconomicAuditCapsuleAttestationRequest(
  capsule: EconomicAuditCapsuleV2,
  capability: EconomicAuditCapsuleAttestationCapability,
  input: CreateEconomicAuditCapsuleAttestationRequestInput,
): Readonly<EconomicAuditCapsuleAttestationRequest> {
  assertEconomicAuditCapsule(capsule);
  if (capsule.protocolVersion !== 2) throw new Error('audit capsule attestation supports protocol v2 capsules only');
  assertEconomicAuditCapsuleAttestationCapability(capability);
  if (capsule.generatedBy.id !== capability.compilerId || capsule.generatedBy.buildDigest !== capability.compilerBuildDigest) {
    throw new Error('audit capsule compiler identity is not authorized by attestation capability');
  }
  const committed = Object.freeze({
    requestId: required('audit capsule attestation requestId', input.requestId),
    capabilityId: capability.capabilityId,
    capabilityRoot: capability.capabilityRoot,
    scope: capability.scope,
    issuerId: capability.issuerId,
    signerKeyId: capability.signerKeyId,
    capsuleDigest: economicAuditCapsuleDigest(capsule),
    protocolVersion: 2 as const,
    statementSnapshotRoot: capsule.statementSnapshotRoot,
    compilerId: capsule.generatedBy.id,
    compilerVersion: capsule.generatedBy.version,
    compilerBuildDigest: capsule.generatedBy.buildDigest,
    issuedAt: timestamp('audit capsule attestation issuedAt', input.issuedAt),
    expiresAt: timestamp('audit capsule attestation expiresAt', input.expiresAt),
    nonce: required('audit capsule attestation nonce', input.nonce),
  });
  const request = Object.freeze({ ...committed, requestRoot: requestCommitment(committed) });
  return canonicalRequest(request, capsule, capability);
}

export function economicAuditCapsuleAttestationPayloadBase64(
  request: EconomicAuditCapsuleAttestationRequest,
  signedAtInput: string,
): string {
  const canonical = canonicalRequest(request);
  const signedAt = timestamp('audit capsule attestation signedAt', signedAtInput);
  if (Date.parse(signedAt) < Date.parse(canonical.issuedAt) || Date.parse(signedAt) >= Date.parse(canonical.expiresAt)) {
    throw new Error('audit capsule attestation signature time is outside request window');
  }
  return signingMessage(canonical.requestRoot, signedAt).toString('base64');
}

export function attachEconomicAuditCapsuleDetachedSignature(
  capsule: EconomicAuditCapsuleV2,
  capability: EconomicAuditCapsuleAttestationCapability,
  request: EconomicAuditCapsuleAttestationRequest,
  detached: EconomicAuditCapsuleDetachedSignature,
): Readonly<EconomicAuditCapsuleAttestation> {
  const canonical = canonicalRequest(request, capsule, capability);
  if (detached.requestRoot !== canonical.requestRoot) {
    throw new Error('audit capsule detached signature does not bind canonical request');
  }
  const signerKeyId = required('audit capsule detached signerKeyId', detached.signerKeyId);
  if (signerKeyId !== canonical.signerKeyId) {
    throw new Error('audit capsule detached signature signer does not match request');
  }
  const signedAt = timestamp('audit capsule detached signedAt', detached.signedAt);
  if (Date.parse(signedAt) < Date.parse(canonical.issuedAt) || Date.parse(signedAt) >= Date.parse(canonical.expiresAt)) {
    throw new Error('audit capsule attestation signature time is outside request window');
  }
  canonicalSignature(detached.signatureBase64);
  const committed = Object.freeze({
    request: canonical,
    signerKeyId,
    signedAt,
    signatureBase64: detached.signatureBase64,
  });
  return Object.freeze({ ...committed, attestationRoot: attestationCommitment(committed) });
}

export function verifyEconomicAuditCapsuleAttestation(
  capsule: EconomicAuditCapsuleV2,
  capability: EconomicAuditCapsuleAttestationCapability,
  attestation: EconomicAuditCapsuleAttestation,
  policy: EconomicAuditCapsuleIssuerTrustPolicy,
  verifiedAtInput: string,
): Readonly<VerifiedEconomicAuditCapsuleAttestation> {
  const canonical = canonicalRequest(attestation.request, capsule, capability);
  const policyIssuerId = required('audit capsule issuer trust issuerId', policy.issuerId);
  const policySignerKeyId = required('audit capsule issuer trust signerKeyId', policy.signerKeyId);
  const policyCapabilityRoot = digest('audit capsule issuer trust capabilityRoot', policy.capabilityRoot);
  const policyValidFrom = timestamp('audit capsule issuer trust validFrom', policy.validFrom);
  const policyValidUntil = timestamp('audit capsule issuer trust validUntil', policy.validUntil);
  if (Date.parse(policyValidUntil) <= Date.parse(policyValidFrom)) {
    throw new Error('audit capsule issuer trust validity window must be strictly increasing');
  }
  if (
    policyIssuerId !== canonical.issuerId
    || policySignerKeyId !== canonical.signerKeyId
    || policyCapabilityRoot !== canonical.capabilityRoot
  ) {
    throw new Error('audit capsule attestation is not authorized by pinned issuer trust policy');
  }
  const signedAt = timestamp('audit capsule attestation signedAt', attestation.signedAt);
  const verifiedAt = timestamp('audit capsule attestation verifiedAt', verifiedAtInput);
  if (Date.parse(verifiedAt) < Date.parse(signedAt)) {
    throw new Error('audit capsule attestation cannot be verified before it was signed');
  }
  if (
    Date.parse(signedAt) < Date.parse(policyValidFrom)
    || Date.parse(signedAt) >= Date.parse(policyValidUntil)
  ) {
    throw new Error('audit capsule attestation signature is outside issuer trust validity window');
  }
  if (attestation.signerKeyId !== policySignerKeyId) {
    throw new Error('audit capsule attestation signer key is not trusted');
  }
  const expectedRoot = attestationCommitment({
    request: canonical,
    signerKeyId: policySignerKeyId,
    signedAt,
    signatureBase64: attestation.signatureBase64,
  });
  if (attestation.attestationRoot !== expectedRoot) {
    throw new Error('audit capsule attestation root does not match canonical contents');
  }
  const publicKey = createPublicKey(canonicalPublicKeyPem(policy.publicKeyPem));
  const signature = canonicalSignature(attestation.signatureBase64);
  if (!verifyEd25519(null, signingMessage(canonical.requestRoot, signedAt), publicKey, signature)) {
    throw new Error('audit capsule attestation signature verification failed');
  }

  const receiptFields = Object.freeze({
    capsuleDigest: canonical.capsuleDigest,
    statementSnapshotRoot: canonical.statementSnapshotRoot,
    attestationRoot: expectedRoot,
    capabilityRoot: canonical.capabilityRoot,
    issuerId: policyIssuerId,
    signerKeyId: policySignerKeyId,
    signedAt,
    verifiedAt,
  });
  const receiptRoot = buildMerkleCommitment([{
    recordType: 'economic_audit_capsule_attestation_verification_receipt_v1',
    ...receiptFields,
  }]).root;
  const verified = Object.freeze({
    capsule,
    attestation: Object.freeze({
      request: canonical,
      signerKeyId: policySignerKeyId,
      signedAt,
      signatureBase64: attestation.signatureBase64,
      attestationRoot: expectedRoot,
    }),
    receipt: Object.freeze({ ...receiptFields, receiptRoot }),
  });
  VERIFIED_AUDIT_CAPSULE_ATTESTATIONS.add(verified);
  return verified;
}

export function requireVerifiedEconomicAuditCapsuleAttestation(
  authority: VerifiedEconomicAuditCapsuleAttestation,
): Readonly<VerifiedEconomicAuditCapsuleAttestation> {
  if (!VERIFIED_AUDIT_CAPSULE_ATTESTATIONS.has(authority)) {
    throw new Error('audit capsule attestation authority must be produced by cryptographic verification');
  }
  return authority;
}
