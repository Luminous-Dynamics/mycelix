# SUP-CIV-000D1D0 — `mycelix-crypto` Protected-Envelope Profile Registry v1

Status: semantic/profile boundary only. No production crypto code is enabled or changed by this tranche.

Source audit root: `main@a85369699099d4c7524e502e531735eed4ab36f4`.

Tracking issue: #2591.

## Purpose

Prevent existing Mycelix encryption constructions from being silently treated as interchangeable merely because they share algorithms such as ML-KEM-768, HKDF-SHA256 or AES-GCM.

The audited crate already contains several materially different protocol constructions. The new protected-storage evidence line defines another. They need explicit profile identity and fail-closed migration rules before runtime integration.

## Audited existing profiles

### `MycelixHybridKemV1Experimental`

Current `hybrid_kem.rs` defines:

```text
X25519 ephemeral DH
+ ML-KEM-768
+ HKDF-SHA256
  salt = ephemeral X25519 public || ML-KEM ciphertext
  ikm  = X25519 shared || ML-KEM shared
  info = "mycelix-hybrid-kem-v1"
+ XChaCha20-Poly1305
+ random 24-byte nonce
```

It owns a distinct `HybridCiphertext` and is explicitly marked experimental/pending crypto audit.

### `PulseV2HybridPqcExperimental`

Current `pulse_v2.rs` defines:

```text
X25519
+ ML-KEM-768
+ Pulse-specific domain-separated HKDF combiner
+ AES-256-GCM
+ caller-owned canonical AAD
```

The useful API theorem is that the primitive layer consumes caller-owned canonical AAD rather than inventing application semantics. The Pulse cryptographic profile itself remains Pulse-specific.

### `ProtectedEnvelopeCmsMlKemV1`

The Civic protected-storage profile is separately defined by the D1/C1/C2 evidence line:

```text
D1A lifecycle
D1B canonical PayloadAadV1 / RecipientWrapContextV1 / ProtectedEnvelopeV1
C1 AES-256-GCM payload protection
C2 ML-KEM-768 + RFC 9629/9936 recipient wrapping
```

It must not inherit wire, KDF or fallback behavior from either existing experimental profile.

## Protocol identity is not algorithm identity

The closed theorem is:

```text
AlgorithmId != ProtocolProfile
EnvelopeFormatVersion != CryptoProfile
SameMLKEMAlgorithm != SameKDFTranscript
SameAESGCMAlgorithm != SameAADSemantics
DecryptableByOneProfile != ValidUnderAnotherProfile
```

The current generic `EncryptedEnvelope` / `SealedEnvelope` types can describe algorithms/layout, but those fields alone do not establish which transcript/KDF/AAD semantics produced a ciphertext.

## No implicit fallback

Failure to use the protected-storage profile never authorizes another construction:

```text
ProtectedEnvelopeCmsMlKemV1 unavailable
!= PulseV2HybridPqcExperimental
!= MycelixHybridKemV1Experimental
!= classical-only
!= plaintext
```

Unknown or legacy profile evidence must not be guessed from lengths or algorithm identifiers.

```text
LegacyEncryptedEnvelopeWithoutProfile != ProtectedEnvelopeCmsMlKemV1
SuccessfulLegacyDecrypt != NewProfileQualified
```

## Runtime module boundary

The candidate Rust split is intentionally narrow:

```text
protected_envelope::profile
protected_envelope::codec
protected_envelope::payload
protected_envelope::recipient
protected_envelope::interop
```

Responsibilities:

- `profile`: closed protocol/profile IDs and admission rules;
- `codec`: exact D1B canonical bytes, no cryptography;
- `payload`: C1 payload protection only;
- `recipient`: C2 recipient-wrapping profile only;
- `interop`: qualification/reference adapters, never an authority source.

Do not force existing `HybridCiphertext`, Pulse `SealedPayload`, or generic legacy `EncryptedEnvelope` into the new wire object merely because fields appear similar.

## Dependency policy

The outer workspace currently pins `aead = 0.6.0-rc.2` and documents why global RustCrypto ML-KEM is disabled, while `mycelix-crypto` carries `ml-kem = 0.3.0-rc.2` only behind off-by-default `hybrid-rc`.

Therefore D1D0 freezes:

```text
enable_ml_kem_globally = false
perturb_workspace_aead_resolution = false
```

Implementation sequence:

```text
1. crypto-free codec/profile types
2. dependency-compatible AES/HKDF/AES-KW conformance
3. isolated/feature-gated ML-KEM recipient adapter
4. dependency convergence only after independent qualification
```

## Upstream evidence remains independent

Every protected-profile dependency remains `ReferencePendingQualification` in this tranche.

```text
DependencyReferenced != DependencyQualified
ProfileRegistryPass != CryptoProfileQualified
```

The registry may qualify while the crypto subjects remain queued because its theorem is only ownership/profile separation.

## Required non-equivalences

```text
AlgorithmId != ProtocolProfile
EnvelopeFormatVersion != CryptoProfile
SameMLKEMAlgorithm != SameKDFTranscript
SameAESGCMAlgorithm != SameAADSemantics
HybridKemV1 != PulseV2HybridPqc
PulseV2HybridPqc != ProtectedEnvelopeCmsMlKemV1
ExistingEnvelopeDecrypts != CivicStorageAdmitted
LegacyEncryptedEnvelopeWithoutProfile != ProtectedEnvelopeCmsMlKemV1
SuccessfulLegacyDecrypt != NewProfileQualified
InteropVectorPass != RuntimeAuthorization
ProfileAvailable != ProfileAuthorizedForDatum
CryptoProfileQualified != StorageProfileQualified
CryptoProfileQualified != ProtectedReadAuthorized
```

## Claim ceiling

A PASS establishes only this exact source audit, closed profile registry, ownership boundaries, no-fallback/migration rules, dependency policy and runtime-promotion block. It does not qualify any upstream crypto subject, modify existing ciphertext, enable ML-KEM globally, establish runtime cryptographic correctness, authorize a protected read, qualify a storage topology, establish legal compliance or deployment readiness.
