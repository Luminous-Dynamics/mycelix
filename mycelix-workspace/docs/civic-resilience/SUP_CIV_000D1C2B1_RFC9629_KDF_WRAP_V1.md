# SUP-CIV-000D1C2B1 — RFC 9629 OtherInfo / HKDF / AES-256-KW Core Vector v1

Status: deterministic recipient-wrap core vector only. No full CMS recipient artifact, no recipient authorization, no production key material.

Structural parent: SUP-CIV-000D1C2A1 / draft #2538 / exact head `9775d9217db44c1c1ba12edbeef9a5aeaff3478e`.

Canonical transcript dependency: SUP-CIV-000D1B / draft #2348 / exact head `1152251e1b8d09bf69cc2497164a0fc1d58da320`.

Tracking issue: #2539.

## Purpose

Freeze the exact standards-defined key-derivation and key-wrap core that sits between one deterministic ML-KEM shared-secret fixture and the encrypted content-encryption/data-encryption key carried by RFC 9629 `KEMRecipientInfo`.

This tranche deliberately stops before the outer recipient identifier and full `KEMRecipientInfo` DER so those semantics remain independently reviewable in C2B2.

## Standards mapping

For this ML-KEM-768 profile:

```text
IKM     = C2A1 deterministic ML-KEM-768 shared-secret fixture
salt    = zero-length string
info    = DER CMSORIforKEMOtherInfo
L       = 32 bytes
KDF     = HKDF-SHA256
wrap    = AES-256-WRAP / RFC 3394
UKM     = exact D1B RecipientWrapContextV1 / wrap-r1 bytes
```

The `CMSORIforKEMOtherInfo` object is:

```text
SEQUENCE {
  wrap      AlgorithmIdentifier(id-aes256-wrap, parameters absent),
  kekLength INTEGER 32,
  ukm       [0] EXPLICIT OCTET STRING(RecipientWrapContextV1)
}
```

## Canonical UKM dependency

The exact D1B `wrap-r1` vector is reused as application context:

```text
length  = 165
SHA-256 = 4a6bbfc2baa973f7bf098f730fb1554d677a0c814ba178269571f1044cf8b53d
```

This matches D1B's frozen fixture hash; C2B1 does not invent another recipient-context encoding.

```text
UKMContextBound != RecipientAuthorized
```

## Exact OtherInfo vector

```text
length  = 190
SHA-256 = c97cb1b98e5ee12d13b3187fa5fd05e36952bf739b71dd56c5baa27b6384eb5e
```

The executable oracle constructs the DER from OIDs/TLV rules rather than treating the checked-in DER as self-validating. It then asks OpenSSL's ASN.1 parser to independently recognize `id-aes256-wrap`.

## Deterministic shared-secret fixture

C2B1 embeds the exact deterministic shared-secret bytes whose SHA-256 is already frozen by C2A1:

```text
42f558b0bc5d700a911b0fc67f62376f7aee4667f1969e03f18bdfdf3c59fbdc
```

This is a test fixture only.

```text
SharedSecretFixture != ProductionSecret
MLKEMSharedSecret != PayloadDEK
```

C2A1 remains the theorem that independently binds this fixture back to deterministic ML-KEM-768 encapsulation under the pinned OpenSSL 3.6.4 toolchain.

## Exact HKDF output

The 32-byte KEK is:

```text
704a80291f05965c17e8573930f2d1dbd38ca7b35d276afb4c14e828b238804c
```

with SHA-256:

```text
9f1f947062bee3b5c274dc80285c1f04c11a0b4aed15d726b3cd3efb498752f9
```

## Exact AES-256-WRAP vector

The fixed test DEK is bytes `0x80..0x9f`:

```text
808182838485868788898a8b8c8d8e8f909192939495969798999a9b9c9d9e9f
```

AES-256-WRAP produces exactly 40 bytes:

```text
bfa650d41df4cef6f7e84acefe5c7d25867970f3d8fa03ef8b1172b1155fa2db178ae3ffae1bdbc0
```

SHA-256:

```text
27899fd049714b7a03ce5af1506c708e099b8663e031495553c1fdce78bfe25b
```

The executable oracle also unwraps the result and requires byte equality with the original test DEK.

## Independence boundary

The Python oracle owns only DER framing and RFC 5869 HKDF logic using the standard library. It invokes the host OpenSSL executable as an independent ASN.1 parser and AES-256-WRAP/unwrap implementation.

The host OpenSSL version is recorded in the run output but is not promoted into a production dependency.

```text
HostOpenSSLUsedForVectorCheck != MycelixRuntimeDependsOnOpenSSL
```

## Split to C2B2

C2B1 deliberately does not freeze:

- `RecipientIdentifier` / `rid`;
- ML-KEM AlgorithmIdentifier placement in outer DER;
- KDF AlgorithmIdentifier placement in outer DER;
- `kemct` placement;
- full `KEMRecipientInfo` DER;
- `OtherRecipientInfo` wrapping;
- certificate/public-key binding;
- recipient authorization/currentness.

C2B2 must bind those exact fields and cross-check the complete artifact under the pinned OpenSSL 3.6.4 CMS KEM implementation.

## Required non-equivalences

```text
KDFWrapVectorPass != KEMRecipientInfoQualified
SharedSecretFixture != ProductionSecret
TestDEK != PayloadDEK
UKMContextBound != RecipientAuthorized
AESKeyWrapRoundTrip != CMSRecipientArtifactValid
C2B1Pass != C2B2Pass
```

## Claim ceiling

A PASS may establish only the exact D1B UKM -> DER `CMSORIforKEMOtherInfo` -> HKDF-SHA256 KEK -> AES-256-WRAP test-DEK vector and successful unwrap for this frozen corpus. It does not establish production ML-KEM correctness, recipient identity/authorization, complete CMS/KEMRecipientInfo correctness, production entropy/key custody, runtime integration, protected storage, legal compliance or deployment readiness.
