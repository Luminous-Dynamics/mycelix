# SUP-CIV-000D1C2B2A — Exact KEMRecipientInfo DER Syntax Vector v1

Status: deterministic ASN.1/CMS recipient-structure vector only. No recipient certificate validity, authorization, or CMS interoperability is established by this tranche.

Structural parent: SUP-CIV-000D1C2B1 / draft #2542 / exact head `dfeff416fa0d3c8be722ac481b31fa361fd7fd5f`.

Canonical transcript dependency: SUP-CIV-000D1B / draft #2348 / exact head `1152251e1b8d09bf69cc2497164a0fc1d58da320`.

Tracking issue: #2548.

## Purpose

Freeze one exact RFC 9629 / RFC 9936 `KEMRecipientInfo` DER object from already-frozen lower-layer test fixtures, while keeping certificate/RID binding and real CMS interoperability for C2B2B.

```text
DERWellFormed != CMSInteroperable
KEMRecipientInfoVector != RecipientCertificateValid
TestRID != RecipientAuthorized
```

## Exact nine-field object

The vector encodes exactly:

```text
KEMRecipientInfo ::= SEQUENCE {
  version       INTEGER 0,
  rid           [0] IMPLICIT OCTET STRING(test subjectKeyIdentifier),
  kem           AlgorithmIdentifier(ML-KEM-768, parameters absent),
  kemct         OCTET STRING(exact deterministic 1088-byte fixture),
  kdf           AlgorithmIdentifier(HKDF-SHA256, parameters absent),
  kekLength     INTEGER 32,
  ukm           [0] EXPLICIT OCTET STRING(exact D1B RecipientWrapContextV1),
  wrap          AlgorithmIdentifier(AES-256-WRAP, parameters absent),
  encryptedKey  OCTET STRING(exact C2B1 wrapped test DEK)
}
```

This tranche deliberately treats the `rid` as syntax material only.

## Test recipient identifier

The frozen 20-byte test subject-key-identifier is:

```text
f61e8bc9b896925256cce487facf27a108eb5b04
```

It is reference test material derived from the deterministic ML-KEM-768 public-key fixture using the conventional RFC 5280 SHA-1 subjectPublicKey method.

```text
SKIMatchesPublicKey != CertificateBindingProven
SKIMatchesPublicKey != CurrentRecipientAuthorization
```

C2B2B must independently bind a `RecipientIdentifier` to a frozen certificate/public key and prove CMS processing under the pinned OpenSSL 3.6.4 toolchain.

## Exact AlgorithmIdentifiers

ML-KEM-768:

```text
OID = 2.16.840.1.101.3.4.4.2
DER = 300b0609608648016503040402
parameters = absent
```

HKDF-SHA256:

```text
OID = 1.2.840.113549.1.9.16.3.28
DER = 300d060b2a864886f70d010910031c
parameters = absent
```

AES-256-WRAP:

```text
OID = 2.16.840.1.101.3.4.1.45
DER = 300b060960864801650304012d
parameters = absent
```

The stdlib oracle constructs those `AlgorithmIdentifier` values from OID arcs and verifies exact DER equality. There is no optional NULL parameter normalization.

## Exact KEM ciphertext fixture

The deterministic ML-KEM-768 ciphertext is stored as an immutable binary fixture:

```text
length  = 1088
SHA-256 = 39826fe40dc54a3fef68b7c228ed8fb22931012b6fa3bd3e7f204d54db0ac1e1
Git blob = 47b67c0882f4c2a9ce51c4df8a25627bdc040172
```

This is a qualification fixture only.

```text
DeterministicKEMCiphertext != ProductionCiphertext
```

## Exact UKM and encrypted key

UKM is the exact D1B `RecipientWrapContextV1 / wrap-r1` vector:

```text
length  = 165
SHA-256 = 4a6bbfc2baa973f7bf098f730fb1554d677a0c814ba178269571f1044cf8b53d
```

The encrypted key is the exact 40-byte C2B1 AES-256-WRAP fixture:

```text
bfa650d41df4cef6f7e84acefe5c7d25867970f3d8fa03ef8b1172b1155fa2db178ae3ffae1bdbc0
```

SHA-256:

```text
27899fd049714b7a03ce5af1506c708e099b8663e031495553c1fdce78bfe25b
```

## Frozen KEMRecipientInfo result

The exact DER object is:

```text
length  = 1378 bytes
SHA-256 = 2f9b7c9c293fdf4592bd98856f312b7ed2cbc054535e2f5ec1d8a76a299aef02
```

The oracle does not merely compare this digest. It constructs the object, re-parses the top-level SEQUENCE, requires exactly nine fields, verifies every tag/value, and refuses parameter or field drift.

## Independent ASN.1 parser

After its own stdlib encode/decode checks, the oracle passes the exact DER to the runner's `openssl asn1parse` and requires recognition of:

- ML-KEM-768 by name or exact OID;
- HKDF-SHA256 exact OID;
- AES-256-WRAP by name or exact OID;
- context-specific `[0]` fields.

Friendly-name availability is not normative; exact OIDs are.

```text
OpenSSLLabelTable != DERMeaning
```

The host OpenSSL is qualification tooling only and becomes no Mycelix runtime dependency.

## Deliberate C2B2B boundary

B2A does not establish:

- an X.509 certificate fixture;
- certificate validity;
- that the test RID selects a real certificate;
- public-key/certificate currentness;
- actual CMS `OtherRecipientInfo` interoperability;
- ML-KEM decapsulation inside CMS;
- KDF/unwrap inside CMS;
- recipient authorization;
- production key custody.

C2B2B must cross-check the field-level object against pinned OpenSSL 3.6.4 CMS/KEM processing and add recipient-substitution/RID-mismatch/context-mutation failures.

## Required non-equivalences

```text
DERWellFormed != CMSInteroperable
KEMRecipientInfoVector != RecipientCertificateValid
TestRID != RecipientAuthorized
SKIMatchesPublicKey != CurrentRecipientAuthorization
AlgorithmIdentifierCorrect != CryptographicExecutionCorrect
B2APass != B2BPass
B2BPass != ProtectedReadAuthorized
```

## Claim ceiling

A PASS may establish only the exact syntax, canonical DER bytes, nine-field parse, AlgorithmIdentifier parameter absence, immutable fixture binding, and independent ASN.1 recognition for this frozen test `KEMRecipientInfo` object. It does not establish recipient certificate validity, recipient authorization/currentness, complete CMS interoperability, production entropy/key custody, runtime integration, protected-storage access authority, legal compliance, municipal legitimacy or deployment readiness.
