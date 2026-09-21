# SUP-CIV-000D1C2A1 — Exact Deterministic ML-KEM-768 Primitive Vectors v1

Status: qualification-reference child of C2A. No CMS recipient artifact and no production key material.

Parent subject: SUP-CIV-000D1C2A / draft #2481 / exact head `805f68b6c9151bc390896b77bf108c00e33bfc93`.

Tracking issue: #2537.

## Purpose

Strengthen the C2A preflight from repeat-equality and size checks to exact deterministic primitive outputs.

The parent C2A script already pins OpenSSL 3.6.4 and uses the documented test-only ML-KEM controls:

```text
hexseed = bytes 00..3f
hexikme = bytes 00..1f
algorithm = ML-KEM-768
```

C2A1 does not rewrite that script. It executes the exact parent script by blob identity and requires the emitted primitive hashes to match the frozen values below.

## Independent reference observation

An independent local reference run using OpenSSL 3.5.5 reproduced the same deterministic public key, KEM ciphertext and shared secret byte-for-byte across two executions.

That observation is **reference only** until the pinned OpenSSL 3.6.4 C2A toolchain reproduces the same outputs.

```text
OpenSSL35ReferenceMatch != OpenSSL364Qualification
```

## Frozen inputs

```text
ML-KEM-768 hexseed:
000102030405060708090a0b0c0d0e0f
101112131415161718191a1b1c1d1e1f
202122232425262728292a2b2c2d2e2f
303132333435363738393a3b3c3d3e3f

ML-KEM-768 hexikme:
000102030405060708090a0b0c0d0e0f
101112131415161718191a1b1c1d1e1f
```

These inputs are testing fixtures only and must never be interpreted as production randomness.

## Frozen outputs

```text
public DER SHA-256
c23e23dd3d485a9256cda09358a4a286e00b373db10761eadf99f710649ca31c

KEM ciphertext length
1088

KEM ciphertext SHA-256
39826fe40dc54a3fef68b7c228ed8fb22931012b6fa3bd3e7f204d54db0ac1e1

shared secret length
32

shared secret SHA-256
0118707cb4fee1ea9004263262448f62d25696336983f298091637c25f1a12dd
```

The local reference run also observed a 1206-byte SubjectPublicKeyInfo DER object. C2A1 records that observation but does not make it an executable assertion because the unchanged parent C2A script does not emit the DER length.

## Qualification construction

The C2A1 wrapper:

1. proves the parent C2A script is the exact expected Git blob;
2. runs it unchanged;
3. therefore re-executes C2A's pinned-source download/hash/build, OpenSSL 3.6.4 version check, ML-KEM/CMS surface checks, deterministic key generation, deterministic encapsulation, repeat equality, decapsulation and length checks;
4. captures the emitted primitive hashes;
5. requires exact equality to this frozen corpus.

This makes C2A1 self-executing rather than qualification-inheriting.

```text
C2A1ExecutionIncludesC2APreflight
!= C2AParentMarkedQualified
```

## Boundary to C2B

C2A1 still does not construct or validate RFC 9629 `KEMRecipientInfo`.

It establishes only the deterministic primitive boundary needed before C2B freezes:

```text
RecipientWrapContextV1 -> UKM
CMSORIforKEMOtherInfo DER
HKDF-SHA256 KEK
AES-256-KW wrapped DEK
KEMRecipientInfo DER
```

```text
PrimitiveVectorMatch != RFC9629RecipientArtifactQualified
C2A1Pass != C2BPass
```

## Required non-equivalences

```text
DeterministicPrimitiveVector != ProductionEntropy
PrimitiveVectorMatch != RFC9629RecipientArtifactQualified
OpenSSL35ReferenceMatch != OpenSSL364Qualification
MLKEMSharedSecret != PayloadDEK
C2A1Pass != C2BPass
```

## Claim ceiling

A PASS may establish only that the exact pinned C2A OpenSSL 3.6.4 execution reproduces the frozen deterministic ML-KEM-768 primitive hashes and the parent C2A size/round-trip checks. It does not establish production entropy, key custody, recipient authorization, CMS KEMRecipientInfo correctness, RFC 9629/9936 recipient-wrap qualification, runtime integration, protected storage, legal compliance or deployment readiness.
