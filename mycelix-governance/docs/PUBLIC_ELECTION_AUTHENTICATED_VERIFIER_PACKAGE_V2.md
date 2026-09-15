# Public Election Authenticated-Verifier Evidence Package V2 (ELECT-017C)

## Purpose

ELECT-017C makes the hosted-qualified ELECT-017B verifier public-key bundle and hosted-qualified Xenia authentication profile mandatory, self-contained offline evidence without changing ELECT-011's already-qualified V1 package contract.

The V2 profile is additive. It wraps a complete `ElectionEvidencePackageManifestV1` and exactly two typed public extension artifacts:

1. `authentication/verifier-public-key-bundle-v1.bin` — the exact canonical bytes produced by ELECT-017B;
2. `authentication/xenia-authentication-profile-v1.txt` — the exact 1120-byte Xenia V1 profile whose qualified SHA-256 is `e8b6ac90028c3064880bfb6b59ac7fbd1a1db04182bd3841e656bba2736d2b90`.

Old ELECT-011 V1 packages remain valid under their original profile. They are not, by themselves, sufficient evidence for authenticated-verifier certification.

## Why a wrapper instead of changing ELECT-011

ELECT-011 V1 has a closed `EvidenceArtifactKind` enum and eight required logical artifacts. Adding new enum variants or mutating `REQUIRED_PACKAGE_ARTIFACT_KINDS` would retroactively change the semantics of the qualified V1 theorem.

ELECT-017C therefore leaves the V1 manifest and its package root unchanged. The V2 wrapper validates the legacy manifest using ELECT-011's original validator, then independently validates and commits the authenticated-verifier extension.

## Important PackageIntegrity distinction

`validate_evidence_package_manifest` proves V1 manifest shape, required kinds, canonical public paths, and nonzero commitments. It does not recompute the V1 `package_root_digest` from archive bytes. That remains the responsibility of the existing offline verifier `PackageIntegrity` stage.

Accordingly, ELECT-017C does not claim legacy package-content integrity merely because the wrapped V1 manifest is structurally valid.

ELECT-017C instead proves content parity for its two new extension artifacts by hashing the actual carried bytes and comparing them to qualified parent evidence.

## Parent evidence is recomputed

The V2 qualifier consumes the actual typed ELECT-017 authorization root/policy, actual ELECT-017B public-key bundle, and actual ELECT-017B requirements binding. It recomputes rather than trusts caller-supplied labels:

```text
ELECT-017 root + policy
        -> ELECT-017 authorization-root digest

ELECT-017B typed public-key bundle + same root/policy
        -> exact canonical bundle bytes
        -> bundle digest

ELECT-017B typed requirements + bundle + same root/policy
        -> ELECT-017B requirements digest
```

The bytes carried by the package must equal the exact canonical ELECT-017B bundle bytes. A correct-looking manifest digest paired with different bytes fails closed.

## ELECT-017B bundle byte grammar

The public-key bundle artifact is not an opaque serialization. Its language-neutral canonical byte grammar is inherited exactly from qualified ELECT-017B:

```text
"MYCELIX:PUBLIC-ELECTION:VERIFIER-PUBLIC-KEY-BUNDLE:V1\0"
|| len(binding_profile_id):u32-be || binding_profile_id:utf8
|| elect017_authorization_root_digest[32]
|| election_definition_digest[32]
|| jurisdiction_snapshot_digest[32]
|| xenia_authentication_profile_digest[32]
|| xenia_authentication_suite_registry_digest[32]
|| record_count:u16-be
|| repeated canonical record_count times(
     verifier_release_digest[32]
     || suite_id:u16-be
     || xenia_authentication_profile_digest[32]
     || signer_key_id[32]
     || public_key_length:u32-be
     || public_key_bytes[public_key_length]
   )
```

Records are ordered lexicographically by `(verifier_release_digest, suite_id, signer_key_id)` before encoding. The qualified Xenia profile independently defines signer-ID derivation, suite IDs, key sizes, and authentication semantics. An offline decoder must consume exactly `record_count` records, respect the declared bounded key lengths, and reject truncation or trailing bytes rather than interpreting them as another format.

ELECT-017C itself avoids a second parser implementation: it asks the already-qualified ELECT-017B implementation to regenerate the exact canonical bundle bytes from the typed parent evidence and requires byte-for-byte equality with the carried artifact. The grammar above makes the same artifact independently implementable outside Rust.

## Exact Xenia profile bytes

The package carries the Xenia profile text itself, not merely its hash. Qualification requires:

```text
length(profile bytes) == 1120
SHA-256(profile bytes)
    == e8b6ac90028c3064880bfb6b59ac7fbd1a1db04182bd3841e656bba2736d2b90
```

This lets an offline verifier recover the suite registry, key sizes, transcript domains, endian rules, context grammar and verification-profile declarations without GitHub, DNS, Holochain, a key server, Symthaea, or vendor infrastructure.

## Extension-root commitment

The extension root is order-independent and duplicate-rejecting. Canonical refs are sorted by typed role and encoded as:

```text
"MYCELIX:PUBLIC-ELECTION:AUTHENTICATED-VERIFIER-EXTENSION-ROOT:V1\0"
|| extension_count:u16-be
|| repeated(
     role:u8
     || path_len:u32-be || canonical_path_utf8
     || content_digest[32]
     || byte_length:u64-be
   )
```

Roles are:

```text
1 = verifier public-key bundle
2 = Xenia authentication profile
```

Exactly one of each is required. Paths are fixed and public. Extension paths may not collide with any V1 artifact path.

## V2 package identity

ELECT-017C never rewrites the legacy package root. It derives a separate package identity:

```text
SHA-256(
  "MYCELIX:PUBLIC-ELECTION:AUTHENTICATED-VERIFIER-EVIDENCE-PACKAGE:V2\0"
  || len(v2_profile_id):u32-be || v2_profile_id
  || legacy_v1_package_root_digest
  || authenticated_verifier_extension_root_digest
  || ELECT-017B_requirements_digest
)
```

There is no self-hash field in the manifest; the qualified object returns the computed identity.

## Resource envelope

ELECT-017B permits at most 128 public-key evidence records. A worst-case record using the 1952-byte ML-DSA-65 public key occupies 2054 canonical bytes before the small bundle header, so even an intentionally pessimistic all-ML-DSA 128-record bundle is below 264 KiB.

ELECT-017C therefore imposes a 512 KiB preflight ceiling on the carried verifier public-key bundle. This leaves substantial framing headroom while preventing an unbounded package allocation. The Xenia profile is exactly 1120 bytes and the extension artifact count is exactly two.

## Adversarial requirements

The executable corpus must reject at least:

- missing or duplicate extension roles;
- non-public extension evidence;
- wrong or noncanonical extension paths;
- collision with a V1 canonical path;
- changed carried public-key bundle bytes despite a correct manifest label;
- changed, truncated, or substituted Xenia profile bytes;
- stale/spliced ELECT-017 authorization root;
- stale/spliced ELECT-017B requirements digest;
- extension-ref digest or length mismatches;
- oversized public-key bundle input;
- invalid legacy V1 manifest shape.

Extension input order is non-semantic.

## Canonical V1 test vector

For the existing ELECT-017/ELECT-017B golden fixture and an ELECT-011 V1 package root of `04` repeated 32 times:

```text
ELECT-017B canonical public-key bundle length
6838 bytes

ELECT-017B public-key bundle digest
97565d554ffaddbcc35d2a22d209ec1fb2646c0f4303146bb73c3d9640fdbefc

ELECT-017B requirements digest
5885c0782e5fcddc7d96f4dea04a08aeec53e3f78934a1ac390a7467ed0cb293

Authenticated-verifier extension root
f39819fd42dfc9c9da4b9c8bb98a10baa37e4b840b0c5e74c6ff4c74b8b0f2a9

Authenticated-verifier V2 package identity
37646d8240afdcaecaaf8f810254a720b5bbafde329cb3ef2f274ca0205bd33d
```

The hosted qualification lane independently reconstructs these values in Python.

## Deliberate non-claims

ELECT-017C does not prove:

- the legacy V1 archive bytes match `package_root_digest` absent the existing PackageIntegrity stage;
- Ed25519 or ML-DSA signature validity;
- private-key possession, custody, HSM integrity, or signer uncompromisedness;
- key lifecycle authorization/currentness;
- independent verifier quorum;
- ballot secrecy, receipt freeness, coercion resistance, cast-as-intended, recorded-as-cast, or tallied-as-recorded;
- legal election certification.

It proves that an authenticated-verifier-capable package carries the exact public-key/profile extension evidence required to reproduce later authentication offline, while preserving ELECT-011 V1 semantics unchanged.
