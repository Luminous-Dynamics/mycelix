# PSI-002B3A3A — Structural RFC 9578 Issuer Directory

Status: **source candidate / structural only / not qualified**

This crate models one normalized RFC 9578 issuer-directory observation and joins it to one exact corrected B3A query-credit policy.

It deliberately does **not** perform HTTP, TLS, certificate validation, Blind-RSA verification, trusted-time evaluation, or policy authorization.

## Governing boundary

```text
exact full token key appears in normalized directory observation
+ exact configuration digest matches B3A policy
!= directory response authentic
!= directory observation fresh
!= not-before satisfied under trusted time
!= key currently admitted
!= token verified
```

## RFC 9578 structure preserved

The observation preserves:

- issuer name;
- directory Origin;
- issuer request URI;
- exact directory media type;
- response-body SHA-256;
- ordered `token-keys` list;
- each key's token type;
- exact public-key/SPKI bytes;
- full 32-byte token-key ID;
- optional normalized `not-before` Unix seconds;
- normalized HTTP cache observations;
- retrieval provider/profile/receipt identity.

Source order remains significant because RFC 9578 uses list order as preference during key rotation.

## Full ID vs truncated issuance ID

For this structural layer, the full key identity is always the canonical lowercase SHA-256 of the exact supplied key bytes.

The RFC issuance request truncation is represented separately as the **last byte** of the full token-key ID.

The adversarial corpus includes two distinct full key IDs with the same one-byte truncated ID.

```text
same truncated ID
!= same full key
!= same admission identity
```

A truncated collision is surfaced but does not cause the full keys to be conflated.

## Exact B3A join

`IssuerDirectoryAdmissionPolicyV1` is constructed from an exact `QueryCreditPolicyV1` and binds:

- exact query-policy commitment;
- service domain;
- issuer;
- token type;
- required full token-key ID;
- exact directory Origin;
- retrieval profile;
- freshness profile identity;
- maximum directory age policy.

The structural evaluator additionally requires:

```text
observation.response_body_sha256
== query_policy.issuer_configuration_sha256
```

and requires the exact full token key named by B3A to appear under the exact token type.

## `not-before` and cache fields

RFC 9578 uses optional `not-before` to stage key rotation and recommends ordinary HTTP cache semantics for the directory. RFC Editor erratum 8680 reports a correction to the cited epoch reference and clarifies JSON-number representation.

This crate stores normalized `not_before_unix_seconds: Option<u64>` and normalized cache observations but **never compares them to trusted time**.

Therefore friendly-looking `Date`, `Age`, `max-age`, `Last-Modified`, or provider-observed timestamps cannot mint freshness/currentness.

## Positive type

The strongest type is deliberately named:

`StructurallyConsistentIssuerDirectoryKeyObservationV1`

It can establish only that the exact B3A key/configuration is structurally present in the normalized observation and that SHA-256 of the supplied key bytes equals the full key ID.

It remains false for:

```text
SPKI RFC9578 profile verification
directory payload authentication
retrieval-provider trust
directory freshness
trusted-clock not-before satisfaction
issuer-key admission/currentness
token cryptographic verification
query credit
application authority
```

## Source corpus

Tests cover exact-key presence, key-byte/digest mismatch, full-key absence despite same token type, duplicate full IDs, ordered rotation semantics, deterministic one-byte truncated-ID collision, raw `not-before` retention, non-authoritative cache claims, configuration-digest mismatch, retrieval-profile substitution, and raw observation serde round-trip.

## Next

A later provider adapter should parse/authenticate the actual RFC 9578 directory response and emit evidence for this normalized observation. A separate trusted-clock/currentness join should evaluate cache lifetime and `not-before`; neither theorem belongs in this structural crate.
