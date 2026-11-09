# Civil Awareness Envelope Validator

Offline, fail-closed structural and policy validator for the v0.1 civilian awareness evidence envelope used by UAS, maritime and civil-aviation research.

## Scope

This crate validates:
- the pinned JSON Schema Draft 2020-12 document and schema identity;
- the repository-pinned field-policy version and exact SHA-256 digest;
- the domain extension URI, revision and field allowlist;
- the `claim.value` allowlist for each `(domain, claim_type)`, rejecting nested objects and arrays in v0.1;
- RFC 8785 JCS payload and whole-envelope digests;
- strict JSON duplicate-key rejection and the 2 MiB input limit;
- event-time ordering, declared purpose vs allowed uses, retention expiry at ingest, rights state, source state, clock quality, freshness, calibration, missingness, abstention, dependency and spatial-purpose evidence.

It has **no network retrieval, device access, operational control integration, sensor ingestion, navigation/flight-control interface, or action authorization**. The JSON Schema is embedded with `include_str!`; remote `$ref` and `$dynamicRef` values are rejected. The compiler runs once and the reusable validator is applied to each input.

## Evidence dispositions

| Disposition | Meaning | CLI exit |
|---|---|---:|
| `REJECTED` | Schema, digest, allowlist, parser or structural invariant failed | 2 |
| `QUARANTINED` | Rights, authorization, privacy, purpose or retention gate failed | 3 |
| `UNRESOLVED` | Evidence quality / provenance is inadequate for replay comparison | 4 |
| `STRUCTURALLY_VALID_REQUIRES_AUTHORITY_RECEIPT` | Local structure and declared gates pass, but independent source attestation and signed authority receipt are still required | 0 |

Exit 0 does **not** mean accepted for deployment or field use. A self-declared `integrity_status: authenticated` is not cryptographic proof. This crate does not implement the shared evidence-plane receipt verifier, so it never emits an operationally accepted disposition.

The machine-readable report contains only stable reason codes, a schema error count and the pinned policy digest. It does not echo source values or raw validation messages.

## Run

From this directory:

```sh
cargo fmt --check
cargo test
cargo clippy --all-targets -- -D warnings
cargo run -- path/to/envelope.json
```

The binary accepts one JSON file path. It prints a JSON validation report to stdout and returns the disposition-specific exit code. Initialization and file/usage errors are reported to stderr with stable, value-free codes.

## Pinned artifacts

The crate embeds these sibling research artifacts from `mycelix-workspace/mycelix-praxis/docs/research/`:
- `civil-awareness-evidence-envelope-v0.1.schema.json`
- `civil-awareness-field-policy-v0.1.json`
- `civil-awareness-envelope-examples-v0.1.json`

Digest contract:
- `field_allowlist_digest`: SHA-256 over exact UTF-8 bytes of the field-policy file.
- `payload_digest`: SHA-256 over RFC 8785 canonical JSON bytes of `domain_extension.payload`.
- `envelope_content_digest`: SHA-256 over RFC 8785 canonical JSON bytes of the entire envelope after removing only `lineage.envelope_content_digest`; it therefore binds the other fields, including correction/supersession lineage.

The JCS library is pinned to the RFC 8785-compatible `serde_json_canonicalizer` implementation. The JSON Schema validator is pinned to `jsonschema 0.61`, Draft 2020-12, with format validation enabled and reference resolution disabled.

## Test strategy

The unit suite covers the three synthetic domain fixtures, unknown schema/policy identity, external-reference rejection, extra fields, wrong digests, nested claim values, unapproved rights, purpose mismatch, retention expiry, unverified sources, stale data, missing dependency groups, reversed event intervals, metadata/lineage digest tampering, duplicate JSON properties, over-size input and a canonicalization vector from RFC 8785.

The included examples are **synthetic and non-operational**. Unit tests prove only local parser/schema/policy/digest behavior. They do not establish source authenticity, independent truth, detector accuracy, vessel/aircraft safety, legal compliance, regulatory authorization or SOTA performance.

## Design constraints

- No weapon, interceptor, effector, mitigation, jamming, spoofing, takeover, targeting, hostile-intent or third-party-control functionality.
- No pilot, crew, controller or remote-operator identity / personal-location fields; no private communications, biometric fields or raw personal audio/video.
- No connections to vessel navigation control, aircraft flight controls, ATC/VTS commands, ACAS/TCAS or certified detect-and-avoid systems.
- Unknown schema revisions, policy mismatches and unapproved fields fail closed.
- Missing, stale, conflicted, withheld, unauthenticated and uncalibrated evidence remains unresolved or quarantined; it is never silently promoted to truth.

## Parent issue

[GCPN-CIVIL-ENVELOPE-VALIDATOR-001 — #5992](https://github.com/Luminous-Dynamics/mycelix/issues/5992)
