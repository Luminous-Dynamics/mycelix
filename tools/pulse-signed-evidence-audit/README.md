# Pulse signed evidence audit

This is the first local cryptographic verifier for Pulse role-separated lifecycle evidence.

## Implemented profile

- DSSE v1 envelope carrying an in-toto Statement v1.
- Ed25519 signature verification over the exact DSSE PAE bytes.
- Digest-bound external trust-policy input.
- Role and predicate-type authorization for each signing key.
- Exact source commit and trace-byte digest binding.
- Unique attestation/event IDs and causal-edge DAG checks.
- Strict JSON decoding with unknown fields rejected.

SCITT RFC 9943 COSE Signed Statements are parsed as an envelope type and their opaque-byte SHA-256 commitment is checked, but COSE signature and receipt verification are **not implemented yet**. Any SCITT envelope therefore yields `INCOMPLETE`, never a cryptographic PASS.

## Run

```bash
cargo run --release -- \
  --bundle evidence-bundle.json \
  --trace multi-conductor-history.json \
  --trust-policy trust-policy.json \
  --output verification-report.json
```

Exit behavior:
- `ROLE_AUTHENTICATED_RELATIVE_TO_SUPPLIED_POLICY`: DSSE signatures and the supplied policy checks passed.
- `FAIL`: a cryptographic signature, role, subject, digest or causal contradiction was found.
- `INCOMPLETE`: a required verification path is absent or unsupported (including SCITT/COSE in this version).

## Trust policy

The trust policy file is a separate input. Its exact bytes must match `bundle.trust_policy.policy_sha256`. It contains:
- `policy_id` and `policy_version`;
- key IDs and 32-byte Ed25519 public keys (lowercase hex);
- roles and in-toto predicate types allowed for each key;
- key status (`active`, `suspended`, `revoked`).

The verifier does **not** authenticate the trust policy file itself. It assumes that the caller obtained it from a separately trusted source. A policy digest proves byte identity only. Root signature, rotation, validity interval, compromise-effective-time and historical validation are follow-on work under #5863 / #1354.

## Signed predicate binding

Each DSSE payload must be an in-toto Statement v1 with exactly one subject:
`pulse-trace:<repository>@<source_commit>`, digest `sha256=<trace_sha256>`.

Its typed predicate binds:
- `role`;
- `event_id`;
- `issuer_key_id`;
- `source_commit`;
- `trace_sha256`;
- role-specific `details`.

The wrapper role/event ID must match the signed predicate. At least one valid signature from a key authorized for that role and predicate type is required. The signed `issuer_key_id` must identify a successfully verified signature.

Unknown signatures, unknown keys, invalid or unauthorized signatures fail closed. The tool does not interpret a valid signature as proof that the asserted event is true.

## Limitations

This tool does not yet:
- verify COSE_Sign1 or RFC 9943 SCITT receipts;
- verify TUF or another trust-root signature over the supplied policy;
- implement key validity intervals, revocation history, or post-compromise historical evaluation;
- prove Holochain conductor honesty or independently query a DHT;
- validate the real-world truth of signed observations;
- authorize any operation or replace conductor CapGrant enforcement.

The report claim ceiling remains relative to the externally supplied trust policy. A later implementation must use a second independent verifier and mutation corpus before being considered qualified.
