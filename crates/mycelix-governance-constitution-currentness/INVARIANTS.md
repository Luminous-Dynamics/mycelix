# Mycelix Governance Constitution Currentness v0.1 — Normative Invariants

Status: **pure evidence contract; genesis-only profile; no Holochain authority**

## 1. This crate defines evidence shape, not provenance

`mycelix-governance-constitution-currentness` contains no Holochain calls, DHT reads, signature verification, institutional trust logic, lifecycle admission or external effects.

A value that passes `LeasedVerifiedCurrentConstitution::validate_at` is internally consistent evidence-shaped data. That fact alone does not prove the value came from the designated local verifier.

Positive consumers must separately establish provenance by invoking the designated verifier themselves.

## 2. One canonical wire ABI

`LeasedVerifiedCurrentConstitution` is the single canonical transport type for this protocol profile. Producer and consumer zomes must depend on this crate instead of maintaining local mirror structs.

Field order and field types are protocol surface. Any incompatible wire change requires explicit protocol/profile versioning.

## 3. Semantic identity and currentness evidence identity are separate

`statement_digest` identifies the constitutional semantic epoch.

`currentness_evidence_digest` identifies one bounded observation that justified treating that epoch as current.

Refreshing the observation window for unchanged DNA and statement semantics must preserve `statement_digest` and change `currentness_evidence_digest`.

## 4. Currentness evidence is canonical

`currentness_evidence_digest` commits the exact:

- currentness evidence domain/profile;
- DNA identity;
- constitutional statement profile;
- statement digest;
- lease basis;
- verification time; and
- validity horizon.

`verification_ref` is derived deterministically from that digest/profile.

A consumer must recompute both rather than trusting serialized values.

## 5. v0.1 is genesis-only

This profile accepts only a valid `ConstitutionStatement` with:

- `version == 1`; and
- `parent_statement_digest == None`.

It also requires:

- `verified_transition_count == 0`;
- `legacy_constitution_authoritative == false`;
- `genesis_currentness_by_amendments_disabled == true`;
- `transition_currentness_supported == false`; and
- `candidate_discovery_used_for_positive_currentness == false`.

A syntactically valid amendment statement is still invalid under this currentness profile.

## 6. Lease width is bounded by contract

`valid_until_ms` must be strictly later than `verified_at_ms`, and their difference may not exceed `GENESIS_CURRENTNESS_REUSE_MS`.

At validation time:

`verified_at_ms <= now_ms < valid_until_ms`.

The 30-second bound is a local reuse cap on a fresh immutable-genesis observation. It is not constitutional authority expiry.

## 7. Construction is canonical but not authoritative

`new_genesis` constructs the only canonical genesis evidence shape for this profile and immediately validates it.

The constructor being public does not make locally constructed values authoritative. Authority provenance remains a host/runtime responsibility outside this pure crate.

## 8. Tampering fails closed

Validation denies at least:

- wrong protocol;
- malformed DNA identity;
- invalid or non-genesis statement;
- statement digest substitution;
- wrong evidence profile;
- unsupported transition/amendment mode;
- wrong lease basis;
- zero, inverted, widened, future-dated or expired lease;
- currentness evidence digest substitution; and
- verification-reference substitution.

## 9. Amendment support requires a new theorem and versioned contract

This v0.1 profile must not be broadened in place to accept transition-derived constitutional currentness.

An amendment-capable successor requires a versioned protocol/profile and must commit every additional currentness dependency needed by the amendment theorem, including complete transition-state evidence and bounded verifier horizons.

## 10. No authority by convenience

Serialization, deserialization, a valid digest, a valid reference, a short lease, local construction, caller possession, DHT availability, reputation, model output or absence of a newer record does not create constitutional authority.
