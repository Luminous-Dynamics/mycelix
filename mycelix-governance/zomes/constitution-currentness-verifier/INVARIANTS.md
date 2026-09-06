# Constitution Currentness Verifier v0.1 — Normative Invariants

Status: **genesis-only leased currentness verifier; workspace candidate; amendment currentness unavailable**

## 1. Currentness is a separate verifier role

The binding constitution authority defines DNA-bound genesis semantics. The transition coordinator stores/projects candidate transition semantics. This verifier answers the separate question: which exact constitutional statement may be reused as current now?

Neither constitution discovery nor transition storage gains currentness authority merely by being convenient.

## 2. Genesis currentness is derived from immutable DNA authority

While `constitution_authority.get_verified_constitution_genesis` reports `amendments_enabled = false`, the only possible binding constitutional state is the exact DNA-derived genesis statement.

The verifier independently recomputes and cross-checks:

- DNA identity shape;
- genesis statement validity;
- statement digest/profile;
- reconstructed genesis manifest digest/profile; and
- the genesis-only amendment mode.

## 3. Candidate discovery is not used for positive genesis currentness

`get_leased_current_constitution` MUST NOT call `get_links`, `get`, transition candidate discovery, transition projection, proposal authority, tally verification or threshold verification.

When amendments are disabled, DHT candidate presence/absence cannot change the DNA-rooted current constitution.

`candidate_discovery_used_for_positive_currentness = false`.

`candidate_discovery_grants_authority = false`.

`absence_of_transition_record_grants_authority = false`.

## 4. Amendment currentness fails closed

If the authoritative genesis boundary reports amendments enabled, this verifier MUST deny positive currentness.

It MUST NOT synthesize a transition lease, infer the newest transition from local DHT visibility, or reuse unbounded transition-verifier receipts.

Amendment currentness remains unavailable until a later theorem provides both:

1. complete/authenticated constitutional transition state; and
2. bounded verifier horizons for every proof dependency needed to establish the current constitutional head.

`amendment_currentness_supported = false`.

`unbounded_transition_verifier_receipts_accepted = false`.

## 5. The genesis lease is a reuse cap, not authority expiry

The DNA genesis itself is immutable for the hosting cell while amendments are disabled. `GENESIS_CURRENTNESS_REUSE_MS` is therefore a deliberately short local reuse/caching cap on one fresh observation, not a claim that constitutional authority naturally expires at that time.

`local_reuse_cap_is_authority_expiry = false`.

A consumer may shorten this window but never lengthen it.

## 6. Exact transport projection

`LeasedVerifiedCurrentConstitution` carries:

- protocol;
- exact DNA hash;
- exact statement bytes and digest;
- `verified_transition_count = 0`;
- `legacy_constitution_authoritative = false`;
- exact lease basis;
- verification reference;
- verification/validity times;
- explicit genesis-currentness mode; and
- explicit denial of transition/candidate currentness authority.

The transport object is deserializable evidence for direct local consumers. Deserializing caller-supplied bytes does not create authority.

## 7. No latest-record heuristic

Highest version, newest timestamp, last DHT arrival, candidate absence, author identity, reputation, stake, Phi, model output or local record ordering cannot establish constitutional currentness.

## 8. Failure semantics

Missing constitution authority, decode failure, malformed DNA identity, invalid statement, digest/profile mismatch, manifest mismatch, positive amendment mode, time overflow or any internal inconsistency denies.

There is no fallback to legacy mutable constitution state or transition candidate discovery.

## 9. Containment

This verifier performs no writes, lifecycle changes, execution actions or external effects. It is deliberately narrower than the transition coordinator and must remain absent from binding `dna.yaml` until its consuming currentness stack is qualified/provisioned together.

## 10. Amendment enablement gate

Before `amendment_currentness_supported` may become true, qualification must prove at minimum:

- an explicit authoritative transition source/head model;
- completeness independent of absence of a later local DHT record;
- bounded tally/right/threshold verifier leases;
- exact transition-lineage continuity and fork ambiguity denial;
- revocation/trust expiry propagation;
- canonical provenance over every transition-currentness dependency; and
- adversarial tests for partition, delayed propagation, stale verifier evidence and conflicting valid children.
