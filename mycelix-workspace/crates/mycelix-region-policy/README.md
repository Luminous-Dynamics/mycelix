# Mycelix Regional Policy Kernel

This is a standalone, dependency-free Rust policy kernel for regional/community federation. It is deliberately separate from the larger Holochain workspace so its deterministic logic can be tested without compiling the full application graph.

## What it does

- Defines versioned region/community profiles, explicit bilateral federation agreements, and ingress/egress rules.
- Exposes deterministic canonical payloads for profile and agreement verification adapters.
- Binds every cross-boundary request to exact source and destination profile IDs/digests and a time-bounded agreement.
- Requires source and destination placement rules and the agreement to approve the exact data class, action, purpose, and recipient.
- Requires verifier-backed classification claims bound to the exact resource ID and content digest; a caller-supplied sensitivity label is insufficient.
- Requires trusted profile, agreement, classification, and consent verifier adapters with explicit evidence-freshness deadlines.
- Rejects unknown schemas, expired or stale evidence, unapproved routes, backdated decisions, missing purposes/recipients, and mismatched consent.
- Uses explicit timestamps; the kernel does not read the wall clock or perform network I/O.

## Important trust boundary

This crate does **not** implement cryptographic signature algorithms, trust-root distribution, legal interpretation, or physical data-residency attestation. The ProfileVerifier adapter must recompute the digest over the canonical profile payload and verify the profile signature. The AgreementVerifier must recompute the agreement digest and verify BOTH role-separated approval messages: the source approver must be authorized for the exact source profile, and the destination approver for the exact destination profile. Both signatures must approve the same agreement digest. All adapters must validate trust roots/key rotation/revocation and return bounded evidence-freshness deadlines. ClassificationVerifier must authenticate the exact resource ID, optional subject identifier, byte digest, data class, issuer and validity window. ConsentVerifier must authenticate a grant scoped to the exact subject, resource ID, content digest, data class, action, recipient, purpose and validity window, check current revocation state, and return a bounded freshness deadline.

The adapter that calls evaluate_transfer must supply a trusted runtime timestamp—not a timestamp supplied by the requester—and re-evaluate immediately before the protected side effect. The kernel rejects evaluation timestamps earlier than evidence verification, but it cannot establish that the caller's clock is trustworthy. Before acting on a decision, the storage/transport adapter must recompute or otherwise authenticate the digest of the exact bytes to be emitted and compare it with the verified classification claim. Never put time-dependent freshness decisions into a Holochain integrity callback that must be deterministic across peers and time.

Do not implement either verifier as a boolean supplied by an untrusted caller. Unit-test mocks in this crate prove policy behaviour only, not cryptographic security or jurisdiction compliance. The policy kernel is one defense-in-depth layer, not proof that the storage/replication network enforces placement rules.


## Effect boundary and replay

The crate now includes a short-lived, single-use effect-lease reference gate. It re-evaluates policy at the last boundary, binds the exact action/resource/content/recipient/purpose/request, pins the agreement and profile digests, enforces a maximum 5-second TTL, and consumes a lease nonce before calling the effect. If that callback errors, the lease remains consumed to avoid blind replay after an uncertain outcome.

The ledger is **process-local and volatile**. It is not cryptographic, durable, distributed, crash-safe, or an exactly-once side-effect guarantee. Production integration must refresh trusted revocation/evidence through adapters, independently derive the sink target, and atomically persist lease consumption with the side effect or use a qualified fencing protocol. See EFFECT_LEASE_V1.md and src/lease.rs.


## Evidence-driven effect recovery

`src/effect_journal.rs` adds a pure lifecycle model for `Prepared`, `Reserved`, `DispatchStarted`, `OutcomeUnknown`, `Committed`, and `Rejected` states. Only destination-verified evidence may resolve an unknown attempt; an uncertain dispatch is reconciliation-only. This module is not a persistent database, exactly-once mechanism, or network adapter. See `EFFECT_JOURNAL_V1.md` for the durable adapter contract, crash matrix, and acceptance criteria. `src/effect_journal_model.rs` enumerates all 2,520 order-preserving four-worker reservation interleavings and models lost-reply recovery against an abstract idempotent sink; it is not a real database or network test. The 24-case machine-readable corpus and closed schema are `EFFECT_JOURNAL_FAULT_CORPUS_V1.json` and `EFFECT_JOURNAL_FAULT_CORPUS_V1.schema.json`.

## Run

```sh
cd mycelix-workspace/crates/mycelix-region-policy
cargo test --locked
cargo clippy --all-targets -- -D warnings
```

No CI result is implied by these instructions. Current verification status must be read from the exact commit's workflow evidence.

## First acceptance targets

- Every asymmetric source/destination rule or missing/mismatched agreement authorization fails closed.
- 100% of seeded policy tampering, agreement replay, stale-profile/classification and mismatched-consent cases reject when the trusted adapters and kernel are tested together.
- Zero restricted-record egress across the integrated storage, DHT/gossip, transport, backup, telemetry and adapter paths.
- Measure local pure evaluation on a documented reference host; p95 < 5 ms is an initial target, not a measured result.


The durable production integration must compose with the existing [PR #780 commit-classification work](https://github.com/Luminous-Dynamics/mycelix/pull/780) and [AGENT-009 #803](https://github.com/Luminous-Dynamics/mycelix/issues/803). Do not create a competing outbox or interpret a persistence verdict as retry authorization.


Restored effect-attempt snapshots include a monotonic dispatch-crossed marker: after dispatch has started, a terminal rejection is valid only with destination-verified outcome evidence. Bounded state-space exploration is test-only and does not model a real durable store.
