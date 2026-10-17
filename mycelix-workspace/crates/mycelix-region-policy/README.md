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

This crate does **not** implement cryptographic signature algorithms, trust-root distribution, legal interpretation, or physical data-residency attestation. The ProfileVerifier adapter must recompute the digest over the canonical profile payload and verify the profile signature. The AgreementVerifier must recompute the agreement digest and verify BOTH role-separated approval messages: the source approver must be authorized for the exact source profile, and the destination approver for the exact destination profile. Both signatures must approve the same agreement digest. All adapters must validate trust roots/key rotation/revocation and return bounded evidence-freshness deadlines. ClassificationVerifier must authenticate the exact resource ID, byte digest, and data class. ConsentVerifier must authenticate the exact grant, recipient/purpose binding and current revocation state, then return a bounded freshness deadline.

The adapter that calls evaluate_transfer must supply a trusted runtime timestamp—not a timestamp supplied by the requester—and re-evaluate immediately before the protected side effect. The kernel rejects evaluation timestamps earlier than evidence verification, but it cannot establish that the caller's clock is trustworthy. Before acting on a decision, the storage/transport adapter must recompute or otherwise authenticate the digest of the exact bytes to be emitted and compare it with the verified classification claim. Never put time-dependent freshness decisions into a Holochain integrity callback that must be deterministic across peers and time.

Do not implement either verifier as a boolean supplied by an untrusted caller. Unit-test mocks in this crate prove policy behaviour only, not cryptographic security or jurisdiction compliance. The policy kernel is one defense-in-depth layer, not proof that the storage/replication network enforces placement rules.

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
