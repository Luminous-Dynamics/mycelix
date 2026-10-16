# Mycelix Regional Policy Kernel

This is a standalone, dependency-free Rust policy kernel for regional/community federation. It is deliberately separate from the larger Holochain workspace so its deterministic logic can be tested without compiling the full application graph.

## What it does

- Defines versioned region/community profiles and explicit ingress/egress rules.
- Exposes a canonical, deterministic profile payload for a cryptographic adapter to hash and authenticate.
- Requires a trusted profile-verifier adapter before profile policy can be evaluated.
- Requires a trusted consent-verifier adapter before an individual grant can authorize a disclosure.
- Requires both source and destination profiles to authorize a cross-boundary action.
- Rejects unknown scopes, expired profiles, unapproved routes, missing purposes/recipients, stale consent evidence, and mismatched consent grants.
- Uses explicit timestamps; the kernel does not read the wall clock or perform network I/O.

## Important trust boundary

This crate does **not** implement a cryptographic signature algorithm, profile trust-root distribution, legal interpretation, or physical data-residency attestation. Its ProfileVerifier adapter must recompute the digest over RegionalProfile::canonical_payload(), verify the signature over signature_message(), validate the signer against the locally configured trust roots, and handle key revocation/rotation. The ConsentVerifier adapter must authenticate the exact grant, check its issuer and current revocation state, and return an evidence-validity deadline no later than its revocation evidence remains trustworthy.

Do not implement either verifier as a boolean supplied by an untrusted caller. Unit-test mocks in this crate prove policy behaviour only, not cryptographic security or jurisdiction compliance. The policy kernel is one defense-in-depth layer, not proof that the storage/replication network enforces placement rules.

## Run

```sh
cd mycelix-workspace/crates/mycelix-region-policy
cargo test --locked
cargo clippy --all-targets -- -D warnings
```

No CI result is implied by these instructions. Current verification status must be read from the exact commit's workflow evidence.

## First acceptance targets

- Every asymmetric or missing source/destination authorization fails closed.
- 100% of seeded policy-tamper, replay, stale-profile and mismatched-consent cases reject.
- Zero restricted-record egress across the integrated storage, DHT/gossip, transport, backup, telemetry and adapter paths.
- Measure local pure evaluation on a documented reference host; p95 < 5 ms is an initial target, not a measured result.
