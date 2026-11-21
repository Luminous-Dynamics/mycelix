# Mycelix Directory Fabric reference model

This is a small, dependency-free Rust reference crate for the pure semantics that sit between replicated identity evidence and a local Policy Enforcement Point (PEP).

It is intentionally **not** a complete authorization engine, identity provider, Holochain zome, LDAP server, SCIM server, OIDC provider, Kerberos KDC, or production enforcement agent.

## What it models

1. **Causal authority-frontier checks.** A local snapshot is compared with a caller-supplied minimum frontier. A higher per-authority sequence number is not accepted as proof of ancestry by itself: the snapshot must carry an exact older authority position that an independent verifier has already validated. Same-sequence/different-digest positions and unproven descendants are incomparable. No wall-clock or last-writer-wins ordering is used.
2. **Snapshot rollback checks.** A realm mismatch and a policy epoch below the persisted local high-water mark are rejected. A result of `SatisfiesRequiredFrontier` is only a freshness precondition; it is not an Allow decision and does not establish global freshness.
3. **Bounded offline leases.** A lease binds realm, principal, resource, action, audience, purpose and policy digest. Only an explicitly classified low-risk read may use this offline path. A clock interval that overlaps the not-before or expiry boundary is indeterminate; clock uncertainty cannot extend the lease.
4. **Bounded group expansion.** Child-to-parent group inheritance is deterministic. Cycles and depth/node/edge budget exhaustion reject the entire expansion; no partial closure is returned.

## Deliberate reuse boundary

The crate does not implement a new policy language or a second constraint algebra. The complete request-context witness and constraint-subsumption/deny-preservation obligations remain with Mycelix authority work:

- [#4743 — one effective evaluation context](https://github.com/Luminous-Dynamics/mycelix/issues/4743)
- [#4854 — decidable constraint subsumption and deny preservation](https://github.com/Luminous-Dynamics/mycelix/issues/4854)

A production PEP must require both a sufficient snapshot assessment and the existing authoritative full-context policy result before an effect. An offline lease is a bounded freshness/availability condition, not a permission grant.

## Run

```sh
cargo test --manifest-path tools/myc-dir-fabric-reference/Cargo.toml
cargo clippy --manifest-path tools/myc-dir-fabric-reference/Cargo.toml --all-targets -- -D warnings
cargo fmt --manifest-path tools/myc-dir-fabric-reference/Cargo.toml -- --check
```

The crate has no third-party dependencies. The test suite contains deterministic unit and adversarial controls; it does not substitute for external property-testing, mutation testing, or a proof assistant.

## Explicit claim ceiling

The model does not verify signatures, issuer admission, Holochain DHT validity, source-chain ancestry proofs, trusted-clock attestation, SCIM/OIDC/SPIFFE conformance, Windows domain behaviour, anti-Sybil properties, or external effects. The caller must authenticate all frontier, policy, membership, issuer, time and lease evidence before constructing these values. The model checks deterministic semantic relationships only.

No compile/test/Clippy result is claimed until these commands run against the exact repository head. No production or benchmark superiority claim is made.
