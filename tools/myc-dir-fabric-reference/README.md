# Mycelix Directory Fabric reference model

This is a small, dependency-free Rust reference crate for the pure semantics that sit between replicated identity evidence and a local Policy Enforcement Point (PEP).

It is intentionally **not** a complete authorization engine, identity provider, Holochain zome, LDAP server, SCIM server, OIDC provider, Kerberos KDC, dependency-closure proof verifier, signature service, or production enforcement agent.

## What it models

1. **Causal authority-frontier checks.** A local snapshot is compared with a caller-supplied minimum frontier. A higher per-authority sequence number is not accepted as proof of ancestry by itself: the snapshot must carry an exact older authority position that an independent verifier has already validated. Same-sequence/different-digest positions and unproven descendants are incomparable. No wall-clock or last-writer-wins ordering is used.
2. **Snapshot rollback checks.** A realm mismatch and a policy epoch below the persisted local high-water mark are rejected. A result of `SatisfiesRequiredFrontier` is only a freshness precondition; it is not an Allow decision and does not establish global freshness.
3. **Bounded offline leases.** A lease binds realm, principal, resource, action, audience, purpose, exact policy epoch and digest, minimum authority frontier, and a named clock profile. The supplied time interval must use that same profile. Only an explicitly classified low-risk read may use this offline path. A clock interval that overlaps the not-before or expiry boundary is indeterminate; clock uncertainty cannot extend the lease.
4. **Bounded group expansion.** Child-to-parent group inheritance is deterministic. Cycles and depth/node/edge budget exhaustion reject the entire expansion; path-depth limits remain enforced even when a shared ancestor was already visited through another path, and no partial closure is returned.

## Canonical evidence bytes

The frontier, frontier-evidence, policy-snapshot and offline-lease types expose versioned, domain-separated canonical byte encodings. The frontier-evidence encoding includes the exact covered authority positions; the lease encoding includes the exact policy epoch, minimum frontier and clock-profile identifier it pins. External signing or digest code must sign these bytes together with the required signer/key and schema profile. This crate does not hash, sign, verify signatures, or validate ancestry proofs.

## Request-scoped causal cuts

`src/causal_cut.rs` defines:
- typed authority dependency kinds (realm policy, membership, issuer admission, credential status, group/role, delegation, federation, device/workload trust, resource policy, revocation and explicit deny);
- deterministic request-scoped cut encoding bound to exact request, realm, policy, dependency-graph snapshot and resolver profile digests;
- a separate closure-verification receipt bound to the exact cut and graph snapshot;
- a fail-closed assessment requiring every required authority position to be exact or covered by a supplied, independently verified ancestry witness.

The dependency closure verifier is an external trust boundary. The module **does not** prove the resolver listed every dependency, authenticate the verifier, verify its signature/proof, or authenticate the policy/entity graph snapshot. A caller must not set `VerifiedComplete` from its own unverified dependency list.

## Decision and effect receipts

`src/receipts.rs` separates:
- an authorization decision receipt (request, policy, cut, closure receipt, snapshot, evaluator build, nonce, clock profile, validity window and exact outcome basis);
- an effect observation receipt (same request and decision receipt, logical operation, physical attempt, idempotency commitment, effect owner and observed outcome);
- a logical effect-history checker that rejects identity substitution, attempt gaps, duplicate commits and new mutations while an earlier attempt remains unresolved.

A known exact forbid remains a denial. A permit or default-deny result cannot become a final decision while required dependencies are incomplete. An Allow receipt is not proof that an effect occurred. Unknown effect outcome is not no-effect: the same logical operation must be reconciled before retry.

These modules emit versioned, domain-separated canonical bytes. They do not hash/sign/verify those bytes or authenticate evaluator/effect-owner identities. Signature verification, trusted-root rotation, nonce consumption, trusted time, complete owner inventory and actual effect observation remain external qualification gates.

See [the RSCAC and receipt composition design](../../docs/research/MYCELIX_DIRECTORY_FABRIC_CAUSAL_CUT_AND_RECEIPTS_V0_1.md), [the 24-case adversarial corpus](../../docs/benchmarks/MYC_DIR_CAUSAL_CUT_RECEIPTS_V0_1.json), and [its Draft 2020-12 schema](../../docs/benchmarks/MYC_DIR_CAUSAL_CUT_RECEIPTS_V0_1.schema.json).

## Finite-state composition model

A TLA+ model and four negative-control configurations are in `mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricCausalCutReceipts.tla`. They exercise omitted dependency closure, scalar frontier substitution, retry while outcome is Unknown, and duplicate logical-effect commit. The [model note](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricCausalCutReceipts-model-2026-11-21.md) records intended invariants and abstraction limits. The model/configurations are source-present only; TLC has not been run.

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

The crate has no third-party dependencies. The branch currently authors 44 deterministic unit tests across frontier/snapshot/lease/group, dependency-closure, decision-receipt and effect-history semantics. They are source-present only until run on the exact head. They do not substitute for independent closure-proof validation, external property-testing, mutation testing, or a proof assistant.

## Explicit claim ceiling

The model does not verify signatures, issuer admission, Holochain DHT validity, source-chain ancestry proofs, trusted-clock attestation, SCIM/OIDC/SPIFFE conformance, Windows domain behaviour, anti-Sybil properties, or external effects. The caller must authenticate all frontier, policy, membership, issuer, time and lease evidence before constructing these values. The model checks deterministic semantic relationships only.

No compile/test/Clippy result is claimed until these commands run against the exact repository head. No production or benchmark superiority claim is made.
