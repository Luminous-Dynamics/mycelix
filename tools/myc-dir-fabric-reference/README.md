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

`src/receipts.rs` separates three records:
- an authorization decision receipt (request, policy, cut, closure receipt, snapshot, evaluator build, nonce, clock profile and validity window);
- a physical effect-attempt receipt (same request and decision receipt, logical operation, unique attempt ID/number, idempotency commitment, effect owner and observed outcome);
- a separate effect-reconciliation receipt (exact prior Unknown/NotObserved attempt ID and number, same logical operation/request/decision/idempotency/effect-owner bindings, and ConfirmedCommitted or ConfirmedNoEffect).

Reconciliation does not consume a physical attempt number. A retry is eligible only after the exact previous attempt is confirmed to have produced no effect. If the retry becomes Unknown, the operation becomes unresolved again. The history checker rejects identity substitution, attempt gaps, duplicate reconciliation, reconciliation of resolved/mismatched attempts, duplicate commits and new attempts while an earlier outcome remains unresolved.

A known exact forbid remains a denial. A permit or default-deny result cannot become a final decision while required dependencies are incomplete. An Allow receipt is not proof that an effect occurred. Unknown effect outcome is not no-effect: a separate reconciliation receipt must identify the exact prior attempt and confirm no effect before a retry.

These modules emit versioned, domain-separated canonical bytes. They do not hash/sign/verify those bytes or authenticate evaluator/effect-owner identities. Signature verification, trusted-root rotation, nonce consumption, trusted time, complete owner inventory and actual effect observation remain external qualification gates.

See [the RSCAC and receipt composition design](../../docs/research/MYCELIX_DIRECTORY_FABRIC_CAUSAL_CUT_AND_RECEIPTS_V0_1.md), [the 24-case adversarial corpus](../../docs/benchmarks/MYC_DIR_CAUSAL_CUT_RECEIPTS_V0_1.json), and [its Draft 2020-12 schema](../../docs/benchmarks/MYC_DIR_CAUSAL_CUT_RECEIPTS_V0_1.schema.json).

## Effect-surface inventory closure

`src/effect_inventory.rs` models the completeness of the mutation-route inventory itself:
- routes bind an exact route ID, surface kind, effect owner and enforcement mode;
- a revocation-specific inventory binds resource scope, revocation digest, topology snapshot, inventory epoch, manifest digest and route-set digest;
- an external closure receipt binds that exact inventory to verifier identity/build, proof digest and verification status;
- only a verified-complete inventory in which every reachable route uses an atomic owner fence is strong-fence eligible;
- CAEP-signal-only, token-expiry-only and best-effort routes prevent a strict mutation-revocation claim;
- revocation-plan construction derives the owner set from the route inventory and rejects missing or extra owner targets.

The module checks structure, not proof authenticity or actual route discovery. The independent inventory verifier remains a critical trust boundary.

## Resource-enforced revocation fencing

`src/effect_fence.rs` models mutation-boundary fencing and requires an independently verified complete effect-surface inventory before a strict revocation plan can be built:
- each fence is scoped to an exact realm/resource/action and effect owner, not a global sequence;
- old tokens become stale when the resource owner installs a higher generation; unissued future generations and cross-resource tokens are rejected;
- a fenced command binds one exact Allow decision digest, request, logical operation, idempotency commitment and owner token;
- a revocation plan declares the required owner inventory and per-owner target generations; strict completion requires every exact owner acknowledgement, not a quorum;
- missing acknowledgements leave revocation pending; inventory completeness is represented by a bound closure receipt, while verifier authenticity and actual route discovery remain external trust boundaries.

Production condition: the effect owner must persist/install the new generation and compare the request token in the **same atomic transaction** as the protected mutation. A separate check followed by a later write is unsafe. The reference constructor starts at generation 1 for model genesis only; production recovery must restore the authenticated durable high-water mark and must never reset the counter to 1 after restart or snapshot rollback. Revocation is only complete after all independently inventoried owners acknowledge durable installation. An external API that cannot enforce this at its mutation boundary cannot receive the strong revocation guarantee.

## Queue delivery and worker handoff semantics

`src/queue_delivery.rs` binds one broker delivery to an exact broker namespace, destination, application message key and payload digest. It also binds the command to the request and decision-receipt digests, logical operation, effect owner, resource-fence generation, clock profile and validity interval.

A matching and time-valid delivery is only `EligibleForOwnerRevalidation`; it is not an Allow decision. The worker/effect owner must still verify the exact signed Allow and atomically compare the current owner fence at the protected mutation. Broker sequence numbers and delivery counters are transport metadata only.

The module separates broker acknowledgement from effect finality. An Unknown outcome blocks both retry and acknowledgement until a durable reconciliation task has been committed. After that handoff, the broker delivery may be acknowledged while reconciliation remains pending; this does not mean the effect is known. A new physical attempt is permitted only for an unstarted operation or after exact owner-side `ConfirmedNoEffect` reconciliation.

## Finite-state composition model

A TLA+ model and four negative-control configurations are in `mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricCausalCutReceipts.tla`. They exercise omitted dependency closure, scalar frontier substitution, retry while outcome is Unknown, and duplicate logical-effect commit. The [model note](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricCausalCutReceipts-model-2026-11-21.md) records intended invariants and abstraction limits. The model/configurations are source-present only; TLC has not been run.

## Deliberate reuse boundary

The crate does not implement a new policy language or a second constraint algebra. The complete request-context witness and constraint-subsumption/deny-preservation obligations remain with Mycelix authority work:

- [#4743 — one effective evaluation context](https://github.com/Luminous-Dynamics/mycelix/issues/4743)
- [#4854 — decidable constraint subsumption and deny preservation](https://github.com/Luminous-Dynamics/mycelix/issues/4854)

A production PEP must require both a sufficient snapshot assessment and the existing authoritative full-context policy result before an effect. An offline lease is a bounded freshness/availability condition, not a permission grant.

The security folder now contains three finite-state companions: `DirectoryFabricCausalCutReceipts.tla` (causal-cut/receipt composition), `DirectoryFabricEffectFence.tla` (per-owner generation and delayed command), and `DirectoryFabricEffectInventory.tla` (complete route inventory and fenceability). The inventory model has safe, omitted-route and CAEP-only negative-control configurations. All TLA models/configurations are source-present only; TLC has not been run.

## PostgreSQL effect-owner reference

The adjacent PostgreSQL reference adapter demonstrates a database-local mutation and an append-only effect/idempotency receipt under one transaction:
- [SQL schema and functions](postgres/001_effect_owner.sql)
- [Transactional self-tests](postgres/test_effect_owner.sql)
- [Concurrency, durability and integration contract](postgres/POSTGRES_EFFECT_OWNER.md)

It uses shared fence-row locks for concurrent mutations and an exclusive generation update for revocation. This is a reference contract, not production wiring; the SQL and two-session races have not been run against PostgreSQL. Reuse Mycelix-DeSci's existing PostgreSQL authority backend, signed write leases, database epochs and outbox rather than introducing a second authority database.

## Run

```sh
cargo test --manifest-path tools/myc-dir-fabric-reference/Cargo.toml
cargo clippy --manifest-path tools/myc-dir-fabric-reference/Cargo.toml --all-targets -- -D warnings
cargo fmt --manifest-path tools/myc-dir-fabric-reference/Cargo.toml -- --check
```

The crate has no third-party dependencies. The branch currently authors 106 deterministic unit tests across frontier/snapshot/lease/group, request-scoped closure, decision/effect receipts, owner fencing and effect-surface inventory closure. They are source-present only until run on the exact head. They do not substitute for independent closure-proof validation, external property-testing, mutation testing, or a proof assistant.

## Explicit claim ceiling

The model does not verify signatures, issuer admission, Holochain DHT validity, source-chain ancestry proofs, trusted-clock attestation, SCIM/OIDC/SPIFFE conformance, Windows domain behaviour, anti-Sybil properties, or external effects. The caller must authenticate all frontier, policy, membership, issuer, time and lease evidence before constructing these values. The model checks deterministic semantic relationships only.

No compile/test/Clippy result is claimed until these commands run against the exact repository head. No production or benchmark superiority claim is made.


Queue delivery adds 14 authored Rust unit tests. Current total across the six reference modules: 94 authored tests; none has been compiled or executed in this environment.


The queue-worker module also exposes WorkerClaimToken and WorkerClaimState. The resource owner must authenticate the claim proof and compare its current durable claim epoch together with the resource fence. Queue worker module tests: 22 authored; total across reference modules: 106 authored tests, not compiled or executed here.
