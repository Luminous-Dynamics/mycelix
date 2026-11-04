# ASSURE-PORTFOLIO-CUT-002 — Multi-valued transitive dependency reference model

**Parent:** [#5762](https://github.com/Luminous-Dynamics/mycelix/issues/5762)
**Canonical profile owner:** QUAL-001E / [#1371](https://github.com/Luminous-Dynamics/mycelix/issues/1371)
**Canonical cut-set owner:** ASSURE-PORTFOLIO-CUT-001 / [#5695](https://github.com/Luminous-Dynamics/mycelix/issues/5695), [#5696](https://github.com/Luminous-Dynamics/mycelix/pull/5696)
**Source dependency owner:** LEX-NET-030 / [#1243](https://github.com/Luminous-Dynamics/mycelix/issues/1243)
**Claim ceiling:** independent reference model over a caller-supplied finite dependency DAG; not authenticated independence or production qualification.

## Why this is separate

The existing v0.1 cut-set implementation is qualified for a bounded profile that has one declared identity per role/dimension. ITOPS and Symtropy integrations exposed a more general case: a role may depend on multiple hosts, clocks, stores, probes, operators or source lineages within the same canonical dimension, and two roles may share only part of that set.

This crate is an intentionally independent, dependency-free reference model. It does not change #5696's public API or composition disposition. Integrate it only after exact differential agreement and review.

## Model

A role owns a set of root dependency node IDs, an explicit classification for each of the 15 canonical fault-domain dimensions, and a transitive dependency closure derived from a directed acyclic graph.

Each node retains a dimension-qualified identity, evidence references, evidence currentness, and dependency-effect semantics. Two different node IDs with the same dimension-qualified identity are one fault-domain identity and remain shared. The same digest bytes in two different dimensions remain different keys.

Dimension coverage is one of:
- **Complete:** the reachable graph contains one or more known dependencies and the inventory is asserted complete.
- **Partial:** known dependencies are retained, but an unknown remainder remains; a rationale is mandatory.
- **Unknown:** the dependency inventory cannot be established; known nodes, if present, remain visible; a rationale is mandatory.
- **NotApplicable:** no dependency node may exist for that role/dimension and a claim-specific rationale is mandatory.

Missing dimensions, contradictory coverage, empty rationales, stale/revoked evidence, dangling edges, cycles, duplicate nodes and orphan nodes fail closed. Unknown-currentness evidence or incomplete coverage prevents a numeric cut-set claim.

Dependency effects are intentionally not collapsed:
- **ClaimCritical:** compromise can invalidate the role's evidence output and is included in the v0.1 any-one-compromise hitting-set model.
- **EvidentiaryLineage:** retained for correlation reporting but excluded from that attack model unless a separately pinned model says otherwise.
- **Optional:** not treated as a mandatory dependency.
- **RedundantOrFallback:** requires AND/OR fault semantics not implemented here.

Encountering optional or fallback nodes returns Unsupported with no numeric cut sets. This is safer than treating an alternative path as a mandatory single point of failure. A later attack-model version may implement Boolean dependency semantics and must be separately qualified.

## Algorithm

1. Bind claim, subject, profile, policy, dependency-manifest and attack-model digests to caller-supplied expected values.
2. Validate role sets, all 15 dimension declarations, node IDs, evidence status and graph references.
3. Reject cycles and compute each role's transitive dependency closure.
4. Canonicalize each dependency by the pair (FaultDomainDimension, Digest), not node ID or display label.
5. Preserve all shared dependencies and their role/effect classifications.
6. Check policy-required pairwise separation against the full identity sets, not one digest per dimension.
7. Require complete inventories and at least one claim-critical dependency per role before enumerating cut sets.
8. Enumerate all minimal hitting sets using a bounded incremental algorithm. On budget exhaustion, discard partial output and return no cardinality.

The cut-set semantics are explicit: compromising any one ClaimCritical key invalidates that role's output; a cut set hits every required role's claim-critical dependency set. This is not an availability model, probabilistic risk model or proof of absence of unknown causes.

## Differential tests

The test panel includes multiple identities in one dimension; partial overlap where only one of several dependencies is shared; distinct immediate services with a shared transitive host; known dependencies plus an unknown remainder; unknown-currentness evidence; shared evidentiary lineage that is reported but not misclassified as claim-critical; optional/fallback path rejection; same digest under different dimensions; full-set pairwise separation; alias node IDs for the same canonical identity; stale evidence, dangling edges, cycles, duplicate and orphan nodes; caller-bound subject/policy mismatch; and a bounded candidate-extension budget that never returns partial cut sets. Additional resource-adversary tests cover excessive edge count, role cardinality, oversized identifiers/evidence references, and oversized/control-character coverage rationales. Cycle detection uses an iterative three-colour DFS rather than recursive call depth, and the input dependency-node count is capped at 100,000 before graph traversal.

It also compares the incremental result against a separately implemented brute-force 2^N minimal-hitting-set oracle on 128 deterministic small graphs. The production and reference algorithms do not call a common minimal-cut-set search helper.

## API binding

The audit takes a typed `AuditBindings` value rather than a long positional argument list. The value groups the separately pinned policy digest, exact expected subject/profile/manifest/attack-model digests, and the candidate-extension budget. This makes accidental argument swaps harder to express and makes the resource limit explicit.

## Run

```sh
cargo fmt --manifest-path research/assurance-portfolio-multidependency/Cargo.toml --check
cargo check --locked --all-targets --manifest-path research/assurance-portfolio-multidependency/Cargo.toml
cargo test --locked --all-targets --manifest-path research/assurance-portfolio-multidependency/Cargo.toml
cargo clippy --locked --all-targets --manifest-path research/assurance-portfolio-multidependency/Cargo.toml -- -D warnings
```

Rust 1.98.1 is pinned; the crate has no third-party dependencies. The workflow runs on the exact pull-request head and checks the working tree after execution.

## Limitations

- Digest values and evidence references are caller-supplied identifiers. The crate does not hash the input DAG bytes or authenticate their origin; callers must verify exact-byte commitments and signer authority before invocation.
- Complete is an inventory declaration, not proof that no dependency was omitted.
- The current fault model includes only claim-critical dependencies with any-one-compromise semantics.
- Evidentiary, optional and fallback paths remain visible but are not assigned an attack probability or silently included in a different fault model.
- Graph closure is only as complete and truthful as the exported LEX-NET-030 source graph.
- The model does not establish statistical independence, organizational separation, truth of the underlying claim or production certification.
