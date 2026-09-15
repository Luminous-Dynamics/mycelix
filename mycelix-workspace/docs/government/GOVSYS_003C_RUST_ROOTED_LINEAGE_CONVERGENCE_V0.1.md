# GOVSYS-003C-RC — Rust rooted-lineage convergence v0.1

## Purpose

This tranche converges two independently hosted-qualified theorem families without adding Root-C adapter semantics:

1. multi-edge observed constitutional history (#875); and
2. production Rust predecessor-owned transition verification (#899).

The convergence exists so the later production Root-C adapter can inherit both proof lineages in Git rather than copying either theorem or trusting serialized `verified=true` receipts.

## Exact parents

Selective merge commit: `5de1485eadb4abba1e2cfca275a296fe1ff94ec7`

Ordered parents:

1. qualified #875 — `8adf899952587e2e63ca19383c7a49bca8618377`;
2. qualified #899 — `9ac0ac1408b39c8ad9e61cd83a41f0889565fcf0`.

Selective merge tree: `1def3315b70a0f5812521b7d5b448cc0ad212619`.

The merge tree is the exact #875 tree plus only seven Rust realization/conformance files copied byte-for-byte from #899:

- `crates/mycelix-constitutional-root/Cargo.toml`;
- `crates/mycelix-constitutional-root/src/lib.rs`;
- `crates/mycelix-constitutional-root/examples/conformance.rs`;
- `crates/mycelix-constitutional-root/vectors/root_a_transition_pair_v1.json`;
- `crates/mycelix-constitutional-transition-verifier/Cargo.toml`;
- `crates/mycelix-constitutional-transition-verifier/src/lib.rs`;
- `crates/mycelix-constitutional-transition-verifier/tests/conformance.rs`.

No #899 workflow, documentation, temporary repair machinery, or unrelated second-parent tree content is imported.

## Qualification theorem

A green qualification proves only:

- the qualification child is a direct child of the exact selective merge;
- the merge has exactly the two ordered qualified parents above;
- the merge tree is exactly the frozen tree above;
- the first-parent → merge diff contains exactly the seven allowed Rust realization/conformance files;
- every imported blob is byte-identical to exact qualified #899;
- the qualified #875 two-edge history surface still executes;
- the imported Rust Root-A and transition verifier still execute under Rust 1.98.1, strict Clippy, and `wasm32-unknown-unknown`;
- qualification leaves the checkout immutable.

## Nonclaims

This convergence does **not** yet prove a production Root-C adapter, replay-set aggregation over Rust positive tokens, global uniqueness, closed-world source coverage, constitutional currentness, legal legitimacy, administrative authority, execution authority, or external-effect authority.

In particular:

`qualified observed history + qualified Rust verifier + exact convergence != current constitutional root`.

The next semantic child must accept only process-local positive Rust verifier tokens, derive the lineage domain internally, exact-rebind every transition projection, enforce replay-nonce conflict semantics at the supplied-set boundary, and project only an **observed rooted history** into CORE-LINEAGE. Root-D remains solely responsible for closed-world source coverage and current-head promotion.

The network remains infrastructure for institutions. It is not the sovereign.
