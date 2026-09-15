# CORE-LINEAGE Stage 2 Current-Head v0.1 — Normative Invariants

Status: **dynamic covered-head structural composition; no source verifier and no institutional authority**

## CH-001 — Stage 1 stays stable

`mycelix-core-lineage-current-head` is a sibling of `mycelix-core-lineage` rather than an extension of its stable structural crate.

Stage 1 rooted-lineage semantics, commitment bytes, lockfile, and test corpus remain unchanged. Refreshing current-head evidence must not mutate stable lineage history.

## CH-002 — Input observation is already domain-qualified

`CoveredHeadObservationFacts` is an adapter shape for facts that a consuming domain has already qualified under its own source-authentication and closed-world coverage theorem.

Constructing this Rust value is not source authentication. This crate does not verify signatures, source registries, DHT state, institutional authority, or closed-world provenance.

For GOVSYS, #842 remains responsible for authenticating exact-source coverage before adaptation, and Root-D remains responsible for complete constitutional rebinding.

## CH-003 — Exact endpoint equality

Positive covered-head structure requires exact equality of:

- lineage-domain identity;
- endpoint source-descriptor identity;
- endpoint generation; and
- endpoint node identity.

A valid historical prefix does not become current merely because it is a valid rooted lineage.

## CH-004 — Complete live coverage only

`CoverageState::Indeterminate` cannot produce a positive token.

`CoveredHeadMode::HistoricalAsOf` cannot produce a live covered-current-head token, regardless of whether its head coordinate equals the current structural endpoint.

## CH-005 — EvidenceLease is reused, never reimplemented

Stage 2 uses #181 `EvidenceLease` directly.

The input lease must validate at qualification time. The positive token retains that exact lease; this crate never widens, replaces, or refreshes it.

Reusing a positive token later requires `validate_reuse_at(now_ms)`. Expired or future-invalid evidence therefore cannot remain silently positive through typestate alone.

## CH-006 — Known scheduled transitions cap the positive horizon upstream

If `next_known_transition_effective_at_ms = T`:

- `T <= now_ms` denies because the old endpoint is no longer eligible as current;
- an input evidence lease with `valid_until_ms > T` denies with a scheduled-transition horizon violation; and
- a lease whose exclusive end is at or before `T` may qualify if every other invariant holds.

Stage 2 does not silently repair an overlong source-coverage lease. The domain coverage theorem must explicitly supply evidence whose horizon is already bounded by known transition semantics.

## CH-007 — Stable lineage identity and dynamic evidence remain distinct

The positive token retains the Stage-1 stable lineage commitment unchanged and separately retains:

- exact head-record identity;
- exact coverage-evidence identity;
- exact verification-evidence identity;
- exact EvidenceLease; and
- optional known-next-transition time.

Refreshing verification/coverage evidence may therefore change dynamic evidence while preserving the same stable rooted-lineage commitment and endpoint.

## CH-008 — Positive token is local and non-serializable

`QualifiedCoveredCurrentHead` has private fields and does not implement `Serialize` or `Deserialize`.

Persisted evidence must be requalified rather than reloaded as an already-positive current-head token.

## CH-009 — No authority amplification

A qualified structural covered-head token does not itself establish source-verifier origin, institutional authority, policy authority, administrative authority, judicial authority, execution authority, or external-effect permission.

It is one prerequisite for a consuming domain's stronger currentness theorem.

## CH-010 — No ambient selection

The crate has no network, DHT, filesystem, database, process, environment, wall-clock read, randomness, reputation, stake, model score, latest-record heuristic, or effect path.

All structural facts and `now_ms` are explicit inputs.
