# CORE-LINEAGE Stage 2 — Dynamic Covered Current Head v0.1

## Purpose

Add the dynamic covered-head half of CORE-LINEAGE without changing the already-qualified stable Stage-1 rooted-lineage theorem.

Stage 2 is implemented as a sibling crate, `mycelix-core-lineage-current-head`, directly above qualified #906. That preserves the Stage-1 crate, stable commitment, vector, and lockfile byte-for-byte while allowing dynamic evidence to depend explicitly on qualified #181 `EvidenceLease`.

## Core decomposition

```text
domain-qualified root + semantic transitions
        ↓
ProjectedRootedLineage                [stable Stage 1]
        +
already domain-qualified closed-world covered-head observation
        +
EvidenceLease                         [dynamic Stage 2]
        ↓
QualifiedCoveredCurrentHead
```

The central non-equivalence remains:

`valid rooted lineage != covered current head != source authentication != institutional authority != external-effect authority`.

## Adapter boundary

`CoveredHeadObservationFacts` is deliberately an adapter shape, not a source verifier.

For a real consuming domain, the observation must already come from that domain's independently qualified source and closed-world coverage theorem. Constructing the Rust value directly does not authenticate a source and does not prove verifier origin.

For GOVSYS, #842 remains responsible for exact-source authentication and closed-world coverage. Root-D later rebinds complete constitutional semantics before Stage-2 composition.

## Exact endpoint theorem

At qualification time Stage 2 requires exact equality between the stable lineage endpoint and the supplied coverage facts for:

- lineage-domain identity;
- source-descriptor identity;
- generation; and
- node identity.

This prevents a valid historical lineage prefix from being promoted merely because it is structurally valid.

`CoverageState::Indeterminate` fails closed. `CoveredHeadMode::HistoricalAsOf` can never create a live covered-current-head token.

## Dynamic evidence lifetime

Stage 2 directly reuses #181 `EvidenceLease`; it does not define another lease algebra.

The exact input lease is retained unchanged in the positive token. Qualification validates it at explicit `now_ms`, and every later reuse must call `validate_reuse_at(now_ms)`.

Therefore a process-local positive token cannot safely be treated as permanently current simply because it was once constructed.

## Scheduled transition horizon

If the already-qualified domain observation knows a next transition becomes effective at `T`, Stage 2 requires:

```text
now_ms < T
evidence_lease.valid_until_ms <= T
```

If `T <= now_ms`, the predecessor endpoint is denied. If the supplied coverage lease extends past `T`, Stage 2 denies rather than silently clipping it. The source/coverage theorem must make the known transition boundary explicit in its own evidence horizon.

This gives the intended behavior:

```text
future successor known, not yet effective
        → predecessor may remain structurally current only under evidence bounded to T

successor effective now or earlier
        → predecessor cannot remain current
```

## Stable history vs refreshed evidence

The positive token separately retains:

- the unchanged Stage-1 stable lineage commitment;
- endpoint generation/node/source identity;
- head-record identity;
- coverage-evidence identity;
- verification-evidence identity;
- exact evidence lease; and
- optional next-known-transition time.

Refreshing coverage or verifier evidence can therefore change dynamic evidence and its lifetime without creating fictional changes to stable lineage history.

## Positive token boundary

`QualifiedCoveredCurrentHead` has private fields and no serialization/deserialization implementation.

It explicitly grants no institutional authority, policy authority, execution authority, or effect authority, and reports that source authentication is not verified inside this generic crate.

It is a prerequisite token for stronger domain-specific currentness, not a universal truth bit.

## Adversarial corpus

The Stage-2 tests freeze at least:

- exact live covered endpoint qualifies structurally;
- later covered generation against an old prefix denies;
- lineage-domain substitution denies;
- node substitution denies;
- source-descriptor substitution denies;
- indeterminate coverage denies;
- historical/as-of observation denies;
- future verifier evidence denies through shared EvidenceLease;
- expired evidence denies through shared EvidenceLease;
- overlong coverage across a known future transition denies;
- already-effective scheduled successor denies the old head;
- coverage exactly bounded to a future transition remains eligible before `T`;
- positive token reuse after evidence expiry denies; and
- refreshed dynamic evidence preserves the same stable Stage-1 lineage commitment.

## Qualification discipline

The hosted exact-head workflow must prove:

- one commit / five files above qualified #906;
- all Stage-1 and EvidenceLease blobs remain unchanged;
- the Stage-1 lockfile SHA-256 remains exactly `f1278453b3baa65c3572183fd85715bbee9e2a7a39f7edda37a54009732f27c1` before and after re-execution;
- stable-vs-dynamic and no-authority static boundaries remain present;
- Stage 1 reruns with `--locked`, strict Clippy and wasm;
- EvidenceLease tests and strict Clippy pass;
- Stage 2 passes rustfmt, all-target tests, strict Clippy and wasm against exact sibling copies; and
- the repository checkout remains immutable.

Independent diagnostic lanes run even after failures and one final aggregator fails closed.

## Follow-on

After this generic Stage-2 theorem qualifies, GOVSYS Root-D may adapt exact #842 authenticated coverage into `CoveredHeadObservationFacts`, bind it to the complete Root-A endpoint semantics, and produce a constitutional current-root token.

That later token must still remain separate from ordinary policy authority, administrative decision authority, judicial finality, execution authority, and external effects.
