# IPB Finality Reference

A standalone, dependency-free Rust reference model for the synthetic conformance lane tracked by Mycelix issue #6136.

## Scope

The crate checks a narrow relation between:
- an already-authorized historical operation binding;
- the exact operation presented at the effectuation boundary;
- an already-resolved current constitution and policy revision;
- owner-supplied current authority and resource-precondition states;
- prior effect/attempt state; and
- qualified retry-safety evidence.

It also reconciles per-action observations without inventing batch atomicity.

## Deliberate non-goals

This is **not** a production authorizer, capability issuer, policy engine, constitution registry, provider client, execution coordinator, cryptographic payload canonicalizer, or finality sink.

It does not verify the authenticity or truth of input references. Owner-native adapters must supply those facts and exact source references. It does not hash application payloads: the canonical operation digest is supplied by the domain owner, and this crate checks exact equality only.

A Permit result means only that this finite reference relation found no mismatch in its supplied inputs. It does not create a production permit or authorize a real effect. The caller still needs the authoritative owner execution path and any independently qualified effectuation controls.

Unknown authority, unknown resource state, unknown prior outcome, incomplete operation binding and partial effects do not become permission. A retry after a confirmed no-effect attempt is permitted by this model only when same-operation/same-idempotency-key safety is explicitly qualified.

## Authorization assurance extensions

The reference model also includes owner-input comparison helpers for:
- **Delegation attenuation:** child scopes must be subsets of immediate parents for action/resource/purpose/field, time, amount and operation-count dimensions.
- **Trajectory-wide privilege:** active task, ambient and tool/session grants are unioned; bounded amount ceilings are summed per currency (never across currencies), while operation-count ceilings are conservatively summed and compared with the approved task scope.
- **Complete mediation:** every reachable route to a protected effect must traverse the exact declared enforcement point; incomplete route inventories remain distinct.
- **Revocation propagation:** active sessions are checked against authoritative and session-observed epochs plus bitemporal revocation effective/observed times.

These helpers do not fetch tokens, discover routes, revoke credentials or intercept real protected-resource effects.

## Run

```bash
cargo fmt --manifest-path Cargo.toml --check
cargo check --locked --all-targets --manifest-path Cargo.toml
cargo test --locked --all-targets --manifest-path Cargo.toml
cargo clippy --locked --all-targets --manifest-path Cargo.toml -- -D warnings
```

**64 Rust unit tests are authored; they have not been executed in this authoring environment.** Exact-head CI remains the validation authority.

## Relationship to owner contracts

- #60, #1161, #1174 — constitutional epoch/currentness;
- #2953, #3117 — decision/authority/attempt/observation lineage;
- #1607 — partial/unknown outcomes and reconciliation-before-retry;
- #6069, #3126 — temporal reconstruction and uncertainty;
- #4835 — independent evaluator separation.

The crate is a benchmark reference slice, not an implementation of those owners.


## Metamorphic regression properties

Seven new tests exercise monotonic and temporal properties:
- changing any approval-bound operation field denies the substituted operation;
- narrowing authority cannot preserve a permit;
- adding a reachable path without the designated enforcement point breaks mediation;
- adding an active grant cannot reduce the effective privilege union;
- attenuation is transitive when every parent-child edge is valid;
- a revocation first remains unknown when not yet known, then becomes denied as knowledge advances, without rewriting the earlier view;
- removing retry-safety evidence cannot preserve a retry permit.

These are finite reference-model properties over owner-supplied data, not exhaustive proofs or live-system tests.


## Currency-dimensional amount bounds

A minor-unit amount is not a dimensionless scalar. `CapabilityScope` now binds every bounded amount to an explicit currency. The trajectory union accumulates ceilings in a currency-keyed map and preserves unbounded grants by currency, so USD 600 + EUR 600 is never treated as one 1,200-minor-unit quantity and an unbounded EUR grant cannot fit inside a USD-only task. A task with a USD ceiling rejects any additional EUR capability unless the task scope explicitly permits that currency dimension. A bounded amount missing its currency is unqualified and fails closed.

No implicit foreign-exchange conversion is performed. An owner adapter that genuinely needs cross-currency comparison must supply a separately versioned conversion policy, valuation timestamp, rate source and rounding rule; this reference model intentionally does not invent them.

Six regression tests cover per-currency aggregation, extra-currency privilege, missing currency, currency substitution across a delegation edge, and currency-scoped versus all-currency unbounded grants.
