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

It does not verify the authenticity or truth of its input references. Owner-native adapters must supply those facts and exact source references. It does not hash application payloads: the canonical operation digest is supplied by the domain owner, and this crate checks exact equality only.

The Permit result means only that this finite reference relation found no mismatch in its supplied inputs. It does not create a production permit or authorize a real effect. The caller still needs the authoritative owner execution path and any independently qualified effectuation controls.

Unknown authority, unknown resource state, unknown prior outcome, incomplete operation binding and partial effects do not become permission. A retry after a confirmed no-effect attempt is permitted by this model only when same-operation/same-idempotency-key safety is explicitly qualified.

## Run

```bash
cargo fmt --manifest-path Cargo.toml --check
cargo check --locked --all-targets --manifest-path Cargo.toml
cargo test --locked --all-targets --manifest-path Cargo.toml
cargo clippy --locked --all-targets --manifest-path Cargo.toml -- -D warnings
```

## Relationship to owner contracts

- #60, #1161, #1174 — constitutional epoch/currentness;
- #2953, #3117 — decision/authority/attempt/observation lineage;
- #1607 — partial/unknown outcomes and reconciliation-before-retry;
- #6069, #3126 — temporal reconstruction and uncertainty;
- #4835 — independent evaluator separation.

The crate is a benchmark reference slice, not an implementation of those owners.


## Authorization assurance extensions

The reference model also includes bounded synthetic checks for:
- child/descendant delegation attenuation across action, resource, purpose, field, validity, amount and operation-count dimensions;
- trajectory-wide union of all active grants, including ambient and tool/session grants;
- complete mediation across every reachable route to a protected effect, with incomplete route inventory represented separately;
- revocation propagation to active sessions using the authority epoch, session-observed epoch, effective time and knowledge time.

These are comparison helpers over owner-resolved fixture inputs. They do not fetch tokens, discover live routes, revoke credentials or intercept real protected-resource effects.

Additional tests are authored for each property. They have not been compiled or executed in this authoring environment.
