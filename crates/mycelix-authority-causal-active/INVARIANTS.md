# Mycelix Authority Causal Active v0.1 — Normative Invariants

Status: **pure historical positive-authority qualification; not current authority**

This crate consumes only an opaque `QualifiedCausalAuthorityStateProjection` from the exact-causal projector above PR #429.

It answers one narrow question:

> Did the exact fully-covered historical causal authority coordinate resolve to `Active`, while the completeness evidence is still reusable now?

It does not discover history, select a causal coordinate, verify signatures, choose an authority source, or grant current execution authority.

## 1. Historical existence is not positive authority

A causal projection can validly resolve to `Active`, `Revoked`, or `Superseded`.

Only `Active` may produce `QualifiedActiveCausalAuthority`.

`Revoked` and `Superseded` are valid historical source truth but fail positive authority qualification.

## 2. Opaque projection input only

The qualifier accepts the non-deserializable #429 projection, not loose caller fields such as:

- subject;
- generation;
- transition digest;
- state;
- lineage digest;
- source reference; or
- verification lease.

Therefore a caller cannot assemble a positive historical capability from individually plausible values.

## 3. Full current source coverage remains inherited

#429 can only exist after #91 re-proves the complete currently covered authoritative lineage.

This theorem does not weaken that condition. A historical prefix cannot hide a later revocation, reactivation, or supersession.

Later transitions remain present in the committed full-lineage/current-head identity even when the selected historical state is older.

## 4. No wall-clock selector

This theorem has no historical `as_of` selector.

The selected historical state was already fixed by #429's exact causal coordinate:

`AuthoritySubjectRef + generation + transition digest`.

The `verification_now_ms` argument serves only to decide whether the already-proven complete-source evidence remains reusable. It cannot choose or alter the historical state.

## 5. Bounded completeness evidence remains bounded

A #429 capability carries `verified_at_ms` and `lease_until_ms` from the complete current source-coverage proof.

Positive historical authority requires:

`verified_at_ms <= verification_now_ms < lease_until_ms`

and a non-zero verification-now value.

An opaque #429 capability retained in memory after its coverage lease expires cannot be promoted into new positive historical authority. The caller/provider must refresh the complete source evidence and reconstruct #429 first.

This is evidence freshness, not authority-state selection.

## 6. Stable positive historical identity

`active_authority_digest` commits:

- exact subject identity;
- exact #429 causal projection identity;
- exact selected snapshot identity;
- exact selected transition digest;
- exact full currently covered lineage identity;
- exact covered current head generation/digest; and
- the explicit `Active` state tag.

Dynamic verification timestamps and lease horizons are carried but excluded from this stable authority identity.

## 7. Later revocation does not rewrite valid history

If generation 1 was `Active` and generation 2 later revoked it, a #429 projection anchored exactly to generation 1 can still qualify historically positive after the complete lineage including generation 2 is proven.

This historical capability cannot become current freshness or current operational authority.

## 8. Later activation cannot legitimize earlier inactivity

If the selected exact causal coordinate resolves to `Revoked` or `Superseded`, a later `Reactivate` generation does not make that older coordinate positive.

The signed causal coordinate must itself resolve to `Active`.

## 9. No live-current conversion

`QualifiedActiveCausalAuthority` exposes no conversion to `VerifiedAuthorityFreshness` or `QualifiedCurrentOperationalAuthority`.

Current execution authority remains owned by the existing current-only authority stack.

## 10. No authority-source or cryptographic ownership

This crate does not:

- verify institutional signatures;
- verify Identity time-policy signatures;
- inspect Holochain/DHT state;
- establish source coverage;
- choose current records;
- evaluate grants/rulebooks; or
- enable effects.

Those facts must already be established by their owning layers.

## 11. Cross-lineage use

A later Identity bridge may consume this capability only after proving that the signed Identity causal anchor maps to the exact generic `AuthoritySubjectRef`, generation and transition digest used by #429.

This crate alone does not make that cross-lineage mapping.
