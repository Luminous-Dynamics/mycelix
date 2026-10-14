# Causal time and long-lived federation integrity v1

Status: **ReferenceModelOnly**

This tranche separates causal order from wall-clock observations and makes long-offline revalidation explicit.

## Causal order

Events carry explicit predecessor references and a base frontier. The reference model derives:

- `Before` / `After` from explicit causal ancestry;
- `Concurrent` when closed histories share the same base frontier but neither is an ancestor of the other;
- `Incomparable` when closed histories have different bases and neither is an ancestor of the other;
- no relation when causal closure is incomplete or cyclic.

Arrival order, transport sequence, and wall-clock timestamp never establish causal order.

## Temporal evidence

`TemporalInterval` represents bounded time knowledge as an earliest/latest interval in an explicit clock domain. Different clock domains are not silently compared.

Freshness is evaluated against an explicit `FreshnessProfile` and `TemporalEvaluationReference`. If uncertainty straddles the freshness boundary, the result is `Indeterminate` rather than an optimistic currentness claim.

## Long-offline branches

A branch returning after a long partition does not regain authority because time elapsed. The branch is either currently valid under the supplied profile or requires revalidation. A branch whose authority is already non-current remains non-current.

## Protocol generations

Temporal replay is bound to the semantic environment and protocol profile that gave the event meaning. A historical event from an old protocol generation is not silently reinterpreted under a new one.

## Historical preservation

Revalidation changes current applicability; it does not rewrite historical event timestamps, provenance, branch identity, or causal ancestry.

## Symthaea boundary

Symthaea may analyze temporal patterns, delayed branches, and likely causal explanations, but those analyses remain advisory evidence. It cannot manufacture currentness, authority, or causal ancestry.

## Qualification boundary

The executable tests qualify only deterministic reference semantics. They do not establish synchronized physical clocks, real-world chronology, legal time, production federation safety, or real-world outcomes.