# Constitutional Closure Coverage — Rust ↔ TLA+ Refinement Crosswalk v0.1

Status: experimental refinement contract for MYC-CONST-003B3A / #1443.

This document maps the lifecycle-facing Rust adapter in `constitutional-closure-coverage`
to the parent `ConstitutionalEvidenceClosure.tla` model. It is not itself a model-check result.
A dedicated exact-head qualification lane must bind this file and the Rust implementation before
these mappings are used as production evidence.

## State correspondence

| TLA+ parent state | Rust parent state | Closure-coverage result |
| --- | --- | --- |
| `fault = TRUE` | `TemporalEvidenceState.integrity_fault.is_some()` | `IntegrityFault` |
| `fault = FALSE`, `R = 1` | healthy temporal state, revocation effective seq 1 | `EmptyPreRevocationInterval` |
| `fault = FALSE`, `R > 1`, `closureThrough < R - 1` or `closureThrough = 0` | no accepted latest closure covering `R-1` | `Open` |
| `fault = FALSE`, `R > 1`, `closureThrough >= R - 1` | accepted latest closure covering `R-1` | `Closed { exact EvidenceClosure, ... }` |

## Dominance rule

Fault status is evaluated before any historical closure or `R=1` shortcut.

Formally, lifecycle authorization is intentionally stronger than the historical condition
`closureThrough >= R - 1`:

`LifecycleClosed(R) == ~fault /\\ (R = 1 \/ closureThrough >= R - 1)`

The Rust adapter refines this by returning the exact closure object whenever `R > 1` is closed.
A historical closure retained after a contradiction therefore remains evidence but is not current
authority to terminalize a claim.

## Effective order vs observation order

The coverage decision compares only values in the finality/effective-order domain:

`closure.closed_through_effective_seq >= revocation_effective_seq - 1`

The verifier observation counter is irrelevant to that precedence decision. This is true in both
parent order modes:

- `SharedComparable`: effective and observed order happen to be normalized into one comparable
  constitutional order, but lifecycle coverage still uses effective order only.
- `Independent`: numeric cross-comparison between effective and observation counters is forbidden;
  coverage remains well-defined because closure and revocation effective sequence are in the same
  finality domain.

## DetectionOnly

In the parent TLA+ model, `Profile = "DetectionOnly" => closureThrough = 0`.
Therefore for a healthy DetectionOnly domain:

- `R = 1` may yield `EmptyPreRevocationInterval` because there is no positive earlier interval;
- `R > 1` cannot yield `Closed` without changing the parent finality/closure policy.

A timeout, DHT query miss, `must_get_*` miss, or absence of a visible competing record is not a
refinement of `closureThrough` and must not be converted into `Closed` by an adapter.

## Concrete provenance refinement

The TLA+ parent is deliberately a quotient-by-effective-sequence model for interval/closure safety.
It does not represent concrete closure IDs, witness identities, proof references, policy-version
strings, or multiple evidence records sharing one effective height.

Rust therefore carries additional provenance that is not erased by the abstraction:

- exact `EvidenceClosure` object;
- `closure_id` and previous-closure lineage;
- finality domain ID;
- policy version;
- observation domain ID and `CrossOrderRelation`;
- concrete `TemporalIntegrityFault` details.

A `Closed` lifecycle decision must preserve this concrete object. Merely copying the integer
watermark is insufficient for future `ClaimResolutionReceipt` work.

## Properties owned by this adapter

The adapter adds/refines these obligations beyond the raw parent model:

1. active integrity fault always dominates historical closure authority;
2. boolean compatibility helper is fail-closed for `Open` and `IntegrityFault`;
3. healthy-domain closure truth table is exactly `R == 1 || closureThrough >= R - 1`;
4. returned `Closed` state retains the exact accepted closure object and policy provenance;
5. Independent observation order cannot change effective-interval coverage.

## Properties not established here

This crosswalk does not prove:

- authenticity of witness signatures or consensus checkpoints;
- completeness of any Holochain DHT view;
- concrete evidence-identity multiplicity properties omitted by the quotient TLA+ model;
- claim lifecycle monotonicity (`PendingExecutable`, `RejectedConflict`, `RevokedClosed`, etc.);
- distributed runtime/Holochain refinement;
- recovery/remedy after a temporal integrity fault.

Those remain separate evidence obligations in #1427, #1333 and later runtime tranches.
