# Temporal Provenance Abstraction Notes

## Two different ordering questions

The temporal-provenance layer distinguishes:

1. **effective order** — order inside the finality/evidence domain in which a proof or revocation became constitutionally effective;
2. **observation order** — strict order in which this verifier accepted or retained distinct evidence events.

These are not automatically the same coordinate system.

## Cross-order policy

`CrossOrderRelation::SharedComparable` means a runtime has normalized both values into one authenticated constitutional order. In that mode the reference model requires:

`effective_seq <= observed_seq`

and the observation-order domain identifier must equal the finality/effective-order domain identifier.

`CrossOrderRelation::Independent` means the two counters belong to different domains—for example, an external consensus height and a verifier-local intake counter. In this mode their numeric values MUST NOT be compared to infer causality. The verifier still requires its own observation order to increase strictly for distinct evidence events.

The ordinary `TemporalEvidenceState::new(policy)` constructor deliberately chooses `SharedComparable` as the conservative default. Runtimes with incomparable domains must opt in explicitly with `new_with_observation_order(...)` rather than silently weakening the ordering contract.

## Observation order is strict for distinct evidence

Distinct newly accepted/retained evidence events must use an observation value strictly greater than the state's previous observation sequence. Exact duplicate replay is checked before that rule and remains idempotent: byte-equivalent replay returns the corresponding `AlreadyObserved` outcome without advancing observation order.

## Effective order may have multiplicity

A finality domain may legitimately place multiple distinct evidence records at the same effective height/round/epoch. The Rust model therefore does not require effective-sequence uniqueness.

`ConstitutionalEvidenceClosure.tla` currently abstracts evidence primarily by effective sequence rather than concrete evidence identity. It is a quotient abstraction for interval/closure properties, not a proof about multiplicity, duplicate IDs, or every identity-level race. Those obligations remain owned by Rust tests until #1333's refinement crosswalk either proves the abstraction sufficient for a claim or introduces explicit evidence identities into the formal model.

## TLA+ modes

The companion model has explicit `OrderMode` values:

- `SharedComparable`
- `Independent`

Canonical safety is checked under both kinds of ordering. A named independent-order reachability profile must find a history where accepted finality has `effective > observed` numerically. That history is forbidden in SharedComparable mode but intentionally legal in Independent mode because the integers are coordinates from different order domains.

No evidence statement should describe the TLA+ closure model as proving uniqueness of concrete finality records at one effective sequence or as establishing a total order across independent domains.
