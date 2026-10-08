# SOV-AI-AUTH-024 — bounded constraint algebra

This research artifact makes the policy semantics denotational before attempting full formal proof.

## Semantic objects

A capability constraint is a conjunction of typed dimensions:

- operation
- target
- audience
- currency
- argument class
- numeric interval
- temporal interval
- purpose
- context
- extension type

A policy is a disjunction of allow-constraints plus an explicit deny-constraint set.

## Three distinct relations

`syntax equality` compares representation.

`semantic equivalence` compares authorization denotation over the bounded request universe.

`authorization subsumption` requires the child denotation to be contained by the parent denotation and refuses unsupported extension types.

These are intentionally not interchangeable.

## Deny semantics

The policy algebra models:

`Effective = Allow  Deny`

under the canonical `deny-overrides` rule.

Attenuation therefore requires both:

`Allow(child) ⊆ Allow(parent)`

and

`Deny(parent) ⊆ Deny(child)`

A child can add new denies; it cannot erase a parent deny and still claim non-expansion.

## Boundedness

The reference universe is finite and deliberately small. This demonstrates decidability of the modeled language, not decidability of arbitrary policy languages.

The next formal layer should mirror this exact semantic definition in TLA+ and Alloy, with the same control matrix and fail-closed handling of unsupported extensions.

Research/specification only. No production authorization, cryptographic trust, legal authority, or complete policy-language coverage is established.
