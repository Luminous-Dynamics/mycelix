# AC-068: Hearth Decision Authorship and Outcome Provenance

## Purpose

Hearth Decisions already performs coordinator-level checks for roles, decision state, and voting flow. The integrity layer must also bind the identity claimed by an entry to the Holochain action that created it.

The core invariant is:

```
declared actor == authenticated Holochain action author
```

not merely:

```
coordinator says actor is valid
```

## Enforced bindings

The integrity zome now rejects:

- a `Decision` whose `created_by` differs from the creating action author;
- a `Vote` whose `voter` differs from the creating action author;
- a new `DecisionOutcome` whose `resolved_by` is missing or differs from the creating action author.

The coordinator explicitly populates `resolved_by` from its authenticated agent identity. Legacy outcomes may decode with an unknown resolver for schema-compatibility, but new outcomes cannot omit the resolver.

## Why this matters

A coordinator function is one execution path, not the entire integrity boundary. Holochain validation must remain useful against malformed or adversarially authored entries presented outside the expected coordinator flow.

This hardening separates:

```
entry-declared identity
+
action-authenticated identity
```

and requires them to agree.

## Compatibility

This tranche does not change:

- vote weighting;
- quorum semantics;
- decision-type semantics;
- DecisionStatus transitions;
- vote amendment behavior;
- AC-064 reflective privacy contracts;
- AC-066 abstention, dissent, and reconsideration semantics.

Historical entries remain append-only. No history is rewritten.

## Tests

Focused tests cover:

- valid Decision creator binding;
- mismatched Decision creator rejection;
- valid Vote voter binding;
- mismatched Vote voter rejection;
- valid outcome resolver binding;
- mismatched outcome resolver rejection.

Existing outcome fixtures now include the explicit resolver identity.

## Qualification boundary

A passing validation establishes only the action-to-entry provenance invariant described above. It does not establish real-world identity, personhood, legal authority, or moral authority.
