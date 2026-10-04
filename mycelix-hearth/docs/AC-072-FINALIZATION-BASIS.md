# AC-072: Exact Decision Finalization Basis

## Purpose

AC-071 makes concurrent `DecisionOutcome` observation deterministic. AC-072 adds the missing provenance binding: each new outcome records the exact `Decision` action/version observed by the resolver.

## Contract

Every new `DecisionOutcome` carries `finalization_basis_action`.

That field is:

- the exact action hash used as the finalization basis;
- populated from the `Record` actually read by `finalize_decision`;
- required for new outcomes;
- validated as a valid `Decision` record;
- required to be in the same Decision update lineage as `decision_hash`;
- required to represent an `Open` Decision version.

Legacy outcomes may omit the field. Such an outcome is represented as `None`; no basis is inferred retroactively.

## Why the action hash matters

A Decision entry can have multiple actions in Holochain's update model. The semantic decision identity alone is therefore insufficient to say exactly which version was observed when a result was produced.

AC-072 records:

```text
decision identity
    +
exact Decision action/version observed
    +
authenticated resolver action author
    ->
auditable resolution candidate
```

## Concurrency

Two agents may still race while observing the same open Decision basis. That is not treated as an error here. Their candidate outcomes remain independently attributable and explicitly bound to their observed basis.

AC-071 then selects the canonical read candidate using:

```text
min(outcome_action_timestamp, outcome_action_hash)
```

The losing candidate is preserved rather than erased.

## Validation boundary

The integrity callback uses deterministic, hash-addressed dependencies only: `must_get_valid_record` and `must_get_action`. Holochain documents these as deterministic validation dependencies, while collection-style reads are not suitable for validation because they can change over time.

The update lineage is followed from the declared basis action back through `Update.original_action_address` until the root `Create` action. The root must equal `DecisionOutcome.decision_hash`.

## Non-goals

This does not establish:

- a distributed mutex or global lock;
- exactly-once finalization;
- that the earliest candidate is substantively correct;
- fairness of the underlying decision rule;
- legitimacy or moral correctness of the decision itself.

Those claims require separate evidence.

## Follow-on

A dedicated two-agent SweetConductor/Tryorama fixture should exercise the real race: both agents observe the same open basis, both attempt finalization, all accepted candidates expose their basis, and the canonical getter remains deterministic after synchronization.

That integration fixture is deliberately separate from this schema/integrity tranche so a passing unit/integrity build is not mistaken for proof of distributed concurrency behavior.