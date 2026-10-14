# AC-075: Deterministic Decision Lifecycle Precedence

## Purpose

Concurrent agents can derive different terminal Decision revisions from the same Open basis. AC-075 makes the lifecycle conflict semantics explicit.

## Canonical precedence

```text
Finalized > Closed > Open
```

`Finalized` represents a substantive resolution. `Closed` represents process termination without substantive resolution. `Open` represents an unresolved process.

When concurrent revisions disagree on lifecycle state, lifecycle rank is considered first. Only revisions in the same lifecycle class are ordered by:

```text
(action timestamp, raw action hash)
```

All revisions remain preserved.

## Why this matters

Suppose two agents concurrently observe the same Open Decision:

```text
Agent A -> Closed
Agent B -> Finalized + DecisionOutcome
```

Without an explicit precedence rule, a later administrative close could hide the substantive result. AC-075 makes the visible Decision state converge on `Finalized` whenever a valid concurrent finalization exists.

## Limits

This is an application lifecycle conflict rule. It does not establish distributed locking, exactly-once finalization, fairness, or substantive correctness of the chosen option.

AC-071 continues to canonicalize competing `DecisionOutcome` records independently.
AC-072 continues to bind each outcome to its exact finalization basis.
AC-073 continues to resolve current Decision revisions deterministically.