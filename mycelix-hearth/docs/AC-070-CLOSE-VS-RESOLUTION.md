# AC-070: Separate Decision Closure from Substantive Resolution

## Purpose

A decision lifecycle should distinguish ending a process from recording a substantive result.

The invariant is:

```
Open -> Closed    = process ended without resolution
Open -> Finalized = process ended with a substantive outcome
```

## Previous ambiguity

The `close_decision` coordinator previously created a `DecisionOutcome` snapshot when votes existed.

That overloaded `DecisionOutcome`: a caller could observe an outcome even though the decision status was `Closed`, not `Finalized`.

It also allowed closure to bypass the finalization path's dedicated checks for deadline, resolver authorization, quorum, and substantive positive-weight choice.

## Enforced boundary

`close_decision` now:

- changes the decision status to `Closed`;
- preserves the existing vote entries and history links;
- emits the existing `DecisionClosed` signal;
- does not create a `DecisionOutcome`.

Only `finalize_decision` creates `DecisionOutcome`.

## Semantic separation

```
votes           -> participation evidence
Closed          -> process termination
Finalized       -> substantive resolution
DecisionOutcome -> resolution evidence
```

A closed decision can therefore have a complete participation history while intentionally having no substantive outcome.

A later successor decision can carry reconsideration or changed evidence without rewriting the closed decision.

## Compatibility

No vote weighting, quorum configuration, decision type, amendment history, AC-064 privacy contract, or AC-066 participation semantics are changed.

## Qualification boundary

Validation establishes only the closure-versus-resolution lifecycle invariant. It does not establish a universal rule for when a successor decision should be opened.
