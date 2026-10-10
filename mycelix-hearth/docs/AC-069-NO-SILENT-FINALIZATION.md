# AC-069: Prevent Silent or Zero-Weight Decision Finalization

## Purpose

A decision system must not manufacture a substantive result when no substantive choice was recorded.

The prior helper behavior allowed:

```
empty tally
  -> winning_option(empty) == 0
  -> DecisionOutcome(option 0)
```

That makes the absence of a positive-weight choice observable as a substantive choice.

## Enforced boundary

The Hearth Decisions coordinator now:

- requires at least one positive-weight option before `finalize_decision` creates a `DecisionOutcome`;
- treats consensus as non-vacuous;
- allows zero-weight votes to remain historical participation without allowing them to select an option;
- closes a decision with only zero-weight votes without creating a substantive outcome.

The pure `winning_option` helper retains its deterministic fallback for utility callers, but finalization no longer treats that fallback as evidence.

## Semantic invariant

```
silence != consent
zero-weight participation != substantive choice
empty tally != consensus
```

This directly hardens the AC-066 epistemic-participation boundary.

## Compatibility

No role weights, consciousness-weight composition, quorum rules, decision types, vote amendment history, AC-064 self-governance privacy semantics, or AC-066 participation states are changed.

## Validation

Regression coverage includes:

- empty tally is not consensus;
- all-zero-weight tally is not consensus;
- a positive-weight option permits consensus;
- zero-voter scenarios do not produce consensus;
- zero-weight roles cannot manufacture a substantive choice.

## Qualification boundary

A passing validation establishes only the no-silent-substantive-outcome invariant. It does not prescribe a universal participation threshold or preferred voting model.
