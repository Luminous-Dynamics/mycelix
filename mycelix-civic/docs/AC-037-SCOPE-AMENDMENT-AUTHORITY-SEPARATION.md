# AC-037 — Scope Amendment Authority Separation

## Purpose

AC-030 prevents an action from omitting dimensions that are already known to have impacts.

AC-035 binds the scope across the action lifecycle.

AC-036 binds the lifecycle to the scope's actual content.

AC-037 closes the remaining preemptive narrowing path: a scope can be amended before an omitted dimension has generated a recorded impact.

## Rule

A scope amendment that reduces required substrate coverage must use a different authority reference from the previous scope's authority.

This is intentionally a reference-level anti-self-relaxation rule.

It does not claim that a different identifier proves organizational independence. Deployment policy must still establish what counts as an independent authority.

## Why reduction is special

Adding required dimensions is conservative: the action becomes subject to more inspection.

Removing required dimensions is potentially permissive: the action becomes subject to less inspection.

Therefore reduction deserves stronger separation.

The model deliberately does not attempt to classify every change to policy text, purpose, or metadata as tightening or relaxation. Such semantic judgments belong to local governance.

## Verification

Before an amendment is accepted:

- the previous scope must match the active scope ID;
- the previous scope must match the active scope fingerprint;
- the new scope must retain the stable action reference;
- the new scope must have a new scope ID;
- coverage reduction is detected using set semantics;
- a coverage-reducing amendment cannot use the previous scope authority;
- terminal actions cannot be amended.

## Research basis

This pattern is consistent with the broader anti-corrosion design already used by AC-019 for governed boundary relaxation: changes that weaken constraints should receive an explicit governance path rather than being smuggled through ordinary state updates.

It also aligns with OCDS's approach of representing material process changes explicitly as amendments with dates and rationale, while preserving the prior release history.

References:

- https://standard.open-contracting.org/latest/en/guidance/map/amendments/
- https://standard.open-contracting.org/latest/en/primer/releases_and_records/

## Security contribution

The progression is now:

`known-impact omission -> blocked`

`preemptive scope narrowing -> explicit amendment`

`coverage-reducing amendment -> different authority required`

This reduces the ability of one authority to gradually relax the set of constraints against which its own economic actions are evaluated.
