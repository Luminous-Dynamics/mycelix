# hearth-automation-replay

Deterministic, side-effect-free qualification harness for Hearth household automation.

HTH-AUTO-004B deliberately reuses the production `hearth-edge` executor. It does **not** implement a second automation engine. A scenario supplies scripted inputs to the real execution kernel:

- deterministic clock start and sleep progression,
- authority snapshots in request order,
- predicate results in evaluation order,
- synthetic adapter command responses,
- verification responses,
- cancellation timing,
- trigger evidence.

The same scenario can then be run twice and compared for exact semantic equality of the resulting receipt, trace, clock progression, and request counters.

## Primary invariant

> Same validated scenario -> same execution result.

This gives Hearth a qualification lab for failure cases before household automations are allowed to touch physical devices.

## Fail-closed behavior

A scenario must include at least one authority snapshot. Predicate scripts may be intentionally incomplete; exhaustion becomes `Unknown`, which the AUTO-003 executor inhibits rather than guessing.

Authority scripts persist their last snapshot once the scripted sequence is exhausted. This makes changes such as capability revocation easy to model deterministically:

```text
snapshot 1: capability present
snapshot 2: capability present
snapshot 3: capability removed
snapshot 3 remains active thereafter
```

## What this can qualify now

- dependency ordering,
- retries/backoff and clock advancement,
- authority expiry/revocation between steps,
- manual override or other policy inhibition,
- unknown precondition behavior,
- command/verification separation,
- cancellation timing,
- completed-receipt invariants,
- reproducibility across repeated runs.

## What comes later

This tranche does not yet replay a full household event log or infer device effects. Later work can add:

- historical Care/Rhythms timelines,
- state-transition and domain-event trigger replay,
- richer virtual household effect models,
- cognitive-load and fairness counterfactual metrics,
- crash-cut matrices around the AUTO-004A journal,
- scenario corpus artifacts for release qualification.

Those layers should build on this harness rather than creating separate execution semantics.