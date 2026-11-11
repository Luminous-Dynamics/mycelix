# CIV-TRACE-001 — Bitemporal institutional reconstruction and contestability

**Status:** protocol and synthetic fixtures; qualifier not yet executed  
**Owner:** [Mycelix #6069](https://github.com/Luminous-Dynamics/mycelix/issues/6069)  
**Parent:** [DELIB-BENCH-001 #5999](https://github.com/Luminous-Dynamics/mycelix/issues/5999)

## Research question

Can an independent reviewer reconstruct what an institution knew, what authority it believed current, what was actually observed, and what remained contestable at a past decision time—without later evidence leaking backward or historical records being rewritten?

## Corpus

The v0.1 corpus contains **36 deterministic synthetic fixtures** across six families, six fixtures each:

1. hindsight isolation;
2. authority at event time;
3. correction and supersession;
4. outcome observation;
5. contestability;
6. privacy-preserving reconstruction.

Each event carries immutable ID, kind, transaction time, valid time, exact subject/principal/target references, parent links, status, visibility, payload digest and supersession reference.

## Two distinct historical views

- **As-known-at-cutoff:** records with transaction time less than or equal to the declared cutoff. This answers what the institution had recorded by then.
- **Current assessment of past authority:** all currently available grant/revocation evidence evaluated against the historical action time. A late-recorded, retroactive revocation can change the current assessment while leaving the original as-known-at-action view unchanged.

Neither view rewrites the event history. The distinction is necessary to audit both contemporaneous decisions and later-discovered defects.

## Independent outcome evidence

An executor's `Succeeded` status is not an outcome observation. Only an `EffectObservation` linked to the exact attempt contributes to outcome state, and it must name an `observer_ref` distinct from the attempt's executor/issuer identity. Same-source observations remain `INDEPENDENCE_UNESTABLISHED`; contradictory independent observations remain `CONTESTED`; missing or unmatched observations remain `UNOBSERVED`.

## Contestability and supersession

A challenge is linked to an exact decision reference. Open challenges remain visible; resolved challenges remain in history. Decision successors use an explicit `supersedes_ref`; the predecessor is never removed.

## Metrics

The qualifier reports these independently: as-of reconstruction, authority-at-action, current reassessment, independent outcome observation, supersession, unresolved challenge visibility and protected-payload projection. No aggregate score can compensate for a failure in one of these dimensions.

## Reproduction

```bash
cargo fmt --manifest-path research/civ-trace-001/qualifier/Cargo.toml --check
cargo test --manifest-path research/civ-trace-001/qualifier/Cargo.toml
cargo run --manifest-path research/civ-trace-001/qualifier/Cargo.toml -- \
  research/civ-trace-001/CIV_TRACE_FIXTURES_V0_1.json \
  research/civ-trace-001/CIV_TRACE_FIXTURES_V0_1.schema.json
```

## Non-claims

The corpus is synthetic. Passing it qualifies only the declared temporal/oracle contract. It does not establish legal admissibility, institutional legitimacy, empirical truth, completeness of any real organization's event capture, production security, or deployment readiness.
