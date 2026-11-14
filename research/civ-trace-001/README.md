# CIV-TRACE-001 — Bitemporal institutional reconstruction and contestability

**Status:** protocol and synthetic fixtures; remedy lifecycle extension under qualification  
**Owner:** [Mycelix #6069](https://github.com/Luminous-Dynamics/mycelix/issues/6069)  
**Parent:** [DELIB-BENCH-001 #5999](https://github.com/Luminous-Dynamics/mycelix/issues/5999)

## Research question

Can an independent reviewer reconstruct what an institution knew, what authority it believed current, what was actually observed, and what remained contestable at a past decision time—without later evidence leaking backward or historical records being rewritten?

## Corpus

The v0.2 corpus contains **42 deterministic synthetic fixtures** across seven families, six fixtures each:

1. hindsight isolation;
2. authority at event time;
3. correction and supersession;
4. outcome observation;
5. contestability;
6. privacy-preserving reconstruction;
7. remedy order, authority, execution and independent observation.

Each event carries immutable ID, kind, transaction time, valid time, exact subject/principal/target references, parent links, status, visibility, payload digest and supersession reference.

## Two distinct historical views

- **As-known-at-cutoff:** records with transaction time less than or equal to the declared cutoff. This answers what the institution had recorded by then.
- **Current assessment of past authority:** all currently available grant/revocation evidence evaluated against the historical action time. A late-recorded, retroactive revocation can change the current assessment while leaving the original as-known-at-action view unchanged.

Neither view rewrites the event history. The distinction is necessary to audit both contemporaneous decisions and later-discovered defects.

## Independent outcome evidence

An executor's `Succeeded` status is not an outcome observation. Only an `EffectObservation` linked to the exact attempt contributes to outcome state, and it must name an `observer_ref` distinct from the attempt's executor/issuer identity. Same-source observations remain `INDEPENDENCE_UNESTABLISHED`; contradictory independent observations remain `CONTESTED`; missing or unmatched observations remain `UNOBSERVED`.

## Contestability and supersession

A challenge is linked to an exact decision reference. The output retains the exact challenge event IDs; open challenges remain visible and resolved challenges remain in history. Decision successors use an explicit `supersedes_ref`; the output retains the complete predecessor→successor reference chain, and the predecessor is never removed.

## Remedy lifecycle extension

The new `RemedyOrder` event links an exact challenged decision and challenge to a remedy order. Its issuer must hold a matching `AuthorityGrant` over the exact remedy-order subject, principal and target at the order time. A `CorrectiveAction` is recorded separately as Open, Verified or Ineffective.

The new six-case family distinguishes: ordered but not started; attempt succeeded without observation; independently observed completion; partial observation; contradicted observation; and reviewer lacking order authority. A seventh case tests a late-recorded, retroactive revocation of the remedy reviewer's authority: as-known-at-order remains authorized while current reassessment becomes unauthorized. The fixture explicitly keeps remedy authority and remedy-effect state separate.

`COMPLETED_OBSERVED` requires an EffectObservation linked to the exact remedy Attempt, with observer identity distinct from the Attempt issuer. An order, executor success, administrator closure, or partial/contradicted observation cannot be promoted into completed remedy.

## Metrics

The qualifier reports these independently: as-of reconstruction, authority-at-action, current reassessment, independent outcome observation, supersession, unresolved challenge visibility, protected-payload projection, remedy-order authority, remedy completion and systemic corrective-action status. No aggregate score can compensate for a failure in one of these dimensions.

## Reproduction

```bash
cargo fmt --manifest-path research/civ-trace-001/qualifier/Cargo.toml --check
cargo clippy --all-targets --manifest-path research/civ-trace-001/qualifier/Cargo.toml -- -D warnings
cargo test --manifest-path research/civ-trace-001/qualifier/Cargo.toml
cargo run --manifest-path research/civ-trace-001/qualifier/Cargo.toml -- \
  research/civ-trace-001/CIV_TRACE_FIXTURES_V0_2.json \
  research/civ-trace-001/CIV_TRACE_FIXTURES_V0_2.schema.json
```

## Non-claims

The corpus is synthetic. Passing it qualifies only the declared temporal/oracle contract. It does not establish legal admissibility, institutional legitimacy, empirical truth, completeness of any real organization's event capture, production security, or deployment readiness.


## Remedy claim boundary

This extension qualifies only the synthetic event sequence and typed distinctions. It does not decide whether a remedy is substantively adequate, accessible, rights-compatible or sufficient to repair harm. Those are evaluated by IPB-REM-001 (#6145) and require human/domain review in any field study. CIV-TRACE remains the owner of the temporal event history and native decision/challenge records.
