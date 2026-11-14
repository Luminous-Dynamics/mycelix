# LARP Contracts Reference

An isolated, dependency-light Rust reference model for the LARP v2 institutional-contract conformance suite.

## Scope

This crate implements a narrow typed contract model and deterministic evaluator over explicit, owner-resolved facts. It includes:
- five-valued dispositions: Permit, Deny, Hold, Indeterminate and ChallengeRequired;
- explicit evaluation modes and time input;
- local-authority, constitutional-currentness, purpose and exact-operation checks;
- challenge/remedy requirements for high-impact contracts;
- preservation of unknown/partial execution outcomes;
- historical-evidence reconstruction separate from current freshness;
- bounded delegation attenuation across action, resource, purpose, field, time, amount and operation count;
- a small contract lifecycle state machine and exact amendment digest binding;
- JSON fixture/oracle types with unknown-field rejection.

## Non-goals

This is not a production authorizer, credential issuer, constitution registry, generic policy engine, legal interpreter, provider client, treasury or execution system. It does not authenticate the owner-resolved facts, sign receipts, call external systems, or mutate protected resources. A Permit is only the result of this finite reference relation over supplied inputs; it is not permission to execute a real operation.

LARP does not replace Cedar, OPA, Zanzibar-style relationship authorization, DMN/BPMN, local law or institution-native processes. Its contract model describes obligations, rights, evidence, remedies and lifecycle. LISE and owner systems remain responsible for authority and effects.

## Run

```sh
cargo fmt --check
cargo check --all-targets
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
```

The conformance inputs and their separate reference oracle are copied into `fixtures/` from the public LARP v2 research proposal. Expected dispositions are not embedded in the scenario inputs.

## Domain contract vertical slices

The crate now also includes pure, owner-input evaluators for two historical LARP contract families:

- **Epistemic Bounty v2:** contestable scientific reward eligibility, reviewer conflict/dependency assessment, honest retraction, appeal/remedy, exact payout binding and separate treasury authorization.
- **Restoration v2:** local mandate and consent, multi-dimensional ecological floors, downstream externalities, source independence, measurement freshness/method bridge, partial outcomes, steward exit and separate treasury authorization.

Each domain has its own 13-case input suite and separate oracle:
- `fixtures/epistemic_scenarios.json` / `fixtures/epistemic_oracle.json`
- `fixtures/restoration_scenarios.json` / `fixtures/restoration_oracle.json`

The three conformance suites therefore contain 55 synthetic cases in total (29 general contract, 13 Epistemic Bounty, 13 Restoration). The domain evaluators do not execute payouts, sanction reviewers, establish scientific truth, certify restoration or replace local ecological methods.

The schemas and documentation copies are kept beside the benchmark inputs. Cross-file scenario/oracle parity is checked in Rust tests; a full standards-compliant JSON Schema validator remains a separate validation requirement.

## Current assurance status

Research reference only. **30 Rust tests are authored (6 unit + 24 integration), but have not been executed or compiled in this authoring environment.** Exact-head CI is the validation authority. The finite fixtures do not establish universal security, legal enforceability, scientific truth, ecological field effectiveness, adapter correctness or production readiness. Do not claim superiority over other policy/contract systems from this crate alone.
