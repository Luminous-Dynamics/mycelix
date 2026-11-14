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

## Current assurance status

Research reference only. The finite fixtures do not establish universal security, legal enforceability, field effectiveness, adapter correctness or production readiness. Do not claim superiority over other policy/contract systems from this crate alone.
