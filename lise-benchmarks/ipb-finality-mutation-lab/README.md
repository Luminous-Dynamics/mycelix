# LISE typed mutation lab

Deterministic adversarial mutation runner for the standalone IPB-FINALITY reference relation.

## Run

```sh
cargo fmt --manifest-path lise-benchmarks/ipb-finality-mutation-lab/Cargo.toml --check
cargo check --locked --all-targets --manifest-path lise-benchmarks/ipb-finality-mutation-lab/Cargo.toml
cargo test --locked --all-targets --manifest-path lise-benchmarks/ipb-finality-mutation-lab/Cargo.toml
cargo run --locked --manifest-path lise-benchmarks/ipb-finality-mutation-lab/Cargo.toml -- \
  candidate-input.json oracle.json report.json
```

The first output is candidate-visible input only. The second output is the evaluator-only expected-answer file. The third output records the reference's observed decisions, expected decisions, mutation-kill counts, false permits and denominators. Never supply oracle.json to a candidate.

## Mutation families

- ten one-field exact-operation substitutions;
- malformed exact-operation/constitution bindings;
- denied, revoked, historical, pending and unknown approval states;
- revoked, expired, not-yet-valid and unknown authority;
- constitution epoch/digest drift and policy revision drift;
- changed and unknown resource preconditions;
- unknown, partial and already-applied prior effects;
- confirmed-no-effect retry with unknown, unsupported and qualified same-key safety;
- pairwise and three-way compositions that check decision precedence.

All mutations are deterministic, explicitly named and applied to a frozen typed baseline. There is no random seed and no network or external-effect capability.

## Interpretation

A mutation is killed only when the reference returns the exact preregistered decision, including the correct Deny-versus-Hold reason. Positive controls are excluded from the mutant denominator. A 100% kill rate means only that this finite named mutation set was caught by this reference relation; it is not a production safety rate, hidden-holdout result or claim of SOTA superiority.

The output manifest serializes the actual owner-resolved reference inputs but contains no expected decision labels. The oracle is generated into a different output file by a separate code path. The lab tests the LISE reference, not an external vendor or agent system.
