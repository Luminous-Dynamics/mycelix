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

The first output is candidate-visible input only and is defined by [input.schema.json](./schemas/input.schema.json). It uses opaque `CASE-0000` identifiers and omits control/mutant labels, mutation names and expected decisions. The second output is the evaluator-only expected-answer file defined by [oracle.schema.json](./schemas/oracle.schema.json). The third output is the self-conformance report defined by [report.schema.json](./schemas/report.schema.json). Never supply oracle.json or the report to a candidate.

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

A mutation is counted as caught only when the reference returns the exact manually authored expected decision, including the correct Deny-versus-Hold reason. Positive controls are excluded from the mutant denominator. This is an **input-mutation conformance suite**, not classical code-mutation testing of a candidate implementation. Its expected answers are maintained in a separate oracle artifact and code path in this crate, but they have not been independently adjudicated by a second domain team. A 100% catch rate means only that this finite named mutation set matches its authored oracle; it is not a production safety rate, hidden-holdout result or claim of SOTA superiority.

The output manifest serializes the actual owner-resolved reference inputs but contains no expected decision labels. The oracle is generated into a different output file by a separate code path. The lab tests the LISE reference, not an external vendor or agent system.


## Reproducibility

The generator is deterministic and versioned. Run it twice and compare the three output files byte-for-byte; record the generator source commit and use `sha256sum candidate-input.json oracle.json report.json` to preserve artifact digests. The input and oracle schemas are separate by design. The generator does not consume a random seed, contact external systems or mutate a provider.
