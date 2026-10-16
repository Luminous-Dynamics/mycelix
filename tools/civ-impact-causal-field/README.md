# CIV-IMPACT causal field artifact verifier v0.1

This standalone Rust crate checks whether a frozen target-trial plan, assignment manifest, and outcome ledger are internally consistent. It is an artifact gate, not a statistical inference engine or field-study platform.

## Commands

```sh
cargo test --manifest-path tools/civ-impact-causal-field/Cargo.toml
cargo run --manifest-path tools/civ-impact-causal-field/Cargo.toml -- validate-plan path/to/trial-plan.json
cargo run --manifest-path tools/civ-impact-causal-field/Cargo.toml -- verify-trial path/to/trial-plan.json path/to/assignment-manifest.json path/to/outcome-ledger.json
```

The verify-trial command validates all three files against the repository schemas; verifies the exact assignment-manifest byte digest named by the plan and the exact plan/assignment byte digests repeated by the outcome ledger; checks trial identity, unique units, arm coverage, cluster assignment integrity, complete outcomes/dispositions, observed-versus-missing value consistency, and the preregistered missingness ceiling.

A successful report means only **artifact and design-gate consistency**. It does not prove that the allocator actually randomized, that an attestation is authentic, that endpoints are valid, that there is no unmeasured confounding or interference, or that the estimated effect is statistically significant. Confirmatory interpretation requires independent allocator authentication, a justified estimator and sample-size/precision plan, domain/ethics review, and exact-head test evidence.

## Claim discipline

- Randomized designs are eligible for review as confirmatory candidates only when the protocol is frozen and required assignment/analysis/interference gates are present.
- Observational and quasi-experimental designs are descriptive-only in v0.1; causal estimates from such designs require a separate identification strategy and review.
- Missing, withdrawn, unmeasured, and non-compliant units remain visible and cannot be silently removed from the assignment census.
- This tool does not estimate treatment effects and never prints a claim that an intervention caused improvement.
- Hashes bind byte sequences; without signatures and an independently retained transparency checkpoint, hashes alone do not authenticate who created the artifact or when it was frozen.
