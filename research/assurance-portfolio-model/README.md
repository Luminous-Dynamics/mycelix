# Mycelix assurance portfolio composition model v0.1

A dependency-free finite-state research model for composing claim-specific evaluator results without majority voting, trust scores, or authority escalation.

## Why this exists

An evaluator can assess a bounded claim. It cannot make its own result true by being repeated, copied, signed, or combined with other evaluators that share the same failure domain.

The model makes the following distinctions executable:

- evaluator outcome: Supports, Refutes, Indeterminate, OutOfScope, Unavailable;
- evidence validity: Current, Stale, Revoked, Invalid;
- claim scope: exact claim identifier plus exact subject digest;
- evaluator identity: exact evaluator and evaluator-profile digests;
- corroboration lineage: explicit identities for implementation, repository control, execution provider, runner image, runtime, input acquisition, operator control, and trust root;
- policy: required evaluator roles and explicitly required pairwise fault-domain separation.

No dimension is converted into a trust score or an independence boolean.

## Composition semantics

1. Only assessments for the exact claim and exact subject are in scope.
2. Exact duplicate assessment replay is deduplicated; replay does not increase corroboration.
3. Non-current evidence cannot support a profile.
4. A current Refutes result on a hard invariant yields HardInvariantRefuted. Additional Supports results cannot outvote it.
5. A Refutes result on a non-hard claim, contradictory results from one evaluator/profile, missing required roles, unavailable/indeterminate results, or a required shared/unknown fault domain yields Indeterminate.
6. MeetsDeclaredProfile is emitted only when every required role supplies current Supports evidence and every declared pairwise separation is observed as distinct.
7. Shared fault domains remain visible even when they are not a required separation. For example, different verifier implementations running on one CI provider are implementation-diverse but provider-correlated.

This is a portfolio-evidence result only. It is not a truth oracle, proof of model adequacy, qualification authority, authorization, execution permission, or outcome claim.

## Run

```sh
cargo fmt --manifest-path research/assurance-portfolio-model/Cargo.toml --check
cargo check --locked --all-targets --manifest-path research/assurance-portfolio-model/Cargo.toml
cargo test --locked --all-targets --manifest-path research/assurance-portfolio-model/Cargo.toml
cargo clippy --locked --all-targets --manifest-path research/assurance-portfolio-model/Cargo.toml -- -D warnings
```

The crate has no third-party dependencies and no default features. It is intentionally isolated from Mycelix's normative ASSURE kernel.

## Initial qualification panel

The 16 unit tests include:
- hard-invariant refutation with multiple supporting evaluators;
- exact replay deduplication;
- shared implementation lineage despite distinct executable digests;
- unknown required lineage;
- missing and stale required evidence;
- exact-subject substitution;
- non-hard refutation;
- contradictory outcomes from one evaluator/profile;
- shared CI provider visibility without falsely claiming provider independence;
- an exhaustive 400-state product over five evaluator outcomes and four evidence-validity states for each of two required roles;
- assessment-order invariance;
- stale hard-invariant refutation and wrong-subject refutation controls;
- repeated exact replay idempotence across five duplicate batches.

Targets are 100% expected disposition on this finite authored panel, zero false MeetsDeclaredProfile outcomes on the adversarial controls, and deterministic replay. These are fixture targets, not a claim of universal assurance correctness.
