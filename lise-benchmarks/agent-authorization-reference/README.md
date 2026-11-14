# Agent Authorization Claim Reference

Pure Rust reference evaluator for a finite, claim-scoped authorization/effect contract.

- Task outcome reports whether the task succeeded, failed or remained incomplete.
- Authorization disposition reports permit, deny, hold, indeterminate or clarification required.
- Effect state keeps committed, partial, unknown, confirmed-no-effect and not-attempted distinct.

A successful task never upgrades an unauthorized action. A failed task does not automatically imply an authorization violation.

The 22 synthetic cases cover clarification, structural parent/child delegation attenuation, revocation, expiry, epoch drift, enforcement bypass, operation substitution, collateral effects, failed tasks, executor-only acknowledgement, unknown/partial effects, replay, amount ceilings and observation mismatch. The input panel and separate oracle are stored independently. Strict serde parsing rejects unknown fields.

This evaluator does not authenticate owner authority, enforce a real tool path, observe a real provider or perform an effect. It has no credential, signer, treasury or network capability.

## Run

```sh
cargo fmt --check
cargo check --all-targets
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
```

**11 unit and 6 integration tests are authored; they have not been compiled or executed in the authoring environment.** Exact-head CI is required before reporting a Rust pass.

A finite fixture pass is not a security guarantee, certification or SOTA superiority claim. Comparative results require matched task semantics, versions, sandbox visibility, oracle rules, repeated runs and full denominators.
