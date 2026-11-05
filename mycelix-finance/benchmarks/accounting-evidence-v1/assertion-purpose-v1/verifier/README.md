# FIN-ACC assertion-purpose reference verifier

Run from this directory:

```sh
cargo fmt --all -- --check
cargo test --locked
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- --out /tmp/fin-acc-assertion-purpose-report.json
```

The CLI accepts `--input`, `--schema`, `--oracle`, `--oracle-schema` and `--out` overrides.

The evaluator is a public 21-case synthetic reference implementation of the declared assertion/procedure obligations. It is not an auditor, does not judge audit sufficiency, and does not claim ISA compliance. The public oracle is a regression oracle, not a blind holdout.
\nThe initial corpus contains 21 candidate-visible cases and a separately stored public regression oracle. The oracle is not hidden evaluation.\n

The qualification workflow runs on branch pushes and pull requests. The branch-push lane is used for direct exact-head qualification; a GitHub PR workflow result marked `action_required` is not a pass.
