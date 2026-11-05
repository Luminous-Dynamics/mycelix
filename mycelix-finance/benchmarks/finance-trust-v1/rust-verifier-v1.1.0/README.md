# FIN-TRUST v1.1.0 assertion-purpose-aware verifier

Standalone offline Rust conformance verifier for the **v1.1.0** extension of FIN-TRUST-BENCH.

This crate is deliberately separate from `rust-verifier/`, which preserves the immutable v1.0.0 S0 receipt and contract.

## Run

From this directory:

```sh
cargo fmt --all -- --check
cargo test --locked --all-targets
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- --out /tmp/fin-trust-s1-report.json
```

The CLI accepts the same `--tasks`, `--task-schema`, `--oracle`, `--oracle-schema`, `--responses`, `--response-schema` and `--out` flags as the v1.0.0 verifier.

## Composition boundary

The source-binding projection follows the assertion/procedure evidence vocabulary owned by #5798 and [PR #5836](https://github.com/Luminous-Dynamics/mycelix/pull/5836): evidence class, polarity, purpose fit, source authentication, issuer authority, temporal fit, transformation state and independent origin group. Its controlled values are intentionally compatible with the separate FIN-ACC-ASSERTION-PURPOSE v1.0.0 contract.

FIN-TRUST does **not** create another generic trust score or replace the canonical assertion-purpose evaluator. It adds task-level interaction: candidate clarifications, recipient selection, per-requirement dispositions, historical knowledge-frontier resumption, exact journal projection and statement reconstruction.

## Additional v1.1 gates

- Every clarification matches the adjudicated recipient and unknown IDs, not just the question ID.
- Every evidence binding matches the evaluator's assertion-purpose profile and the exact source revision, digest, locator and recorded time.
- Every task requirement has a disposition, reason codes and evidence references.
- A positive requirement disposition is blocked unless all declared evidence obligations are met, including independent-origin count and population completeness where required.
- Evidence recorded after the knowledge frontier cannot support a historical disposition.
- Journals must cite exactly the expected source IDs, and every cited source must support a qualified requirement in the requested entity/period/currency frontier.
- Population scope and control totals remain explicit; absent is not zero.

## Dataset boundary

v1.1.0 is still a **public ten-task seed**, not the 50-case S0 acceptance target and not a blind model leaderboard. The evaluator oracle and reference responses are public. Blind candidate/evaluator isolation, matched arms and run manifests remain owned by #5799.

Passing this suite proves only these declared synthetic behaviors; it is not an audit, professional assurance opinion, ISA/IFRS/GAAP compliance result, tax conclusion, proof of authentic sources or proof of complete books.
