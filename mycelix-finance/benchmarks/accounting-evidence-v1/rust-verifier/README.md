# FIN-ACC reference verifier (development draft)

This standalone crate opts out of the enclosing `mycelix-finance` workspace via its own empty `[workspace]` table. This keeps the benchmark verifier independent of Holochain/WASM workspace membership and dependency resolution. It is the first independent verifier for the three synthetic cases in
`../smoke_corpus_v0.1.0.json`. It is intentionally outside the Holochain/WASM workspace:
the reference calculation does not need a conductor, private integration dependencies,
Bevy, Python, hosted models or network access.

## Current scope

- Full JSON Schema Draft 2020-12 validation of the corpus.
- Canonical UTF-8 JSON / SHA-256 verification for each nested source-document content object.
- Strict non-negative integer-string parsing and checked integer arithmetic.
- Document identity, duplicate/replay, entity, currency-profile and policy-reference checks.
- Seed-specific purchase-order / goods-received / invoice and bank-settlement disposition checks.
- Independent closing trial-balance and simple statement-total reconstruction from opening balances
  and the supplied journal oracle.
- Canonical order-independent journal commitment over set-like source references and posting lines.
- Unit tests for digest tampering, entity mismatch, amount mutation, duplicate handling,
  canonical posting permutation and integer overflow.

This is a **reference-corpus verifier**, not the candidate-system runner and not a production
accounting kernel. In v0.1 it verifies that the frozen seed oracle is internally coherent and
reconstructable; it does not establish that arbitrary real-world invoices are authentic, that
all economic events have been observed, or that an accounting policy is correct.

## Run

From this directory:

```sh
cargo generate-lockfile
cargo fmt --all
cargo fmt --all -- --check
cargo test --locked
cargo clippy --all-targets -- -D warnings
cargo run --locked -- --corpus ../smoke_corpus_v0.1.0.json \
  --schema ../smoke_corpus_v0.1.0.schema.json \
  --protocol ../../../docs/ACCOUNTING_EVIDENCE_BENCHMARK_V1.md \
  --toolchain "$(rustc --version)" \
  --out verification.json
```

The verifier has a committed `Cargo.lock` and is qualified by the dedicated branch-push workflow using Rust 1.99.1.

**Successful exact-head qualification (4 November 2026):**
- `cargo fmt --all -- --check`: passed.
- `cargo test --locked --all-targets`: 19 passed, 0 failed.
- `cargo clippy --locked --all-targets -- -D warnings`: passed.
- `cargo run --locked` against the three seed cases: passed; receipt status `PASS`.
- Full input corpus Draft 2020-12 validation: passed.
- Output receipt Draft 2020-12 validation: passed.
- ACC-SYN-0001: 73 invariant checks, all passed.
- ACC-SYN-0002: 57 invariant checks, all passed.
- ACC-SYN-0003: 29 invariant checks, all passed.

Run: [GitHub Actions run #19](https://github.com/Luminous-Dynamics/mycelix/actions/runs/41886814130), exact run head `0992981f210229e1ec3a4de15cfdf9bf71265b88`. Rust toolchain: `rustc 1.99.1 (58cf8f9c3 2026-10-13)`.

This is a qualification of the **three synthetic reference fixtures only**. It is not a reproduction of FINBALANCE, DualBench100, APEX or BlueFin; it is not IFRS/IFRS for SMEs/GAAP/SARS compliance, audit assurance, source authenticity or entity-wide completeness. The PR remains draft pending independent review and candidate/oracle separation.

The branch-push workflow is now read-only: `contents: read`, committed lockfile, and locked validation. A source or protocol change triggers a new exact-head run. No missing, queued, skipped or unavailable workflow is treated as a pass.
## Amount and projection semantics

The smoke profile uses synthetic ZAR minor units and non-negative decimal integer strings.
The verifier converts them to checked signed integer balances only for debit-minus-credit
arithmetic. It does not use binary floating point.

Statement reconstruction is limited to the simple account classes in this smoke schema:
asset (debit-normal), liability/equity/income (credit-normal), and expense (debit-normal).
It is not a general IFRS presentation engine and does not model contra-accounts, deferred tax,
FX, consolidation, or jurisdictional tax.

## Receipt fields

The deterministic JSON report includes raw-byte digests for corpus, schema and protocol,
a source digest over this crate's manifest / README / report schema / main source file, each case's invariant
checks, per-entry canonical commitments, and a terminal status. The source digest is a source
snapshot identifier, not a signed build attestation. The receipt includes source, corpus, schema, protocol and toolchain identifiers; the lockfile has a separate SHA-256 digest recorded in the run artifact.

## Next

- Keep Cargo.lock committed and exact-head qualification locked.
- Review the oracle independently with an accounting professional.
- Review the oracle independently with an accounting professional.
- Extend the corpus and separate candidate-visible inputs from evaluator-only expected outputs.
