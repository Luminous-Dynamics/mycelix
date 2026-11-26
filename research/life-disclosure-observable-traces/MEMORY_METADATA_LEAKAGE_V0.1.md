# LIFE-MEMORY-METADATA-LEAKAGE-001

## Purpose

A memory service can deny direct access to a protected record while still leaking whether that record exists, changed, was deleted, matches a query, or survives in a cache/index. This experiment evaluates the sequence of observer-visible responses, including follow-up requests chosen from prior visible history.

This is the memory-specific owner for Mycelix #6730. It composes the generic ObserverView and compare_observer_views abstraction from #5460; it does not introduce a second generic trace system, memory store, access-control engine or revocation service.

## Frozen seed panel

memory-sequence-manifest.json freezes 48 wholly synthetic sequence seeds:

- six memory-state families: existence, protected value, update/supersession, deletion/tombstone, query-match/ranking, cache/index lifecycle;
- one-shot and repeated-adaptive query modes;
- single and colluding principals;
- forbidden and explicitly authorised disclosure.

Each seed is crossed with two observer profiles (normalized and a family-specific planted leak) and complete/incomplete trace coverage, yielding 192 qualification rows.

The attacker trains on one-shot sequences and is evaluated on repeated/adaptive sequences. Adaptive follow-ups are selected only from the attacker's prior visible target-value observation; protected-world labels and evaluator state are never inserted into the candidate or attacker view. Colluding principals combine only the transcripts each principal could individually see.

## Controls

- **Normalized forbidden control:** paired worlds differ only in protected state; all required observer-visible fields must remain equivalent.
- **Family-specific leak controls:** a planted forbidden signal is sent only to the designated recipient principal. The control verifies detection and attribution across all six families.
- **Authorised sensitivity:** explicitly permitted disclosure must visibly change the authorised disclosure field while the other required transcript fields remain equivalent.
- **Incomplete trace:** a missing required observer field always produces INDETERMINATE, never a privacy pass.
- **No constant-response shortcut:** the task result remains usable (ELIGIBLE), and authorised sensitivity must still be observed.

## Run

```sh
cargo fmt --manifest-path research/life-disclosure-observable-traces/Cargo.toml --all -- --check
cargo test --locked --manifest-path research/life-disclosure-observable-traces/Cargo.toml --all-targets
cargo clippy --locked --manifest-path research/life-disclosure-observable-traces/Cargo.toml --all-targets -- -D warnings
cargo run --quiet --locked --manifest-path research/life-disclosure-observable-traces/Cargo.toml --bin memory-metadata-leakage
```

The existing generic #5460 executable and report format remain unchanged. The memory-specific executable produces a separate deterministic CSV with one row per sequence/profile/coverage combination. CI executes it twice, compares output bytes, checks the row count, and hashes the report and both channel/sequence manifests against the exact checked-out commit.

## Metrics

Report, do not collapse:
- exact paired transcript mismatch and mismatched field list;
- attacker held-out balanced accuracy (one-shot train, repeated-adaptive test);
- leakage family, query mode, principal mode and disclosure mode;
- collusion lift relative to a single principal;
- authorised disclosure sensitivity;
- observer coverage and INDETERMINATE count.

This 48-sequence panel is a deterministic coverage seed, not a statistically powered estimate. It does not report ROC-AUC, average precision, confidence intervals or empirical prevalence. The baseline attacker is intentionally simple and must be supplemented by stronger held-out attackers and query-policy holdouts before empirical claims.

## Limitations and claim ceiling

Everything is synthetic. No deployed memory API, Holochain conductor, network, cache, vector index, model, or serialization stack is instrumented. Response timing is not measured. The benchmark does not establish differential privacy, formal noninterference, production privacy, anonymity, deletion completeness, or superiority over another system.

A PASS means only that the exact declared observer fields in this finite synthetic paired-world panel are equivalent. Channels outside the manifest and real infrastructure behavior remain untested.
