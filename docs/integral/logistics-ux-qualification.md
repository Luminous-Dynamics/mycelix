# Logistics Commons UX qualification process

This process applies to the **synthetic, read-only** UI prototype in PR #4934. It does not qualify the S0 corpus, simulator/verifier, Holochain runtime behavior, or any physical logistics process.

## Stage 1 — Dependency-free source contract

From the repository root, run:

```sh
python3 scripts/integral/verify_logistics_ux_contract.py
```

The tool first executes its own five Python self-tests, then inspects the actual repository source and workflow. It checks fixture count/unique IDs/category distribution, the oversubscribed conflict example, completeness of each fixture's guidance, enum-typed filter semantics, shared search/count matching, reset and no-results behavior, expected Rust test definitions, route registration, synthetic-only labeling, basic accessibility/responsive rules, exact-head checkout assertions, pinned actions, and workflow integration.

A green result means **source contract only**. It does not mean the Rust code compiles or Rust tests execute. Fix any failed invariant, then run the preflight again before spending build time.

To test only the checker itself:

```sh
python3 scripts/integral/verify_logistics_ux_contract.py --self-test-only
```

## Stage 2 — Rust formatting and unit tests

The targeted workflow uses Rust 1.99.0 and runs these commands:

```sh
rustfmt --edition 2024 --check --config skip_children=true mycelix-workspace/mycelix-commons/apps/leptos/src/pages/logistics.rs
cargo test --manifest-path mycelix-workspace/mycelix-commons/apps/leptos/Cargo.toml
```

Record the full `git rev-parse HEAD`, `rustc --version`, `cargo --version`, commands, exit codes and logs. These are compile/test evidence, not browser evidence.

## Stage 3 — WASM compilation

```sh
cargo check --manifest-path mycelix-workspace/mycelix-commons/apps/leptos/Cargo.toml --target wasm32-unknown-unknown
```

Tie the result to the same exact SHA and toolchain identity. A successful Cargo check does not exercise rendered browser interactions or Trunk's post-build optimization hook.

## Stage 4 — Browser smoke test

The automated browser gate is **not yet implemented**. Do not mark it PASS until an automated test and output exist. It should use the actual served CSR app and capture:
- `/transport/logistics` renders and shows the synthetic-only notice.
- Searching changes the matching rows and all four filter counts.
- Search and filter intersect; reset clears the query and restores the default filter.
- A record disclosure opens through keyboard and pointer input.
- An impossible query exposes the accessible no-results state.
- No live inventory query or mutation occurs.

A source scan is not a substitute for this stage.

## Dependency policy

The standalone Leptos app currently has no committed `Cargo.lock`; dependency resolution is therefore not frozen. Do not hand-author or guess a lockfile. Generate it under the intended app boundary, review and commit the resolved dependency graph, establish an update policy, then use `cargo test --locked` and `cargo check --locked --target wasm32-unknown-unknown`.

Leptos `view!` markup should have a pinned `leptosfmt` gate once the exact formatter version and installation inputs are committed and verified. Ordinary `rustfmt` does not replace a macro-aware formatter.

## Queue triage

When a workflow is queued:
1. Confirm its `head_sha` equals the current PR head.
2. Check job status, runner assignment, and steps. If no runner is assigned and no steps ran, record **QUEUED / NOT EXECUTED**—not PASS and not a code failure.
3. Inspect repository/organization Settings → Actions → Runners → GitHub-hosted runners → **All jobs usage** for active jobs and concurrency limits. API queue counts alone cannot prove the cause.
4. Avoid unrelated commits intended only to “retry” the run. With `cancel-in-progress: true`, a head change cancels the earlier attempt and creates a new queued run.
5. For a started failure, diagnose the first failing step on that exact SHA; distinguish formatter, dependency resolution, Rust tests/compilation, WASM, and browser failures.

## Evidence ledger

For every stage, record the full source SHA, stage, outcome, command/exit code or canonical run URL plus step log, toolchain/target, and limitations. Permitted outcomes are **PASS / FAIL / INCOMPLETE / QUEUED / CANCELLED**. Queued, skipped, cancelled, stale-head, and absent evidence are never PASS.

Keep this UX path separate from S0 simulator and independent-verifier qualification.
