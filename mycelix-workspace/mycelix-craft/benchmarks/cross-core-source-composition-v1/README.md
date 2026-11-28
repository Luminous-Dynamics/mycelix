# Craft cross-core source composition V1

This increment addresses the source-composition prerequisite in [issue #6790](https://github.com/Luminous-Dynamics/mycelix/issues/6790).

It materializes seven independently developed, standard-library-only Rust owner crates from exact PR head commits into one temporary Cargo workspace: six domain owners plus the separate immutable action-denominator owner.

| PR | Owner core | Pinned commit |
|---|---|---|
| #6750 | need-intake-v1 | `3ae2f0cfeee3576f78ecf7ce6a4b8a007e51c34d` |
| #6733 | project-formation-v1 | `1bc1f221814dda2bd8786535866ed586c7114be6` |
| #6754 | claim-support-v1 | `30befb4bc5190af891cff6f820c5eb77d72fb8e6` |
| #6757 | decision-closure-v1 | `c19d12c671a4cb8ddc050dacd3ac7749e38f3314` |
| #6762 | evidence-lineage-v1 | `2d892c9eaf945ccd31d1816e4edadbbe1236a359` |
| #6769 | impact-qualification-v1 | `02c140cbaa43ae7ec4276c7edbc904a1526a4803` |
| #6792 | craft-action-denominator-v1 | `8cbbb38c44bfc69822703f6d28185d33b619894d` |

## Run

From a checkout of this repository:

```sh
rustup toolchain install 1.96.0 --profile minimal
rustup component add rustfmt clippy --toolchain 1.96.0
OUT_DIR="$(mktemp -d)"
bash mycelix-workspace/mycelix-craft/benchmarks/cross-core-source-composition-v1/materialize.sh "$OUT_DIR"
cargo +1.96.0 generate-lockfile --manifest-path "$OUT_DIR/Cargo.toml"
cargo +1.96.0 fmt --manifest-path "$OUT_DIR/Cargo.toml" --all -- --check
cargo +1.96.0 test --manifest-path "$OUT_DIR/Cargo.toml" --workspace --locked
cargo +1.96.0 clippy --manifest-path "$OUT_DIR/Cargo.toml" --workspace --all-targets --locked -- -D warnings
```

The materializer checks both the remote branch tip and the Git blob IDs for each package manifest and library source before copying them. Any branch movement or file substitution fails closed. The generated source receipt records the exact revisions and content IDs. The separate `harness` workspace member is taken from the exact harness commit checked out by CI; its manifest and source SHA-256 values are also recorded.

## Claim boundary

A green result establishes that all seven pinned packages can coexist in one Cargo workspace and that each package's own unit tests, formatting and Clippy gates pass in that combined source snapshot.

## Executable cross-core slice (current)

The harness calls the public APIs of the six domain owners and the seventh action-denominator owner, then runs an independent, small service-level oracle:

- WSR-001: provider-reported completion with no current pressure-restoration or water-quality support must remain NotEligible; Decision Closure reports OpenMissingElements.
- WSR-004: current pressure and quality support may reach EligibleForLocalReview, while one of the two required mapping approvals is absent; Decision Closure reports MappingReviewRequired, never Closed.
- A source expiry must propagate NeedsRevalidation to dependent claim and decision artifacts through Evidence Lineage.
- WSR-009: an expired permit and revoked crew qualification keep the safety-critical work package BlockedBySafetyGate and the dependent work authorization NeedsRevalidation.
- Two measurements assigned to one common-cause group cannot satisfy a two-independent-source rule.
- WSR-006: a pressure observation with a contradictory Scope qualifier for another zone returns HumanReviewRequired and cannot become direct support for the target zone.

The Need Intake test constructs a valid synthetic community proposal and confirms it remains Draft without community/domain/provenance/safety review and explicit local ratification. Project Formation receives a consenting participant with no capability evidence and returns UnknownEvidence rather than a negative judgement or qualification.

The oracle is deliberately independent of the owner evaluators. The executable slice currently exercises seven named cases among the 16 frozen WSR cases (WSR-001, WSR-004, WSR-006, WSR-009, WSR-010, WSR-011, WSR-012), using eleven harness tests. It also checks expiry propagation with an unaffected-source precision control and common-cause independence. It does **not** claim the other nine WSR cases or all 20 registered mutations have executed.

## Claim boundary

The green result demonstrates this exact source composition plus the listed synthetic, typed cross-core flows. It is not a full WSR benchmark score, field validation, professional water-quality certification, legal authorization or permission to close a municipal service incident. The harness uses synthetic reviewer and authority references; no real authority or resident data is present.


## Benchmark protocol

See [WATER-SERVICE-CROSS-CORE-BENCHMARK-PROTOCOL-V1.md](WATER-SERVICE-CROSS-CORE-BENCHMARK-PROTOCOL-V1.md) for the baseline matrix, full-corpus qualification targets, independent-oracle requirements, water-service outcome metrics, performance reporting and field-pilot limits. It distinguishes the six-test executable slice from the remaining WSR corpus and the 20 proposed metamorphic transformations.



**WSR-010 implementation update:** the harness imports the standalone `craft-action-denominator-v1` owner through its public API. The owner recomputes a canonical plan digest from the complete action records, requires sequential authority-bound amendments, binds each observation to the exact plan and evidence-profile digest, and reports covered/uncovered action IDs. The known-answer fixture now uses a six-action P1 plan, five qualified action observations, 99 identical duplicate replays, and an unauthorized action mutation that must not increase coverage. A separate P1 six-action → authority-approved P2 five-action amendment proves that historical P1 replay is preserved and P1 observations do not count toward P2. Full P2 action coverage still cannot auto-close the service episode. Authority signatures and source truth remain unauthenticated by this reference core.

## Qualification history and current status

**Historical six-owner baseline:** workflow [#46005931741](https://github.com/Luminous-Dynamics/mycelix/actions/runs/46005931741) passed on earlier harness head `cd5affd6ff41c48aa8e6653170d22d8b4af5d37a`: six domain-owner crates passed 159 tests, ten harness tests passed, formatting and Clippy passed. This is valid evidence for that earlier six-owner source set only; it does not qualify the later action-denominator integration.

**Latest completed seven-owner attempt:** workflow [#46045308672](https://github.com/Luminous-Dynamics/mycelix/actions/runs/46045308672) checked out then-current harness head `2f27a4ec65a02913e58e210867d0585e98688ed0`. All seven owner packages materialized; 195 owner tests passed and combined-workspace formatting passed. The harness ran 11 tests: 10 passed and WSR-010 failed because the test helper panicked while trying to construct an observation for an action absent from the sealed plan. The WSR-010 fixture was repaired to mutate a valid plan-bound observation instead. The subsequent revision now expands WSR-010 to the six-action / five-observation / 99-duplicate control and an authorized 6→5 amendment. Clippy and the composition receipt were skipped after the failing test.

**Current-head rule:** the current seven-owner integration is qualified only by a completed workflow whose checked-out SHA exactly equals the current head displayed on [PR #6790](https://github.com/Luminous-Dynamics/mycelix/pull/6790). A workflow for an ancestor commit is historical evidence only; queued, skipped, cancelled, absent or failed checks remain unqualified. This rule is intentionally stated without hard-coding a moving branch tip into the README.

The materializer records the harness manifest/source digests and all seven owner manifest/source blob IDs. The lockfile is regenerated from the frozen materialized source set during each run, and its digest belongs in that run's evidence receipt.

