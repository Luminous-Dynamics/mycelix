# Craft cross-core source composition V1

This increment addresses the source-composition prerequisite in [issue #6790](https://github.com/Luminous-Dynamics/mycelix/issues/6790).

It materializes six independently developed, standard-library-only Rust owner cores from exact PR head commits into one temporary Cargo workspace:

| PR | Owner core | Pinned commit |
|---|---|---|
| #6750 | need-intake-v1 | `3ae2f0cfeee3576f78ecf7ce6a4b8a007e51c34d` |
| #6733 | project-formation-v1 | `1bc1f221814dda2bd8786535866ed586c7114be6` |
| #6754 | claim-support-v1 | `30befb4bc5190af891cff6f820c5eb77d72fb8e6` |
| #6757 | decision-closure-v1 | `c19d12c671a4cb8ddc050dacd3ac7749e38f3314` |
| #6762 | evidence-lineage-v1 | `2d892c9eaf945ccd31d1816e4edadbbe1236a359` |
| #6769 | impact-qualification-v1 | `02c140cbaa43ae7ec4276c7edbc904a1526a4803` |

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

### Independent PM4Py OCEL 2.0 oracle (test-only)

An optional differential oracle is provided at `oracles/pm4py_ocel2_roundtrip.py`. It imports the fixture with PM4Py's standard OCEL 2.0 JSON importer, exports it again, and compares a normalized inventory of declarations, typed attributes, timestamps, event/object identities, and qualified E2O/O2O relationships.

Run it in an isolated test environment with Python 3.11+ and the exact PM4Py release used by the oracle:

```sh
python3 -m venv "$OUT_DIR/pm4py-venv"
"$OUT_DIR/pm4py-venv/bin/python" -m pip install 'pm4py[stable]==2.7.23.15'
"$OUT_DIR/pm4py-venv/bin/python" \
  mycelix-workspace/mycelix-craft/benchmarks/cross-core-source-composition-v1/oracles/pm4py_ocel2_roundtrip.py \
  --source mycelix-workspace/mycelix-craft/benchmarks/cross-core-source-composition-v1/ocel/ocel-water-service-v1.json \
  --roundtrip "$OUT_DIR/pm4py-roundtrip.json"
```

PM4Py 2.7.23.15's `stable` extra pins the project's tested baseline dependency versions. The exact PM4Py wheel SHA-256 is `6bc2a053dab98e555b33b6d2ba966972bfcadace7b75bb7afaf788a846ff7139` (PyPI, uploaded 2026-11-24). This still does not provide a full hash-locked environment receipt for every transitive wheel or the Python interpreter. Treat it as an executable exploratory oracle, not an exact-head CI qualification, until the independent-oracle workflow has a reviewed lock/constraints file and emits its own reproducible environment receipt. Python and PM4Py remain outside the Rust production dependency graph.

## Claim boundary

A green result establishes that the six pinned packages can coexist in one Cargo workspace and that each package's own unit tests, formatting and Clippy gates pass in that combined source snapshot.

## Executable cross-core slice (current)

The harness calls the public APIs of all six pinned owners and runs an independent, small service-level oracle:

- WSR-001: provider-reported completion with no current pressure-restoration or water-quality support must remain NotEligible; Decision Closure reports OpenMissingElements.
- WSR-004: current pressure and quality support may reach EligibleForLocalReview, while one of the two required mapping approvals is absent; Decision Closure reports MappingReviewRequired, never Closed.
- A source expiry must propagate NeedsRevalidation to dependent claim and decision artifacts through Evidence Lineage.
- WSR-009: an expired permit and revoked crew qualification keep the safety-critical work package BlockedBySafetyGate and the dependent work authorization NeedsRevalidation.
- Two measurements assigned to one common-cause group cannot satisfy a two-independent-source rule.
- WSR-006: a pressure observation with a contradictory Scope qualifier for another zone returns HumanReviewRequired and cannot become direct support for the target zone.

The Need Intake test constructs a valid synthetic community proposal and confirms it remains Draft without community/domain/provenance/safety review and explicit local ratification. Project Formation receives a consenting participant with no capability evidence and returns UnknownEvidence rather than a negative judgement or qualification.

The oracle is deliberately independent of the owner evaluators. The executable slice currently covers four of the 16 frozen cases (WSR-001, WSR-004, WSR-006, WSR-009) plus expiry propagation (including an unaffected-source precision control) and common-cause independence. It does **not** claim the other 14 known-answer cases or all 20 registered mutations have executed.

## Claim boundary

The green result demonstrates this exact source composition plus the listed synthetic, typed cross-core flows. It is not a full WSR benchmark score, field validation, professional water-quality certification, legal authorization or permission to close a municipal service incident. The harness uses synthetic reviewer and authority references; no real authority or resident data is present.


## Benchmark protocol

See [WATER-SERVICE-CROSS-CORE-BENCHMARK-PROTOCOL-V1.md](WATER-SERVICE-CROSS-CORE-BENCHMARK-PROTOCOL-V1.md) for the baseline matrix, full-corpus qualification targets, independent-oracle requirements, water-service outcome metrics, performance reporting and field-pilot limits. It distinguishes the six-test executable slice from the remaining WSR corpus and the 20 proposed metamorphic transformations.

## Exact-head qualification result

Workflow [run #46001747298](https://github.com/Luminous-Dynamics/mycelix/actions/runs/46001747298) passed on harness head `93eadac660638db3110d8ff0190777a868768804`.

- Six source branch heads and both source blob IDs per owner were verified before materialization.
- Six owner crates passed their combined-workspace unit tests: 159 tests total.
- Six cross-core tests passed: WSR-001, WSR-004, WSR-006, WSR-009, expiry propagation with unaffected-source preservation, and common-cause independence.
- Exact-head formatting and combined-workspace Clippy passed.
- Generated Cargo.lock SHA-256: `d87aaed918e666446a61dbdb784b1c8fcb252d27d57e4fbce16320be6ae7da63`.
- Source-manifest SHA-256: `fc0a70c72967cbc366abe9aac866c097dfa96c1cf9648baad9e2a4bf685bb9fe`.

The materializer records two harness-source SHA-256 values (Cargo manifest and Rust source) and the six pinned owner manifest/source blob IDs. The temporary lockfile is regenerated by CI and its digest is recorded; it is not committed as a permanent lock for six changing upstream branches.



## OCEL 2.0 water-service interoperability slice

The follow-on branch [research/craft-water-ocel-interoperability-v0.1-20261127](https://github.com/Luminous-Dynamics/mycelix/tree/research/craft-water-ocel-interoperability-v0.1-20261127) adds a restricted OCEL 2.0 JSON profile:

- fixture: [ocel-water-service-v1.json](./ocel/ocel-water-service-v1.json), with 13 object types, 11 event types, 17 object instances and 12 events;
- validator/canonicalizer: `harness/src/ocel_profile.rs`;
- dual-frontier replay: `harness/src/temporal_replay.rs`;
- public module: `craft_water_cross_core_harness::ocel_profile`;
- declared OCEL dependencies: pinned `serde_json` and `sha2`;
- materializer: copies the complete harness source directory and records the SHA-256 of every copied Rust source file, not only `lib.rs`.

The fixture demonstrates a same-fault pair of complaints linked to one callout, a distinct repeat service episode, source-recorded time and host sequence on events, timed object-attribute history, and separately linked work-order, crew, permit, pressure observation, sample and lab-result objects. It also includes a correction event that supersedes an earlier pressure assertion and a revocation event that withdraws an earlier lab-result assertion.

The validator is a **restricted water-service OCEL 2.0 profile**, not a full OCEL schema implementation. It checks the four core collections, type declarations, typed attributes, timestamp syntax, unique IDs, and qualified event-to-object/object-to-object references. Its canonical JSON digest is a representation receipt for this profile; it is not a signature, source-authenticity proof or owner-core evidence receipt. The temporal replay module separately evaluates an inclusive valid-time cutoff and an inclusive host-sequence knowledge frontier. It validates `source_recorded_at` but does not pretend that a source timestamp or host sequence is a trusted platform transaction-time clock. A replay digest proves deterministic reconstruction of this synthetic event set only.

OCEL 2.0 JSON is the stable interoperability baseline. OCEL 2.1.0pre5 remains an experimental follow-on and is not a production dependency. See issue [#6794](https://github.com/Luminous-Dynamics/mycelix/issues/6794).

The cross-core source-composition workflow now copies and hashes every source file under `harness/src`. Exact-head OCEL tests, the combined six-owner test suite, Rust formatting and Clippy must all pass before the new profile is called qualified.
