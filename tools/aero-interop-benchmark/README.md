# aero-interop-benchmark

An evidence-backed, vendor-neutral QIF/AP242 differential benchmark evaluator.

## What it does

- Freezes a versioned benchmark manifest with exact source artifacts, rights status, an independent oracle artifact, constrained oracle method/profile and adjudicated claim labels.
- Requires a complete tool × case run grid. A missing run is not silently removed from the denominator.
- Verifies the exact bytes of every registered tool package (executable, arguments, environment profile), corpus input and oracle record before scoring, even when a tool/case slot is Blocked or NotRun.
- Reuses AeroAdapterRunReceiptV1 validation and exact payload verification for every executed tool run. Pins each tool's argument vector, environment-profile bytes, configuration digest, locale, timezone, timeout and network policy before the run.
- Binds each run to the registered tool ID/version/executable, predeclared invocation profile, benchmark ID, case ID and exact one-to-one source input set.
- Requires a successful run to report each expected claim exactly once.
- Preserves Failed, TimedOut, Blocked and NotRun attempts; failure and timeout receipts remain in the score denominator.
- Reports confusion matrices and execution outcomes separately by engineering dimension.
- Keeps Oracle Unadjudicated, tool Unknown, Unsupported, Error, NotRun and Blocked distinct.
- Counts safety-critical false accepts explicitly.
- Never collapses the result into a single aggregate quality/trust score.

## Run

    cargo fmt --manifest-path tools/aero-interop-benchmark/Cargo.toml -- --check
    cargo test --manifest-path tools/aero-interop-benchmark/Cargo.toml
    cargo clippy --manifest-path tools/aero-interop-benchmark/Cargo.toml --all-targets -- -D warnings

## Input contract

`AeroInteropBenchmarkV1` contains:
- benchmark/profile and QIF/AP242 edition identities;
- a tool roster with exact executable, argument-vector and environment-profile artifacts, configuration digest, locale/timezone, timeout, network policy and tool category;
- cases with exact input artifacts and explicit testing/redistribution rights;
- a separately pinned oracle artifact and rights status, distinct from every source input;
- a constrained oracle method (independent hand enumeration, dual review, independent tool plus review, published reference, or explicitly unadjudicated) and profile version; no LLM-only oracle mode;
- claim-level labels: Accept, Reject or Unadjudicated;
- source locator, rationale code and safety criticality per claim.

Every tool/case cell has one `ToolRunEvidence` record:
- Succeeded / Failed / TimedOut requires a valid AdapterRunReceiptV1 and exact receipt payloads;
- Blocked / NotRun cannot carry a fabricated receipt or output and requires a reason code;
- Succeeded must emit exactly one observation per oracle claim;
- Failed and TimedOut runs cannot contribute semantic claim observations;
- missing observations, duplicate claim IDs, unknown tools/cases and missing grid cells reject.

`evaluate_benchmark` requires the exact corpus input/oracle payloads as an explicit argument and verifies them before any metrics are calculated. There is no `corpus_verified=true` shortcut.

## Scorecard dimensions

Each tool receives independent dimensions:
- QIF structure;
- normative QIF checks;
- AP242 PMI conformance;
- source extraction;
- identity resolution;
- semantic preservation;
- measurement-result status retention;
- reproducibility;
- execution security.

Each dimension reports offered and adjudicated denominators, TP/FP/TN/FN, oracle-unadjudicated claims, tool Unknown/Unsupported/Error, NotRun/Blocked and critical false accepts.

Do not compare different tool categories as if they do the same work. A QIF XSD validator is not an AP242 PMI analyzer; a CAD/CMM converter is not a PLM workflow; a trace resolver is not a metrology acceptance engine.

## Oracle policy

Oracle labels must be authored/adjudicated independently from the implementation under test. The manifest binds an oracle artifact, rights status, method and profile version. Unadjudicated claims are retained but never counted as true/false outcomes.

NIST STEP Analyzer, CAx-IF practice reports, QIF XSD/XSLT outputs and commercial adapters are valuable external observations, but none is declared universal ground truth. Conflicts require documented adjudication; majority vote and LLM-only labels are not acceptable oracle construction.

## Benchmark targets (not results)

- 100% adjudicated-oracle agreement on frozen trace-critical positive/negative cases;
- zero false accepts on wrong-QPId, wrong-subject, stale-revision, dropped-field and namespace-collision controls;
- zero QIF local-ID/xId/STEP UUID conflation;
- 100% retention of Passed/Failed/NotEvaluated/Unknown measurement-result trace rows;
- 100% exact-byte replay for corpus, tools, receipts, reports and outputs;
- zero missing tool × case cells or unaccounted claim observations;
- 100% explicit reporting of warning/unknown/unsupported/error/timeout/blocked states.

Receipt-backed wall-time p50/p95/min/max includes succeeded, failed and timed-out attempts; blocked/not-run cells remain visible but have no fabricated duration. Peak memory/CPU, bytes processed, throughput and total execution cost require separate instrumentation and are not yet implemented in this scorer. No performance/SOTA claim is valid without matched hardware, tool version/configuration, input bytes, corpus digest and complete budget accounting (including failed runs, setup, tuning and audit).

## Claim ceiling

This crate scores supplied tool observations against a versioned oracle. It does not execute adapters, parse STEP/QIF, create oracle labels, authenticate tool publishers or prove that the oracle is physically true. A zero false-pass rate on a finite synthetic corpus is not certification, metrology validity, product acceptance or airworthiness.
