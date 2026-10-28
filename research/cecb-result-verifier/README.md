# CECB Result Verifier

Standalone Rust-first, fail-closed verifier for Capability-Economy Comparative Benchmark (CECB) result bundles.

## Status and claim ceiling

The verifier checks that result artifacts conform to the supplied, execution-authorized design manifest. A passing report establishes **artifact-contract conformance only**. It does not establish that the Civilization Twin is empirically valid, that counterfactuals are causally correct, or that any economic institution is superior.

The current CECB paired-plan generator intentionally emits the status design_only_no_policy_execution, sets execution_authorized=false, leaves numeric scenario mappings unresolved, and marks policy adapters adapter_pending. The verifier must reject that plan until a reviewed adapter contract binds the plan to executable Civilization Twin behavior, a source commit, simulator digest and policy-profile digests. Do not change that fail-closed behavior merely to make a fixture pass.

## Use

    cargo fmt --manifest-path research/cecb-result-verifier/Cargo.toml -- --check
    cargo test --manifest-path research/cecb-result-verifier/Cargo.toml
    cargo run --manifest-path research/cecb-result-verifier/Cargo.toml -- \
      --plan /path/to/authorized-plan.json \
      --results /path/to/result-bundle.json \
      --report /path/to/verification-report.json

Exit codes: 0 for a verified artifact bundle, 1 for a structurally readable but invalid bundle, and 2 for invocation or input parsing/I/O errors.

## Contract checks

- Requires plan schema cecb.housing.paired_design.v0.1, result schema cecb.housing.policy_result_bundle.v0.1, an explicitly execution-authorized plan, resolved Civilization Twin/numeric-scenario/policy-adapter mappings, a non-empty resolved_parameters object for every configuration, a source commit SHA, simulator digest and a ready/pinned policy-profile digest for each arm.
- Strictly parses both JSON inputs and rejects duplicate object keys at any depth before canonical hashing. This avoids silent last-value-wins ambiguity.
- Uses serde_json_canonicalizer 0.4.1 for RFC 8785 JCS bytes and SHA-256 for plan and per-configuration identity. It rejects integer-valued inputs outside the exact IEEE-754 safe-integer range rather than hashing rounded quantities as if they were exact. Current schema values that need larger exact precision must be redesigned in a versioned contract to use strings or scaled safe integers. Non-finite JSON tokens are rejected.
- Requires exactly one terminal result per planned run_id; rejects missing, duplicate and unexpected rows, and validates each row's design/configuration/seed/arm/source/profile/simulator identities.
- Requires exactly 16 registered result fields and all 12 registered outcome metrics with pinned units, aggregation, direction and missing policy. These metric definitions are checked against the v0.1 contract; matching metric names with relabeled units do not pass. Unknown is represented explicitly as JSON null; omitted metrics are errors and are never replaced by zero. Object-valued group/resource metrics preserve their per-group/per-resource coordinates.
- Requires each result to list artifact SHA-256 digests and a terminal disposition. Any non-empty guardrail-violation list must be labelled guardrail_failed; guardrail-failed runs are reported but never counted among completed guardrail-clear runs. The report separately counts completed guardrail-clear runs and complete outcome vectors; any vector with unknown cells is excluded from the latter.
- Rejects unknown result fields, including attempted addition of a universal welfare scalar.
- Emits a deterministic report with contract status, coverage counts, terminal dispositions, eligible/guardrail counts, unknown outcome-cell counts and all verification errors.

## Known limitations

- The result contract and manifest digest profile are being integrated with the plan generator under [CIV-IMPACT-003 / #5506](https://github.com/Luminous-Dynamics/mycelix/issues/5506). Current design-only manifests are expected to fail until those execution bindings are reviewed and added.
- The verifier does not implement the Civilization Twin, allocation policies, statistical effect estimation, model calibration or hidden-set isolation.
- It verifies declared SHA-256 digests and typed field relationships; it cannot prove that a source-controlled simulator actually produced a claimed output unless execution provenance/attestation is bound and independently trusted.
- The candidate result bundle is not itself a trusted oracle. Keep verifier/oracle information outside the candidate process in any future competitive evaluation.

## Research references

- [RFC 8785 — JSON Canonicalization Scheme](https://www.rfc-editor.org/rfc/rfc8785.html), an Informational RFC defining deterministic JSON serialization for hashing/signing.
- [serde_json_canonicalizer 0.4.1](https://docs.rs/crate/serde_json_canonicalizer/0.4.1), selected for its RFC 8785-compatible canonicalizer and published test coverage. The crate documents the IEEE-754 precision boundary; duplicate-key rejection is handled separately here.
- [NIST TEVV-Athlon Framework](https://www.nist.gov/artificial-intelligence/ai-research/tevv-athlon-framework-evaluating-ai-systems) for structured Test, Evaluation, Verification and Validation.
- [CIV-IMPACT-004 / #5507](https://github.com/Luminous-Dynamics/mycelix/issues/5507) for exogenous-event alignment across paired counterfactuals.

The verifier's report claim ceiling remains artifact-contract conformance, not model realism, causal effect or policy superiority.
