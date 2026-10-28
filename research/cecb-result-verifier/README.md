# CECB Result Verifier

Standalone Rust-first, fail-closed verifier for Capability-Economy Comparative Benchmark (CECB) result bundles.

## Status and claim ceiling

The verifier checks that result artifacts conform to the supplied, execution-authorized design manifest. A passing report establishes **artifact-contract conformance only**. It does not establish that the Civilization Twin is empirically valid, that counterfactuals are causally correct, or that any economic institution is superior.

The public CECB paired-plan generator currently emits paired_design.v0.1 as a design-only artifact. This verifier deliberately accepts only the execution-authorized paired_design.v0.2 contract: it requires a pinned source commit, resolved Civilization Twin/numeric-scenario/policy-adapter mappings, non-empty resolved_parameters for every configuration, an event-keyed RNG contract and exogenous schedule digest, an equal-resource/compute-budget profile digest, a simulator digest, and ready policy arms with pinned profile digests. Do not simply relabel a v0.1 manifest as v0.2 or weaken the checks to make a fixture pass; generate v0.2 only after the relevant adapters and evidence contracts have been reviewed.

## Use

    cargo fmt --manifest-path research/cecb-result-verifier/Cargo.toml -- --check
    cargo test --locked --manifest-path research/cecb-result-verifier/Cargo.toml
    cargo run --manifest-path research/cecb-result-verifier/Cargo.toml -- \
      --plan /path/to/authorized-plan.json \
      --results /path/to/result-bundle.json \
      --report /path/to/verification-report.json

Exit codes: 0 for a verified artifact bundle, 1 for a structurally readable but invalid bundle, and 2 for invocation or input parsing/I/O errors.

## Contract checks

- Requires plan schema cecb.housing.paired_design.v0.2, result schema cecb.housing.policy_result_bundle.v0.2, an explicitly execution-authorized plan, resolved Civilization Twin/numeric-scenario/policy-adapter mappings, a non-empty resolved_parameters object for every configuration, an event-keyed RNG contract/version and exogenous schedule digest, an equal resource/compute-budget profile digest, a source commit SHA, simulator digest and a ready/pinned policy-profile digest for each arm.
- Strictly parses both JSON inputs and rejects duplicate object keys at any depth before canonical hashing. This avoids silent last-value-wins ambiguity.
- Uses exact direct dependency versions for serde 1.0.229, serde_json 1.0.154, serde_json_canonicalizer 0.4.1 and sha2 0.10.9. It uses the canonicalizer for RFC 8785 JCS bytes and SHA-256 for plan and per-configuration identity. It rejects integer-valued inputs outside the exact IEEE-754 safe-integer range rather than hashing rounded quantities as if they were exact. Current schema values that need larger exact precision must be redesigned in a versioned contract to use strings or scaled safe integers. Non-finite JSON tokens are rejected. The verifier pins the exact v0.2 manifest/design/configuration schemas, six-factor vocabulary, terminal dispositions, declared counts and metric semantics.
- Requires exactly one terminal result per planned run_id; rejects missing, duplicate and unexpected rows, and validates each row's design/configuration/seed/paired-block/arm/source/profile/simulator/RNG/budget identities, and requires identical initial-world and realized exogenous-event-stream digests within every paired block.
- Requires exactly 23 registered result fields and all 12 registered outcome metrics with pinned units, aggregation, direction and missing policy. Each row carries the paired-block ID, RNG contract version, exogenous schedule/stream digests, initial-world digest, budget-profile digest and a distinct policy-internal RNG namespace. Every result row carries the event-keyed RNG version, planned exogenous schedule digest, realized exogenous stream digest, initial-world-state digest, budget-profile digest, and the policy arm's isolated internal RNG namespace. These metric definitions are checked against the v0.1 contract; matching metric names with relabeled units do not pass. Each scenario's resolved_parameters must enumerate unique registered_group_ids and resource_units (resource ID to unit). The verifier requires the two groupwise outcomes to contain exactly the registered group IDs and the resource-use vector to contain exactly the registered resource IDs; dropped subgroups or resources are rejected. Every paired block must report one identical initial-world digest and one identical realized exogenous-event-stream digest across its arms; each must match the same digest in artifact_digests. Unknown is represented explicitly as JSON null; omitted metrics are errors and are never replaced by zero.
- Requires each result to list artifact SHA-256 digests and a terminal disposition. Any non-empty guardrail-violation list must be labelled guardrail_failed; guardrail-failed runs are reported but never counted among completed guardrail-clear runs. The report separately counts completed guardrail-clear runs and complete outcome vectors; any vector with unknown cells is excluded from the latter.
- Rejects unknown result fields, including attempted addition of a universal welfare scalar.
- Emits a deterministic report with contract status, coverage counts, terminal dispositions, completed guardrail-clear counts, complete outcome-vector counts, unknown outcome-cell counts, verifier package version, verifier source commit SHA, Cargo.lock SHA-256 and all verification errors. A PASS is refused if the verifier cannot embed a valid source commit and dependency-lock digest. A completed run with unknown privacy/authority guardrail evidence is not counted as guardrail-clear; a result vector can remain complete even when its terminal disposition is a declared guardrail failure.

## Known limitations

- The execution-authorized paired_design.v0.2 contract—including event-keyed RNG schedule identity and equal resource/compute-budget profile identity—still needs to be implemented by the plan generator under [CIV-IMPACT-003 / #5506](https://github.com/Luminous-Dynamics/mycelix/issues/5506), with event-stream invariance tests tracked in [#5507](https://github.com/Luminous-Dynamics/mycelix/issues/5507). The existing design-only v0.1 manifest must fail verification; no automated migration or relabelling is implied.
- The verifier does not implement the Civilization Twin, allocation policies, statistical effect estimation, model calibration or hidden-set isolation. It cross-checks event/world digest strings and their declared artifact-digest entries, but does not open/hash the trace files themselves; a trusted runner or separate artifact verifier must establish that those files actually contain the claimed events and that the simulator honored the event-keyed contract.
- It verifies declared SHA-256 digests and typed field relationships; it cannot prove that a source-controlled simulator actually produced a claimed output unless execution provenance/attestation is bound and independently trusted.
- The candidate result bundle is not itself a trusted oracle. Keep verifier/oracle information outside the candidate process in any future competitive evaluation.

## Research references

- [RFC 8785 — JSON Canonicalization Scheme](https://www.rfc-editor.org/rfc/rfc8785.html), an Informational RFC defining deterministic JSON serialization for hashing/signing.
- [serde_json_canonicalizer 0.4.1](https://docs.rs/crate/serde_json_canonicalizer/0.4.1), selected for its RFC 8785-compatible canonicalizer and published test coverage. The crate documents the IEEE-754 precision boundary; duplicate-key rejection is handled separately here.
- [NIST TEVV-Athlon Framework](https://www.nist.gov/artificial-intelligence/ai-research/tevv-athlon-framework-evaluating-ai-systems) for structured Test, Evaluation, Verification and Validation.
- [CIV-IMPACT-004 / #5507](https://github.com/Luminous-Dynamics/mycelix/issues/5507) for exogenous-event alignment across paired counterfactuals.

The verifier's report claim ceiling remains artifact-contract conformance, not model realism, causal effect or policy superiority.
