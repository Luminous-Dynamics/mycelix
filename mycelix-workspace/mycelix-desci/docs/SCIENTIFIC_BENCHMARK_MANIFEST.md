# Scientific Benchmark Manifest v1

**Status:** review draft. This document specifies a typed run record, not a claim that any experiment has passed.
**Implementation:** `src/core/src/scientific_benchmark_manifest.rs`
**Module:** `mycelix_desci_core::scientific_benchmark_manifest`

## Why this exists

Scientific evidence must be more structured than "the job passed" or "the author says it worked." A benchmark manifest should bind an observed result to its exact inputs, source revision, execution environment, reference comparison and reproduction status.

The manifest complements rather than replaces the canonical append-only scientific-event kernel. Publish the serialized manifest as a content-addressed `EvidenceArtifact`, attach it to an appropriate `ComputationalReproduction` or `IndependentReplication` attestation, and use the existing signed event and receipt-time authority path to establish who published or verified it. Do not use the legacy mutable claim/trust API as the canonical record.

## Required invariants

- Versioned protocol identifier and schema version; unknown struct fields are rejected during deserialization.
- A full 40- or 64-character hexadecimal source commit, repository URI, toolchain, target, and exact reproduction command.
- Explicit input and output artifact locators, media types, byte sizes and BLAKE3 content hashes.
- Scalar metrics must be finite. If a reference value is declared, an absolute tolerance is mandatory and must be finite and non-negative.
- A result outside a declared tolerance cannot be labelled locally verified or independently reproduced. It must be reported as `reference_mismatch`.
- Completed outcomes must bind inputs, outputs and metrics to a clean committed source tree.
- Independent reproduction requires a reference plus a verifier record whose actor and organization differ from the producer. The canonical identity/governance system must still authenticate those identities; this struct alone is not proof of independence.
- Failed executions require a completion timestamp and an explanatory limitation.
- Files are hashed only after the caller supplies an explicit local path. The implementation never downloads remote locators or treats a URI as a filesystem path.
- A cryptographic digest proves byte integrity, not scientific validity.

## Status semantics

| Status | Meaning | Minimum conditions |
|---|---|---|
| `proposed` | Benchmark planned | No completion timestamp or verifier |
| `not_run` | Explicitly not executed | No completion timestamp or verifier |
| `locally_verified` | Author-side run satisfies its declared checks | Clean commit, completion time, inputs, outputs and finite metrics |
| `independently_reproduced` | A separate actor and organization recorded reproduction | All local requirements, a reference and an independently identified verifier |
| `reference_mismatch` | At least one comparable metric exceeded its fixed tolerance | Reference, completion time, inputs, outputs, metrics and clean commit |
| `execution_failed` | Execution finished without a valid benchmark result | Completion time, clean commit and an explanatory limitation |

These statuses are evidence labels. They do not supersede event-signature verification, authority receipts, governance policy, or domain-specific scientific review.

## Example: cosmology background benchmark

A Symthaea background-distance test can use a stable identifier such as:

- Benchmark: `cosmology.background.eds.z1.v1`
- Source revision: exact full commit SHA in `Luminous-Dynamics/symthaea`
- Input artifacts: parameter JSON, analytic reference fixture, and the exact solver/test configuration
- Output artifacts: machine-readable measured values and test report
- Metrics: \(E(z=1)\) relative error, comoving-distance relative error, and distance-reciprocity residual
- Reference: closed-form Einstein–de Sitter solution and the formula/version used to compute it
- Limitations: background FLRW only; no CMB spectra, perturbations, nonlinear structure formation or survey likelihoods

The benchmark's tolerances must be frozen before a run. A later tolerance change requires a new benchmark version or a separately documented protocol amendment; do not rewrite the old manifest to retroactively make it pass.

## Integration with the canonical evidence kernel

`manifest.encode_json()` first validates the manifest and returns the exact compact JSON bytes to store. `manifest.to_evidence_artifact(locator, license, availability)` returns a Mycelix-DeSci `EvidenceArtifact` whose BLAKE3 content hash is calculated over those exact bytes, along with the bytes themselves. The caller stores the bytes at the explicit locator and attaches the returned artifact to a signed `ScientificEventPayload` through the existing event API. These helpers perform no network I/O and do not sign the event.

The signed event, canonical event verification, and receipt-time authority workflow remain responsible for attribution and authorization. The manifest's verifier record is not authoritative by itself.

## Local artifact verification

`verify_local_artifact(path, expected_hash)` streams a file and compares its BLAKE3 digest. `verify_local_artifact_binding(path, binding)` additionally checks byte length while streaming. The caller supplies the path deliberately; neither helper follows manifest locators.

Call `manifest.validate()` after deserializing a manifest. Deserialization confirms the data shape, not the lifecycle, numerical, reference or independence invariants.

## Next integration gates

1. Add a stable CLI command to verify a manifest and every locally available artifact binding.
2. Connect the manifest's content-addressed output to a canonical scientific event and computational-reproduction attestation.
3. Have a separate verifier replay from frozen inputs and publish a verifier manifest and signed attestation.
4. Add tampering controls: wrong bytes, wrong size, changed source commit, dirty tree, non-finite metric, relaxed tolerance, same-organization "independent" verifier, and stale/missing reference.
5. Ensure status reporting distinguishes queued/not-run, local pass, independently reproduced, mismatch, and infrastructure failure.

## What this does not claim

This schema does not execute the solver, verify a Git checkout against a commit, interpret every scientific metric's units, authorize an event, establish organizational independence on its own, or decide whether a physical theory is true. Those require separate executable checks and the canonical signed evidence workflow.
