# MCRB correctness-gate attestation profile v0.1

**Status:** Proposed assurance contract; not yet implemented or qualified  
**Tracking:** [#6346 — MCRB-GATE-ATTEST-001](https://github.com/Luminous-Dynamics/mycelix/issues/6346)  
**Related:** #6322, #6335, #6342, PR #6323

## 1. Purpose

MCRB currently has three distinct pieces of measurement evidence:

1. an offer-first workload ledger, which fixes the denominator before dispatch;
2. a metric-integrity report, which validates timing, clock identity, peer coverage and event pairing;
3. scenario-specific correctness gates, which determine whether operations actually preserved authority, privacy, recovery and effect semantics.

None of these should be allowed to self-authorize the others.

A hash-valid offer ledger proves internal consistency, not that the workload generator dispatched every promised operation. A structurally valid metric report is not trustworthy merely because a caller sets `status = Valid`. A boolean `hard_correctness_gates_passed = true` is not proof that an independent verifier evaluated the scenario's required invariants.

This profile makes the evidence chain explicit and aligns it with established software-supply-chain attestation standards rather than inventing a wholly new signed-receipt protocol.

## 2. Standards alignment

### in-toto Attestation Framework

Use the in-toto Statement / Predicate / Envelope / Bundle model:

- **Predicate:** MCRB-specific correctness-gate result, bound to run, scenario, source, runtime and evidence.
- **Statement:** binds the predicate type and subject digest(s).
- **Envelope:** authenticates and serializes the statement. DSSE v1 is the preferred envelope for interoperable payload signing.
- **Bundle:** groups multiple attestations needed to qualify a complete run.

Use Statement v1 and a versioned MCRB predicate type:

`https://luminousdynamics.io/attestations/mcrb-correctness-gate/v0.1`

Do not confuse a signature with a trust decision. Verifiers must validate the statement subject, predicate type, signer identity, allowed signer-builder pair, trust policy, signature and any required transparency evidence.

### SLSA Build Provenance

Use the SLSA Provenance v1 predicate for the build provenance of:
- the workload generator;
- the metric-integrity verifier;
- the gate verifier;
- any custom Wind Tunnel scenario binaries or benchmark adapters.

SLSA provenance should bind artifact subjects to the source revision, resolved dependencies, build parameters and builder identity. The predicate type is `https://slsa.dev/provenance/v1`.

SLSA build provenance proves how an artifact was built according to the claimed builder trust model. It does **not** prove that a runtime benchmark result is correct. Runtime measurements need their own MCRB run and correctness-gate attestations.

### DSSE and Sigstore

Use DSSE-compatible envelopes for statements that need independent signatures. The in-toto envelope specification expects an array of signatures and an authenticated payload type.

Sigstore bundles can carry signature verification material and transparency-log evidence. The current Sigstore bundle format is single-signature, so do not assume one Sigstore bundle itself represents a multi-signer quorum. For independent attestations, emit one verifiable envelope/bundle per signer and group the resulting attestations in the MCRB run bundle; alternatively use a multi-signature DSSE envelope where that is the selected interoperable implementation.

The exact signer, builder identity, certificate policy, transparency evidence and revocation rules must be pinned in the trust policy.

## 3. Evidence graph

The intended graph is:

```text
frozen scenario contract
        |
        +--> required hard-gate IDs
        |
exact source + runtime + topology + workload
        |
        +--> offer ledger -----> offer-ledger verification
        |
        +--> raw metric samples -> metric-integrity verification
        |
        +--> raw scenario evidence -> independent gate verifiers
                                      |
                                      +--> one signed attestation per required gate
                                      |
                                      v
                           VerifiedCorrectnessGateSet
                                      |
VerifiedOfferLedger + VerifiedMetricIntegrity
                                      |
                                      v
                               QOPS reducer
                                      |
                                      v
                         MCRB run receipt + in-toto Bundle
```

The runner, workload generator, metrics collector, scenario gate verifier and QOPS reducer are distinct responsibilities. A runner must not be allowed to assert its own correctness gate result as independent proof.

## 4. Correctness-gate predicate

Each gate attestation contains one terminal result for one required gate. The predicate must bind:

- run ID and scenario ID;
- exact scenario-contract digest and version;
- repository, exact source commit and tree digest;
- hApp/DNA/runtime identity and topology-profile digest;
- gate ID and result: `pass`, `fail` or `incomplete`;
- evidence artifact digest(s) and evidence type;
- verifier identity, verifier version and verifier binary digest;
- trust-policy ID and exact trust-policy digest;
- run/measurement-window identity where the gate is time-dependent.

The in-toto Statement subject must bind the canonical run or scenario evidence artifact by digest. The envelope signature authenticates the statement. The predicate must never refer only to mutable URLs, branch names, PR numbers or unversioned artifact names.

A starter predicate schema is provided at `MCRB_GATE_ATTESTATION_V0_1.schema.json`. It is a Mycelix predicate profile, not a replacement for the in-toto Statement or DSSE envelope schemas.

## 5. Required gate-set derivation

The required gate set is derived from the frozen scenario contract, not supplied by the caller after the run.

For the selected scenario, the verifier obtains the required gate IDs from the versioned scenario manifest, including its `hard_invariants` and any applicable global hard gates. The exact mapping from invariant ID to gate verifier is versioned and digest-bound.

The verifier then requires:
- every required gate ID appears exactly once, or meets an explicitly versioned multi-verifier quorum;
- no missing, duplicate or unexpected gate IDs;
- every gate result is terminal and bound to the same run, scenario, contract, source and runtime;
- every required evidence digest resolves to the captured evidence package;
- every signer is trusted for that gate and is not revoked under the pinned trust policy;
- every signer-builder pair is allowed by policy;
- no stale or superseded attestation is replayed for a different run or scenario.

Any missing, duplicate, failed, incomplete, stale, mismatched, unsigned, untrusted or unverifiable gate makes the set `invalid` or `incomplete`; it cannot produce QOPS eligibility.

## 6. Opaque verifier outputs

The Rust API should have three non-caller-constructible inputs to QOPS:

- `VerifiedOfferLedger` — produced only by the canonical offer-ledger verifier;
- `VerifiedMetricIntegrity` — produced only by the full metric batch verifier, including required cross-host checks;
- `VerifiedCorrectnessGateSet` — produced only by an attestation verifier applying the pinned trust policy.

All three should have private fields and constructors. Public display/serialization projections may be exposed, but they must not be accepted back as authority-bearing values.

This closes two distinct source-structural weaknesses:
- `QopsInput.hard_correctness_gates_passed` is a caller-supplied boolean;
- `IntegrityReport` currently exposes public fields and can be constructed by a caller rather than returned by `validate_batch`.

QOPS should derive eligibility from the opaque verifier results. A public report whose fields say Valid is not equivalent to a verifier-created capability.

## 7. Trust policy and independent verification

The trust policy is outside the runner's mutable workload inputs. It must have:
- stable policy ID and exact digest;
- authorized signer identities and allowed builder identities;
- signer-to-gate authorization mapping;
- key/certificate expiry and revocation policy;
- transparency-log requirements where applicable;
- quorum requirements for high-impact gates;
- policy version and an independent governance owner.

A valid signature only proves that a private key signed bytes. It does not prove that the signer measured the world correctly, that the evidence collector was uncompromised, or that the verifier was semantically correct.

For higher-assurance gates, use two independent verifiers with separate trust roots or implementation diversity. Keep the policy explicit; do not silently turn all gates into a single central trust authority.

## 8. Run receipt changes

When QOPS is eligible, the receipt must include:
- correctness-gate set status and complete gate counts;
- required gate-set digest and individual attestation digests;
- in-toto bundle digest;
- trust-policy ID and digest;
- verifier identity/version/artifact digest;
- SLSA build-provenance digests for the generator and verifier;
- exact source/tree/runtime/topology/contract identities;
- raw evidence artifact digests.

The QOPS status may be `eligible` only when:
- offer ledger status is Valid;
- metric-integrity status is Valid;
- cross-host status is Valid when required by the scenario;
- correctness-gate set status is Valid;
- every required scenario gate is satisfied;
- no hard correctness invariant failed.

Otherwise all performance rates are withheld or explicitly marked not evaluated. A schema-valid receipt is not itself evidence authenticity.

## 9. Adversarial tests

The exact-head verifier test corpus must include:
- a caller-supplied true boolean with no attestation;
- caller-constructed `IntegrityReport { status: Valid, ... }`;
- missing/duplicate/unexpected gate;
- wrong run, scenario, contract digest, source SHA, tree digest or runtime;
- altered evidence digest;
- altered statement payload after signing;
- unrecognized signer, revoked signer, expired certificate or stale trust policy;
- valid signature from a signer not authorized for that gate;
- signer-builder pair mismatch;
- stale run replay and superseded attestation;
- failed/incomplete gate hidden by an aggregate pass bit;
- one gate reported as pass by two conflicting signers;
- quorum count met with duplicate identity or common trust root.

The positive corpus must contain independently generated, verifier-confirmed gate attestations for every required gate in a frozen scenario.

## 10. Claim ceiling and implementation order

This is a design profile only. It does not claim that the current QOPS reducer verifies signatures or that any MCRB run has been independently attested.

Implementation order:
1. freeze the scenario-to-required-gate mapping;
2. select maintained in-toto Rust types/serialization and DSSE implementation;
3. implement the gate predicate encoder and strict verifier;
4. implement pinned trust policy and signer-builder pair checks;
5. make `VerifiedMetricIntegrity` and `VerifiedCorrectnessGateSet` opaque;
6. adapt QOPS and run-receipt emission;
7. add positive/adversarial corpus and independent exact-head verification;
8. only then allow a QOPS result to be published as eligible.

## References

- in-toto Attestation Framework v1.2: https://github.com/in-toto/attestation
- in-toto Envelope / DSSE guidance: https://github.com/in-toto/attestation/blob/main/spec/v1/envelope.md
- SLSA Build Provenance v1: https://slsa.dev/spec/v1.2/build-provenance
- Sigstore Bundle format: https://docs.sigstore.dev/about/bundle/
- Tracking issue: https://github.com/Luminous-Dynamics/mycelix/issues/6346


## 11. Source implementation snapshot — 20 November 2026

A first Rust implementation of the v0.1 verification boundary is now present in `mycelix-workspace/tools/mcrb-metric-integrity/src/attestation.rs` and `qops.rs`.

Implemented in source:

- The signed payload is an **in-toto Statement v1** with the MCRB correctness predicate nested under `predicateType` and `predicate`.
- The Statement is carried in a DSSE v1 envelope using `application/vnd.in-toto+json` and the DSSE pre-authentication encoding.
- Each in-toto Statement subject binds one evidence artifact SHA-256 digest.
- Ed25519 signatures are checked against a signer record in a versioned trust-policy document.
- The verifier checks signer key ID, signature, gate scope, builder scope, verifier identity, signer validity window, revocation flag, policy digest and exact run/scenario/contract/source/tree/runtime/topology context.
- Required gate IDs are derived from the embedded, frozen scenario manifest: every scenario hard invariant plus the global hard-gate set. A caller cannot supply a smaller required list.
- The verifier rejects missing, duplicate, unexpected, failed, incomplete, stale-context, malformed, unsigned, untrusted and digest-mismatched gate attestations.
- `VerifiedCorrectnessGateSet` is opaque with private fields and is emitted only after complete verification.
- `VerifiedMetricIntegrity` is also opaque and is emitted only by `verify_measurement_batch` after raw monotonic timing validation.
- QOPS no longer accepts `hard_correctness_gates_passed: bool` or a caller-constructed `IntegrityReport`. It requires `VerifiedOfferLedger`, `MetricIntegrityVerification` and `GateSetVerification`; it also checks the gate set's run, scenario and contract against the offer ledger.
- An unresolved `Unknown` outcome now withholds all performance rates as `Incomplete` pending reconciliation.
- The offer-ledger and attestation verifiers now use the canonical `MCRB-XX` manifest `id`, consistent with the run receipt and gate schema.

### Important limits

This is **source implementation**, not qualification. No Cargo format, compile, unit-test, Clippy, workflow or live benchmark result is claimed by this update.

The expected trust-policy digest must come from a protected configuration source outside the benchmark runner. The Rust function verifies bytes against the supplied pin; it cannot, by itself, prove who controls the configuration that supplies the pin. The authority and rotation process for that external trust root remain deployment/governance requirements.

The v0.1 verifier requires one Ed25519 signer per gate envelope. It can accept multiple independently signed gate envelopes, but a formal multi-signer quorum rule, common-trust-root analysis, transparency-log verification, SLSA provenance verification, and cryptographic revocation/status-list freshness are not yet implemented. The receipt schema reserves fields for build-provenance evidence; the current Rust code does not verify SLSA provenance.

The signature proves that a key signed an attestation. It does not prove that the verifier measured the world correctly, that evidence custody was uncompromised, or that the trust-policy owner is honest. Those are separate controls and must remain visible in the claim ceiling.

### Updated execution order

1. Run the exact-head format, unit-test and Clippy workflow for the new source.
2. Review the DSSE/in-toto serialization against the official Statement v1 and DSSE v1 specifications.
3. Supply the trust-policy pin from protected operator configuration and verify signer/builder governance independently.
4. Add independent-verifier diversity and explicit quorum policy before using high-impact correctness gates for public qualification.
5. Add SLSA provenance and transparency-log verification as separate, versioned controls.
6. Only after these steps and exact runtime evidence may any QOPS receipt be labelled eligible for an external performance claim.


## 12. Per-operation outcome authenticity remains a separate hard gate

The current QOPS outcome list is still diagnostic caller input; it is not an independently verified event ledger. This matters because a frozen denominator and signed scenario-gate set do not authenticate the numerator.

A caller could otherwise classify every offered operation as `QualifiedSuccess` without proving the operation's terminal effect or the applicable denial/failure proposition.

To prevent that from becoming a public performance claim:

- `QopsReport` now records `outcome_evidence_status = NotImplemented` and has no outcome-ledger digest/count yet.
- `compute_qops` always includes `OutcomeEvidenceNotVerified`; the report remains `Incomplete` and every QOPS rate is withheld.
- The run receipt schema requires `outcome_evidence_status = verified`, an outcome-ledger digest, positive outcome-ledger entry count and positive outcome-attestation count before `qops.status = eligible` can validate.
- The follow-on contract is tracked in [#6357 — MCRB-OUTCOME-ATTEST-001](https://github.com/Luminous-Dynamics/mycelix/issues/6357).

The next implementation must replace `QopsInput.outcomes` with an opaque `VerifiedOutcomeLedger` produced from exact offer-correlated outcome records and independent evidence. Provider acknowledgement, application API success, local Holochain commit and remote DHT observation are different propositions; the scenario's success contract must state which one is required. For external effects, compose #6140's evidence-kind, source-relationship, independence and expected-coverage rules.

**Until #6357 is implemented and qualified, this MCRB code must not emit an eligible QOPS rate.** This ceiling is deliberate even where the offer ledger, timing batch and correctness-gate attestations individually verify.


## 13. Outcome authenticity is now a frozen global hard gate

The scenario manifest now includes the global gate:

- ID: `outcome-ledger-authenticity`
- Metric: `outcome_ledger_verification_status`
- Required value: `verified`

This is separate from each scenario's domain-specific correctness invariants and separate from the timing/offer-ledger checks. The gate verifier derives it automatically from the embedded scenario manifest, so every scenario's required signed gate set includes it.

The current outcome ledger implementation is intentionally not present yet. Consequently, the MCRB reducer records `OutcomeEvidenceStatus::NotImplemented`, emits `OutcomeEvidenceNotVerified`, stays `Incomplete` and withholds rates. This is not a waived gate or a provisional PASS. #6357 owns the missing `VerifiedOutcomeLedger` that must eventually supply the evidence for this gate.
