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
