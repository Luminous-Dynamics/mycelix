# AeroCommons adapter-run receipt — v1

Status: synthetic structural-verifier increment; no real CAD/metrology adapter is executed by this crate  
Tracker: https://github.com/Luminous-Dynamics/mycelix/issues/6354  
Related: semantic interop #6348 / PR #6349; component packet #6345 / PR #6347; change-to-test #6350 / PR #6351

## Purpose

The semantic comparator evaluates normalized assertion sets. This receipt closes the next evidentiary seam: it binds a supplied assertion set to exact raw input/output bytes, the adapter executable, a configuration, a declared invocation profile, rights dispositions and a runtime outcome.

Without this binding, a caller could submit a correct-looking normalized assertion JSON that was not derived from the claimed AP242/QIF input. The receipt is therefore a precondition for treating an adapter output as a traceable benchmark observation.

## Data flow

```text
AP242 / QIF inputs ─┐
adapter executable ─┼─> declared invocation ─> raw output + normalized assertions
configuration ─────┘             │                         │
                                 └──── run receipt ─────────┘
                                           │
                           canonical receipt digest + exact byte verification
                                           │
                         separate trust policy / signature / independent replay
                                           │
                      semantic comparator (#6349) and component lineage (#6347)
```

## V1 contract

A receipt carries:
- typed input roles and immutable AeroEvidence ArtifactRef references;
- SHA-256, byte length, media type and format profile for every input, output and adapter executable;
- adapter identifier/version and an optional reference to SLSA build provenance;
- content-addressed, byte-verifiable argument-vector and non-secret environment-profile artifacts, plus configuration bytes bound to the exact configuration input;
- locale, timezone, timeout, declared network policy and observed egress;
- raw output and normalized assertion-set artifacts;
- exit outcome, elapsed-time metadata, warnings and explicit unsupported assertion IDs;
- a versioned, canonical receipt digest.

The adapter configuration must also appear as exactly one input with role adapter_configuration, and its digest must match invocation.configuration_sha256.

## Canonical digest

V1 computes SHA-256 over UTF-8 JSON serialization of a fixed Rust struct payload, excluding receipt_sha256. The struct field order is part of the versioned profile. Input and output arrays are sorted by (role, artifact reference ID, artifact reference kind); declared/observed egress, warnings and unsupported assertion IDs are sorted. The canonicalization profile is schema-specific; it is not a claim of RFC 8785/JCS compatibility.

Known-answer digest for the checked-in synthetic receipt:
`6d1b14d1aacdaa92fc7c03fdba626191e9c4c7d1132131c88fe106161c273ea1`

The exact argument-vector and non-secret environment-profile files are part of the verified artifact set:
- arguments: `6c0ef43428d609b7a9518e8f2a06bc48886d1bbc2160734ded4fc83679e1a98a` (83 bytes)
- environment profile: `9839f815354d2ff6f0033e1161d7fce6a74f9d85eccac1ce1bfe0e80bff21c19` (60 bytes)

The argument vector is parsed as a JSON string array. The environment profile is a strict object whose locale, timezone and network-policy values must match the corresponding receipt fields. The argument and environment profiles are descriptive non-secret manifests. Do not place credentials, tokens or other secret values in them.

Every artifact reference ID is `sha256:<lowercase digest>`. The byte verifier requires an exact payload set: missing, duplicate or unexpected references fail.

## Execution status and admissibility

Failed and timed-out runs remain recordable so that the offered benchmark denominator does not silently discard failed attempts. Only status=succeeded with exit_code=0 and exactly one normalized assertion output plus at least one raw output is eligible for semantic comparison. A structurally valid timeout is not a successful conversion.

All source and output rights declarations must be CLEARED_FOR_TESTING or CLEARED_FOR_REDISTRIBUTION before this v1 verifier admits the receipt. This is a policy gate over a declared status, not an independent legal determination. Rights evidence must be reviewed by the caller.

Network-disabled runs must declare and observe no egress. Allowlisted runs must declare every observed origin. This is an accounting invariant over supplied metadata; the crate does not instrument sockets or enforce a sandbox.

## Attestation interoperability

SLSA Build Provenance v1.2 describes how an adapter executable/container was built. It is not a runtime conversion receipt and must not be repurposed as one. The optional build_provenance reference is only a typed pointer in this increment; the crate does not parse or authenticate the referenced attestation.

A future signed runtime envelope should use an in-toto Statement/DSSE profile and a separately configured signer/runner trust policy. Signature verification, publisher authorization, runner identity, sandbox enforcement, egress instrumentation and proof that a process actually ran are intentionally outside this pure structural verifier.

References:
- SLSA Build Provenance v1.2: https://slsa.dev/spec/v1.2/build-provenance
- NIST STEP File Analyzer and Viewer: https://www.nist.gov/services-resources/software/step-file-analyzer-and-viewer
- CAx-IF Recommended Practices and Test Cases 7/8: https://www.mbx-if.org/home/cax/recpractices/

## Benchmark protocol

First benchmark tranche:
1. Freeze rights-cleared CAx-IF STEP/QIF Test Cases 7 and 8 and NIST PMI/AP242 cases by exact version and SHA-256.
2. Record adapter binary/container, build provenance, configuration, arguments, environment, network policy and all raw/normalized outputs.
3. Compare semantic outputs using #6349; use NIST SFA and CAx-IF expected behavior only within their documented scopes.
4. Re-run with an independently implemented adapter path where available.
5. Keep integrity-valid, signer-authenticated, independently replayed and semantically preserved as four separate outcomes.

Report:
- exact input/output/executable byte-integrity rate;
- unbound or missing artifact count;
- undeclared egress and rights-admission rejection counts;
- successful / failed / timeout run rates;
- exact and semantic replay rates;
- semantic assertion coverage, unsupported coverage and false-preservation rate;
- parser disagreement and cross-document identity-join errors;
- p50/p95 elapsed time, peak memory and compute cost.

Prospective targets are 100% byte-integrity verification, zero accepted unbound artifacts, zero silent unsupported-semantic loss, complete egress accounting and 100% detection of the frozen negative controls. These are software contract targets, not industrial performance results.

## Comparison with the state of the art

NIST SFA already supports AP242 analysis, semantic/graphic PMI reports and recommended-practice checks. CAx-IF's vendor implementor group defines and validates exchange practices through test rounds; its downstream-use practice publishes paired STEP/QIF trace cases. We should consume and extend these ecosystems rather than duplicate their parsers or vendor conformance programme.

AeroCommons' proposed advantage is the chain that spans exact input bytes → specific adapter build/configuration/invocation → exact normalized assertions → physical-unit/evidence lineage → change-impact receipts. Compare this against manual checklists, NIST SFA, CAx-IF reference outcomes and rights-cleared commercial workflows on the same cases. Do not claim to beat CAD geometry engines, enterprise PLM breadth, or supplier QMS/MES execution.

## Claim ceiling

An integrity-checked receipt proves only that the supplied receipt and exact supplied bytes agree under the v1 contract. It does not prove that an executable ran, that its publisher is authentic, that the runner is trusted, that egress was actually observed, that a rights declaration is lawful, or that normalized assertions are semantically correct. It does not establish manufacturing capability, metrology accuracy, product acceptance, certification or airworthiness, and it never authorizes fabrication, production or flight.
