# Current Freshness — Coordinator Deployment Gate v0.1

Status: **mandatory future provisioning/effect-admission prerequisite; not yet satisfied**

## Why this gate exists

The v0.11 current-freshness theorem binds local `DnaHash`, but coordinator WASM can change without changing that hash. Therefore `same DNA != same coordinator implementation`.

Current Holochain 0.6 Admin APIs retrieve/update coordinator definitions by exact `CellId`, so deployment attestation must bind **DNA hash + cell agent public key**, not DNA alone.

## No coordinator self-attestation

A coordinator must never satisfy this gate by reporting its own compile-time hash, source commit, package version or expected deployment digest. `dna_info()`/`zome_info()` are not trustworthy current coordinator-WASM attestations to the executing coordinator.

## Required independent sources

An independently trusted native host boundary must query conductor-owned state for the exact installed `CellId` and coordinator definitions/WASM hashes.

The expected **complete approved coordinator set** must come independently from authenticated release/deployment policy. Target CellId provenance, release-policy authenticity and conductor-observation provenance remain separate inputs to the pure matcher.

## Exact installed set, not required-subset matching

The observed installed coordinator set must equal the approved release set exactly by zome name + `WasmHash`.

Missing, substituted, duplicate or any **unexpected coordinator** denies. Unexpected executable code cannot be ignored simply because every expected verifier is present.

## Observation freshness is bounded but not atomicity

The v0.1 contract caps one conductor observation at five seconds of local reuse. That is not proof that `UpdateCoordinators` cannot occur inside the window.

A future effect path must re-observe at admission and qualify an explicit **update-race / atomicity policy** so code cannot change between attestation and effect admission without denial/restart.

## Consumer rule

A future lifecycle/effect consumer needs all of these independently:

1. fresh v0.11 operational currentness from a direct local verifier call;
2. exact stable/dynamic deployment evidence + lease;
3. native-attested coordinator deployment for the exact target CellId and complete approved coordinator set;
4. qualified update-race/atomicity semantics; and
5. later lifecycle/executor/effect-safety authority.

The v0.11 receipt cannot substitute for item 3 because it was emitted by coordinator code whose identity is being checked. A caller-supplied coordinator snapshot cannot substitute for a native attestor observation.

## Provisioning state

Until native conductor/AdminWebsocket attestation, authenticated release requirements and update-race policy are qualified:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no effect-capable consumer may interpret current deployment evidence as coordinator-code-attested authority; and
- external effects remain disabled.

## Qualification required to satisfy the gate

At minimum: exact CellId retrieval; installed coordinator `WasmHash` extraction; authenticated approved release closure; exact whole-set matching including unexpected-code denial; bounded re-observation across coordinator updates; an update-race/atomicity theorem; binding into final effect admission; and adversarial forged/wrong-cell/stale/over-wide/missing/substituted/extra/race tests.
