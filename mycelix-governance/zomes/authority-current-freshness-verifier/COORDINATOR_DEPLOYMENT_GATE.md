# Current Freshness — Coordinator Deployment Gate v0.5

Status: **native observer candidate + signed-release authentication + pure release-currentness + pure exact deployment composition implemented; live provenance/update-race/effect admission still incomplete**

## Why this gate exists

The v0.11 current-freshness theorem binds local `DnaHash`, but coordinator WASM can change without changing that hash. Therefore `same DNA != same coordinator implementation`.

Current Holochain 0.6 Admin APIs retrieve/update coordinator definitions by exact `CellId`, so deployment attestation binds **DNA hash + cell agent public key**, not DNA alone.

## No coordinator self-attestation

A coordinator must never satisfy this gate by reporting its own compile-time hash, source commit, package version or expected deployment digest. `dna_info()`/`zome_info()` are not trustworthy current coordinator-WASM attestations to the executing coordinator.

## Native observer candidate implemented

`mycelix-authority-coordinator-native-attestor` is the candidate conductor-observation trust domain.

Its live API accepts only a validated loopback admin `SocketAddr` plus exact `CellId`, establishes its own `AdminWebsocket`, calls `get_dna_definition(cell_id)`, enumerates the complete installed coordinator set, extracts every exact `WasmHash`, samples time only after observation/extraction and owns the fixed five-second reuse cap.

The observer does not accept an approved release requirement or perform release/deployment matching.

## Signed release authentication theorem implemented

`mycelix-authority-coordinator-release` defines the independent expected-release side.

Its manifest commits exact DNA identity, the complete approved coordinator name + `WasmHash` closure, DNA bundle/source/lock/toolchain/build-recipe/SBOM digests, source/build references and exact release-policy identity.

The release remains DNA/code scoped. It does not choose an installation-specific cell agent.

The pure theorem still does **not** prove its signature-proof receipt originated from the designated cryptographic verifier.

## Pure release currentness / withdrawal theorem implemented

`mycelix-authority-coordinator-release-currentness` requires both an independently verified current release-registry head and independently verified status of the exact authenticated release at that exact head.

Only `Active` qualifies. `Withdrawn`, `Superseded`, an old-head `Active` proof, alternate v0.1 registry-head/status-record profiles, or release-policy mismatch deny.

Its lease is no wider than authenticated-release, current-head or status-proof validity.

## Pure exact target/release/observation composition implemented

`mycelix-authority-coordinator-deployment-composer` now joins three separate facts:

1. `TargetCellSelection` — exact DNA hash + exact target agent + selection reference;
2. `QualifiedCurrentCoordinatorRelease` — exact currently Active approved DNA/code release; and
3. `ObservedCoordinatorDeployment` — exact observed installed coordinator set for one CellId.

The composer first requires target DNA == release DNA, specializes the current release to the independently supplied target agent, then calls #262 `match_required_coordinator_deployment` for exact whole-set equality.

The positive `QualifiedCoordinatorDeploymentComposition` is non-deserializable and commits current-release identity, target-selection identity, specialized requirement identity, #262 match identity and the minimum release/observation evidence window.

This theorem proves equality/composition only. It does **not** prove target-selection provenance, conductor-observation provenance or release/head/status verifier provenance.

Therefore:

`target selection != approved current release != observed deployment != provenance`.

## Exact installed set, not required-subset matching

The conductor-observed installed coordinator set must equal the current authenticated approved release set exactly by zome name + `WasmHash` under the exact selected CellId.

Missing, substituted, duplicate or any **unexpected coordinator** denies through #262.

## Observation freshness is bounded but not atomicity

One conductor observation may be reused for at most five seconds. That is not proof that `UpdateCoordinators` cannot occur inside the window.

A future effect path must re-observe at admission and qualify an explicit **update-race / atomicity policy** so coordinator code cannot change between attestation and effect admission without denial/restart.

## Consumer rule

A future lifecycle/effect consumer needs all of these independently:

1. fresh v0.11 operational currentness from a direct local verifier call;
2. exact stable/dynamic deployment evidence + lease;
3. independently established exact target CellId selection;
4. direct native coordinator observation for that exact target CellId;
5. authenticated coordinator release semantics;
6. independently proven release-signature verifier provenance;
7. independently proven current release-registry head/completeness;
8. independently proven exact release status at that head;
9. exact target/current-release/observation composition through the pure composer + #262;
10. qualified coordinator-update race / atomicity policy; and
11. later lifecycle/executor/effect-safety authority.

Caller-supplied target, coordinator, head, status or signature-proof bytes cannot substitute for the independent live provenance boundaries.

## Provisioning state

The pure composition theorem still does not satisfy this gate.

Until the native observer is runtime-qualified against a real conductor, target-selection provenance is qualified, release-signature verifier provenance is qualified, registry-head completeness/currentness is independently qualified, exact status-at-head provenance is qualified, the live composer obtains those facts from their designated origins, and update-race semantics are bound to effect admission:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no effect-capable consumer may interpret current deployment evidence as coordinator-code-attested authority; and
- external effects remain disabled.

## Qualification required to satisfy the gate

Remaining work includes: real-conductor native observer qualification; trusted target-cell selection provenance; independent release-signature verifier; current registry-head verifier/completeness theorem; exact status-at-head verifier; live provenance-preserving composition; before/after or equivalent coordinator-update race fencing; binding matched coordinator deployment into final effect admission; and adversarial forged/wrong-cell/stale/over-wide/missing/substituted/extra/forged-release/old-head/withdrawn/superseded/target-substitution/local-endpoint-impersonation/race tests.
