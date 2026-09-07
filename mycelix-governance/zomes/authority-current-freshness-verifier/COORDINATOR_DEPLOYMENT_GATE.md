# Current Freshness — Coordinator Deployment Gate v0.6

Status: **native observer candidate + signed-release authentication + pure release-currentness + exact deployment composition + pure pre/post stability fence implemented; live provenance/atomic effect admission still incomplete**

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

The release remains DNA/code scoped and does not choose an installation-specific cell agent.

## Pure release currentness / withdrawal theorem implemented

`mycelix-authority-coordinator-release-currentness` requires both an independently verified current release-registry head and independently verified status of the exact authenticated release at that exact head.

Only `Active` qualifies. `Withdrawn`, `Superseded`, an old-head `Active` proof, alternate v0.1 registry-head/status-record profiles, or release-policy mismatch deny.

Its lease is no wider than authenticated-release, current-head or status-proof validity.

## Pure exact target/release/observation composition implemented

`mycelix-authority-coordinator-deployment-composer` joins three distinct inputs:

1. `TargetCellSelection` — exact DNA hash + exact target agent + selection reference;
2. `QualifiedCurrentCoordinatorRelease` — exact currently Active approved DNA/code release; and
3. `ObservedCoordinatorDeployment` — exact observed installed coordinator set for one CellId.

The composer requires target DNA == release DNA, specializes the release to the independently supplied target agent, then delegates whole-set equality to #262.

The non-deserializable positive composition commits current-release identity, target-selection identity, exact requirement identity, #262 match identity and the minimum release/observation lease.

## Pure pre/post coordinator stability fence implemented

`mycelix-authority-coordinator-stability-fence` now adds a second exact observation around one named admission attempt.

Positive fencing requires:

```text
pre QualifiedCoordinatorDeploymentComposition
        +
CoordinatorAdmissionSubject(subject digest/profile + per-attempt nonce/ref)
        +
strictly later post ObservedCoordinatorDeployment
        ↓
reconstruct exact same release/target requirement
        ↓
#262 exact post whole-set match
        ↓
QualifiedCoordinatorStabilityFence
```

The post observation must be strictly later than the pre observation and must occur inside the pre-composition evidence window. The pre composition must still be live at qualification.

The reconstructed requirement identity and post CellId must equal the pre composition's exact requirement/target. Missing, substituted, duplicate or unexpected coordinator code continues to deny through #262.

The fence identity commits the pre composition, post match, exact admission subject/attempt nonce and final evidence window.

Changing the admission attempt nonce changes fence identity so one stability interval is not anonymous reusable evidence for unrelated admission attempts.

## Stability is still not atomicity

Two matching exact observations establish **no detected coordinator deployment change across that observed interval** under their evidence assumptions.

They do **not** prove that `UpdateCoordinators` cannot run immediately after the post observation. The pure fence provides no conductor mutex, generation lock, transaction or OS-level update exclusion.

The future native admission path must establish that pre/post observations genuinely bracket the admission work, own the admission nonce, and define what happens if coordinator update can race after the final observation.

## Live provenance remains separate

None of the pure composition/fence theorems prove that:

- target selection came from trusted host/application policy;
- conductor observations came from the #264 native observer;
- release signature proof came from the designated cryptographic verifier;
- registry-head/status proofs came from their designated live verifiers; or
- the admission subject/nonce came from the native effect-admission orchestrator.

Therefore:

`target selection != current approved release != observed deployment != stability interval != live provenance != atomic effect admission`.

## Consumer rule

A future lifecycle/effect consumer needs all of these independently:

1. fresh v0.11 operational currentness from a direct local verifier call;
2. exact stable/dynamic deployment evidence + lease;
3. independently established exact target CellId selection;
4. authenticated/current coordinator release from designated live verifier roles;
5. native pre coordinator observation for that exact CellId;
6. exact target/current-release/pre-observation composition through #290/#262;
7. native post coordinator observation for the same exact CellId;
8. subject/attempt-bound stability fencing through the stability theorem;
9. an explicit native policy for coordinator-update races after the final observation;
10. later lifecycle/executor/effect-safety authority; and
11. an effect path whose actual execution cannot be replaced by caller-supplied serialized positive receipts.

Caller-supplied target, coordinator, head, status, signature, stability or admission-subject bytes cannot substitute for the independent live provenance/orchestration boundaries.

## Provisioning state

The pure stability theorem still does not satisfy this gate.

Until the native observer is runtime-qualified against a real conductor, target-selection provenance is qualified, release-signature/head/status verifier provenance is qualified, a native orchestrator obtains and brackets the pre/post observations around one admission attempt, coordinator-update race semantics are qualified, and the final lifecycle/effect path consumes only in-process positive results:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no effect-capable consumer may interpret current deployment evidence as coordinator-code-attested atomic authority; and
- external effects remain disabled.

## Qualification required to satisfy the gate

Remaining work includes: real-conductor native observer qualification; trusted target-cell selection provenance; independent release-signature verifier; current registry-head verifier/completeness theorem; exact status-at-head verifier; live provenance-preserving composition; native pre/post admission orchestration; an explicit post-observation update-race/atomicity policy; binding the subject-specific stability result into final lifecycle/effect admission; and adversarial forged/wrong-cell/stale/over-wide/missing/substituted/extra/forged-release/old-head/withdrawn/superseded/target-substitution/replayed-attempt/local-endpoint-impersonation/update-race tests.
