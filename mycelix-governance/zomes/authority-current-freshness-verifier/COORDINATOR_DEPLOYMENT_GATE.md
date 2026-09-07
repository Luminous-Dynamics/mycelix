# Current Freshness — Coordinator Deployment Gate v0.4

Status: **native observer candidate + signed-release authentication + pure release-currentness theorem implemented; live verifier provenance/composition/update-race admission still incomplete**

## Why this gate exists

The v0.11 current-freshness theorem binds local `DnaHash`, but coordinator WASM can change without changing that hash. Therefore `same DNA != same coordinator implementation`.

Current Holochain 0.6 Admin APIs retrieve/update coordinator definitions by exact `CellId`, so deployment attestation binds **DNA hash + cell agent public key**, not DNA alone.

## No coordinator self-attestation

A coordinator must never satisfy this gate by reporting its own compile-time hash, source commit, package version or expected deployment digest. `dna_info()`/`zome_info()` are not trustworthy current coordinator-WASM attestations to the executing coordinator.

## Native observer candidate implemented

`mycelix-authority-coordinator-native-attestor` is the candidate conductor-observation trust domain.

Its live API:

- accepts only a validated loopback admin `SocketAddr` plus exact `CellId`;
- establishes its own `AdminWebsocket` connection;
- calls `get_dna_definition(cell_id)` directly;
- enumerates the complete installed `coordinator_zomes` set;
- obtains every exact coordinator `WasmHash` and preserves raw 39-byte identity;
- samples host time only after the conductor response and hash extraction complete; and
- constructs the fixed five-second observation reuse window itself.

The public live API accepts no caller-supplied observation time, validity horizon, approved release requirement or match result.

This narrows provenance to a direct native loopback Admin API query, but it does **not** prove conductor binary/OS/socket ownership integrity. A malicious local process remains outside the theorem.

## Signed release authentication theorem implemented

`mycelix-authority-coordinator-release` defines the independent expected-release side.

`CoordinatorReleaseManifest` commits exact DNA identity, the complete approved coordinator name + `WasmHash` closure, DNA bundle/source/lock/toolchain/build-recipe/SBOM digests, source/build references and exact release-policy identity.

A signature-proof receipt can be cross-bound locally to that exact manifest, producing non-deserializable `QualifiedCoordinatorReleaseRequirement` with a lease no wider than the signature proof or release lifetime.

The release is deliberately DNA/code scoped. The installation-specific cell agent is selected independently when an authenticated release is specialized to a target `CellId` requirement.

The pure theorem still does **not** prove its signature-proof receipt originated from the designated cryptographic verifier. That remains a separate live verifier boundary.

## Pure release currentness / withdrawal theorem implemented

`mycelix-authority-coordinator-release-currentness` now separates two additional facts:

1. an independently verified **current release-registry head**; and
2. an independently verified status of the **exact authenticated release at that exact head**.

The local join requires exact equality of:

- release-policy digest/profile;
- registry generation;
- registry-head digest/profile; and
- authenticated release manifest digest/profile.

Only `Active` qualifies. `Withdrawn` and `Superseded` deny.

An `Active` proof for an older or different registry head also denies, even if it is otherwise cryptographically valid.

The current release lease is capped by the minimum of authenticated-release, current-head and status-proof horizons.

## Currentness verifier provenance/completeness is still separate

The pure currentness theorem cannot prove that either deserializable receipt came from its designated verifier.

The current-head verifier must independently prove registry completeness/currentness; the status verifier must independently prove the exact release status under that head. Neither may infer currentness from local DHT/cache "latest" heuristics or absence of later records.

Therefore:

`signed authenticity != current registry head != exact status at head != verifier provenance`.

## Exact installed set, not required-subset matching

The conductor-observed installed coordinator set must equal the current authenticated approved release set exactly by zome name + `WasmHash` when the later composer runs the #262 pure matcher.

Missing, substituted, duplicate or any **unexpected coordinator** denies. Unexpected executable code cannot be ignored simply because every expected verifier is present.

## Observation freshness is bounded but not atomicity

One conductor observation may be reused for at most five seconds. That is not proof that `UpdateCoordinators` cannot occur inside the window.

A future effect path must re-observe at admission and qualify an explicit **update-race / atomicity policy** so code cannot change between attestation and effect admission without denial/restart.

## Consumer rule

A future lifecycle/effect consumer needs all of these independently:

1. fresh v0.11 operational currentness from a direct local verifier call;
2. exact stable/dynamic deployment evidence + lease;
3. direct native coordinator observation for the exact target CellId;
4. authenticated coordinator release semantics;
5. independently proven release-signature verifier provenance;
6. independently proven current release-registry head/completeness;
7. independently proven exact release status at that head;
8. exact whole-set matching through #262;
9. qualified update-race / atomicity policy; and
10. later lifecycle/executor/effect-safety authority.

The v0.11 receipt cannot substitute for items 3-9 because it was emitted by coordinator code whose identity is being checked. Caller-supplied coordinator/head/status/signature-proof receipts cannot substitute for the independent native/cryptographic/currentness verifier boundaries.

## Provisioning state

The observer, signed-release theorem and pure currentness join still do not satisfy this gate.

Until the native observer is runtime-qualified against a real conductor, the release-signature verifier provenance is qualified, registry-head completeness/currentness is independently qualified, exact status-at-head provenance is qualified, observation+current-release composition is qualified and update-race semantics are bound to effect admission:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no effect-capable consumer may interpret current deployment evidence as coordinator-code-attested authority; and
- external effects remain disabled.

## Qualification required to satisfy the gate

Remaining work includes: real-conductor native observer qualification; independent release-signature verifier; current registry-head verifier/completeness theorem; exact status-at-head verifier; exact target-cell specialization; exact current-release + observation matching composition; before/after or equivalent coordinator-update race fencing; binding matched coordinator deployment into final effect admission; and adversarial forged/wrong-cell/stale/over-wide/missing/substituted/extra/forged-release/old-head/withdrawn/superseded/local-endpoint-impersonation/race tests.
