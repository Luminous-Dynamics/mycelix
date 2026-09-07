# Current Freshness — Coordinator Deployment Gate v0.3

Status: **native observer candidate + signed-release authentication theorem implemented; full provisioning/effect-admission prerequisite not yet satisfied**

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

`CoordinatorReleaseManifest` commits:

- exact DNA identity;
- complete approved coordinator name + `WasmHash` closure;
- DNA bundle digest;
- source-tree digest;
- lockfile digest;
- toolchain digest;
- build-recipe digest;
- SBOM digest;
- source/build references; and
- exact release-policy identity.

A deserializable signature-proof receipt can be cross-bound locally to that exact manifest, producing a non-deserializable `QualifiedCoordinatorReleaseRequirement` with a lease no wider than the signature proof or release lifetime.

The release is deliberately DNA/code scoped. The installation-specific cell agent is selected independently when an authenticated release is specialized to a target `CellId` requirement.

## What signed-release qualification still does not prove

The pure release theorem does **not** establish that its signature-proof receipt came from the designated cryptographic verifier. That verifier provenance still needs a direct independently qualified adapter.

It also does not prove that the signed release remains current, non-withdrawn, non-revoked or authorized under the presently binding release policy.

Therefore:

`signed release authenticity != release currentness / withdrawal status`.

A later release-currentness theorem is mandatory before live deployment/effect admission.

## Exact installed set, not required-subset matching

The conductor-observed installed coordinator set must equal the authenticated approved release set exactly by zome name + `WasmHash` when the later composer runs the #262 pure matcher.

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
5. independently proven signature-verifier provenance;
6. current/non-withdrawn release-policy state;
7. exact whole-set matching through #262;
8. qualified update-race / atomicity policy; and
9. later lifecycle/executor/effect-safety authority.

The v0.11 receipt cannot substitute for items 3-8 because it was emitted by coordinator code whose identity is being checked. A caller-supplied coordinator snapshot or signature-proof receipt cannot substitute for the independent native/cryptographic verifier boundaries.

## Provisioning state

The observer and signed-release theorem still do not satisfy this gate.

Until the native observer is runtime-qualified against a real conductor, the release-signature verifier provenance is qualified, release currentness/withdrawal is independently established, observation+requirement composition is qualified and update-race semantics are bound to effect admission:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no effect-capable consumer may interpret current deployment evidence as coordinator-code-attested authority; and
- external effects remain disabled.

## Qualification required to satisfy the gate

Remaining work includes: real-conductor native observer qualification; independent release-signature verifier; current release-policy/withdrawal theorem; exact target-cell specialization; exact whole-set match composition; before/after or equivalent coordinator-update race fencing; binding matched coordinator deployment into final effect admission; and adversarial forged/wrong-cell/stale/over-wide/missing/substituted/extra/forged-release/withdrawn-release/local-endpoint-impersonation/race tests.
