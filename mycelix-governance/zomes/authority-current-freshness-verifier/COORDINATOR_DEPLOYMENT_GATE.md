# Current Freshness — Coordinator Deployment Gate v0.2

Status: **native observer candidate implemented; full provisioning/effect-admission prerequisite not yet satisfied**

## Why this gate exists

The v0.11 current-freshness theorem binds local `DnaHash`, but coordinator WASM can change without changing that hash. Therefore `same DNA != same coordinator implementation`.

Current Holochain 0.6 Admin APIs retrieve/update coordinator definitions by exact `CellId`, so deployment attestation binds **DNA hash + cell agent public key**, not DNA alone.

## No coordinator self-attestation

A coordinator must never satisfy this gate by reporting its own compile-time hash, source commit, package version or expected deployment digest. `dna_info()`/`zome_info()` are not trustworthy current coordinator-WASM attestations to the executing coordinator.

## Native observer candidate implemented

`mycelix-authority-coordinator-native-attestor` is now the candidate conductor-observation trust domain.

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

## Approved release requirement remains independent

The expected **complete approved coordinator set** still must come independently from an authenticated release/deployment policy.

The native observer deliberately does not import `RequiredCoordinatorDeployment` or call `match_required_coordinator_deployment`. Observation provenance and release-policy authenticity meet only in a later trusted composer.

## Exact installed set, not required-subset matching

The observed installed coordinator set must equal the approved release set exactly by zome name + `WasmHash` when the later composer runs the #262 pure matcher.

Missing, substituted, duplicate or any **unexpected coordinator** denies. Unexpected executable code cannot be ignored simply because every expected verifier is present.

## Observation freshness is bounded but not atomicity

One observation may be reused for at most five seconds. That is not proof that `UpdateCoordinators` cannot occur inside the window.

A future effect path must re-observe at admission and qualify an explicit **update-race / atomicity policy** so code cannot change between attestation and effect admission without denial/restart.

## Consumer rule

A future lifecycle/effect consumer needs all of these independently:

1. fresh v0.11 operational currentness from a direct local verifier call;
2. exact stable/dynamic deployment evidence + lease;
3. direct native coordinator observation for the exact target CellId;
4. an authenticated approved release requirement;
5. exact whole-set matching through #262;
6. qualified update-race / atomicity policy; and
7. later lifecycle/executor/effect-safety authority.

The v0.11 receipt cannot substitute for items 3-6 because it was emitted by coordinator code whose identity is being checked. A caller-supplied coordinator snapshot cannot substitute for the native observer.

## Provisioning state

The observer candidate alone does not satisfy this gate.

Until the native observer is runtime-qualified against a real conductor, the authenticated release requirement exists, the observation+requirement composer is qualified and update-race semantics are bound to effect admission:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no effect-capable consumer may interpret current deployment evidence as coordinator-code-attested authority; and
- external effects remain disabled.

## Qualification required to satisfy the gate

Remaining work includes: real-conductor native observer qualification; authenticated approved release closure; exact whole-set match composition; before/after or equivalent coordinator-update race fencing; binding matched coordinator deployment into final effect admission; and adversarial forged/wrong-cell/stale/over-wide/missing/substituted/extra/local-endpoint-impersonation/race tests.
