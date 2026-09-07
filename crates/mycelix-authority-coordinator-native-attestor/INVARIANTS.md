# Authority Coordinator Native Attestor v0.1 — Normative Invariants

Status: **native observation candidate; no release-policy authentication, deployment matching, effect admission or provisioning**

## 1. This trust domain is native, not zome/WASM

Coordinator code identity cannot be established by the coordinator whose code is being checked. This crate therefore runs outside zome/WASM authority and uses the Holochain Admin API.

## 2. The live API creates its own admin connection

`observe_local_coordinator_deployment` does not accept an already-created `AdminWebsocket`. It receives a validated `LocalAdminEndpoint` plus exact `CellId`, creates the admin connection itself and then calls `get_dna_definition(cell_id)`.

This establishes that the adapter itself performed the Admin API query. It does not prove conductor-process integrity.

## 3. Admin endpoint is loopback-only

`LocalAdminEndpoint` rejects port zero and every non-loopback address. The attestor must not silently widen its trust boundary to a network-reachable admin interface.

Loopback is still a host trust assumption: another malicious local process could impersonate the endpoint. OS/process/conductor integrity remains outside this theorem.

## 4. Exact CellId is preserved

The returned observation copies exact raw 39-byte `DnaHash` and exact raw 39-byte cell agent public key from the requested `CellId`.

The attestor does not replace CellId with DNA-only identity.

## 5. Complete installed coordinator set is returned

The adapter enumerates every entry in `DnaDef.coordinator_zomes`, obtains each exact coordinator `WasmHash` through `get_wasm_zome_hash`, and returns every name/hash pair.

It must not filter to a caller-selected or security-critical subset. A non-WASM/unrepresentable coordinator causes denial rather than omission.

## 6. Exact Holochain hash bytes are preserved

DNA, agent and coordinator identities use `get_raw_39()` and are projected without alternate encoding or re-hashing.

## 7. Observation time follows evidence production

`observed_at_ms` is sampled only after `get_dna_definition` succeeds and the complete coordinator hash extraction finishes.

The caller cannot supply observation time or validity horizon to the public live API.

## 8. Reuse horizon is owned by the attestor contract

`valid_until_ms = observed_at_ms + MAX_OBSERVATION_REUSE_MS` is constructed internally. Overflow denies.

The observer then runs the pure `ObservedCoordinatorDeployment::validate` contract before returning.

## 9. Observation and approved release policy remain separate

This crate does not import, receive, authenticate or match `RequiredCoordinatorDeployment`. It does not call `match_required_coordinator_deployment`.

The expected release closure must come from an independent authenticated release/build policy and meet this observation only in a later trusted composer.

## 10. Source reference is audit metadata, not provenance proof

The source profile/reference records which adapter path was used and which loopback endpoint/cell was targeted. Those strings are not authority. The live function's direct native control flow is the provenance boundary.

## 11. Local Admin API use does not prove conductor integrity

A successful query proves only that the native adapter interacted with the Admin protocol at the validated loopback endpoint. It does not prove the conductor binary, OS, local process table, socket ownership or host kernel are uncompromised.

Those stronger platform-attestation questions remain outside this tranche.

## 12. Coordinator-update race remains open

The observation is a point-in-time snapshot. The five-second reuse cap does not make `UpdateCoordinators` atomic with a subsequent currentness call or external effect.

A later native effect-admission theorem must define before/after re-observation or equivalent locking/atomicity semantics.

## 13. No provisioning/effect authority

This crate does not modify `dna.yaml`, enable authority zomes, call app currentness, execute lifecycle actions or perform external effects.

The #262 provisioning gate remains unsatisfied until release-requirement authenticity, native observation qualification, deployment matching and update-race/effect binding are complete.
