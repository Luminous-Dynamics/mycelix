# Authority Coordinator Deployment Contract v0.1 — Normative Invariants

Status: **pure exact-cell/exact-approved-set matcher; native conductor provenance not yet implemented; provisioning blocker**

## 1. DNA identity is not coordinator code identity
Holochain coordinator zomes may change without changing `DnaHash`. Integrity/DNA identity and coordinator deployment identity are separate facts.

## 2. Zome self-report cannot establish coordinator code identity
A coordinator MUST NOT establish its own installed code identity by reporting a compile-time hash, version or commit. Replaced code could report the expected value.

## 3. Native conductor observation is the required provenance source
A future live adapter must obtain the exact installed definition from conductor-owned Admin API state. Current Holochain 0.6 APIs address `GetDnaDefinition`/`UpdateCoordinators` by `CellId`, so the observation and requirement bind both raw 39-byte `DnaHash` and raw 39-byte cell agent public key.

The pure crate validates shape/equality only; it cannot prove the caller actually used AdminWebsocket/conductor state.

## 4. Requirement authenticity is separate
`RequiredCoordinatorDeployment` combines the exact target cell identity with the complete independently approved coordinator set. Production composition must source target-cell identity independently from the authenticated release/deployment policy that supplies approved code hashes.

## 5. Raw Holochain hash identity is preserved
DNA, agent and coordinator hashes use exact raw 39-byte Holochain hash bytes. Zero/wrong-length values deny.

## 6. Approved coordinator set is canonical
The requirement digest commits protocol/profile, exact DNA, exact cell agent and the canonical zome-name-sorted approved coordinator name + WASM hash set. Empty/oversized sets and duplicate names deny.

## 7. Installed coordinator set must equal the approved set exactly
Every approved coordinator must exist with the exact approved `WasmHash`, and no unexpected coordinator may be installed. Missing, substituted, duplicate or extra executable code denies. There is no live-authority "required subset" mode.

## 8. Deployment match identity is dynamic evidence
`MatchedCoordinatorDeployment` commits exact requirement identity, exact CellId, conductor source profile/reference, observation window and exact approved coordinator closure. It is deployment evidence, not institutional authority.

## 9. Observation lease is explicit and bounded
`observed_at_ms <= now_ms < valid_until_ms` is mandatory and the window may not exceed `MAX_OBSERVATION_REUSE_MS = 5_000`. This is only a local reuse ceiling; it is not natural code expiry or an atomicity theorem.

## 10. Matching does not prove provenance
`ObservedCoordinatorDeployment` is deserializable. Even the non-deserializable match result proves only exact equality over provided data. It does not prove conductor origin, release-policy authenticity, conductor integrity, absence of an update race or effect permission.

## 11. No zome or caller oracle
Live authority/effect admission must obtain the observation directly through an independently trusted native host boundary. Caller-supplied or zome-self-reported snapshots never become positive code identity.

## 12. Coordinator-update races remain a separate theorem
A five-second reuse cap narrows exposure but does not make `UpdateCoordinators` atomic with a later effect. The native effect path must qualify re-observation/locking/atomicity semantics separately.

## 13. Current authority stack remains unprovisioned
Until native conductor provenance, authenticated release requirements and update-race policy are qualified, the current authority stack remains absent from binding `dna.yaml`; #251 remains host-DNA-bound but not coordinator-code-attested.

## 14. Future provisioning gate
Before `coordinator_deployment_identity_bound = true` may be claimed, qualification must prove: direct native retrieval for the exact `CellId`; exact installed coordinator `WasmHash` extraction; authenticated approved release closure; exact whole-set matching including extra-zome denial; bounded observation freshness; update-race/atomicity semantics; binding into deployment/effect admission; and adversarial missing/substituted/extra/wrong-cell/stale/forged/race tests.
