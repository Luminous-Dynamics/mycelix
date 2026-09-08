# Authority Coordinator Native Observation Composer v0.1 — Normative Invariants

Status: **native target + coordinator observation provenance composition inside one conductor-process fence; historical only, not release/effect authority**

## 1. The composer owns provenance-producing calls

The public live function accepts only two already-provisioned local stores:

- `TrustedConductorProcessStore`; and
- `TrustedTargetCellStore`.

It accepts no caller-supplied target CellId, Admin endpoint, PID/process snapshot, coordinator observation, observation timestamp or observation horizon.

## 2. Process fencing begins before every Admin observation

The first positive step is #381 `begin_fence()`.

That establishes the pinned listener/process pre-snapshot and retains its pidfd + exclusive process-state lock until the final post observation has completed.

No #373/#264 Admin call may precede process-fence opening.

## 3. Target and process stores must pin the same endpoint

The process-fence endpoint and #373 trusted target-state endpoint must be exactly equal before live target qualification.

The composer does not choose either endpoint.

A mismatch denies before coordinator-code observation.

## 4. Target provenance is consumed as a non-deserializable capability

The composer calls #373 `qualify_live_target()` itself.

The returned `QualifiedTargetCellSelection` must preserve the exact binding digest and DNA/agent identity from the state-owned target summary.

Caller-supplied inner `TargetCellSelection` bytes are not accepted by this live function.

## 5. CellId construction is local and typed

The exact raw 39-byte DNA and agent identities from the qualified target are decoded with typed Holochain hash constructors and combined into one `CellId` locally.

Malformed/wrong hash-type raw bytes deny rather than being passed to the conductor unchecked.

## 6. Both code observations are direct #264 calls

The composer invokes `observe_local_coordinator_deployment` twice with:

- the exact process-fence endpoint; and
- the exact locally constructed qualified target CellId.

The caller cannot supply either observed deployment.

Each #264 observation remains a complete coordinator-zome/WasmHash snapshot with its own fixed short reuse horizon.

## 7. Causal order is exact

A positive composition requires:

```text
process fence start
<= target observed_at
<= pre code observed_at
<  post code observed_at
<= process fence end
```

The post code observation must also occur no later than the pre observation's own reuse horizon so immediate #298 qualification remains possible.

If millisecond clock resolution produces equal pre/post timestamps, the attempt denies and may be retried; this layer does not synthesize a fake later timestamp.

## 8. Evidence leases must cover process-fence close

At the instant #381 closes successfully, all three short-lived evidence objects must still be live:

- #373 target capability;
- #264 pre coordinator observation;
- #264 post coordinator observation.

The composer validates both coordinator observations at the process-fence end timestamp.

It never widens any of those horizons.

## 9. Exact CellId agreement is rechecked

Both code observations must name exactly the DNA hash + agent public key in the qualified #373 target.

A different DNA or agent denies even if the coordinator set itself looks identical.

## 10. Process fence closes after both code observations

#381 `finish()` occurs only after target + PRE + POST native observations have completed.

The process theorem therefore rechecks listener/PID/start-time/UID/executable identity after those Admin observations.

A changed process/listener snapshot denies before this composer can produce a positive bundle.

## 11. The composition is historical only

`QualifiedProcessBoundCoordinatorObservations` intentionally has no `valid_until_ms` field.

The bundle commits a historical interval plus the retained underlying short-lived evidence. It must not be interpreted as “the conductor process is trusted for another N milliseconds.”

A downstream admission orchestrator must immediately re-check/use the underlying target/code horizons through #290/#298 and still solve the effect-race problem separately.

## 12. Positive composition is non-deserializable

`QualifiedProcessBoundCoordinatorObservations` derives `Serialize` but not `Deserialize`.

The public live path constructs it only after direct #373/#264 calls and successful #381 fence closure.

## 13. Bundle identity commits all three trust domains

The composition digest commits:

- #373 target qualification digest/profile;
- canonical PRE coordinator observation identity;
- canonical POST coordinator observation identity;
- #381 process-fence qualification digest/profile; and
- exact process-fence start/end times.

Observation identity commits exact CellId, source profile/reference, timestamps/horizon and complete canonical coordinator code set.

## 14. No release policy in this layer

This crate does not consume an authenticated/current release and does not call #262/#290/#298.

Its output proves native provenance of observations only.

Keeping release/currentness outside prevents a historical process interval from accidentally inheriting #290's future lease semantics.

## 15. No admission subject authority in this layer

The crate does not accept or generate `CoordinatorAdmissionSubject`, subject digest or attempt nonce.

The next native admission orchestrator must own those semantics and bind this historical observation bundle into #290/#298.

## 16. No effect authority

This theorem does not prove target/process/code state remains unchanged after process-fence close.

A conductor replacement, target stop/start or coordinator update may occur immediately afterward.

Therefore this output is not lifecycle permission, executor permission or external-effect authority.

## 17. Underlying host limitations remain

The bundle inherits #373/#381 limits: owner-protected local filesystem assumptions, no same-UID arbitrary-write/process-compromise resistance, no full-machine anti-rollback, and no cryptographic TCP peer credential.

## 18. Provisioning remains blocked

No binding `dna.yaml` or external-effect path may be enabled from this theorem alone.

The next required step is a native admission orchestrator that consumes a current release plus this historical bundle, owns the subject/attempt nonce, invokes #290/#298 without exposing deserializable positive receipts, and then solves post-observation update/effect atomicity.
