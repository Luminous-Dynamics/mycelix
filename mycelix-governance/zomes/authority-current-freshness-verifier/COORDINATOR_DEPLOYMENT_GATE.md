# Current Freshness — Coordinator Deployment Gate v0.16

Status: **release-key trust, release authentication/currentness, durable complete registry state, exact target provenance, Linux conductor-process fencing and native process-bound target/code observation composition now exist as candidates; admission-subject ownership and atomic effect admission remain incomplete**

## Core separation

The deployment path treats these as independent facts:

```text
operational authority
!= approved release
!= release-registry currentness
!= target CellId
!= Admin endpoint
!= process owning that endpoint
!= installed coordinator code
!= observation provenance
!= admission attempt
!= effect authority
```

`same DNA != same CellId != same conductor process != same coordinator implementation`.

## Release authentication/currentness

The release side remains:

```text
independent key-policy root
→ #341 offline hybrid root/currentness
→ #347 crash-durable latest key-policy state
→ manifest-bound signing key
→ #326 strict Ed25519 AND ML-DSA-65 authentication
→ authenticated release

independent registry root
→ #355 complete canonical registry
→ #365 crash-durable latest complete snapshot
→ local exact Active lookup
→ #275 current release
```

Release signing keys cannot authorize their own key-policy root. Registry status is derived locally from one complete authenticated snapshot rather than a detached status oracle.

The trusted filesystem stores remain crash/restart continuity theorems, not same-UID arbitrary-write or full-machine rollback resistance.

## Exact target CellId provenance

#373 pins an exact raw `DnaHash` + `AgentPubKey` CellId and loopback Admin endpoint out of band.

Normal live qualification accepts neither target nor endpoint from the caller. It calls Holochain `list_cell_ids()` directly, requires the pinned CellId to be live exactly once, samples time after the Admin response, durably advances its clock floor and only then returns non-deserializable `QualifiedTargetCellSelection`.

There is no in-band retargeting in v0.1.

## Linux conductor process/listener fence

#381 independently pins the process policy for that endpoint: expected effective UID, absolute executable path, complete executable-byte BLAKE3 digest and binding identity.

The process fence resolves the exact unique LISTEN socket from `/proc/net/tcp{,6}`, joins its socket inode to the unique `/proc/<pid>/fd` owner, commits PID + process start time + effective UID + executable path/digest/dev/inode/length, opens a pidfd, and requires an identical second pre-snapshot before the interval begins.

Fence close requires the pidfd still alive and the exact same listener/process/executable snapshot. Trusted state is durably advanced before non-deserializable `QualifiedConductorProcessFence` escapes.

The fence is historical only: it has `started_at_ms` / `ended_at_ms` and deliberately no future `valid_until_ms`.

It is not TCP peer authentication and does not claim same-UID/root/kernel compromise resistance, transient same-process exec-and-restore ABA resistance or full-machine anti-rollback.

## Native process-bound observation composition

`mycelix-authority-coordinator-native-observation-composer` now owns the provenance sequence that was previously only a future requirement:

```text
#381 begin process fence
→ read #373 state-owned target binding
→ require target/process Admin endpoints equal
→ #373 direct live-target qualification
→ decode exact typed CellId locally
→ #264 PRE get_dna_definition coordinator observation
→ #264 POST get_dna_definition coordinator observation
→ #381 finish process fence
→ require exact process/listener snapshot survived
→ require target/PRE/POST timestamps inside the same interval
→ require all short evidence horizons still cover fence close
→ non-deserializable historical observation bundle
```

The public live function accepts only the already-provisioned process and target stores. It accepts no caller-supplied CellId, endpoint, PID/process snapshot, coordinator observation, timestamp or horizon.

### Exact typed target

The qualified target's raw 39-byte DNA and agent identities are decoded with typed Holochain hash constructors and combined into one `CellId` locally. Malformed or wrong-type hash bytes deny rather than being sent to the conductor unchecked.

Both #264 calls use that exact typed CellId and the exact endpoint from the open process fence.

### Causal observation interval

A positive bundle requires:

```text
process fence start
<= target observed_at
<= PRE code observed_at
<  POST code observed_at
<= process fence end
```

Equal PRE/POST millisecond timestamps deny; the adapter never synthesizes artificial time ordering.

The POST observation must also occur inside the PRE observation's own short reuse window.

### Lease containment at fence close

The target, PRE code observation and POST code observation must all still be live when #381 closes the process fence.

Both code observations are validated at `process_fence.ended_at_ms`.

This preserves the general rule:

`composition may preserve/shorten evidence lifetime; it may never widen it`.

### Exact CellId agreement

Both coordinator observations must name the exact DNA hash + agent key in #373's qualified target. Same code under another cell identity is insufficient.

### Historical bundle only

`QualifiedProcessBoundCoordinatorObservations` contains the non-deserializable #373 target capability, PRE and POST #264 observations, and the non-deserializable #381 process fence.

It intentionally has **no `valid_until_ms` of its own**.

This is important: completing a process fence must not create the fiction that the conductor process remains trusted for another five seconds. Downstream admission must immediately consume the retained underlying short-lived evidence and still solve the post-fence race separately.

The bundle digest commits target qualification identity, canonical PRE/POST observation identities, process-fence qualification identity and exact fence start/end timestamps.

## Release/deployment equality remains downstream

The native observation composer deliberately does not consume a release and does not invoke #262/#290/#298.

#290/#262 remain the exact equality theorem:

```text
current authenticated Active release
+ exact target
+ exact observed coordinator closure
→ exact whole-set deployment match
```

Missing, substituted, duplicate or unexpected coordinator code denies.

The next live admission layer should project #373's inner target only locally from the process-bound positive bundle; caller-supplied deserializable `TargetCellSelection` must not regain authority.

## Subject-bound stability remains downstream

#298 already proves that one exact pre-deployment composition plus a strictly later exact observation still matches the same release/CellId and binds the result to an admission subject + attempt nonce.

The native process-bound composer now supplies the provenance context #298 was missing, but it intentionally does not choose or authenticate the admission subject.

The next orchestrator must own the subject/attempt identity and consume the bundle directly through #290/#298.

## The remaining race is now explicit

Even a successful process-bound PRE/POST observation bundle proves only the historical interval before #381 closes.

Immediately afterward any of these can still occur:

- target cell stops/restarts;
- conductor process is replaced;
- `UpdateCoordinators` changes coordinator code; or
- release/operational authority expires or is withdrawn.

Therefore:

```text
process-bound native observations
!= atomic external-effect admission
```

The final effect path must either own a stronger exclusion/transaction boundary or perform a final fail-closed revalidation immediately adjacent to an effect mechanism whose race semantics are explicitly qualified.

## Remaining independent boundaries

The dominant unresolved boundaries are now:

- production delivery/protection of initial key-policy, registry, target and process-binding fingerprints;
- native admission-subject/attempt ownership;
- direct local #290/#298 consumption of the process-bound observation bundle;
- post-fence target/conductor/coordinator-update atomicity with the effect;
- lifecycle/executor/effect-safety authority; and
- stronger same-UID/full-machine rollback anchoring if required by the deployment threat model.

## Consumer rule

A future effect-capable path needs independently:

1. fresh operational authority currentness;
2. production-rooted + durably current release-key policy;
3. #326 authenticated release;
4. production-rooted + durably latest complete registry yielding #275 current Active release;
5. one process-bound native observation bundle that directly owns #373 target + PRE/POST #264 observations inside #381;
6. native subject/attempt ownership;
7. local #290/#262 exact release/target/PRE-code composition;
8. local #298 exact POST-code stability qualification;
9. explicit target/conductor/coordinator-update/effect atomicity;
10. lifecycle/executor/effect-safety authorization; and
11. an effect path consuming only in-process positive qualifications.

## Provisioning state

The v0.16 native-observation candidate does **not** satisfy the complete deployment/effect gate.

Until production pin provenance, admission-subject ownership and update/effect atomicity are qualified:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no consumer may interpret the process-bound observation bundle as future process or atomic external-effect authority; and
- external effects remain disabled.

## Highest-value next work

1. native admission orchestrator that owns subject + fresh attempt nonce and consumes the process-bound bundle directly through #290/#298;
2. explicit target-liveness + conductor-replacement + coordinator-update/effect atomicity boundary;
3. final lifecycle/executor/effect-safety binding; and
4. optional keyed/TPM/hardware/enterprise rollback anchors for the trusted state stores.
