# Current Freshness — Coordinator Deployment Gate v0.15

Status: **release-key trust, release authentication/currentness, durable complete registry state, exact target provenance, coordinator-code equality and a Linux conductor-process/listener stability fence now exist as candidates; native observation composition and atomic effect admission remain incomplete**

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
!= code stability
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

#373 `mycelix-authority-coordinator-target-cell` pins exact:

- raw 39-byte `DnaHash`;
- raw 39-byte `AgentPubKey`;
- loopback Admin `SocketAddr`;
- binding id/reference.

The binding is accepted only during first-use out-of-band bootstrap. Normal live qualification accepts neither target nor endpoint from the caller.

The adapter calls Holochain `list_cell_ids()` itself and requires the exact pinned CellId to be present exactly once. Observation time is sampled after the Admin response, the trusted clock floor is durably advanced, and only then does a non-deserializable `QualifiedTargetCellSelection` escape.

There is no in-band retargeting in v0.1.

## Linux conductor process/listener fence

`mycelix-authority-coordinator-conductor-process` adds an independent host theorem for the process owning one pinned Admin endpoint.

The first-use process binding commits exact:

- loopback Admin endpoint;
- expected effective process UID;
- absolute executable path;
- complete BLAKE3 executable-byte digest;
- fixed executable profile; and
- binding/provisioning identity.

The process binding has its own out-of-band pin protocol/profile. Normal fencing loads this policy from its store-owned trusted state and accepts no caller PID, listener inode, process snapshot, endpoint or timestamp.

### Listener/process resolution

For the pinned endpoint the adapter:

```text
/proc/net/tcp or /proc/net/tcp6
→ require exact unique LISTEN socket
→ obtain listener socket inode
→ /proc/<pid>/fd/* socket:[inode]
→ require exact unique owning PID
→ read /proc/<pid>/stat starttime
→ read effective UID
→ resolve/open /proc/<pid>/exe
→ hash complete executable bytes
```

The process snapshot commits listener inode, PID, process start time, effective UID, executable path/digest, and executable device/inode/length.

Numeric PID equality alone is never process identity.

### pidfd-backed interval opening

After the first procfs resolution the adapter calls Linux `pidfd_open`, checks that process for exit, and then performs a second full process/listener observation.

The two snapshots must be exactly equal before the fence interval opens.

The pidfd remains owned by the guard until fence close.

### Fence close

After caller-owned work inside the interval:

```text
pidfd still alive
→ re-resolve listener/process/executable
→ exact pre/post snapshot equality
→ pidfd still alive
→ post-observation host clock
→ durable trusted-state advancement
→ QualifiedConductorProcessFence
```

Changed socket inode, PID/starttime, UID, executable path, executable bytes or executable inode/device identity denies.

## Historical evidence only

`QualifiedConductorProcessFence` deliberately has:

```text
started_at_ms
ended_at_ms
```

and **no `valid_until_ms`**.

The fence proves only a historical process/listener interval under the stated host model. It must not become reusable future conductor authority.

A later orchestrator must prove that the exact target/code observations it relies upon occurred inside this interval and at the same endpoint.

## Process-fence limits remain explicit

This is not cryptographic TCP peer authentication. TCP loopback does not provide a Unix-domain `SO_PEERCRED` equivalent for the accepted Admin connection.

The theorem also does not claim resistance to:

- same-UID arbitrary process tampering;
- root/kernel/procfs compromise;
- full-machine rollback;
- transient same-process exec-and-restore ABA while retaining the listener; or
- a caller performing unrelated work between `begin_fence` and `finish`.

A stronger deployment may later use a supervised/private conductor transport, dedicated namespace, authenticated local proxy or OS/hardware attestation.

## Native coordinator-code observation remains separate

#264 `mycelix-authority-coordinator-native-attestor` queries the exact target CellId through Holochain Admin `get_dna_definition`, enumerates the complete coordinator zome set and preserves exact `WasmHash` identities.

It remains candidate observation provenance only. It does not choose the approved release or prove the process fence by itself.

## Exact deployment composition

#290/#262 still establish:

```text
exact target
+ current authenticated release
+ exact observed coordinator closure
→ exact whole-set deployment match
```

Missing, substituted, duplicate or unexpected coordinator code denies.

For live admission, caller-supplied deserializable `TargetCellSelection` must not regain authority; target data must be projected from #373's non-deserializable positive capability.

## Subject-bound stability

#298 adds a later exact coordinator observation and binds stability to one admission subject + attempt nonce.

It proves no detected coordinator-code change across its observed interval, but is not a mutex or transaction. Target stop/start, conductor replacement or `UpdateCoordinators` may still race after observation.

## Next native composition theorem

The next high-value layer must itself own the sequence:

```text
begin qualified conductor-process fence
→ obtain fresh #373 target capability
→ require target endpoint == process-fence endpoint
→ obtain #264 exact code observation for that exact target/endpoint
→ require target/code evidence timestamps inside process fence
→ finish process fence
→ require same process snapshot survived the observation interval
→ compose current release + target + code through #290/#262
```

The caller must not supply substitute target/process/code positive receipts.

This closes **observation provenance composition**, but still does not make the later external effect atomic with coordinator updates or conductor replacement.

## Remaining independent boundaries

The dominant unresolved boundaries are now:

- production delivery/protection of initial key-policy, registry, target and process-binding fingerprints;
- native target/process/code observation orchestration;
- native ownership of admission subject + attempt nonce;
- post-observation target-liveness/conductor-replacement/coordinator-update atomicity with the effect;
- lifecycle/executor/effect-safety authority; and
- stronger same-UID/full-machine rollback anchoring if required by the deployment threat model.

## Consumer rule

A future effect-capable path needs independently:

1. fresh operational authority currentness;
2. production-rooted + durably current release-key policy;
3. #326 authenticated release;
4. production-rooted + durably latest complete registry yielding #275 current Active release;
5. #373 pinned + freshly live exact target capability;
6. qualified Linux conductor-process fence for the same pinned endpoint;
7. #264 exact coordinator-code observation produced inside that fence for the exact target;
8. #290/#262 exact target/current-release/code composition;
9. native subject/attempt ownership and #298 stability;
10. explicit target/conductor/coordinator-update/effect atomicity;
11. lifecycle/executor/effect-safety authorization; and
12. an effect path consuming only in-process positive qualifications.

## Provisioning state

The v0.15 process-fence candidate does **not** satisfy the complete deployment/effect gate.

Until production pin provenance, native observation composition and update/effect atomicity are qualified:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no consumer may interpret process fencing or deployment stability as atomic external-effect authority; and
- external effects remain disabled.

## Highest-value next work

1. native target/process/code observation composer;
2. native admission orchestrator owning subject + attempt nonce;
3. explicit target-liveness + conductor-replacement + coordinator-update/effect atomicity boundary;
4. final lifecycle/executor/effect-safety binding; and
5. optional keyed/TPM/hardware/enterprise rollback anchors for the trusted state stores.
