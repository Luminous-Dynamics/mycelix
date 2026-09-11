# Coordinator Admin Mutation Exclusion v0.1 — Normative Invariants

Status: **shared broker-side mutation/response exclusion theorem; actual response effect and effect-start state remain absent**

## 1. Isolation precedes meaningful mutex authority

A process-local mutex or lockfile is not coordinator-update exclusion while arbitrary host processes can bypass it through the Holochain Admin websocket.

This theorem therefore depends on #535 `BrokerProcessBinding` + `ExclusiveAdminIsolationGuard`: under its stated Linux/root assumptions, ordinary host-network processes cannot reach the isolated loopback Admin endpoint.

`file lock without #535 isolation != Admin mutation exclusion`.

## 2. One exclusion domain has one fixed path

v0.1 uses exactly:

`/var/lib/mycelix/authority/admin-mutation.lock`

The lock path is protocol semantics, not caller input.

A future operation may not choose another lock path for another Admin mutation class.

## 3. The provisioned lock inode is authority-relevant

`AdminMutationExclusionBinding` commits exact:

- #535 broker binding digest;
- fixed lock path;
- lock device;
- lock inode; and
- provisioning reference.

The lock file must exist before live operation. Runtime code does not create a replacement lock inode.

Every acquisition opens with `O_NOFOLLOW | O_CLOEXEC` and requires the open file to retain exact owner, mode `0600`, device and inode from the binding.

Path replacement therefore denies instead of silently creating split-brain exclusion.

## 4. Lock parent is owner-private and canonical

The immediate lock directory must resolve canonically to itself, must be a real directory owned by the broker effective UID and must have exact mode `0700`.

Symlinked/non-canonical parent paths, group/world access and wrong ownership deny.

This filesystem theorem does not claim resistance to root or compromise of the dedicated broker Unix principal.

## 5. The response path binds a durable attempt

`begin_response_critical_section` accepts the non-deserializable #531 `DurablyReservedResponseAttempt`, not an arbitrary attempt digest or reference.

The response exclusion identity commits:

- exact durable reservation digest;
- stable effect identity digest;
- durable journal reference;
- exact Admin mutation exclusion binding;
- exact #535 isolation qualification; and
- exact guarded interval.

A merely prepared/non-durable attempt cannot enter this boundary.

## 6. Ordering for response exclusion is exact

Positive response exclusion requires:

```text
acquire exact pinned flock(LOCK_EX)
  -> #535 begin exclusive Admin isolation
  -> caller-controlled critical interval
  -> #535 finish/recheck
  -> revalidate exact open lock inode
  -> construct QualifiedResponseAdminMutationExclusion
  -> lock drops
```

The lock must remain alive while #535 closes.

A positive result is historical interval evidence; it does not provide a future exclusion lease.

## 7. Coordinator update uses the same exclusion domain

`update_coordinators_exclusive` must acquire the exact same pinned `flock(LOCK_EX)` before #535 begins.

The ordering is:

```text
acquire exact pinned lock
  -> #535 begin
  -> create broker-owned AdminWebsocket
  -> update_coordinators(exact payload)
  -> close Admin websocket
  -> #535 finish/recheck
  -> revalidate exact open lock inode
  -> construct QualifiedCoordinatorUpdateOperation
  -> lock drops
```

There is no unlocked coordinator update API in this crate.

## 8. The update lock is conductor-global in v0.1

The exclusion inode is not keyed by CellId, DNA, app, response, action class or authority generation.

Every coordinator update routed through this broker serializes against every response critical section.

This intentionally favors a simple strong theorem over premature lock sharding.

## 9. Direct update call ownership is mechanically closed

Within the authority/response security stack, the direct production `.update_coordinators(` call may exist only in this crate.

Future reviewed broker/runtime code must depend on this boundary rather than add another direct update path.

If another target-affecting Admin mutation is later admitted, it MUST join this same exclusion domain. It may not introduce a sibling lock and claim equivalent atomicity.

## 10. Update completion is not approved deployment

`QualifiedCoordinatorUpdateOperation` proves serialization + isolated broker provenance for an Admin coordinator update.

It does not prove the new coordinator set is an approved release. Current-release authentication/currentness and exact post-update deployment matching remain separate theorems.

## 11. Response exclusion does not prove caller work

`QualifiedResponseAdminMutationExclusion` proves that the exact durable attempt occupied the Admin mutation exclusion domain over one interval.

It does not prove what arbitrary caller-controlled work occurred inside that interval.

A later native response orchestrator must own the exact sequence of target/current-release/deployment observations, authority requalification, effect-start transition and eventual adapter invocation.

## 12. Positive results are non-deserializable

Both:

- `QualifiedResponseAdminMutationExclusion`; and
- `QualifiedCoordinatorUpdateOperation`

derive `Serialize` but not `Deserialize`.

Transporting old bytes does not recreate live lock ownership.

## 13. No external response effect

This crate contains no external effect adapter, no response effect invocation, no lifecycle `call_zome`, and no effect-start journal state.

`QualifiedResponseAdminMutationExclusion` permanently reports:

- durable attempt bound: true;
- coordinator update excluded: true;
- effect started: false;
- work inside interval verified: false;
- execution authority: false.

## 14. Remaining root of trust is explicit

The theorem assumes:

- #535 isolation deployment is correct;
- the pinned broker executable is the reviewed broker implementation;
- root/kernel are not compromised;
- the dedicated broker Unix principal is not compromised; and
- privileged namespace/filesystem manipulation is outside the attacker model.

These assumptions must remain visible rather than being laundered into cryptographic authority claims.

## 15. Next composition must avoid nested #381 fences

#535 already keeps #381 open while the response exclusion guard lives. A later final orchestrator MUST NOT call #387 in a way that attempts to acquire the same #381 trusted-state lock recursively.

Instead it should use the already-open #535/#381 interval and directly compose the state-owned target + native PRE/POST observations + current-release/#262/#290/#298 checks inside this exclusion interval.

This preserves exact causal ordering without deadlocking the process-fence store.
