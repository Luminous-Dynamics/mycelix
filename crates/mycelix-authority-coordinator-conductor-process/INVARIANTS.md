# Authority Coordinator Conductor Process Fence v0.1 — Normative Invariants

Status: **Linux host process/listener stability theorem; not cryptographic conductor identity and not effect authority**

## 1. Loopback is not process identity

A loopback Holochain Admin endpoint proves network locality only. It does not prove which process owns the listening socket.

This crate adds a separate host theorem for one out-of-band-pinned endpoint/process policy. It does not change the meaning of #264 or #373.

## 2. The process policy is pinned out of band

`ConductorProcessBinding` commits exact:

- binding id;
- loopback Admin `SocketAddr`;
- expected effective process UID;
- absolute executable path;
- BLAKE3 digest of the complete executable bytes;
- exact executable profile; and
- provisioning/binding reference.

`ConductorProcessBindingPin` has its own fixed protocol/profile and commits the exact binding digest/profile.

Only `bootstrap_from_out_of_band_pin` accepts the binding/pin. Normal fencing loads policy from the store-owned state path.

The theorem cannot prove how the operator obtained the initial binding fingerprint.

## 3. No in-band process-policy replacement

v0.1 exposes no rotate/rebind/set-endpoint/set-executable API.

Changing endpoint, UID or executable identity requires an explicit decommission + fresh external provisioning ceremony rather than silently inheriting the old process identity.

## 4. Listener identity comes from Linux procfs

For the pinned endpoint, v0.1 reads `/proc/net/tcp` or `/proc/net/tcp6` and requires exactly one `LISTEN` socket entry for that exact local IP+port.

The socket inode is then joined to `/proc/<pid>/fd/* -> socket:[inode]` and exactly one owning process PID is required.

No caller-supplied PID, socket inode, listener owner or process snapshot is accepted by `begin_fence`.

## 5. Numeric PID alone is never sufficient

The process snapshot commits:

- listener socket inode;
- PID;
- `/proc/<pid>/stat` field-22 process start time;
- effective UID;
- `/proc/<pid>/exe` resolved path;
- complete executable BLAKE3 digest;
- executable device/inode/length.

PID reuse therefore cannot satisfy snapshot equality by PID alone.

## 6. pidfd is acquired before the interval opens

After the first procfs observation, v0.1 calls Linux `pidfd_open` for that PID, checks liveness, then re-observes the complete process/listener snapshot.

The first and second snapshots must be exactly equal before the fence starts.

The pidfd remains owned by the fence guard until `finish` and is checked for process exit again before and after the post snapshot.

## 7. Executable identity is byte-level and path-level

The pinned executable path must be absolute. `/proc/<pid>/exe` must resolve to that exact path and may not be a deleted executable.

The opened executable must be a nonempty regular file, no larger than the v0.1 bound, and not writable by group/other users.

Its complete BLAKE3 byte digest must equal the independently pinned executable digest.

Executable path equality without byte equality is insufficient.

## 8. Fence close requires exact snapshot equality

`finish` re-resolves the endpoint listener and process from procfs and requires the post snapshot to equal the pre snapshot exactly.

Thus a changed listener inode, PID/start time, UID, executable path, executable bytes or executable inode/device identity denies the fence.

## 9. The fence is historical, not a future lease

`QualifiedConductorProcessFence` carries only:

`started_at_ms <= ended_at_ms`.

It intentionally has no `valid_until_ms` or reusable future authority horizon.

A later orchestrator must require the evidence it wants to bind to have been produced inside this exact interval and at the same endpoint.

A completed fence alone cannot authorize a later Admin request or external effect.

## 10. Positive fence is non-deserializable

`QualifiedConductorProcessFence` derives `Serialize` but not `Deserialize`.

Serialized process snapshots and persisted state remain evidence/data shapes, not positive authority.

## 11. Persistence precedes positive fence authority

On successful finish:

```text
pidfd alive
→ exact post process/listener observation
→ exact pre/post equality
→ post-observation host time
→ trusted-state generation/clock advancement
→ temp fsync
→ atomic rename
→ containing-directory fsync
→ QualifiedConductorProcessFence
```

If durable state replacement fails, no positive fence escapes.

## 12. Trusted clock floor is monotone locally

The store persists `last_trusted_time_ms` and refuses observed host time below that floor.

This catches ordinary local clock rollback under the protected-state continuity model.

It does not survive full-machine rollback of the state and clock together.

## 13. Filesystem state is crash-durable, not hardware anti-rollback

The store uses:

- one canonicalized real parent directory;
- effective-UID ownership checks;
- parent inaccessible to group/other;
- exact `0600` state/lock/temp files;
- `O_NOFOLLOW | O_CLOEXEC`;
- one exclusive `File::lock` transaction;
- bounded parsing;
- same-directory `create_new` temporary replacement;
- file fsync;
- atomic rename; and
- directory fsync.

The self-digest is unkeyed. Same-UID arbitrary rewrite, root compromise and complete machine-image rollback remain outside this theorem.

## 14. This is not TCP peer authentication

TCP loopback does not provide the Unix-domain `SO_PEERCRED` style peer identity used for local Unix sockets.

The theorem identifies and fences the unique process owning the pinned listening socket under the stated Linux host-isolation model. It does not cryptographically authenticate a particular accepted TCP connection.

A later architecture may strengthen this through a supervised/private conductor transport, dedicated namespace, authenticated proxy or hardware/OS attestation.

## 15. Transient same-process exec ABA remains outside v0.1

The pidfd proves continuity of one kernel process object and pre/post executable identity checks detect persistent exec replacement.

A sufficiently privileged/same-UID attacker able to transiently exec arbitrary code in the same process while retaining the listener and restore the original executable before the post observation is outside the v0.1 theorem.

The deployment threat model must not describe this fence as resistance to arbitrary same-UID process compromise.

## 16. Process fencing does not prove observation use

`begin_fence` / `finish` do not know what work a caller performed during the interval.

A later native admission orchestrator must itself own the sequence and require:

- #373 target observation endpoint/CellId provenance;
- #264 coordinator-code observation at the same endpoint/CellId;
- observation timestamps inside this exact process fence;
- release/deployment equality;
- admission subject/attempt binding; and
- final update/effect race semantics.

## 17. No release/code/effect semantics in this crate

This crate does not import release policy, release registry, coordinator deployment matching, Holochain Admin APIs, lifecycle authorization or effect execution.

It proves only a pinned Linux listener/process stability interval.

## 18. Provisioning remains blocked

Even with #373 and this fence, external effects remain blocked until a native orchestrator binds target, process fence, coordinator code observation, current release, operational authority, attempt identity and update/effect atomicity into one fail-closed admission path.
