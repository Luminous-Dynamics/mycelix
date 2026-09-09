# Coordinator Admin Isolation v0.1 — Normative Invariants

Status: **native Linux reachability/isolation interval theorem; coordinator-update exclusion and external effects remain blocked**

## 1. Loopback is not isolation

A Holochain Admin listener bound to `127.0.0.1` is reachable by ordinary processes in the same network namespace. Loopback binding alone MUST NOT be treated as exclusive control-plane provenance.

## 2. The broker is the live caller

The positive path observes `/proc/self` directly. The caller cannot supply a broker PID, process start time, UID, executable observation or network-namespace identity.

The broker process must match one out-of-band `BrokerProcessBinding` on exact effective UID, absolute executable path and BLAKE3 digest of the live `/proc/self/exe` bytes.

The binding also commits the exact #381 conductor-process binding digest and exact loopback Admin endpoint.

## 3. #381 opens before namespace evidence

`begin_exclusive_admin_isolation()` MUST call `TrustedConductorProcessStore::begin_fence()` before positive namespace/process observations.

The #381 guard retains its pidfd and trusted-state lock while the isolation guard lives.

Caller-controlled work may occur only while this combined guard remains open.

## 4. Admin endpoint agreement is exact

The endpoint returned by the live #381 guard must equal the endpoint committed by the broker binding.

The final #381 fence must still report that exact endpoint and exact pinned conductor binding digest.

## 5. The Admin network namespace must not be the host namespace

The broker namespace identity is read from `/proc/self/ns/net` and the host/init namespace from `/proc/1/ns/net`.

They must differ exactly by Linux namespace device/inode identity.

A broker still running in the host network namespace denies.

## 6. The namespace is loopback-only

`/proc/self/net/dev` must contain exactly one interface: `lo`.

Any veth, bridge, physical, tunnel or other interface denies. v0.1 intentionally prefers an extremely narrow Admin namespace over attempting to reason about firewall/routing configuration.

## 7. Exact process membership

The current Linux `/proc/*/ns/net` membership set for the isolated namespace must contain exactly two process IDs:

1. the pinned broker process; and
2. the exact conductor PID independently proven by the completed #381 fence.

Unexpected persistent namespace members deny.

The set is bounded and canonicalized before identity commitment.

## 8. Broker identity is stable over the interval

Before and after caller-controlled work the broker must preserve exact:

- PID;
- process start time;
- effective UID;
- executable path;
- executable digest; and
- network namespace.

A broker restart/exec/namespace move denies.

## 9. Namespace topology is stable over the interval

Interface set and namespace member set are sampled before caller-controlled work, immediately before #381 close, and immediately after #381 close.

All samples must be exactly equal and loopback-only.

This is not a theorem against privileged transient `setns`/ABA by root. Root/kernel compromise and privileged namespace manipulation remain outside v0.1.

## 10. Positive isolation is non-deserializable

`QualifiedExclusiveAdminIsolation` derives `Serialize` but not `Deserialize`.

A caller cannot recreate isolated Admin reachability by transporting old positive bytes.

## 11. Stable identity commits both trust domains

The isolation digest commits:

- exact broker binding identity;
- exact live broker snapshot;
- exact #381 conductor fence qualification;
- isolated network-namespace identity;
- host/init network-namespace identity;
- exact canonical member PID set;
- exact interface set; and
- exact guarded interval.

## 12. This closes host-loopback bypass, not update atomicity

Positive isolation means ordinary host-network processes cannot reach the loopback Admin endpoint under the stated Linux/procfs/root assumptions.

It does **not** prove that the trusted broker itself serializes `UpdateCoordinators` against a response effect critical section.

Therefore:

`isolated Admin reachability != coordinator-update exclusion`.

A later broker/update-exclusion layer must make every coordinator update and every effect-side admission critical section acquire one shared broker-enforced exclusion primitive.

## 13. No Admin mutation in this crate

Production code MUST NOT call `AdminWebsocket`, `update_coordinators`, `install_app`, `uninstall_app`, `enable_app`, `disable_app`, or any external-effect adapter.

This crate observes host isolation only.

## 14. No execution authority

A positive result permanently reports:

- broker + conductor namespace bound here: true;
- ordinary host-loopback bypass excluded here: true;
- coordinator update excluded here: false;
- grants execution authority: false.

## 15. Deployment requirements

A production deployment must arrange the broker and conductor inside one dedicated network namespace with only loopback present. NixOS/systemd deployment hardening is a separate operational artifact and must not be inferred from this Rust theorem merely because the theorem exists.
