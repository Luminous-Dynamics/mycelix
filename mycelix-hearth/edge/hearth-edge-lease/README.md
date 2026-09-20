# hearth-edge-lease

HTH-AUTO-004C closes the process-ownership assumption left explicit by the crash-safe edge journal.

The core rule is:

> A durable journal is not sufficient authority for physical execution; exactly one cooperating Hearth executor must hold the journal-root lease while physical adapter access exists.

## Ownership theorem

`ExecutorLease::acquire(root)` creates/opens a stable `.hearth-edge-executor.lock` file inside the canonical journal root and acquires a non-blocking exclusive OS file lock.

- the **held file lock** is the ownership witness;
- lockfile existence, PID text, age, or timestamps are not authority;
- a second independent handle to the same lockfile is rejected while the first lease is alive;
- dropping the lease releases ownership automatically when the file handle closes;
- the lockfile is intentionally not deleted on release, avoiding delete/recreate inode races;
- stale diagnostic contents from a crashed process do not prevent a later lease.

`ExclusivePhysicalAdapter` holds both the lease and the existing `JournaledAdapter<A, FileJournal>` for its entire lifetime. It exposes the `EdgeAdapter` contract by delegation but does not expose an ownership-consuming escape hatch.

## Construction order

```text
canonicalize/create journal root
        |
        v
acquire exclusive executor lease
        |
        v
open durable FileJournal
        |
        v
construct crash-safe JournaledAdapter
        |
        v
physical adapter may be used
```

If lease acquisition fails, no physical wrapper is returned.

## Why the lockfile remains

Removing the path after unlocking is unsafe: another executor may already hold the old inode while a third process creates and locks a new inode at the same pathname. Keeping one stable lockfile avoids that split-lock namespace.

## Scope and limits

This is a **cooperating-process execution invariant**, not a hostile-process sandbox. OS file locks are advisory on ordinary Unix filesystems; a malicious process with direct filesystem/device access can ignore Hearth entirely.

The initial qualified target is a local NixOS/Linux journal filesystem. Network filesystems such as NFS/CIFS have filesystem- and mount-dependent locking semantics and are not admitted for physical Hearth execution until separately qualified.

This tranche deliberately does not add:

- Matter, Home Assistant, MQTT, OCPP, OpenADR, or vendor adapters;
- device discovery or credentials;
- a daemon/service supervisor;
- distributed multi-host leases;
- fencing tokens for shared remote storage;
- automatic takeover of an executor that is still alive;
- Holochain/DHT authority.

The existing bare `JournaledAdapter::new_physical` predates this theorem. Real-device adapter admission must use the leased wrapper; a later compatibility tranche can narrow/remove the bare constructor after executable qualification confirms downstream call sites.
