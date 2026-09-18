# Mycelix Forge FORGE-004D2B2 — Linux Isolation Observation Collector

**Status:** implementation candidate  
**Depends on:** FORGE-004D2B1 isolation evidence contract  
**Launcher/NAR qualification:** deferred

## Purpose

FORGE-004D2B2 implements the host-I/O side of the two-channel isolation design without duplicating its qualification predicates. The collector emits D2B1 evidence objects; only D2B1 may decide whether those observations satisfy the policy.

## Inside observation

The dedicated probe process must run inside an already-created 004D2A sandbox and call `collect_inside_evidence` exactly once. It collects namespace links, capability masks, environment, filesystem visibility, mounted artifact identity, route state, and active denial probes.

Artifact hashing is streaming and algorithm-aware. A mounted object must be a regular file; its actual digest and byte size are reported independently of the policy values.

Nested `CLONE_NEWUSER` creation is attempted last. A success makes qualification fail and the probe process is never re-used for verifier execution.

## Parent observation

The parent consumes bubblewrap's JSON status stream and locates a `child-pid` record rather than depending on record ordering. While bubblewrap holds the child behind `--block-fd`, the parent reads host and child namespace identities and child mountinfo from the host `/proc` view.

After probe exit, the parent binds the final status stream and exit code.

This yields two channels over the same namespace/mount state:

```text
inside probe  ── /proc/self/... ──┐
                                  ├── D2B1 qualification
parent       ── /proc/<pid>/... ──┘
```

## Deliberate non-claims

D2B2 does not:

- launch bubblewrap;
- decide qualification itself;
- prove bubblewrap or collector binary provenance;
- re-derive Nix NAR hashes;
- prove a verifier trust profile;
- mint repository `OfflineEvidence`.

The next tranche will bind the actual Nix closure contents and launcher/runtime identity rather than smuggling those claims into the namespace probe.
