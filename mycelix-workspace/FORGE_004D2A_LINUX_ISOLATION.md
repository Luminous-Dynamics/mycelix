# Mycelix Forge FORGE-004D2A — Canonical Linux Isolation Policy

**Status:** implementation candidate  
**Depends on:** FORGE-004D1 evidence-bound execution contract  
**Isolation substrate:** Nix closure + bubblewrap

## Purpose

FORGE-004D2A freezes one exact Linux isolation policy for Forge repository verification. It defines the canonical policy subject and deterministic no-shell bubblewrap argv construction, but it does **not** claim that a particular host kernel actually enforced the requested isolation. Runtime qualification is FORGE-004D2B.

## Exact dependency boundary

The policy commits to:

- an exact `NixClosureManifest` containing individual `/nix/store/<entry>` roots and NAR hashes;
- an exact `VerifierInvocation` containing the executable store path and argv;
- exact input/trust artifact roles, destinations, digests, and byte sizes;
- the complete strict v1 namespace/mount/environment policy.

The verifier executable must be a child of one committed closure root. The command builder never bind-mounts the host `/nix/store` wholesale.

## Strict v1 policy

The v1 policy requires:

- explicit user, IPC, PID, network, and UTS namespaces;
- nested user namespaces disabled;
- a new session and child lifetime bound to the parent;
- all Linux capabilities dropped;
- inherited environment cleared;
- private `/proc` and minimal `/dev`;
- ephemeral `/tmp` and `/work`;
- hidden host home directory;
- no `/sys`, `/run`, host D-Bus, SSH agent, or ambient host paths;
- only the exact committed Nix closure mounted read-only;
- committed evidence/trust artifacts mounted read-only under `/inputs` or `/trust`.

A serialized v1 policy with any required control disabled is invalid.

## Invocation integrity

Tool identity is insufficient if the same binary can be invoked with different semantics. `VerifierInvocation` therefore commits to the exact executable and argv. The command builder never invokes a shell.

This prevents a full-history verifier subject from silently becoming a `--latest-only` invocation while retaining the same executable digest.

## Runtime transport

Host paths used to transport committed artifacts are intentionally excluded from the canonical policy. FORGE-004D2B must hash each runtime artifact and match its committed digest/size before execution.

## Claim boundary

FORGE-004D2A establishes deterministic Linux isolation-policy formation and command construction. It does not establish that bubblewrap or the Linux kernel enforced the policy, that NAR hashes were re-derived on the current machine, that runtime artifact bytes match their commitments, or that the verifier trust profile is hermetic. Those are 004D2B/004D2C responsibilities.
