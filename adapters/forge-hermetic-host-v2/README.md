# FORGE-004D4B — Hermetic Host v2

D4B carries the concrete host orchestration theorem onto the acyclic D4A capsule transport.

The process lifecycle is intentionally unchanged from D3B2D2C:

```text
pre-run NAR qualification
  -> exact host/bubblewrap closure checks
  -> qualified guest tool map
  -> seal exact M0 capsule inputs
  -> construct bubblewrap with 5 policy + 3 control FD mounts
  -> child blocked after sandbox creation
  -> parent namespace observation
  -> pidfd_open on exact reported child before release
  -> release block FD
  -> guest execution + bounded framed stdout evidence
  -> same pidfd observes exit
  -> parent requalifies raw guest envelope
  -> parent+inside isolation qualification
  -> post-run NAR qualification
  -> pre == post
  -> seven-phase same-run qualification
```

## What changes from host v1

The old host bound `QualifiedSealedInputEvidence`, which assumed every non-Nix input was a `LinuxIsolationPolicyV1` artifact. That surface was discovered to be non-constructible once guest-plan/tool-map/policy JSON were included recursively.

Host v2 instead binds `QualifiedSealedM0CapsuleEvidence` from D4A:

- five policy/data mounts remain independently observed by the isolation probe;
- three control-plane files remain exact sealed `ExecutionSpec` inputs but are outside the policy artifact ancestry;
- the host evidence domain is `mycelix-forge/hermetic-host-evidence/v2`;
- the runtime role remains `forge-hermetic-host`, with exact semantic version `0.2.0` and exact executable bytes enforced later by Runtime Tool Evidence.

## Claim boundary

A positive `QualifiedHermeticHostRunV2` proves one concrete run under the M0 trusted-host boundary. It does not mint repository `OfflineEvidence`; D4C/v6 must compose this host receipt with repository replay, exact runtime tool evidence, and repository policy/history evidence.
