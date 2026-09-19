# FORGE-004D4A — Acyclic Sealed M0 Capsule Inputs

This tranche repairs a construction-cycle discovered before M0 qualification.

The previous draft treated every capsule file as a `LinuxIsolationPolicyV1` artifact. That made the policy digest depend on the exact guest-plan/tool-map/policy JSON bytes, while the guest plan itself committed the policy digest. In particular, mounting the policy JSON as one of its own digest-bearing policy artifacts creates a direct self-hash fixed point.

D4A separates **sandbox-policy artifacts** from **capsule control-plane artifacts** without weakening exact byte identity.

## Exact split

The Linux isolation policy contains exactly five independent artifacts:

```text
repository-bundle
repository-bundle-manifest
nix-closure-manifest
sandbox-invocation
run-challenge
```

Three control-plane files remain exact `ExecutionSpec` inputs and are still copied into fully sealed memfds and mounted with `--ro-bind-fd`, but they are not ancestors of the policy digest they carry or consume:

```text
guest-verification-plan
guest-tool-map
linux-isolation-policy
```

Therefore the finite construction order becomes:

```text
bundle/manifest + closure + invocation + challenge
        ↓
LinuxIsolationPolicyV1 (5 artifact mounts)
        ↓
policy digest
        ↓
GuestVerificationPlanV1
        ↓
GuestToolMapV1
        ↓
serialize policy / plan / tool-map bytes
        ↓
ExecutionSpec (exact 8 inputs)
        ↓
sealed five policy inputs + sealed three control inputs
```

No hash fixed point is required.

## Evidence

`QualifiedSealedM0CapsuleEvidence` binds:

- exact `ExecutionSpec` digest;
- exact Linux policy digest;
- the existing qualified five-artifact sealed-policy evidence;
- exact role/destination/digest/size/kernel-seal observations for all three control inputs;
- one aggregate sealed-capsule evidence commitment.

The canonical bubblewrap command is still produced by the existing isolation/sealed-input builders for policy mounts. D4A only inserts the three sealed control `--ro-bind-fd` mounts before environment and invocation setup.

## Claim boundary

D4A proves immutable transport and an acyclic dependency surface. It does not qualify Linux isolation, execute the guest, prove repository history, or mint `OfflineEvidence`.

The three control mounts are intentionally not part of the inside artifact-observation set. Their byte identity is established by the `ExecutionSpec` plus sealed-capsule evidence; the five Linux-policy artifacts remain independently observed from inside the sandbox. Parent and inside mount-namespace observations still bind the actual sandbox instance.
