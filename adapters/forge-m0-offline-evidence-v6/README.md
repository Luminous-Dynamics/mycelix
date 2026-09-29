# FORGE-004D4C — M0 Offline Evidence v6

D4C is the repaired repository-authority gate for the first local SSH/GPG hermetic Forge profile.

It supersedes D3A-v5. The v5 verifier cross-linked a strong runtime theorem, but its capsule surface was discovered before qualification to contain a hash fixed point: the Linux isolation policy committed guest-plan/tool-map/policy JSON bytes while the guest plan committed the policy digest, and the policy JSON was effectively required to commit its own bytes.

v6 preserves the useful v5 evidence chain while replacing that non-constructible topology with the D4A **5+3 split**.

## Exact capsule topology

`ExecutionSpec` still commits exactly eight input artifacts.

Five are Linux-policy artifacts and therefore independently observed by the in-sandbox isolation probe:

```text
repository-bundle
repository-bundle-manifest
nix-closure-manifest
sandbox-invocation
run-challenge
```

Three are immutable sealed control-plane inputs proven by D4A and the v2 host receipt:

```text
guest-verification-plan
guest-tool-map
linux-isolation-policy
```

The control files are deliberately absent from `LinuxIsolationPolicyV1::artifact_mounts()`. This makes policy construction finite while retaining exact digest, size, destination, and kernel-seal evidence for every one of the eight inputs.

## Final positive theorem

A `QualifiedM0OfflineEvidenceV6` requires all of these to name one exact subject:

- exact portable Git/gittuf replay and replay receipt;
- execution subject v6;
- `ExecutionSpec` v2 using denied network, `VerificationTimePolicy::NotUsed`, hermetic filesystem, no external trust material, exact six-tool set, and exact eight-input set;
- exact five-artifact Linux isolation policy;
- guest plan and guest tool map semantic links;
- exact JSON/transport fingerprints for policy, plan, tool map, closure, invocation, and manifest;
- D4A `QualifiedSealedM0CapsuleEvidence` for all eight immutable inputs;
- D4B `QualifiedHermeticHostRunV2`;
- parent+inside Linux isolation qualification;
- pidfd same-process relation;
- pre/post canonical NAR equality;
- seven-phase same-run evidence;
- Runtime Tool Evidence for all six executable roles;
- `EvidenceBoundExecution` with output equal to the qualified guest-transcript evidence digest and execution evidence equal to the v2 host evidence digest;
- repository history/policy-lineage qualification with the final v6 evidence commitment.

## Exact runtime roles

```text
bubblewrap
forge-hermetic-host        0.2.0
forge-hermetic-guest       0.1.0
forge-isolation-probe      0.1.0
git
gittuf                     0.16.0
```

The NAR auditor remains linked into the exact host artifact and separately pinned by `auditor_subject()`.

## Trust boundary

M0 assumes the kernel and privileged host/root remain trustworthy during the run. It does not claim resistance to a malicious kernel, firmware, hypervisor, or privileged host capable of falsifying observations or transiently altering and restoring state.

A positive result grants only the repository-level `OfflineEvidence` capability for this exact local-key profile. It does not imply source correctness, vulnerability absence, reproducible builds, release authorization, hostile-host resistance, or Sigstore-hermetic verification.

## Supersession

D3A-v5 / PR #1787 must not be promoted as an M0 authority subject. D4C/v6 is the first composition design in this stack intended to be both independently verifiable **and finitely constructible**.
