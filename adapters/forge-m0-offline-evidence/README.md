# mycelix-forge-m0-offline-evidence

FORGE-004D3A is the exact composition gate allowed to mint repository-level `OfflineEvidence` for the Linux/Nix/gittuf local-key M0 profile.

This revision consumes the self-contained guest plan from FORGE-004D3B2B and treats the bubblewrap child invocation as the **Forge guest runner**, not gittuf directly.

## Exact composition

The composer requires one exact repository request, portable replay, guest plan, v2 execution spec/result, Linux isolation policy, Nix closure, sandbox invocation, qualified isolation result, qualified runtime NAR closure, local-key trust profile, and same-run plan/result.

## Self-contained guest capsule

The exact execution input set and Linux read-only mount set are both:

- `repository-bundle`
- `repository-bundle-manifest`
- `guest-verification-plan`
- `linux-isolation-policy`
- `nix-closure-manifest`
- `sandbox-invocation`
- `run-challenge`

For JSON protocol artifacts, the composer checks both semantic object commitments and SHA-256/size of the exact mounted JSON bytes.

A separate `capsule_artifacts` commitment binds all mounted transport identities together.

## Sandbox invocation

`VerifierInvocation` is now the exact bubblewrap-child invocation:

```text
/nix/store/.../bin/forge-hermetic-guest --plan /inputs/guest-verification-plan.json
```

The guest runner derives the inner gittuf invocation from the typed `RepositoryVerificationRequest` embedded in the guest plan. Host-precomputed gittuf argv is not authority.

## Exact tool set

The M0 execution tool set is exactly:

- `bubblewrap`
- `forge-hermetic-host`
- `forge-hermetic-guest`
- `forge-isolation-probe`
- `git`
- `gittuf`
- `nar-auditor`

Extra tools fail closed.

## Guest-plan cross-links

The guest plan must name the exact execution subject, typed request digest, semantic bundle manifest, repository policy state, isolation policy, Nix closure, sandbox invocation, local trust profile, run challenge, expected replay receipt, and strict Git object-validation policy.

## Environment and trust

The local SSH/GPG profile requires `VerificationTimePolicy::NotUsed`, exact equality with the strict isolation environment, and **no external trust material**. Embedded policy keys remain the sole trust source for this profile.

## Acyclic subject graph

The guest plan is an execution input but binds the execution subject rather than the enclosing execution-spec/run-plan digest:

```text
foundational subjects -> execution subject -> guest plan
exact guest-plan JSON -> ExecutionSpec
ExecutionSpec digest -> HermeticRunPlan
HermeticRunPlan -> same-run evidence
```

No recursive hash construction is required.

## OfflineEvidence

Only after all semantic, transport-byte, runtime, same-run, and trust edges agree does the composer create an evidence-backed repository observation with `OfflineEvidence` and qualify it through the existing `VerificationProfile::m0_protected_source()` path.

## Claim boundary

D3A is still a composition theorem. The concrete guest runner and host pidfd/FD orchestrator must produce the exact positive inputs consumed here.
