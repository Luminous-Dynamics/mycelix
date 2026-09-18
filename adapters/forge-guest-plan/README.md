# mycelix-forge-guest-plan

FORGE-004D3B2B freezes the **canonical in-sandbox verification plan** for the M0 Linux/Nix/gittuf local-key profile.

The guest runner is deliberately not an arbitrary command executor. It receives one exact plan plus exact read-only protocol artifacts and must execute one fixed sequence.

## Fixed guest sequence

The v1 guest sequence is:

1. run the isolation probe in a subprocess that must terminate afterward;
2. reconstruct the portable Git bundle into `/work/repository.git`;
3. reject undeclared Git object sources;
4. run strict full Git object validation;
5. collect the complete gittuf policy trust inventory;
6. qualify the local embedded-key trust profile;
7. derive the canonical gittuf invocation from the exact typed repository request and run cache-free verification;
8. emit the canonical guest transcript.

The phase sequence is part of the plan commitment and cannot be reordered or replaced with arbitrary argv.

## Self-contained guest inputs

The guest plan carries the full typed `RepositoryVerificationRequest`, while the sandbox also mounts exact JSON artifacts for:

- portable bundle manifest;
- guest verification plan;
- Linux isolation policy;
- Nix closure manifest;
- sandbox invocation;
- plus the raw repository bundle and 32-byte run challenge.

This allows the guest to re-derive `GittufInvocation::from_request(...)`, validate its own isolation/closure/invocation subjects, and avoid trusting host-precomputed verifier argv.

## Git object-validation policy

`GitObjectValidationPolicyV1` is a zero-configuration positive type: there is no constructor that can weaken it. Its commitment fixes:

- `git --no-replace-objects fsck --full --strict --no-reflogs --no-progress`;
- no `--connectivity-only` shortcut;
- lazy fetch disabled;
- system/global Git configuration disabled;
- no `GIT_OBJECT_DIRECTORY`;
- no `GIT_ALTERNATE_OBJECT_DIRECTORIES`;
- no `objects/info/alternates` file;
- no shallow repository state;
- no promisor-pack sidecar state;
- no configured remotes in the reconstructed repository.

## Acyclic subject graph

The guest plan is itself an exact read-only `ExecutionSpec` input. Therefore it binds the execution **subject**, not the enclosing execution-spec or same-run-plan digests.

```text
guest plan -> execution subject
execution spec -> guest-plan artifact
same-run plan -> execution-spec digest
```

No recursive/fixed-point hash construction is required.

## Plan subject

`GuestVerificationPlanV1` binds the exact:

- execution subject;
- full typed repository verification request (committed semantically by request digest);
- portable bundle-manifest commitment;
- repository policy-state digest;
- Linux isolation-policy digest;
- Nix closure digest;
- sandbox-invocation digest;
- local trust-profile digest;
- per-run challenge digest;
- expected portable-replay receipt;
- strict Git object-validation policy.

The constructor derives the Git-policy commitment internally. Deserialization re-derives it and rejects any altered policy digest.

## Fixed sandbox paths

The v1 plan commits fixed paths for the repository bundle, manifest, run challenge, guest plan, isolation policy, Nix closure, sandbox invocation, replay repository, and guest transcript. The guest cannot redirect those artifacts through host-controlled path parameters without changing the plan commitment.

## Claim boundary

This crate defines what the guest **must** execute. It does not launch bubblewrap, reconstruct a repository, inspect `/proc`, call Git/gittuf, or mint `OfflineEvidence`.

The following tranche must implement the guest runner and emit evidence for this exact plan. The host orchestrator remains a separate process/FD/pidfd responsibility.
