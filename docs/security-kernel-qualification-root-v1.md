# Security Kernel Independent Qualification Root v1

## Purpose

The ordinary Security Kernel Qualification workflow remains the fast PR feedback lane.
It is intentionally unprivileged, but GitHub documents that a pull_request workflow uses
the workflow definition associated with the pull request's merge commit. Therefore the
PR workflow cannot be treated as an independent trust root for deciding whether its own
security workflow is correct.

This profile separates the planes:

```text
PR qualification
  -> trusted default-branch dispatcher (S0)
  -> trusted exact-head executor (S1)
  -> trusted read-only result verifier (S2)
```

## S0 — trusted dispatcher

`security-kernel-trusted-dispatch.yml` runs from the default branch on completion of the
existing Security Kernel Qualification workflow. It never checks out candidate code.
It validates the source workflow ID/path, exact source run/attempt, candidate repository,
candidate SHA, and exact open PR before dispatching S1.

The dispatcher binds source workflow ID `372951439` and path
`.github/workflows/security-kernel-qualification.yml`. Changing either is a trusted
configuration change and must therefore pass ordinary protected-branch review.

## S1 — trusted exact-head executor

`security-kernel-independent-qualification.yml` exists on the default branch and uses
`workflow_dispatch`. The candidate does not supply the workflow definition or harness
commands.

S1:

- validates that its inputs came from an exact completed source qualification run;
- fetches the exact candidate repository and commit SHA rather than a mutable PR branch;
- materializes the exact commit as source data;
- performs an independent static trust-surface audit;
- runs rustfmt, default-feature tests, identity-feature tests, and Clippy using Rust 1.99.0;
- records the candidate tree identity and trusted workflow identity; and
- fails if the candidate source changes during qualification, excluding only Cargo's `target/` output.

The candidate executes without repository write permission, secrets, or OIDC access. S1 now also requires the source PR to remain an exact open-head match at execution time, and records the pre-execution source digest in a workflow step output so the candidate cannot rewrite the expected digest or subject metadata through the shared temporary filesystem. Candidate Cargo gates explicitly select Rust 1.99.0 and re-check compiler identity between gates. The current S1 profile still executes candidate code on the GitHub-hosted VM rather than inside a dedicated container sandbox; this remains an explicit evidence ceiling because the candidate shares the runner user/filesystem and can potentially tamper with other mutable runner state.

## S2 — trusted result verifier

`security-kernel-trusted-result-verifier.yml` runs from the default branch after S1.
It does not execute candidate code and does not consume candidate-produced PASS text.
It independently validates:

- S1 workflow path and `workflow_dispatch` event;
- S1 workflow reference is the default branch;
- the exact workflow blob executed by the S1 run matches the registered immutable profile;
- S1 workflow commit is an ancestor of current protected `main`;
- exact candidate SHA and current PR head binding;
- exact source qualification run and attempt;
- the complete required S1 gate set and their individual successful conclusions.

The current S2 implementation is deliberately read-only. It produces a machine-readable
verification result in the trusted job log but does not hold status-write permission. The
verifier reads the S1 workflow blob at the exact workflow commit recorded by the run,
not merely at current `main`, preventing a malicious workflow version from being accepted
because it was later reverted. It also rejects PR-head drift between S1 dispatch and result
verification.
This keeps result publication as a separate least-privilege decision rather than silently
granting another trusted workflow mutation authority.

## Evidence ceiling

PASS under this profile means that the selected gates passed under the registered trusted
qualification mechanism for one exact immutable candidate commit.

PASS does not mean:

- formal verification;
- absence of implementation vulnerabilities;
- runner or kernel escape resistance;
- GitHub platform compromise resistance;
- independent human security review; or
- runtime authorization.

A qualification receipt is evidence, never runtime authority.