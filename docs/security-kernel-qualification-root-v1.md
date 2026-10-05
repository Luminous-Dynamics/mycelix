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

The candidate executes without repository write permission, secrets, or OIDC access.

## S2 — trusted result verifier

`security-kernel-trusted-result-verifier.yml` runs from the default branch after S1.
It does not execute candidate code and does not consume candidate-produced PASS text.
It independently validates:

- S1 workflow path and `workflow_dispatch` event;
- S1 workflow reference is the default branch;
- S1 workflow commit is an ancestor of current protected `main`;
- exact candidate SHA and PR binding;
- exact source qualification run and attempt;
- the complete required S1 gate set and their individual successful conclusions.

The current S2 implementation is deliberately read-only. It produces a machine-readable
verification result in the trusted job log but does not hold status-write permission.
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