# CI-GOV-001E — least-privilege CI trust boundary

This directory contains the deterministic, read-only policy verifier for
CI-GOV-001E. It does not mutate GitHub, workflows, runs, branches, or repository
settings.

## v0.1 theorem

A successful check establishes only that the inspected workflow text satisfies
the frozen v0.1 CI trust-boundary policy:

- workflow token defaults to `contents: read`;
- the path-filter admission job has only `contents: read` and
  `pull-requests: read`;
- no job-level permission explicitly grants `write`;
- the admission job executes only the frozen checkout and paths-filter action
  revisions;
- its checkout disables persisted Git credentials;
- the repository-facing `ci-pass` summary job executes no external action.

This is a source-policy theorem, not runtime or repository-enforcement evidence.

## Deliberate scope

v0.1 does **not** require every ordinary build/test action in the repository to
be SHA-pinned. It first closes the admission and required-summary trust boundary.
Broader action pinning can be expanded in later CI-GOV tranches with its own
review and compatibility evidence.

## Running

```sh
python3 .github/ci-governance/check_ci_policy.py
python3 -m unittest .github/ci-governance/test_ci_policy.py
```

The checker uses only the Python standard library.

## Nonclaims

PASS does not establish product correctness, scientific validity, successful
workflow execution, runner integrity, GitHub service integrity, branch
protection, constitutional authority, political legitimacy, or overall system
security.
