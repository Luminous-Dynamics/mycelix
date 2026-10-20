# Qualification Harness v1

Status: executable repository-scope qualification kernel; no domain semantic correctness claimed.

Frozen parent: `62d9fc51e506c2fbc0b8e08be348c02c31b5edeb`.

## Purpose

QUAL-HARNESS-001 defines one small deterministic verifier for exact Git subject identity and review scope so evidence-oriented tranches do not repeatedly reimplement pull-request head, ancestry, path census, file-mode, and working-tree checks.

The governing theorem is:

```text
ScopeHarnessPass
!= domain semantic PASS
!= security correctness
!= scientific validity
!= legal validity
!= production readiness
```

A caller must run its domain validator separately.

## QH-001 — authored head, not synthetic merge ref

For pull-request events, the harness verifies the checked-out commit against `pull_request.head.sha`. A GitHub-generated merge commit is not accepted as the authored subject when exact-head qualification is requested.

## QH-002 — exact parent

The subject must have the exact caller-supplied parent. The harness does not infer a nearest merge base or silently rebase the evidence claim.

## QH-003 — explicit commit-count policy

The default qualification profile requires exactly one authored commit over the expected parent. Other policies require an explicit versioned caller profile; retries or merge refs may not silently alter the count.

## QH-004 — exact changed-path set

Changed paths are compared as exact byte-safe/NUL-delimited Git path data. Unexpected extra files and missing expected files fail closed.

## QH-005 — change kinds fail closed

The v1 default allows only `A` and `M`. Deletes, type changes, unmerged states, unknown raw-diff statuses, or rename/copy behavior that appears as delete/add under `--no-renames` cannot silently satisfy the scope theorem.

## QH-006 — file modes are optionally exact

When callers provide expected modes, the harness verifies exact Git tree modes for those paths. Path-only scope checks do not silently prove executable-bit or type invariants.

## QH-007 — history insufficiency is explicit

The harness performs no network fetch. If the expected parent or required history is absent from the checkout, the result is `HistoryInsufficient`, not a best-effort PASS.

## QH-008 — working tree immutability

A dirty working tree before qualification fails. Callers should run the harness again after domain validation or otherwise prove postflight immutability; validators must not mutate the frozen subject.

## QH-009 — event data is parsed, not shell code

Where event identity is checked, `GITHUB_EVENT_PATH` is parsed as JSON. Event-derived values are data inputs, not interpolated shell program text.

## QH-010 — scope receipt is typed evidence only

A successful harness emits facts such as `HeadVerified`, `ParentVerified`, `CommitCountVerified`, `PathSetVerified`, `ModeSetVerified` or `NotRequired`, `WorkingTreeImmutable`, and `ScopeHarnessPass`.

Those facts establish repository identity/scope only. They do not upgrade a caller's domain semantics.

## Interface

The reference script accepts:

- `--expected-parent SHA`;
- `--expected-head SHA`;
- `--expected-commit-count N`;
- repeated `--expected-path PATH`;
- repeated `--allowed-status STATUS` (v1 normally `A` and `M`);
- repeated `--expected-mode PATH=MODE`;
- optional `--event-path PATH`;
- optional `--require-clean-tree`;
- optional `--output PATH`.

The verifier uses argument-vector subprocess calls. It never constructs shell commands and never performs network access.

## Raw diff discipline

Changed-path/status/mode input is derived from:

```text
git diff --raw -z --no-abbrev --no-renames <parent> HEAD
```

The raw NUL-delimited format lets unusual filenames remain data rather than line-oriented control syntax. Rename/copy detection is disabled deliberately: a rename becomes delete+add and therefore fails unless the exact caller profile explicitly expects the resulting paths/statuses.

## Self-test corpus

The frozen corpus includes:

- exact happy path;
- synthetic PR merge/wrong head;
- wrong parent;
- two commits under a one-commit policy;
- hidden/unexpected extra path;
- disallowed delete;
- file-mode mismatch;
- dirty working tree;
- insufficient history;
- PR event head mismatch;
- a path containing whitespace/newline characters that remains exactly distinguishable.

The self-test corpus is synthetic and does not establish GitHub Actions reliability or repository semantic correctness.

## Security posture

The kernel requires no secrets and no repository write permission. External actions are pinned by full commit SHA. The verifier itself is zero-dependency Python and performs no network I/O.

## Explicit nonclaims

A QUAL-HARNESS-001 PASS does not establish:

- domain semantic correctness;
- code correctness;
- cybersecurity or supply-chain security;
- scientific validity;
- legal validity or compliance;
- policy legitimacy;
- external-standard conformance;
- production readiness.

It establishes only the exact frozen repository identity/scope facts produced by this harness profile.
