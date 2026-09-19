# CI-GOV-001K-D — source-only live queue:max pilot fixture v0.1

This tranche freezes the exact workflow bytes and static verifier for the future bounded live scheduler experiment tracked by #2067. The fixture is deliberately stored outside `.github/workflows/`, so this source tranche cannot trigger GitHub Actions.

## Exact fixture purpose

The future active workflow is byte-for-byte identical to `ci_gov_001k_live_pilot.yml.fixture`; only its repository path changes when copied into `.github/workflows/` on isolated pilot branches.

The fixture is designed to test one narrow live theorem: with repository-wide concurrency group `mycelix-heavy-qualification-v1` and `queue: max`, one pilot can run while at least two independently admitted pilot runs remain pending instead of replacing/cancelling one another.

It does not test product code.

## Frozen workflow semantics

```text
trigger:             pull_request / labeled only
pilot label:         ci:qualify-pilot
PR state:            draft only
permissions:         contents: read
concurrency group:   mycelix-heavy-qualification-v1
queue:               max
cancel-in-progress:  absent / false
jobs:                exactly 1
runner:              ubuntu-24.04
timeout:             5 minutes
hold:                90 seconds
third-party actions: 0
checkout:             none
repository mutation: none
```

The runner step emits only PR number, exact PR head SHA, workflow run ID/attempt, and UTC begin/end timestamps around the fixed 90-second hold.

## Why labeled-only

Opening or synchronizing a pilot PR must not consume a runner. The dedicated `ci:qualify-pilot` label is the sole event that creates a pilot run. It is intentionally distinct from production `ci:qualify`.

## Bounded live protocol

The active experiment must not begin while repository-wide runner assignment is visibly starved.

1. Prepare three draft PRs A/B/C with stable current heads and the exact fixture bytes copied to the registered workflow path.
2. Confirm the pilot label exists and that no pilot subject already carries it.
3. Confirm repository runner assignment is functioning; do not launch from a snapshot equivalent to `0 in_progress` plus a large queued backlog.
4. Apply the pilot label to A only.
5. Wait for A's exact pilot run to become `in_progress` within the registered health window.
6. If A does not become active, classify `RUNNER_UNAVAILABLE`; do not label B/C. The unhealthy experiment therefore creates at most one new run.
7. Once A is active, label B and C in a short bounded interval.
8. Observe the exact repository-wide concurrency group using the independently qualified 001K-B mechanism.
9. Require a snapshot containing A `in_progress` plus distinct B and C members simultaneously `pending`.
10. Let all three runs finish naturally. Do not use cancellation as pilot cleanup.
11. Bind final run IDs, attempts, PR numbers, exact head SHAs, workflow blob identity, group snapshots, statuses, conclusions, and timestamps into the pilot receipt.

## Evidence states

- `PASS`: exact workflow identity verified; A became active; B/C were simultaneously pending behind A; at most one pilot was active; no pilot was scheduler-replaced/cancelled; all three completed successfully.
- `RUNNER_UNAVAILABLE`: A did not become active inside the health window; B/C were never admitted.
- `INCONCLUSIVE`: runner assignment worked, but the registered simultaneous state was not captured and no registered invariant was violated.
- `FAIL`: exact identity/configuration drift, replacement/cancellation of admitted pending pilot work, more than one pilot active in the group, or another registered gate violation.

No FIFO/fairness claim is made.

## Static verifier

`ci_gov_001k_pilot_fixture_verify.py` verifies both the exact fixture SHA-256 and the safety-critical semantic surface. It rejects alternate triggers, queue replacement mode, `cancel-in-progress`, broader labels, missing draft guard, write permissions, actions/checkout or any `uses:`, additional jobs/steps, runner/timeout/hold drift, secrets/token use, network clients, and noncanonical fixture bytes.

The committed adversarial corpus contains 25 cases.

## Platform boundary

Current GitHub Actions documentation states that `queue: max` allows multiple pending runs in one concurrency group, up to 100 pending entries, and cannot be combined with `cancel-in-progress: true`. The live experiment remains necessary: static source qualification cannot establish GitHub's actual scheduler behavior at execution time.

## Nonclaims

This source tranche does not activate a workflow, add a label, create a pilot run, qualify live scheduler semantics, establish runner availability, authorize cancellation, grant merge authority, qualify ASSURE, or establish any product/scientific claim.
