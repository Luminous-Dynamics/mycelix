# CI-GOV-001D-B — bounded superseded-run executor v0.1

This tranche adds the separately authorized execution half of CI-GOV-001D. It is intentionally stacked on the read-only 001D-A planner and does **not** itself cancel any workflow run.

## Authority boundary

The executor can act only on a complete, unexpired `mycelix-ci-superseded-run-plan-v1` envelope produced by 001D-A. It has exactly one mutating API operation:

`POST /repos/{repository}/actions/runs/{run_id}/cancel`

There is no rerun, dispatch, workflow edit, artifact deletion, PR/issue mutation, or generic request facility.

## Explicit operator requirements

Dry-run is the default. Apply mode additionally requires:

- `--apply`;
- one or more explicit `--run-id` selections;
- no more than 5 selected runs;
- `--confirm-plan` exactly equal to the plan commitment;
- a non-empty `--authority-ref`;
- `--receipt-output` so mutation intent/outcomes are durably recorded.

There is deliberately no `--all` or workflow-scope override.

## Fail-closed revalidation

Immediately before each selected run can reach the cancel endpoint, the executor re-fetches and requires:

- exact repository from the plan;
- exact run ID and run attempt;
- exact workflow ID and fixed `.github/workflows/ci.yml` path;
- `pull_request` event;
- exact planned queued head SHA;
- live status still `queued` or `in_progress`;
- exact PR-association set from the plan;
- every associated PR still open;
- every associated PR still on exactly the successor SHA recorded by the plan;
- no associated PR points back to the queued head.

V0.1 is intentionally stricter than the parent design: if a PR advances again after planning, execution refuses and requires a fresh plan rather than reclassifying during mutation.

## TOCTOU and partial-batch semantics

Runs are processed sequentially and revalidated immediately before their own mutation. The batch policy is `stop-on-first-refusal-or-mutation-error`.

Apply mode writes a `MutationIntentRecorded` receipt to disk before issuing POST. It then atomically replaces that entry with one of:

- `CancelRequestAccepted`;
- `CancelRequestRejected` when GitHub returns a definite HTTP error;
- `CancelOutcomeUnknown` when transport/process state makes the mutation outcome uncertain.

If a later run becomes invalid after earlier cancellation requests were accepted, the executor records `RefusedAtRevalidation` and stops. It never treats prior accepted requests as transactional rollback.

## Receipt semantics

Cancellation receipts are administrative scheduling provenance only. They must never be interpreted as product PASS/FAIL or scientific evidence. The historical GitHub run record remains the evidence of what was queued/cancelled.

## Tests

The deterministic unit corpus covers dry-run nonmutation, exact selection, max-5 bounds, plan expiry/tamper/confirmation, noneligible selection, run/head/attempt/workflow/association changes, closed/current/further-advanced PRs, completed and in-progress runs, durable pre-mutation intent emission, mutation uncertainty, and stop-on-first-refusal/error behavior.

No real cancellation is authorized by committing this source or opening its review PR. A pilot requires a fresh planner output, explicit operator-selected IDs, and a separately recorded authority reference.
