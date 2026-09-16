# CI-GOV-001A — Read-Only Actions Queue Census v1

Status: draft operational-observability contract for CI-GOV-001 / #969.

## Purpose

Provide a bounded, reproducible census of GitHub Actions queue state without creating cancellation, rerun, dispatch, or repository-mutation authority.

## Governing theorem

`queue telemetry != queue authority`, and `queued != started != executed != failed != passed`.

The census may identify runs that appear older than a newer queued head for the same workflow/PR. That is **supersession telemetry only**. It is not permission to cancel, rewrite, merge, or discard any evidence-bearing run.

## v1 data model

The report records:

- exact repository and observation time;
- queued and in-progress run counts;
- oldest/newest queued run and age;
- queued counts by workflow, event, and branch;
- groups with multiple queued runs for the same workflow + PR/branch scope;
- older distinct-head candidates and duplicate exact-head candidates;
- a bounded oldest/newest sample of queued job shapes;
- the most recent successful workflow run visible to the API;
- explicit status vocabulary and nonclaims.

The v1 script deliberately does **not** fetch every PR to determine draft/readiness state. That would turn a queue census into hundreds of additional API requests. Draft/ready correlation can be added later under an explicit bounded sampling/profile contract.

## Read-only authority boundary

The implementation:

- performs GitHub API `GET` requests only;
- contains no cancel, force-cancel, rerun, dispatch, workflow-enable/disable, branch-write, issue-write, or repository-write path;
- never treats an older queued head as safe to cancel merely because a newer head exists;
- never treats queue age or size as proof of GitHub outage, billing exhaustion, runner quota exhaustion, or repository misconfiguration;
- never interprets cancellation as semantic PASS/FAIL unless a separate exact gate defines that meaning.

The optional workflow is `workflow_dispatch` only. Merging this tranche therefore does not create periodic or PR-triggered queue load.

## Supersession telemetry

For each `(workflow, PR)` pair, or `(workflow, branch)` when no PR number is available, the census may report:

- queued run count;
- newest queued run/head;
- number of older queued runs with a different head;
- number of older queued runs with the same exact head.

These are candidates for **later operator review**. They are not a cancellation list.

A frozen scientific/evidence head may remain worth executing even when a newer authoring head exists. CI-GOV-001A intentionally cannot decide that question.

## Job-shape sampling

The script may inspect a bounded sample of oldest/newest queued runs and report whether GitHub has created jobs, whether queued jobs still have no steps, and whether any sampled jobs have begun exposing steps.

This is diagnostic evidence only. `steps = null`, zero jobs, or a queued job without steps must not be reported as a product failure.

## Operational interpretation

Useful patterns include:

- large queued count + zero in-progress at one instant: scheduler/capacity symptom, not root-cause proof;
- oldest queue age increasing across repeated censuses: sustained backlog evidence;
- recent successful run plus many old queued runs: intermittent execution rather than total execution outage;
- many older distinct heads for the same PR/workflow: possible future concurrency-collapse opportunity;
- many duplicate exact heads: possible rerun/trigger duplication worth separate review.

Account/org hosted-runner capacity, budgets, billing, and GitHub service health remain external checks.

## Usage

Local/operator use with a read-capable token:

```bash
GITHUB_TOKEN=... python3 scripts/ci_queue_census.py --repo Luminous-Dynamics/mycelix
```

Self-test requires no network:

```bash
python3 scripts/ci_queue_census.py --self-test
```

The manual workflow runs the self-test first and then emits the live census to its logs.

## Privacy and load

The census reports repository Actions metadata already visible to the authorized caller. It does not fetch workflow logs or artifacts. Job inspection is sampled and bounded by `--job-sample` (default 12) to avoid turning observability into API pressure.

## Nonclaims

A CI-GOV-001A PASS or census does not establish:

- the root cause of runner delay;
- GitHub service outage;
- account/org billing or quota state;
- that any queued run is safe to cancel;
- product or scientific qualification;
- workflow semantic correctness;
- permission to mutate Actions state.

This tranche is measurement-only.