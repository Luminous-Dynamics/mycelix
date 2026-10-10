# GitHub Actions queue diagnostics

This runbook and the adjacent census script are read-only diagnostics for the public
Mycelix repository. They gather evidence while runner admission is blocked; they
do not replace CI or qualify Rust/Sweettest changes.

## Capture a reproducible snapshot

Requirements: Python 3.10+, GitHub CLI (gh), and an authenticated session with
permission to read Actions metadata for the repository.

    gh auth status
    python3 .github/actions_queue_census.py \
      --repo Luminous-Dynamics/mycelix \
      --job-samples 6 \
      --output /tmp/mycelix-actions-queue.json

The tool paginates the queued and in-progress run endpoints instead of silently
using only the first page. It records API-reported counts separately from the
number of unique runs fetched, groups runs by workflow/event/PR, records oldest
and newest run identities, and samples a bounded number of queued runs for job
status, runner labels, and whether a runner name has been assigned. The default
sample uses up to six additional job-list API requests. Use --job-samples 0 to
capture only workflow-run metadata.

The JSON output is timestamped in UTC. Keep it with incident notes and record
when/how it was captured. Do not attach credential material or verbose HTTP debug
output.

## How to interpret the evidence

- If active hosted jobs equal the plan's maximum concurrency, inspect plan/usage
  limits and running jobs before changing workflow configuration.
- If active jobs are below the maximum but sampled queued jobs remain unassigned,
  inspect organization/repository Actions policy, enabled runner types/labels, and
  the runner UI's queue reason; escalate to GitHub Support if those checks do not
  explain the gap.
- For self-hosted jobs, check for online idle runners matching every required
  runs-on label and runner-group constraint.
- If many older exact-head qualification jobs are queued, inventory which ones
  are deliberate immutable evidence runs and which are supersedable developer
  feedback. Do not bulk-cancel evidence runs just to lower the displayed queue.

## Qualification boundary

Queued, waiting, skipped, or unexecuted is not PASS. A runner name or an
in-progress status is not a build/test result either. Review actual job steps,
exit codes, test output, and the exact tested SHA before making a qualification
claim.

This tool does not inspect restricted organization settings and cannot determine
root cause on its own. It performs GET-only API calls and makes no workflow
changes, reruns, dispatches, cancellations, or policy edits.

## Offline tests

    cd .github
    python3 -m unittest -v test_actions_queue_census.py
    python3 -m py_compile actions_queue_census.py test_actions_queue_census.py

The tests cover pagination/deduplication, incomplete-count visibility, queue age
and grouping, API error handling, runner-assignment metadata, interpretation
limits, and bounded sampling.
