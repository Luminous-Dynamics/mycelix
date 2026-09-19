# CI-GOV-001K-B — read-only live concurrency-group observer v0.1

This tranche observes the exact repository-wide live concurrency group used by CI-GOV-001K without adding a workflow or mutation authority.

## Frozen endpoint

```text
repository:  Luminous-Dynamics/mycelix
group:       mycelix-heavy-qualification-v1
method:      GET
endpoint:    https://api.github.com/repos/Luminous-Dynamics/mycelix/actions/concurrency_groups/mycelix-heavy-qualification-v1
API version: 2026-03-10
```

GitHub documents this endpoint as the live repository-wide state of one concurrency group. A `200` response contains `group_name`, `total_count`, and `group_members`; members are represented as `in_progress` or `pending`. GitHub documents `404` for a group with no active items.

A 404 therefore proves only an **empty live group at observation time**. It does not prove that any workflow is configured to use the group.

## Read authority

The observer performs exactly one bounded HTTPS GET to the frozen endpoint. It may use `GITHUB_TOKEN` only as an Actions-read credential. It does not log or retain the token.

Redirects are disabled. The response body is capped at 1 MiB for both success and HTTP-error responses; oversized error bodies fail closed rather than being reinterpreted by status alone. The request timeout is bounded.

There is no POST/PATCH/PUT/DELETE, cancellation, rerun, labeling, merge, status-publication, or workflow-dispatch authority.

## 200 validation

A live response is complete only when:

- `group_name` equals the frozen group exactly;
- `group_url` equals the frozen endpoint exactly;
- `total_count` is a positive integer and equals the exact `group_members` length;
- every member has a positive integer `run_id`, non-empty run name, and repository-bound run URLs;
- optional job identity is all-or-nothing and its URLs bind the same repository/run/job;
- member identities `(run_id, job_id?)` are unique;
- every member status is exactly `in_progress` or `pending`;
- at most one member is `in_progress`;
- at most 100 members are `pending`.

Unknown statuses or schema/identity drift fail closed rather than being treated as capacity.

## 404 semantics

```text
HTTP 404
-> complete = true
-> active_count = 0
-> pending_count = 0
-> group_present = false
-> configuration_established = false
-> reason = inactive_or_nonexistent_group
```

This is valid capacity evidence for an empty live group, but not workflow-configuration evidence.

## Other failures

Transport errors, redirects, oversized bodies, malformed JSON, unexpected HTTP status, count mismatch, duplicate members, URL drift, unknown status, platform-bound violations, or invalid observer clock state produce an incomplete observation.

Incomplete observations must not be converted to zero capacity.

## Observation receipt

The observer records:

- exact repository/group/API identity;
- observation UTC and epoch timestamp;
- complete/incomplete state;
- active/pending/total counts when authoritative;
- normalized run/job member identities and statuses;
- source HTTP status/reason;
- explicit false authority fields;
- SHA-256 commitment over the observation body.

The observer intentionally does **not** output `age_seconds`. The later trusted admission step must derive age from `observed_at_epoch_seconds` at decision time and enforce the v0.2 30-second freshness bound.

Invalid clock state itself produces a committed incomplete observation rather than an exception or fabricated timestamp.

## Tests

The hardened v0.1 corpus covers 26 cases including 404 empty-state semantics, 200 active/pending normalization, non-authoritative HTTP responses, group/group-URL drift, count mismatch, zero-member 200 rejection, unknown status, duplicate identity, >1 active, >100 pending, malformed member identity/URLs, malformed JSON, transport failure, timestamp/commitment binding, invalid-clock refusal, oversized-body refusal, token handling, redirect refusal, GET-only source audit, fixed endpoint identity, and explicit non-authority fields.

## Nonclaims

Observer PASS does not establish live `queue: max` scheduler behavior, workflow configuration, capacity admission, fairness/FIFO, runner availability, cancellation authority, merge authority, product correctness, or any scientific claim.
