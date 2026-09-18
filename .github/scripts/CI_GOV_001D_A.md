# CI-GOV-001D-A — superseded-run reclamation planner

This branch implements only the **read-only planning half** of CI-GOV-001D.

It consumes the qualified CI-GOV-001A GET-only client and emits a deterministic plan for a deliberately narrow v1 scope:

- repository identity is explicit and committed;
- run status must be `queued`;
- event must be `pull_request`;
- workflow path is hard-coded to exactly `.github/workflows/ci.yml`;
- the CLI cannot override or widen workflow scope;
- live observation time always comes from the system UTC clock and cannot be overridden by CLI input;
- run ID, run attempt, workflow ID, queued head, and creation timestamp must be present;
- every associated PR must resolve successfully;
- every associated PR must still be open;
- every associated PR's current head must differ from the queued run head.

Only runs inside that fixed authority scope enter the committed plan envelope. Unrelated exact-qualification, Forge, push, or in-progress traffic is deliberately excluded rather than merely marked ineligible. This prevents queue activity the planner can never act on from invalidating an otherwise unchanged broad-CI cleanup plan.

Malformed/incomplete metadata on an otherwise in-scope broad-CI PR run is **not** filtered away: it remains in the envelope and fails closed.

Only a fully resolved stale broad-CI PR run is classified `SupersededHead`.

The planner has **no cancellation authority**. It contains no POST/PUT/PATCH/DELETE request path and opens no workflow. Exact qualification workflows are out of scope by construction.

PR metadata lookups are scope-bounded: only PRs attached to queued `pull_request` runs from `.github/workflows/ci.yml` count against the v1 lookup budget.

Each plan entry commits to repository + run identity + run attempt + workflow identity + queued head + all normalized PR associations + classification.

The top-level `plan_commitment` commits to:

- repository;
- observation and expiry time;
- fixed planner scope;
- total **in-scope** inspected-entry count;
- eligible-entry count;
- the ordered commitment of every in-scope inspected entry, including current/unknown entries;
- the ordered eligible subset.

Plans expire after 15 minutes (`900` seconds). A future executor must consume the **whole committed, unexpired plan envelope** rather than accepting a detached entry commitment. Once expired, the operator must regenerate and re-review the plan even if a run still appears superseded.

The live CLI cannot supply a fake `--now` value to extend plan lifetime. Deterministic tests inject timestamps directly into `build_plan()` instead of widening runtime authority.

A future executor, if separately approved and qualified, must also re-fetch/revalidate repository, run attempt, run status/head, and all PR associations immediately before mutation. CI-GOV-001D-B / #1675 owns that mutation boundary; this branch does not implement it.

Important distinctions:

```text
SupersededHead != failed
SupersededHead != scientifically invalid
SupersededHead != cancellation already authorized
draft != stale
old != stale
queue pressure != cancellation authority
expired plan != reusable authority
out-of-scope queue churn != plan churn
```

Multi-PR associations are fail-closed: **every relevant open PR association returned for the run** must prove supersession. The first PR returned by GitHub is never treated as the sole owner of a run. Missing, malformed, closed, current-head, or identity-incomplete state makes the entry ineligible.
