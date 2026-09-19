# CI-GOV-001K-C — read-only qualification admission adviser v0.1

This tranche closes the policy/observation composition gap without granting mutation authority.

It is a direct child of the hardened CI-GOV-001K-B observer, which itself descends from the frozen CI-GOV-001K-A v0.2 policy source.

## Purpose

Given an exact qualification subject `(PR number, frozen subject head)`, the adviser answers a narrow question:

> If `ci:qualify` were added to this exact current PR subject now, would the frozen v0.2 admission/capacity policy consider that label addition policy-eligible?

It does **not** add the label, dispatch a workflow, cancel a run, publish a status, merge anything, or grant operational admission authority.

## Read sequence

The adviser performs at most three bounded reads:

```text
1. GET exact PR snapshot
2. GET exact live concurrency-group observation through the frozen 001K-B observer
3. GET exact PR snapshot again
```

The two PR snapshots bind the fields that can change the policy result:

- open/closed state;
- draft state;
- current head SHA;
- exact current label set.

Any change across the observation window fails closed as `pr_changed_during_observation`.

## Subject model

The caller supplies the exact frozen `subject_head` that is proposed for qualification.

The adviser reads the current PR head from GitHub and builds the proposed oracle subject using the *current* label set plus `ci:qualify` if it is not already present.

Therefore:

- a moved current head is classified by the frozen oracle as superseded;
- a closed PR is denied;
- an already-tokened PR is reported as `TokenAlreadyPresent` rather than generating a second admission recommendation;
- ready state alone never bypasses the explicit-token v0.2 policy.

## Capacity/freshness binding

The live observation is produced by the inherited hardened 001K-B observer. The adviser verifies the observer receipt commitment, repository/group/API identity, authority ceiling, and complete-count invariants.

After the post-observation PR re-read, the adviser takes a decision clock sample and derives:

```text
age_seconds = decision_at_epoch_seconds - observed_at_epoch_seconds
```

Future observations, invalid decision clocks, or observer identity/commitment drift fail closed. The exact derived age and counts are passed to the frozen 001K-A v0.2 oracle, which enforces its 30-second freshness bound, max-active width, platform cap, and 8-entry soft pending budget.

## Adviser states

```text
IncompleteFailClosed
TokenAlreadyPresent
PolicyAdmissionDenied
PolicyLabelAdmissionDeferred
PolicyLabelAdmissionEligible
```

`PolicyLabelAdmissionEligible` is only an offline/read-only policy result.

Even in that state the receipt contains:

```text
grants_operational_label_mutation = false
grants_actions_mutation           = false
grants_cancellation_authority     = false
requires_live_scheduler_qualification = true
```

The live `queue:max` scheduler pilot must qualify before any later mutation component may treat this policy result as operational admission authority.

## Network boundary

The adviser itself owns only the PR GET. Capacity observation is delegated to the inherited 001K-B observer.

PR endpoint template:

```text
GET https://api.github.com/repos/Luminous-Dynamics/mycelix/pulls/{pr_number}
X-GitHub-Api-Version: 2026-03-10
```

Reads use a bounded timeout, 1 MiB response limit, redirects disabled, and optional `GITHUB_TOKEN` only as a read credential.

There is no POST, PATCH, PUT, DELETE, label mutation, workflow dispatch, cancellation, rerun, status publication, or merge client.

## Local preflight

The authored adviser/test logic passed a 28-case local preflight against interface-compatible copies of the frozen A/B contracts. The corpus covers:

- empty/active/pending/budget-full capacity states;
- observation staleness and backwards-clock refusal;
- PR head/label/draft/open-state TOCTOU drift;
- superseded and closed subjects;
- already-present token behavior;
- proposed-token construction;
- PR API/status/repository/label validation;
- observer commitment and authority-ceiling validation;
- decision-time receipt binding;
- fixed PR endpoint, bounded timeout, redirect refusal, token non-retention, and no-mutation source audit.

This is implementation preflight only. An exact-source qualifier must independently execute the committed corpus against the canonical inherited A/B bytes.

## Nonclaims

This tranche does not establish live scheduler configuration or `queue:max` behavior, does not mutate a PR, does not enqueue a workflow, does not guarantee fairness/FIFO, does not solve runner incident #697, and establishes no product or scientific claim.
