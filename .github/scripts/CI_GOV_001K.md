# CI-GOV-001K — bounded qualification capacity oracle v0.2

This tranche makes the admission/capacity policy from #1964 executable without adding a workflow or any GitHub mutation authority.

## Authority boundary

The oracle is pure local policy evaluation. It does not query GitHub, start a workflow, cancel a run, rerun a job, add/remove a label, merge a PR, or publish a status.

It intentionally cannot convert queue age, project priority, draft age, or runner pressure into cancellation authority.

## Frozen v0.2 policy

```text
shared group:             mycelix-heavy-qualification-v1
queue mode:               max
cancel-in-progress:       false
max active heavy:         1
soft pending budget:      8
platform pending cap:     100
explicit label:           ci:qualify
ready auto-admission:     false
capacity snapshot max age: 30s
```

V0.2 is deliberately **explicit-token-only** while runner incident #697 is unresolved. A ready PR without `ci:qualify` is not automatically admitted to the heavy qualification lane.

The soft pending budget of 8 is intentionally far below the documented platform cap of 100. The oracle refuses new queue admission at 8 pending subjects, so the policy itself now carries the safety margin instead of leaving it as prose.

## Admission algebra

```text
closed PR
-> ClosedOutsideCapacityAuthority

subject_head != current_head
-> SupersededOutsideCapacityAuthority

current PR + ci:qualify
-> QualificationAdmitted

current PR without ci:qualify
-> AuthoringNotAdmitted

malformed/unknown metadata
-> MetadataInvalid
```

Ready state alone is intentionally insufficient in v0.2.

`SupersededOutsideCapacityAuthority` is observation only. CI-GOV-001D remains the only planned authority path for superseded-run cancellation.

## Capacity observation contract

A separately qualified observer must supply exactly:

```text
complete
age_seconds
active_count
pending_count
```

The oracle fail-closes:

```text
incomplete observation
-> AdmissionDeferredObservationUnknown

snapshot older than 30s
-> AdmissionDeferredObservationUnknown

malformed counts / active > 1 / pending > 100
-> AdmissionDeferredObservationInvalid

pending >= 8
-> AdmissionDeferredBudgetFull

active == 0 && pending == 0
-> ExecutionEligible

otherwise, while pending < 8
-> QueueAdmissionEligible
```

`ExecutionEligible` and `QueueAdmissionEligible` are offline policy decisions only. They do not prove GitHub requested, queued, assigned, or started a runner.

## Receipt

The canonical receipt binds the complete v0.2 policy—including the 8-entry soft budget and 100-entry platform cap—the exact subject/current head, labels, complete capacity observation, admission/capacity states, explicit false authority fields, proposition/nonclaims, and a SHA-256 commitment over the receipt body.

## Tests

The committed suite contains 28 cases covering exact policy; refusal of cancellation, ready auto-admission and budget/cap drift; explicit-token admission; closed/superseded boundaries; incomplete/stale/malformed capacity observations; active-width and platform-cap violations; exact budget-full behavior; immediate vs queued eligibility; metadata fail-closed behavior; receipt budget/non-authority/commitment binding; and proof that the oracle source contains no network or mutation client.

## Nonclaims

A source/test PASS does not qualify GitHub's live concurrency implementation, establish the correctness or freshness of a future capacity observer, solve #697, alter an existing queued run, qualify product code, authorize cancellation, or guarantee fairness/FIFO ordering. A future live pilot must independently prove actual scheduler behavior on exact workflow bytes.
