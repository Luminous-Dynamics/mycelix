# hearth-care-ledger

Pure Rust contract for recurring Hearth Care occurrence/completion evidence.

HTH-AUTO-005R1 deliberately contains no HDK/HDI dependency. It freezes the
semantics that the Care zome, browser UI, planner, replay harness, and future
edge runtime must agree on before the DHT schema is migrated.

## Core model

`CareSchedule` remains the durable template. A concrete horizon produces a
`CareOccurrence`; completing that occurrence produces immutable
`CareCompletion` evidence. Completing one occurrence never retires the source
schedule.

Occurrence identity is deterministic from `(schedule_id, concrete window)`.
The identity is length-prefixed instead of relying on process/runtime hashing,
so the same inputs yield the same identifier across architectures.

Completion evidence separates:

- `performed_by`: the member who actually did the work and receives workload attribution;
- `recorded_by`: the member/agent who authored or attested the completion record.

This matters because a guardian may record a child's or dependent member's
completion without becoming the person who performed that work.

## Duplicate and conflict semantics

Holochain coordinator pre-checks cannot prove global uniqueness under
concurrency. R1 therefore makes canonicalization explicit:

- repeated identical record IDs are deduplicated before canonicalization;
- identical occurrence materializations with the same occurrence id collapse
  to the lexicographically-smallest record id;
- the same occurrence id with different semantic content is a conflict and
  fails closed;
- multiple completions for one occurrence count as **one** completed task;
- different recorders may independently attest the same performer/duration
  without creating a workload conflict;
- disagreement about `performed_by` or `actual_minutes` is surfaced and
  excluded from member workload totals instead of being guessed.

The DHT wiring tranche should use these same rules rather than claiming global
uniqueness from `get_links` followed by `create_entry`.

## Effort provenance

Digest v2 keeps actual and estimated effort separate:

- `known_actual_minutes`
- `unknown_actual_duration_count`
- `estimated_minutes_for_completed_tasks`

Legacy one-hour-per-task digests can be imported only as
`LegacyEffortEstimate`, explicitly tagged `LegacyOneHourPerTask`; they are not
actual work evidence.

## Intended DHT follow-up

HTH-AUTO-005R2 should add Care occurrence/completion entries and links, bind
`recorded_by` to DHT authorship, preserve legacy records as readable history,
use this contract for identity/canonicalization/digest semantics, and move new
recurring-task UI/actions away from `complete_task` on the schedule template.
