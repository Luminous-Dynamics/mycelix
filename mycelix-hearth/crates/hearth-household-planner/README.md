# hearth-household-planner

Deterministic Care + Rhythms work planning for Mycelix Hearth.

HTH-AUTO-005 is the first automation vertical intended to improve an ordinary household with **zero smart-home hardware**. It turns normalized care occurrences, member availability, current presence, and rhythm commitments into an explainable work plan.

## Important boundary

The current Hearth DHT schemas are not silently reinterpreted:

- `CareSchedule.recurrence` is a template-level cadence, not an exact due-time.
- `Rhythm.schedule` is human-readable free text and is **not parsed by this crate**.
- the current care digest estimates every completion as one hour; this planner does **not** treat that estimate as verified effort.
- a missing task duration or scheduling window is surfaced as `NeedsInput`, never invented.

An adapter may normalize DHT records into planner inputs, but that normalization must make any additional planning metadata explicit.

## Planner inputs

`MemberProfile` contains only planning facts supplied by the household or an authoritative adapter:

- stable member ID and display name,
- active/inactive state,
- current presence,
- explicit availability windows,
- planning capacity,
- already-planned minutes,
- verified recent-care minutes,
- declared capabilities.

`CareOccurrence` is a horizon-local occurrence derived from a schedule/template or entered directly. It carries:

- stable occurrence/source references,
- current assignee when one exists,
- assignment policy (`Fixed`, `PreferCurrent`, `Pool`),
- explicit estimated minutes,
- explicit scheduling window,
- required capabilities,
- optional eligible-member allowlist,
- priority.

`RhythmCommitment` blocks time for participating members. The planner does not parse `Rhythm.schedule`; adapters must resolve a concrete window first.

## Deterministic assignment

Occurrences are ordered by:

1. higher priority,
2. earlier scheduling-window end,
3. stable occurrence ID.

Candidate members are filtered by active status, presence, explicit eligibility, required capabilities, planning capacity, and availability. Hard rhythm commitments and already-assigned work block time.

Candidate ordering is deterministic and minimizes projected burden using integer-only arithmetic. Recent verified care effort may be included with a configurable basis-point history weight. Stable member ID is the final tie-breaker.

The planner never converts this into a ranking of people. The score describes **projected workload for one scheduling decision**, not worth, reliability, or productivity.

## Fail-closed / needs-human states

The planner does not guess when it lacks evidence. Examples:

- missing effort estimate,
- missing concrete time window,
- unknown presence when household policy forbids planning on unknown presence,
- fixed assignee unavailable,
- no capable/eligible member,
- no non-conflicting time slot.

These appear as `PlanIssue` records with explicit reasons.

## Automation output

`compile_recommendation_plan` converts successful assignments into A2 `Recommend` plan steps using capability `home.care.recommend`. It does not mutate Care entries or reassign anyone automatically. Consequential household changes remain separate, authorized actions.

This is intentional: AUTO-005 proves useful orchestration before introducing a durable care-occurrence/completion migration or device adapters.

## Next tranches

- add an immutable care-occurrence/completion ledger so recurring schedules are not completed permanently after one occurrence,
- add a Holochain normalization adapter for Care/Kinship/Rhythms,
- replay historical household timelines through AUTO-004B,
- add opt-in workload/fairness counterfactuals,
- add a virtual Hearth action adapter for accepted household recommendations.