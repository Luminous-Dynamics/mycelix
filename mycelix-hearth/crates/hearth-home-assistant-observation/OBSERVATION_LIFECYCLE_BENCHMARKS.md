# Observation lifecycle benchmarks (HA-0.2C)

**Status: proposed targets, not measured results.** No latency, freshness, durability, or provider-conformance result is claimed by this catalogue. Each result must bind exact source SHA, profile/corpus digest, runtime and hardware profile, raw sample set, and run identity.

## Paradigm shift: observation is revocable evidence

A state value is not a permanent fact. Every positive observation is scoped by an entity, lifecycle revision, session epoch, and local freshness deadline. A stale positive must become unusable even when a transport or projector is still holding a cache entry.

| ID | Measurement | Acceptance target | Current status |
|---|---|---|---|
| HLC-001 | Positive currentness after unknown/unavailable | Zero stale positive revisions remain current; exactly one invalidation record per active positive revision | Not measured |
| HLC-002 | Explicit entity removal | Zero removed entities remain current; removal never emits a positive physical-state claim | Not measured |
| HLC-003 | Freshness expiry | 100% of positives become non-current at `now >= valid_until_micros`; repeated expiry emits no duplicate invalidation | Not measured |
| HLC-004 | Disconnect and session replacement | Zero old-epoch positives are current as soon as lifecycle leaves `Live` | Not measured |
| HLC-005 | Reconnect snapshot omission | Every prior positive is invalidated immediately when the old session is replaced; zero omitted entities resurrect; snapshot omission emits no duplicate tombstone | Not measured |
| HLC-006 | Malformed/current sample rejection | Zero partial positive emissions; all current positives are invalidated before the session enters `NeedsNewSession` | Not measured |
| HLC-007 | Replay resistance | Zero old/lower session epochs restore currentness; per-entity revision never decreases across checkpoint restore | Not measured |
| HLC-008 | Bootstrap gate | Zero positive observations become current before matching subscription acknowledgement and explicit snapshot-applied acknowledgement | Not measured |
| HLC-009 | Invalidation idempotence | Zero duplicate invalidation records for repeated removal/unknown/unavailable/expiry signals after the positive is already invalidated | Not measured |
| HLC-010 | Semantic freshness | Equivalent state replay and metadata-only changes emit zero new positive revisions and extend freshness zero times | Not measured |
| HLC-011 | Public evidence privacy | Zero raw state, arbitrary attributes, URLs, credentials, or token markers in lifecycle records/checkpoints | Not measured |
| HLC-012 | Small-delta overhead | p95 lifecycle commit ≤ 2 ms for one-entity updates on named Raspberry Pi 5 hardware, excluding JSON transport and provider latency | Blocked on runtime measurement |
| HLC-013 | 1,024-entity snapshot | p95 snapshot reconciliation ≤ 25 ms on the same named host; separately report allocation and record count | Blocked on runtime measurement |
| HLC-014 | Checkpoint representation | A round-trip preserves profile binding, highest epoch, and all tombstone revisions while restoring zero current positives | Not measured |
| HLC-015 | Crash durability | No acknowledged lifecycle revision can roll back after process/power failure on the supported storage profile | Blocked on a durable persistence implementation |
| HLC-016 | Clock-regression fail-closed | 100% of negative-time checks return no current revision, emit invalidation for each active positive, and keep prior positives non-current until a strictly newer session/bootstrap | Not measured |
| HLC-017 | Append-before-publish fault injection | Zero candidate phase/currentness/observation publication when append-and-sync fails; wrapper becomes unhealthy and returns no positive revision | Not measured |
| HLC-018 | Journal replay integrity | 100% rejection of sequence gaps/duplicates, unsupported schema, profile mismatch, illegal transition order, revision gaps, and invalidation references to the wrong positive revision | Not measured |
| HLC-019 | Cold recovery | Zero positives restored as current; recovery barrier must be durable before any new event; a strictly higher epoch and fresh snapshot are required | Not measured |
| HLC-020 | Durable append latency | Report p50/p95/p99 sync latency separately from reducer overhead on each named backend; no target until a real backend exists | Blocked on durable backend |
| HLC-021 | Recovery rollback | Zero acknowledged ledger sequence rollback across process-kill/power-loss points on the named storage profile | Blocked on durable backend |
| HLC-022 | Entity-ID rename / identity drift | 100% of rename/removal cases invalidate the exact old positive; zero automatic aliasing or publication for a newly seen, unallowlisted ID; raw new state remains absent from the minimized journal | Not measured |

## Correctness workload

Test at least the following transitions and permutations: session start, subscription bind/ack, initial snapshot projection, snapshot acknowledgement, positive state, semantically equivalent replay, unknown, unavailable, removal, expiry, malformed delta, metadata drift, disconnect, higher-epoch reconnect, reconnect snapshot with the entity present, reconnect snapshot omission, stale epoch delivery, and checkpoint restore.

For every rejected projector update, capture both the projector pre/post-cache digest in the *test harness* and the lifecycle before/after state. Test evidence can carry a local test digest, but production lifecycle records must not publish unsalted state hashes.

For clock behavior, run the authoritative `check_currentness` gate with a regressed local monotonic timestamp, then with a later/recovered timestamp. Require the first call to return a clock-fault invalidation and enter `NeedsNewSession`; the second must not revive any prior positive. A read-only lookup returning `None` once is not sufficient evidence of a committed invalidation.

Include a two-entity delta where the first entity is valid and a later sorted entity violates its codec/range bound. Acceptance requires zero staged observations escaping and explicit invalidation of every prior positive in the rejected session.

## Reproducible method

- Use deterministic event seeds and preserve the seed and corpus digest with every counterexample.
- Measure lifecycle bookkeeping separately from JSON decode, projection, semantic mapping, local persistence, transport latency, and device postcondition.
- Use at least 5 independent timing runs and report p50/p95/p99, allocations, max retained bytes, and failure counts.
- Run mutation checks that remove expiry checking, epoch monotonicity, subscription binding, snapshot acknowledgement, source-state invalidation, removal invalidation, and rejected-stream teardown. Every mutation must be killed by a named test.
- Report checkpoint crash/power-cut testing separately. A serializable struct is not durable until storage ordering, fsync, atomic replacement, and rollback behavior have been tested.

## Claim ceiling

Passing this pure lifecycle suite would establish behavior only for the tested source, profile and synthetic events. It would not prove Home Assistant's authenticity, persist the checkpoint, establish household authority, verify any physical device state, qualify any provider, or replace Hearth's 004C exclusive lease, 004D adapter admission, 004E runtime binding, durable effect journal, or postcondition reconciliation.


## Identity drift rationale

Home Assistant's [October 2026 entity lifecycle update](https://developers.home-assistant.io/blog/2026/10/12/new-entity-lifecycle-hooks/) supports entity IDs changing in place, and [Core 2026.11 restore-state changes](https://developers.home-assistant.io/blog/2026/10/06/restore-state-last-states-removal/) move restored-state lookup to entity registry identity instead of assuming the text entity ID remains a permanent identity. Hearth still uses an exact, operator-admitted entity ID as the observation-profile coordinate; a rename must therefore invalidate the old coordinate and require explicit profile remapping. Name, model, or matching state is not sufficient evidence to merge identities.
