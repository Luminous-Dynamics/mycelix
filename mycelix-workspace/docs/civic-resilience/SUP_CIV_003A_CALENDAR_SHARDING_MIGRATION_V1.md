# SUP-CIV-003A — calendar-correct Support sharding migration note v1

Tracking issue: #2048  
Audited parent: `main@a85369699099d4c7524e502e531735eed4ab36f4`

## Purpose

Document the compatibility boundary for replacing the approximate Support year/month calculation with an exact deterministic UTC Gregorian conversion.

## Defect being repaired

The legacy helper computed:

```text
days = epoch_seconds / 86400
year = 1970 + days / 365
day_of_year = days % 365
month = floor(day_of_year / 30.44) + 1
```

That relation ignores leap years and real month lengths. Proven counterexamples include:

```text
2024-02-29 -> 2024-03
2024-12-31 -> 2025-01
2026-09-19 -> 2026-10
```

The replacement uses integer-only proleptic-Gregorian civil-date decomposition in UTC and defines pre-1970 behavior with Euclidean division.

## Audited current caller behavior

At the exact parent, the inspected `support-tickets` coordinator creates `ShardedTickets` links when:

- creating a normal support ticket; and
- promoting a preemptive alert to a support ticket.

The inspected coordinator read surfaces use requester/status/ticket-specific links and do not currently expose a monthly `ShardedTickets` query.

This means correcting the helper changes where **future writes** are indexed without, by itself, changing the currently inspected read APIs.

That observation is limited to the exact audited subject. It is not a theorem that no other future or external consumer can use the shard anchors.

## Historical compatibility

Existing links created with the old approximation remain under their old anchor strings. This tranche does **not** rewrite or delete those links.

Therefore:

```text
CorrectNewShardFunction != LegacyIndexMigrated
CorrectNewShardFunction != HistoricalLinksReindexed
```

Any future API that queries ticket history by calendar month must do one of the following before claiming complete historical results:

1. bounded dual-read of the corrected calendar anchor and the legacy approximate anchor(s) relevant to the requested period, with deduplication; or
2. an evidence-bearing reindex migration that creates corrected links for historical records and records its coverage/completeness limitations.

It must not silently query only the new anchor and claim historical completeness.

## Migration recommendation

Prefer an explicit migration receipt containing at least:

```text
migration_version
source_index_profile = legacy-approximate-v1
target_index_profile = utc-gregorian-v1
scanned_record_cut_ref
records_examined
legacy_links_observed
corrected_links_created
already_correct_links
refusals_or_decode_failures[]
started_at_ref
completed_at_ref
operator_or_automation_ref
```

Migration should be idempotent. Creating a corrected discovery link must not alter the underlying ticket entry or imply a new ticket revision.

```text
ReindexLink != TicketMutation
ReindexLink != CurrentStateAdmission
```

## Time semantics

The corrected shard is based on the `Timestamp` supplied to `sharded_anchor`. It establishes only deterministic mapping of that timestamp to a UTC calendar year/month.

It does not prove that the timestamp is trustworthy real-world event time.

```text
CalendarCorrectShard != TrustedEventTime
Ticket.created_at != IndependentlyVerifiedOccurrenceTime
```

## Pre-1970 timestamps

The new algorithm supports signed/pre-epoch Holochain timestamps using floor/Euclidean day semantics and the proleptic Gregorian calendar. This avoids an undocumented rejection boundary and keeps the helper total over the practical `Timestamp` range.

Support-domain policy may separately decide that particular business records must not predate a certain epoch; that is not a calendar-conversion concern.

## Required query rule

Until a historical migration is qualified:

```text
month query over mixed-era data
-> legacy-aware query or explicit incomplete-result status
```

Never:

```text
new calendar anchor queried
-> assume all historical records were stored there
```

## Claim ceiling

A PASS for SUP-CIV-003A establishes only that new calls to the exact helper map signed Holochain timestamps deterministically to the correct proleptic-Gregorian UTC year/month for the tested range, and that the legacy compatibility requirement is explicitly documented.

It does not establish historical reindex completion, current-ticket state, trusted event time, municipal chronology, Civic authority, privacy compliance, Johannesburg readiness, or deployment readiness.
