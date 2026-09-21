# Hearth Snapshot Publication Contract v1

Status: design freeze for the canonical `mycelix-workspace/mycelix-hearth` Leptos frontend.

This contract follows HTH-UI-004 (#2512), which prevents stale asynchronous reads from publishing after their connection/signing generation becomes obsolete. HTH-UI-004 establishes **cross-generation isolation**. This document freezes the stronger next invariant: **within one still-valid generation, the primary Hearth read set must not become visible as a hybrid of different stages of the same refresh**.

## 1. Problem

The primary Hearth snapshot is assembled from multiple independent Holochain reads:

- caller Hearth catalog / current Hearth source;
- caller role;
- members;
- kinship graph / bonds;
- gratitude stream;
- Care schedule;
- rhythms;
- presence;
- Decisions;
- per-Decision current votes.

HTH-UI-004 generation-guards every awaited boundary, but a valid generation can still publish each family as soon as that family returns.

That permits a temporary state such as:

```text
new members
+ new bonds
+ old decisions
+ old votes
+ decisions availability = Unknown
```

or:

```text
new decisions
+ old votes
```

while the same refresh is still running.

Those states are not necessarily false individually, but they are not one coherent source snapshot. Derived UI such as Home attention, Inbox alignment, membership/name lookup, vote alignment, relationship presentation, or later multi-Hearth selection must not mistake an intermediate refresh state for a completed snapshot.

## 2. Core theorem

For one accepted primary load token `G`:

```text
source reads for G
        -> private staged draft G
        -> validate G is still current
        -> one publication boundary
        -> visible snapshot G
```

Never:

```text
source read 1 -> visible state
source read 2 -> visible state
source read 3 -> visible state
...
```

The public theorem is:

```text
partial valid-generation read progress
!= published Hearth snapshot
```

and:

```text
published snapshot generation N
contains only state staged for generation N
```

## 3. Snapshot draft

The loader must build a private draft that is not directly observed by page components.

The draft carries, at minimum:

- selected/current Hearth value;
- caller role;
- members;
- bonds;
- Care schedules;
- Decisions;
- current votes;
- gratitude;
- rhythms;
- presence;
- per-family `AvailabilityStateKind` for every source-backed family in this read set.

Unsupported families remain explicitly `Unavailable`; the draft must not fabricate data for Stories, Emergency, Resources, Milestones, Autonomy, or any later family that is not part of the read set.

The draft is ordinary private Rust data. It is not authority, is not a second cache, and must not outlive the load that produced it.

## 4. Availability belongs to the same transaction

Data and provenance must commit together.

For example:

```text
members = new Vec<MemberView>
members availability = Live
```

are one publication fact.

The implementation must never expose:

```text
new members + previous availability
```

or:

```text
previous members + Live availability for the new query
```

as the completed snapshot.

`Live`, `Empty`, `Degraded`, and `Unavailable` remain source-quality/provenance states. Atomic publication does not strengthen their meaning.

## 5. Error staging

A source error does not abort the entire draft unless the error makes downstream source identity impossible.

Examples:

- members query fails -> stage members `Unavailable` and an empty members payload for this snapshot;
- bonds query partially decodes -> stage decoded bonds + `Degraded`;
- gratitude query returns zero valid records from a successful empty source -> stage empty gratitude + `Empty`;
- one vote query fails -> stage the vote family according to the existing partial-query rules, normally `Degraded` or `Unavailable` depending on the evidence collected.

A failed source must never silently retain the previous generation's payload while attaching the new generation's availability.

The visible snapshot may therefore be partially available, but it is still one coherent refresh result.

## 6. Root-source failure

`get_my_hearths` is a root dependency for the current single-Hearth loader.

If it cannot be established:

```text
current Hearth = none
all Hearth-scoped supported payloads = empty
root-dependent supported availability = Unavailable
```

must be staged and published together, provided the generation remains current.

If the established source proves the caller has zero Hearths:

```text
current Hearth = none
all Hearth-scoped supported payloads = empty
root-dependent supported availability = Empty
```

must be staged and published together.

No previous-Hearth payload may survive either result as if it belonged to the new snapshot.

## 7. Decision / vote closure

Decisions and votes are one dependency family for snapshot coherence.

The draft may not publish newly loaded Decisions while retaining current votes from the previous visible snapshot.

Required order inside the private draft:

```text
load Decisions
-> validate/decode Decisions
-> load current votes for the staged Decisions
-> derive Decision availability
-> derive Vote availability
-> stage both
```

Only the final publication boundary may expose either family.

Immutable vote-history and DecisionOutcome providers remain separate source contexts with their own generation/alignment rules. This contract does not merge those sources into the primary snapshot. They must continue to key/reload from the newly published Decision snapshot rather than observing its private build process.

## 8. Generation interaction

HTH-UI-004 remains the outer gate.

A draft may publish only when:

```text
draft.token == active LoadGeneration
```

immediately before publication.

If the generation changes at any point:

```text
staged draft -> discard
```

No field from that draft may become visible.

A stale load also has no authority to clear a newer load's `loading` state; HTH-UI-004 already owns that invariant.

## 9. Publication boundary

Publication must have a single reactive commit boundary.

Acceptable implementation strategies include:

1. a Leptos reactive batch that updates all Hearth payload signals and the availability signal as one notification boundary; or
2. replacing the many payload signals with one immutable published snapshot signal and deriving views from it.

For the current frontend, the minimal-change implementation should prefer a reactive batch unless code review proves the framework/version does not provide the required notification semantics.

Within that boundary, payloads should be assigned before the final provenance/publication marker so any downstream gate cannot observe a claimed established family before its matching payload is present.

## 10. Refresh semantics

Starting a refresh may continue to set the public readable families to `Unknown` so route/domain boundaries do not present the previous snapshot as freshly established.

However, the new payload draft remains private until commit.

Therefore:

```text
refresh starts
-> visible availability Unknown
-> previous payloads may remain physically stored but are gated/non-authoritative for presentation
-> draft builds privately
-> atomic publication replaces payloads + availability
```

An implementation may alternatively clear visible payloads at refresh start, but it must do so consistently; it must not selectively clear only some source families.

## 11. Multi-Hearth boundary

The current loader still uses the first `get_my_hearths` record as a provisional single-Hearth selection rule. This contract does not bless that behavior as final multi-Hearth semantics.

A later explicit Hearth-selection tranche must make the selected Hearth identity part of the snapshot key:

```text
(connection generation, connected agent, selected Hearth)
```

Changing the selected Hearth invalidates the current primary load and requires a fresh atomic snapshot for the newly selected Hearth.

No staged or published payload from Hearth A may be reused as established state for Hearth B.

## 12. Signal boundary

Future conductor signals are invalidation/reconciliation hints, not permission to mutate the published snapshot directly.

The intended future flow is:

```text
signal
-> mark relevant source stale / request reconciliation
-> source reads
-> staged draft
-> atomic publication
```

not:

```text
signal payload -> authoritative UI mutation
```

## 13. Mutation reconciliation

Existing successful mutations may reconcile an exact authoritative returned `Record` into local state under their already-reviewed transition contracts.

This contract does not remove that behavior.

However, a subsequent full snapshot refresh is authoritative for snapshot reconciliation and must replace the visible primary read set atomically. A mutation result from an obsolete connection/snapshot generation must not be used to complete a newer draft.

## 14. Required regression coverage

The implementation child must add pure or host-free tests for at least:

1. **no partial publication** — staging one family does not mutate the published context;
2. **commit replaces all supported payload families together**;
3. **availability and payload agree after commit**;
4. **root Unavailable clears old scoped payloads in the published result**;
5. **root Empty clears old scoped payloads in the published result**;
6. **new Decisions cannot publish with previous votes**;
7. **stale generation discards a fully built draft**;
8. **current generation may publish a partially available but internally coherent draft**;
9. **unsupported families remain Unavailable and do not inherit demo data**.

Browser/e2e qualification should later exercise a deliberately delayed source sequence and verify the UI never observes a mixed completed snapshot.

## 15. Non-claims

Atomic snapshot publication does not prove:

- continuous real-time freshness;
- global DHT completeness;
- write authorization;
- current civic eligibility;
- multi-Hearth selection correctness;
- semantic truth of human relationship quality;
- absence of concurrent remote writes after the snapshot was read;
- transactionality across Holochain zomes.

It proves only a browser presentation/state invariant:

> one completed Hearth refresh is published as one coherent locally observed source snapshot, or not published at all.

## 16. Intended implementation sequence

The implementation child should remain narrow:

1. introduce a private `HearthSnapshotDraft` / equivalent;
2. convert existing source reads from direct `RwSignal` writes into draft writes;
3. retain the HTH-UI-004 generation checks after awaits;
4. publish the draft through one reactive boundary only if its token is still current;
5. preserve existing Record->View validation and availability derivation rules;
6. add the regression matrix above;
7. make no zome/API/schema changes.

Only after this qualifies should the primary loader be extended with explicit multi-Hearth selection or real-time signal-driven reconciliation.
