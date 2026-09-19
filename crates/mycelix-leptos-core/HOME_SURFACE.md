# Mycelix Home Surface v1

Status: **experience architecture contract**

Scope: the shared Mycelix Home surface in the task-first shell.

## Governing rule

> **Home helps the user continue and decide what needs attention; it does not summarize the entire network.**

Home is a presentation projection over provider-owned state. It does not become a new authority, event log, ranking oracle, telemetry service, or cross-domain database.

## Primary sections

The first Home surface SHOULD remain small and task-oriented:

1. **Continue** — work the user can meaningfully resume.
2. **Needs attention** — provider-supplied items that currently need a decision, response, repair, or review.
3. **Local work** — locally durable work whose remote synchronization state still matters to the user.
4. **Pinned** — user-chosen people, spaces, projects, documents, conversations, or workflows.

A section MAY be absent when no provider supplies applicable items. Absence of a section MUST NOT imply that unavailable or unknown providers were successfully queried and found nothing.

## HOME-001 — Home is not a domain dashboard

The shared Home surface MUST NOT become one widget from every Mycelix domain.

A domain contributes an item only when it supports one of the Home tasks above. Domain topology remains available through Find, Apps, deep links, and domain-local navigation.

The shell MUST NOT make a domain more prominent merely because it has more metrics to display.

## HOME-002 — Continue is explicit resumability, not inferred recency

A Continue item means its provider supplied a valid resume destination or draft reference.

The shell MUST NOT infer resumability from:

- a recent page view;
- a transport connection;
- a cached route alone;
- a stale browser history entry;
- a prior successful action;
- an analytics event.

Recency MAY help order already-resumable items, but recency itself does not establish that an item can still be resumed safely.

## HOME-003 — Needs attention is provider-owned

A provider may surface an item as needing attention when its domain state establishes that a user-visible response, decision, repair, acknowledgement, or review is currently useful.

The shell MUST NOT infer attention from:

- trust tier;
- visual severity color;
- activity volume;
- popularity;
- message count alone;
- an AI confidence score;
- stale cached state whose current applicability is unknown.

An attention item is not automatically urgent, authorized, dangerous, or high priority.

## HOME-004 — Keep status vocabulary small

Shared Home presentation SHOULD begin with the smallest useful distinctions.

Recommended presentation-level distinctions are:

- `Continue`
- `Needs attention`
- `Waiting locally / not fully synchronized`
- `Pinned`

More specific lifecycle meaning belongs in the item or domain workflow, for example `Awaiting your vote`, `Conflict needs review`, or `Draft saved locally`.

The shell MUST NOT create a universal lifecycle enum that collapses incompatible domain meanings merely to make Home visually uniform.

## HOME-005 — Completed work should recede

Home SHOULD emphasize unfinished, actionable, resumable, or intentionally pinned work.

Completed items SHOULD normally disappear from Needs attention or become visually secondary when their provider establishes completion.

The shell MUST NOT mark an item completed merely because the user visited it or because a prerequisite completed.

## HOME-006 — Local durability and synchronization remain independent

Home's Local work section SHOULD consume the shared local/synchronization truth model when available.

At minimum it must preserve:

`Saved locally != synchronized != federated != confirmed != settled`

Examples of useful copy include:

- `Saved on this device`
- `Saved locally; waiting to sync`
- `Synchronizing`
- `Sync conflict needs review`
- `Remote synchronization unavailable`

The Home shell MUST NOT translate `Disconnected` into `Your work is lost` or translate `Connected` into `Everything is synchronized`.

## HOME-007 — Provider state is visible

A provider returning zero Home items is different from a provider that is unavailable or whose status is unknown.

Home SHOULD preserve at least:

- `Ready` — provider successfully supplied its Home contribution, which may be empty;
- `Unavailable` — provider cannot currently supply its contribution;
- `Unknown` — provider state cannot currently be established.

Home MAY compress this for ordinary users, but it must not strengthen unavailable/unknown state into a successful empty result.

## HOME-008 — Privacy is local-first by default

Continue and recent-work behavior SHOULD be computable from local/provider-owned state without requiring centralized behavioral telemetry.

The shared shell MUST NOT require a global clickstream, cross-domain behavioral profile, or remote analytics identity in order to populate Home.

If a provider uses remote history or recommendations, that provenance and privacy policy belong to that provider and MUST NOT be silently generalized as a requirement of the shared Home architecture.

## HOME-009 — Ordering is not authority

Home ordering MAY consider explicit user pinning, provider-supplied attention state, resumability, and recency.

Ordering MUST NOT be presented as:

- trustworthiness;
- authority;
- importance to the community;
- identity verification;
- correctness;
- evidence validity.

Any ranking rule used across providers SHOULD be inspectable and deterministic enough to test. User pinning SHOULD override machine ordering where practical.

## HOME-010 — Notifications are not Home items by default

Home is pull-oriented. It should let users inspect current work without turning every state change into an interruption.

Non-urgent updates SHOULD normally use ordinary status presentation or polite status announcements when dynamically inserted.

Assertive alerts SHOULD be reserved for important, time-sensitive conditions. A sync completion, background refresh, or routine provider update SHOULD NOT automatically become an assertive alert.

## HOME-011 — Every consequential item remains inspectable

An item that asks the user to make or review a consequential decision SHOULD provide enough context to understand:

- what needs attention;
- which provider/domain supplied the item;
- what current state is actually established;
- what selecting the item will do;
- where relevant evidence/provenance can be inspected.

Home MUST NOT replace domain evidence with a stronger summary.

## HOME-012 — Home actions hand off; they do not execute

Selecting Home items should navigate or hand off into domain-owned flows.

The shared Home surface MUST NOT directly execute consequential actions such as:

- voting;
- payments;
- capability grants/revocations;
- governance execution;
- publication;
- destructive deletion;
- settlement;
- identity changes.

Home may expose a safe preview or resume target. Current domain authority checks remain authoritative.

## Proposed provider contract

A future runtime contract SHOULD separate provider state from items:

```text
HomeBatch
  provider_id
  state: Ready | Unavailable | Unknown
  items: [HomeItem]

HomeItem
  id
  provider_id
  section: Continue | NeedsAttention | LocalWork | Pinned
  title
  detail?
  destination / draft handoff
  provider-supplied state label?
  updated_at?       # presentation recency only
```

The shared model should avoid a universal `priority` number until user research demonstrates that comparable cross-domain priority is meaningful.

## Qualification scenarios

`UX-HOME-002` and later browser/human qualification SHOULD cover:

1. resume a local draft while offline;
2. distinguish a Ready-empty provider from an unavailable provider;
3. identify which items actually need attention;
4. verify completed work recedes without being falsely marked complete;
5. inspect the provider/domain behind a consequential item;
6. recover from a sync conflict without losing the local-work truth;
7. pin/unpin an item without changing its domain authority or lifecycle state;
8. complete the same Home tasks with keyboard and narrow/mobile layouts;
9. verify routine dynamic updates are not announced as urgent alerts;
10. verify the shell does not require centralized behavioral telemetry to populate local Continue/Pinned state.

## Research alignment

This contract follows task-oriented service-design guidance: simplify the journey first, emphasize incomplete work that needs action, and keep task/status vocabularies as small as practical until research demonstrates additional distinctions are necessary.

It also follows accessible status-message guidance by separating ordinary state updates from genuinely important, time-sensitive alerts.

## Proposed PR sequence

1. `UX-HOME-002` — typed `HomeBatch` / `HomeItem` contract.
2. `UX-HOME-003` — local Continue + Pinned store.
3. `UX-HOME-004` — integrate local/sync truth from `UX-OFFLINE-001`.
4. `UX-HOME-005` — provider-owned Needs attention aggregation.
5. `UX-HOME-006` — accessible Home component.
6. `UX-HOME-007` — browser/offline/responsive qualification.
7. `UX-HOME-008` — human task-comprehension baseline.

No item should move to the next authority/lifecycle state merely because it appears on Home.
