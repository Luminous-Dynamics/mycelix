# Mycelix Inbox / Activity Surface v1

Status: **experience architecture contract**

Scope: shared cross-domain Inbox / Activity presentation in the task-first Mycelix shell.

## Governing rule

> **Inbox aggregates things the user may need to know or revisit; it does not become the authority that created, acknowledged, resolved, or executed them.**

The shared Inbox is an envelope over provider-owned records. Domain lifecycle, delivery, acknowledgement, capability, authorization, execution, settlement, and evidence remain provider/domain concerns.

## INBOX-001 — Activity and Inbox are related but not identical

`Activity` is a historical/recent-events presentation.

`Inbox` is a user-oriented projection of provider-supplied items that are useful to inspect, revisit, review, or respond to.

The shared shell SHOULD NOT assume every domain event belongs in Inbox. High-volume event streams, telemetry, audit logs, and provenance histories remain separate unless a provider intentionally projects a user-facing item.

The existing generic `ActivityFeed` may render low-consequence recent activity, but it is not itself the Inbox truth model.

## INBOX-002 — Read/seen state is presentation state only

The shared shell MAY track whether an Inbox item is locally `Unread` or `Seen`.

That state MUST NOT be treated as:

- acknowledged by the provider;
- accepted;
- resolved;
- approved;
- rejected;
- executed;
- synchronized remotely;
- evidence that the user understood the item.

`Seen != acknowledged != resolved`.

If a domain has a real acknowledgement protocol, that remains a separate provider-owned action and lifecycle state.

## INBOX-003 — Provider requirements remain provider-owned

A provider MAY describe an item as requiring one of these presentation-level responses:

- no response required;
- review recommended;
- response requested.

These are user-facing interaction cues, not universal domain lifecycle states.

The shell MUST NOT infer a response requirement from color, message count, trust tier, popularity, an AI confidence score, or a generic severity number.

A provider-specific label such as `Vote requested before Sept 22`, `Resolve sync conflict`, or `Review access change` MAY be shown directly without translating it into a stronger shared claim.

## INBOX-004 — Provider availability is independent from an empty Inbox

At minimum, each Inbox provider contribution SHOULD preserve:

- `Ready` — provider successfully supplied its current Inbox projection; items may be empty;
- `Unavailable` — provider cannot presently supply its Inbox projection;
- `Unknown` — provider state cannot presently be established.

A provider that is unavailable MUST NOT disappear into `No new items`.

## INBOX-005 — An Inbox item is not authority

An item appearing in Inbox MUST NOT itself establish:

- identity verification;
- authority to act;
- capability validity;
- evidence validity;
- proposal passage;
- payment settlement;
- message delivery;
- remote acknowledgement;
- execution success.

The item may summarize provider-owned state, but consequential claims remain bound by the frontend truth invariants and should be inspectable when evidence/provenance exists.

## INBOX-006 — Selection hands off; it does not execute

The shared Inbox MAY:

- navigate to a domain-owned record/workflow;
- open a provider-owned preview or review surface;
- resume a draft/reply composer.

The shared Inbox MUST NOT directly:

- cast a vote;
- send money;
- grant or revoke capability;
- publish content;
- accept/reject a legal/governance action;
- delete consequential records;
- mark provider-owned work resolved;
- execute or settle an action.

A one-click action in a future Inbox requires its own explicit typed preview/authorization contract; it is not part of v1.

## INBOX-007 — Local dismissal does not mutate provider truth

The shell MAY support local presentation operations such as:

- mark seen/unseen;
- pin/save for later;
- hide from the local Inbox view;
- archive locally when the product definition makes that meaning clear.

These MUST NOT be presented as provider acknowledgement, provider deletion, domain resolution, or remote archival unless an authoritative provider operation actually occurs.

Local presentation state SHOULD be recoverable/inspectable enough that hiding an item does not destroy the underlying domain record.

## INBOX-008 — Ordering is not urgency or authority

Inbox ordering MAY consider:

- explicit user pinning;
- provider-supplied response requirement;
- timestamps/recency;
- locally unread/seen state.

Ordering MUST NOT be presented as a universal measure of:

- urgency;
- harm;
- trustworthiness;
- authority;
- correctness;
- social importance.

The shared model SHOULD NOT introduce one cross-domain numeric priority score until research demonstrates that such comparisons are meaningful and the scoring rule can be inspected.

## INBOX-009 — Notification interruption is separate from Inbox presence

An item being present in Inbox does not imply it should interrupt the user.

The shell SHOULD distinguish:

- durable Inbox presence;
- passive badge/count presentation;
- polite live-region update;
- assertive/time-sensitive alert;
- operating-system push notification.

The strongest interruption level must require an explicit provider/product policy. Routine synchronization, ordinary new activity, or background refresh SHOULD NOT automatically become assertive alerts.

## INBOX-010 — Counts must say what they count

If the UI presents a badge/count, its meaning must be explicit and stable, for example:

- `3 unread`;
- `2 responses requested`;
- `1 sync conflict`.

A generic red number MUST NOT silently combine unrelated concepts such as unread activity, unresolved conflicts, pending votes, and security changes.

Unknown/unavailable provider contributions also mean an aggregate count may be incomplete; the UI SHOULD expose that limitation rather than presenting the number as exhaustive.

## INBOX-011 — Offline behavior preserves local truth

Previously materialized Inbox items MAY remain inspectable offline when their local persistence permits it.

While offline, the shell MUST distinguish:

- cached/materialized item availability;
- provider freshness/availability;
- unsent local reply/draft state;
- synchronization state.

Opening a cached item while offline MUST NOT imply the provider is currently reachable or that a reply/acknowledgement has been delivered.

`Draft saved locally != reply sent != reply delivered != reply acknowledged`.

## INBOX-012 — Privacy and minimization

The shared Inbox MUST NOT require all provider records, private messages, or domain event streams to be copied into one central cross-domain database.

Providers SHOULD contribute the minimum presentation-safe envelope needed for the user-facing Inbox. Sensitive detail can remain behind provider-owned navigation/progressive disclosure.

Cross-domain analytics/telemetry MUST NOT be required merely to know whether an item is locally seen or pinned.

## INBOX-013 — Provenance of the envelope remains visible

Every item SHOULD retain its `provider_id` or equivalent source identity so the user can tell which Mycelix context supplied it.

Provider identity is provenance, not a trust or authority score.

If the shell aggregates multiple items referring to the same underlying record, it SHOULD preserve the contributing providers rather than erase disagreements or gaps to create a cleaner narrative.

## Proposed v1 runtime contract

```text
InboxBatch
  provider_id
  state: Ready | Unavailable | Unknown
  items: [InboxItem]

InboxItem
  id
  provider_id
  title
  detail?
  local_seen: Unread | Seen
  response: NoneRequired | ReviewRecommended | ResponseRequested
  provider_state_label?
  occurred_at? / updated_at?   # presentation timestamps only
  target: Navigate | Preview | ResumeDraft
```

The shared type intentionally omits:

- `acknowledged` unless supplied as a separate provider-owned domain state;
- `resolved` as a shell-controlled boolean;
- universal severity/priority numbers;
- execution callbacks.

## Qualification scenarios

Future Inbox qualification SHOULD cover at least:

1. distinguish Unread from provider acknowledgement;
2. mark an item Seen without changing provider lifecycle truth;
3. distinguish Ready-empty from Unavailable/Unknown provider state;
4. navigate/review an item without executing its consequential action;
5. inspect cached/materialized Inbox items offline;
6. create a reply/draft offline while correctly representing delivery/sync state;
7. show an aggregate count that becomes explicitly incomplete when a provider is unavailable;
8. preserve provider provenance for consequential items;
9. keyboard/mobile access to list, filters, and detail handoff;
10. verify routine updates are not announced as urgent/assertive alerts;
11. verify hiding/archiving locally does not destroy the provider record;
12. verify no centralized behavioral telemetry is required for local Seen/Pinned state.

## Relationship to Home

Home and Inbox are different projections:

- **Home** asks: `What should I continue or pay attention to now?`
- **Inbox** asks: `What has arrived or been projected for me to inspect/revisit/respond to?`

A provider item MAY appear in both when the provider deliberately contributes both projections. The shell MUST NOT assume every Inbox item needs Home attention, or every Home attention item came through Inbox.

## Proposed PR sequence

1. `UX-INBOX-002` — typed provider/item envelope.
2. `UX-INBOX-003` — local Seen/Pinned presentation state.
3. `UX-INBOX-004` — cached/offline Inbox projection.
4. `UX-INBOX-005` — accessible Inbox list/detail presentation.
5. `UX-INBOX-006` — explicit count semantics and incomplete-provider disclosure.
6. `UX-INBOX-007` — browser/offline/keyboard qualification.
7. `UX-INBOX-008` — first representative provider integration.

The v1 shell should prefer clarity over a sophisticated notification center. If an item needs a domain-specific lifecycle, show that lifecycle rather than inventing a universal one.