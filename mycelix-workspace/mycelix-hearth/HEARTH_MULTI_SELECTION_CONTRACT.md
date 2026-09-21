# Hearth Multi-Selection Contract v1

Status: design freeze for the canonical `mycelix-workspace/mycelix-hearth` Leptos frontend.

Depends on:

- HTH-UI-004 / #2512 — stale async snapshot publication is generation-guarded;
- HTH-UI-005A / #2547 — a completed Hearth refresh is staged and published atomically;
- #2549 — Kinship must distinguish current Active Hearth membership from historical `AgentToHearths` discovery.

This contract defines how a browser chooses one Hearth when the connected agent may be associated with zero, one, or many Hearths.

## 1. Established source problem

The current Kinship query is:

```text
get_my_hearths()
= get AgentToHearths links
-> resolve linked Hearth records
```

`leave_hearth` changes the matching membership state to `Departed` but preserves the historical `AgentToHearths` link.

Therefore:

```text
returned by get_my_hearths
!= current Active member
```

The current frontend then chooses the first returned Hearth.

Therefore a second independent non-theorem exists:

```text
first record returned by discovery query
!= user's selected Hearth
```

Neither historical link presence nor source-list order may be promoted into current selection authority.

## 2. Three distinct concepts

Hearth must model these separately:

### 2.1 Discovered Hearth

A Hearth reachable from historical/discovery evidence such as `AgentToHearths`.

Discovery means only:

> this agent has provenance connecting it to this Hearth.

It does not establish current membership.

### 2.2 Active Hearth

A Hearth for which the authoritative Kinship source establishes the connected agent's **current latest matching membership state** as `Active`.

The Active catalog is the only catalog from which the live browser may select a current Hearth.

### 2.3 Selected Hearth

One exact typed Hearth `ActionHash` chosen for the current browser presentation/session.

Selection is navigation/presentation state. It is not mutation authority.

Every consequential zome call continues to prove its own membership, role, civic eligibility, deadline, object binding, or other server-side authorization independently.

## 3. Required source contract

The canonical browser should consume an explicit source-backed **Active Hearth catalog**.

Preferred source direction is an endpoint such as:

```text
hearth_kinship.get_my_active_hearths(())
```

or a compatibility-reviewed equivalent whose semantics are explicitly current Active membership.

The Active catalog must not decide membership from `AgentToHearths` alone.

For every included Hearth, the server/source boundary must establish the caller's canonical latest matching membership revision as Active.

A historical Active revision followed by a later Departed revision must not qualify.

## 4. Catalog availability

The Active catalog itself has source availability independent from selection:

- `Unknown` — not yet established;
- `Unavailable` — authoritative query could not be completed;
- `Degraded` — only a partial/invalid catalog could be established;
- `Empty` — successful authoritative query established zero Active Hearths;
- `Live` — successful authoritative query established one or more Active Hearths;
- `Mock` — explicit Demo only.

The browser may not turn Unknown, Unavailable, Locked, or Degraded catalog evidence into a selected live Hearth.

## 5. Zero / one / many theorem

### Zero Active Hearths

```text
catalog = established Empty
-> selected Hearth = None
-> Hearth-scoped primary snapshot = established Empty / no target
```

The UI may offer founding/invitation onboarding, but it must not preserve an old selected Hearth as current.

### Exactly one Active Hearth

```text
catalog = Live([H])
-> browser may auto-select H
```

This is convenience, not authority. Zome mutations still verify independently.

### More than one Active Hearth

```text
catalog = Live([A, B, ...])
-> no source-list-order auto-selection
```

The browser must require an explicit selection unless a valid remembered preference rule below applies.

`records.first()` is prohibited as a selection policy.

## 6. Remembered preference

A browser may remember the last selected Hearth as local convenience state.

The remembered value must be an exact typed `ActionHash` carrier.

It may auto-select only when all of the following are true:

1. the fresh Active catalog is established Live;
2. the remembered hash parses as the expected ActionHash kind;
3. exactly one catalog entry has that exact hash;
4. the connected agent identity matches the preference namespace/session identity.

If any condition fails:

```text
remembered preference -> ignore
```

Never:

```text
remembered preference -> grant membership / resurrect departed Hearth / create catalog entry
```

Preferences are not synchronized into consensus state merely to support navigation.

## 7. Presentation ordering is not selection

The selection UI may sort the Active catalog for stable presentation.

Acceptable presentation keys include human-readable name with ActionHash tie-break, or another documented deterministic UI key.

Sorting has no authority meaning.

```text
first item after presentation sort
!= automatically selected Hearth
```

unless the exactly-one rule applies.

## 8. Selection state machine

The browser should carry a first-class selection state rather than overloading `current_hearth`.

Conceptually:

```text
SelectionState::Unestablished
SelectionState::NoActiveHearths
SelectionState::ChoiceRequired { catalog }
SelectionState::Selected { hearth_hash }
SelectionState::Degraded { reason }
```

Names may vary, but the distinctions must remain explicit.

`current_hearth` should become the source-backed Hearth value corresponding to a `Selected` hash, not the place where the selection decision itself is hidden.

## 9. Snapshot key

Once selection exists, the primary snapshot identity is at minimum:

```text
(connection generation,
 connected AgentPubKey,
 selected Hearth ActionHash)
```

A snapshot result is publishable only for the exact key that initiated it.

Changing any component invalidates the active snapshot generation.

## 10. Switching Hearths

Changing selection from A to B is a hard source-boundary transition.

Required sequence:

```text
explicit selection B
-> invalidate active A snapshot generation
-> mark/clear A-scoped established availability
-> invalidate dependent A-derived sources
-> load private B draft
-> atomically publish B snapshot
```

The UI may retain old bytes internally while B loads only if all route/data gates treat them as non-current and non-established.

No A record may appear as established B state.

## 11. Dependent source invalidation

A selection change must invalidate every source whose meaning is scoped by current Hearth.

At minimum today:

- caller role;
- members;
- bonds / kinship graph;
- Care schedules;
- Decisions;
- current votes;
- gratitude;
- rhythms;
- presence;
- Decision outcomes;
- immutable vote-history lookups/alignment;
- personal unvoted-Decision attention;
- personal Care-duty attention;
- Home calm-state derivation;
- any current-Hearth resource added later.

Each dependent provider may keep its own generation/token implementation, but its source key must include selected Hearth identity.

## 12. Cross-Hearth async isolation

The critical theorem is:

```text
request started for Hearth A
+ selection changes to Hearth B
+ A response arrives later
-> A response is stale and discarded
```

This applies even when:

- the conductor connection never changed;
- the same AgentPubKey remains connected;
- the A response is otherwise perfectly valid;
- B has not completed loading yet.

Selection identity is therefore an invalidation boundary equal in importance to reconnect/signer loss.

## 13. Mutation dispatch

Action forms must bind the selected Hearth they were built against.

Before dispatch, a form/action should establish that its target Hearth identity still equals the current selected Hearth when the action is intended to be current-Hearth scoped.

If selection changed while a form was open:

```text
old form -> no silent retarget
```

The browser should require the user to review/reopen/reconcile rather than rewriting A's request to B.

This is a client safety invariant only. Server-side authorization remains authoritative.

## 14. Post-dispatch selection changes

If a mutation for Hearth A was actually dispatched and the user switches to B before its result is known:

- do not report the eventual A result as a B mutation result;
- do not insert the returned A record into B's snapshot;
- preserve existing Unknown-outcome/reconciliation semantics for A;
- surface any result only in an A-scoped audit/reconciliation context if the product supports that safely.

Selection switching never proves a dispatched write did not occur.

## 15. Home and Inbox semantics

Home and Inbox are selected-Hearth projections.

A switch A -> B must make A-derived personal attention non-current immediately.

The browser must not show:

```text
B header
+ A Care attention
+ A Decision attention
```

as an established page state.

The same applies to the calm-state theorem: `a quiet moment` may only derive from established-empty attention sources keyed to the currently selected Hearth.

## 16. No cross-Hearth social inference

Multi-Hearth support must not produce comparative ranking such as:

- healthiest Hearth;
- strongest family;
- weakest relationships;
- best members;
- most harmonious household;

unless a future feature has a separately reviewed, user-chosen, evidence-appropriate purpose.

Selection is navigation, not social scoring.

## 17. Invitation and departure transitions

### Joining a new Hearth

After an invitation is accepted or a Hearth is founded, a fresh Active catalog is required before the browser treats the new Hearth as selectable Live state.

A returned membership/create record does not silently append a permanent browser catalog entry without reconciliation.

### Leaving the selected Hearth

After a successful departure transition for selected Hearth A:

```text
A cannot remain selected as Active
```

The browser must refresh/reconcile the Active catalog.

If other Active Hearths remain:

- exactly one -> may auto-select it after authoritative catalog reconciliation;
- multiple -> explicit choice/remembered-valid preference rules apply.

If none remain -> `NoActiveHearths`.

Historical discovery of A may remain available in a separate history/audit surface, but not the Active selector.

## 18. Duplicate and malformed catalog records

Every Active catalog entry must establish a valid typed Hearth ActionHash and decodable Hearth record.

Duplicate source identities or malformed records must not silently produce multiple selector choices.

Recommended rule:

```text
all source records valid + unique -> Live
zero source records -> Empty
any malformed/duplicate semantic ambiguity -> Degraded
```

Valid entries may be retained for diagnostics, but Degraded catalog state may not auto-select a live Hearth.

## 19. Privacy boundary

The selector should expose only Hearth information the Active catalog source returns to the connected member.

Do not fetch broad global Hearth directories merely to implement switching among the caller's own Hearths.

A remembered selection preference should use the minimum local information required, ideally exact Hearth ActionHash plus connection/agent namespace. Do not persist unrelated membership or household data in preference storage.

## 20. Accessibility / UX

When a choice is required, use an explicit labeled selection control or task surface.

The UI should communicate:

- which Hearth is currently selected;
- that changing Hearth changes the scope of Home/Inbox/domain views;
- loading/reconciliation state after a switch;
- source unavailable/degraded states without silently reverting to another Hearth.

Keyboard and assistive-technology users must be able to select and identify the current Hearth without relying on color or spatial location alone.

## 21. Required regression matrix

The implementation sequence must demonstrate at least:

### Active catalog / Kinship

1. founder-created Hearth appears Active;
2. accepted-invitation Hearth appears Active;
3. departed Hearth does not appear in Active catalog even if historical `AgentToHearths` link remains;
4. departure from A does not remove Active B;
5. stale historical Active membership cannot resurrect departed A.

### Pure selection state

6. zero Active Hearths -> `NoActiveHearths`;
7. one Active Hearth -> exact auto-selection;
8. two Active Hearths -> ChoiceRequired, never `.first()`;
9. valid remembered preference in fresh catalog -> selected;
10. remembered departed/malformed/non-catalog hash -> ignored;
11. presentation ordering does not affect selection outcome.

### Async isolation

12. A load arriving after A -> B switch is discarded;
13. A Decision/Care attention result arriving after switch is discarded;
14. selected B cannot render A payload as established;
15. old form built under A cannot silently dispatch against B.

### Reconciliation

16. leaving selected A refreshes catalog and clears A selection;
17. joining C requires fresh catalog establishment before C becomes Live-selectable;
18. source Unavailable/Degraded never falls back to stale remembered selection.

## 22. Intended implementation order

Do not implement this as one large PR.

Recommended exact sequence:

1. **Kinship Active catalog source** — repair/freeze current membership semantics independently of UI;
2. **pure selection state** — zero/one/many + remembered-preference validation, host-free tests;
3. **bind selection into HTH-UI-005 atomic snapshot key**;
4. **invalidate/re-key Decision outcome/history/Care/Decision attention providers**;
5. **selection UX** in task shell / Home scope disclosure;
6. **SweetConductor + browser qualification** for departure/join/switch races.

## 23. Non-claims

Explicit Hearth selection does not prove:

- mutation authorization;
- global DHT completeness;
- membership in historical Hearths;
- that one Hearth is more important than another;
- continuous real-time freshness;
- legal/household/family status outside the recorded Hearth membership model.

It proves a narrower browser theorem:

> the current Hearth shown by the product was selected from freshly established Active membership evidence, and all selected-Hearth projections are isolated to that exact scope.
