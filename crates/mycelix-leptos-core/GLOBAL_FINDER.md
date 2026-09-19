# Mycelix Global Finder Architecture v1

Status: **design contract**

Scope: cross-domain discovery, navigation, and command initiation for Mycelix frontends.

## Goal

Give ordinary users one fast place to find people, spaces, conversations, documents, projects, learning items, applications, and actions without requiring them to understand the internal Mycelix domain/cluster topology.

The Finder is a presentation/orchestration surface. It is **not** a new authority layer, global database, capability engine, or centralized index of all Mycelix activity.

## Governing rule

> **The shell owns the interaction; domains own the meaning, visibility, and authority of their results.**

A Finder result may help a user discover or initiate something. Its presence MUST NOT imply that the user is authorized to perform the resulting action.

## User interaction

The Finder SHOULD be reachable through:

- a visible search/find affordance;
- `Ctrl+K` / `Cmd+K` as an accelerator;
- a task-oriented Home surface where appropriate.

The keyboard accelerator MUST NOT be the only way to discover the feature.

The initial interaction model SHOULD follow the WAI editable combobox + listbox pattern rather than inventing custom keyboard behavior.

Expected core behavior:

- text input retains DOM focus;
- `Down Arrow` / `Up Arrow` move the active suggestion;
- `Enter` activates the selected result;
- `Escape` closes the result popup without executing it;
- standard platform text editing keys continue to work;
- result focus/selection remains visually and semantically inspectable;
- the result popup is named and related to the input with the appropriate ARIA relationship.

## Result kinds

V1 SHOULD support a closed shared presentation taxonomy while allowing domains to provide their own records:

- `Person`
- `Space`
- `Conversation`
- `Document`
- `Project`
- `Learning`
- `App`
- `Action`

A domain MAY add display metadata, but it SHOULD map to one of the shared presentation kinds before entering the common Finder surface.

This taxonomy is for presentation. It does not redefine domain data models.

## Result contract

A shared Finder result SHOULD contain only presentation-safe data such as:

- stable result ID within its provider;
- result kind;
- title;
- optional subtitle/context;
- optional icon/glyph;
- provider/domain identity;
- optional destination/navigation intent;
- optional typed proposed action reference;
- search keywords or locally computed match metadata;
- optional availability/freshness disclosure supplied by the provider.

The shared Finder MUST NOT synthesize:

- identity verification;
- authorization;
- trust tier;
- evidence validity;
- action success;
- federation/confirmation/settlement state.

## Navigation vs action

Navigation and consequential actions are separate result behaviors.

### Navigation result

A navigation result may open a person, space, document, conversation, project, learning item, or app.

### Action result

A consequential action MUST follow the existing truth/authority sequence:

`query -> matching result -> typed proposed action -> preview -> user authorization -> current capability/authority check -> execution`

The Finder MUST NOT convert an action result directly into execution merely because the user pressed Enter.

## Provider model

Domains SHOULD register Finder providers or catalogs with the shell rather than the shell importing domain internals.

A provider owns:

- what records are discoverable;
- how records are matched/ranked inside that provider;
- what presentation-safe fields leave the domain boundary;
- what navigation destination or typed proposed action is attached;
- whether local/offline results are available;
- whether remote discovery is supported;
- privacy and authorization checks required before returning a result.

The shell owns:

- query input;
- result aggregation;
- grouping/presentation;
- keyboard interaction;
- recent-result presentation where permitted;
- opening navigation results;
- handing proposed actions to the normal preview/authorization flow.

## Local-first discovery

The Finder SHOULD prefer useful local results immediately when the required local indexes are available.

Remote discovery MAY enrich the result set, but the UI MUST preserve the difference between:

- locally available results;
- remote discovery still in progress;
- remote discovery unavailable;
- remote discovery failed/rejected;
- unknown remote state.

Loss of connectivity MUST NOT erase locally discoverable results.

`No remote results != no results exist`.

## Privacy

Global findability creates a privacy risk if implemented as indiscriminate query fan-out.

V1 MUST therefore follow these rules:

1. Do not broadcast every keystroke to every domain/network peer.
2. Prefer local indexes and explicitly authorized providers first.
3. A provider receives only the query/context necessary for its declared purpose.
4. Providers MUST NOT return records the caller is not permitted to discover.
5. Recent searches/results MUST NOT be persisted by default when the query may contain sensitive domain information unless an explicit product contract permits it.
6. Search telemetry, if introduced, MUST be separately consented/defined and is outside this architecture contract.

## Ranking

The shell MAY merge provider rankings, but ranking MUST NOT silently become a trust or authority score.

Recommended V1 inputs include:

- textual relevance;
- exact/prefix match;
- explicit user recency/frequency where locally available and privacy-appropriate;
- provider-supplied relevance;
- current-context affinity.

Prohibited ranking inputs unless separately justified and disclosed:

- hidden political/social desirability;
- inferred user worth;
- trust tier as a generic relevance multiplier;
- capability/authority as a proxy for semantic relevance.

If personalization is used, its source should be inspectable and disable-able where practical.

## Empty, loading, and unavailable states

The Finder MUST distinguish at least:

- no query yet;
- searching local sources;
- local results available;
- remote enrichment in progress;
- no matches found in the sources actually searched;
- one or more providers unavailable;
- result state unknown.

An unavailable provider MUST NOT be silently collapsed into an empty result set.

## App discovery and information architecture

The current app/cluster topology may remain discoverable through the Finder, but it SHOULD NOT define the primary navigation mental model for ordinary users.

Recommended stable top-level product concepts are task-oriented:

- Home
- Find
- Create
- Inbox / Activity
- Me

Domain apps become destinations/results/capabilities behind those tasks rather than a mandatory first decision.

This is a product IA recommendation, not a mandate to merge domain runtimes or authority models.

## Proposed PR sequence

### UX-FIND-002 — typed finder registry

Add shared Rust types for result kind, presentation result, provider identity, navigation intent, and typed proposed-action handoff. No search UI yet.

### UX-FIND-003 — accessible finder component

Implement the editable combobox/listbox interaction, including active-result semantics, Escape behavior, pointer/touch selection, visible focus, and narrow-screen layout.

### UX-FIND-004 — local catalog adapter

Allow domains/apps to register locally searchable entries without giving the shell direct ownership of domain state.

### UX-FIND-005 — command/action handoff

Connect `Action` results only to the typed preview/authorization pipeline. No direct execution from the finder.

### UX-FIND-006 — async remote enrichment

Introduce provider-scoped remote discovery with explicit loading/unavailable/unknown states and privacy-preserving query boundaries.

### UX-FIND-007 — browser qualification

Playwright/axe + keyboard + touch/mobile tests for open/search/navigate/escape/action-preview behavior and truth-preserving failure states.

## Qualification scenarios

At minimum:

1. open Finder with pointer;
2. open Finder with keyboard accelerator;
3. find and navigate to a known local document;
4. find a person/space with keyboard only;
5. close with Escape without activating a result;
6. search while offline and retain local results;
7. expose remote-provider unavailability without claiming no result exists;
8. select an Action result and land on preview rather than execution;
9. verify mobile/touch use without relying on hover;
10. verify assistive technology can recover the result kind, title, active result, and popup state.

## Non-goals for v1

- natural-language autonomous execution;
- centralized indexing of all private Mycelix data;
- semantic/vector search as a requirement;
- cross-domain authority inference;
- hidden behavioral ranking;
- replacing domain-native detailed search/filter screens.

The Finder is a universal front door, not the sole way to explore a domain.
