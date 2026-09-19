# Mycelix Task-First Shell Architecture v1

Status: **design contract**

Scope: shared top-level information architecture and shell behavior for Mycelix frontends.

## Goal

Make Mycelix feel like one coherent environment organized around what people are trying to accomplish, without erasing domain boundaries, authority boundaries, or provenance.

The shell should help a person answer:

- What needs my attention?
- What was I doing?
- What can I find?
- What can I create or start?
- What belongs to me / my current identity?

It should not require the person to first understand which internal protocol, domain crate, cluster, or subsystem owns the task.

## Governing rule

> **Top-level navigation expresses user tasks; domain topology remains inspectable but secondary.**

Navigation is not a sitemap. The existence of many Mycelix domains is not a reason to expose all of them as peers in primary navigation.

## Recommended stable shell

V1 SHOULD converge toward five stable product concepts:

1. **Home** — current context, recent work, and what needs attention.
2. **Find** — cross-domain Finder/search entry point.
3. **Create** — safe initiation of common creation/proposed-action flows.
4. **Inbox / Activity** — items that need review, response, acknowledgment, or follow-up.
5. **Me** — identity, devices, recovery, capabilities/permissions disclosure, preferences, and personal state.

The exact labels MAY be refined through usability research, but the conceptual split should remain task-oriented.

## Home

Home is not a dashboard containing one widget from every domain.

It SHOULD prioritize:

- resume/recent work;
- explicit attention items;
- locally pending or unsynchronized work;
- user-chosen pinned spaces/projects;
- a concise current-state summary;
- contextual next actions.

Home MUST NOT manufacture urgency by ranking every domain event as attention-worthy.

Home cards SHOULD link back to the owning domain/context and preserve provider provenance.

## Find

Find follows `GLOBAL_FINDER.md`.

The shell owns the entry point and interaction. Domains own discoverability, result meaning, and authority.

Primary navigation SHOULD NOT duplicate the Finder by exposing every searchable domain destination.

## Create

Create is a launcher for typed initiation, not a universal execution button.

Examples may include:

- message / conversation;
- document / note;
- project / space;
- learning item;
- proposal;
- listing / offer;
- request for help;
- other domain-specific creation flows.

The shell MAY present these as user-language actions, but each option must hand off to the owning domain workflow.

Consequential actions MUST preserve:

`intent -> typed proposed action -> preview -> user authorization -> current authority/capability check -> execution`

The shell must never infer authority from the fact that an item appears in Create.

## Inbox / Activity

Inbox is a unified presentation surface, not a new central event authority.

Items SHOULD carry:

- provider/domain provenance;
- stable item identity;
- plain-language title/summary;
- typed reason for attention;
- lifecycle state supplied by the provider;
- freshness when relevant;
- destination or typed proposed action;
- optional local/sync disclosure when relevant.

The shell MUST preserve provider distinctions such as:

- informational update;
- action requested;
- approval needed;
- conflict requires resolution;
- synchronization issue;
- security/authority change;
- completed item.

An unread item is not automatically urgent. An attention item is not automatically authorized.

## Me

Me SHOULD consolidate user-centric concerns that otherwise become scattered across apps:

- current identity and profile;
- device/recovery status;
- local data/recovery controls;
- preferences and accessibility settings;
- connected/runtime state disclosures;
- capability/authorization inspection where the domain model supports it;
- sign-out / identity switch where applicable.

Trust tier, identity verification, and authority MUST remain separate concepts.

## App/domain switcher

The app/domain switcher remains useful for expert navigation, development, and direct domain exploration.

It SHOULD be secondary to the task-first shell and available through one or more of:

- Finder results;
- an explicit “Apps” disclosure;
- expert/developer surfaces;
- domain-specific deep links.

It SHOULD NOT occupy the user's primary decision point on every page.

## Domain pages

Once a person enters a domain workflow, local navigation MAY expose domain-specific sections when repeated multi-task navigation is genuinely useful.

If a workflow has a clear sequence, prefer a task/step model over adding more persistent navigation.

Domain navigation MUST NOT be promoted into global shell navigation solely because the route exists.

## Responsive model

### Narrow/mobile

V1 SHOULD target a compact persistent bottom or equivalent primary navigation for the most frequent shell tasks, with overflow for less frequent items if needed.

Touch targets, labels, and state must not rely on hover.

### Desktop

Desktop MAY use a header, rail, or hybrid shell, but the conceptual destinations should match mobile rather than becoming a different information architecture.

Keyboard accelerators MAY complement visible navigation but not replace it.

## Context preservation

Moving between shell surfaces and domains SHOULD preserve enough context to avoid needless restarts.

Examples:

- returning from a detail page to the Finder should restore the prior query/results when safe;
- returning from a domain task to Inbox should preserve the reviewed item position;
- Create cancellation should return to the invoking context;
- offline work should not disappear merely because the user changes shell section.

Context preservation MUST NOT retain revoked authority or stale actionability.

## Protocol-language policy

Primary shell copy SHOULD prefer ordinary user language.

Protocol terms such as zome, conductor, DHT, capability grant, provenance record, or federation receipt should appear when:

- the user explicitly opens technical detail;
- the term is required for informed consent/authority understanding;
- the user is in an expert/developer surface.

Simplifying language MUST NOT strengthen truth. For example, “Saved” must not be used if the established state is only “queued to save.”

## Progressive disclosure

Every high-level summary SHOULD support deeper inspection when the underlying system exposes meaningful evidence or provenance.

Recommended layers:

1. plain-language state;
2. concise detail / why this matters;
3. provenance/evidence/authority inspection;
4. technical identifiers and protocol details.

Experts can drill down without forcing every user to start at layer 4.

## Proposed PR sequence

### UX-IA-002 — shared shell slots

Evolve `AppShell` to accept typed/explicit slots for Home/Find/Create/Inbox/Me affordances without importing domain state.

### UX-HOME-001 — attention/recent-work presentation contract

Define typed, provider-owned Home items and attention semantics. No centralized ranking authority.

### UX-CREATE-001 — typed creation launcher

Define presentation-safe Create entries whose consequential actions hand off to typed preview/authorization flows.

### UX-INBOX-001 — unified attention envelope

Define provider-owned inbox/activity entries with explicit lifecycle, freshness, provenance, and destination/action references.

### UX-IA-003 — secondary Apps surface

Move the current broad domain/app list behind an explicit secondary disclosure/Finder route while retaining direct deep-link access.

### UX-IA-004 — first domain migration

Migrate one representative domain to the task-first shell as a qualification pilot without changing its domain authority or route semantics.

### UX-IA-005 — responsive + keyboard qualification

Qualify the shell across phone/desktop, keyboard/touch, zoom/reflow, and local/offline states.

## Qualification questions

For each shell migration:

1. Can a novice reach the intended task without knowing the owning domain name?
2. Can an expert still directly inspect/navigate the domain?
3. Is the primary nav materially smaller than the domain/app topology?
4. Are current lifecycle, sync, freshness, and authority states preserved?
5. Does keyboard/touch/mobile use expose the same conceptual shell?
6. Can the user return to the invoking context without losing eligible local work?
7. Has any shell summary strengthened provider truth?

## Research rationale

Large multi-task systems benefit from persistent navigation only when users repeatedly move among genuinely useful top-level tasks. Navigation should be simplified before adding more links, and clear sequential journeys should use task/step structures instead of becoming permanent navigation sections.

The Mycelix shell should therefore represent a small stable set of user tasks while allowing domains to retain rich internal workflows behind those tasks.
