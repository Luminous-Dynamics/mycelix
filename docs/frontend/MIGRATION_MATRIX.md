# Mycelix Frontend Migration Matrix v1

## Purpose

This matrix turns the Mycelix Experience Architecture into an incremental migration program. It is intentionally organized around risk reduction: shared semantics and composition come before broad visual redesign.

## Migration principles

1. Preserve domain behavior while extracting shared infrastructure.
2. Do not rewrite working domain logic merely to make frontend structure uniform.
3. Move only genuinely cross-domain concerns into `mycelix-leptos-core`.
4. Prefer typed state and explicit contracts over CSS-only convergence.
5. Use Governance as the first reference migration because it already exercises routing, finance integration, governance state, shared providers, accessibility primitives, and substantial application breadth.
6. Treat Commerce and Pulse as later pressure tests for consumer simplicity, mobile use, real-time behavior, notifications, and offline-first interaction.

## Tranches

| Tranche | Scope | Primary output | Exit gate |
| --- | --- | --- | --- |
| PR0 | Architecture + invariants | Experience contract, truth invariants, migration plan | No product behavior changes |
| PR1 | Shared composition | Canonical `MycelixApplication`/shared-provider boundary | Governance behavior preserved |
| PR2 | Design foundation | Semantic tokens + primitives | No domain-specific semantics in primitives |
| PR3 | Shared shell | Responsive navigation, account, connectivity, notifications | Desktop + mobile + keyboard paths pass |
| PR4 | Trust UX | Identity, verification, capability, evidence, provenance components | No truth strengthening under stale/unknown state |
| PR5 | Activity | Typed cross-domain activity feed | Source attribution + freshness preserved |
| PR6 | Search/commands | Global search and typed command registry | Commands cannot bypass domain authorization |
| PR7 | Governance | Full reference migration | Shared shell replaces domain shell duplication |
| PR8 | Identity/Personal | Canonical person/device/account experience | Same identity semantics across domains |
| PR9 | Commerce | Scan/browse/buy/pay/receipt/return flows | Consumer tasks work without protocol knowledge |
| PR10 | Pulse | Messaging, notifications, offline/realtime stress test | Real-time degradation remains legible and safe |

## Inventory dimensions

Each Leptos application should be inventoried against these dimensions before migration:

| Dimension | Questions |
| --- | --- |
| Root composition | Which common providers are initialized locally? |
| Navigation | Does the app own global or domain-only navigation? |
| Identity | How are person/account/device states presented? |
| Connectivity | Are connection, health, freshness, and offline states distinguished? |
| Actions | How are authorization, pending, execution, and failure represented? |
| Activity | Does the app expose events that belong in cross-domain activity? |
| Search | Which entities/actions should be globally discoverable? |
| Notifications | Which events require attention versus passive history? |
| Evidence | Which consequential states can expose provenance or evidence? |
| Styling | Which tokens/primitives are reusable versus domain-specific? |
| Accessibility | Keyboard, focus, semantic labels, status announcements, reduced motion |
| Responsive | Desktop, narrow desktop/tablet, phone |
| Testing | Unit, component, E2E, accessibility, visual, degraded-state coverage |

## Initial application priorities

### Governance

Reference consumer for PR1–PR7.

Why first:

- substantial route surface;
- shared governance + finance context;
- existing Holochain root composition;
- existing accessibility semantics;
- consequential authority and execution states;
- good test bed for provenance and capability UX.

Migration goals:

- remove duplicated global shell concerns;
- preserve all current routes and domain contexts;
- adopt canonical runtime status semantics;
- adopt shared trust/evidence surfaces;
- register globally discoverable governance entities/actions.

### Identity / Personal

Second reference family.

Migration goals:

- canonical person/account/device components;
- device and credential revocation UX;
- shared verification semantics;
- universal entity presentation contract.

### Commerce / Marketplace

Consumer simplicity pressure test.

Migration goals:

- task-first navigation;
- scan/browse/buy/pay/receipt/return journeys;
- product provenance without cryptographic jargon;
- clear payment authorization/settlement distinction;
- mobile camera/barcode-ready shell integration.

### Pulse / communications

Realtime/offline pressure test.

Migration goals:

- notifications;
- unread/activity integration;
- connection and freshness semantics;
- local-save/queued/replicated distinctions;
- consistent identity and conversation entities;
- mobile-first interaction quality.

## Shared-component extraction rule

A component should move into shared infrastructure only when at least one of these is true:

1. multiple domains already implement the same semantic concept;
2. consistency is security- or truth-relevant;
3. accessibility or responsive behavior should be centralized;
4. the component expresses a Mycelix-wide entity or runtime state;
5. future domains would otherwise be forced to reinvent a global interaction.

A component should remain domain-local when its meaning depends on domain rules that other domains do not share.

## Compatibility strategy

During migration, shared components should support coexistence with existing domain components. Avoid flag-day changes across all applications.

Recommended pattern:

```text
existing domain app
  ↓
adopts shared root/runtime
  ↓
adopts shared primitives
  ↓
adopts shared shell
  ↓
adopts trust/activity/search integration
  ↓
removes obsolete local duplicates
```

Each step should be independently reviewable.

## Evidence expected per PR

Every migration PR should include, where applicable:

- exact base commit;
- files changed;
- build/test commands;
- behavior-preservation notes;
- screenshots or visual diffs for visible changes;
- accessibility evidence for changed interaction paths;
- degraded/offline-state evidence for runtime-state changes;
- explicit statement of any remaining duplicate implementation.

## Stop conditions

Pause a migration tranche instead of forcing convergence if:

- shared extraction weakens a domain's authority boundary;
- a common abstraction requires domain-specific exceptions to function;
- offline behavior becomes less explicit;
- a visual simplification strengthens truth state;
- migration requires unrelated backend redesign;
- qualification cannot distinguish regressions from pre-existing failures.

The objective is compounding frontend quality, not uniformity for its own sake.
