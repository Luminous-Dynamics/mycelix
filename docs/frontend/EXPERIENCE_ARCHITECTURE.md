# Mycelix Experience Architecture v1

## Status

This document defines the first shared frontend architecture contract for Mycelix. It is intentionally product- and invariant-first: domain applications remain independently evolvable, but common human-facing concerns must converge on shared semantics and shared infrastructure.

## Product objective

Mycelix should feel like one coherent computing environment rather than a collection of unrelated hApps. Security, decentralization, provenance, and sovereignty should primarily surface as confidence, resilience, explainability, and control—not additional cognitive burden.

The governing product rule is:

> Mycelix must expose greater sovereignty with lower cognitive burden.

Derived rules:

1. Security should mostly appear as confidence.
2. Decentralization should mostly appear as resilience.
3. Provenance should mostly appear as explainability.
4. Sovereignty should mostly appear as control.

## Architectural boundary

The canonical frontend boundary is the shared Mycelix experience layer around domain-specific routes and workflows.

```text
Mycelix Experience
├── shared identity
├── shared navigation
├── shared search
├── shared activity
├── shared capabilities
├── shared provenance
├── shared connectivity/freshness
├── shared accessibility
└── shared error/degraded-state behavior
        ↓
   domain surfaces
```

A domain owns its business semantics, domain routes, domain actions, and domain-specific views. A domain should not independently redefine global identity, navigation, connectivity, trust semantics, evidence semantics, or generic interaction primitives.

## Canonical layers

`mycelix-leptos-core` is the natural shared foundation and should evolve around the following conceptual boundaries before being split into additional crates:

```text
foundation/
  accessibility
  responsive
  focus
  keyboard
  motion

design/
  tokens
  typography
  surfaces
  forms
  primitives

shell/
  navigation
  launcher
  command_palette
  search
  activity

runtime/
  connection
  availability
  freshness
  offline
  error_boundary

trust/
  identity
  capabilities
  consent
  provenance
  evidence

experiential/
  consciousness
  homeostasis
  thermodynamic
  flow
```

The boundaries above are conceptual for v1. Physical module or crate moves should occur only when they reduce coupling rather than simply making the tree look tidy.

## Canonical application composition

Domain applications should converge on a shared root composition instead of manually assembling common providers and shell behavior.

Target shape:

```rust
#[component]
pub fn App() -> impl IntoView {
    view! {
        <MycelixApplication manifest=GovernanceManifest::default()>
            <GovernanceRoutes />
        </MycelixApplication>
    }
}
```

`MycelixApplication` should own only cross-domain concerns. Domain-specific providers remain outside the shared boundary unless more than one domain genuinely shares the same semantics.

## Application manifest

Each domain should eventually describe its integration into the experience layer declaratively.

Proposed contract:

```rust
pub struct MycelixAppManifest {
    pub id: AppId,
    pub name: &'static str,
    pub icon: IconId,
    pub routes: Vec<RouteDescriptor>,
    pub capabilities: Vec<CapabilityDescriptor>,
    pub search_providers: Vec<SearchProvider>,
    pub activity_providers: Vec<ActivityProvider>,
    pub offline_policy: OfflinePolicy,
    pub notification_topics: Vec<NotificationTopic>,
}
```

The manifest must describe integration. It must not become a second source of truth for domain authorization or business state.

## Interaction model

Mycelix should be task-oriented at the experience layer and domain-oriented underneath it. Users should be able to think in terms such as:

- pay Alice
- message Jordan
- vote on the park proposal
- scan a product
- revoke a device

rather than needing to know which domain owns each action.

Global search and a command palette should therefore operate over typed domain providers. Natural-language interpretation may eventually propose typed actions, but must never bypass preview, authorization, capability checks, or domain execution rules.

```text
intent
  ↓
typed proposed action
  ↓
preview
  ↓
user authorization
  ↓
capability check
  ↓
domain execution
```

## Universal entity presentation

The experience layer should provide consistent presentation semantics for common entity classes:

- person
- organization
- community
- proposal
- transaction
- conversation
- asset
- product
- place
- credential
- evidence

A person or organization should remain recognizably the same entity when encountered in different domains. Domain context may add information but should not redefine identity semantics.

## Trust and provenance UX

Cryptographic and authority information should use progressive disclosure.

Default presentation answers human questions:

- Who is this?
- Are they verified?
- What are they allowed to do?
- Who granted that authority?
- How fresh is this information?
- Why did this action happen?

Expert surfaces may expose the underlying identifiers, signatures, capability chains, hashes, and evidence.

Every consequential state should support a provenance path when evidence exists, for example:

```text
Treasury payment
  ↓
Proposal
  ↓
Vote outcome
  ↓
Authorization
  ↓
Execution
  ↓
Settlement evidence
```

## Calm-state design

Mycelix has many potentially useful runtime signals. Healthy operation should compress them instead of turning the product into an operations dashboard.

Examples:

- Healthy: `Everything is current`
- Offline but safe: `Saved locally — will share when connected`
- Stale: `Some information may be outdated`

Detailed topology and diagnostic state belong behind progressive disclosure.

## Offline and freshness semantics

The frontend must distinguish at least the following concepts even when the ordinary UI compresses them:

- live
- synced
- local
- queued
- stale
- unavailable
- conflicted

`connected`, `healthy`, `current`, `replicated`, `authorized`, and `settled` are not synonyms and must not be rendered as though they are.

## Accessibility

Accessibility is a release property, not a visual-polish task. Core frontend paths should support:

- keyboard-only operation
- visible focus
- screen-reader semantics
- reduced motion
- zoomed layouts
- sufficient touch targets
- sufficient contrast
- semantic status announcements
- field/error association
- modal focus containment

No critical state may rely on color alone.

## Migration strategy

The migration must be incremental. Existing domain behavior should remain usable while shared infrastructure is extracted.

Recommended sequence:

1. Freeze frontend invariants and inventory existing applications.
2. Introduce a canonical `MycelixApplication` composition boundary.
3. Introduce semantic design tokens and shared primitives.
4. Implement the shared responsive shell.
5. Add canonical trust and provenance surfaces.
6. Unify cross-domain activity.
7. Add global search and typed command dispatch.
8. Fully migrate Governance as the reference application.
9. Migrate Identity/Personal.
10. Migrate Commerce and Pulse as high-pressure consumer experiences.

## Non-goals for v1

This architecture does not require:

- rewriting all existing frontends
- merging all domain state into one store
- replacing domain routing with a monolithic router immediately
- exposing every protocol detail in the default UI
- making AI an authority boundary
- making visual redesign a prerequisite for architectural convergence

The v1 goal is simpler: establish shared semantics, shared composition, and truth-preserving presentation boundaries so future frontend work compounds instead of fragments.
