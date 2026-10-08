# Mycelix Leptos Frontend Platform v1

**Decision:** One shared frontend platform, four deployment surfaces, institution-specific profiles.

## Core decision

Do not create one frontend codebase for every institution.

Use one shared Leptos platform with:

- one design system;
- one typed domain/client SDK layer;
- one evidence/authority visualization language;
- one accessibility model;
- one security UX model;
- one test/qualification harness;
- one component library;
- institution-specific capability profiles and deployment configuration.

The institution type changes the enabled workflows and terminology, not the underlying UI architecture.

## Four deployable surfaces

### 1. Mycelix Member

Audience: individuals, households, communities, customers/members.

Primary workflows:

- identity and credentials;
- balances and obligations;
- payments;
- consent/privacy;
- community participation;
- personal evidence;
- disputes/support;
- personal Symthaea assistance.

Security posture: least privilege, personal-data minimization, hardware/passkey-friendly authentication, clear transaction confirmation, recoverable identity.

### 2. Mycelix Institution

Audience: banks, credit unions, payment institutions, fintechs, corporate treasury teams, cooperatives, custodians, asset managers and other licensed/regulated financial entities.

This is the main professional product.

Workspace modules:

- Executive / overview;
- Accounting / journal;
- Reconciliation;
- Treasury / liquidity;
- Payments;
- Credit;
- Collateral;
- Risk;
- Compliance / AML;
- Cases / exceptions;
- Settlement;
- Evidence / audit;
- Policy / controls;
- Administration.

Institution profiles:

| Profile | Primary configuration |
|---|---|
| Bank | deposits, lending, treasury, capital/liquidity, payments, AML |
| Credit union / cooperative | member finance, cooperative governance, pooled resources |
| Payment institution / fintech | payment orchestration, safeguarding, fraud, reconciliation |
| Corporate treasury | cash, liquidity, counterparties, FX, funding, obligations |
| Custodian / market infrastructure | assets, positions, settlement, corporate actions, reconciliations |
| Commons / cooperative finance | pooled funds, allocations, grants, stewardship, mutual credit |

These are capability profiles, not separate forks.

### 3. Mycelix Operations

Audience: settlement operators, bridge operators, infrastructure administrators, service reliability teams.

Primary workflows:

- rail health;
- dependency health;
- queued/indeterminate settlement;
- reconciliation breaks;
- message replay/duplicate detection;
- key and signer state;
- witness/quorum state;
- external-chain receipts;
- incident response;
- recovery;
- deployment/configuration provenance.

Security posture: strongly privileged, separate deployment and authorization boundary from ordinary institution users.

### 4. Mycelix Supervisor

Audience: internal auditors, external auditors, regulators, supervisors, authorized assurance parties.

Primary workflows:

- read-only financial reconstruction;
- evidence lineage;
- policy/version inspection;
- model/risk provenance;
- control testing;
- regulatory report packages;
- exception history;
- settlement finality evidence;
- audit queries;
- selective disclosure.

Security posture: independently scoped read-only/assurance authority, with explicit disclosure boundaries. It should not be an administrator console.

## Why not five or ten institution frontends?

A bank and a credit union have very different products, but their operators still need the same fundamental interaction primitives:

```text
find entity
inspect evidence
view position
approve decision
resolve exception
reconcile
review risk
authorize action
inspect receipt
trace lineage
```

Forking those primitives causes:

- duplicated security bugs;
- inconsistent permission UX;
- divergent accessibility;
- incompatible audit workflows;
- increased qualification burden;
- institutional lock-in to one UI implementation.

Instead use schema/profile-driven composition.

## Shared architecture

```text
                   Mycelix Design System
                           |
              +------------+------------+
              |            |            |
          Domain UI     Security UI   Evidence UI
              |            |            |
              +------------+------------+
                           |
                     Workspace Shell
                           |
          +----------------+----------------+
          |                |                |
       Member         Institution      Supervisor/Ops
                           |
                Institution Profile
            /      /       |       \       \
         bank  credit  payments  treasury  custody
```

Operations and Supervisor remain separately deployable because their trust boundaries differ, even though they reuse the same components.

## Leptos architecture

Use **Leptos 0.8.x stable** for the initial production foundation. As of the current 2026 release stream, 0.8.19 is the latest stable release while 0.9.0 is still a prerelease beta; production work should not make a beta the required baseline.

Prefer the official Axum full-stack template for new full-stack applications. Leptos currently recommends Axum for new projects and supports SSR, hydration, server functions and islands.

### Recommended rendering split

```text
public / marketing / documentation
    -> SSR

authenticated financial workspace shell
    -> SSR shell + client hydration

highly interactive dashboards/forms
    -> islands / targeted hydration

Holochain conductor interaction
    -> browser WASM client

secrets / privileged external APIs
    -> server-side functions/service layer
```

This uses Leptos where it is strongest while minimizing unnecessary browser-side code. Leptos islands can reduce the shipped WASM surface because only interactive islands are hydrated. Leptos server functions remain public APIs and therefore require normal authentication, authorization, input validation, rate limiting and disclosure controls.

## Holochain integration

Use the existing browser-native `mycelix-leptos-client` rather than reintroducing a JavaScript Holochain client dependency where it is not required.

Frontend adapters should expose typed domain methods such as:

```text
finance.get_position(...)
finance.get_risk_snapshot(...)
finance.create_reconciliation_case(...)
finance.submit_control_decision(...)
finance.request_settlement(...)
finance.get_settlement_receipt(...)
```

The UI should not know raw zome wire formats beyond the typed client boundary.

## Security UX rules

The frontend must visually distinguish:

- observed;
- verified;
- qualified;
- authorized;
- pending;
- included;
- finalized;
- reconciled;
- disputed;
- superseded;
- indeterminate.

Never show a green success state for a merely queued operation.

Examples:

```text
RPC success                != payment settled
transaction included      != economically final
oracle response            != qualified valuation
AI recommendation          != authorization
balance display             != accounting authority
```

The UI must expose the security state rather than hiding it behind generic status badges.

## Authority UX

Every material action should expose:

- acting identity;
- institution/legal entity;
- role;
- authority/capability;
- policy version;
- evidence references;
- affected amount/assets;
- expiration if any;
- expected external effects;
- irreversible boundary;
- confirmation state;
- resulting receipt/reference.

High-risk actions should use a two-step review pattern where the underlying policy requires dual control or approval.

## Evidence UX

Every important number should support a "Why this number?" path.

For a displayed exposure:

```text
$4.8M exposure
  -> source positions
  -> valuation observations
  -> collateral
  -> netting
  -> policy
  -> model version
  -> evidence frontier
  -> timestamp
```

An auditor should be able to navigate the same lineage without switching products.

## Role-based navigation

Do not create one giant navigation menu containing every possible financial function.

Navigation should be generated from:

```text
workspace profile
+ legal entity scope
+ user role
+ capabilities
+ jurisdiction
+ enabled modules
+ current risk/incident state
```

Example:

```text
Treasurer
  Dashboard | Liquidity | Funding | Collateral | FX | Settlement

Controller
  Dashboard | Journal | Reconciliation | Close | Evidence

CRO
  Dashboard | Exposure | Concentration | Stress | Models

MLRO
  Dashboard | Monitoring | Alerts | Cases | Reporting

Auditor
  Evidence | Reconstruction | Controls | Reports
```

Same application platform, radically different cognitive surface.

## Multi-tenant architecture

Institution deployments need strong tenant and legal-entity isolation.

Tenant identity must never be inferred from URL or frontend state alone.

Every privileged server action and Holochain query should resolve an explicit:

```text
principal
tenant
legal entity
workspace
capability
policy profile
```

and fail closed when any required binding is absent or ambiguous.

## White-labeling

Allow institution-specific:

- logo/identity;
- terminology;
- color tokens;
- domain;
- module visibility;
- jurisdiction policy labels;
- workflow defaults.

Do not allow white-label configuration to alter security semantics.

Brand theme != security policy.

## Accessibility

Accessibility should be centralized in the design system rather than reimplemented per institution.

Target WCAG 2.2 AA where practical, with keyboard-first operation, screen-reader semantics, reduced-motion support, high-contrast support, clear status text and non-color-only security indicators.

## Mobile strategy

Do not build a second mobile web codebase.

Responsive web should cover member and lightweight approvals.

Use installable/PWA or native wrapper only where offline, secure hardware integration, push notifications or constrained workflows justify it.

Professional treasury and operations should optimize for desktop/tablet, not pretend every workflow is phone-native.

## Design-system primitives

First shared components should be:

- `MoneyAmount`;
- `AssetAmount`;
- `EntityIdentity`;
- `EvidenceState`;
- `AuthorityBadge`;
- `PolicyVersion`;
- `RiskMetric`;
- `LiquidityMetric`;
- `SettlementStatus`;
- `ReconciliationCase`;
- `ApprovalPanel`;
- `DecisionReceipt`;
- `AuditTrail`;
- `LineageDrawer`;
- `ConflictBanner`;
- `IndeterminateState`;
- `FreshnessIndicator`;
- `ExternalRailBadge`.

These are not decoration. They are security semantics made visible.

## Testing strategy

One shared UI test corpus should exercise:

- unauthorized action hidden vs rejected;
- server-side authorization despite UI manipulation;
- stale data presentation;
- ambiguous/indeterminate state;
- duplicate submission;
- double-click/retry;
- policy version mismatch;
- tenant substitution;
- legal-entity substitution;
- wrong settlement rail;
- evidence unavailable;
- AI output presented as recommendation, never authority;
- audit reconstruction;
- accessibility;
- keyboard-only operation.

Use Playwright or the project's chosen browser harness for deterministic end-to-end workflows. The UI test is evidence of frontend behavior, not proof of backend economic correctness.

## Deployment topology

Recommended starting point:

```text
mycelix.app              -> Member
finance.mycelix.app      -> Institution
ops.mycelix.app          -> Operations
audit.mycelix.app        -> Supervisor/Audit
www.mycelix.app          -> public information/developer portal
```

These can share one source tree and one design system while being independently deployed and independently authorized.

Institutions can also receive dedicated deployment instances without creating a source fork:

```text
same binary/configurable application
        + institution profile
        + tenant policy
        + branding
        + enabled modules
        + deployment isolation
```

## What we should build first

### Frontend foundation

- shared design tokens;
- typed security/evidence components;
- auth/session boundary;
- workspace/tenant resolver;
- navigation capability resolver;
- Holochain client provider;
- global event/status stream;
- standardized loading/error/indeterminate states.

### First professional workspace

Start with the institution console and build three workflows deeply:

1. Reconciliation;
2. Treasury/liquidity;
3. Settlement/evidence.

These create a coherent triangle between daily operations, financial control and the new Mycelix economic substrate.

Then add accounting, risk, compliance, credit and tokenisation.

## Why this can beat separate institutional products

One shared platform lets improvements propagate:

```text
security fix
 -> every institution

accessibility improvement
 -> every institution

new settlement rail
 -> every institution

new evidence visualization
 -> every institution

new audit capability
 -> every institution
```

while the institution sees only what its policy/profile permits.

## Final recommendation

Build **one Mycelix Leptos frontend platform**.

Ship **four deployment surfaces**:

1. Member;
2. Institution;
3. Operations;
4. Supervisor/Audit.

Within Institution, support multiple professional profiles rather than separate products.

Keep the public/developer website as a separate low-trust surface if desired; it does not need access to the financial workspace.

Do not fork the frontend for banks, credit unions, fintechs, treasuries, custodians, etc. Fork only when the **security boundary, deployment boundary, or fundamentally different workflow model** requires it.

## Nonclaims

This architecture does not establish frontend security merely by using Leptos, SSR, WASM or a typed client. Browser code is untrusted. All material authorization, tenant isolation, policy enforcement and economic validation must remain enforced server-side/Holochain-side.

Leptos server functions are public APIs and must be treated as such.

## References

- Leptos Getting Started: https://book.leptos.dev/getting_started/index.html
- Leptos SSR: https://book.leptos.dev/ssr/index.html
- Leptos Islands: https://book.leptos.dev/islands.html
- Leptos Server Functions: https://book.leptos.dev/server/25_server_functions.html
- Leptos Axum starter: https://github.com/leptos-rs/start-axum
- Leptos releases: https://github.com/leptos-rs/leptos/releases
- mycelix-leptos-client: https://github.com/Luminous-Dynamics/mycelix-leptos-client