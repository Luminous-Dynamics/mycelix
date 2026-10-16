# Regenerative Market Substrate — Mycelix Architecture

## Thesis

Markets are powerful coordination mechanisms because they combine incentives, decentralized choice, experimentation, and trust.

Their failure mode is not simply greed.

The deeper failure is:

> private optimization can consume the physical, social, epistemic, or institutional stocks that make future coordination possible.

Mycelix can address this without replacing markets by adding a substrate layer underneath them:

```
market activity
    ↓
attribution
    ↓
substrate accounting
    ↓
boundary check
    ↓
reciprocity / restoration
    ↓
continued productive activity
```

The objective is not to compute one universal score for society.

The objective is to preserve the conditions under which decentralized agency can continue.

## The six-layer protocol

### 1. Incentive layer — Markets

Profit, reward, curiosity, competition, cooperation, and local choice remain sources of economic energy.

Mycelix Finance already supplies SAP, TEND, MYCEL, demurrage, commons reserves, and counter-cyclical liquidity mechanisms.

The protocol does not attempt to abolish incentive.

It attempts to constrain destructive incentive spillovers.

### 2. Identity and trust layer

Every consequential action needs an attributable actor.

Identity, credentials, reputation, and Sybil resistance determine who is participating and under what authority.

Existing Mycelix identity and governance mechanisms provide this foundation.

Trust remains contextual rather than becoming a single claim about a person's entire character.

### 3. Epistemic layer

Economic decisions require evidence.

The DKG, provenance, attestation, contradiction handling, and epistemic classification systems provide a way to distinguish:

- measured observations;
- contested observations;
- normative choices;
- uncertain claims;
- stale information.

A measurement should never become stronger merely because it is convenient.

### 4. Substrate layer — AC-017

AC-017 introduces dimension-specific substrate accounts for:

- financial;
- physical;
- ecological;
- social;
- epistemic;
- institutional stocks.

Each account has a native unit, baseline, explicit boundary, warning band, and hard/soft classification.

The key invariant is:

> a healthy dimension cannot compensate for a breached hard boundary elsewhere.

This is deliberately stronger than a weighted sustainability score.

A river system in collapse cannot be made healthy by better quarterly earnings.

### 5. Reciprocity layer — AC-018

AC-018 carries an observed impact from:

action → affected substrate → attribution → obligation → restoration.

Impacts remain in native units.

Attribution can be:

- direct;
- contractual;
- contributory;
- shared-chain;
- unresolved.

An unresolved impact is not treated as zero.

A depletion impact that becomes fully attributed opens a restoration obligation. Monetary payment is intentionally separate from the empirical obligation.

This mirrors an important distinction in environmental policy: price signals can correct market failure, but price, causality, measurement, and remediation are not the same thing.

### 6. Constitutional boundary layer — AC-019

AC-019 protects the boundary itself.

A system that can silently redefine "safe" will eventually redefine failure away.

Boundary revisions therefore carry:

- scope;
- subject;
- native unit;
- predecessor;
- authority;
- independent authority;
- evidence;
- rationale;
- proposal/effective timestamps.

Tightening a boundary can be straightforward.

Relaxing one requires independent authority and a cooling period under the reference policy.

No revision can rewrite history retroactively.

## Numerical integrity — AC-020

AC-020 hardens the existing commons reserve mechanism.

The constitutional 25% reserve split is now represented through exact integer arithmetic rather than floating-point multiplication.

Reserve validation retains the existing 0.1 percentage-point tolerance but expresses it as an exact rational inequality.

This is small but important.

A constitutional invariant should not depend on representation artifacts.

## Governance integrity — AC-001 through AC-016

Substrate preservation cannot work if the measurement and governance system is capturable.

The existing anti-capture series addresses:

- identity ambiguity;
- provenance;
- evidence coverage;
- procurement concentration;
- robustness across analytical specifications;
- adversarial composition;
- authority concentration;
- constitutional constraints.

This creates a complementary relationship:

```
AC-001…016  →  prevent capture of the decision system
AC-017       →  prevent invisible substrate depletion
AC-018       →  prevent invisible externalization
AC-019       →  prevent capture of the boundary
AC-020       →  prevent arithmetic corruption
```

## The governing conservation laws

The architecture can be summarized by eight invariants.

### 1. Non-compensation

A protected hard boundary cannot be offset by performance elsewhere.

### 2. Non-omission

Missing required evidence is not equivalent to a healthy state.

### 3. Non-retroactivity

Current policy cannot rewrite the historical interpretation of past activity.

### 4. Non-reflexivity

Reputation, authority, resource allocation, and metric creation must not form an unchecked positive feedback loop.

### 5. Repairability

Maintenance and restoration remain possible during breach.

Protecting a system must not prevent it from repairing itself.

### 6. Evidence preservation

A bad outcome remains observable.

A failed claim, breach, dispute, or challenge does not disappear because it is inconvenient.

### 7. Separation of concerns

Measurement, attribution, obligation, payment, and governance authority are distinct layers.

A monetary transaction cannot by itself prove ecological restoration.

### 8. Local legitimacy

Boundaries and obligations remain locally governed and context-sensitive.

The protocol supplies evidence and enforcement primitives; it does not become a universal planner.

## Why this is compatible with capitalism

The proposal is not:

> replace prices with moral scores.

It is:

> keep prices, competition, entrepreneurial freedom, and decentralized coordination, while making previously invisible dependencies and externalized costs legible.

That creates a different optimization surface.

A firm may still seek profit.

It simply cannot assume that depletion of a protected substrate is equivalent to free surplus.

The desired condition is:

[
	ext{private gain} subseteq 	ext{safe operating envelope}
]

rather than:

[
	ext{private gain} + 	ext{externalized damage} = 	ext{reported profit}
]

## Relationship to established accounting

This architecture does not require Mycelix to invent a competing environmental-accounting standard.

The UN System of Environmental-Economic Accounting already provides a framework for tracking ecosystem extent, condition, ecosystem-service flows, and changes in ecosystem assets. It can operate at national, subnational, river-basin, urban, and other spatial scales, and physical accounts do not require monetary valuation.

South Africa already maintains SEEA-aligned ecosystem/resource accounts and has a National Natural Capital Accounting Strategy. Its work includes strategic water-source areas, sub-national water-resource accounts, physical energy flows, protected areas, biodiversity, and other ecosystem accounts.

This creates a practical division of labor:

```
SEEA / national statistics
        ↓
validated observations
        ↓
Mycelix provenance + attribution
        ↓
local governance + contractual obligations
        ↓
economic action / procurement / treasury decisions
```

Mycelix therefore becomes an operational decision substrate around existing statistical infrastructure.

## Relationship to nature-related disclosure

TNFD's current reporting ecosystem already asks organizations and financial institutions to assess and disclose nature-related dependencies, impacts, risks, and opportunities. Its 2026 status report reports more than 1,000 organizations across 56 countries or areas with some level of aligned disclosure.

The harder question is what happens after disclosure.

Mycelix can potentially move:

```
disclosure → attributable state → obligation → monitored remediation
```

rather than stopping at reporting.

## A municipal example

Consider a procurement decision between two projects.

Project A:

- lower monetary cost;
- high short-term employment;
- significant expected water-system degradation.

Project B:

- higher monetary cost;
- lower direct short-term return;
- preserves catchment condition.

A conventional procurement model may rank A higher.

A Mycelix substrate model does not automatically declare B "morally better."

Instead it can expose:

- the monetary cost;
- the physical ecological impact;
- evidence quality;
- attribution uncertainty;
- restoration obligation;
- effects on other required substrate dimensions.

Governance can then decide which trade-offs are legitimate.

The critical difference is that the ecological cost is no longer allowed to remain an invisible externality simply because it is difficult to price.

## The feedback loop

The full system becomes:

```
1. Observe
2. Attribute
3. Account
4. Detect boundary pressure
5. Gate or condition action
6. Restore / maintain
7. Verify
8. Learn
9. Continue
```

This is closer to an economic metabolism than to a static balance sheet.

## Research agenda

The strongest remaining research problems are:

1. How should substrate boundaries be selected from scientific evidence without making science itself a political monopoly?
2. How should shared causal responsibility be represented when attribution is genuinely indeterminate?
3. How should restoration be verified without turning restoration metrics into new Goodhart targets?
4. How should private information be proved through ZK mechanisms without making the substrate opaque?
5. How should local communities participate in defining obligations affecting them?
6. How should cross-border and supply-chain impacts compose without double counting?
7. What minimum substrate dimensions are necessary for a real municipality, cooperative, company, or commons?
8. Which economic actions should be gated during each kind of breach?
9. How can the system distinguish reversible stress from irreversible threshold crossing?
10. How can long-term feedback be tested through adversarial multi-agent simulation rather than human optimism?

## Final design target

The goal is not a machine that decides what a flourishing civilization is.

The goal is a system in which:

> **economic agency can remain decentralized because the consequences of that agency are increasingly observable, attributable, bounded, and repairable.**

That is the sense in which Mycelix could help preserve the foundations of generative capitalism.

