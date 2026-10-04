# Economic OS Architecture — Policy-Neutral, Jurisdiction-Aware, Interoperable

## Thesis

Mycelix should evolve toward an **Economic Operating System (Economic OS)**, but
the kernel must remain policy-neutral.

The Economic OS should provide the stable computational substrate for economic
activity:

**observe → authorize → commit → settle → reconcile → finalize → publish**

Countries, monetary unions, municipalities, cooperatives, mutual-credit
networks, and other economic jurisdictions should provide versioned **policy
profiles** that configure the kernel rather than fork it.

This makes the architecture analogous to an operating system ABI:

- the kernel supplies stable primitives;
- policy profiles define local rules;
- adapters translate external standards;
- authorities decide policy;
- applications implement sector-specific workflows.

## Why this direction is timely

The international economic-data system already has multiple complementary
standards rather than one universal runtime.

The UN 2025 System of National Accounts provides the overarching framework for
national economic accounts and is explicitly designed for countries at different
levels of development.

IMF BPM7 updates external-sector statistics, while the IMF Government Finance
Statistics framework supports fiscal analysis. These frameworks are designed to
remain comparable even as national institutions and policies differ.

SDMX provides an ISO standard for exchanging statistical data and metadata.

SEEA adds environmental-economic accounting, including ecosystem condition,
services, and asset accounts.

ISO 20022 provides a common semantic and message-modeling approach for financial
communications.

The BIS Project Agorá work demonstrates that multi-currency programmable
infrastructure with jurisdiction-specific monetary ledgers is becoming an
active institutional research direction.

Mycelix should therefore **interoperate with these ecosystems rather than
attempt to replace them**.

## Layered architecture

### 1. Economic Kernel

The kernel owns invariants that should survive policy changes:

- identity and authority references;
- timestamps and temporal ordering;
- units and quantities;
- evidence/provenance;
- scopes;
- obligations;
- execution receipts;
- reconciliation;
- settlement state;
- append-only history;
- finalization;
- conflict preservation;
- deterministic content identities;
- privacy boundaries.

The kernel should not contain a universal tax rate, inflation target, welfare
formula, reserve requirement, employment rule, or preferred monetary theory.

### 2. Policy Profile Layer

An `EconomicPolicyProfile` specifies the context in which the kernel is used:

- jurisdiction;
- economic/monetary regime;
- policy version;
- recognized currencies/units;
- authorities;
- policy rules;
- interoperability profiles;
- effective period;
- evidence;
- predecessor/supersession relationship.

Profiles are content-addressed so that a historical economic action can always
identify the exact policy context under which it was authorized.

### 3. Measurement Layer

Measurements remain distinct from judgments.

A measurement observation can represent:

- prices and price changes;
- labor capacity;
- productive capacity;
- inventories;
- energy/material availability;
- ecological conditions;
- financial conditions;
- external balances;
- exchange-rate conditions;
- public-sector flows and stocks;
- household/business conditions.

The same observation can be consumed by different policy models.

### 4. Policy Engine Layer

Policy engines interpret measurements under explicit policy profiles.

Examples could include:

- inflation-targeting;
- fiscal-rule;
- MMT-informed;
- green-growth;
- degrowth/provisioning;
- social-democratic;
- industrial-policy;
- cooperative/mutual-credit;
- emergency/war-economy;
- municipal-budget;
- development-bank;
- custom constitutional policy.

The engine should produce **recommendations and decisions**, never silently
mutate the kernel.

### 5. Authority Layer

The authority layer establishes who may authorize:

- monetary policy;
- taxation;
- public expenditure;
- procurement;
- transfer programs;
- credit facilities;
- emergency actions;
- regulatory changes.

Identity, delegated authority, signatures/proofs, quorum and constitutional
rules belong here.

A non-empty `authority_ref` is not enough by itself.

### 6. Settlement / Payment Adapters

External monetary systems should be adapters:

- ISO 20022;
- domestic payment rails;
- central-bank systems;
- commercial-bank APIs;
- stable-value instruments;
- local/community currencies;
- mutual-credit ledgers;
- DLT/tokenized settlement systems.

The Economic OS should not require a blockchain for every use case.

### 7. Statistical / Reporting Adapters

Adapters should map kernel observations and transactions into:

- 2025 SNA;
- BPM7;
- GFSM;
- SEEA;
- SDMX;
- labor/statistical classifications;
- national procurement reporting.

The critical rule is **lossless semantic preservation where possible**.

When a source standard cannot represent a Mycelix field, the export must say
what was dropped, transformed, or generalized rather than silently pretending
the representations were equivalent.

## The interoperability contract

The core interoperability promise should be:

> **same economic event, different policy interpretation, preserved evidence.**

For example, an action could contain:

- physical output;
- labor hours;
- financial settlement;
- ecological impact;
- tax treatment;
- public-purpose classification;
- evidence;
- authority;
- policy profile.

A South African policy profile, an EU profile, a Japanese profile, a US profile,
or a cooperative profile could interpret the same semantic event differently
without changing its underlying evidence.

## No country forks

The target should be:

**one kernel + many policy profiles + many adapters**

not:

**one kernel fork per country**.

A country-specific implementation should primarily be configuration and adapter
code.

Only genuinely jurisdiction-specific legal or technical primitives should
require specialized modules.

## Economic ABI

The stable API should eventually expose a minimal semantic ABI:

1. `observe`
2. `authorize`
3. `commit`
4. `settle`
5. `reconcile`
6. `finalize`
7. `publish`

Each operation should carry explicit:

- policy-profile identity;
- actor/authority identity;
- scope;
- time;
- units;
- evidence references;
- predecessor/causal references where applicable.

## Multiple economic theories without fragmentation

The OS should make theories **pluggable interpretations**, not mutually
exclusive databases.

For instance, a single observed state could be evaluated by:

### MMT engine
Focuses on monetary sovereignty, employment, real resources and inflation
constraints.

### Mainstream monetary-policy engine
Focuses on inflation expectations, output gaps, policy rates, exchange rates,
financial conditions and credibility.

### Ecological economics engine
Focuses on resource throughput, ecosystem condition, provisioning and
biophysical boundaries.

### Ostrom-style commons engine
Focuses on governance, local fit, monitoring, reciprocity, sanctions and
collective-choice institutions.

### Industrial-policy engine
Focuses on productive capacity, strategic sectors, investment, technology and
public-purpose missions.

The output is not forced into a single score.

Conflicting policy evaluations can coexist.

## The anti-monoculture rule

The Economic OS should reject three dangerous assumptions:

**Money = wealth**

Financial settlement is one state transition, not the definition of wellbeing
or productive capacity.

**One metric = the economy**

No composite score should be allowed to compensate for an independent hard
constraint merely because its weighted average looks healthy.

**One ideology = the protocol**

The kernel should not encode MMT, neoliberalism, socialism, degrowth, or any other
economic theory as a universal constitutional truth.

## Governance and capture

An Economic OS creates an enormous potential concentration of power if the policy
profile registry or authority system becomes centralized.

Therefore the architecture should support:

- multiple authorities;
- explicit delegation;
- constitutional constraints;
- contestability;
- historical audit;
- profile versioning;
- profile supersession;
- independent observation providers;
- conflict-preserving evidence;
- forkability and exit;
- privacy-preserving proofs where appropriate.

Decentralization is useful only when it changes who can exercise power.

## Interoperability tiers

### Tier 0 — Lossless native

Mycelix-to-Mycelix or profile-to-profile transfer with complete semantic
preservation.

### Tier 1 — Standards mapping

Structured mappings to SNA, BPM7, GFSM, SEEA, SDMX, ISO 20022, etc.

### Tier 2 — Constrained legacy export

Older systems that cannot represent all semantics. Every loss is declared.

### Tier 3 — Human-readable reporting

Reports/PDF/dashboard forms intended for human review rather than machine
round-tripping.

## Country/profile examples

The system should make it possible to define, without changing the kernel:

- South Africa;
- United States;
- European Union / euro area;
- Japan;
- India;
- Switzerland;
- municipalities;
- indigenous/community governments;
- cooperatives;
- mutual-credit networks;
- development zones;
- humanitarian economies.

This does not mean Mycelix declares any of those systems legally valid. A profile
is an explicit technical model that can reference the relevant authorities,
rules and evidence.

## The practical product

The most defensible public description becomes:

> **Mycelix is an open Economic OS for auditable economic coordination across
> jurisdictions and policy regimes.**

Not:

> “Mycelix replaces national currencies.”

Not:

> “Mycelix implements MMT.”

Not:

> “Mycelix is a blockchain central bank.”

The kernel is infrastructure. The policy belongs to the jurisdiction.

## Research boundaries

The next engineering seams are:

- AC-052: measurement-only real-resource observations;
- AC-053: versioned economic policy profiles;
- policy-profile fingerprint binding on governed decisions;
- authority proof/signature integration;
- canonical cross-language serialization;
- SNA/BPM7/GFSM/SEEA/SDMX adapters;
- ISO 20022/payment-rail adapters;
- conflict-preserving multi-source observations;
- jurisdiction conformance suites;
- economic simulation and counterfactual testing.

## Design objective

The long-term test is simple:

> Can two jurisdictions use the same Economic OS kernel, apply genuinely
> different economic policies, exchange interoperable economic information,
> settle transactions across boundaries, and still preserve exactly what each
> side observed, authorized, executed, reconciled, and finalized?

If yes, Mycelix has become an economic operating system rather than another
currency project.
