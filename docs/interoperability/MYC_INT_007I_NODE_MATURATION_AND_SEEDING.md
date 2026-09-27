# MYC-INT-007I — Node Maturation and Independent Seeding Lifecycle

Status: design/fixture only. Tracks #3267. Cross-cutting child of the Integral federation showcase and productive-node showcase.

## Purpose

Show a node progressing from a grassroots/proto-node state through measurable local capability and resilience, then helping another community start a node **without becoming its parent authority**.

The public story is:

```text
grassroots group
    ↓
visible needs + capabilities
    ↓
instrumented node
    ↓
operational productive node
    ↓
locally resilient node
    ↓
reduced dependency in selected domains
    ↓
seeder node
    ↓
independent receiving node
    ↓
optional federation as peers
    ↓
receiving node later seeds another node
```

This is recursion without hierarchy.

## External source basis

Observed 2026-09-27, Integral's current public transition material describes:

- proto-nodes and mutual-aid foundations;
- gradual emergence of the five systems;
- scaling through use rather than theoretical completeness;
- hybrid coexistence with existing institutions;
- later federation and inter-node cooperation;
- relationship-building and mutual accountability between proto-nodes before mature technical federation.

007I is a Luminous technical interpretation and experiment profile. It is **not** an Integral-ratified node-maturity specification.

## Do not model `self_sustaining: bool`

A node can be highly capable in food and repair while still depending on external electronics, medicine, legal structures, finance, energy, software maintainers, or critical materials.

Therefore the lifecycle records capability and dependency independently by domain.

```text
local capability in domain X
!= independence in domain Y

reduced external dependency
!= autarky
```

The fixture tracks dimensions including food, water, energy, repair/fabrication, compute/network operations, governance operations, evidence/provenance, health/safety support, legal/finance dependencies, critical imports, skill coverage, recovery/spares, and federation dependency.

There is no scalar self-sufficiency score.

## Lifecycle generations

### N0 — Grassroots / proto-node

People and relationships exist before a complete technical system.

Expected characteristics include:

- basic needs/capabilities inventory;
- accessible decision records;
- practical local cooperation such as food, tool sharing, repair, skills or mutual aid;
- explicit external dependencies;
- no claim of a complete Mycelix/Integral node.

### N1 — Instrumented node

Adds source-owned observation and operational visibility:

- asset/resource inventory;
- local Luminous Edge acquisition where relevant;
- explicit missing/stale/error state;
- dependency mapping;
- qualified Mycelix semantic/provenance projection only where upstream gates permit.

No autonomous physical control is required.

### N2 — Operational productive node

Adds at least one real productive loop such as the H1/productive-node line:

- food/water/energy/material activity;
- work/material observations;
- maintenance and repair;
- outcome feedback;
- optional Integral-shaped adapters;
- continued external interfaces remain visible.

### N3 — Locally resilient node

Demonstrate bounded continuity and graceful degradation under selected faults:

- internet/federation outage;
- provider/service outage;
- component failure and repair;
- local inventory shortage;
- software rollback/recovery;
- stale or unknown remote data;
- Symthaea unavailable.

The node reports which functions continue, degrade or stop rather than displaying a single health bit.

### N4 — Reduced-dependency / regenerative node

Demonstrate measured changes in selected dependency dimensions, for example:

- higher water reuse;
- higher local energy share;
- greater local repair/fabrication capability;
- more reusable/closed material loops;
- improved spare readiness;
- explicit remaining critical imports.

If one dependency falls while another rises, both remain visible.

### N5 — Seeder node

A node can prepare a **Node Seed Package** for another community.

Possible package classes include:

- open designs, BOMs and process profiles;
- reproducible deployment artifacts;
- fixtures, validators and conformance evidence;
- operations/maintenance/recovery documents;
- training and optional mentoring;
- calibration/check procedures;
- locally fabricated starter parts or tools;
- optional starter inventory/material support;
- federation introduction/recognition artifacts.

Every package has `authority = none`.

### N6 — Independent seeded node

The receiving community:

- accepts or rejects each seed component;
- generates new keys and node identity;
- owns its governance locally;
- validates/adapts designs for local conditions;
- records translation/adaptation loss;
- establishes its own currentness/evidence state;
- can operate before joining federation;
- can decline or leave federation;
- does not automatically inherit certification, reputation, credit, standing or authority.

After bootstrap, the relationship is peer-to-peer.

### N7 — Recursive seeder

The strongest demonstration is when the seeded node can later produce its own seed package **without the original node being required**.

That is the recursion theorem:

```text
A helps B start
B becomes independent
B helps C start

A is not C's root authority
```

## Seed-package anti-collapse rules

```text
DesignPack
!= locally accepted design

foreign certification
!= local certification

deployment image
!= node identity

training completion
!= governance standing

resource donation
!= authority

mentor recommendation
!= local decision

federation introduction
!= mandatory federation membership
```

A copied deployment image must never copy source-node private keys, secrets, local authority subjects, balances, reputation or standing.

## Relationship to 007F/007G federation

The existing F0 showcase begins with:

- Node A — one physical H1/productive node;
- Node B — replay-backed second productive node.

007I gives that relationship a future progression:

```text
Node A N2/N3
    ↓
A reaches N5 Seeder capability
    ↓
A exports exact SeedPackage generation
    ↓
Node B transitions from replay-backed fixture
into an independently bootstrapped node
    ↓
B creates its own keys, local policy and evidence state
    ↓
A and B optionally federate as peers
    ↓
B later demonstrates N7 by seeding another node
```

007G run manifests should eventually bind:

- node-maturation generation;
- exact seed-package generation;
- exact imported/rejected/adapted artifacts;
- bootstrap operator steps;
- seeder availability/outage conditions;
- federation status separately from local-operability status.

## Relationship to the productive-node line

The `018*` line provides the first physical capabilities a node can actually mature around:

- hydroponic/productive H1 node;
- water/energy/material observation;
- field instrumentation through Luminous Edge;
- repair/fabrication expansion;
- hardware/run evidence;
- later domain modules such as compost, soil comparison, preservation and distribution.

The maturity model must never imply that adding more modules automatically increases institutional legitimacy or authority.

## Symthaea boundary

Symthaea may help with:

- dependency analysis;
- fault diagnosis;
- design adaptation;
- counterfactual resource planning;
- seed-package adaptation recommendations;
- identifying missing skills/resources;
- forecasting bootstrap risks.

But:

```text
Symthaea adaptation recommendation
!= local acceptance
!= certification
!= governance authority
```

## Nix / Spore / Xenia boundary

Reproducible deployment and secure communications can reduce bootstrap friction, but they are not institutional identity.

```text
same Nix closure
!= same node

same software generation
!= shared keys
!= shared governance
```

A seed image must instantiate a **new** node identity and local secret set.

## Adversarial campaign

The machine-readable fixture freezes S01–S18, including:

- copied private keys;
- copied governance standing;
- automatic foreign-certification acceptance;
- copied reputation/credits;
- stale seed artifacts;
- unavailable local material/process;
- declared translation loss;
- seeder outage during bootstrap;
- selective rejection of seed components;
- operation before federation;
- voluntary federation exit;
- donation interpreted as authority;
- mentor mutation attempt;
- locally wrong recommendation;
- unserviceable imported hardware;
- dependency improvement in one domain worsening another;
- false binary self-sufficiency claim;
- successful second-generation seeding without the original node.

## Measurement model

Measure separate dimensions rather than ranking nodes:

- local service continuity by domain;
- external dependency count and criticality;
- measurable local energy/water/material shares;
- repair turnaround;
- spare availability;
- maintainer/skill coverage;
- bootstrap time and operator steps;
- imported versus rejected/adapted seed artifacts;
- translation/adaptation loss;
- time until the receiving node can operate while the seeder is unavailable;
- federation-optional operation duration;
- provenance completeness.

The showcase may visualize these dimensions over time, but must not collapse them into one "maturity" or "self-sufficiency" score.

## Public demonstration narrative

A good demonstration would visibly begin small:

```text
N0
community inventory + relationships

N1
first sensors / resource map / evidence

N2
first useful productive loop

N3
survive faults without pretending nothing degraded

N4
reduce selected external dependencies

N5
package reusable designs/software/training/tools

N6
another community starts independently

N7
that community later helps another
```

The compelling moment is not Node A becoming dominant. It is **Node A becoming less necessary**.

## Nonclaims

007I does not establish:

- economic self-sufficiency;
- ecological sustainability;
- social or political legitimacy;
- legal compliance;
- superiority of any governance/economic model;
- that every community should follow one maturation sequence;
- that resource independence is always desirable;
- that federation is required for local operation.

It defines a testable technical transition and showcase model only.
