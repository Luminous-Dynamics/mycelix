# REGEN-000 — Regenerative Commons Constitution v0.1

Status: normative architecture draft; no runtime or physical authority.

## 1. Purpose

Mycelix Regenerative Commons defines the cross-domain constitutional boundary for communities that want to improve food, water, soil, nutrient, biomass, energy, repair, ecological, and infrastructure resilience without collapsing those systems into one optimizer, one score, one authority, or one application.

The first proving family is a Terra-Preta-inspired soil loop using qualified biomass, pyrolysis/biochar, compost or co-composting, field trials, useful heat recovery, and evidence-bearing local learning.

The program is intentionally broader than biochar. Biochar is one possible transformation inside a regenerative material-flow network.

The governing objective is:

```text
regenerative resilience
    = ability to preserve essential services,
      restore degraded capability,
      improve ecological condition,
      and reduce brittle dependency
      without manufacturing authority or hiding trade-offs
```

This constitution does not define a universal recipe for agriculture, biochar, compost, community self-sufficiency, carbon removal, or local production.

## 2. Core constitutional theorem

```text
observation
!= inference
!= recommendation
!= adopted policy
!= authority
!= physical action
!= verified outcome
```

No model output, sustainability label, carbon claim, resilience metric, market listing, community vote, capital investment, or previous successful trial may skip these boundaries.

A second theorem applies to resource circularity:

```text
locally available
!= sustainably available
!= safe feedstock
!= suitable process input
!= qualified product
!= suitable local application
```

## 3. Domain ownership

REGEN is a composition layer. It MUST NOT become a second authoritative implementation of existing Mycelix domains.

Authoritative ownership remains with the systems that already own the relevant state, including where applicable:

- Water — water rights, water quality, water infrastructure, allocation, and service state;
- Energy — energy assets, service capability, metering, and energy-domain authority;
- Climate — carbon/project claims, climate project lifecycle, climate evidence, and carbon-credit semantics;
- Finance — authoritative amounts, settlement, financing, claims, and payment state;
- Commons / Property — stewardship, ownership, asset-lock, and shared-resource governance;
- Marketplace / Commerce — market listings, offers, orders, and exchange semantics;
- Supply Chain — custody, logistics, source, and material-movement evidence;
- Manufacturing / Craft — production capability and process/work artifacts;
- Praxis / Governance — community decisions, adopted rules, roles, and institutional authority;
- Knowledge — procedures, recipes, documentation, and provenance;
- Identity / Justice — actor identity, dispute, accountability, and rights;
- Climate / planetary evidence types — environmental observations, provenance, and risk composition.

REGEN may bind, reference, compose, or project these facts. It MUST NOT silently duplicate them as a new source of truth.

## 4. Mycelix and Symthaea boundary

Mycelix owns social/economic/institutional coordination and evidence-bearing shared records.

Symthaea may:

- model;
- simulate;
- compare;
- estimate;
- generate hypotheses;
- identify uncertainty;
- propose experiments;
- rank policy-valid alternatives;
- produce recommendation-only artifacts.

Symthaea MUST NOT acquire authority merely because it predicts well.

```text
SymthaeaRecommendation
    -> review / adopted policy / independent authority checks
    -> possible prepared action
    -> separately qualified effect boundary
```

There is no permitted shortcut:

```text
SymthaeaRecommendation -> physical actuation
```

## 5. No universal self-sufficiency objective

REGEN MUST NOT optimize `maximize(local_production)` or `maximize(self_sufficiency)` as a universal objective.

A locality decision must preserve visible trade-offs including, where relevant:

- essential-service resilience;
- ecological burden;
- safety;
- cost and affordability;
- energy and water burden;
- labor and skill burden;
- repairability;
- import concentration;
- recovery time;
- nutritional adequacy;
- land use;
- pollution;
- uncertainty.

Importing a product may be more resilient and ecologically sound than local production. Local production may be superior where external dependencies are brittle or where local loops materially improve resource recovery. The architecture must be able to represent either outcome.

## 6. Ecological floor

Circularity is subordinate to ecological retention requirements.

A biomass stream MUST NOT be considered freely recoverable merely because it is technically collectable.

The conceptual allocation boundary is:

```text
gross biomass occurrence
- ecological retention requirement
- soil-cover / erosion requirement
- habitat requirement
- existing higher-priority use
- contamination / exclusion
= candidate recoverable biomass
```

Each subtraction may be uncertain or policy-dependent. Missing evidence MUST NOT silently become zero retention.

Resource availability never creates permission to extract the resource.

## 7. Biochar and soil claim separation

The program freezes the following non-equivalences:

```text
biochar
!= fertilizer

biochar quality
!= agronomic suitability

agronomic suitability
!= carbon-removal eligibility

carbon-removal eligibility
!= crop-yield improvement

soil-carbon increase
!= whole-system sustainability

one successful field trial
!= universal treatment recipe
```

A batch may be chemically acceptable but unsuitable for a particular soil/crop/context. A batch may support a carbon-accounting claim while lacking evidence of agronomic benefit. An amendment may improve a field outcome while not meeting a carbon-removal methodology. These propositions remain independently evidenced.

## 8. Quality-profile adoption

REGEN core contracts SHOULD NOT permanently hard-code one private, national, or international biochar/compost/soil standard as universal authority.

Instead, an institution or community may adopt one or more versioned quality profiles that bind:

- profile identity;
- issuer / authority context;
- exact version;
- immutable content digest where feasible;
- applicable material classes;
- required measurements;
- thresholds / decision rules;
- temporal validity or supersession rules;
- jurisdiction / program scope.

A result such as `passes_profile_X` proves only conformance to the adopted profile at the evidence available. It does not automatically imply agronomic suitability, ecological desirability, carbon eligibility, or legal compliance outside that profile.

## 9. Evidence model

REGEN SHOULD reuse Mycelix planetary/environmental evidence primitives for measurements and observations instead of inventing another generic measurement schema.

Domain records should primarily contain canonical references to observations for quantities such as:

- soil pH;
- texture;
- organic carbon;
- moisture / water retention;
- bulk density;
- nutrient observations;
- contaminant measurements;
- feedstock composition;
- process temperature history;
- product composition;
- crop outcome;
- water use;
- energy use;
- ecological indicators.

This preserves:

```text
measurement payload
!= regenerative interpretation
```

A field-trial record can therefore bind observations without relabeling them as deterministic truth about causality.

## 10. Core subject identities

Future executable tranches may define canonical identities for at least:

- `RegenerativeSiteId`;
- `SoilPlotId`;
- `BiomassLotId`;
- `BiocharBatchId`;
- `CompostBatchId`;
- `CoCompostedAmendmentBatchId`;
- `RegenerativeRecipeId`;
- `FieldTrialId`;
- `TreatmentArmId`;
- `RegenerativeFacilityId`;
- `RegenerativeProjectId`;
- `QualityProfileId`;
- `RegenerativeEvidenceBundleId`.

Stable semantic identity and exact revision/content identity SHOULD remain separate when a subject can evolve through revisions.

Identity does not establish quality, existence, availability, suitability, currentness, authority, or ownership.

## 11. Feedstock provenance

A biomass/feedstock claim SHOULD preserve enough information to distinguish:

- source identity;
- source category;
- custody / collection lineage;
- prior use;
- contamination evidence;
- moisture / composition evidence where material;
- ecological retention decision;
- competing-use decision;
- intended transformation;
- applicable quality profile.

Unknown provenance MUST remain unknown. A market label such as `clean wood` or `agricultural waste` MUST NOT substitute for evidence where a consequential use requires stronger assurance.

## 12. Batch lineage

A biochar, compost, or combined amendment batch SHOULD bind:

```text
input lots
+ transformation process identity
+ process evidence
+ output batch identity
+ laboratory / field quality evidence
+ custody lineage
```

For pyrolysis-derived products, useful process facts may include temperature/time/process-family evidence, but the constitution does not define a universal thermal recipe.

For compost/co-compost, maturation or stabilization evidence MUST remain distinct from mere elapsed time.

## 13. Regenerative recipe boundary

A `RegenerativeRecipe` is a versioned evidence/procedure artifact, not universal agronomic truth.

It may bind:

- intended context;
- input material assumptions;
- process assumptions;
- product quality requirements;
- soil/site context;
- crop context;
- application method;
- expected effects;
- known contraindications;
- uncertainty;
- supporting evidence;
- revision lineage.

A new context may require a new recipe revision or a new trial. Copying a recipe to a different soil, climate, crop, feedstock, or process configuration MUST NOT automatically preserve outcome claims.

## 14. Trial constitution

A qualified field-trial contract SHOULD preserve at least:

- site / plot identities;
- preregistered treatment arms;
- baseline observations;
- treatment material/batch identities;
- application timing and method;
- relevant environmental observations;
- outcome observations;
- deviations;
- missing data;
- uncertainty;
- analysis identity;
- trial conclusion / non-conclusion boundary.

Where practical, controls or comparators SHOULD be explicit.

A trial result MUST NOT erase heterogeneous response. If two sites respond differently, the system records that difference rather than averaging it into a false universal claim.

## 15. Negative results

Negative and null results are first-class evidence.

The architecture MUST permit records such as:

```text
soil property improved
crop outcome unchanged
```

or:

```text
water retention improved
nutrient availability worsened
```

without requiring a single success/failure label.

A negative trial MUST NOT be silently dropped from evidence lineage merely because it weakens a favored recipe or project narrative.

## 16. Nutrient and material conservation

Future nutrient/material accounting MUST prevent double-spending.

A recovered unit of carbon, nitrogen, phosphorus, potassium, biomass, water, or energy cannot satisfy two simultaneous downstream claims unless the physical model explicitly supports that relationship.

The architecture should prefer transparent stock-and-flow accounting over an opaque `circularity_score`.

```text
circularity evidence
!= zero external dependency
!= zero waste
!= sustainability proof
```

## 17. Resilience representation

REGEN MUST NOT define one canonical universal resilience score.

A community-facing resilience state should preserve separate dimensions such as:

- nutritional service;
- potable water service;
- irrigation reserve;
- soil condition;
- nutrient import dependency;
- energy essential-load capability;
- sustainable biomass availability;
- seed / propagation capability;
- equipment repairability;
- critical skills / succession;
- logistics concentration;
- ecological condition;
- recovery paths;
- uncertainty.

An adopted governance process may define contextual thresholds, but the underlying dimensions remain available for inspection.

## 18. Essential-service floor

Optimization MUST respect an adopted non-optimizable essential-service floor.

The exact policy is contextual, but may include minimum obligations around:

- safe drinking water;
- minimum nutrition;
- sanitation;
- shelter / thermal safety;
- essential energy;
- minimum soil / watershed / ecological protection;
- emergency healthcare support;
- human standing / non-coercion.

A recommendation that improves a scalar objective while violating an adopted essential-service floor is inadmissible for that profile.

## 19. Human standing and labor

Resource scarcity or resilience analysis MUST NOT manufacture authority over people.

A model may identify:

- a skill bottleneck;
- a missing operator;
- a training need;
- a labor estimate;
- a dangerous dependency concentration.

It may not infer that a specific person must work, relocate, train, surrender property, or accept an institutional role.

```text
capability dependency on a human skill
!= ownership of the skilled person
```

## 20. Markets and local preference

A local-procurement preference may rank already-qualified alternatives. It MUST NOT make an unsafe or unqualified product eligible.

```text
hard safety / quality / rights constraints
    -> qualified candidate set
    -> optional local / cost / energy / resilience preferences
```

Soft preferences may alter ranking. They do not alter eligibility.

## 21. Capital-to-commons relationship

Shared regenerative infrastructure may include assets such as:

- composting facilities;
- pyrolysis / biochar facilities;
- greenhouses;
- food-storage / processing infrastructure;
- water-storage / treatment / irrigation infrastructure;
- repair / fabrication workshops;
- shared energy systems.

Where such infrastructure has public-good, essential-service, local-monopoly, or long-lived commons characteristics, it SHOULD be able to compose the Capital-to-Commons pattern rather than inventing a new financing constitution.

Investor return, asset stewardship, operating rights, community economics, and protocol authority remain separate rights.

## 22. Climate/carbon firewall

Agronomic evidence and climate/carbon evidence are separate authority lines.

A regenerative project may expose evidence to the Climate domain, but REGEN MUST NOT directly mint carbon credits or carbon-removal truth.

```text
RegenerativeEvidenceBundle
    -> Climate-domain methodology / verification
    -> possible climate claim
```

No shortcut is permitted from `biochar batch exists`, `crop yield improved`, or `soil carbon increased` directly to a credit issuance state.

Likewise, a carbon certification or credit state does not prove that applying the material to a particular soil is safe or beneficial.

## 23. Physical-action boundary

REGEN-000 authorizes no actuator, pyrolyzer, irrigation valve, pump, spreader, vehicle, robot, dosing system, or industrial controller.

The initial operating model is:

```text
observation
    -> Mycelix evidence
    -> Symthaea analysis / recommendation
    -> human / community / institutional decision
```

Consequential automation, if ever added, requires a separate explicitly qualified authority-and-effect stack with current state, safety constraints, exact action binding, commit uncertainty, reconciliation, and local physical interlocks.

## 24. Security and adversarial assumptions

Future REGEN implementations SHOULD test at least:

- forged feedstock provenance;
- mislabeled batch identity;
- omitted contaminant evidence;
- stale laboratory evidence;
- recipe/result substitution;
- cherry-picked positive trial outcomes;
- hidden failed trials;
- double-counted material recovery;
- duplicated carbon claims;
- locality preference bypassing safety;
- model confidence masquerading as authority;
- market listings claiming stronger quality than evidence supports;
- recycled material treated as automatically safe;
- resilience claims hiding common-mode dependency;
- capital financing attempting to acquire protocol/governance authority.

## 25. First proving scenario

The first integrated proving scenario SHOULD be deliberately narrow:

> A community has one qualified clean biomass-residue stream, a bounded pyrolysis process, compost or co-compost capacity, a greenhouse or agricultural field, an optional useful-heat sink, and repeated soil/crop observations. Can Mycelix preserve exact material/evidence lineage and can Symthaea compare interventions without inventing universal agronomic truth or acquiring execution authority?

The proving scenario should include:

1. an untreated/control context;
2. at least one treatment context;
3. explicit feedstock/batch lineage;
4. explicit quality evidence;
5. explicit material and energy accounting;
6. explicit unknown/unusable coproducts;
7. repeated measurements;
8. at least one adverse or null case;
9. no automatic physical execution.

## 26. Immediate tranche map

The immediate program is intentionally small:

```text
REGEN-000  this constitution
REGEN-001  Symthaea regenerative-systems research profile
REGEN-002  canonical regenerative subject identities
REGEN-003  adopted quality-profile contract
REGEN-004  cross-repository identity/golden-vector contract
REGEN-005  authority/claim matrix
REGEN-010  soil-site observation profile
REGEN-011  biomass feedstock provenance
REGEN-012  biochar batch lineage
REGEN-013  compost batch lineage
REGEN-015  amendment field-trial contract
REGEN-030  Symthaea terrestrial PIE projection
REGEN-080  synthetic community soil-loop proving scenario
```

Later PRs are gated on evidence from these tranches rather than being assumed in advance.

## 27. Deliberate non-claims

REGEN-000 does not establish:

- agronomic efficacy;
- biochar safety;
- compost safety;
- carbon permanence;
- carbon-credit eligibility;
- climate additionality;
- legal or regulatory compliance;
- food safety;
- water safety;
- plant or equipment qualification;
- local self-sufficiency;
- resilience superiority;
- economic viability;
- community legitimacy;
- investment suitability;
- physical-action authority.

It defines only the constitutional composition and evidence boundaries that future executable work must preserve.
