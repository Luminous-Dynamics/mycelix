# Local Capability Compiler: Context Profile Contract (v0.1)

**Status:** design and validation prototype; not a deployment recommendation  
**Scope:** model what a place can sustain before proposing an intervention  
**Owner boundary:** Mycelix records provenance, permissions, review, decisions, and outcomes; Sol-Atlas provides spatial context without exposing sensitive locations; Symtropy evaluates explicit, validated physical models; Symthaea may propose bounded hypotheses and surface uncertainty.

## Why this exists

A universal technology bundle is the wrong abstraction. A dense city with reliable electricity, a semi-arid rural settlement with scarce cash, and an offline community workshop have different constraints and different failure modes. Even within one region, household and neighborhood conditions can diverge sharply.

The compiler should accept a **Local Capability Profile** and compare candidate interventions against its resource envelope, stated priorities, evidence quality, safety gates, and maintenance capacity. The profile is not a measure of a people's sophistication, value, or development. Do not build a single “civilization score,” rank cultures, or use labels such as “Stone Age” as machine-readable categories.

## Design rules

1. **Prioritize outcomes, not novelty.** Start with the needs people identify. Water safety, sanitation, food security, shelter and thermal safety, healthcare access, repairable tools, energy, communications, transport, ecological stewardship, learning, livelihoods, and civic coordination are capability areas—not a universal rank order.
2. **Represent constraints as a vector, not an income label.** Capture power reliability, communication options, cash access, manufacturing/repair capacity, supply-chain reach, water-service situation, health access, transport, and knowledge-sharing options independently. Unknown is a valid value.
3. **Separate fact from hypothesis.** Each asserted resource value carries an evidence basis and references. A schema validates record shape; it does not prove a source exists, a fact is accurate, or a solution works. Illustrative fixtures are not measurements of actual communities.
4. **Fail closed on missing evidence.** Missing consent, legal authority, water/clinical/structural safety, source rights, maintenance ownership, accessibility, privacy, or ecological checks must not silently become a pass.
5. **No opaque city or community score.** First enforce hard safety, rights, consent, and maintenance gates. Then compare feasible options on separate dimensions—benefit, lifecycle cost, repairability, risk, resilience, accessibility, ecological impact, and distribution of gains/burdens. Show trade-offs and uncertainty; do not let a weighted average erase a failed rights gate.
6. **Offline-first and locally reproducible.** The field package must remain useful as paper/illustrations and oral demonstration, with no required account, internet, QR code, cloud API, or continuously powered device. Digital interfaces are optional accelerators.
7. **Repair and exit are first-class requirements.** Record required skills, consumables, failure modes, substitutes, repair procedures, ownership of maintenance, and what happens when imported parts or outside support disappear. Prefer a community's ability to reproduce a capability over permanent vendor dependence.
8. **Use reversible, testable steps.** Begin with the smallest locally relevant intervention that can be safely evaluated. Record a baseline, a target, counter-metrics, stop conditions, maintenance funding, and post-pilot review before deployment.

## Economic and infrastructure envelope

The following labels are only convenient enum values for input profiles. They are not a universal ladder, and they must never be used to infer a community's preferences or skills.

| Dimension | Illustrative values | What it changes |
|---|---|---|
| Electricity | none/unreliable, intermittent, community system, reliable grid | Whether a design can require powered equipment, refrigeration, or continuous monitoring |
| Digital communication | none, offline devices, intermittent connection, reliable connection | Whether instructions and coordination must work without a network |
| Cash access | none/rare, scarce, local markets, formal finance, institutional procurement | Up-front affordability, pooling, financing, and recurring-cost constraints |
| Manufacturing | hand tools, shared workshop, local machine shop, industrial supply chain | Which parts can realistically be made or repaired locally |
| Supply chain | local reuse, seasonal/limited, road-dependent, diverse network | Spares availability and failure recovery |
| Water service | no improved service, variable small supply, managed small supply, network service | Risk assessment and locally appropriate water-safety planning |
| Healthcare access | community care only, periodic clinical support, basic clinic, referral network | Emergency planning and where clinical expertise must remain involved |
| Transport | walking/carry, local paths, seasonal vehicle access, road-connected, multimodal | Tool/material movement and service access |
| Knowledge exchange | oral demonstration, paper/visual, offline digital, intermittent online, connected | Training, documentation, and continuity without literacy or power assumptions |

The system must not infer that every off-grid community needs a solar panel, that every rural context needs a new road, or that every cash-constrained context should be treated as a market. Assess local resources and priorities first.

## Minimal deployment package

When a community has actively requested assistance and deployment is lawful, the minimal package is a process plus one selected intervention—not a fixed crate of devices:

- **Listen and agree:** identify who has authority to participate, what outcome they want, what data may be recorded, and how they can reject or stop the work.
- **Choose one capability:** make the choice with affected people; document alternatives and why simpler/locally available options were accepted or rejected.
- **Verify safety and fit:** apply relevant current health, engineering, ecological, accessibility, and legal requirements. For water, use risk-based water safety planning and appropriate testing/surveillance; a visually clear source is not proof of drinking-water safety.
- **Teach, repair, and reproduce:** use locally intelligible instructions and demonstrations. Include spare parts, substitutions, maintenance intervals, failure signs, and repair training.
- **Measure and leave capability behind:** track outcomes and unintended effects; publish only information that people have authorized for publication; provide a maintenance/exit plan and do not make continued platform access a condition of use.

### Special protection state: protective_no_contact

This is a **policy sentinel**, not a field deployment mode and not a way to create dossiers about isolated Indigenous peoples.

For Indigenous peoples in voluntary isolation or initial contact, the first action is not “deliver the minimal technology package.” Do not initiate contact, map or publish precise locations, collect cultural or ecological knowledge for external use, leave unsolicited objects, or infer their needs from an outsider's model. Respect self-determination and applicable national protection measures, and refer genuine protection concerns to the appropriate Indigenous protection authority.

The United Nations' 2025 expert paper names no contact, self-determination, and cultural sensitivity among its protection principles; Brazil's FUNAI explains that isolation can reflect historical harms including disease, violence, and dispossession. These principles are documented at:
- UN Department of Economic and Social Affairs, *Protecting the rights of Indigenous Peoples in isolation and initial contact* (2025): https://social.desa.un.org/sites/default/files/IPDB/N2502412.PDF
- FUNAI, *Povos Isolados*: https://www.gov.br/funai/pt-br/atuacao/povos-indigenas/povos-indigenas-isolados-e-de-recente-contato-2/povos-isolados-1
- FAO, *Free, Prior and Informed Consent*: https://www.fao.org/indigenous-peoples/pillars-of-work/free--prior-and-informed-consent/

In this state, the profile must be non-geographic, must contain no local resource claims, capability inventory, priorities, sources, or restricted knowledge, and must not authorize intervention. The validation fixture is deliberately synthetic and contains no information about any real people or place. The profile contract cannot authorize field work; law, protection authorities, and rights-holders remain authoritative.

## Evidence anchors

These are methodological anchors and do not themselves validate any local profile or intervention:

- WHO, *Guidelines for drinking-water quality: fourth edition incorporating the first, second and third addenda* (17 June 2026): https://www.who.int/publications/i/item/9789240121225
- WHO, *Guidelines for drinking-water quality: small water supplies* (2024): https://www.who.int/publications/i/item/9789240088740
- WHO, *Sanitary inspection packages for small water supplies* (2024): https://www.who.int/publications/i/item/9789240089006
- Sphere Handbook and humanitarian minimum standards covering WASH, food security/nutrition, shelter/settlements, health, and protection/accountability: https://handbook.spherestandards.org/en/sphere
- UNDRR Disaster Resilience Scorecard for Cities, useful as a facilitated gap-finding framework rather than an objective city ranking: https://mcr2030.undrr.org/disaster-resilience-scorecard-cities

The Sphere handbook is in a 2026–2028 revision cycle; check the current edition and relevant national standards at implementation time. The WHO water guidelines are updated resources, not a substitute for local regulation, qualified testing, or independent surveillance.

## Project ownership

- **Mycelix:** schema, source/rights provenance, authorization state, review/dissent, governance decisions, maintenance commitments, and outcome records. Unknown or delayed evidence stays unknown.
- **Sol-Atlas:** map and compare permitted geographic evidence, spatial scales, hazards, service coverage, and uncertainty. Sensitive locations are never a default visualization layer.
- **Symtropy:** simulate a specific intervention only when the physical model, inputs, boundaries, units, and validation evidence are versioned. Simulation output is not a measured outcome or a safety certification.
- **Symthaea:** propose hypotheses, compare trade-offs, identify missing evidence, and explain uncertainty. It does not choose a community's priorities or override consent/safety gates.

## Prototype acceptance gates

- Schema validates against JSON Schema Draft 2020-12 with URI-format checking enabled.
- Positive illustrative profiles span high-service urban, semi-arid/low-cash, offline resource-constrained, and non-geographic protective-policy cases.
- Negative controls reject precise location use in the no-contact sentinel, maturity claims without consent evidence, dangling source references, and measured-outcome claims without measured-outcome sources.
- Mutation tests reject unexpected top-level/nested fields, invalid enum substitutions, malformed URIs, and wrong-typed evidence references.
- A passing test proves these software contracts only. It does not prove that profile data are accurate, a proposed solution is feasible, a community consents, or an intervention is safe.
