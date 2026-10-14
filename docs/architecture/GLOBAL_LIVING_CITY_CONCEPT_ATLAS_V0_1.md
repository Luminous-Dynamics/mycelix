# Global Living-City Concept Atlas — v0.1

Status: design proposal; concept portfolio and evaluation framework.
Purpose: build a global, culturally grounded library of beautiful, ecologically responsive, socially useful urban futures that can inspire communities and progressively connect to evidence-backed planning.
Scope: reference concepts first; Johannesburg is the first validation context for the shared technical workflow, not the template every city must copy.


## Implementation progress — 2026-10-14

The first data-contract slice now exists on this branch:

- Schema: `schemas/living-city-concept-v0.1.schema.json` (JSON Schema Draft 2020-12).
- Valid concept fixtures: Johannesburg / Highveld, an illustrative hot-arid context, and an illustrative river-delta context under `fixtures/living-city/concepts/`.
- Deliberately invalid fixtures under `fixtures/living-city/invalid/`: missing climate provenance, missing cultural-review status, and missing source-rights metadata.
- Fixtures intentionally contain no precise coordinates, personal records, or cultural knowledge presented as community-authorized.

**Verification boundary:** these files have been committed, but this documentation/fixture change is not yet an automated schema-validation pass. The next gate is a repository-native validator that proves all valid fixtures are accepted and all negative controls are rejected. Until that gate runs against an exact revision, do not report the fixtures as validated.

### Research anchors for the first implementation

- UN-Habitat, [Integrating Nature-based Solutions into Urban & Territorial Planning](https://unhabitat.org/integrating-nature-based-solutions-into-urban-territorial-planning) (2025): supports evaluating green-blue infrastructure across landscape, city, and neighborhood scales, with governance and distributional impacts considered.
- UN-Habitat, [Global Public Space Programme](https://unhabitat.org/programme/global-public-space-programme): supports treating connected, inclusive, safe, accessible public space as urban infrastructure, not decoration.
- UN-Habitat, [Urban Indicators Database guidance](https://data.unhabitat.org/pages/guidance): provides candidate measurement families for public-space access, tenure, basic services, transport, and participation. These indicators should be adopted only when their definitions and data coverage fit the specific pilot.

These sources are methodological anchors, not evidence that any particular fixture is locally accurate or that a proposed intervention will work.

## North star

Create many distinct urban futures, not a single global style. Each concept should express the relationship between place, climate, culture, materials, public life, ecology, infrastructure, and local aspirations. The atlas is a learning library, not a ranking of cultures or a claim that an outside designer can define a community's identity.

A concept becomes useful when it moves through a traceable chain:

cultural and ecological brief -> design principles -> spatial prototype -> infrastructure model -> scenario comparison -> community critique -> revised concept -> locally validated pilot.

Keep imaginative concepts, researched precedents, simulated predictions, and measured real-world outcomes explicitly distinct.

## Design principles

1. Cultural agency: invite local architects, craftspeople, historians, residents, and cultural custodians to define what is meaningful. Do not treat cultures as fixed, homogeneous, or reducible to decorative motifs.
2. Living heritage: learn from historic settlement patterns, public life, craft, landscape, and building practices while supporting contemporary adaptation. Credit sources and honor community-defined restrictions on sensitive knowledge or sacred designs.
3. Climate-first design: respond to actual solar exposure, wind, rainfall, humidity, heat, cold, flood, drought, wildfire, and local ecological conditions.
4. Beauty as a public good: assess streets, thresholds, shade, acoustics, materials, night lighting, views, gathering places, gardens, and everyday delight—not only iconic buildings.
5. Everyday dignity: prioritize housing access, universal accessibility, safety, sanitation, clean water, mobility, health, learning, care, and places for informal livelihoods.
6. Repairability and circularity: prefer maintainable systems, local skills, repairable components, appropriate materials, low lifecycle impact, and adaptable buildings.
7. Pluralism: support different household structures, ages, abilities, incomes, lifestyles, languages, and ways of gathering.
8. Evidence discipline: label assumptions, data, simulations, uncertainty, and actual outcomes separately.
9. Non-extractive participation: do not publish private community data or culturally restricted knowledge without consent; establish attribution, benefit-sharing, and withdrawal/review paths.
10. No aesthetic displacement: measure affordability, tenure security, and who benefits. Beautification must not become a pretext for displacement or exclusion.

## Concept portfolio: initial research prompts

These are concept directions for investigation, not definitive depictions of any culture. Names are working titles. Local collaborators must shape and challenge them before they are presented as culturally authentic.

### A. Johannesburg — The Highveld Garden City Rewoven

- Place logic: Highveld grassland ecology, summer storms, winter sun, mining and industrial histories, diverse contemporary communities, and existing neighborhood patterns.
- Spatial idea: linked shaded streets, civic gardens, repaired stream corridors, productive landscapes, mixed-use neighborhood centers, public art, and safe walking/cycling connections.
- Infrastructure questions: water reliability and reuse; stormwater infiltration and flood safety; energy resilience; heat and shade; transit access; mine-land and soil constraints.
- Beauty language: contemporary African-led architecture, durable local craft, deep shade, robust public spaces, indigenous and climate-suitable planting selected with ecologists, and an evolving rather than homogenized cultural identity.
- Validation: choose one bounded area only after source availability, land rights, service definitions, safety, and community partners are understood.

### B. Sahelian Shade-and-Courtyard City

- Research questions: how can courtyards, shaded pedestrian networks, compact settlement patterns, thermal mass, ventilation, and locally available materials inform present-day hot-arid urban design?
- Avoid: copying sacred forms or assuming one architectural vocabulary fits the Sahel's many cultures and climates.
- Test: thermal comfort, water demand, dust and heat resilience, accessibility, fire safety, maintenance, and affordability.

### C. Swahili Coast Ocean-and-Courtyard City

- Research questions: how can coastal trade histories, courtyards, shaded lanes, craft, passive cooling, and public relationships with the sea inform resilient coastal neighborhoods?
- Test: humidity, salt corrosion, sea-level and storm exposure, freshwater security, coastal ecosystems, tourism pressure, and resident tenure.
- Governance: involve local residents and heritage practitioners; protect community access and livelihoods.

### D. Japanese Satoyama-Connected Compact City

- Research questions: how might compact neighborhoods, careful thresholds, small gardens, transit, repair culture, and links between settlement and managed landscapes support contemporary urban life?
- Avoid: treating Japan as culturally uniform or turning everyday practices into superficial aesthetic props.
- Test: earthquake and fire resilience, aging and accessibility, energy, housing affordability, biodiversity, and maintenance.

### E. Mediterranean Water-Wise Civic City

- Research questions: how can shaded streets, courtyards, arcades, compact mixed-use blocks, civic squares, and water-sensitive landscapes adapt to hotter and drier summers?
- Test: extreme heat, drought, wildfire at the urban edge, housing pressure, seasonal population, heritage conservation, and public-space access.
- Do not assume all Mediterranean regions share the same materials, history, or social customs.

### F. Andean Terraced Watershed City

- Research questions: how can steep-terrain settlement, terracing, watershed stewardship, locally appropriate seismic design, and pedestrian connections inform safe contemporary neighborhoods?
- Test: landslides, earthquakes, water storage, slope stability, altitude, biodiversity, and the rights and knowledge of Indigenous communities.
- Use local co-authorship and explicit consent before adapting community-held knowledge.

### G. Amazonian Forest-Edge and River City

- Research questions: how might river mobility, flood-adapted settlement, forest stewardship, and low-impact building support communities without accelerating forest loss?
- Test: flood cycles, humidity, disease ecology, biodiversity, material supply, sanitation, and transport resilience.
- Avoid presenting Indigenous knowledge as freely extractable design content.

### H. South Asian Monsoon Courtyard City

- Research questions: how can shaded public networks, courtyards, water-sensitive streets, mixed-use neighborhoods, and climate-adapted roofs respond to intense seasonal rainfall and heat?
- Test: drainage, flood exposure, humidity, heat, water quality, crowding, access to services, and informal livelihoods.
- Treat South Asia as many distinct regions and cultures, not a single design type.

### I. Nordic Daylight-and-Commoning City

- Research questions: how can durable public interiors, walkable mixed-use districts, daylight-aware buildings, transit, and shared civic amenities support long dark winters and seasonal variation?
- Test: cold, snow and ice, energy demand, daylight, construction cost, loneliness, and inclusive access.
- Do not transplant high-cost systems into regions where simpler locally maintainable alternatives are better.

### J. Desert Night-and-Oasis City

- Research questions: how might compact shade networks, protected courtyards, water stewardship, night-time public life, and high-performance envelopes support safe living in extreme heat?
- Test: water budgets, heat exposure, dust, cooling demand, grid resilience, construction logistics, and ecological limits.
- Any oasis or heritage reference must be locally grounded; avoid a generic luxury-resort aesthetic.

### K. River Delta Amphibious City

- Research questions: how can flood-compatible public space, raised critical infrastructure, wetlands, water transport, and flexible ground floors support delta communities?
- Test: flood frequency and depth, salinity, evacuation, sanitation, structural safety, insurance, livelihood continuity, and managed retreat where necessary.
- Never represent speculative flood protection as guaranteed.

### L. Tropical Canopy-and-Transit City

- Research questions: how can continuous shade, rain protection, natural ventilation, transit-oriented density, and connected urban biodiversity make daily life comfortable in hot-humid climates?
- Test: humidity, heavy rain, windstorms, local public-health considerations, biodiversity, accessibility, and tree maintenance.
- Ensure canopy proposals do not obstruct visibility, utilities, safe movement, or equitable access.

These twelve directions are a starter portfolio, not a complete world survey. The next research pass should add locally authored concepts for more regions, including Central Asia, the Caucasus, the Pacific Islands, the Caribbean, Indigenous North American contexts, and diverse European, African, and Asian urban traditions.

## Common concept schema

Each concept should be a versioned record with the following fields:

- concept_id, title, version, status, authorship and contributors
- geography and scale; urban, peri-urban, rural-urban, or settlement context
- climate/ecoregion profile and data sources
- cultural brief, community-defined values, consultation status, attribution, and knowledge restrictions
- precedents with citations, rights, and a clear distinction between evidence and inspiration
- spatial grammar: street networks, blocks, thresholds, public/private gradients, density, public spaces, and landscape structure
- architecture and material strategy, including local availability, maintenance, embodied impacts, and adaptability
- infrastructure strategy: water, sanitation, energy, food, waste, mobility, communications, and emergency response
- ecological strategy: watersheds, habitat, soil, canopy, biodiversity, and restoration
- housing, affordability, tenure, accessibility, safety, livelihoods, and inclusion
- climate and hazard assumptions; constraints and failure modes
- design representations: brief, diagrams, map layers, reference images with provenance/rights, model files, and versioned renders
- scenario IDs, lifecycle cost ranges, uncertainty, sensitivity, and trade-offs
- community review and dissent; changes requested and responses
- validation status: concept, researched, simulated, community_reviewed, pilot_tested, or measured_outcomes
- run manifests and links to observations, scenario results, decisions, and outcomes

Do not let status advance automatically because a rendering looks convincing or a test suite passes.

## Integration across the four projects

### Sol-Atlas
Owns place, geography, spatial layers, climate/ecoregion overlays, site constraints, and comparison of alternative spatial arrangements. Display data lineage and missing coverage. Keep concept geography distinct from precise sensitive cultural locations.

### Symthaea
Helps generate and critique alternative layouts, formulate hypotheses, compare trade-offs, identify missing evidence, and explain why a design might succeed or fail. Require bounded objectives and constraints. Do not let one scalar beauty or optimality score erase plural community values.

### Symtropy
Hosts interactive spatial experiences and appropriate simulations: sun/shade, pedestrian movement, public-space use hypotheses, resource flow, or other domains only when the implemented model is credible. Make assumptions, model limits, and stochasticity inspectable.

### Mycelix
Records source provenance, contributor attribution, permission and license terms, community review, proposals, disagreements, decisions, and later outcome evidence. Keep private and restricted knowledge off public ledgers; use access-controlled references and revocable permissions where feasible.

### Shared integration
Use versioned JSON fixtures and small Rust adapter types before choosing a transport. A common schema should coordinate the projects without making each project depend on a single monolithic shared crate.

## Evaluation framework

Avoid one global score that ranks cultures. Evaluate each concept across a transparent dashboard, with thresholds and trade-offs:

- Basic needs: access to water, sanitation, housing, energy, health, education, and mobility.
- Thermal and climatic comfort: shade, indoor comfort, seasonal usability, heat risk, and passive-design contribution.
- Ecology: habitat continuity, water balance, soil, tree survival, biodiversity, and pollution.
- Beauty and meaning: locally defined criteria, craft quality, human scale, sensory experience, and cultural continuity.
- Social inclusion: affordability, universal access, tenure, gender and age safety, informal livelihoods, and distribution of benefits.
- Resilience: redundancy, hazards, repair time, supply dependencies, and service continuity.
- Feasibility: lifecycle costs, local skills, supply chains, approvals, maintenance burden, and implementation phasing.
- Evidence quality: source quality, coverage, uncertainty, reproducibility, validation method, and conflicts of interest.

Use a Pareto comparison and documented community priorities rather than pretending every value can be reduced to a universal weighted score. Report who benefits, who bears costs, and whose knowledge or preferences are absent.

## First executable milestone: Concept Atlas vertical slice

1. Add a versioned schema and three valid fixtures: Johannesburg, one hot-arid context, and one flood/coastal context. Include no sensitive personal data.
2. Add invalid fixtures for missing climate/source provenance, undocumented cultural review status, and missing rights for reference assets.
3. Render the three concepts as spatially distinct cards/layers in Sol-Atlas or the existing concept explorer, with explicit concept status and visible evidence gaps.
4. Let Symthaea compare two bounded alternatives for one shared objective (for example, shade access) while showing trade-offs and unknowns.
5. Use Symtropy only if a minimal simulation can be validated against a suitable reference or clearly marked synthetic fixture.
6. Store source, authorship, permissions, review, and version links through Mycelix records.
7. Produce an independent manifest with exact revisions, input hashes, licenses, test results, and output hashes.


### Fixture manifest and validator semantics

The fixture inventory is machine-readable at `fixtures/living-city/manifest.v0.1.json`. It declares the expected result for each positive and negative fixture and the specific missing-field defect each negative control is meant to exercise. A future test must check that a negative fixture fails for its intended reason, not merely for any error.

JSON Schema Draft 2020-12 treats `format` as annotation by default unless format assertion is explicitly enabled. Therefore a validator must not silently assume that `format: uri` proves source URLs are syntactically valid; enable the relevant format-assertion behavior or add an explicit application-level URI check. Even then, syntactic URI validity does not prove that a source exists, is trustworthy, or grants reuse rights.

## Acceptance criteria

- [ ] A concept schema is versioned and validated.
- [ ] At least three distinct concepts have sourced briefs, explicit status, and documented assumptions.
- [ ] Each concept distinguishes local facts, sourced precedents, design hypotheses, and speculative elements.
- [ ] No cultural group is treated as homogeneous or represented as having approved a concept without evidence.
- [ ] Reference assets have usable rights/permissions and attribution; restricted knowledge is not exposed.
- [ ] Spatial views reveal climate, terrain, hazards, and data gaps without presenting missing data as safe conditions.
- [ ] Comparisons show separate metrics and trade-offs; no opaque universal beauty/culture score is produced.
- [ ] A deliberately missing provenance/rights field causes validation to fail.
- [ ] A no-op design change cannot claim a measured benefit.
- [ ] Every simulation result links to a versioned scenario, model, and input manifest.
- [ ] Community review status is evidence-backed and remains separate from technical test status.
- [ ] Pilot readiness is not claimed until a locally grounded partner, legal/rights review, implementation path, maintenance plan, and evaluation method exist.

## Decision requested from reviewers

1. Is the concept schema small enough to implement without freezing future design possibilities?
2. Which initial three concept briefs should be researched and co-authored first?
3. What is the simplest existing Sol-Atlas surface for showing concept layers without duplicating the UI?
4. Which Mycelix evidence/permission primitives can be reused instead of creating parallel governance records?

The atlas should make it easier for every community to imagine a future that feels like home to them, while learning from the world without flattening its differences.
