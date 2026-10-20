# Furnishing catalog interoperability: Symtropy ↔ Mycelix Marketplace v0.1

**Status:** research/integration contract plus synthetic fixtures; no live integration or commerce execution is claimed.  
**Mycelix Marketplace implementation:** the active monorepo path is `mycelix-marketplace/`. The separate `Luminous-Dynamics/Mycelix-Marketplace` repository is archived; do not start new work there.  
**Symtropy counterpart:** [furnishing-layout benchmark PR #2223](https://github.com/Luminous-Dynamics/symtropy/pull/2223) and [SPATIAL-DESIGN-001 #1766](https://github.com/Luminous-Dynamics/symtropy/issues/1766).  
**Related existing Mycelix work:** [circular-marketplace zome](../../mycelix-commons/zomes/circular-marketplace/), [construction provenance and approval receipts #5149](https://github.com/Luminous-Dynamics/mycelix/issues/5149), and [Marketplace guarded contract/Leptos migration](../README.md).

## Recommendation

**Yes: connect interior co-design to Mycelix Marketplace.** The valuable product is not a "buy this chair" button bolted onto a renderer. It is a consentful, evidence-aware bridge that compares a proposed layout against real offers and lifecycle options—new, used, repaired, refurbished, remanufactured, local/artisan, and reclaimed—before a resident decides what to acquire.

Make the design-to-commerce loop reversible:

1. Residents define tasks, non-negotiable needs, budget, locality and preferences in a local/private design context.
2. Symtropy computes geometry and declared hard-constraint results. Symthaea may propose alternative layouts and explain trade-offs; neither treats missing inputs as a pass.
3. A narrow catalogue adapter queries Mycelix for existing offer references and reads their versioned terms. It does not mint listings or invent stock.
4. A separate compatibility report compares the exact item/profile revision with the exact design revision, including geometry-use rights, evidence state and any unknown fit envelopes.
5. Residents compare Pareto alternatives: spatial fit, installed cost, availability/delivery, maintainability, repair/reuse options, declared aesthetics and other supported dimensions. No universal score quietly overrides their priorities.
6. Only after explicit resident selection does the existing Marketplace transaction lifecycle begin from the authoritative listing and exact terms. Exploration is **effects-disabled**: no checkout, reservation, token movement, public sharing of a floor plan, or vendor contact occurs just because the solver explored an alternative.
7. Delivery, installation, inspection, maintenance and eventual repair/reuse can later append provenance-linked lifecycle events, each with its own author, scope, rights and evidence status.

## Current implementation reuse and boundary

The current Mycelix Marketplace README describes a pre-alpha Holochain marketplace with a guarded Leptos path for listing discovery/detail, authoritative purchase creation, transaction recovery, lifecycle updates, arbitration and settlement projections. Those contracts should remain the authority for listings and transactions.

The active Commons `circular-marketplace` zome currently represents **secondary-material** listings/orders—such as recycled plastic, metal, glass, paper, compost, and similar material streams. That is useful for reclaimed-material supply, workshops and fabricators. Its material-centric entry type is not a drop-in schema for completed furniture. Do not force chairs, sofas, one-off cabinetry or used furniture into a material listing just because reuse is desired.

This v0.1 is therefore an adapter/profile proposal, not a new Marketplace zome:
- **Marketplace owns:** listing identity/revision, seller, current commercial terms, availability claims, authorized transaction lifecycle, dispute process and fulfillment records.
- **Furniture catalog profile owns:** normalized item descriptors needed for discovery—category, instance/model identity, dimensions, task/access envelopes, condition, material and repair claims, geometry rights and evidence links. It references the listing rather than duplicating its commercial authority.
- **Symtropy owns:** exact design revision/geometry, spatial constraints, geometry intersection, scenario execution and geometric compatibility diagnostics.
- **Symthaea owns:** reasoning over resident-provided tasks/preferences, generating alternatives, asking useful clarifying questions and communicating uncertainty; it must not certify physical properties or infer preferences from sensitive demographics.
- **Mycelix consent/provenance capabilities own:** authorization, who contributed a claim/approval, selective disclosure and event lineage, once the relevant adapters are reviewed.
- **Sol-Atlas may later contribute:** location/climate/orientation context only through qualified source adapters. Do not scrape or silently assume live site data.

## Versioned listing-revision receipt

The additive [Furnishing Catalog Revision Profile v0.2](FURNISHING_CATALOG_REVISION_PROFILE_V0_2.md) extends this v0.1 contract without changing it. v0.2 carries a stable listing root and the exact locally observed head set. A concurrent update graph has no selected action and must leave price, availability, quantity and delivery unknown; coverage is explicitly local-only, not global DHT completeness. Its schema and Rust auditor tests are developed on a stacked draft branch.

## Profile and synthetic fixture

Files introduced with this branch:

- `contracts/furnishing-catalog-profile-v0.1.schema.json`
- `contracts/fixtures/furnishing-catalog-profile-v0.1.json`

The profile separates:
- item identity from the marketplace listing/transaction;
- product dimensions from its approach, swing, reach, transfer and service envelopes;
- declared versus independently checked claims;
- geometry digest/usage license from permission to advertise a listing;
- the catalog's geometry-use-license enum from separate, referenced `geometry_use_rights` evidence—the audit rejects an allowed enum that contradicts the scoped rights record;
- seller assertions from buyer observations, manufacturer declarations, and third-party evidence;
- catalogue evidence from its validity/currentness and the exact scope of the claim it supports;
- resident preferences from physical constraints and from model-estimated preference fit.

The fixture is conspicuously synthetic. It includes (1) a refurbished, modular desk with a measured footprint but unverified seller claims about materials/repair, (2) a dimensioned manufacturer's desk whose geometry licence does not grant layout-use redistribution and whose task envelope is unknown, and (3) a used cabinet whose dimensions, price, availability and delivery are unknown. These are test cases, **not real products or marketplace offers**. Synthetic hashes and prices must never be displayed as live evidence.

Unknown geometry, missing permission, stale listing terms, or a missing access envelope must produce `indeterminate`/blocked for the affected claim—not a guessed pass. A geometry-use right is separate from the right to display a product image or the right to republish its mesh/model.

## Paradigm shifts

### 1. From room shopping to requirement-aware procurement

Search should begin with tasks and constraints, then discover products that satisfy them. For example, a "desk" candidate must have the needed footprint, approach zone and task envelope, with evidence that permits geometry use. Category labels and a product photo cannot prove fit.

### 2. From product pages to product evidence

A listing should link back to claims such as model/instance identity, dimension source and tolerance, material composition, assembly/disassembly instructions, replaceable parts, warranty, delivery/installation quote and environmental declarations where available. Each evidence item needs source, digest, issuer/claimant, scope, time/currentness state and rights. Cryptographic integrity alone does not establish issuer trust or truth.

### 3. From linear retail to a circular decision ladder

For comparable candidates, allow residents to consider what already exists before buying new: re-layout/reuse, repair, refurbish, remanufacture, reclaimed component/material, then new production. Do not automatically privilege reused items if condition, safety scope, delivery distance, missing dimensions or repair evidence make the choice unsuitable. Compare full delivered/installed and lifecycle costs only when those costs are sourced; otherwise mark unknown.

### 4. From algorithmic ranking to resident-controlled trade-offs

Apply hard constraints first. Then preserve a set of Pareto-efficient alternatives across declared dimensions such as cost, lead time, local production, repairability, reuse, aesthetics, material evidence and maintenance access. Keep alternatives incomparable when an objective is missing or preferences conflict. Never translate one seller's rating/reputation into proof of product safety or geometric fit.

### 5. From platform lock-in to open, exportable interoperability

Build a narrow mapping that can ingest/export an IFC furniture reference or open geometry asset when rights permit; do not invent a parallel full building model. IFC 4.3 defines `IfcFurniture` and associated property sets, while GS1 Digital Link illustrates how recognized identifiers can link a physical item to digital product information. Neither identifier nor IFC interoperability proves current stock, dimensions in the delivered instance, durability or compliance.

## Hard privacy and authority boundary

Full room geometry, occupant/access requirements, exact residential location, budget history and household disagreements are sensitive. The default is local computation/private state. Queries to Mycelix should reveal the minimum necessary fields; public DHT entries should not contain a user's floor plan or inferred personal/access profile. Coarse region is preferable until delivery quotes require a more precise address.

A listing or "verified product" badge must not imply:
- that the item is in stock right now;
- that a quoted price includes shipping, duties, installation or taxes;
- that dimension claims are measured or use of geometry is licensed;
- that a reused/recycled-material assertion is third-party verified;
- that a room is accessible, fire-safe, structurally adequate, code-compliant or otherwise professionally approved;
- that the resident has authorized a purchase;
- that an unsigned/uncorroborated provenance event is true.

The adapter must use the existing marketplace's actual authenticated transport and contracts, and must fail closed when that role is deferred, unavailable or only represented by development fixtures.

## Benchmark protocol: furnishing-to-offer matching v0.1

All targets below are **proposed gates**, not achieved measurements.

### A. Synthetic oracle gates

- 100% of profile/listing/design references are syntactically valid and resolve within the fixture corpus.
- 100% of placements use a permitted geometry representation and unit; unsupported geometry rights yield indeterminate rather than copied/redistributed geometry.
- 100% of seeded unknown/stale/revoked/conflicting states remain visible and cannot be silently promoted to current, verified or compatible.
- Zero false-safe compatibility decisions in the frozen synthetic hard-constraint corpus: a seeded missing dimension, blocked approach envelope, wrong unit, mismatched geometry digest, stale revision or non-permitted geometry-use license must not pass the affected claim.
- 100% of purchase previews (once added) bind to the exact current listing action/revision, listing terms digest, buyer authorization and design decision; any changed terms require re-confirmation.
- Exploration must produce zero side effects: no transaction, reservation, payment, direct seller message or public design disclosure.
- 100% of seeded supplier substitutions invalidate/recompute only explicitly dependent fit/material/cost results; unrelated source evidence is retained.
- A malformed or incomplete fixture, missing required tool/model, empty result set or truncated search yields explicit failure/incomplete status, not a zero-cost or exhaustive-optimality claim.

### B. Comparative industry evaluation

Compare the integrated workflow against separate, equivalent-input baselines:
1. manual designer + spreadsheet/product-page workflow;
2. retailer/marketplace category filters and ordinary product comparison;
3. an openBIM/CAD workflow with the same IFC/geometry inputs and declared model capabilities;
4. a circular-material/reuse search without layout-aware constraints;
5. the integrated Symtropy/Symthaea → Mycelix profile/query pipeline.

Use the same frozen candidate corpus, constraints, region, availability snapshot and user tasks wherever possible. Report by scenario and cohort:
- proportion of candidates with complete, scoped dimensions/access envelopes and source rights;
- false-compatible and false-incompatible rate against the synthetic oracle;
- unknown and stale-data detection;
- feasible-alternative recall and Pareto coverage on small enumerable cases;
- user time to a valid shortlist, correction count and reasons for rejection;
- full delivered/installed cost and lead time only where quotes are source-backed;
- repair-part/instruction availability and provenance completeness;
- reused/refurbished candidates surfaced before new items when eligible—not a raw reuse percentage that rewards unsuitable goods;
- export/import loss for geometry and lifecycle claims;
- response latency, memory, search truncation and network-partition behavior.

Do not claim "better than SOTA" from a more attractive render or the number of listed products. The differentiator to prove is **fewer false fits, clearer unknowns, better multi-objective choices and lower end-to-end effort with no increase in unsafe or unauthorized decisions**.

### Named industry baselines to include

This is a capability decomposition, not a claim that the products below are interchangeable or deficient:

- **Planner 5D AI Designer / AI Furniture Placement:** its current official material describes automated furnishing or rearrangement, photo/floor-plan inputs, and interactive 2D/3D visualization. Treat it as a strong visual/layout workflow baseline. Run identical room inputs and tasks; independently score the resulting geometry and requirements rather than scoring screenshots alone. Product features cited by the vendor are not an independent correctness evaluation: https://support.planner5d.com/en/articles/8540993-ai-designer-furnish-or-style-your-room-with-our-ai-designer-feature-ios and https://ai.planner5d.com/ai-furniture-placement
- **IKEA Kreativ:** the official UK guide describes whole-room scanning on LiDAR-enabled devices and room design continued across phone/web. Treat it as a retailer-integrated capture and product-visualization baseline, with device/region constraints explicitly recorded: https://www.ikea.com/gb/en/customer-service/knowledge/articles/250fg37g-2498-40gg-916c-f27d354683f0
- **BIMobject furniture catalog:** offers manufacturer-specific furniture BIM objects and filters such as dimensions, brands, certifications and sustainability information. Treat it as a product-data/specification discovery baseline—not a neutral, multi-vendor checkout or proof that a selected item fits this exact design: https://www.bimobject.com/en/categories/furniture
- **IFC / openBIM workflow:** use the same furniture geometry and supported property sets through an IFC 4.3 exchange path where possible. Report round-trip information loss and model-view coverage rather than assuming every tool implements all IFC concepts: https://technical.buildingsmart.org/standards/ifc/ and https://standards.buildingsmart.org/IFC/RELEASE/IFC4_3/HTML/lexical/IfcFurniture.htm
- **Ordinary marketplace + manual design workflow:** use the same listing corpus with its available filters, plus a human designer/spreadsheet to do the fit checks. This baseline is essential: the combined system should save real work and reduce mistakes, not merely add provenance fields.

The differentiated hypothesis for Symtropy + Symthaea + Mycelix is the **joined decision loop**: resident-authored functional constraints → geometry/unknown checks → rights-aware product offers → repair/reuse/new alternatives → visible multi-objective trade-offs → resident-authorized marketplace handoff → lifecycle traceability. None of the cited products is assumed to lack all these capabilities; the claim to test is whether the composed workflow measurably improves end-to-end outcomes on equivalent tasks and data.

### C. Scaling tiers

Start with 25, 100 and 1,000 synthetic offers; then 10k only once pagination, memory, provenance resolution and candidate truncation are measurable. For each tier freeze seed, corpus digest, design manifest, database/DHT snapshot, hardware, query semantics, and software revision. Report p50/p95 latency and peak memory alongside completeness and correctness; speed alone never excuses lost candidates or weaker gates.

### D. Human study

Before real-world superiority claims, run paid, consent-based tasks with distinct household/access needs, local makers, repairers and design professionals. Pre-register success definitions and report cohort-specific results, disagreement, uncertainty comprehension, and whether users can explain why candidates were excluded. Synthetic fixtures cannot establish accessibility benefit, cultural fit or occupant satisfaction.

## Research anchors

- buildingSMART, *IfcFurniture*, IFC 4.3.2.0: https://standards.buildingsmart.org/IFC/RELEASE/IFC4_3/HTML/lexical/IfcFurniture.htm
- buildingSMART, *Industry Foundation Classes*: vendor-neutral, lifecycle-oriented exchange model, ISO 16739-1:2024: https://technical.buildingsmart.org/standards/ifc/
- GS1, *GS1 Digital Link*: identifiers can link barcodes/IDs to product, certification, traceability and use information; the identifier itself is not certification: https://www.gs1.org/standards/gs1-digital-link
- NIST, *Building Digitization and Semantic Interoperability*: machine-readable, interoperable building data can reduce manual mapping across design/operation systems; source and domain semantics still matter: https://www.nist.gov/programs-projects/building-digitization-and-semantic-interoperability
- European Commission, *Ecodesign for Sustainable Products Regulation*: framework aimed at longer-lasting, repairable/recyclable, resource-efficient products; product-specific duties and schedules must be checked for each market rather than assumed globally: https://environment.ec.europa.eu/news/sustainable-products-be-norm-consumers-new-regulation-2024-07-19_en
- Wilson et al. (2025), *Trustworthy Second-hand Marketplace for Built Environment*: research proposal on traceability and reuse marketplaces; it is a comparator, not proof that any specific implementation has commercial or sustainability outcomes: https://arxiv.org/abs/2509.04085

## Implementation sequence

1. Agree this profile with the Marketplace domain owner and inspect the actual current wire contracts; do not add a second listing/order schema to live zomes before the gap audit.
2. Add a small Rust-first independent fixture validator and mutation tests for stale offers, rights, digest mismatches, unit confusion, false availability and unauthorized effects.
3. Add an effects-disabled adapter from Symtropy's versioned design/requirement snapshot to existing Marketplace listing reads. Any adapter must pass the Marketplace's fail-closed live/fixture gates.
4. Add client-side shortlist and explanation panels that display provenance/currentness and explicit exclusions; keep buyer layout and resident preferences private by default.
5. Only after the read path and independent checks pass, design a resident-confirmed handoff into the existing authoritative transaction lifecycle.
6. Later, link post-purchase lifecycle events—delivery, installation, repairs, component substitution and resale/reuse—back to the item identity without turning a marketplace claim into engineering certification.

## Current limitations

This branch is a contract proposal with synthetic fixture data. Authoring-time JSON parsing and a structural/reference mirror are not formal JSON Schema validation, Rust tests, Holochain conductor tests, privacy review, rights review, or live marketplace evidence. No live query, product listing, stock/price check, purchase, ranking, repairability certification or environmental calculation was performed.

## Independent Rust audit slice

A standalone Rust audit utility now lives at `tools/furnishing-profile-audit/` with two checks:

- **Profile semantics:** required identifiers and evidence references, unique IDs, geometry dimensions/digests, claim/evidence consistency, rights-aware compatibility, and the prohibition on treating fixture-only price/availability/delivery as live confirmed facts.
- **Discovery-request policy:** verifies that design discovery stays in `discovery_only`, all commercial effects remain false, only allowlisted query fields are requested, detailed room geometry/address/occupant profile and correlatable design/query digests remain local, and purchase authorization is not requested during exploration.
- **Formal shape validation:** uses the pinned Rust `jsonschema` dependency to validate every profile against the Draft 2020-12 profile schema and the request fixture against the Draft 2020-12 request schema, with format validation enabled.
- **Mutation tests:** seed changed geometry digests, unknown dimensions labeled compatible, geometry without layout-use rights promoted to compatible, confirmed stock/quotes from fixtures, missing evidence, contradictory hard-constraint results, unauthorized reservation/order, seller contact, floor-plan/address transmission, unapproved disclosure fields and malformed digests.

Run from the repository root:

```sh
cargo fmt --manifest-path mycelix-marketplace/tools/furnishing-profile-audit/Cargo.toml -- --check
cargo test --locked --manifest-path mycelix-marketplace/tools/furnishing-profile-audit/Cargo.toml
cargo clippy --locked --manifest-path mycelix-marketplace/tools/furnishing-profile-audit/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path mycelix-marketplace/tools/furnishing-profile-audit/Cargo.toml -- mycelix-marketplace/contracts/fixtures/furnishing-adapter-request-v0.1.json
```

The audit is deliberately narrow; it does not authenticate Mycelix, prove seller claims, or independently verify source digests against any external source bytes. A format-valid digest is not proof of artifact integrity unless the verifier recomputes it over the actual artifact. It is also not a marketplace production integration. Exact-head formatting/tests/Clippy must pass on the pinned head before treating this verifier as qualified. The standalone crate now includes a committed `Cargo.lock`; the exact-head workflow uses `--locked` for tests and Clippy to verify the pinned dependency closure.
