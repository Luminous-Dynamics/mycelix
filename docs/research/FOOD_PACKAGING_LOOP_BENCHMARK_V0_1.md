# Food Packaging Loop and Format Benchmark V0.1

**Status:** research design / unqualified  
**Integration issue:** https://github.com/Luminous-Dynamics/mycelix/issues/6294  
**Parent food cooperative:** https://github.com/Luminous-Dynamics/mycelix/issues/5001  
**Revision date:** 2026-11-16

This document specifies an evidence and comparison contract for food packaging formats. It is not a declaration that reusable packaging is always better, a food-safety protocol, or a product passport implementation.

## 1. Thesis: the package is part of the food system

The object to optimize is not grams of wrapper per retail unit. It is the delivery of an **accepted quantity of edible food in a required condition**, including food damage, spoilage and edible loss; packaging production and end of life; transport, collection and empty-return logistics; washing, sanitation, energy, water and labor; access, affordability, convenience and actual consumer participation.

A packaging change can reduce material while increasing food waste, or avoid food damage while adding more material. The entire food + package system must be measured.

## 2. Format families

| Format | Potential value | Risks / required evidence |
|---|---|---|
| Right-sized portions | Lower leftover ingredient loss; accessible quantities | More units/closures/material; cost per serving |
| Bulk + refill | Less repeated primary packaging for stable products | Contamination, dispensing losses, stock rotation and return friction |
| Reusable returnable | Multiple trips on suitable dense routes | Return rate, washing, transport, damage, attrition and reverse logistics |
| Pack-to-store / pack-to-prep | One cleanable container may serve several stages | Food-contact scope, temperature rating, allergens, residue and cleaning |
| Loose / minimal | Avoids unnecessary pack mass | Handling, damage, hygiene, weighing and retail infrastructure |
| Single-use protective | Can prevent food damage/contamination or support remote logistics | Material use and end-of-life burden |

Do not force one shape, material or reuse model across all food types. Dry grains, flour, spices, fresh produce, dairy, meat, frozen meals and ready-to-eat food have different storage and contact requirements.

## 3. Initial evidence schemas

These are proposed domain contracts, not existing deployed schemas. Reuse canonical Mycelix identity, product/lot and repair references.

### 3.1 PackDesignV1

- design identity, revision, manufacturer reference and rights reference;
- format class: single-use, refill, reusable-return, bulk, loose or hybrid;
- intended product categories and direct/indirect contact status;
- material identity, grade, composition and food-contact evidence reference;
- nominal/usable capacity, unit, tare mass, dimensions and nesting/stacking properties;
- closure/seal and any measured oxygen, moisture, light or temperature barrier properties;
- approved storage/use temperature range; pressure-use defaults to false unless separately qualified;
- cleaning, inspection, repair, end-of-life and retirement procedure references;
- declared unknowns and scope/expiry of each supporting claim.

### 3.2 ContainerUnitV1

Use serial identity only where per-container tracking creates real value; otherwise use a controlled manufacturing lot to avoid unnecessary personal tracking.

- container or batch identity and exact PackDesignV1 revision;
- manufacturer lot, ownership/custody state and lifecycle status;
- completed service count and last inspection/clean/release records;
- damage, leakage, staining, crack, closure wear, contamination and exception flags;
- repair/replacement parts and post-repair inspection evidence;
- optional deposit/return settlement reference, disclosed only to authorized parties.

### 3.3 PackCycleEvidenceV1

- pack/container ID and food product-lot ID;
- event kind: manufacture, fill, seal, dispatch, transfer, receipt, open, refill, return, inspect, wash, release, repair, retire or dispose;
- event timestamp, actor/equipment reference, location precision policy and event ordering;
- food mass and unit; tare/gross mass when actually measured;
- measured storage/transport temperature and missing intervals where relevant;
- cleaning method, cycle parameters, inspection outcome and release authority;
- instrument/method, calibration, uncertainty, source digest and evidence class;
- schema revision, rights, retention and disclosure policy;
- append-only correction/supersession link.

Keep measured, modelled, simulated, declared and independently attested evidence distinct. A digital record can preserve provenance and correction history; it cannot prove that an inspection, sensor or laboratory method was correct.

## 4. Lifecycle functional unit and comparison

Primary functional unit:

**1 kg edible food delivered to the point of consumption in the required condition.**

Also report **one accepted edible serving** for consumer-use comparisons. Do not compare one disposable tub with one reusable jar without normalizing for delivered food mass, losses and completed service.

For each food category and route, compare the appropriate alternatives: current packaging; right-sized/portion; refill/bulk; reusable-return; loose/minimal where suitable; and protective single-use if it avoids a measurable food-loss risk.

Freeze the product, starting food condition, packaging mass/material, fill ratio, transport mode/distance, ambient/storage conditions, wash system, cleaning chemistry, return distance, reuse frequency and end-of-life route. Include packaging mass/material per functional unit; edible food loss/damage/spoilage; contamination/hygiene failures and rewash/reject rates; wash water, electricity, heat, detergent, labor and wastewater; actual return rate, loss/breakage/attrition, dwell time and collection distance; completed uses before retirement; transport/storage density and empty-return volume; lifecycle cost/impact with boundaries, allocation, uncertainty and sensitivity; accessibility, convenience, return friction, affordability and producer/retailer economics.

**Reuse break-even is scenario-specific.** One 2024 LCA of reusable plastic takeaway containers found lower global-warming impact after six uses for its studied system, while identifying customer transport and washing-facility efficiency as consequential. A separate interdisciplinary 2024 study linked environmental break-even to material, washing, return rates and willingness to reuse. Six uses is not a universal design rule.

## 5. Hygiene, food contact and user interface

- Exact material/design revision must be scoped to intended food, contact, temperature, cleaning chemistry and reuse cycle.
- Food-contact suitability, migration and hygiene are separate from material recyclability.
- A container with a crack, damaged seal, persistent residue, unknown wash state or unqualified repair must not be silently returned to service.
- Allergen/cross-contact risks require explicit process boundaries; visually clean is not equivalent to verified cleaning.
- Pack-to-prep is not pack-to-cook. No heated, oven, microwave, boiling or pressure use unless exact design evidence supports it.
- Provide easy inspection, fully accessible cleaning surfaces and seal/closure replacement without destructive disassembly where feasible.
- R638 and other applicable South African rules require product- and process-specific review. A QR code or provenance claim does not make a package hygienic or food-safe.

## 6. Return-loop and refill economics

A reuse loop must measure both container and system:
1. Distribution of clean containers.
2. Fill/dispatch and customer handoff.
3. Actual return and time out of circulation.
4. Collection distance and route density.
5. Inspection, washing and drying.
6. Release, refill and next use.
7. Damage, repair, loss and retirement.

Calculate the minimum use count and return rate needed for the declared environmental and economic outcomes. Then test whether actual observed behavior exceeds the break-even threshold with an uncertainty margin. Include washing water/energy, transport, reverse logistics, container attrition and user behavior. A design-life promise does not equal actual reuse.

## 7. Initial pilot design

Start with a bounded Johannesburg/Highveld cooperative-to-household route using the producer, lot, offer, pickup and fulfillment model already described by MYC-FOOD-001. Select one dry staple and one fresh product only after local availability, handling conditions and participating operators are confirmed.

Suggested design:
- Baseline: existing locally used single-use or bulk distribution format.
- Variant A: suitable right-sized or refill format.
- Variant B: returnable crate/container with a defined collection and washing route.
- Where product protection requires it, retain a suitable single-use control instead of forcing reuse.

The local pilot records actual route distance, return rate, container dwell time, wash energy/water, product loss, rejection, affordability and user effort. Sol Atlas supplies regional context only where source resolution and evidence support it; do not infer farm-scale suitability or customer demand from a map.

## 8. Adversarial and negative controls

- Return rate below environmental break-even.
- Long empty-return trips; washing energy/water omitted.
- Container lost, cracked, scratched, stained or used beyond qualified cycle count.
- Incorrect pack revision attached to a product lot.
- Lot split/blend without parent-child lineage.
- Stale or missing temperature history interpreted as continuous compliance.
- Unknown cleaning state interpreted as clean.
- QR copied onto another pack; revoked/expired material or lab evidence accepted.
- Bulk dispensing spills or contaminates more food than it saves in packaging.
- Portion packs reduce food loss but increase material/cost per serving.
- Simulated/forecast loss reduction counted as measured household waste prevention.
- Household identity or detailed purchase history disclosed unnecessarily.

Retain every failure, missing record, lost unit, unreturned unit, rejected batch and rewash in denominators.

## 9. Project boundaries

- **Mycelix:** product/lot references, custody and correction history, supplier/producer relationships, container events and selective disclosure. Do not add a competing marketplace or generic provenance ledger.
- **Symtropy:** physical return/wash/transport resource model and kitchen/storage interface trade-offs; simulation is not observed lifecycle data.
- **Symthaea:** forecast or recommend quantity/portion options when evidence exists; do not create stock or safety claims from predictions.
- **Sol Atlas:** regional supply, source distance, climate, water, energy and recovery-service context with source/time/space/rights metadata.

## 10. Exit criteria

1. PackDesignV1, ContainerUnitV1 and PackCycleEvidenceV1 schemas and deterministic test fixtures.
2. Compatibility adapters to existing MYC-FOOD-001 product/lot/fulfillment entities.
3. One matched dry-product trial and one separate fresh-product trial plan.
4. Lifecycle break-even model with explicit boundaries and sensitivity analysis.
5. Actual return/wash/transport collection plan.
6. Food-contact, cleaning, allergen and jurisdictional review.
7. Stop/continue/pivot decision based on food loss, packaging impact, affordability, hygiene, user adoption and return economics.

No single packaging type is the default winner. The correct outcome may differ for a dry staple, a short-lived fresh food, an urban takeaway meal and a long-distance supply route.

## 11. Research references

- UNEP, [Food Waste Index Report 2024 — key messages](https://wedocs.unep.org/bitstream/handle/20.500.11822/45275/Food-Waste-Index-2024-key-messages.pdf). 2022 estimate: 1.05 billion tonnes of consumer-level food waste; household sources account for 631 million tonnes.
- Yadav et al., [Life cycle assessment of reusable plastic food packaging](https://doi.org/10.1016/j.jclepro.2024.141529), 2024.
- [Integrating behavioural, material and environmental science to inform the design and evaluation of a reuse system for takeaway food](https://doi.org/10.1016/j.resconrec.2024.107815), 2024.
- European Commission, [Packaging and Packaging Waste Regulation](https://environment.ec.europa.eu/topics/waste-and-recycling/packaging-waste/packaging-packaging-waste-regulation_en) and [application from 12 August 2026](https://environment.ec.europa.eu/news/new-eu-rules-packaging-enter-application-2026-08-11_en).
- South African Department of Health, [Food Control regulations](https://www.health.gov.za/food-control-regulations/); R638 official text: [General hygiene requirements](https://ugu.gov.za/wp-content/uploads/2024/07/Regulation-R638-Regulations-governing-general-hygiene-requirements-for-food-premises.pdf).
- Mycelix, [MYC-FOOD-001 #5001](https://github.com/Luminous-Dynamics/mycelix/issues/5001).
- Symthaea, [FOODSTEWARD-001 #10325](https://github.com/Luminous-Dynamics/symthaea/issues/10325).
- Sol Atlas, [AGRO-CLIMATE-001 #107](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/107).

## 12. Nonclaims

This document does not establish food safety, shelf-life extension, reduced food waste, improved consumer acceptance, reuse adoption, lower lifecycle impact, cost savings or legal compliance. Such claims require matched physical evidence and appropriate independent review.
