# Furnishing Discovery Evaluation v0.1

**Status:** benchmark protocol proposal plus a fail-closed typed read-only projection; not yet a live Marketplace adapter qualification.  
**Related work:** [Mycelix furnishing profile PR #5251](https://github.com/Luminous-Dynamics/mycelix/pull/5251), [adapter task #5270](https://github.com/Luminous-Dynamics/mycelix/issues/5270), and [Symtropy furnishing-layout benchmark PR #2223](https://github.com/Luminous-Dynamics/symtropy/pull/2223).

## Research question

Can a household reach a better **verified shortlist**—not merely a better-looking render—while maintaining correct uncertainty, currentness, evidence rights, resident control, and no unintended commerce or privacy effects?

The initial read-only projection is intentionally conservative. The existing typed Listing payload contains title, description, integer-cent price, broad category, photo CIDs, quantity, status and an epistemic classification. It does not itself carry typed furniture dimensions, permitted geometry-use rights, a current-revision receipt, pagination/completeness receipt, or fixture-versus-live transport provenance. The adapter therefore marks those concerns unresolved rather than deriving them from prose or a successful RPC.

## Research and standards anchors

These are interoperability references, not a claim of conformance:

- [IFC 4.3 IfcFurniture](https://standards.buildingsmart.org/IFC/RELEASE/IFC4_3/HTML/lexical/IfcFurniture.htm) and [Pset_FurnitureTypeCommon](https://standards.buildingsmart.org/IFC/RELEASE/IFC4_3/HTML/lexical/Pset_FurnitureTypeCommon.htm): align future item identity and declared dimensions to established built-asset concepts.
- [buildingSMART Information Delivery Specification (IDS)](https://technical.buildingsmart.org/projects/information-delivery-specification-ids/): define machine-checkable exchange requirements for units, required properties and values.
- [GS1 Digital Link](https://ref.gs1.org/standards/digital-link/): consider standardized product identifiers and links to distinct product information sources; it is not a truth oracle.
- [Regulation (EU) 2024/1781](https://eur-lex.europa.eu/eli/reg/2024/1781/oj/eng): design the evidence model to support item/model identity, differentiated access, repair/maintenance and lifecycle information where relevant product-specific requirements apply. This is an interoperability consideration, not legal advice or a claim that every furniture item currently requires a passport.

The product model should preserve source attribution and access rights. Do not collapse listing identity, model identity, individual physical item, revision, seller, offer and lifecycle evidence into one mutable product row.

## Fair comparison design

Do not compare a renderer's image quality with an evidence verifier's hard-constraint accuracy. Run separate, task-matched tracks with the same frozen room/tasks, item candidates, time limits and accessible input data.

| Track | Baseline | What is compared |
|---|---|---|
| Visual layout | Planner 5D / IKEA Kreativ, where task and export access allow | time to a visually acceptable draft, edits, room-scale fidelity and user preference—not safety proof |
| Open asset/BIM interoperability | IFC + IDS / BIM-object workflow | required-field coverage, unit/reference preservation, export loss and validation time |
| Human workflow | paid interior designer plus resident co-design, with catalog browsing | operator time, shortlist quality, rework, stated preferences, disagreements and missing evidence |
| Minimal algorithmic baseline | fixed rules: observed availability filter, hard-gate-first checks and documented greedy preference policy | false-safe/false-blocked outcomes, runtime and Pareto coverage |
| Candidate system | Symtropy independent geometry oracle + Mycelix source-attributed listing projection + Symthaea alternative generation | the same metrics, plus uncertainty calibration, revision invalidation and privacy/effects controls |

Record product, software version, jurisdiction/market, operator experience, input/export route and disabled features. Do not impute unsupported vendor features or treat a vendor demo as a benchmark result. Human work must be compensated and the protocol must retain disagreement rather than flattening it into one rating.

## Frozen corpus and adversarial cases

First synthetic corpus sizes: 25, 100, 1,000, and 10,000 listing records; room scenarios: the four furnishing-layout v0.1 cases plus expanded multi-room cases after the first gate. Every corpus has a version, byte digest, declared synthetic-only status, independent expected outcomes and negative controls.

Seed at least one each of: stale listing revision, status changed to sold/withdrawn, quantity changed, missing geometry, wrong units, redisplay-only geometry rights, dimensions contradicted by evidence, recycled-content assertion without source evidence, broken product identifier, missing required IDS property, duplicate candidate, conflicting revisions, truncated result set, fixture data presented as live, and empty/unavailable transport. Include mutations that alter one field while leaving other digests unchanged.

## Metrics and proposed acceptance gates

These are targets, not measured results.

### Epistemic and semantic correctness

- **0 false-safe outcomes** on frozen known-truth hard-constraint cases.
- 100% of missing, stale, contradictory, unlicensed or out-of-scope critical inputs remain `unknown`, `blocked`, or `review_required`; none silently become `pass`.
- 100% of fixture/live ambiguity, unknown completeness and unresolved latest-revision state is visible to consumers.
- 100% of returned source identities/revisions remain bound to outputs; a result from revision A cannot be presented as verified for revision B.
- 100% rejection or explicit quarantine of seeded corrupt/unsupported units, digests, references and claim scopes.
- Report false blocks and the useful-resolution rate as well as false-safe rate; a system that blocks every case is safe but not useful.

### Commercial truth and privacy

- 0 order, reservation, seller-contact, payment, transaction-commit or public floor-plan side effects in the discovery API surface.
- 0 resident floor plans, exact addresses, access/occupant profiles, household preferences, or correlatable design/query digests sent in listing discovery calls.
- Seller assertions stay distinct from independently verified evidence for material, repairability, recycled content, durability, dimensions, price, quantity and delivery.
- A successful empty read remains `empty_with_unknown_coverage` until the data source can attest to completeness; an RPC success does not prove a live complete catalog.
- Preserve revisions and invalidate downstream compatibility on any changed listing/design revision. Never silently reuse a prior compatibility verdict.

### Performance and utility

For each tier report p50/p95 end-to-end time, p50/p95 local projection time, peak memory, bytes transferred, rows scanned/returned, candidate recall on known-positive cases, exact hard-constraint confusion matrix, missing-field coverage, stale-detection rate, Pareto-frontier recall and manual-review burden. Publish hardware, build profile, input digest, code SHA, configuration and raw machine-readable results.

Provisional engineering targets to test—not prior results:
- deterministic output for identical frozen inputs and source revisions;
- local projection p95 under 100 ms at 10,000 records on the declared reference machine;
- under 100 MiB incremental peak memory for the same local-only projection;
- no performance score may compensate for a false-safe, fabricated offer claim, privacy leak or commerce side effect;
- network and DHT times are reported separately from local projection. Do not set or claim a live p95 SLA until conductor/runtime measurements exist.

### Resident outcomes

Only after the technical gates pass: consent-based and paid studies for time-to-shortlist, correct-fit rate after measurement, redesign/replacement effort, repair/refurbished selection, accessibility/task outcomes with qualified review, affordability, preference satisfaction, and perceived control/trust. Stratify results by task and relevant constraints; do not infer need or style from protected or demographic proxies.

## Current implementation boundary

`marketplace-client::furnishing_discovery` is a **read-only projection scaffold**:

- requires only a `ListingReadPort` capability, not a transaction-capable client surface;
- calls the existing typed `get_all_listings` method, filters to the broad Home & Garden category, and preserves the exact returned action hash and observed fields;
- marks item subtype as needing confirmation, geometry as absent, compatibility as not evaluated, revision currentness as unproven, read completeness as unknown, and live-versus-fixture provenance as unclassified;
- performs no checkout, contact, payment, reservation or mutation calls.

It is not yet wired to the furnishing-profile schema in #5251, does not prove the newest listing revision, has no pagination receipt, and does not distinguish fixture transport from a configured live transport. Those are real blockers, not documentation cleanup. Next: reconcile listing-update semantics and transport provenance; add a verified read snapshot/completeness contract; then project into the v0.1 furnishing profile under its rights and evidence gates.

## Decision rule

The first success criterion is not “we have more features than an industry product.” It is: **on the same inputs, we can demonstrate that the system never promotes unknown geometry, stale offers, unsupported seller claims, or a partial catalog into a verified fit or current purchase option—and that it still finds useful candidates once trustworthy evidence is available.** Comparative superiority must wait for the matched benchmark and observed results.
