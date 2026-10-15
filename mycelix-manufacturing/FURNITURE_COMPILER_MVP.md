# Furniture Compiler MVP: Design and Delivery RFC

**Status:** Proposed; architecture/specification only  
**Date:** 2026-10-13  
**Target cluster:** `mycelix-manufacturing`  
**Related systems:** Symthaea (advisory configuration), Symtropy (optional simulation/optimization), Mycelix (manufacturing and product-lifecycle coordination)  
**Scope:** First manufacturable furniture family; no claim of production readiness

## 1. Decision

Build a small, evidence-driven “Furniture Compiler” on top of the existing Mycelix Manufacturing cluster. It converts customer constraints into a *versioned, verified manufacturing packet* and then connects that packet to the existing bill-of-materials (BOM), routing, work-order, and material-requirements-planning (MRP) flows.

Do not create a parallel marketplace/manufacturing backend, do not let an LLM issue shop-floor instructions directly, and do not make decentralized consensus a prerequisite for checkout or inventory reservation.

The first goal is not to replace IKEA’s global supply chain. It is to prove that one small family of original, modular products can be specified, quoted, manufactured, delivered, assembled, repaired, and reconfigured with defensible economics.

## 2. What exists today — and the evidence boundary

The public Mycelix tree already includes:

- `mycelix-manufacturing/zomes/bom`: immutable BOM records and BOM explosion;
- `mycelix-manufacturing/zomes/operations`: manufacturing operations and routing sequences;
- `mycelix-manufacturing/zomes/workorders`: work-order lifecycle and status updates;
- `mycelix-manufacturing/zomes/planning`: MRP planning, material-shortage records, and planned orders;
- `mycelix-manufacturing/crates/manufacturing_common`: shared manufacturing types;
- manufacturing conformance and Sweettest-related tests.

References:
- [Manufacturing BOM coordinator](zomes/bom/coordinator/src/lib.rs)
- [Manufacturing BOM integrity](zomes/bom/integrity/src/lib.rs)
- [Planning coordinator](zomes/planning/coordinator/src/lib.rs)
- [Shared manufacturing types](crates/manufacturing_common/src/lib.rs)
- [Conformance harness and claim ceiling](crates/cos_conformance/README.md)

This is a useful substrate, not evidence that furniture products can already be generated, safety-qualified, produced, or sold profitably. The existing conformance harness explicitly excludes physical productivity, safety, qualification, and economic/ecological outcome claims. Keep that boundary intact.

## 3. MVP product boundary

Start with one local manufacturing partner and a narrowly defined product family:

- modular desk components and low, freestanding storage using common, replaceable panel/hardware parts;
- a bounded set of allowed widths, depths, heights, materials, finishes, and accessories;
- supplier-supported sheet formats, tooling, edge finishing, drilling, and packaging;
- a single initial delivery market and published fulfillment policy.

Exclude seating, beds, children's products, wall-mounted/fold-down units, tall unsecured storage, powered furniture, and any configuration requiring specialized compliance until each category has its own engineering and qualification plan. The first family is a product hypothesis, not a safety determination. Tip-over risk, anchoring needs, sharp edges, pinch points, load limits, emissions, and applicable market requirements must be assessed for the actual product and intended use.

## 4. Architecture and authority boundaries

### 4.1 Customer intent and candidate configurations

The customer interface captures explicit requirements: room/doorway dimensions and their uncertainty, intended use, required storage, budget, accessibility needs, aesthetic preferences, delivery window, and assembly preference.

Symthaea may clarify incomplete requirements, explain trade-offs, and propose candidates from a bounded design family. It is advisory only. A language-model response is never a geometry authority, a safety approval, a price authority, or a release token.

### 4.2 Deterministic design compiler

The compiler accepts a design-family version plus typed parameters and emits a canonical design artifact, part list, assembly graph, BOM input, required operations, package geometry, and validation report.

Canonicalize units and representation before hashing. Recommended rules:

- integer millimetres for dimensions and integer degrees for angles where practical;
- explicit material/specification identifiers rather than free-text names;
- explicit quantity units and positive bounded quantities;
- explicit currency and integer minor units for money;
- stable ordering of component and operation lists before serialization;
- a versioned schema and canonical serialization profile;
- reject NaN/infinity, overflow, unknown parameters, missing material properties, duplicate component IDs, unresolved references, and configurations outside released bounds.

Do not hash presentation data such as localized labels as a substitute for hashing the canonical manufacturing inputs.

### 4.3 Hard constraints vs. optimization objectives

Hard constraints fail closed: geometry and access fit; component compatibility; material availability; permitted machine/tooling; fastening and edge-treatment requirements; validated load/stability envelope; applicable safety/market gate; package and carrier limits.

Optimization objectives are considered only after hard constraints pass: delivered price, scrap, setup time, production lead time, packaging volume, assembly time, repairability, and estimated lifecycle cost.

Symtropy may be used where its physical model is appropriate and independently validated. Cutting-stock optimization, nesting, resource scheduling, supplier assignment, and route consolidation require explicit discrete optimization algorithms and benchmark evidence; they must not be described as capabilities of the physics engine merely because both are “optimization.”

### 4.4 Manufacturing and order execution

Reuse current BOM/routing/work-order/MRP concepts through typed adapters. Preserve current security and transition rules; do not weaken existing manufacturing controller authority or temporal-evidence work to fit the furniture flow.

Use a separate transactional commerce boundary for checkout, payment, and inventory reservation. A Mycelix record can provide provenance and interoperable evidence, but it must not be treated as an atomic stock reservation or payment settlement by itself.

No automatic shop-floor release in MVP. An authorized operator must approve the manufacturing packet and order release.

## 5. Frozen Manufacturing Packet (FMP)

Every quote and release must bind to an immutable packet that contains:

1. Product-family ID and released design revision.
2. Canonical parameter set, units, and design artifact digest.
3. Ordered component/part list and assembly graph.
4. BOM digest, routing digest, and required machine/tooling capabilities.
5. Materials specification, permitted substitutions, and supplier identity/version.
6. Cut-list/nesting result digest and the exact optimizer/tool version that produced it.
7. Package dimensions, gross mass where known, packing plan, and assembly-instruction revision.
8. Feasibility report, qualification status, evidence references, and explicit unresolved risks.
9. Itemized quote input snapshot with source timestamps and validity/expiry.
10. Compiler/schema version, deterministic output digest, and creation provenance.

Any change to a manufacturing-relevant field requires a new revision and invalidates dependent quote, nesting, verification, and release artifacts. A passing result for revision A cannot qualify revision B.

Suggested conceptual identity:

- `design_digest = H(canonical_design_spec)`
- `bom_digest = H(canonical_BOM)`
- `routing_digest = H(canonical_routing)`
- `quote_digest = H(design_digest, cost_inputs, supplier_quotes, logistics_inputs, pricing_policy_version)`
- `release_digest = H(design_digest, bom_digest, routing_digest, qualification_evidence, approved_quote_digest)`

Use a reviewed, domain-separated hash encoding in implementation; these equations define the binding intent, not a chosen wire format.

## 6. Quote model and cost honesty

A quote must separate estimated, supplier-confirmed, and actually observed values. Each line needs a source, currency, timestamp, validity interval, and whether it is a binding price or estimate.

Minimum cost components:

- purchased material and hardware, including yield loss and usable offcuts;
- machine setup, run time, tool wear, edge finishing, finishing, assembly, and inspection;
- inbound freight, packaging, storage/handling, and outbound delivery;
- optional in-home assembly and take-away services;
- payment fees, expected support, returns, damage, and warranty reserve;
- applicable tax and duties, shown separately;
- overhead and contribution margin.

Use integer minor currency units (no floating-point monetary arithmetic) and explicit tax treatment. Do not combine currencies without a timestamped FX source and a declared conversion policy.

A feasible quote is not necessarily a profitable quote. Release a customer-facing price only when required cost sources are sufficiently fresh, capacity and fulfillment assumptions are explicit, and the quote is within its validity window. Otherwise return a reasoned “cannot quote yet” result rather than inventing a price or delivery date.

**Primary economic metric:** total delivered-and-assembled cost for a comparable specification. Include warranty/returns and support costs; do not compare a bare furniture price against an all-in competitor price.

## 7. Material nesting and production batching

For the first prototype, integrate an existing manufacturer-compatible CAD/nesting tool where possible rather than starting by writing a general-purpose nesting solver.

The integration contract should capture:

- exact input part list and geometry digest;
- sheet stock, thickness, grain direction, defects/reserved zones, trim margins, and kerf;
- allowed rotations, tool/bit constraints, cutting order constraints, and machine profile;
- generated placement/cut plan, scrap/offcut inventory, estimated cycle time, and tool version;
- tool output digest and a reproducible comparison report.

Benchmark against the partner's current process using identical inputs. Track material yield, machine/setup time, cut quality/rework, and rejected-part rate. Cross-order batching is allowed only when design revision, material, finish, grain, tolerance, and promised dates are compatible. Never sacrifice traceability or delivery commitments for a superficially better nesting score.

## 8. Product identity and lifecycle record

Every sold unit should have a product-instance identifier linked to its released design revision and the parts actually installed. A QR code can point to a controlled product page with:

- assembly/repair instructions and compatible spare parts;
- material and care information, warranty terms, and repair history;
- applicable qualification claims, bounded to the tested revision and use;
- refurbishment/resale assessments and documented changes.

A digital product passport should be treated as an interoperable data model and export interface, not as a requirement to place every field on a public distributed ledger. Keep customer identity, delivery addresses, transaction details, and commercially sensitive supplier data under appropriate access controls. Publish only what is required, authorized, or useful to downstream repair/reuse actors.

The EU Commission's ESPR working plan lists furniture for product-specific delegated-act work in 2028, and notes that listing is not itself a universal immediate DPP mandate. Build adaptable export and versioning now; confirm applicable law and transition dates for the actual product/market before making compliance claims.

## 9. Explicit lifecycle state machines

Do not overload a single “approved” flag to represent several different facts.

**Design:** Draft → ChecksPassed → EngineeringReview → Released → Suspended/Retired.

**Qualification:** NotAssessed → TestPlanApproved → Tested → QualifiedForScope → Suspended/Expired. A failed test, missing evidence, or material/configuration mismatch cannot be silently treated as qualified.

**Quote:** Draft → CostInputsBound → Feasible → Priced → Accepted/Expired/Withdrawn.

**Manufacturing order:** Draft → MaterialAndCapacityReserved → ReleaseApproved → InProduction → InspectionPending → Accepted → Packed → Dispatched → Delivered; explicit alternatives include OnHold, Rejected, Rework, Cancelled, and Indeterminate when an external effect's outcome cannot be established.

Transitions require the exact prior state/version and idempotency key. Recovery must reconcile uncertain external effects rather than blindly repeat a payment, reservation, or production instruction.

## 10. Tests and evidence gates

Create a frozen, versioned conformance corpus with at least these classes:

- known-valid configurations producing stable canonical digests;
- out-of-range dimensions, conflicting parameters, bad units, and unsupported materials;
- missing hardware, unresolved subassembly BOMs, cycles, invalid quantities, and duplicate IDs;
- doorway/room-fit edge cases with measurement uncertainty;
- load/stability/anchoring gates whose acceptable criteria come from qualified engineering sources, not invented generic limits;
- quote inputs that are stale, incomplete, inconsistent, negative, overflowing, or in mixed currencies;
- supplier changes, expired quotes, stock races, partial payment responses, duplicate webhooks, and retry/reconciliation cases;
- BOM or routing revision drift between quote, approval, and release;
- nesting outputs that violate grain, kerf, trim, or tooling constraints;
- downgrade, missing-evidence, and qualification-expiry cases;
- data-access checks preventing public exposure of customer and supplier-private fields.

Minimum gates:

1. Compiler output is deterministic for a frozen input/toolchain.
2. Invalid hard constraints cannot produce a release packet.
3. Every release points to the exact BOM, routing, quote, and applicable qualification evidence.
4. A stale or mismatched digest fails closed.
5. Retry tests prove checkout, reservations, and external effects are idempotent or explicitly indeterminate/reconciled.
6. Independent packet verification can run without trusting the Symthaea recommendation that originated the configuration.
7. Physical test evidence and economic evidence are recorded separately from unit-test or simulation results.

No numeric score, passing unit-test suite, or conformance report should be presented as proof of physical safety or profitability.

## 11. Pilot scorecard

Capture a baseline before optimizing:

- delivered-and-assembled cost for a matched product specification;
- quote accuracy versus final cost;
- promised vs. actual lead time;
- material yield and usable-offcut recovery;
- setup/runtime, rework, and defect rates;
- assembly time, missing-part incidents, damage, returns, and support burden;
- percentage of configurations independently verified;
- spare-part availability, repair turnaround, and refurbishment recovery;
- contribution margin after variable fulfillment, return, payment, and warranty costs.

Set the actual go/no-go thresholds before pilot orders. A proposed target is at least 10% lower all-in customer cost on a defined comparable basket, but this is a hypothesis to test, not a current result or a promise. If the target is not met, identify the dominant cost term; do not hide it with excluded labor, delivery, quality, or support costs.

## 12. Implementation sequence

1. Write and freeze the product-family spec and target-market requirements.
2. Build a deterministic compiler prototype for a few hand-authored configurations.
3. Generate packet, BOM, routing, cut list, and assembly instructions from the same canonical source.
4. Integrate a real nesting workflow and source-bound quote model.
5. Add a thin adapter to current Mycelix Manufacturing records; preserve existing integrity and authority checks.
6. Independently verify the release packet and add adversarial tests.
7. Manufacture one prototype with a qualified partner and collect physical/actual-cost evidence.
8. Only after those gates pass, add customer-facing configurator, checkout, partner onboarding, and lifecycle/resale services.

Avoid expanding the catalogue or adding autonomous production agents until the first physical product has passed its declared qualification process and economics have been measured.

## 13. External prior art and standards to evaluate

- [O-LAP: open-source parametric furniture designs](https://github.com/O-LAP/home) demonstrates a gallery/configurator concept that emits CAD drawings for CNC fabrication. Customization and parametric designs are therefore not unique differentiators on their own.
- [European Commission: Digital Product Passport FAQs](https://single-market-economy.ec.europa.eu/single-market/digital-product-passport/explore-our-faqs_en) describes the staged, product-specific ESPR approach and indicative furniture timeline.
- [ISO 7170:2021](https://www.iso.org/standard/76864.html) specifies test methods for storage-unit strength, durability, and stability; it explicitly does not itself specify acceptance criteria.
- [BIFMA standards overview](https://www.bifma.org/page/standardsshortdesc) lists relevant furniture safety/durability, ergonomic, and sustainability standards, including X6.5 for home-office/occasional-use products and e3-2024 for sustainability.

Select applicable standards and acceptance criteria for the intended market and exact product. This list is a starting point for qualification review, not a compliance determination.

## 14. Definition of done

The MVP is not “done” when a configurator renders a 3D model. It is done only when one frozen configuration can be independently checked; its BOM/routing/quote agree with the same revision; a qualified producer can manufacture it; the finished item passes the declared inspection/test plan; its delivered cost is reconciled to actual costs; and another party can verify the evidence packet without trusting the recommendation engine.

Until then, describe the work as prototype, specification, or simulation according to the evidence that actually exists.
