# Home Design and Compliance Compiler: architecture and benchmark plan

**Status:** proposal accompanying the first narrow Rust evidence-gate prototype. This is a research/engineering plan, not a statement that the complete system exists.

## Thesis: compile a home, do not merely draw one

The paradigm shift is to treat a custom home as a versioned, testable project assembled from owner intent, a site model, a building model, engineering analyses, jurisdiction-specific rules, evidence, construction changes, and operational records. Drawings remain important outputs, but they are not the sole source of truth.

A language model may help translate a homeowner's request into questions and candidate constraints. It must not be the authority that declares a design legally compliant. Deterministic validators and domain tools should produce findings; qualified professionals and public authorities retain their real-world authority.

## Seven paradigm shifts

1. **Intent-to-model instead of prompt-to-render.** Capture each desire as a typed constraint: hard requirement, preference, budget cap, safety invariant, accessibility need, or negotiable trade-off. Surface contradictions instead of silently choosing one.
2. **Evidence before verdicts.** Every finding binds to a named rule-pack version and exact artifact evidence. Four-valued outcomes—Pass, Fail, Unknown, NotApplicable—prevent missing information from silently becoming success. A NotApplicable disposition requires an explicit rationale and approval record.
3. **Jurisdiction packs instead of a fictitious global code.** Separate reusable engineering logic from local rules. Packs need jurisdiction, scope, edition, official source locator, digest, publication/effective dates, status, amendments, and review history. Draft bills and consultations must never silently become active law.
4. **Open building data instead of vendor lock-in.** Use IFC as the building-model interchange spine and IDS for machine-readable information-delivery requirements where appropriate. Preserve stable object identity, units, classifications, quantities, provenance, and explicit losses across import/export. Do not invent a competing proprietary BIM universe.
5. **Lifecycle performance instead of cheapest initial construction.** Optimize for lifecycle cost, repairability, replaceable components, embodied and operational impacts, thermal comfort, water autonomy, energy reliability, maintainability, accessibility, and occupant choice. Report Pareto trade-offs, not a magical single score.
6. **Design-build-operate continuity.** The same object identities should connect design documents, procurement, site inspections, commissioning, as-built changes, warranties, maintenance, and future renovation. Site changes trigger re-evaluation of affected dependency graphs while preserving whole-package provenance.
7. **Sovereign automation instead of cloud-dependent control.** Local manual control and essential operation should survive internet or cloud failure. Separate recommendation from actuation; typed device authority, explicit consent, least privilege, revocation, safe fallback, and an independently testable executor are required before controlling physical equipment.

## Proposed system boundaries

- **Owner intent and negotiation:** Symthaea can help elicit requirements, clarify trade-offs, and explain why a design failed. It is not a code interpreter or final authority.
- **Site and geographic context:** Sol-Atlas can contribute location-bound data such as terrain, climate, solar context, access, and known infrastructure, with data age, coverage and uncertainty visible.
- **Building model and exchange:** a new dedicated design/model component should own geometry, spaces, openings, assemblies, systems, schedules, quantities and editable exports. IFC and IDS should be evaluated as exchange standards; BCF can be evaluated for issue exchange.
- **Engineering adapters:** Symtropy can contribute selected simulations when validated for the intended physical phenomenon. Structural design, fire, HVAC, electrical, plumbing, daylight, drainage and energy analysis must use independently validated methods/tools where Symtropy is not demonstrably suitable. A generic physics engine is not a substitute for domain validation.
- **Rules and evidence core:** the new standalone Rust crate owns typed review-gate semantics only. Future adapters provide rule-pack validation, BIM checks, calculation outputs, signed review attestations, digest verification and structured reports.
- **Household governance and operations:** Mycelix Hearth can record household/device authority and lifecycle events. Its current device-authority design is explicitly authority-and-audit only; it does not contain a physical executor or adapter. Do not treat that schema as deployed safe automation.
- **Humans and authorities:** architect, engineer, specialist installer, building-control official and owner actions remain distinct roles. The software prepares evidence and reveals gaps; it does not forge professional sign-off or municipal approval.

## Minimal evidence contract

Each rule-pack release should carry:
- stable pack ID, jurisdiction and precisely bounded applicability;
- version/edition, source locator, source status and publication/effective/retirement dates;
- SHA-256 digest of the distributed pack artifact, release provenance and reviewer/signature metadata;
- rule IDs with importance, applicability logic, required input data, evaluator version, tolerances and clause locator;
- explicit exclusions, assumptions, known limitations, licensing conditions, and unresolved interpretation questions.

Each check finding should carry:
- stable check ID and exact rule-pack ID/edition;
- Pass, Fail, Unknown or NotApplicable;
- explanatory rationale and references to digest-bound artifacts;
- evaluator name/version, timestamp, input-model revision, units, uncertainty/tolerance, and dependencies;
- reviewer/approver identity evidence where required. A string saying who approved something is not sufficient identity verification.

The prototype currently implements only the scoped review gate and digest-shape checks. It does not hash files, validate identities, evaluate code rules, or confirm the authenticity/currentness of external law.

## South Africa first, with a global extension path

Start with an explicitly scoped South African residential demonstrator, selected municipality, property/land-use constraints, and a modest single-storey home. Before encoding a rule, classify the source as enacted/current, officially published standard, municipal by-law/scheme, proposed bill, guidance, or unresolved interpretation. Keep SANS clause references and independently authored rule summaries separate from the copyrighted full text; check licensing before redistribution. Do not infer that a proposal or consultation is in force.

The National Building Regulations and Building Standards Act 103 of 1977 provides the national statutory framework, with municipal approval and other applicable law in scope. Municipality guidance says approval is required before construction. Electrical installations and their certificates have separate legal and competent-person requirements. The government has also published a 2026 amendment bill for public comment; the rule-pack workflow must treat proposals distinctly from effective requirements.

Once one municipal profile is reviewed and traceable, add other South African municipalities as data packs, then other countries as separate jurisdiction adapters. Do not mark a jurisdiction supported until its scope, sources, professional boundaries, test fixtures, and update policy are auditable.

## Candidate benchmarks (targets, not measured results)

### Gate correctness and safety
- **Zero false review-ready results** in the versioned adversarial suite: missing mandatory check, Fail, Unknown mandatory, stale/future/draft/withdrawn rule pack, bad pack digest syntax, finding bound to the wrong edition, duplicate/unrecognized finding, Pass without evidence, malformed evidence digest, unjustified NotApplicable, and empty or optional-only scope.
- **100% mandatory-scope accounting:** each mandatory check is Pass, Fail, Unknown, or justified NotApplicable; no silent omission.
- **100% positive-finding traceability:** every Pass references the exact rule pack/edition and at least one artifact ID plus expected SHA-256 digest.
- **No legal-status overclaim:** output schema never emits Compliant, Approved, Certified, or ReadyToBuild from this core.
- Mutation-test every gate condition and retain every minimized regression fixture in the repository. Before any pilot, have an independent professional label the expected results.

### Determinism and performance
- Same input, schema version and evaluator version must yield the same ordered report and digest once canonical serialization is added.
- Initial budget to test on a published reference machine: evaluate 10,000 findings and 10,000 scoped checks in under 100 ms in optimized release mode; test 100,000 as a scale curve. These are proposed budgets, not current performance claims.
- Rule-pack activation and revision changes must invalidate affected findings; never silently reuse a finding computed against a different edition.

### Model interoperability
- Use a curated test set covering spaces, doors, windows, walls, slabs, roof assemblies, site boundary, levels, units, classifications and quantities.
- Across IFC import/export round trips, target 100% preservation for explicitly required fields/IDs; report every unsupported field or geometry loss instead of silently dropping it.
- Validate required information against IDS 1.0 where appropriate, with one positive fixture and malformed/omission fixtures for each supported requirement pattern.
- Run interoperability checks through at least two independent openBIM tools before describing an exchange path as verified.

### Engineering fidelity and design usefulness
- Keep a versioned suite of reference cases for each engineering adapter. Critical safety cases require zero missed known-critical violations in the controlled fixture suite; also track false positives, numeric error, solver version and tolerance.
- Before pilot, compare energy/thermal results against an independently accepted reference tool on calibrated cases; record error distributions rather than cherry-picking the best case.
- Candidate design target: produce at least three materially distinct, feasible concept variants for at least 80% of a curated set of feasible residential briefs within 60 seconds on a declared reference machine. Record infeasible briefs and explain which constraints conflict.
- Cost-estimation targets should be calibrated from completed local projects; initially publish mean absolute percentage error and uncertainty by design stage, not one universal accuracy claim.
- Resilience tests should simulate declared outage, heat and water scenarios and report which services remain available, for how long, and with what assumptions. The owner must choose acceptable service levels.

### Human and owner outcomes
- Measure how long a non-expert needs to create a complete brief, identify design conflicts, compare lifecycle options and prepare a reviewable package.
- Track professional review changes, reasons for municipality rejection/requests for information, construction change orders, estimate-to-final cost deviation, commissioning defects, and the difference between predicted and measured operational energy/water use.
- Publish a plain-language explanation and machine-readable trace for every blocker. Do not optimize away legitimate professional disagreement to improve a headline success rate.
- Verify offline operation, manual overrides, permission revocation, safe state on runtime failure, and homeowner data export before device control is enabled.

## Suggested work sequence

**P0 — evidence gate (this branch):** standalone, dependency-free Rust types and deterministic gate rules; adversarial unit tests; no Holochain or hardware side effects.

**P1 — schema and fixtures:** versioned JSON schema for rule-pack metadata, check definitions and findings; synthetic South African-like fixtures clearly marked non-authoritative; canonical serialization and digest verification.

**P2 — site and model:** define stable IDs and units, evaluate IFC 4.3/IDS 1.0 adapters, generate a capability report listing supported/unsupported fields.

**P3 — one reviewed jurisdiction profile:** professionally reviewed, legally sourced rules and mutation fixtures for a deliberately small residential scope. Never claim completeness outside that scope.

**P4 — reference home:** open fixture model plus design alternatives, cost/energy estimates, negative controls, a sample review package, and reproducible benchmark command.

**P5 — operations:** Mycelix event/provenance integration; smart-device executor only after capability authorization, threat model, physical safety review and end-to-end tests.

## Sources and design constraints

- South African Government, National Building Regulations and Building Standards Act 103 of 1977: https://www.gov.za/documents/national-building-regulations-and-building-standards-act-16-apr-2015-1302
- George Municipality, building-plan approval before construction: https://www.george.gov.za/notices/obtain-building-plan-approval-before-commencing-with-construction/
- South African Government, electrical installation laws and CoC requirements (22 Oct 2025): https://www.gov.za/news/media-statements/employment-and-labour-electrical-installation-laws-and-certificate-compliance
- South African Government, 2026 amendment bill for public comment: https://www.gov.za/documents/notices/national-building-regulations-and-building-standards-act-publication-national
- buildingSMART, IFC 4.3.2.0 / ISO 16739-1:2024: https://www.buildingsmart.org/standards/bsi-standards/industry-foundation-classes/
- buildingSMART, IDS 1.0 standard: https://technical.buildingsmart.org/Projects/Information-Delivery-Specification-Ids/
- W3C, Shapes Constraint Language (SHACL): https://www.w3.org/TR/shacl/

Standards establish data exchange or constraint languages; they do not themselves certify a building design. This plan is engineering research, not legal advice, code certification, or a claim that Symthaea, Symtropy, Sol-Atlas or Mycelix already implements this complete workflow.
