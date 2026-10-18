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

The prototype performs metadata/scope checks and defines an `EvidenceVerifier` interface. `evaluate_review_gate` cannot advance beyond `MetadataCompleteAwaitingEvidenceVerification`; only `evaluate_verified_review_gate` can return `ReadyForProfessionalReview`, after the supplied trusted adapter confirms artifact existence and recomputes SHA-256 over the actual bytes. No production artifact store/verifier implementation is included, so deployments must not treat a mock verifier as proof. The crate does not validate identities, evaluate code rules, or confirm the authenticity/currentness of external law.

## South Africa first, with a global extension path

Start with an explicitly scoped South African residential demonstrator, selected municipality, property/land-use constraints, and a modest single-storey home. Before encoding a rule, classify the source as enacted/current, officially published standard, municipal by-law/scheme, proposed bill, guidance, or unresolved interpretation. Keep SANS clause references and independently authored rule summaries separate from the copyrighted full text; check licensing before redistribution. Do not infer that a proposal or consultation is in force.

The National Building Regulations and Building Standards Act 103 of 1977 provides the national statutory framework, with municipal approval and other applicable law in scope. Municipality guidance says approval is required before construction. Electrical installations and their certificates have separate legal and competent-person requirements. As of 18 October 2026, the official government listing says the 2026 amendment bill's public-comment deadline was extended to 12 November 2026. The proposal must remain a proposal in the registry unless and until its legal status changes through an authoritative, recorded update: https://www.gov.za/documents/notices/national-building-regulations-and-building-standards-act-extension-submission .

Once one municipal profile is reviewed and traceable, add other South African municipalities as data packs, then other countries as separate jurisdiction adapters. Do not mark a jurisdiction supported until its scope, sources, professional boundaries, test fixtures, and update policy are auditable.

## Candidate benchmarks (targets, not measured results)

### Gate correctness and safety
- **Zero false review-ready results** in the versioned adversarial suite: missing mandatory check, Fail, Unknown mandatory, stale/future/draft/withdrawn rule pack, bad pack digest syntax, finding bound to the wrong edition, duplicate/unrecognized finding, Pass without evidence, malformed evidence digest, unjustified NotApplicable, and empty or optional-only scope.
- **100% mandatory-scope accounting:** each mandatory check is Pass, Fail, Unknown, or justified NotApplicable; no silent omission.
- **100% positive-finding traceability and byte integrity:** every Pass references the exact rule pack/edition and an artifact ID plus digest; the review-ready status is unavailable unless a trusted adapter confirms the object exists and hashes to that digest. Tests must distinguish missing object, digest mismatch, unavailable store, and successful verification.
- **No legal-status overclaim:** output schema never emits Compliant, Approved, Certified, or ReadyToBuild from this core.
- Mutation-test every gate condition and retain every minimized regression fixture in the repository. Before any pilot, have an independent professional label the expected results.

### Determinism and performance
- Same input, schema version and evaluator version must yield the same ordered report and digest once canonical serialization is added.
- The metadata-only API must never emit `ReadyForProfessionalReview`; verified readiness requires every evidence object to be resolved and hashed by the configured verifier. Zero unavailable or digest-mismatched objects may be accepted.
- Initial budget to test on a published reference machine: evaluate 10,000 findings and 10,000 scoped checks in under 100 ms in optimized release mode; test 100,000 as a scale curve. These are proposed budgets, not current performance claims.
- Rule-pack activation and revision changes must invalidate affected findings; never silently reuse a finding computed against a different edition.
- Change-impact reporting must achieve 100% downstream invalidation recall and 0 unrelated invalidations on the reference dependency corpus, including simultaneous changes to multiple roots. The invalidated set must equal the union of each root's downstream closure with the changed roots themselves excluded from the dependent list. Every invalidated node must have a reconstructible deterministic shortest cause chain; changed roots remain explicit, and malformed/cyclic graphs yield an error rather than a partial report. Target linear-space cause reporting at 10,000 nodes and test 100,000-node shapes; these are acceptance targets, not current measured performance.
- A dependency-impact benchmark is available with `cargo bench --offline --bench dependency_impact`; it records median, minimum, and maximum closure/cause-tree times across five samples for chain, binary-tree, and fan-out graphs at 1,000, 10,000 and 100,000 edges. Each sample asserts closure size, ordering, and a selected parent link. CI compiles the harness without timing gates. No timing result should be advertised until run on a documented reference machine and toolchain.

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

**P0 — evidence gate (this branch):** standalone, dependency-free Rust types, deterministic review-gate semantics, temporal source applicability, conflict-aware normalized requirement resolution, revision lineage, invariant-gated migration planning, dependency invalidation, and adversarial unit tests; no Holochain or hardware side effects.

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
- W3C PROV-O, provenance interchange vocabulary: https://www.w3.org/TR/prov-o/
- NIST Building Digitization and Semantic Interoperability, lifecycle data and shared building-system semantics: https://www.nist.gov/programs-projects/building-digitization-and-semantic-interoperability
- NIST Digital Twin Standardization, common models and interfaces for interoperability: https://www.nist.gov/digital-twins/digital-twin-standardization
- ISO 14721:2025 (OAIS), long-term preservation reference model: https://www.iso.org/standard/87471.html

Standards establish data exchange or constraint languages; they do not themselves certify a building design. This plan is engineering research, not legal advice, code certification, or a claim that Symthaea, Symtropy, Sol-Atlas or Mycelix already implements this complete workflow.


## Century-scale adaptability: design for evolution, not timeless prediction

A century-ready system cannot assume that the current architectural canon, materials catalogue, energy system, construction method, code edition, file format, software vendor, or hazard model will remain stable. It must make change safe, inspectable, reversible where possible, and explicit about information that cannot be translated.

### Temporal separation

Store four distinct truths rather than overloading one “current state”:

1. **Historical intent and evidence:** what the owner requested, which design revision was assessed, the exact input files, which tool versions ran, and the results produced at that time.
2. **Historical legal context:** which rule-pack editions were recorded as applicable, the source and effective-date assertions, and which professional/authority decisions were captured. Never retroactively rewrite a historic receipt.
3. **Current reassessment:** a new assessment of the chosen current building revision against a separately pinned current rule pack. This can produce changed outcomes without falsifying the historical assessment.
4. **Physical reality:** as-designed, as-permitted, as-built, as-maintained and as-observed states are different. A drawing must not silently stand in for an inspected as-built condition.

A building may be legal when built yet require new controls for a later alteration or a changed use. The software must report the distinction rather than make a blanket legal conclusion.

### Version everything that can change

Give stable identities and independently versioned lifecycles to:
- building objects and their assemblies;
- model schemas, units and classification dictionaries;
- construction methods, materials and supplier specifications;
- engineering solvers, libraries, assumptions and reference fixtures;
- regulatory packs, amendments, interpretations and jurisdiction boundaries;
- device protocols, firmware, automation policies and safety envelopes;
- migration tools, conformance suites, source datasets and signed review attestations.

Do not treat every version number as semantic versioning. A legal edition, material standard, construction technique and file schema have different change meanings. Model explicit compatibility assertions and effective dates for each family.

### Append-only provenance and conservation of meaning

A revision should be a new immutable record with a parent link, stable artifact identity, exact-byte digest, format/version identifier, recorded time, author/source attribution and a clear reason for change. Corrections add a superseding record; they do not erase the old one. In real deployments, signed and externally anchored manifests are needed to protect against an administrator rewriting both data and the local hash chain.

For long-term preservation, keep the original bitstream when lawful and feasible, a canonical open representation, a human-readable rendering, an interpretation guide, units and coordinate references, schemas, software/tool versions, dependencies, licensing metadata, and a known-good sample corpus. Periodically verify fixity, refresh media, duplicate across independent custodians, and test retrieval without the original vendor software. Use a preservation plan consistent with the OAIS concepts in ISO 14721:2025; do not assume keeping an IFC file alone preserves all design intent or semantics.

### Explicit migration graph, never a magic “upgrade”

Migrations are versioned, reviewable edges between exact format versions:
- **Verified lossless:** eligible for automated planning only after independently reviewed conformance fixtures demonstrate preservation of required meaning and properties.
- **Requires human review:** may be proposed, but must not be executed invisibly or receive a lossless claim.
- **Lossy:** only with an explicit, recorded acceptance of identified losses; never choose automatically for safety-critical or legal evidence.
- **Experimental or retired:** not eligible for automatic migration until the status is reviewed.

Retain the source revision and migration report after every conversion. Compare semantic invariants and required object identities before and after. A syntactically valid digest is not proof that bytes match it, and a migration marked lossless by its author is not independent proof of fidelity. The evolution module now requires a candidate migration to declare critical invariants, source and target invariant digests, source/target artifact digests, a verification-evidence digest, tool version, and an explicit ApprovedForAutomaticUse disposition. A validator compares every declared critical invariant and only then creates an opaque VerifiedMigration value that the automatic path planner accepts. This API-level guard reduces accidental misuse but is not a cryptographic boundary: callers can still lie about extracted observations or approval unless the production adapter recomputes actual content digests and verifies reviewer authorization. The module supplies parent-chain validation, bounded path planning, semantic-invariant comparison, and deterministic transitive dependency closure. The closure identifies what the caller must mark stale, but does not itself persist invalidation. It does not execute migrations, hash artifact bytes, prove the required invariant set is complete, or establish independent assurance.

### Adaptation to future architectures and construction methods

Make the domain model capable of expressing new materials, joinery, robotic assembly, additive manufacturing, prefabrication, bio-based materials, repair patches, reuse, modular extensions and construction approaches not yet invented. New method plugins should declare:
- applicable object types and inputs;
- geometry/units conventions and coordinate frame;
- safety and quality assumptions;
- required calculations, tests and specialist approvals;
- resource and environmental impacts with uncertainty;
- versioned output artifacts and explicit compatibility boundaries.

Unknown materials or methods must be represented as unknown/unverified, not coerced into the closest known category. Core schemas should allow namespaced extensions, but unknown extensions must not silently satisfy mandatory checks. Safety-critical unknowns block the relevant gate while preserving the project and its unrelated work.

### Revalidation based on dependency graphs

Every result should record which model objects, rules, datasets, solver versions and assumptions it consumed. When any dependency changes, calculate the affected dependency closure and invalidate stale results. A changed roof material may affect fire, structural loads, thermal performance, maintenance and embodied-carbon analyses, but need not rerun unrelated identity records. Results not rerun must display as stale relative to the new input digest.

### Reference-home scenario and executable change propagation

The synthetic input brief at `fixtures/johannesburg-adaptable-home-v1.json` supplies a shared project vocabulary for a hypothetical Johannesburg-area home. It intentionally separates owner goals from verified legal constraints and engineering evidence. Alternative styles (courtyard, contemporary, regional-vernacular interpretation, multigenerational flexibility) and construction families (masonry, light-gauge steel, engineered timber, stabilized earth, panelized/modular) are candidates to evaluate, not recommendations that they are all safe, approved, or locally code-compliant.

The integration tests encode two golden dependency-closure cases:

| Mutation | Expected invalidation | Must remain unaffected by this mutation alone |
|---|---|---|
| Energy rule-pack revision | energy finding, package-level assessment, permit submission | floor-plan geometry, structural review, floor-plan drawing, IFC export |
| Floor-plan geometry revision | structural and energy findings, energy model, drawing/export, package assessment, submission | energy rule-pack source itself |

The gate now treats the rule pack's declared check manifest as part of the pack model and compares the assessment's check list against it. A missing declared check, an unexpected extra check, duplicate manifest IDs, or a changed mandatory flag blocks the result. This prevents an accidental scope edit from being treated as the same assessment. It still does not prove the manifest is legally complete or that its declared digest/authorship is authentic: trusted source-pack loading, digest recomputation, signatures and jurisdiction-specific review remain outside this crate.

The test graph is deliberately small and explicit. It tests an architectural invariant: **recompute the smallest complete downstream closure, and do not make unaffected evidence stale by association**. In a larger graph, a package-level submission can be invalidated even while a sibling calculation remains current. These tests verify the closure helper's returned set; they do not yet persist stale status or rerun evaluators.

The new integration suite also verifies that an unknown mandatory energy result blocks professional-review readiness and that a proposed amendment with a conflicting value cannot displace a current binding assertion. The Rust helper now extracts dependency-edge pairs from the versioned JSON brief; it no longer duplicates that graph in test code. The published schema is accompanied by a standard-library-only CI verifier that checks the schema's used keyword subset, cross-references graph nodes against stable artifact IDs, checks each declared mutation against reachability, and runs six deliberately invalid mutation controls. This is not a generic JSON Schema engine, the Rust runtime does not load the fixture, and full IFC parsing plus geometry/solver integration remain future work.

#### Near-term executable benchmarks

| Benchmark | Acceptance target | Current evidence |
|---|---|---|
| Dependency precision | 100% of required transitive dependents returned; 0 unrelated sibling artifacts included in golden mutation fixtures | Two synthetic closure scenarios authored; execution must be confirmed by exact-head tests |
| Graph scale and determinism | Complete, lexically deterministic closure for a 10,000-edge chain; no missing descendants | A 10,000-edge correctness stress test is authored; latency is deliberately not asserted by a flaky wall-clock unit test |
| Dependency graph validity | 0 cyclic graphs accepted; malformed or cyclic graphs return an error instead of a partial closure | A non-self cycle with a downstream node is now a fail-closed regression test; exact-head execution not yet confirmed |
| Gate fail-closed behavior | 0 review-ready outcomes when any mandatory check is Unknown, failed, missing, ambiguous, mismatched to the rule-pack manifest, or tied to the wrong rule edition | Unit and integration tests authored; latest exact-head compile/test run not yet confirmed; no engineering validator is implied |
| Proposal/current-law separation | 100% of proposed-only assertions excluded from binding resolution; unresolved/unknown status remains indeterminate | Synthetic source-status integration case authored |
| Fixture/schema integrity | Versioned schema contract checked; all dependency edges consumed from the same fixture; invalid authority/compliance claims and dangling dependencies rejected by negative controls | Schema-subset and semantic verifier authored with fourteen negative controls, including unsupported schema keywords, array-constraint enforcement, cyclic dependency rejection, and simultaneous-change closure mutations; runtime typed loading and a generic JSON Schema validator remain unimplemented |
| Variant feasibility | For a curated suite of feasible briefs, generate at least 3 distinct, constraint-satisfying candidates in a measured latency budget; report infeasible briefs rather than relaxing constraints silently | Not implemented |
| Engineering validity | 0 missed critical violations in a curated, independently reviewed reference corpus; tolerances and solver/model versions published | Not implemented; no structural/HVAC/fire/electrical solver integration |
| Lifecycle migration | 100% preservation of declared critical invariants or explicit blocked/lossy disposition; 0 silent dropping of fields | Generic migration guards implemented, artifact hashing and real adapters not implemented |

### Benchmark targets by time horizon

These are targets to implement and measure, not achieved claims.

| Horizon | Required capability | Proposed acceptance test |
|---|---|---|
| 1 year | Reproducible revision and rule-pack lifecycle | 100% of assessed packages pin exact design and rule-pack revisions; changing a pack never mutates an existing historic receipt |
| 10 years | Routine format/schema migration | A published fixture corpus round-trips all required semantic fields; every loss is surfaced, and unknown fields survive archival export in an extension-safe form |
| 25 years | Vendor and maintainer independence | A clean-room team can recover, interpret and validate an archived project using only the published package, open specifications and documented tools, without original vendor accounts |
| 50 years | Materials and systems obsolescence | Every critical maintainable component has an inspectable specification, replacement strategy or explicit end-of-support hazard; obsolete equipment does not make the whole record unreadable |
| 100 years | Historical truth plus current reassessment | A historical assessment reproduces against its archived inputs/toolchain where feasible, while a separate assessment can run against a new schema, new hazard data and then-current rules |
| 100+ years | Organizational continuity | At least two independent custodians can restore the information package and verify fixity; ownership, provenance, units, location context and unresolved uncertainty remain intelligible after a mock vendor/service shutdown |

### Long-horizon acceptance criteria

1. **Historical immutability:** zero in-place changes to accepted revision records; corrections create new linked revisions.
2. **Migration honesty:** zero automatic migrations through edges that are lossy, experimental, retired or awaiting human review.
3. **Semantic preservation:** 100% preservation of the declared critical invariants in the versioned test corpus; zero silent dropping of unknown required fields.
4. **Rule-time correctness:** every finding identifies the exact rule pack and assessment date; no draft/proposed rule pack is silently treated as effective.
5. **Reproducibility:** given preserved inputs and pinned software/toolchain, reruns reproduce byte-identical canonical reports where deterministic execution is claimed; where numerics or toolchains prevent this, publish tolerances and explain the gap.
6. **Revalidation precision:** every dependency change invalidates all affected findings, and a mutation suite tests that no impacted result remains current.
7. **Recoverability:** scheduled restore exercises verify at least two independent copies and the associated schemas, keys/credentials recovery procedures, dependency manifests and instructions.
8. **Intergenerational comprehension:** a cold-read study asks professionals unfamiliar with the original project to identify the building's layout, materials, structural assumptions, system interfaces, inspection evidence, limitations and change history without oral help from the original designer.

Century-scale preservation cannot be certified by a single benchmark today. We can test the invariants, run annual preservation drills, and make the organization capable of replacing formats, tools, custodians and funding sources without discarding the record.

### Open model contracts and simulation validation

The home compiler should distinguish three different kinds of validation rather than collapse them into a single "BIM passed" result:

1. **Information exchange contract.** Use a pinned IFC release plus IDS requirements for required entities, properties, classifications, materials, values and units. buildingSMART states that IDS 1.0 became a final standard in June 2024 and is designed for machine-interpretable exchange requirements. IDS is tied to IFC and does not cover geometry, so a successful IDS check must not be presented as proof of geometric, structural, fire or accessibility validity. References: [official IDS standard overview](https://www.buildingsmart.org/standards/bsi-standards/information-delivery-specification-ids/) and [official IFC specification release table](https://technical.buildingsmart.org/standards/ifc/ifc-schema-specifications/).

2. **Geometry and domain analysis.** Maintain separate validators for spatial relationships, egress, structural systems, moisture/durability, accessibility, energy and services. Each must declare the model properties it consumes, its domain and code scope, numerical tolerances, known limitations and the exact solver version. Unknown or missing inputs must remain visible rather than silently receiving defaults that create a pass.

3. **Numerical verification and empirical validation.** For any future building-energy simulator, use applicable ANSI/ASHRAE Standard 140 test procedures as a software-capability baseline; the standard is specifically intended to reveal predictive differences from algorithmic differences, modeling limitations, code faults, documentation gaps or input errors. Comparative test success is not the same as empirical accuracy for a particular site: measured validation also depends on input quality and experimental uncertainty. Reference: [ASHRAE/IBPSA building energy simulation resources](https://www.ashrae.org/technical-resources/bookstore/ashrae-building-energy-simulation-and-performance-modeling-resources) and [ASHRAE Handbook discussion of verification and validation](https://handbook.ashrae.org/Handbooks/F21/IP/f21_ch19/f21_ch19_ip.aspx).

NIST's building digitization work highlights the need for machine-readable semantic data across design and operations rather than repeated manual mapping. Its 2026 digital-twins workshop summary also identifies interoperability, verification/validation/uncertainty quantification, cybersecurity and workforce readiness as research priorities. These findings support contracts and independent evidence at every boundary, not a single monolithic generative model. References: [NIST Building Digitization and Semantic Interoperability](https://www.nist.gov/programs-projects/building-digitization-and-semantic-interoperability) and [NIST Digital Twins Workshops Summary Report (July 21, 2026)](https://www.nist.gov/publications/digital-twins-workshops-summary-report).

#### Standards-aligned benchmark gates

| Benchmark | Acceptance target | Current state |
|---|---|---|
| IFC/IDS information completeness | 100% required information requirements checked against a pinned IFC/IDS edition; zero silent required-field loss on export/import; every unsupported/geometry-only requirement routed to the relevant independent validator | Not implemented; current fixture is not IFC and the schema verifies only the software scenario contract |
| Geometry separation | Zero geometry-critical requirements represented as "passed" solely because an IDS/alphanumeric property check passed | Architectural requirement; no geometry engine integrated |
| Energy simulation software QA | All applicable Standard 140 cases for the declared software scope executed and reported; no out-of-scope accuracy claim | Not implemented |
| Site-specific empirical validation | Publish residuals, calibration data, parameter uncertainty, missing sensors and weather/source editions; do not claim precision beyond measured evidence | Not implemented |
| Lifecycle semantic preservation | Required identifiers, units, classifications and critical properties survive round trips or yield a machine-readable loss report | Future IFC corpus and exchange tests required |

### Standards are inputs, not permanent foundations

Track standards by publisher, identifier, version, status, publication/effective dates, amendment/errata links, local adoption status and licence. The buildingSMART standards registry visibly distinguishes final, draft, withdrawn and retired schemas. Its IDS page marks version 1.0 as final while future versions can evolve. For digital preservation, ISO 14721:2025 (OAIS, edition 3) is a current reference model for long-term digital information preservation. These are reasons to track status over time—not reasons to hardcode today’s standards as eternal truth.

### New implementation included in this branch

The standalone Rust crate now has an evolution module that:
- validates supplied artifact-revision lineage for missing parents, cross-artifact parent links, duplicate revision IDs, invalid digest syntax, time inversions and cycles;
- models migration edges with explicit assurance classes;
- verifies declared critical semantic invariants and only then mints an opaque migration value accepted by the bounded, deterministic automatic path planner;
- fails closed when the migration catalogue contains malformed metadata.

It does not enforce append-only storage, recompute content digests, cryptographically validate signers, execute transformations, prove semantic equivalence, or constitute an archival service. Those remain subsequent work with dedicated fixtures and independent review.


### Regulatory-source clock and status contract

The new standalone Rust regulatory module keeps **law-as-of date** separate from **registry-knowledge-as-of date**. Each source records jurisdiction, edition, official locator, source digest, legal status, publication date, effective interval and the date its currentness was last checked. Eligibility has three distinct results: Applicable, NonBinding, NotInEffect, and Indeterminate. The implementation uses a status enum plus these outcomes so that proposed instruments, guidance, withdrawn sources, stale source checks, unknown status and contradictory dates cannot quietly act as binding in-force rules.

This does not automatically discover amendments or prove legal status. A production source-maintenance service must poll or receive publisher/authority updates, retain each dated source snapshot, require independent confirmation for status transitions, and produce an append-only update event. Historical assessments read the registry snapshot pinned to that assessment; current reassessments use the latest verified snapshot. Source freshness thresholds must be configured per jurisdiction and risk class, not hardcoded as a universal legal requirement.

The module currently checks metadata and digest shape only. It cannot validate the underlying document, legal authority, signatures or exact law without a source-verification adapter and professional/legal review.

### Rule conflict resolution: no silent winner

The regulatory module now has a conflict-aware resolver for overlapping assertions about the same normalized requirement. Its conservative default is RequireAgreement: divergent current, applicable assertions return an explicit Conflict with source/clause provenance. An optional UseExplicitPrecedence policy is allowed only when every applicable assertion has a jurisdiction-profile rank; only a unique highest-ranked value, or agreeing values at the highest rank, can resolve. Lower-ranked assertions remain recorded as superseded sources. Equal-rank disagreement remains a conflict.

The resolver never derives legal hierarchy from recency, assumes that the numerically largest/smallest value is safest, or infers that one standard overrides another. The jurisdiction profile must supply the precedence model and human/legal review. Non-binding sources do not satisfy a binding requirement; an indeterminate source status blocks a partial resolution because the source set may be incomplete. If there are no applicable assertions, the outcome is indeterminate rather than “no rule applies.”

This is exact-value resolution over normalized assertions, not a building-code interpreter. It cannot determine whether two clauses are semantically equivalent, whether numeric bounds can be safely combined, or whether the source set is complete. Those must be handled by rule-pack authoring and independently reviewed fixtures.


Relevant reference: ISO 14721:2025 (OAIS), whose published description covers long-term preservation and information-model changes over time: https://www.iso.org/standard/87471.html .

## Global jurisdiction capability registry

The new `jurisdiction_registry` module uses capability cards so global rollout is a **measured, bounded assertion**, not a country-name toggle. A declared Supported card has to account for all 11 defined regulatory domains; each domain must either have source-backed module coverage or a documented not-applicable decision with rationale. The card pins a source-registry snapshot and test-corpus digest, reports a positive test count, has a dated registry check subject to a caller-configured freshness threshold, documents an update policy and accountable maintainer, and contains two independent reviews pinned to the same immutable subject digest. A challenging or inconclusive review prevents the supported-claim metadata gate from passing; dissent is kept rather than averaged away.

The gate checks metadata, not the law or source bytes. Production must authenticate reviewers and publishers, recompute hashes over actual archived artifacts, verify authority records, demonstrate test-corpus adequacy, and define a legal scope for the supported claim. The profile cannot imply support for omitted domains. Every domain is either Covered, ExplicitlyNotApplicable with a recorded decision, or Missing; only the last form blocks research omissions in a declared Supported profile.

Acceptance targets:
- **Zero unsupported jurisdiction labels:** every Supported claim has complete domain accounting, source/test snapshot references, update ownership, and two independent supportive reviews.
- **Zero hidden gaps:** 100% of regulatory domains have either source-backed coverage or an explicit, reasoned not-applicable decision.
- **Zero stale support cards:** an expired registry freshness window or future check date blocks the supported-claim metadata gate.
- **100% review-snapshot agreement:** each reviewer attests to the same canonical subject digest, excluding the review-attestation list to avoid a self-referential hash.
- **No global extrapolation:** support for one jurisdiction never implies support for a sibling jurisdiction or a different country.

## Global code composition and evidence-backed reform

The new `composition` module makes rule-module ancestry explicit while refusing to infer legal precedence from geographic scope or numeric ordering. Global-reference modules cannot declare binding requirements. Binding statements from indeterminate/non-applicable sources are rejected; conflicts return multiple provenance-carrying candidates without a selected winner. Explicit overrides name the inherited statement, decision record, reviewer, rationale, and evidence digest. The metadata-only composer rejects any override; a separate `compose_verified_rule_modules` entry point requires an external jurisdiction-specific `OverrideVerifier` to authenticate the decision, reviewer, source bytes, and authority to vary that exact rule. No production verifier is included. Owner preferences, engineering baselines, proposals, and evidence-backed reform proposals remain separate from binding requirements.

The new `reform` module prepares code-reform dossiers with study metadata, opposing evidence, methods, limitations, uncertainty, counter-evidence search, affected-group impacts, conflicts of interest, and independent reviews pinned to an immutable dossier snapshot. Its strongest state is `ReadyForPublicConsultation`, which is a preparation/workflow state—not a declaration that the evidence is conclusive or that a code changed. The authorized regulator or standards body remains responsible for adoption. No universal numerical evidence threshold is imposed across fundamentally different technical domains. It also models a public-consultation closeout with pinned notice, feedback archive, response record, and dates, followed by a distinct adoption-decision record linked to the exact dossier snapshot, consultation ID, and closeout digest. Adopted decisions require a recorded effective date no earlier than the decision; rejected/deferred decisions cannot carry an effective date. These validations check record consistency, not legal authority or signatures.

Reference design/standards context: buildingSMART's IFC and regulatory information requirements work target vendor-independent information exchange for digital permitting; OECD's Regulatory Impact Assessment guidance recommends examining alternatives and likely positive/negative impacts; performance-based code paths are an established way to compare outcomes rather than enforce one prescriptive design. Relevant sources:
- https://technical.buildingsmart.org/standards/IFC/
- https://www.buildingsmart.org/standards/calls-for-participation/regulatory-information-requirements/
- https://www.oecd.org/en/publications/2020/02/regulatory-impact-assessment_0bf78a03.html
- https://shop.iccsafe.org/2024-icc-performance-coder-for-buildings-and-facilities.html

### Global composition and reform acceptance targets

- **Zero implicit global-law imports:** a global reference module must never supply a binding requirement merely because a project imports it.
- **100% conflict visibility:** every disagreeing applicable binding statement produces a conflict record with exact source/module provenance; zero hidden "winner" selection from module order, numeric magnitude, or geographic scope.
- **100% override traceability:** every accepted override names the inherited statement, jurisdictional authority decision, reviewer, rationale, and evidence artifact. A production verifier must authenticate those items and the actual source bytes.
- **Zero proposal leakage:** guidance, future-adopted instruments, proposals, engineering baselines, owner preferences, and evidence-backed reform recommendations must never silently modify active binding rules.
- **100% reform snapshot pinning:** reviewers must be linked to the exact immutable dossier snapshot they reviewed; missing/mismatched digests, author-as-independent-reviewer, or blank uncertainty/counter-evidence sections block readiness.
- **Dissent preservation:** no reviewer or counter-evidence record is dropped when reviewers disagree; a consultation-preparation state is not described as scientific conclusiveness or formal adoption.
- **Freedom of design:** on a reviewed reference corpus, measure feasible design alternatives retained after hard constraints, false rejections of feasible alternatives, and whether every rejection names a verifiable applicable requirement rather than a style preference.
- **Human-centered immersive review:** compare static plans, floating camera, guided route, and optional VR against pre-seeded spatial/accessibility issue detection, user pause/exit, motion discomfort, and perceived agency. VR routes must remain optional and are not a substitute for geometry or code checks.

Related Symtropy route work is being developed separately in [draft PR #1993](https://github.com/Luminous-Dynamics/symtropy/pull/1993). Its current route playback and synthetic-room demo do not yet import real building models, implement XR, check camera collisions, or validate compliance.

This remains a prototype and is not an authoritative global code registry, source-signature verification service, or legal advice.
