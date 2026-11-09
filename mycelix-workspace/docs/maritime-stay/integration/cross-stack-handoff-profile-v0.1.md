# Maritime Commons Cross-Stack Evidence Handoff Profile v0.1

**Status:** integration contract proposal; the template is intentionally empty and unexecuted  
**Tracking:** Mycelix #5968  
**Consumers:** Symthaea #11173, Symtropy #4276, Sol-Atlas #654  
**Compatibility:** references owner-native artifacts; does not replace Mycelix receipts, Sol-Atlas provenance, Symtropy run manifests, or Symthaea decision-evaluation records

## 1. Purpose and boundary

This profile defines the minimum reference graph needed to carry evidence between Mycelix, Sol-Atlas, Symtropy and Symthaea without flattening their domain semantics into a new universal data model.

It is an **index of owner-native artifacts**, not an evidence store, identity registry, event bus, geospatial database, simulation format, cognitive representation or authority engine.

The manifest can say which artifact was consumed by which component, for what purpose, at what exact code/profile version, and what transformation/evaluator produced a derivative. It cannot prove the truth of the referenced claims. Each owner retains its own validation and claim boundary.

## 2. Owner responsibilities

| Owner | Produces/owns | Consumes | Must not claim |
|---|---|---|---|
| Sol-Atlas | Source-bound spatial/temporal projection, asset/provenance references, coverage, CRS/datum and transformation lineage | Qualified source/product references; provider/location/resource references | Provider commitment, live availability, navigation clearance or engineering approval |
| Symtropy | Deterministic scenario/optimization/physical feasibility run evidence, constraint-level outcomes, model capability and applicability | Sol-Atlas projections; Mycelix provider-scoped constraints/commitments; scenario inputs | Booking authority, provider allocation, legal permission, physical truth outside model validation |
| Mycelix | Identity/recognition, provider/community authority, offer/hold/commitment/effect receipts, reconciliation and historical stay view | Owner-native feasibility and context references as non-authoritative evidence | That model feasibility creates a provider commitment or government clearance |
| Symthaea | Read-only DecisionBrief, contradiction/uncertainty analysis, alternatives, value-of-information and human-review request | Source-bound Mycelix, Sol-Atlas and Symtropy artifacts | Provider/legal/navigation authority or external action authorization |

The logical workflow is:

1. Sol-Atlas projects source-qualified spatial and temporal context.
2. Symtropy evaluates explicitly bounded operational/resource feasibility.
3. Mycelix independently evaluates identity, recognition, provider commitment and applicable authority.
4. Symthaea may synthesize the evidence read-only, identify conflicts, compare options and abstain/request information.
5. The provider or authorized human makes the decision. Mycelix records the authorized external effect/receipt and reconciliation state.
6. The resulting evidence references can be consumed by future Sol-Atlas projections, Symtropy runs and Symthaea briefs.

This is not a mandatory synchronous call chain. Asynchronous and local operation remain first-class.

## 3. Manifest structure

The machine-readable schema defines four building blocks:

### Artifact reference

Every referenced artifact binds:
- opaque owner-native artifact ID and owner;
- exact repository, commit, path and content digest;
- artifact kind and exact profile/version/maturity where applicable;
- subject/resource references using the owner's native identifier scope;
- source/effective/observation/delivery/retrieval time references where relevant;
- claim ceiling and nonclaims.

A repository path without a commit and digest is not a reproducible reference. A commit/digest establishes integrity and identity of bytes, not source authenticity, lawful use, physical truth or issuer authority.

### Handoff edge

Each edge identifies:
- exact source artifact references;
- destination owner and purpose;
- produced artifact reference, if one exists;
- transformation/evaluator artifact reference where applicable;
- semantic fidelity disposition: preserved, lossy-declared, blocked, unknown or conflicting;
- declared losses, unresolved fields and independent-verifier reference.

The edge does not reinterpret source claims. The destination owner must validate the referenced inputs under its own contract.

### Dependency snapshot artifact

The artifact-kind vocabulary includes `dependency_snapshot` for a Mycelix-owned reference to an exact time-qualified source-dependency hypergraph snapshot (Mycelix #5923, composed with #5904 and EPI-008 #2631). This is a reference to an owner-native assessment, not an independence score.

Consumers preserve the exact observation-time scope, dependency inventory snapshot, evidence frontier and three-way outcome: SHARED_DEPENDENCY, UNKNOWN_DEPENDENCY or NO_DECLARED_COMMON_DOMAIN. The last means no common domain was declared within a complete/current inventory; it is not proof of physical or statistical independence.

Do not infer dependency semantics from artifact IDs, signatures or repository diversity. The #5972 manifest verifier checks only reference closure and type parsing; #5923 remains the semantic dependency evaluator.

### Privacy and purpose

Each manifest declares the intended audience and purpose and points to the owner-native disclosure/authorization policy references. It must not embed private customer, crew, household or itinerary data merely to make cross-stack correlation convenient.

### Execution status

The manifest separates a template, an unexecuted synthetic fixture and a frozen executed result. An executed result must bind exact-head identities and evaluator outputs. A blank/template manifest is not evidence of a completed integration.

## 4. Required integration invariants

1. **Reference, do not replicate:** keep owner-native artifacts in their owning systems; the manifest carries references and digests.
2. **No authority laundering:** Symtropy and Symthaea outputs cannot create Mycelix commitments or local/public authority.
3. **No physical-truth laundering:** a map, product, model or simulation output is bounded by its source, model and validation evidence.
4. **No temporal laundering:** preserve event/effective/observation/delivery/retrieval times, timezone context and profile version; receipt time cannot refresh stale source evidence.
5. **No spatial laundering:** preserve CRS, horizontal/vertical datum, coordinate epoch, units, coverage and every transformation reference.
6. **No semantic strengthening:** adapter fidelity and claim-promotion checks compose with Mycelix #5963/#5964.
7. **No silent loss:** unknown, unsupported, stale, partial and conflicting states remain explicit.
8. **No action side effects in analysis:** planning and cognitive analysis are read-only; provider action is separately authorized and receipted.
9. **Historical immutability:** corrections/supersession create new linked artifacts; prior decisions retain their exact input references.
10. **Independent reconstruction:** a coordinator replacement can rebuild the same historical disposition from the same authorized artifact graph; it does not gain new action authority.

## 5. End-to-end minimum demonstrator

Use a wholly synthetic coastal stay:
- one synthetic provider-owned berth commitment;
- one provider-authorized compound service constraint (e.g. shore power with separate W and Wh limits);
- one pinned synthetic harbour/spatial projection;
- one deterministic Symtropy feasibility/recovery run;
- one read-only Symthaea DecisionBrief with explicit uncertainty and abstention;
- one Mycelix independent stay disposition and a separately authorized synthetic provider effect receipt;
- one second coordinator reconstructing the historical view.

Each artifact must be generated by its owner-native implementation. The demonstrator must include positive controls and injected stale/conflict/missing-evidence cases. No live booking, payment, navigation or authority integration is implied.

## 6. Cross-stack benchmark

Report independently:
- semantic claim preservation and adapter loss;
- spatial provenance/coverage/CRS/datum/time correctness;
- physical/operational model applicability and constraint correctness;
- cognitive calibration, selective risk/coverage, contradiction detection and value-of-information;
- provider commitment safety and external-effect reconciliation;
- coordinator reconstruction and partition recovery;
- privacy/purpose minimization and reviewability;
- marginal time/code/maintenance cost to add a provider, product version or jurisdiction.

Hard gates are non-compensatory: zero false provider/authority claims, false subject/resource binding, stale-to-current promotion, false feasibility from missing required inputs, double allocation, duplicate qualified provider effects, unauthorized disclosure or loss of historical lineage.

Coverage and correctness remain separate. Synthetic results establish harness behavior only, not industry superiority or physical/legal readiness.

## 7. Standards and sandbox notes

- Pin each IHO product specification and its exact declared S-100 framework, feature catalogue, portrayal catalogue, DCEG and applicable validation artifacts. The current IHO catalogue lists S-97 Ed.2.0.0, S-98 Ed.3.0.0 and S-100 Ed.5.2.1, but individual product specs may retain earlier declared dependencies. Do not replace their dependency closure with a blanket latest version.
- DCSA Port Call conformance should be run iteratively during implementation; this handoff profile does not replace DCSA conformance.
- IALA's endorsed MCP test instance is development-only; its operator explicitly warns it must not be used operationally. Participation requires access approval and is not a production dependency.
- ISO/CD 26166 is a committee draft, not a published International Standard. Record its maturity as draft/under development if referenced.

## 8. Implementation order

1. Review this reference-only profile against existing Mycelix receipt/authorization contracts.
2. Resolve MC-JCS-1 canonical request semantics before any handoff identity is used for signing/idempotency.
3. Create one owner-native artifact reference per repository using the exact current head and digest.
4. Execute the synthetic end-to-end scenario with independent per-owner verification.
5. Replace the coordinator and reconstruct the same historical view.
6. Only then begin authorized provider/operator field work.

## 9. Claim ceiling

A valid manifest proves that its declared references and handoff edges are structurally bound. It does not prove that referenced source claims are authentic, current, physically true, lawful, recognized or operationally authorized. Those claims remain with the respective owner and competent external authority.
