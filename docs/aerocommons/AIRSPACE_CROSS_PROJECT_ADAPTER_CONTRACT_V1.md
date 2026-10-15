# AeroCommons Cross-Project Adapter Contract V1

**Status:** Design contract / proposed interoperability seam  
**Authority ceiling:** Evidence qualification, research simulation, candidate planning, and audit only  
**Reviewed:** 2026-10-15  
**Parent work:** [AeroCommons airspace coordination PR #5024](https://github.com/Luminous-Dynamics/mycelix/pull/5024) and [AIRSPACE-COMMONS-001](https://github.com/Luminous-Dynamics/mycelix/issues/5023)

## 1. Purpose

Mycelix, Symthaea, Sol-Atlas, and Symtropy have complementary capabilities, but their current contracts must not be mistaken for one shared, qualified airspace interface. This profile defines the narrow handoff required before the projects can compose their outputs:

- **Sol-Atlas:** source acquisition, rights, provenance, observation metadata, and read-only infrastructure/geospatial projections.
- **Symtropy:** explicitly framed geometry/transform operations and candidate-route/resource-capacity optimization.
- **Mycelix AeroCommons:** admission, lifecycle/authority boundaries, trajectory and restriction evaluation, cross-provider coordination semantics, and independent checks.
- **Symthaea:** advisory inference, hypothesis generation, anomaly prioritization, and human-readable evidence summaries.

This is an adapter contract, not a common runtime, aircraft model, air-traffic-control implementation, regulatory profile, or safety case. Keep the projects independently buildable and testable. Start with versioned fixtures and adapters; do not add direct crate dependencies solely to share terminology.

## 2. Non-negotiable invariants

1. A source record, checksum string, signature, DID, consensus result, reputation value, simulation result, or resource reservation is not by itself proof of physical truth, legal authority, airworthiness, or permission to fly.
2. A claimed digest is not a verified digest. The verifier must hash the exact acquired bytes and bind the resulting receipt to the source locator/version, byte count, media type, and acquisition record. Hashing a parsed-and-reserialized document is not equivalent to hashing the acquired source bytes.
3. Every spatial value carries a named coordinate reference, axis convention, units, vertical datum, and transformation history as applicable. A frame label alone does not make two frames compatible.
4. Every time value carries an explicit time profile/scale and meaning. Source-event time, observation time, acquisition time, repository-recorded time, and validity interval remain distinct.
5. Every motion result identifies the exact model profile/version and its applicability domain, including the uncertainty treatment and unsupported dimensions.
6. Unknown, stale, contradictory, unqualified, unsupported, or over-budget inputs cannot be downgraded to a clear/no-impact result merely because another layer produced a positive recommendation.
7. Hard eligibility and conflict gates are non-compensatory. Throughput, cost, energy, emissions, fairness, or convenience cannot offset a failed hard constraint.
8. Symthaea recommendations remain advisory. They cannot grant authority, fill missing source evidence, choose a winner across legal priorities, certify a model, or override an independent checker.
9. An optimization result is exact only over its explicitly supplied candidate pool and declared objective/constraints. It is not proof of candidate completeness or globally feasible real-world operation.
10. A successful result is always scoped to the specific input manifest, source versions, model/build identities, ruleset, and checker version that were actually evaluated.

## 3. The qualified observation envelope

Before any source is converted into a flight intent, constraint, or infrastructure-capacity value, the adapter should produce a versioned envelope with these logical sections.

### A. Source and evidence identity

Required fields:

- schema identifier and version;
- stable evidence/record identifier;
- canonical source locator and source publisher/authority identifier;
- source product/schema profile and source version or release identifier, when supplied;
- acquisition state, rights qualification, and the evidence qualification scope;
- exact acquired-byte SHA-256, byte count, media type, and acquisition receipt when bytes are available;
- source event time, observation time, acquisition time, and repository-recorded time as separate optional fields;
- validity start/end, freshness policy, and last successful refresh;
- authentication/signature verification result and trust-policy version, if the source offers verifiable authentication;
- explicit missing-data and conflict status.

A structurally valid hash, URL, or publisher string is metadata only. Source authenticity, content integrity, licensing, and fitness for the particular scenario are separate checks. A public source discovery page is not evidence that the underlying dataset was acquired or that redistribution is permitted.

### B. Spatial and temporal reference

Required for geometry or motion:

- source and target frame IDs;
- named axis convention (for example ENU or NED), axis order, handedness, units, and coordinate reference profile;
- vertical datum/reference and any geoid/altitude conversion profile;
- named time profile/scale and epoch; interpolation semantics for values between time samples;
- a transform identity/version, explicit direction **T_target_from_source**, source/target frame binding, and source evidence/provenance;
- bounded transform error and its units, where known;
- half-open/closed boundary semantics for temporal validity and the exact boundary policy used by the checker.

Unknown transforms, unqualified vertical references, ambiguous axis order, unsupported temporal interpolation, and unbounded or absent transform error produce **Unresolved** (or the stricter applicable reject state), not a guessed conversion.

The airspace kernel currently uses integer millimetres in a synthetic local Cartesian frame. Keep that deterministic local-frame contract explicit. A future geographic adapter must not silently route decimal geographic coordinates through this kernel or treat a robotics-frame conversion as a verified geographic datum transformation.

### C. Model applicability

Each model evaluation binds:

- vehicle/traffic profile ID and profile version;
- model ID, model version, source commit, and build identity;
- supported operating regime and explicitly unsupported regimes;
- supported geometry dimensions and dynamics (including whether acceleration, turn dynamics, wind, energy state, wake interaction, or vehicle footprint are represented);
- coordinate/time assumptions;
- uncertainty model, horizon, and whether it is validated, synthetic, or absent;
- bounded-work policy and failure modes;
- evidence references for the applicability claim and independent qualification result;
- authority ceiling and permitted result vocabulary.

**No model match is implied by a familiar vehicle label.** In particular, the current constant-velocity kernel does not become a fixed-wing, rotorcraft, UAS, eVTOL, balloon, HAPS, high-speed, launch, or re-entry dynamics model by accepting coordinates for one of those vehicles. The current traffic catalogue correctly marks real-world archetypes as unsupported. Preserve that status until each model has its own applicability contract, tests, and independently checked oracle.

### D. Evaluation result and decision lineage

Every result should identify:

- the exact input manifest digest and each dependency/version it consumed;
- the evaluator/model build and ruleset version;
- the checker/oracle identity and result, separately from the producer result;
- classification, reasons, detected limitations, and unresolved fields;
- validity window and evaluation time basis;
- any candidate routes/resources considered and the declared candidate-pool completeness;
- authority ceiling and explicit non-claims.

Do not reduce **Conflict**, **Degraded**, **Unresolved**, **Reject**, or **RequiresAuthorityReview** into one scalar score. Retain safety/hard-constraint outcomes separately from performance metrics.

## 4. Project-specific adapter responsibilities

### Sol-Atlas → source and infrastructure evidence

Use Sol-Atlas's source-acquisition and transport-projection contracts to describe evidence and infrastructure; do not reinterpret a renderer projection as an operational route or authoritative airspace feed. Add an explicit byte-level receipt verifier before describing a declared hash as verified. Keep source rights, acquisition, content integrity, freshness, and scenario fitness as distinct states.

The current **TransportEvidenceV1** contract explicitly limits **content_sha256** to shape validation, and the source-manifest validators are structural rather than byte acquisition/authentication engines. A closely related draft in Symtropy, [Maritime Data Receipt PR #1642](https://github.com/Luminous-Dynamics/symtropy/pull/1642), already includes an offline verifier that recomputes SHA-256 over exact local artifact bytes and checks byte length. It lives in a maritime-oriented CLI today, not a shared generic library; its receipt report also deliberately remains **NotAdmitted** for navigation/physical use. Treat it as a candidate source for reusable receipt semantics and negative controls, not as completed airspace validation. Before reuse, qualify that PR's exact-head tests, define a domain-neutral receipt contract, and keep digest integrity separate from publisher authentication, rights, freshness, data quality, and physical truth.

### Symtropy → candidate generation and resource capacity

Use Symtropy's qualified frame conventions as a design reference for explicit axes and transform direction, and its shared-capacity allocator/verifier as a candidate-planning reference. Do not treat an unreviewed transform as geodetic truth.

Before aviation use, the adapter must add or verify time-scoped resource availability, expiry/freshness, conflict-independent hard constraints, demand and resource units, and candidate-generation coverage. The current batch allocator documents exactness over supplied candidate pools; the verifier checks report integrity and static-edge capacity but does not prove that candidates are complete. A runway, vertiport, charging pad, approach segment, airspace cell, or separation corridor also needs a domain-appropriate resource identity and time model.

Do not use a weighted safety-risk objective to trade safety against throughput. Hard admissibility/conflict checks precede optimization; infeasible candidates must not be rescued by a favorable score.

### Mycelix → admission, coordination, and independent qualification

Mycelix is responsible for preserving the distinction between proposed intent, a planning reservation, competent-authority authorization, physical aircraft behaviour, and independently checked evidence. It should reject unsupported model/profile combinations before invoking the trajectory kernel. The current research-only constant-velocity and planar polygon modules remain bounded submodels, not a universal collision checker.

For shared restrictions and intents, reconcile version/predecessor history and duplicates before consuming a snapshot. An identified impact requires reassessment; stale/partitioned inputs without an impact found remain degraded rather than no-impact. An empty conflict list is meaningful only within the evaluated model and qualified evidence domain.

#### Implemented admission gate on this draft branch

The first executable slice now lives in `tools/airspace-sim/src/model_applicability.rs`, driven by `fixtures/model_applicability.json`. Every `FlightIntent` carries `traffic_profile_id` and `model_profile_id`; the fields deserialize missing values as blank solely so the evaluators can return an explicit `Unresolved` report rather than quietly defaulting to a model.

The current registry admits only `synthetic_fixture_kinematics_v1` + `constant_velocity_toy_v1`, labelled synthetic-fixtures-only and not validated. It intentionally contains exactly one pair, and the runtime schema guard rejects attempts to widen it by adding a registry entry alone. Nominal trajectory evaluation, dynamic-constraint screening, and the direct polygon API stop before intersection work for a missing or unsupported pair. The 58 committed scenario/dynamic-constraint flight records and 34 polygon cases are explicitly bound to that pair; unit tests cover unknown/missing bindings, the polygon API bypass path, and all 15 traffic-diversity archetypes.

This is a **model-selection gate**, not a qualified applicability argument for an operational model. The current branch's exact-head Rust tests must pass before treating the implementation as verified; until then this is committed code with pending evidence.

### Symthaea → advisory inference and explanation

Symthaea may identify suspicious observations, summarize conflicting evidence, suggest alternative route candidates, or prioritize manual review. Persist source relations and distinct observations rather than relying only on a lossy bundled hypervector. Its recommendation should cite evidence IDs and uncertainty, and the independent Mycelix checker must recompute any machine-enforced predicate from typed input.

No confidence score, learned association, or Phi-related measure grants priority, authorization, clearance, or source trust.

## 5. Initial conformance cases

A future machine-readable cross-project fixture set should include, at minimum:

1. **Known synthetic local-frame case:** matching frame, axis convention, datum, time profile, and explicit toy model; only the declared research result is accepted.
2. **Unsupported vehicle family:** a real traffic archetype passed to the constant-velocity toy model; the result must be **Unresolved**/unsupported, never accepted.
3. **Axis/datum mismatch:** same numeric coordinates but different axis convention or vertical datum; no implicit conversion.
4. **Unverified source digest:** a digest-shaped string that differs from a fresh hash of the exact acquired bytes; reject the evidence admission.
5. **Stale restriction snapshot:** no detected intersection in the stale view; result remains degraded/unresolved.
6. **Contradictory authority or sequence history:** fork, duplicate identity with divergent payload, or invalid predecessor; do not apply last-writer-wins.
7. **Unbounded transform error:** a transform parses but has no defensible error bound for the requested decision; unresolved.
8. **Missing uncertainty contract:** trajectory coordinates exist but the model's uncertainty domain is absent or incompatible; unresolved.
9. **Capacity over-allocation:** two individually valid candidates cumulatively exceed a shared resource capacity; the independent verifier rejects the batch.
10. **Incomplete candidate set:** the allocator returns a valid result from a heuristic subset; the report must not claim network-wide optimality.
11. **Safety gate beats objective:** a hard-ineligible candidate is cheaper/faster and is still rejected before ranking.
12. **AI suggestion disagrees with oracle:** retain the disagreement as evidence; the independently recomputed hard check prevails.
13. **Source rights unknown:** a source is technically accessible but redistribution/usage rights are not qualified; block reuse outside the permitted scope.
14. **Launch/re-entry or high-speed traffic:** if a dedicated time-dependent trajectory/hazard model is unavailable, do not approximate it with the straight-line aircraft model and declare no conflict.

Each expected result should be tested with negative controls and a separately written oracle. Fixture coverage demonstrates conformance to the fixture contract only; it is not a regulatory or safety qualification.

## 6. Integration sequence

1. **Freeze the envelope and status vocabulary** as language-neutral JSON fixtures and Rust types with no new cross-repository runtime dependency.
2. **Verify source bytes and provenance** in the source-ingestion layer; distinguish digest syntax from actual digest verification and signature/authenticity checks.
3. **Qualify spatial/temporal transformations** with explicit axis, datum, epoch, transform direction, error bounds, and adversarial boundary vectors.
4. **Wire model applicability into Mycelix admission** so real-world traffic archetypes cannot reach the toy kernel without a matching, explicitly qualified model.
5. **Add a bounded planning/capacity adapter** after independent hard-constraint screening, with time-windowed capacity and an independent report verifier.
6. **Add Symthaea advisory output** last, with evidence references and disagreement reporting; never make it a hard authority dependency.
7. **Generate a per-head evidence receipt** recording repository/commit identities, fixture manifest digest, commands, exit status, and exact check results. Queued, skipped, stale-head, or unrelated checks are not passes.

## 7. External interoperability references

Use standards for their stated purpose and version, not as shorthand for safety or legal approval:

- [FAA Aeronautical Information Manual — UAS Traffic Management](https://www.faa.gov/air_traffic/publications/aim_html/chap11_section_4.html): describes UTM information sharing, flight-intent coordination, and airspace-constraint information.
- [EUROCONTROL ECHO 2](https://www.eurocontrol.int/project/european-concept-higher-altitude-operations): research on integrating HAPS and space launch/re-entry and advanced high-speed/suborbital operations with ATM.
- [AIXM 5.1.1 specification](https://aixm.aero/page/aixm-511-specification): a source model for aeronautical information where adopted by the relevant authority/provider.
- [OGC Moving Features](https://www.ogc.org/standards/movingfeatures/): standardized encoding concepts for the exchange of moving-feature trajectories.

None of these references implies that Mycelix conforms to the standard, is entitled to use a feed, has verified any source, or can perform operational separation.

## 8. Explicit non-claims

This profile does not implement source acquisition, hash/signature verification, geodetic transformations, aircraft dynamics, reachable-set analysis, certified separation logic, operational scheduling, aviation regulatory priority rules, or flight authorization. It defines the contracts and test path that should precede integration of those capabilities.
