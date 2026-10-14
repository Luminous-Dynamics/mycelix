# Mobility Configuration Contract V1 — Qualification Vectors

**Status:** design qualification corpus

This corpus tests whether the proposed domain-neutral contract preserves the semantic boundaries that make it reusable across transport domains.

It is deliberately implementation-independent. A future Rust crate may execute these vectors, but passing this document does not establish physical safety or regulatory compliance.

## Vector format

Each vector contains:

- scenario
- input condition
- expected semantic result
- forbidden inference

## MC-CONFIG-001 — Same design, different physical artifacts

**Scenario:** Two physical artifacts are manufactured from the same configuration.

**Input:** identical design/configuration identity; distinct manufacturing events and physical instances.

**Expected:** one configuration lineage, two distinct physical-artifact identities.

**Forbidden inference:** treating the two artifacts as one physical object or assuming identical lifecycle history.

## MC-CONFIG-002 — Same CAD, different manufacturing process

**Scenario:** identical nominal CAD is manufactured using materially different processes.

**Input:** same design artifact; different manufacturing process/material/batch metadata.

**Expected:** distinct manufacturing lineage and, where the process is engineering-significant, distinct configuration or configuration-qualified state.

**Forbidden inference:** CAD equality proves physical equivalence.

## MC-CONFIG-003 — Component substitution

**Scenario:** a bearing/component is replaced by another nominally compatible part.

**Input:** predecessor component, replacement component, compatibility claim.

**Expected:** ChangeSet with explicit substitution, affected evidence discovery, and revalidation obligations as required.

**Forbidden inference:** matching dimensions or part numbers alone establish equivalence.

## MC-CONFIG-004 — Failed test remains evidence

**Scenario:** an artifact fails a test and later passes a modified/repeated test.

**Input:** failed TestRecord followed by a later TestRecord.

**Expected:** both remain in lineage; later evidence references its own configuration, conditions, and basis.

**Forbidden inference:** later pass erases historical failure.

## MC-CONFIG-005 — Prediction remains prediction

**Scenario:** a simulation predicts 100 units and a measurement observes 100 units.

**Input:** Simulation evidence + Measurement evidence with equal numerical result.

**Expected:** separate evidence classes and separate provenance.

**Forbidden inference:** numerical equality converts simulation into observation.

## MC-CONFIG-006 — Negative evidence

**Scenario:** inspection finds a defect.

**Input:** InspectionRecord with negative finding.

**Expected:** negative evidence remains queryable and can invalidate or require review of dependent claims.

**Forbidden inference:** only successful tests are authoritative.

## MC-CONFIG-007 — Unknown dependency

**Scenario:** a change affects an artifact whose dependency graph is incomplete.

**Input:** ChangeSet + evidence with undeclared dependency.

**Expected:** Unknown or RequiresReview with explicit obligation.

**Forbidden inference:** missing dependency information means unaffected.

## MC-CONFIG-008 — Private payload, public provenance

**Scenario:** manufacturing process data is confidential.

**Input:** public provenance commitment + controlled payload reference.

**Expected:** public metadata remains verifiable without requiring payload disclosure.

**Forbidden inference:** public commitment discloses or proves access to private payload.

## MC-CONFIG-009 — Foreign identifier

**Scenario:** a STEP/AP242, QIF, supplier, or regulatory identifier is supplied where a native engineering identity is expected.

**Input:** foreign identifier + claimed native role.

**Expected:** explicit foreign binding or rejection; no silent identity substitution.

**Forbidden inference:** spelling equality makes foreign and native identities identical.

## MC-CONFIG-010 — Holochain hash substitution

**Scenario:** an EntryHash or ActionHash is supplied as a Mobility Configuration or PhysicalArtifact identity.

**Input:** Holochain hash + engineering identity role.

**Expected:** reject semantic substitution unless an explicit qualified mapping exists.

**Forbidden inference:** protocol hash is automatically an engineering identity.

## MC-CONFIG-011 — Observation vs diagnosis

**Scenario:** an operational sensor observes elevated temperature.

**Input:** Observation evidence.

**Expected:** preserve observed value and conditions; any diagnosis is a separate derived/assertional object with its own basis.

**Forbidden inference:** observation proves root cause or safety condition.

## MC-CONFIG-012 — Repair lineage

**Scenario:** a physical artifact is repaired using a replacement component.

**Input:** predecessor physical artifact state + MaintenanceEvent + replacement component.

**Expected:** repair lineage preserves history and identifies resulting state/configuration implications.

**Forbidden inference:** repair creates an unrelated artifact with no historical continuity.

## MC-CONFIG-013 — External authority

**Scenario:** an authority reviews evidence and issues a disposition.

**Input:** ExternalAuthorityReference + submitted evidence + authority disposition.

**Expected:** authority action remains attributable to that authority and distinct from internal graph state.

**Forbidden inference:** peer attestations or graph consensus can generate the authority disposition.

## MC-CONFIG-014 — Assurance metadata

**Scenario:** a profile declares a component as high criticality.

**Input:** descriptive assurance metadata.

**Expected:** stronger evidence obligations may be associated by the profile.

**Forbidden inference:** a criticality label itself proves safety or certification.

## MC-CONFIG-015 — Multimodal interface

**Scenario:** a cargo module transfers between two transport systems.

**Input:** interface definition covering geometry, mass/load, coupling, power/data, and environmental constraints.

**Expected:** interface claims become explicit dependencies with their own evidence.

**Forbidden inference:** physical interoperability follows from a shared connector name alone.

## MC-CONFIG-016 — Configuration supersession

**Scenario:** configuration B supersedes configuration A.

**Input:** explicit predecessor relationship and lifecycle event.

**Expected:** A remains historically addressable; B becomes the newer configuration according to explicit lifecycle semantics.

**Forbidden inference:** supersession deletes or retroactively invalidates all evidence produced under A.

## MC-CONFIG-017 — Revalidation obligation is not evidence

**Scenario:** impact analysis requires a new load test.

**Input:** ImpactAssessment + RevalidationObligation.

**Expected:** obligation is open until a new TestRecord/evidence satisfies it.

**Forbidden inference:** existence of an obligation means the test has passed.

## MC-CONFIG-018 — Two-domain preservation

**Scenario:** instantiate the contract once for a cargo bicycle and once for a small workboat.

**Input:** domain-neutral lifecycle objects plus separate Ground/Marine profile requirements.

**Expected:** shared identity/configuration/evidence/lifecycle semantics; profile-specific requirements remain isolated.

**Forbidden inference:** domain neutrality requires identical physical or regulatory semantics.

## MC-CONFIG-019 — Software/firmware change

**Scenario:** software controlling an electrically assisted vehicle changes while mechanical CAD remains unchanged.

**Input:** unchanged design artifact + changed software/firmware identity.

**Expected:** ChangeSet identifies software change and determines affected evidence/obligations through declared dependencies.

**Forbidden inference:** unchanged CAD proves unchanged engineering configuration.

## MC-CONFIG-020 — Retirement

**Scenario:** a physical component is retired and removed from service.

**Input:** lifecycle transition + disassembly/decommission event.

**Expected:** artifact remains historically queryable while its current lifecycle state prevents accidental representation as active.

**Forbidden inference:** historical existence implies current operational validity.

## Qualification rule

A future implementation is conformant only if it can represent all 20 cases without collapsing any forbidden inference into an accepted identity, evidence, lifecycle, or authority relationship.

The corpus must eventually be executed independently by at least two implementations or one implementation plus an independent reconstruction.

## Safety boundary

Passing these vectors establishes semantic/structural conformance only.

It does **not** establish:

- physical correctness
- structural integrity
- safety
- road legality
- seaworthiness
- airworthiness
- certification
- manufacturing conformity
- operational authorization

## Relationship to AeroCommons

This corpus intentionally mirrors the authority and identity ceilings established by AeroCommons while remaining transport-domain neutral.

AeroCommons remains the first high-assurance proving ground; Mobility Commons should not bypass its qualification gates.
