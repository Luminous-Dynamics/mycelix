# Mobility Configuration Contract v1

**Status:** design contract / pre-implementation

## 1. Purpose

Mobility Commons needs a domain-neutral core without pretending that bicycles, rail vehicles, boats, road vehicles, and aircraft share one engineering standard.

The core contract therefore models **engineering lineage and evidence relationships**, while transport profiles own domain-specific physics, standards, regulatory requirements, and authority workflows.

The central invariant is:

> **The design is not the vehicle.**

A physical mobility artifact is the result of a particular configuration, manufacturing process, inspection/test history, software state where applicable, operating constraints, and lifecycle history.

## 2. Layering

```
External standards / engineering systems
    STEP/AP242 · QIF · MTConnect · CAD · CAE · MES · inspection · test
                         │
                         ▼
Mobility Commons domain-neutral contract
    identity · configuration · lineage · evidence · lifecycle · impact
                         │
              ┌──────────┼──────────┐
              ▼          ▼          ▼
           Ground      Marine    Aviation
           profile     profile    profile
              │          │          │
              └──────────┼──────────┘
                         ▼
                Jurisdiction adapters
                         │
                         ▼
          External competent authorities
```

Mobility Commons is not a replacement for engineering standards or regulators.

## 3. Domain-neutral objects

### RequirementReference

Identifies a requirement without claiming that the requirement has been satisfied.

Fields:

- requirement identity
- source/authority identity
- version/effective interval
- jurisdiction or scope
- requirement text or controlled reference
- applicability conditions

### DesignArtifactReference

Identifies an engineering artifact such as CAD, drawing, schema, calculation, software/firmware artifact, or manufacturing definition.

It references existing artifact/content identity primitives rather than creating a second universal hash scheme.

### ComponentReference

Identifies a component used by a configuration.

A component may itself have:

- design lineage
- physical-artifact lineage
- supplier/manufacturer lineage
- inspection/test evidence
- substitution history
- lifecycle state

### MobilityConfiguration

Represents a frozen engineering state.

Minimum semantic dependencies:

- parent/predecessor configuration where applicable
- design artifacts
- component references
- material/process declarations
- relevant software/firmware identities
- tolerances and constraints
- operating constraints
- applicable requirements
- evidence references
- lifecycle state

A configuration is not merely a CAD revision.

### ManufacturingEvent

Records an event connecting a configuration to manufacturing activity.

It may include:

- facility/process identity
- source configuration
- machine/process identity
- material/batch identity
- operator or responsible organization
- process parameters
- deviations/substitutions
- timestamps
- inspection/test references

Private process payloads may remain controlled.

### PhysicalArtifact

Identifies a manufactured or assembled physical instance.

It must remain distinct from:

- design identity
- content identity
- configuration identity
- execution/observation identity

Two artifacts may originate from the same configuration while having different manufacturing and lifecycle histories.

### InspectionRecord

Represents an inspection observation and its evidence.

The record must preserve:

- measured/observed values
- instrument identity
- calibration evidence where applicable
- method
- conditions
- uncertainty
- operator/agent
- subject physical artifact
- result
- negative findings

### TestRecord

Represents a test event.

Prediction, test setup, measured result, interpretation, and disposition must remain separately addressable.

A passing test does not erase earlier failed tests.

### OperationalObservation

Represents an observation of an artifact or system during operation.

An observation is not automatically:

- a diagnosis
- a simulation
- a safety conclusion
- a certification
- a causal explanation

### MaintenanceEvent

Represents repair, replacement, refurbishment, remanufacture, quarantine, retirement, or disassembly.

Maintenance changes must preserve the predecessor state and identify affected configuration/evidence.

### ChangeSet

Represents a proposed or enacted engineering change.

At minimum:

- predecessor configuration
- proposed configuration
- changed artifacts
- changed components
- changed materials/processes
- changed software/firmware
- changed requirements
- changed methods/toolchains
- declared rationale

Impact analysis is a separate result; a ChangeSet does not prove its own safety.

### ImpactAssessment

Maps a ChangeSet to affected evidence and obligations.

Recommended classes:

- Unaffected
- ConditionallyValid
- Invalidated
- RequiresReview
- Unknown

Unknown must remain an explicit outcome when dependency information is incomplete.

### RevalidationObligation

A workflow obligation produced by impact analysis.

Examples:

- rerun analysis
- repeat inspection
- repeat test
- review requirement applicability
- verify manufacturing process
- obtain competent authority review

An obligation is not evidence that the obligation has been satisfied.

### ExternalAuthorityReference

References a standard, regulator, certification body, class society, railway authority, inspection authority, or other competent external authority.

It must not be converted into an internally generated certification claim merely because a graph contains attestations.

## 4. Identity boundary

Mobility Commons must reuse qualified Mycelix semantic/EPI identity primitives.

The following remain distinct:

```
Content
  ≠ Artifact
  ≠ Configuration
  ≠ PhysicalArtifact
  ≠ Execution/Observation
  ≠ SourceEvidence
  ≠ EpistemicRelation
  ≠ Holochain EntryHash
  ≠ Holochain ActionHash
```

Holochain hashes identify Holochain records/actions. They are not silently promoted into engineering identities.

Foreign identifiers such as STEP/AP242 or QIF identifiers are foreign bindings unless explicitly mapped to a qualified native identity.

## 5. Evidence semantics

Every consequential claim should retain its epistemic basis.

At minimum distinguish:

- measurement
- observation
- analysis
- calculation
- simulation
- inspection
- test
- qualification
- attestation
- derived claim

The contract must preserve:

```
prediction ≠ observation
evidence ≠ interpretation
attestation ≠ measurement
consensus ≠ physical evidence
simulation ≠ physical validation
```

Negative evidence is first-class.

## 6. Component substitution

A substitution is a ChangeSet dependency, not a metadata edit.

A valid substitution lineage should identify:

1. predecessor component
2. replacement component
3. reason
4. compatibility constraints
5. affected requirements
6. affected evidence
7. required inspection
8. required test
9. required revalidation
10. manufacturing provenance

Nominal dimensional similarity alone does not establish engineering equivalence.

## 7. Multimodal interfaces

Interfaces are first-class dependencies where transport systems interact.

Core interface categories:

- mechanical coupling
- geometric envelope
- mass/load
- power/energy
- data/control
- cargo/passenger module
- charging/fueling
- environmental conditions

An interface claim must retain whether its basis is measured, calculated, simulated, observed, or externally authorized.

This permits shared cargo modules and multimodal systems without collapsing their domain-specific assurance requirements.

## 8. Lifecycle

The domain-neutral lifecycle should support:

```
design
→ manufacture
→ inspect
→ test
→ operate
→ maintain
→ modify
→ revalidate
→ refurbish/remanufacture
→ retire/disassemble
```

Repair and reuse are lineage-preserving operations, not deletion/recreation.

A retired component can remain historically relevant without being represented as currently valid.

## 9. Assurance profiles

Assurance metadata is descriptive.

A profile may declare:

- domain
- criticality classification
- applicable standards
- required evidence classes
- required authority roles
- disclosure constraints
- jurisdiction references

It must never be a numeric or reputation-derived safety score.

The protocol must not infer:

- safety from popularity
- engineering correctness from reputation
- certification from consensus
- legality from open-source status
- physical validity from simulation
- current state from a historical record
- authority from graph connectivity

## 10. Transport profiles

The core contract is intentionally small.

Profiles own domain-specific semantics:

### Ground

Bicycles, wheelchairs, trailers, road vehicles, agricultural/industrial vehicles.

### Rail

Rolling stock, freight wagons, maintenance vehicles, infrastructure interfaces.

### Marine

Small craft, workboats, propulsion, hull/stability, marine operating constraints.

### Aviation

UAS, experimental aircraft, general aviation, certified aircraft.

### Multimodal

Containers, cargo modules, couplings, transfer interfaces, charging/fueling, interoperability.

Space systems are deliberately deferred until the common contract has survived lower-domain implementations.

## 11. Digital-thread boundary

Mobility Commons should consume and connect established engineering standards rather than invent replacements.

NIST's digital-thread work identifies STEP, QIF, and MTConnect as important pieces of standards-based product definition, manufacturing, and quality interoperability.

The boundary is:

```
Engineering system
  → standard artifact/event
  → Mobility Commons identity + evidence binding
  → configuration/change graph
  → profile-specific obligations
```

The commons adds lineage and epistemic semantics around the engineering data; it does not become a new CAD, PLM, MES, QMS, or metrology standard.

## 12. Open manufacturing

Open hardware means the design is publicly available for study, modification, distribution, making, and selling; OSHWA also emphasizes preferred editable source formats and practical manufacturability.

Therefore the commons should optimize for reproducibility without requiring every payload to be public.

Useful design-for-commons metadata includes:

- standard/available components
- repairability
- replaceable modules
- inspectable joints
- documented tolerances
- supplier alternatives
- process alternatives
- local-manufacturing capability
- open CAD/STEP/AP242
- machine-readable BOM
- test fixtures
- calibration workflows
- service documentation

Commercial manufacturing remains compatible with open hardware. The commons can support both community and commercial production.

## 13. Controlled disclosure

Public provenance and private engineering payloads are separate concerns.

A public record may prove:

- that a payload commitment exists
- who asserted it
- when it was authored
- which configuration it concerns
- what evidence class it claims to contain
- what lifecycle state applies

without revealing:

- trade secrets
- security-sensitive manufacturing parameters
- personal information
- controlled test data
- proprietary supplier details

A commitment must never be interpreted as disclosure of the committed payload.

## 14. Regulatory boundary

A jurisdiction adapter can represent:

```
requirement
→ applicability
→ required evidence
→ submitted evidence
→ authority review
→ authority disposition
```

It must not collapse that chain into:

```
graph consensus → certified
```

Modern vehicle regulation can itself require explicit configuration, software/update lineage, cybersecurity risk management, testing, monitoring, and traceability. The model therefore needs lifecycle/configuration primitives even where a particular transport profile has additional requirements.

## 15. First cross-domain qualification

Do not immediately implement every profile.

First demonstrate the same core model on two materially different domains:

1. human-powered cargo bicycle / utility trailer
2. small workboat or another substantially different physical system

For each, demonstrate:

```
requirement
→ design
→ component
→ configuration
→ manufacturing
→ physical artifact
→ inspection
→ test
→ operation
→ maintenance
→ change
→ impact
→ revalidation
```

Then deliberately introduce a substitution and verify that domain-neutral impact semantics survive.

Only after this should common abstractions be promoted from design contract to implementation API.

## 16. Authority ceiling

The Mobility Commons implementation inherits the AeroCommons authority ceiling:

**Protocol validity is not engineering validity.**

A valid protocol record can establish that a correctly structured claim was authored under the protocol's rules. It cannot establish physical correctness, safety, conformity, certification, airworthiness, road legality, seaworthiness, or operational authorization.

This ceiling must remain executable in future validation/qualification fixtures.

## 17. Implementation sequence

1. Complete AeroCommons real HDI validation fixture.
2. Reconcile/qualify Mycelix identity substrate.
3. Extract only demonstrated domain-neutral primitives.
4. Implement a Rust domain-neutral contract crate.
5. Build a cargo-bike/utility-trailer fixture.
6. Build a materially different marine or ground fixture.
7. Run substitution/change-impact corpus across both.
8. Add digital-thread adapters.
9. Add manufacturing/repair lineage.
10. Add jurisdiction adapters.
11. Integrate Symthaea only as evidence reasoning/orchestration.
12. Expand to higher-assurance transport profiles.

This order deliberately prevents the commons from becoming a speculative universal ontology before the invariants have been demonstrated.
