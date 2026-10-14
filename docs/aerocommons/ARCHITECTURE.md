# AeroCommons Architecture

Status: research architecture / non-certifying prototype
Tracking: AEROCOMMONS-001

AeroCommons explores whether open engineering and distributed manufacturing can become more trustworthy by making the engineering lineage reproducible rather than merely publishing CAD files.

## 1. Boundary

AeroCommons does not replace aviation authorities, certification standards, qualified engineering judgment, conformity inspection, or flight-test authority.

It provides an evidence substrate around them.

- Symthaea: represent, simulate, compare, detect contradictions, estimate state, and propose.
- Mycelix/Holochain: identity, provenance, evidence lineage, attestations, contestability, configuration history, and durable receipts.
- Aviation ecosystem: requirements, accepted means of compliance, engineering approval, inspection, certification, and operational authority.

The system must preserve the distinction between model output and measurement; evidence and interpretation; attestation and physical truth; governance and engineering authority; and open source and regulatory approval.

## 2. Evidence graph

The fundamental object is not an aircraft file. It is a graph:

Requirement -> Analysis / Test / Inspection -> Evidence -> Claim -> Attestation -> Configuration -> Change -> Impact analysis -> New evidence

A configuration change must be able to identify what it invalidates.

Core records:
- AeroArtifactV1
- AeroConfigurationV1
- AeroRequirementV1
- AeroAnalysisV1
- AeroSimulationV1
- AeroMaterialBatchV1
- AeroManufacturingRecordV1
- AeroInspectionV1
- AeroTestPlanV1
- AeroTestResultV1
- AeroEvidenceV1
- AeroAttestationV1
- AeroChangeSetV1
- AeroDeviationV1
- AeroMaintenanceEventV1
- AeroOperationalObservationV1
- AeroPredictionErrorV1
- AeroDecisionReceiptV1

These names are intentionally domain-oriented but should be designed so the underlying evidence protocol can later serve robotics, energy, industrial equipment, and other physical infrastructure.

## 3. Content-addressed identity

Every consequential artifact should be independently addressable by schema/version, canonical payload digest, producer identity, parent configuration, toolchain identity, source/evidence references, creation time, validity scope, and supersession/revocation state.

Derived artifacts should retain links to the exact inputs and tool versions used to produce them.

A result without reproducible inputs is evidence of an observation, not automatically reproducible engineering evidence.

## 4. Configuration lineage

A change should produce an explicit impact record:

ChangeSet -> affected components -> affected requirements -> affected analyses -> invalidated evidence -> required re-analysis -> required inspection -> required test -> certification/conformity review surface -> resulting configuration

This should be machine-readable.

The goal is to make a fork behave like a configuration lineage, not merely a Git branch.

## 5. Distributed manufacturing

A future manufacturing network could represent capabilities without making a single authority the global owner.

A manufacturer may attest available processes, machine capability, material capability, achievable tolerances, inspection capability, calibration state, and historical production records.

A particular build then binds:

Design -> Process -> Material -> Machine/Tooling -> Operator/Organization -> Inspection -> Test -> Configuration

The system should preserve the distinction between a claimed capability and evidence demonstrating that capability.

## 6. Symthaea interface

Symthaea should receive and emit typed engineering evidence rather than a generic knowledge blob.

Example flow:

Requirement -> Symthaea constructs analysis plan -> simulation/model execution -> predicted result + uncertainty + model validity domain -> comparison with measurement -> prediction error -> proposed design change -> Mycelix evidence receipt

Symthaea must never silently convert a prediction into an authoritative fact.

## 7. Mycelix interface

Mycelix should provide identity, authorization context, provenance, evidence references, attestation, contest/dispute, lifecycle obligations, supersession/revocation, and durable decision/evolution receipts.

The Holochain model is a better fit for this than forcing all engineering events onto a global blockchain: manufacturing and inspection naturally originate at separate organizations and agents, while cross-party verification can reference their signed evidence.

## 8. Certification boundary

AeroCommons can represent certification artifacts and the evidence supporting them.

It must not claim that a cryptographic receipt is itself a certificate of airworthiness.

Intended relationship:

Regulatory requirement -> accepted means of compliance -> engineering/test evidence -> qualified review -> certification artifact

The protocol makes this lineage inspectable.

## 9. First vertical slice

Do not begin with a complete aircraft.

Use a non-flight-critical engineering fixture or component.

Reproduction loop:
1. Publish component definition.
2. Publish requirements.
3. Publish analysis inputs and toolchain identity.
4. Produce analysis result.
5. Publish manufacturing process.
6. Produce build record.
7. Perform independent inspection.
8. Perform a deterministic physical test.
9. Record evidence and attestations.
10. Create a configuration receipt.
11. Fork the design.
12. Demonstrate automated impact analysis.
13. Verify which prior evidence remains valid.
14. Verify which evidence must be regenerated.

The success criterion is independent reproducibility, not visual similarity.

## 10. Safety invariants

1. Consensus never substitutes for physical evidence.
2. Reputation never substitutes for inspection.
3. AI prediction never substitutes for measurement.
4. Historical outcomes never silently become safety rules.
5. Superseded evidence remains discoverable as superseded.
6. Revocation does not erase provenance.
7. A decision receipt does not grant physical authority.
8. Open licensing does not imply regulatory approval.
9. Sensitive/export-controlled information is not assumed to be public.
10. Flight-critical autonomy requires an explicit, separately justified assurance path.

## 11. Existing open tooling

AeroCommons should compose mature tools rather than recreate them.

NASA's OpenVSP already provides open-source parametric aircraft geometry and analysis capabilities. OpenMDAO and other open engineering tools can occupy analysis/optimization roles.

The AeroCommons layer should bind their outputs into reproducible engineering lineage.

## 12. Initial research questions

- How should evidence schemas map to FAA/ASTM certification artifacts?
- Which DO-178C, DO-254, and ARP-4754A-style assurance concepts can be represented as provenance without claiming equivalence?
- How should uncertainty and model validity domains be encoded?
- How should negative evidence and failed tests propagate?
- What constitutes reproducible manufacturing evidence?
- How should multi-party conformity work in an agent-centric architecture?
- What is the minimum Symthaea engineering-twin interface?
- Which non-critical component provides the best first demonstration?

## 13. Long-term hypothesis

The aircraft is a proving ground.

The deeper objective is a reusable protocol for forkable, verifiable physical engineering:

design -> analysis -> manufacture -> inspect -> test -> certify -> operate -> maintain -> observe -> learn

That could eventually support aircraft, robotics, energy infrastructure, industrial machinery, and other complex physical commons without pretending that software governance can replace physics or professional responsibility.