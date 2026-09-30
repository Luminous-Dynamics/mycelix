# AeroEvidenceV1

Status: research contract / non-certifying
Purpose: define a minimal, machine-readable boundary between an engineering claim and the evidence that supports it.

## Design principle

An evidence record does not establish physical truth merely because it is signed, replicated, endorsed, or highly trusted. It records a claim, its basis, its production context, its scope, and its current lifecycle state.

## Required fields

`evidence_id`
: Content-derived identifier for this evidence record.

`schema`
: Exact schema identifier and version.

`claim`
: The proposition being asserted. Claims must be specific enough to test or inspect.

`claim_kind`
: One of measurement, analysis, simulation, inspection, test, calculation, qualification, attestation, observation, or derived.

`configuration`
: Exact design/configuration identity to which the claim applies.

`subject`
: Component, assembly, material, process, software, or aircraft entity to which the claim applies.

`basis`
: References to the observations, artifacts, tests, analyses, or source records from which the claim derives.

`method`
: Method/procedure identifier and version.

`inputs`
: Exact input artifact identifiers and relevant environmental/setup parameters.

`toolchain`
: Software, hardware, solver, compiler, instrument, calibration, or process identities needed to reproduce the result.

`result`
: Canonical result payload, including units and reference frame where applicable.

`uncertainty`
: Known measurement uncertainty, numerical error, confidence interval, or other explicitly applicable uncertainty information. If unknown, that must be explicit.

`validity_domain`
: Conditions under which the evidence is intended to apply. Examples include load range, temperature range, material batch, geometry revision, software version, or operating envelope.

`producer`
: Identity of the agent or organization that generated the evidence.

`review`
: Independent review/reproduction records, if any. Review is not equivalent to physical validation.

`lifecycle`
: active, superseded, disputed, revoked, or withdrawn, with a reason and lineage.

`provenance`
: Parent records, source records, timestamps, and signatures/receipts required to reconstruct lineage.

## Evidence semantics

### Measurement

Records an observation made with an identified instrument/procedure. Instrument calibration and setup should be linked where material.

### Analysis

Records a result obtained from a defined engineering method. Inputs, assumptions, boundary conditions, and method version are part of reproducibility.

### Simulation

Records a computational model execution. A simulation is not a measurement and should expose model validity assumptions and numerical uncertainty.

### Inspection

Records examination against defined acceptance criteria. The criteria themselves must be addressable.

### Test

Records a physical or software test execution, including procedure version, setup, instrumentation, environmental conditions, result, and disposition.

### Attestation

Records an agent's statement about evidence or process. An attestation adds provenance and accountability; it does not transform an unsupported claim into physical fact.

### Derived

Records a deterministic transformation of prior evidence. All source evidence and transformation logic must be addressable.

## Negative evidence

Failures are first-class records.

A failed test, rejected inspection, invalid simulation, out-of-calibration measurement, or contradictory observation must remain discoverable and linked to the affected configuration.

Negative evidence must never be silently replaced by a later successful result.

## Validity and invalidation

Evidence is scoped, not globally true.

A change to configuration, method, material, tooling, environment, software, or assumptions may invalidate only a subset of prior evidence.

Therefore invalidation should be represented as an explicit relation:

`ChangeSet -> Evidence -> invalidation_reason -> required_action`

Examples of required actions:
- re-run analysis;
- repeat test;
- repeat inspection;
- obtain engineering review;
- establish that the evidence remains valid under the changed scope.

## Prediction versus observation

Symthaea-generated predictions must carry a distinct claim kind and explicit model identity.

A prediction can be compared with a later observation:

`Prediction -> Observation -> PredictionError`

The error record should preserve both original records rather than rewriting the prediction.

## Independent reproducibility

A reproduction should identify:
- reproducer identity;
- source evidence set;
- toolchain/environment;
- execution or physical procedure;
- result;
- difference from the original;
- disposition.

A reproduction result is itself evidence and must not overwrite the original.

## Lifecycle rules

Supersession:
: A newer record replaces the operational use of an older record while preserving its historical existence.

Revocation:
: The producer or authorized authority declares a record unreliable or unauthorized for a specified reason. Revocation does not erase provenance.

Dispute:
: A party records a challenge to a claim. The original claim remains visible while the dispute is resolved.

Withdrawal:
: A producer retracts a record, with reason. Historical lineage remains intact.

## Canonicalization

Serialization must be deterministic before hashing/signing.

Canonicalization must define:
- field ordering;
- numeric representation;
- units;
- timestamp representation;
- optional-field semantics;
- Unicode normalization;
- binary artifact references;
- collection ordering;
- schema version.

Do not hash a human-readable rendering whose formatting can change independently of semantics.

## Privacy and controlled disclosure

Not all evidence can be public.

Support references to restricted evidence without publishing its contents. The public record should be able to prove that a claim depends on controlled evidence without exposing the controlled payload.

Potential disclosure classes:
- public;
- consortium;
- authorized reviewer;
- regulator/authority;
- restricted/export-controlled;
- private.

## Minimal conformance tests

An implementation claiming AeroEvidenceV1 support should demonstrate:

1. deterministic canonicalization;
2. stable content identity;
3. complete configuration binding;
4. explicit uncertainty semantics;
5. explicit validity domain;
6. preservation of negative evidence;
7. explicit supersession/revocation/dispute;
8. prediction/observation separation;
9. independent reproduction without mutation of the original record;
10. verification of lineage from claim to source evidence.

## What this contract intentionally does not do

- It does not define aircraft certification requirements.
- It does not replace ASTM, FAA, EASA, or other authority requirements.
- It does not prescribe a particular CAD/CAE/CFD tool.
- It does not make an AI model an engineering authority.
- It does not assign safety significance from reputation alone.
- It does not require a global blockchain.

The purpose is narrower: make the provenance and epistemic boundary of engineering evidence explicit enough that independent systems can interoperate.