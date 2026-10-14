# AeroCommons Authority Ceiling Qualification V1

## Status
Design-qualification corpus. This file is implementation-independent and is intended to gate future Holochain integrity implementations.

The corpus tests a stronger property than schema validity: **AeroCommons must be able to demonstrate what it refuses to believe.**

## Authority ceiling
A Holochain validator may establish deterministic protocol validity and addressable dependency relationships. It must not thereby establish physical correctness, safety, airworthiness, certification, manufacturing conformity, or truth of a measurement/simulation beyond the declared evidence semantics.

Holochain validation is deterministic: validation outcomes for a given operation must not depend on mutable current state, and addressable dependencies that cannot be retrieved produce unresolved validation rather than an invented result. The valid-record primitive provides an inductive validity boundary around the referenced CreateRecord; it does not automatically validate later operations associated with that record.

## Vector contract
Each vector in AERO_AUTHORITY_CEILING_V1.json contains:
- boundary: the semantic boundary under test;
- input: adversarial condition;
- expected_validator_result: Valid, Invalid, or Unresolved;
- engineering_status: the engineering-layer status that must remain after protocol validation;
- forbidden_inference: the authority-amplifying interpretation the implementation must never derive.

A Valid result means only that the protocol operation conforms to the protocol's deterministic rules. It is deliberately possible for a vector to be protocol-valid while the engineering interpretation remains Unknown or unchanged.

## Boundary families
### Identity leakage
EntryHash and ActionHash are Holochain protocol identities. They must not silently become Artifact, Configuration, Execution, or EpistemicRelation identities.

### Epistemic leakage
Attestations, simulations, predictions, and numeric values retain their declared epistemic roles. Similar data shapes do not make them interchangeable.

### State leakage
Current link collections and mutable projections are not deterministic validation dependencies.

### Authority leakage
Reputation, consensus, and current certification projections may be useful application-layer information, but they cannot become physical evidence or certification authority merely because they are represented in the same graph.

### Dependency leakage
A missing addressable dependency is not evidence of falsity. It is an unresolved validation dependency and must remain retryable.

## External engineering boundary
Engineering payloads such as STEP/AP242 and QIF remain external engineering representations. NIST's digital-thread work treats STEP, QIF, and related manufacturing/quality standards as lifecycle interoperability infrastructure, including conformance testing and traceability. AeroCommons should bind those artifacts into provenance rather than replace their engineering semantics.

## Qualification rule
A future implementation passes this corpus only if it preserves the declared distinction at the boundary being tested.

Passing does **not** mean:
- an aircraft is safe;
- a component is conforming;
- a simulation is true;
- a measurement is correct;
- a design is certified;
- a configuration is airworthy.

It means the protocol did not accidentally acquire one of those authorities.

## Exit condition
The corpus becomes a prerequisite for the first AeroCommons Holochain graph implementation. Any future zome schema should map each vector to a deterministic test and retain the same authority ceiling.

The next implementation step should be a small harness that consumes this corpus and verifies that concrete integrity-zome types cannot cross these boundaries accidentally.
