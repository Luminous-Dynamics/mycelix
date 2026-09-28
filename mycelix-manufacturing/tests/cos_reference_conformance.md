# COS Reference Conformance Harness — implementation note

This file is the executable-harness contract for #3334.

The harness should be implemented as deterministic Rust tests, preferably in a small dedicated test module/crate so that the COS semantic model does not become coupled to Holochain runtime state.

## Test ordering

1. negative corpus COS-N-001..016;
2. positive counterpart P-001..016;
3. formal obligation coverage check;
4. report generation.

## Required result vocabulary

- `Accepted`
- `Rejected`
- `Unknown`
- `Conflicting`
- `Stale`
- `Superseded`
- `Unauthorized`
- `Unbound`

A validator must never turn Unknown/Conflicting/Stale/Unauthorized/Unbound into Accepted by default.

## Minimal semantic records

The harness needs only the following conceptual records:

- Plan
- Requirement
- AvailabilityObservation
- Assignment
- ObservedWork
- MaterialConsumption
- ProcessObservation
- QualityEvidence
- OutputDisposition
- FailureEvent
- OutcomeObservation
- ForeignAttestation
- FRSFinding
- FRSRecommendation
- Authorization
- EffectReceipt
- ITCProjection

Each record carries source identity and temporal/provenance metadata sufficient to prevent origin rewriting.

## Evidence binding

The harness should expose explicit functions equivalent to:

- bind_plan_to_execution(...)
- bind_requirement_to_availability(...)
- bind_assignment_to_observed_work(...)
- bind_plan_to_consumption(...)
- qualify_output(...)
- recognize_foreign_evidence(...)
- project_to_itc(...)
- project_to_frs(...)
- authorize_recommendation(...)
- record_effect(...)

No implicit conversion should exist between these semantic classes.

## Report

Generate a machine-readable and human-readable report containing:

- corpus ID;
- test ID;
- formal obligation IDs;
- expected result;
- actual result;
- source/provenance;
- validity interval;
- binding/authorization references;
- refinement status;
- claim ceiling.

The report should explicitly list any COS-FV obligation without at least one negative and one positive test.

## Current implementation boundary

The existing manufacturing common crate remains the source owner for work orders, BOMs, routing, machines, MRP and scheduling. The harness must test whether those objects have sufficient evidence bindings; it must not silently reinterpret their existence/status as proof of observed production.

This document is a contract for implementation, not evidence that the harness itself has passed.
