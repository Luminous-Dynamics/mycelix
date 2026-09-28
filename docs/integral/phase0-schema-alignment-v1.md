# Integral Phase-0 schema alignment v1

Status: Public-Draft-Reference / SourceRefined

Integral's public Technical Specifications page currently labels SPEC-DS-01 through SPEC-DS-06 as DRAFT, while SPEC-IF-01 through SPEC-IF-03 are PENDING. The Development Guide separately documents LaborEvent and distinguishes raw labor capture from weighted contribution recognition. This artifact therefore aligns to the public draft shape without treating it as a ratified API.

## Public field families captured

### SPEC-DS-01 CertifiedDesign

The public draft lists: design_id, version, bill_of_materials, production_steps, ecological_flag, itc_access_cost, design_lineage.

### SPEC-DS-02 LaborEvent

The Development Guide describes: event_id, timestamp, participant_id, coop_id, task_ref, production_plan_ref, design_ref, hours, hours_verified, skill_tier, context.

The adapter requires event identity, participant/task/plan/design references, positive claimed hours, positive verified hours, verified hours <= claimed hours, and evidence current at the event timestamp.

Critically: hours != hours_verified. Neither is silently invented.

### SPEC-DS-03 MaterialConsumptionEvent

The public draft lists: material_id, quantity_consumed, ecological_flag, source, production_ref.

The reference record adds event identity/timestamp wrapper so provenance and temporal validity remain explicit at the evidence boundary.

## Semantic mapping

CertifiedDesign -> COS production basis
CertifiedDesign != production execution

LaborEvent -> source-owned observed labor
LaborEvent != ITC ledger entry

MaterialConsumptionEvent -> source-owned consumption
MaterialConsumptionEvent != ITC ledger entry

FRS signal -> derived projection
FRS signal != COS source observation

## Why this is a material improvement

We now bind the public Integral field families directly rather than using generic labor_hours/material_ref placeholders. We preserve event identity, timestamps, participant/task/plan/design lineage, claimed vs verified hours, material quantity/source, and local/foreign provenance.

## Qualification ceiling

Public-Draft-Reference / SourceRefined

Not claimed: ratified Integral API compliance, complete COS runtime, ITC issuance, FRS diagnosis, physical production, safety qualification, governance authority, or Integral validation.

## Next closure

The next high-value seam is SPEC-IF-01 OAD -> COS once its pending contract is available: version negotiation, authentication, retry, idempotency, error semantics and semantic admission. Until then, the neutral seam_profile reference is the compatibility test rather than an asserted Integral implementation.
