# Water Purity Evidence Separation v0.1

**Issue:** Mycelix #6100  
**Architecture parent:** Mycelix #3233  
**Related metrology:** #6034, #6057  
**Date:** 2026-11-12

## Purpose

Separate raw source observations from standards-bound derived assessments in Mycelix water-purity. The current water-purity zome source is audited in #6100: QualityReading combines physical sample fields with a caller-supplied potability score and WHO/EPA booleans, and the coordinator uses those booleans to derive a potability result.

This directory is a documentation/schema-only starter contract. It does not change the live zome or rewrite historical records.

## Core invariant

`Observation != Assessment != Certification != Authorization`

A sample can contain measurements. A standards assessment must bind those exact observations, an immutable profile revision, a jurisdiction/intended use, analytical methods, required analyte coverage, uncertainty and decision rule. A profile-specific disposition is one of Compliant, NonCompliant, Indeterminate or Unsupported. A universal score/boolean is not a substitute.

## Contents

- schema/raw-observation.schema.json — source measurements and sample provenance only. No pass flags, potability score or safety conclusion.
- schema/derived-assessment.schema.json — exact input observation IDs, profile identity/revision, methods, thresholds, uncertainty and per-analyte outcome.
- fixtures/observation-*.json — synthetic observation records.
- fixtures/assessment-*.json — corresponding synthetic Compliant, NonCompliant and Indeterminate examples.

All fixtures intentionally use fictional analytes marker_alpha / marker_beta, fictional units and a fictional profile. No WHO, EPA, SANS 241 or other real standards values are included.

## Decision example

For a synthetic upper-limit profile, a measured value is Compliant only when its entire declared uncertainty interval is within the synthetic limit; NonCompliant when the entire interval exceeds it; Indeterminate when the interval overlaps or required evidence is absent. This is a simplified contract example, not a universal laboratory decision rule. A real method profile must define its own handling of uncertainty, detection limits, censored values, sample quality, rounding and measurement uncertainty.

## Integration rules

- Reuse EPI / derived-assessment architecture (#3233).
- Reuse instrument capability, calibration and sample-level evidence (#6034).
- Reuse condition-applicability and timebase evidence (#6057 and related owners).
- Preserve historic source records; use a versioned migration and interpretation path.
- Do not treat mass-balance arithmetic, sensor success, or a laboratory result as automatic water-safety certification.
- Keep intended use explicit: drinking water, domestic hygiene, irrigation/reuse, process water, wastewater process monitoring and environmental discharge are different scopes.

## Negative controls to add to executable evaluator

1. missing required analyte -> Indeterminate;
2. user-supplied WHO/EPA flag true while a required measurement fails -> NonCompliant;
3. threshold uncertainty overlap -> Indeterminate;
4. missing/out-of-date profile or method -> Unsupported / Indeterminate;
5. stale calibration or outside-envelope instrument -> Indeterminate;
6. missing detection limit or invalid sample chain -> Indeterminate;
7. unit/analyte mismatch -> reject;
8. below-detection result coerced to zero -> reject;
9. wastewater effluent passed to potable-water profile -> Unsupported;
10. model/Symthaea output promoted to raw source observation -> reject.

## Evidence status

The schemas and fixtures are synthetic contract artifacts only. No regulatory profile, water-quality result, SANS/WHO/EPA conformity, potable-use decision, lab accreditation or public-health claim is established by this folder.
