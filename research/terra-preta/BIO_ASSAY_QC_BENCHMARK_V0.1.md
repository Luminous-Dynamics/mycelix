# Soil biological potential-activity assay qualification v0.1

**Status:** synthetic-only assay-QC auditor. v0.1 checks whether a potential enzyme-activity record satisfies a declared method-specific QC profile. It does not estimate actual in-situ process rates, biological health, microbial function, treatment effects or field benefits.

## Evidence hierarchy

The auditor keeps the following evidence levels distinct:
1. community composition (amplicons, PLFA/PLEL);
2. functional potential (metagenomic pathways, qPCR abundance);
3. potential activity under standardized assay conditions (enzyme assays, substrate-induced respiration);
4. actual process rate under declared soil / field conditions (mineralisation, nitrification, denitrification, gas flux, leaching);
5. ecosystem outcome (plant uptake, nutrient export, crop response, GHG balance, resilience).

Only level 3 is processed by this v0.1 crate. It can never promote an assay into a level-4 process-rate claim. Sequencing abundance and taxonomic diversity are not enzyme activity; gene abundance is not flux.

## Why matrix-specific QC matters

Librici et al. (3 Aug 2026), *Biochar as a Modulator of Soil Microbial Diversity and Enzymatic Function*, DOI [10.1007/s42729-026-03480-6](https://doi.org/10.1007/s42729-026-03480-6), describe context-dependent microbial and enzyme responses and note assay artefacts, including substrate sorption and co-varying pH/moisture changes.

A 2026 soil-function indicator framework for biochar highlights sample-specific calibration curves, substrate-only blanks, quench correction, dry-soil normalization and conservative interpretation (DOI [10.1007/s44378-026-00371-1](https://doi.org/10.1007/s44378-026-00371-1)).

ISO/TS 18721:2026, *Ecological soil functions — Characteristics, indicators and methods*, provides a broad method catalogue including microbial biomass, respiration, DNA/qPCR, PLFA/PLEL, mineralisation and enzyme-related indicators. It is a methods catalogue, not a single contextual scoring or causal framework: [ISO Online Browsing Platform](https://www.iso.org/obp/ui?_escaped_fragment_=iso%3Astd%3Aiso%3Ats%3A18721%3Aed-1%3Av1%3Aen).

## v0.1 input contract

Every record binds:
- experimental unit, sample and assay identity;
- declared potential-activity class, analyte, method, substrate, unit and conditions;
- rate, standard error and explicit dry-soil-mass normalization;
- calibration curve range, slope, intercept, R² and observed sample signal;
- substrate-only, matrix and sample blanks;
- spike recovery and matrix-spike recovery;
- optical/fluorescence quench fraction and whether a documented correction was applied;
- saturation / linearity flags;
- sample and method-profile SHA-256 identities;
- a method-specific acceptance profile with authority, version, digest, calibration minimum, recovery intervals and maximum uncorrected quench.

Acceptance thresholds are not universal. The method profile carries the approved limits for the chosen assay, instrument, substrate, matrix and laboratory SOP. The synthetic fixture uses manufactured limits and cannot qualify empirical evidence.

## Qualification gates

A record is qualified as `qualified_potential_activity` only when:
- the input is synthetic and all identities / digests are valid;
- the evidence class is exactly `potential_activity`;
- the method matches the acceptance profile;
- dry-soil-mass basis is declared;
- calibration slope is positive, R² meets the profile floor and the sample signal is within the calibration range;
- substrate-only, matrix and sample blanks are present and finite;
- spike and matrix-spike recoveries fall within the declared intervals;
- optical quench is below the profile's maximum. A declared correction method/digest is captured, but v0.1 does not execute correction math; any over-limit quench remains unqualified, even when a correction is declared;
- the assay is not flagged as saturated.

Failures remain explicit reasons in the report. V0.1 does not silently correct the measured activity value or pool failed replicates. Declaring a correction method is not evidence that the corrected signal was correctly computed; over-limit quench remains unqualified until a later version represents and verifies the corrected signal computation.

## What the auditor does not do

- Does not calculate enzyme activity from raw fluorescence/absorbance; calibration-specific kinetics, dilution and extraction calculations remain method-owned.
- Does not infer in-situ nutrient cycling or C stabilization from potential enzyme activity.
- Does not promote genes, taxa, microbial biomass or diversity into direct process rates.
- Does not aggregate activities into a biological-health score.
- Does not estimate treatment effects, mediation, causal pathways or field resilience.
- Does not propagate shared calibration, batch, extraction, optical-interference or serial covariance into a qualified uncertainty interval.

## Future benchmark

Compare potential assays with direct process rates on the same sample/experimental-unit hierarchy and independent site/soil/time holdouts. Evaluate matrix-specific bias, recovery, linearity, within-lab repeatability, between-lab transfer, rank stability and predictive association with actual process rates. Include stable-function/changed-community and changed-enzyme/unchanged-process controls to test redundancy and proxy failure.

## Boundaries

All v0.1 fixtures are synthetic. This is analytical QC software, not an approved laboratory method, field diagnosis, inoculant qualification, amendment recommendation or product claim. Physical sampling and wet-lab work require qualified lab/agronomic review, method SOPs, permissions, preregistration and local ecological governance.
