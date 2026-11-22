# Pyrogenic-carbon source and CDR double-counting auditor v0.1

**Status:** synthetic-only registry accounting control. It does not measure pyrogenic carbon, infer natural-PyC source, verify a live registry or issue carbon credits.

## South African scientific motivation

Zhou et al. (4 November 2025), *Pyrogenic carbon contribution to tropical savanna soil carbon storage*, Nature Communications 16:9730, DOI https://doi.org/10.1038/s41467-025-64699-y, sampled 253 sites in Kruger National Park, South Africa. Using H2O2/HNO3 digestion, the study reported PyC averaging 14.08% (SE 0.36%) of surface-soil SOC, with values up to 40%. Fire regime, grass biomass, texture and rainfall influenced PyC stocks. The authors also discuss mobility by wind/runoff and method-comparability problems.

Rathnayake et al. (2024), *Quantifying soil organic carbon after biochar application: how to avoid (the risk of) counting CDR twice?*, Frontiers in Climate 6, DOI https://doi.org/10.3389/fclim.2024.1343516, review the MRV problem: standard SOC analyses include biochar/PyC; more advanced methods can distinguish some PyC from non-pyrogenic SOC, but the reviewed methods cannot sufficiently distinguish applied biochar-C from pre-existing non-biochar PyC in soil. They propose registry-based accounting and correction of overlapping SOC claims for already credited biochar-C.

Therefore:
- total SOC is not equivalent to native SOC;
- total PyC is not equivalent to applied biochar-C;
- a post-application PyC measurement does not, by itself, identify a new batch's carbon;
- a higher SOC stock after application does not automatically create a second credit when the batch carbon has already been credited through a biochar pathway.

## v0.1 architecture

The auditor consumes four evidence sets: batch/application records tied to the versioned batch passport; existing carbon-sink registry entries; a candidate SOC-stock-change claim tied to an ESM audit; and optional PyC analytical evidence with an explicit attribution state.

It does **not** estimate PyC from a spectrum or assign measured PyC to a batch. Its accounting identity is:

eligible residual SOC CDR = max(0, gross ESM SOC stock change - already-credited overlapping biochar CDR - any additional declared correction)

The declared correction must be at least the sum of active/retired biochar-credit entries whose batch was applied to the same reviewed spatial-overlap group during the SOC claim period. The registered credited quantity, not gross applied mass, is the amount corrected in this synthetic accounting example.

Where correction is smaller than required, the candidate SOC claim is blocked. Where correction equals/exceeds gross stock change, eligible residual is zero.

## Spatial and temporal overlap

v0.1 does not perform GIS polygon intersection. It requires a reviewed spatial_overlap_group_id and explicit exact_overlap_group_resolved status supplied by the accountable spatial data owner. Entries in the same group are treated conservatively as overlapping. This is a declared spatial evidence dependency, not something the auditor independently proves.

If spatial relationship is unknown, the SOC claim is blocked even when no registry entry currently matches. Different group IDs are not automatically proof of disjointness unless the spatial catalogue has attested completeness.

Only applications dated within the candidate SOC quantification window are considered for this simple v0.1 incremental stock-change correction. Earlier applications should be represented in the baseline stock; a future version will support explicit carry-over / residual-stock schedules.

## PyC evidence semantics

- not_measured: no PyC measurement is declared.
- total_pyrogenic_only: PyC is measured using a named method; the result is total PyC in the sample and is not source-specific.
- source_unresolved: the available PyC result cannot distinguish newly applied biochar from natural fire-derived PyC.

v0.1 does not accept an unqualified applied-batch-identified status. A future isotope/tracer module would require batch-specific tracer design, background distributions, analytical recovery, method-specific uncertainty and independent validation. Even a tracer result does not replace registry accounting for a credit that has already been issued.

## Registry controls

- Every registry ID and carbon tranche ID is unique.
- An active/retired biochar CDR record must match an application record and batch passport digest.
- Biochar registry quantity cannot exceed the declared applied carbon mass for that batch/application.
- A second SOC CDR record overlapping the same spatial group and time period blocks the candidate as a duplicate/overlap.
- Withdrawn/rejected registry entries do not count as credited, but remain in the immutable registry manifest.
- Registry, application, batch passport, ESM report and candidate claim identities are digest-bound.

## Qualification targets (proposed, not achieved)

- Zero SOC CDR residual while known overlapping biochar CDR is uncorrected.
- Zero conversion of total PyC into batch-origin attribution.
- Zero assumption of spatial disjointness when the overlap catalogue is unresolved.
- Exact known-truth accounting in synthetic overlap / non-overlap / under-correction / over-correction cases.
- Report total SOC stock delta, prior biochar CDR overlap and eligible residual separately.
- No carbon permanence, additionality, leakage, reversal or lifecycle emissions claim.

## Boundaries

All fixtures are manufactured. The v0.1 audit does not quantify natural background PyC, identify applied biochar analytically, establish carbon permanence, issue credits, verify real registry data, calculate lifecycle emissions, or qualify a carbon project. Real use requires registry governance, spatial overlap review, qualified MRV methodology, legal review and independent verification.
