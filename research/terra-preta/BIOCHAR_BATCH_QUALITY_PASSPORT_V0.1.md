# Biochar batch quality passport auditor v0.1

**Status:** synthetic-only batch lineage and quality-profile audit. The auditor does not certify a real product, compute a carbon-removal credit, establish ecological sustainability or determine field suitability.

## Why batch identity matters

A product description such as "wood biochar" is not an analytical result. Feedstock, prior use, contamination history, processing conditions, mixing, sampling and analytical methods all affect what is actually in a delivered batch.

The 2025 European Biochar Certificate (EBC v10.5E) specifies batch identity, representative sampling and batch-linked analytical properties. The International Biochar Initiative (IBI) v2.1 separates required utility properties and toxicant assessment from optional advanced analysis. Commission Delegated Regulation (EU) 2026/285, adopted under the EU Carbon Removals Certification Framework, requires batch-level laboratory testing and representative sampling for its BCR methodology. The EU rules apply to that certification route; they are not automatically universal South African agronomic law.

This tool builds an interoperable evidence passport around those engineering principles without copying another scheme's thresholds or claiming certification.

## Three non-interchangeable propositions

1. **Product quality conformity:** does this exact batch meet this exact versioned, end-use-specific quality profile, under the stated analytical basis and uncertainty?
2. **Carbon-removal methodology eligibility:** is the complete feedstock-to-production-to-transport-to-application lifecycle admissible and net-removal-positive under a separately identified MRV methodology? Not assessed by v0.1.
3. **Agronomic suitability:** is this batch appropriate for this soil, crop, climate, application route and intended outcome under evidence from suitable trials? Not assessed by v0.1.

Passing proposition 1 does not imply propositions 2 or 3.

## Batch identity and lineage

A batch is bound to:
- unique batch and production-unit identities;
- source feedstock lots with prior use, dry mass, origin, contamination-screening and ecological-retention evidence;
- common processing record and process conditions;
- input/output mass accounting;
- a defined representative-sampling plan;
- sample, seal, chain-of-custody and retention evidence;
- laboratory report and exact analytical results;
- a frozen quality-profile identity and SHA-256.

Changed production unit, feedstock mix, or common processing conditions require a new batch or an explicit new sub-batch/profile review. A supplier catalogue description does not override batch-specific test results.

## Profile-driven requirements

The quality profile carries its own authority, jurisdiction, intended end use, version, effective date, process constraints, sampling minima, retention requirements, analytical methods, units, dry-matter basis and analyte limits.

No universal contaminant threshold is hardcoded in the auditor. The profile owner must select the applicable legal, certification or procurement profile. A synthetic fixture profile is only a manufactured test oracle.

## Conservative result comparison

For a maximum limit (L), quantified result (x) with expanded uncertainty (U):
- pass if (x + U le L);
- fail if (x - U > L);
- otherwise indeterminate.

For a minimum limit:
- pass if (x - U ge L);
- fail if (x + U < L);
- otherwise indeterminate.

A below-LOD / below-LOQ result may only support a maximum criterion when its stated censoring bound is at or below the profile limit. It cannot prove a minimum property. Missing required result, incompatible unit/basis/method, inadequate laboratory scope, unknown feedstock status, unrepresentative sampling or missing custody blocks an overall pass.

## Synthetic v0.1 output

The auditor returns separate feedstock, process, sampling and analytical gates, per-analyte comparisons, and one synthetic profile-conformity status:
- `synthetic_profile_pass`
- `synthetic_profile_fail`
- `synthetic_profile_indeterminate`
- `incomplete_or_blocked`

The report always returns:
- carbon-removal status = `not_assessed_by_v0_1`
- agronomic-suitability status = `not_assessed_by_v0_1`

It never emits "certified", "safe for every soil", "carbon negative", or "recommended application rate".

## Industry comparison and next benchmark

This is not a replacement for EBC / WBC / IBI certification or an accredited laboratory. Its proposed differentiator is machine-readable evidence continuity and explicit fail-closed claim separation across multiple standards and jurisdictions.

A future independent benchmark should compare:
- profile version/criteria correctness against the chosen scheme's official test vectors;
- batch homogeneity and sampling bias using replicate stratified samples;
- analytical repeatability, recovery, LOD/LOQ and uncertainty propagation;
- detection of feedstock/process drift and batch splitting attacks;
- end-use profile mapping errors;
- lineage completeness and duplicate/counterfeit batch resistance;
- lifecycle accounting completeness under an external MRV method;
- cost/time per qualified batch.

## Scientific / standards sources

- European Biochar Certificate, *Guidelines for a Sustainable Production of Biochar*, v10.5E, 14 August 2025: https://www.european-biochar.org/media/doc/7/ebc_en_10_5_red.pdf
- International Biochar Initiative, *Standardized Product Definition and Product Testing Guidelines for Biochar That Is Used in Soil*, v2.1: https://biochar-international.org/biochar-standards/
- Commission Delegated Regulation (EU) 2026/285, EU CRCF permanent carbon removal methodologies, 17 April 2026: https://eur-lex.europa.eu/eli/reg_del/2026/285/oj/eng

## Boundaries

All fixtures are manufactured. No live feedstock or biochar batch is admitted. A valid synthetic profile pass is not a real quality certificate, regulatory determination, carbon-removal credit, safety assessment or site-specific agronomic recommendation.
