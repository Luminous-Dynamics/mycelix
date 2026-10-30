# Terra Preta research data acquisition v0.1

**Purpose:** reproducibly acquire public source artifacts for research and independent audit. Raw datasets stay outside the Git repository.

## 1. Acquisition command

From the Mycelix repository root, use:

```sh
bash research/terra-preta/tools/acquire-public-sources.sh
```

By default, files are stored in `$XDG_DATA_HOME/mycelix/terra-preta/raw`, or `$HOME/.local/share/mycelix/terra-preta/raw` when `XDG_DATA_HOME` is unset. To choose another storage location:

```sh
bash research/terra-preta/tools/acquire-public-sources.sh /path/to/immutable/raw-data
```

Requirements: Bash, curl, jq, GNU coreutils (`stat`, `md5sum`, `sha256sum`). Figshare and Zenodo API bases can be overridden with `FIGSHARE_API_BASE` and `ZENODO_API_BASE` for an approved mirror.

The script fetches item/record metadata, downloads the named artifacts, checks publisher file size and any MD5/SHA-256 checksum exposed by the publisher, and writes local SHA-256 digests to `acquisition-manifest.tsv`. Downloading is not data validation, license approval, or permission to redistribute. Preserve the raw directory read-only after acquisition; perform transformations on copies.

## 2. Artifacts and scientific role

| Source | Artifacts | Intended role | Important limitation |
|---|---|---|---|
| [BiocharDS CSV](https://doi.org/10.6084/m9.figshare.24781764.v1) | `BiocharDS_V1.0.csv` | Global literature-extracted treatment/comparator outcomes | Study-level and arm-level crosswalk must be reviewed; not raw local soil time series |
| [BiocharDS full workbook](https://doi.org/10.6084/m9.figshare.24781737) | `BiocharDS_V1.0.xlsx` | References, variable definitions, units, and source-study reconciliation | Workbook inventory and CSV crosswalk are mandatory before confirmatory modeling |
| [Soils4Africa field survey](https://doi.org/10.5281/zenodo.17699297) | README, tableschema, field CSV | African site, land use, crop, management, erosion, and sampling context | No biochar treatment labels; publisher reports ongoing QC and disclaims completeness/accuracy guarantees |
| [Western Highveld soil NIR/MIR](https://doi.org/10.5281/zenodo.17510653) | `NWU_NIR_MIR_Dataset.csv` | South African cultivated-topsoil reference chemistry and spectral modelling | Not natural ADE soils or biochar treatment data; coordinates are incomplete |

The Soils4Africa field survey is a context dataset, not an amendment-response dataset. Its publisher reports field observations from over 15,000 locations across 33 African countries and explicitly says quality control is ongoing. The released 55-column tableschema documents plot/location, slope and erosion, land use, crop, crop residues, manure, inorganic inputs, water supply, irrigation and grazing fields. It is valuable for stratification and selecting future sampling regions, but it cannot establish a biochar effect.

The Western Highveld paper reports 790 retained composite topsoil samples. Its methods state that 13 samples with flat/blank MIR spectra were excluded; a third-party catalogue lists 803. This is consistent with a pre-exclusion count of 803 and a retained count of 790, but the exact Zenodo file must be inspected to determine which rows are actually present. Do not force an expected row count before that audit.

## 3. License and redistribution gate

- BiocharDS CSV has been reported as CC0 1.0 by the NLM dataset catalogue. The full workbook's official Figshare item has been listed as CC0 1.0. Recheck the exact version/item terms at acquisition.
- Soils4Africa's retrieved Zenodo record did not expose an explicit license value in the rights section. Treat it as `review_required`; do not redistribute until terms are confirmed with the data steward.
- The Western Highveld Data in Brief article is CC BY 4.0, while the exact Zenodo record/file terms must still be verified at acquisition. Preserve attribution and any file-level notices.

A public URL is not, by itself, evidence that every artifact can be redistributed. Keep source-specific license evidence in the manifest or source register.

## 4. Post-acquisition gates

1. Verify the generated manifest and independently recompute hashes before analysis.
2. Run the BiocharDS CSV structural audit:
   ```sh
   cargo test --manifest-path research/terra-preta/tools/biochar-audit/Cargo.toml
   cargo run --release --manifest-path research/terra-preta/tools/biochar-audit/Cargo.toml -- \
     /path/to/raw/BiocharDS_V1.0.csv /path/to/audits/biochards-audit.json
   ```
3. Inventory the full workbook's sheets, row counts, key columns, `Reference` sheet, and `Explanation and unit` sheet. Reconcile every CSV row to a source study and treatment/control arm; retain a reviewed crosswalk and unresolved cases. Do not use row index as an experiment identifier.
4. Audit Soils4Africa identifiers, country coverage, coordinate validity, missingness, duplicate plots/TSUs, categorical encodings, and quality flags. Keep observations and MIR-estimated properties in separate data classes.
5. Audit Western Highveld's exact row count, excluded spectra, sample-ID correspondence, spectral shape, units and reference chemistry. Keep the spectra and wet-chemistry labels paired by the publisher's sample identifier; split by defensible site/provider/sample dependencies.
6. Freeze reviewed dependency keys and confirmatory grouped splits before model fitting. Overlapping studies or locations must not cross training and confirmatory partitions.
7. Publish the data audit and all failed checks before reporting model performance.

## 5. What these sources can and cannot answer

BiocharDS can support study-grouped synthesis of published treatment responses once its source/arm structure is reconciled. Soils4Africa and Western Highveld can help characterize the distribution of African/South African soil and management contexts, but neither supplies biochar treatment effects. Neither one substitutes for fresh laboratory characterization or prospective local pot/field trials.

No model should be presented as evidence of South African efficacy, carbon permanence, product safety, or regulatory acceptance until those claims have their own appropriate evidence and approvals.


## 6. NEON event-scale preferential-flow benchmark sources

A new derived benchmark should be acquired before or alongside the raw NEON reconstruction:

### Published NEON preferential-flow event database (HydroShare v1.1)

[NEON Soil Preferential Flow Database, v1.1](https://hydroshare.cuahsi.org/resource/847b15cd15524b78acd58a2a8be242c1/) — DOI [10.4211/hs.847b15cd15524b78acd58a2a8be242c1](https://doi.org/10.4211/hs.847b15cd15524b78acd58a2a8be242c1), published 30 September 2025. HydroShare reports a 1,015.6 MB resource and CC BY 4.0. Its page indicates that content is externally hosted rather than stored by HydroShare, so identify the live file host, individual filenames, checksums and file-level notices before declaring acquisition.

This is a **fast-start derived-event benchmark**, not raw observations and not an independent validation network. Its declared lineage includes NEON soil-water RELEASE-2023 plus provisional observations, soil physical/chemical products, and NEON site metadata. The readme documents event start/end and rainfall, soil-moisture before precipitation/onset/peak, depth-specific apparent velocity, event flow classifications, and multiple Rosetta Ksat estimates. Version 1.1 adds Ksat point/range/standard-deviation variants.

The dataset readme says BONA, DEJU, HEAL, TOOL and BARR are not recommended, and KONA is excluded because it is closer to KONZ. These are source-specific notes; they must not be silently applied to the 2026 paper cohort or used to infer its exact site roster.

Use this derived source to:
1. inspect the event/sensor schema and distributions;
2. crosswalk event definitions, apparent velocity and Ksat columns to the 2026 paper;
3. test our eventizer on supplied smBeforePrecip and smOnsetTime summaries where comparable;
4. compare labels under alternative Ksat point/range definitions;
5. identify site/sensor mapping and missingness before attempting raw reconstruction.

Do not score the HydroShare dataset as an external holdout from the 2026 model without reconciling shared NEON site/sensor/event provenance. Do not treat its Ksat-derived flag as tracer truth. The source is tracked in neon-preferential-flow-source-plan.v0.1.json with acquisition status still marked not acquired.


A second research source set is now tracked in
[`neon-preferential-flow-source-plan.v0.1.json`](./neon-preferential-flow-source-plan.v0.1.json).
It is **metadata-reviewed only**; it contains no local file digest, downloaded
raw data, frozen site roster or verified row count.

| NEON product | Role | Temporal-resolution constraint |
|---|---|---|
| [DP1.00094.001 — soil water content and water salinity](https://data.neonscience.org/data-products/DP1.00094.001/RELEASE-2026) | Multi-depth soil-moisture series | Release-2026 landing page reports April 2016–June 2025; 1-minute and 30-minute time indices |
| [DP1.00006.001 — legacy precipitation](https://data.neonscience.org/data-products/DP1.00006.001) | Historical reproduction of the 2026 paper's cited precipitation input | Deprecated in October 2025; historical releases remain the reproduction candidate |
| [DP1.00044.001 — weighing gauge](https://data.neonscience.org/data-products/DP1.00044.001) | Gauge-volume cross-check | NEON's product-split notice describes hourly and daily aggregations; not a substitute for minute-scale rainfall intensity |
| [DP1.00045.001 — tipping bucket](https://data.neonscience.org/data-products/DP1.00045.001) | Current high-frequency event/intensity extension | 1-minute and 30-minute aggregations; crosswalk to the legacy series before comparison |
| [DP1.00046.001 — throughfall](https://data.neonscience.org/data-products/DP1.00046.001) | Under-canopy water input | Keep distinct from open precipitation; do not pool gauge types |

NEON's 6 October 2025 notification states that the legacy precipitation
product was split into DP1.00044.001, DP1.00045.001 and DP1.00046.001. Its
21 October 2025 notice says DP1.00006.001 is deprecated and no longer publishes
new data. Historical releases remain available. The 2026 paper still cites
the legacy code; do not silently replace its input with a new product.

The paper's data statement points to DP1.00094.001 and DP1.00006.001. Its
reported 33-site/586-sensor/15,554 event-sensor benchmark is a paper-level
count, not an audited count of the raw NEON files now available. Pin exact
releases, site-month availability, checksums, sensor/plot mappings and QA
filters before claiming a faithful reproduction. Use the historical product
for replication; evaluate current-product tipping-bucket inputs as a separate
extension. Throughfall is not open rainfall.

NEON's public Data Products endpoint provides product/site/month availability;
its Data Query endpoint is used to query files by product, site and date range
and requires API token authentication. Store the exact API responses and
downloaded file checksums in the acquisition manifest. Confirm current NEON
data-use terms at acquisition and preserve attribution. 


**Implementation tracker:** [TERRA-NEON-001](https://github.com/Luminous-Dynamics/mycelix/issues/5537).

**Cross-method validation tracker:** [PF-TRIANG-001](https://github.com/Luminous-Dynamics/mycelix/issues/5538).

### NEON metadata inventory (preflight)

Before downloading files, create a local text file containing one NEON site code per line (comments beginning with # are ignored). Use only the paper's reconciled site roster; do not infer it from the paper's aggregate site count.

```sh
cat > /path/to/neon-sites.txt <<'SITES'
SRER
# Add only site codes verified against the paper's supplement and NEON release inventory.
SITES

export NEON_API_TOKEN='…'  # retrieve locally; never commit or paste into chat
bash research/terra-preta/tools/inventory-neon-data.sh \
  --product DP1.00094.001 --sites-file /path/to/neon-sites.txt \
  --start 2016-04 --end 2025-06 --release RELEASE-2026 \
  --package basic --output /path/to/neon-metadata-inventory
unset NEON_API_TOKEN
```

The metadata-only preflight makes one authenticated query per site and writes the exact response JSON under metadata/, a query manifest with a digest for each response, and a normalized file inventory containing site/month/package, filename, publisher byte size, MD5, optional CRC32C and query-response SHA-256. It downloads no raw product files and does not claim that an observation row is valid or that a site is part of the paper's analysed cohort. The supplied example site is illustrative, not an endorsed paper site. Empty site/month availability is retained in the query manifest. Output files are created with restrictive permissions; the directory should still be protected as local research metadata.

### NEON acquisition command

The NEON-specific downloader is separate from the Figshare/Zenodo helper. It queries the authenticated NEON Data Query endpoint, stores its query response, verifies publisher byte size and MD5 for every file, computes local SHA-256, makes files read-only and refuses to overwrite existing artifacts:

```sh
export NEON_API_TOKEN='…'  # retrieve from your own NEON account; do not commit
bash research/terra-preta/tools/acquire-neon-data.sh \
  --product DP1.00094.001 --site SRER \
  --start 2021-01 --end 2021-12 \
  --release RELEASE-2026 --package basic \
  --output /path/to/immutable/neon-raw
unset NEON_API_TOKEN
```

This is an example, not a claim that SRER or every month has a file in RELEASE-2026. Query product/release availability before selecting site-month ranges. The tool reads the token from a protected header file, uses `X-API-Token` only for the metadata request, and does not forward it to the returned storage URL. Preserve query JSON and the acquisition manifest with the raw data.
