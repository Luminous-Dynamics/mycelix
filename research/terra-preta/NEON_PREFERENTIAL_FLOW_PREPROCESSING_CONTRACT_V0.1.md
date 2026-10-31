# NEON Preferential-Flow Raw Preprocessing Contract v0.1

**Purpose:** freeze the reported 2026 Li et al. preprocessing steps before parsing raw NEON packages. This is a reproduction contract and a list of unresolved source-code crosswalks; it is not yet an executed data pipeline. No raw NEON site/month inventory or event rows are claimed.

## 1. Source identity: cite the paper's data, not whichever release is newest

The 2026 paper's Availability Statement cites these NEON datasets:

| Product | DOI cited by the paper | Role |
|---|---|---|
| DP1.00094.001 | [10.48443/7EXD-N727](https://doi.org/10.48443/7EXD-N727) | 1-minute soil moisture |
| DP1.00006.001 | [10.48443/DXDR-W376](https://doi.org/10.48443/DXDR-W376) | Primary/secondary precipitation input product in the historic product system |
| DP1.00096.001 | [10.48443/T70Z-NP08](https://doi.org/10.48443/T70Z-NP08) | Megapit soil physical/chemical data |
| DP1.10047.001 | [10.48443/r517-kg13](https://doi.org/10.48443/r517-kg13) | Distributed initial soil characterization |

The citation year and dataset DOI are not by themselves an exact NEON API release tag. Resolve each DOI to its exact data release and preserve the DOI-to-release relationship before acquisition. Do not substitute RELEASE-2026 for the cited historical dataset without a separate version crosswalk. The current tipping-bucket DP1.00045.001 and throughfall DP1.00046.001 are current-product extensions, not silent replacements for the paper's cited legacy precipitation product.

## 2. Soil-water preprocessing reported by Li et al. (2026)

1. Start with 1-minute volumetric soil-water observations and preserve raw files, calibration metadata, sensor IDs, sensor position/depth and internal quality flags.
2. Exclude observations with raised internal quality flags.
3. Exclude volumetric soil-water observations exceeding the maximum porosity of the entire soil profile across all soil pits. The exact source of the maximum-porosity reference and its sensor/profile linkage must be retained.
4. Resample to 10-minute bins using the median of good observations within each bin.
5. If a 10-minute bin contains fewer than seven good 1-minute observations, mark that bin missing (not zero). Preserve the count of good source observations.
6. Linearly interpolate soil-moisture gaps shorter than six hours in the resampled series. Preserve original missingness, interpolation start/end, interpolated status and the method; do not overwrite raw measurements.

Required audit columns include raw observation count, good count, rejected count by QA reason, median value, maximum-profile-porosity cutoff, bin status (`observed_median`, `missing_below_7_good`, `linear_interpolated_gap_lt6h`, `missing_gap_ge6h`, or `missing_edge`), and source-file digest.

## 3. Precipitation preprocessing reported by Li et al. (2026)

Two precipitation measurement paths are described:

- **Primary:** 5-minute weighing gauge, within 0.5 km of the tower infrastructure at core sites, stated precision 0.05 mm. Apply the internal final quality flags.
- **Secondary:** 1-minute tipping bucket at tower top, above canopy at 8–60 m, stated precision 0.001 mm. Apply plausible-value-range flags and NEON science-review flags for known unfavorable conditions.

Use whichever source is available at a site/time. Where both primary and secondary sources are available, prefer the higher-frequency secondary tipping-bucket data. Never average the two gauges or silently stitch them together inside one event without an explicit source-selection record. Retain a per-bin source selector, gauge ID/type, good count and QA reasons.

The paper says filtered precipitation is aggregated to 10-minute intervals and assigns zero when the original 1-minute or 5-minute data contain fewer than five good QA-passing measurements. That zero-fill rule is unusual because it can convert low-observation coverage into a dry bin. Reproduction must implement the reported rule in a separately labelled paper-reproduction track while also retaining a missing-preserving sensitivity track. Do not erase the distinction between observed zero rainfall and paper-rule zero imputation.

The exact aggregation operator and source-specific time-bin alignment must be confirmed against the cited 2025 preprocessing/code before claiming bitwise reproduction. The 2026 text says “aggregated” but does not fully specify the code-level precipitation summation/alignment operator. Preserve source interval depth and derived 10-minute value so alternative operators can be compared.

Required audit columns include source product/release, gauge ID/type, input time span, source observation count, good count, rejected count by flag, observed depth, derived 10-minute depth, `observed`, `paper_zero_imputed_lt5_good`, or `missing_preserved` status, and source-file digest.

## 4. Event and response definitions reported by Li et al. (2026)

- A precipitation event begins at a nonzero 10-minute precipitation pulse.
- Additional precipitation within the following six hours is part of the same event.
- Maximum event duration is five days.
- Soil-moisture response onset is the timestamp at which the soil-moisture increase is at least 0.01 cm³/cm³. The paper states that the threshold can delay onset relative to the first physical response, but reduces sensor-noise detections.
- Apparent event response velocity is sensor depth divided by event response time (precipitation onset to soil-moisture response onset). It is a sensor-influence-volume celerity, not direct pore-water velocity.
- Only sensors at 10–100 cm are retained for the 2026 PF threshold analysis.
- PF proxy class for the 2026 model is velocity-threshold based: velocity greater than the maximum Rosetta3-derived saturated hydraulic conductivity for the soil thickness above the sensor.

The 2026 paper explicitly says it does **not** use NSR for its binary PF label. The profile-flow NSR diagnostic in this repository is a complementary method-evaluation track, not part of the paper's exact binary target.

The 2025 Li et al. multi-ecoregion paper is a separate method track: it uses both NSR and velocity-threshold approaches and describes additional analysis choices (including a 2 mm event-rainfall minimum). Do not transfer that 2 mm filter into the 2026 replication unless the 2026 supporting information or author code confirms it.

### 4.1 Companion 2025 site-probability target

Li et al. (2025), DOI [10.1029/2025GL118045](https://doi.org/10.1029/2025GL118045), defines site-level PF occurrence probability separately for VT and NSR as the fraction of qualifying rainfall events (cumulative rainfall at least 2 mm) for which that method reports PF among events with usable soil-moisture data. It uses 40 sites across 17 ecoregions and a 24-site/16-site Random Forest training/testing split. VT uncertainty is represented by maximum, median, and median-plus-one-standard-deviation depth-weighted Rosetta Ksat alternatives.

Preserve the 2 mm eligibility rule, VT/NSR labels and Ksat variants in a separate 2025 reproduction track. It is not the 2026 event-sensor ResNet's 2026 binary target. The 2026 paper explicitly omits NSR from its own binary PF target. Both datasets come from NEON and require source-overlap reconciliation before combined training or external-validation claims.

## 5. Reproduction versus robustness tracks

| Track | Moisture gaps | Rain bins with fewer than five good observations | Gauge selection | Purpose |
|---|---|---|---|---|
| Paper reproduction | Interpolate gaps shorter than six hours | Zero-fill exactly as reported | Prefer secondary 1-minute data when both sources exist | Reconstruct published data and benchmark |
| Missingness sensitivity | Preserve missing gaps; separate interpolated track | Preserve as missing | Keep source type explicit; do not pool | Quantify dependence on preprocessing and incomplete observations |
| Gauge sensitivity | Same moisture data | Same paper rule and missingness alternative | Primary-only, secondary-only and preferred-source selection | Quantify measurement-system effects |

All tracks share site/sensor identity and train/validation/test split manifests, but a split must never cross site/sensor/event dependencies when evaluating transfer. Historical within-sensor chronological reproduction is not equivalent to leave-site-out transfer.

## 6. Critical validation checks

1. Reconcile the exact 33 sites, 16 ecoregions, 586 sensors, five plots/site, sensor depths and excluded records against the paper's supplement/author code. Do not infer the roster from aggregate counts.
2. Reconcile every NEON release DOI, exact release tag, file, month, site, sensor, plot, calibration and QA version.
3. For soil moisture, test bin edges, median, `n_good >= 7`, maximum-porosity exclusion, and the six-hour interpolation boundary (strictly less than six hours).
4. For rainfall, test source preference where both gauges exist, source-specific QA, the `<5` good-measurement zero rule, and observed zero versus imputed zero.
5. Compare our eventizer's event IDs, onset times, response times and velocity with the HydroShare derived database fields; record disagreements rather than overriding them.
6. Reconcile multiple sensor response signatures: the 2025 paper says it retains earliest response for multiple signatures. Confirm the 2026 code's corresponding handling before adopting it.
7. Reconstruct the 15,554 event-sensor rows before the 70/15/15 split; publish every discrepancy and exclusion reason.
8. Keep the published within-sensor temporal split, site-held-out split, ecoregion-held-out split, and prospective external network evaluation as distinct benchmarks.

## 7. Implementation status

- `tools/preferential-flow-eventizer/` implements event grouping and response calculation on normalized 10-minute data; it does not parse NEON packages or apply the source QA filters.
- `tools/inventory-neon-data.sh` records per-site Data Query responses and file metadata without downloading data.
- `tools/acquire-neon-data.sh` downloads selected NEON files and verifies file size/MD5 plus local SHA-256 after the user supplies a local API token.
- `tools/profile-flow-audit/` implements NSR as a separate diagnostic with interval-censored response timing; it must not be inserted into the 2026 ResNet reproduction target. Pair order is considered robust only when response intervals are strictly separated; overlap is reported as ambiguous.
- `tools/pf-threshold-audit/` tests sensitivity across Rosetta Ksat and modelled matrix-velocity threshold families without assuming an undocumented log base.

Implementation tracker: [TERRA-NEON-001](https://github.com/Luminous-Dynamics/mycelix/issues/5537).

## References

- Li, B. et al. (2026). Dominant Controls on Preferential Flow and Their Implications for Future Soil Water Fluxes. Earth's Future. https://doi.org/10.1029/2026EF008296
- Li, B. et al. (2025). Ubiquity and Causes of Soil Water Preferential Flow Across 17 Ecoregions. Geophysical Research Letters. https://doi.org/10.1029/2025GL118045
- HydroShare. NEON Soil Preferential Flow Database v1.1. https://doi.org/10.4211/hs.847b15cd15524b78acd58a2a8be242c1