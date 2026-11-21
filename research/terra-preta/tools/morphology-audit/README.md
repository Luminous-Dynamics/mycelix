# Morphology audit v0.1

This Rust CLI summarizes already segmented ImageJ macropore-object CSV exports. It does not process raw scans, segment pores, infer 3D connectivity or calculate hydraulic conductivity.

## Run

    cargo test --manifest-path research/terra-preta/tools/morphology-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/morphology-audit/Cargo.toml -- \
      imagej-export.csv morphology-report.json \
      --horizon Ap1 --resolution-um 180 \
      --area-col Area --perimeter-col 'Perim.' \
      --min-feret-col MinFeret --feret-col Feret --angle-col FeretAngle \\
      --min-area-mm2 3.141592653589793

ImageJ headings are configurable because export variants differ. The Dryad Konza archive identifies its three MLT analysis CSVs as 0.18 mm-resolution object geometry and states that length fields are in mm. Confirm units and scale for each exact image/export before running the audit.

The report records the exact input SHA-256, horizon, pixel resolution, requested ImageJ headings, input/retained/excluded object counts, configurable minimum-area threshold, per-feature distributions, 10-degree Feret-angle counts, 2D circularity (4πA/P²), Feret aspect ratio, nematic orientation order, and—only when the exact analyzed ROI area is supplied—the projected 2D pore-area fraction. Objects below the threshold are counted as excluded; the threshold is never silently inferred. The default is 0 mm² (no size filter).

## Interpretation boundary

Feret angle is an image-plane orientation statistic. Projected pore-area fraction is not 3D total porosity. Object-level 2D morphology does not establish connectedness, tortuosity, vertical continuity, hydraulic conductance or preferential-flow truth. CT, orthogonal sections and tracer/flux evidence remain separate validation requirements.

## Tests

The synthetic suite checks required headers, invalid/nonfinite geometry, impossible Feret ordering, orientation bounds, ROI area consistency, quantile calculations, digest propagation and explicit 2D interpretation boundaries. It contains no measured Konza or NEON data.

## Published dynamic-macropore bridge dataset

Hirmas et al. (2025), [Dryad DOI 10.5061/dryad.zs7h44jkn](https://doi.org/10.5061/dryad.zs7h44jkn), published a CC0 1.0 dataset containing three horizon-specific ImageJ MLT exports (Ap1, Bt and Btkss2), soil-water time series at 10, 40 and 120 cm, measured bulk-density/porosity/COLE and water-retention data, model outputs, and the authors' processing scripts. The source imagery used a 180 μm grid; ImageJ geometry lengths are reported in mm. For the paper's macropore analysis, objects with projected area smaller than π mm² were excluded (equivalent-circle radius 1 mm). This tool exposes that filter as `--min-area-mm2 3.141592653589793`; select it only for a faithful Hirmas-style reproduction, not as a universal threshold for every soil.

The paper's key paradigm is **dynamic morphology–hydrology coupling**: dry-state cross-section morphology is combined with matrix-water-content time series to update interpedal macropore widths and transient hydraulic conductivity, rather than treating pore structure and conductivity as fixed. Its model assumes, among other simplifications, equidimensional/equivalent structural units and uses a single Konza Prairie fine-silty, shrink–swell-prone soil profile. Reproduce it first on its published data, then challenge its assumptions across soil families; do not transfer its coefficients directly to South African or other soil classes.

The present auditor computes descriptive 2D geometry only. The next coupling benchmark must compare (a) static morphology, (b) matrix-water-conditioned dynamic morphology, and (c) independent transient flux/tracer observations, while propagating image segmentation/scale uncertainty. A 2D image is not 3D connectivity, a macropore width is not hydraulic conductance by itself, and a fitted conductivity response is not an independent PF ground truth.

## Tests

The suite additionally checks size-threshold accounting, shape descriptors and 180-degree (nematic) orientation handling. It contains no measured Konza or NEON data.
