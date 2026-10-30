# Morphology audit v0.1

This Rust CLI summarizes already segmented ImageJ macropore-object CSV exports. It does not process raw scans, segment pores, infer 3D connectivity or calculate hydraulic conductivity.

## Run

    cargo test --manifest-path research/terra-preta/tools/morphology-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/morphology-audit/Cargo.toml -- \
      imagej-export.csv morphology-report.json \
      --horizon Ap1 --resolution-um 180 \
      --area-col Area --perimeter-col 'Perim.' \
      --min-feret-col MinFeret --feret-col Feret --angle-col FeretAngle

ImageJ headings are configurable because export variants differ. The Dryad Konza archive identifies its three MLT analysis CSVs as 0.18 mm-resolution object geometry and states that length fields are in mm. Confirm units and scale for each exact image/export before running the audit.

The report records the exact input SHA-256, horizon, pixel resolution, mapping of requested ImageJ headings, per-feature min/max/mean/median/P90/P95, 10-degree Feret-angle counts, and—only when the exact analyzed ROI area is supplied—the projected 2D pore-area fraction.

## Interpretation boundary

Feret angle is an image-plane orientation statistic. Projected pore-area fraction is not 3D total porosity. Object-level 2D morphology does not establish connectedness, tortuosity, vertical continuity, hydraulic conductance or preferential-flow truth. CT, orthogonal sections and tracer/flux evidence remain separate validation requirements.

## Tests

The synthetic suite checks required headers, invalid/nonfinite geometry, impossible Feret ordering, orientation bounds, ROI area consistency, quantile calculations, digest propagation and explicit 2D interpretation boundaries. It contains no measured Konza or NEON data.