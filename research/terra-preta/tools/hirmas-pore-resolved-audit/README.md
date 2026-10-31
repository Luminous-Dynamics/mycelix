# Hirmas per-pore dynamic macropore audit v0.1

This standalone Rust CLI joins an ImageJ MLT per-object CSV export to the report from `dynamic-macropore-audit`. It implements per-pore Hirmas et al. (2025) Equations 33, 35, 36 and 46: object-specific dynamic pore widths, volumetric pore fractions, matric heads and cumulative pore-water-retention points.

## Run

    cargo test --manifest-path research/terra-preta/tools/hirmas-pore-resolved-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/hirmas-pore-resolved-audit/Cargo.toml -- \
      imagej-pore-objects.csv dynamic-macropore-report.json config.json new-per-pore-report.json

The output path must not already exist. The report records exact SHA-256 digests for the CSV, dynamic report and config.

## Config

Config pins study/profile/horizon/scan identity, raw image digest and evidence references, pixel size, cross-section ROI area, strict minimum object area and exact ImageJ column headings. The Dryad Hirmas protocol reports an MLT grid of 180 μm and excludes gap objects with area less than or equal to π mm²; the filter is explicit and not a universal threshold.

## Implemented equations

For each dynamic matrix-water state, this tool uses the upstream mean pore width d(theta_m) and structural-unit width w(theta_m). Each object's dry MinFeret is scaled by the mean width ratio: d_i(theta_m) = d_i,ds * d(theta_m)/d_ds. This proportional per-object scaling is the implementation of the paper's stated pore-width rescaling and must be verified against the authors' R scripts before claiming exact parity.

- Eq. 35: R_pi(theta_m) = ((w+d_i)^3 - w^3) / ((w+d_i)*d_i*(2w+d_i)).
- Eq. 36: f_pi(theta_m) = R_pi(theta_m) * A_pi*d(theta_m) / (A_xs*d_ds).
- Eq. 33: h_i = -sigma/(rho_w*g*d_i), with d_i converted from mm to m.
- Eq. 46: sort pores by ascending dynamic width and accumulate f_pi to form the macropore water-retention curve.

The output retains both the per-pore sum of f_pi and the upstream aggregate Eq. 22 volume fraction, with absolute discrepancy. It never rescales or forces the per-pore sum to equal the aggregate result.

## Strict input consistency gates

The tool checks that retained ImageJ object count, summed projected area, summed perimeter, geometric-mean dry MinFeret and dry structural-unit width match the geometry declared by the dynamic-macropore report. It also checks horizon and profile identity, ROI, image resolution and object-size filter. A failed crosswalk is an error, not a warning.

## Interpretation boundary

This is an object-resolved **2D geometry-derived** pore-retention estimate. It does not establish 3D connectivity, flux, tracer-confirmed preferential flow or field efficacy. Matrix-water-dependent geometry and per-pore capillary heads are model outputs, not direct measurements. A full Hirmas reproduction also requires author-script parity for water partitioning, alpha2/n2 refitting, unsaturated K(theta, theta_m), and validation against the Stewart et al. (2016) model.

Implementation tracker: [TERRA-HIRMAS-PORE-001](https://github.com/Luminous-Dynamics/mycelix/issues/5568).

Acceptance contract: [Hirmas Dynamic Morphology Reproduction Benchmark](../../HIRMAS_DYNAMIC_MORPHOLOGY_REPRODUCTION_BENCHMARK_V0.1.md).

Reference: Hirmas et al. (2025), Water Resources Research 61(9), e2024WR039836, DOI 10.1029/2024WR039836. Dataset: Dryad DOI 10.5061/dryad.zs7h44jkn (CC0 1.0).