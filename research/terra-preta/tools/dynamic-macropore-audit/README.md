# Dynamic macropore audit v0.1

This standalone Rust CLI independently implements the equation-level morphology-conditioned **dynamic saturated-conductivity** core of Hirmas et al. (2025), Water Resources Research, DOI 10.1029/2024WR039836. It consumes already-derived 2D MLT horizon geometry and matrix-water-content observations; it does not segment images, partition total soil moisture, solve Richards' equation, or simulate solute transport.

## Run

    cargo test --manifest-path research/terra-preta/tools/dynamic-macropore-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/dynamic-macropore-audit/Cargo.toml -- \
      research/terra-preta/tools/dynamic-macropore-audit/fixtures/synthetic-konza-like-input.json \
      /tmp/dynamic-macropore-report.json

The input contains 2D image summaries (cross-section area, total macropore area/perimeter and dry mean macropore width), soil shrink/swell properties, fluid properties, and a time series of matrix water content. Every input file is SHA-256 bound in the report.

## Implemented equations

- Eq. 8: dry structural-unit width, (w_{ds}=4(A_{xs}-A_p)/P).
- Eqs. 9–11: matrix-water-conditioned bulk density, saturated bulk density and dynamic COLE.
- Eqs. 13–16: structural-width correction and dynamic interpedal pore width (d(\theta_m)).
- Eqs. 20–23: slit-pore saturated conductivity (K_{sp}), whole-soil transient saturated conductivity (K_s), macropore volume fraction (f_p), and cross-section-to-volume conversion (R_p).
- Eq. 37: optional empirical width-ratio relationship for structural-domain (alpha_2), (\alpha_2/\alpha_{ds}=\beta_0+\beta_1(d/d_{ds})). The positive sign is consistent with the paper's Figure 6 and Table 1.

## Required interpretation boundaries

- Matrix water content `theta_m` must be independently measured or produced by a separately audited partition model. Do not feed total sensor water content and label it matrix content.
- MLT geometry is a dry-state 2D cross-section. The paper's equivalent structural-unit and planar slit assumptions are retained.
- Dynamic pore widths are model-inferred from one dry-state image plus changing matrix water content; they are not repeated time-lapse pore-width measurements.
- The macro saturated conductivity is theoretical under laminar slit flow. Matrix conductivity is supplied as an input. No unsaturated conductivity function (Hirmas Eq. 38), Richards solver, 3D pore network, hysteresis, tracer transport or boundary mass balance is included.
- The area threshold is recorded from input metadata; select the Hirmas paper's `pi mm2` macropore cutoff only for that reproduction, not as a universal soil threshold.
- Hirmas coefficients were fitted on one Konza APT soil profile. This tool does not claim transfer to other soils, Terra Preta or South African soils.

## Tests

Synthetic unit tests cover Eq. 8 geometry, dry-state pore width, moisture-induced pore contraction, saturated conductivity units, the positive Eq. 37 alpha relation, out-of-range moisture, duplicate timestamps and impossible image geometry. These are equation/software tests, not a replication of the Dryad dataset or independent validation.


Implementation tracker: [TERRA-MORPH-DYN-001](https://github.com/Luminous-Dynamics/mycelix/issues/5545).

The CLI and tests are registered in `.github/workflows/ci.yml` for rustfmt, unit tests, a synthetic end-to-end fixture assertion, and strict Clippy. A queued or superseded run is not counted as a pass.
