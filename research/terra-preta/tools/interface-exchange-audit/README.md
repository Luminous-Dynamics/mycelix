# Wetted-interface macropore exchange audit v0.1

This standalone Rust CLI validates the dimensional product and time integral for an explicitly declared CT-informed macropore-to-matrix exchange series. It is not a hydraulic solver or a reconstruction of the full Chaif et al. model.

## Run

    cargo test --manifest-path research/terra-preta/tools/interface-exchange-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/interface-exchange-audit/Cargo.toml -- \
      research/terra-preta/tools/interface-exchange-audit/fixtures/synthetic-constant-exchange.json \
      /tmp/terra-preta-interface-exchange-report.json

## v0.1 dimensional convention

- Specific wetted interface area: m² interface / m³ bulk soil.
- Macropore-to-matrix flux density: m³ water / (m² interface · s), equivalent to m/s.
- Exchange rate: area × flux density = s⁻¹.
- Time integration uses trapezoidal quadrature over actual elapsed UTC seconds; irregular intervals are preserved and not treated as equally spaced.
- Integrated output is gross one-way exchange volume per m³ bulk soil. It is not clamped to [0,1], is not net exchange and can exceed one under cycling/repeated transfer.

Input includes source digests and per-sample provenance/status for interface area and flux density. It requires at least two samples and exact start/end observations; it does not interpolate or fill missing measurements.

## Interpretation boundary

The exact normalization and sign convention must be checked against the full Chaif et al. (2025) equations/supplement before claiming reproduction. This tool does not derive wetted area from CT, solve horizontal Richards flow to obtain macropore flux density, model reverse matrix-to-macropore exchange, close the whole soil water balance or infer a treatment effect. Modelled inputs stay modelled; a digest establishes artifact identity, not physical validity.

Reference: Chaif et al. (2025), Journal of Hydrology 661, 133791, DOI 10.1016/j.jhydrol.2025.133791.