# Soil assessment agreement audit

Standalone Rust auditor for synthetic soil-function alternatives. It compares six transparent scoring archetypes, five declared weight profiles, method-rank agreement, rank reversals, Pareto relations and hard guardrails.

## Run

cargo fmt --manifest-path research/terra-preta/tools/assessment-agreement-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/assessment-agreement-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/assessment-agreement-audit/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/assessment-agreement-audit/Cargo.toml -- research/terra-preta/tools/assessment-agreement-audit/fixtures/synthetic-assessment-agreement.json /tmp/assessment-agreement-report.json

## Important

The six methods are representative archetypes only. They are not reproductions of CASH, SMAF, Soil Navigator, SAS or other named protocols. Scores are method-specific and must not be treated as interchangeable.

The Pareto vector and hard-guardrail results are independent of scalar rankings. Uncertainty intervals that overlap remain incomparable for robust dominance. The synthetic thresholds and guardrails have no field authority.

The 2026 review by Zwaan et al. (DOI 10.1016/j.soilbio.2026.110184) found low agreement between soil-health assessment methods applied to the same 87-farm dataset. The objective here is to expose assumptions and rank instability, not to choose a universal winning index.
