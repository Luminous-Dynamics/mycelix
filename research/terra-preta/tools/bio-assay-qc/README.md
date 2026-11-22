See `BIO_ASSAY_QC_BENCHMARK_V0.1.md`. Run `cargo test --manifest-path research/terra-preta/tools/bio-assay-qc/Cargo.toml` for synthetic method-specific QC tests. All fixture values are synthetic; the auditor reports potential activity only and does not estimate process rates.


**Optical-quench limitation:** v0.1 does not execute any quench-correction calculation. A high-quench assay remains unqualified even when a correction method ID and digest are declared; those fields are provenance only.