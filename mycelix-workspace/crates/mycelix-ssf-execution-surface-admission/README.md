name: SSF Execution Surface Admission

on:
  pull_request:
    paths:
      - 'mycelix-workspace/crates/mycelix-ssf-execution-surface-admission/**'
      - 'mycelix-workspace/crates/mycelix-ssf-use-time-lease-revalidation/**'
      - '.github/workflows/ssf-execution-surface-admission.yml'
  push:
    branches: [main]
    paths:
      - 'mycelix-workspace/crates/mycelix-ssf-execution-surface-admission/**'
      - 'mycelix-workspace/crates/mycelix-ssf-use-time-lease-revalidation/**'
      - '.github/workflows/ssf-execution-surface-admission.yml'

permissions:
  contents: read

jobs:
  admission:
    runs-on: ubuntu-latest
    timeout-minutes: 10
    steps:
      - uses: actions/checkout@v4

      - uses: dtolnay/rust-toolchain@stable
        with:
          components: rustfmt, clippy
          targets: wasm32-unknown-unknown

      - name: Format
        run: cargo fmt --manifest-path mycelix-workspace/crates/mycelix-ssf-execution-surface-admission/Cargo.toml -- --check

      - name: Native check
        run: cargo check --manifest-path mycelix-workspace/crates/mycelix-ssf-execution-surface-admission/Cargo.toml --all-targets

      - name: no_std WASM check
        run: cargo check --manifest-path mycelix-workspace/crates/mycelix-ssf-execution-surface-admission/Cargo.toml --lib --target wasm32-unknown-unknown

      - name: Tests
        run: cargo test --manifest-path mycelix-workspace/crates/mycelix-ssf-execution-surface-admission/Cargo.toml

      - name: Strict Clippy
        run: cargo clippy --manifest-path mycelix-workspace/crates/mycelix-ssf-execution-surface-admission/Cargo.toml --all-targets -- -D warnings

      - name: Rustdoc
        env:
          RUSTDOCFLAGS: -D warnings
        run: cargo doc --manifest-path mycelix-workspace/crates/mycelix-ssf-execution-surface-admission/Cargo.toml --no-deps
