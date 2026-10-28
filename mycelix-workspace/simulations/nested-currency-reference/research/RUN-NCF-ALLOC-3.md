# Running NCF-ALLOC-3

The research harness is deliberately separate from the Rust crate: it records a reproducible Python model and attack sweep, not a production implementation.

From this directory:

    python3 -m unittest -v test_ncf_alloc3.py
    python3 ncf_alloc3.py --output ncf-alloc3-results.json --sweep-cases 20000

Expected baseline: 9 unit tests pass. The default research profile evaluates 20,000 randomized claim-inflation cases. A fixed-seed 10,000-case test checks conservation and invariance under input reordering.

The Rust policy variant is integrated into ../src/allocation.rs and the matched-trace binary includes 2x and 4x variants. Neither should be reported as built or passing until an exact-head receipt reports formatting, tests and Clippy. After Rust is available, run:

    cargo fmt --manifest-path ../Cargo.toml -- --check
    cargo test --locked --manifest-path ../Cargo.toml
    cargo clippy --locked --manifest-path ../Cargo.toml --all-targets -- -D warnings

The Python output cannot substitute for model checking, distributed fault tests or production evidence.
