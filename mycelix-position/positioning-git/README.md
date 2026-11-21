# Mycelix Position v2 — standalone Git package facade

This package makes the canonical Mycelix Position library consumable as a
uniquely named Cargo Git package.

It does **not** fork or copy the positioning implementation. The library
target points directly at:

`mycelix-position/lib/positioning/src/lib.rs`

The facade package name is `mycelix-positioning-v2`; its Rust library name
remains `positioning`. This avoids collisions with downstream development
stubs that may also use the Cargo package name `positioning`.

## Consumer example

```toml
[dependencies]
positioning = {
  package = "mycelix-positioning-v2",
  git = "https://github.com/Luminous-Dynamics/mycelix",
  rev = "<immutable-commit-sha>",
  optional = true
}
```

Pin an immutable commit. Do not use a moving branch for a production or
qualification build.

## Verification

```sh
cargo fmt --manifest-path mycelix-position/positioning-git/Cargo.toml -- --check
cargo clippy --manifest-path mycelix-position/positioning-git/Cargo.toml --all-targets -- -D warnings
cargo test --manifest-path mycelix-position/positioning-git/Cargo.toml --all-targets
```

The facade validates package consumability and reuses the canonical library
source. It does not establish physical navigation performance, source
independence, calibration, integrity protection levels, or operational safety.
