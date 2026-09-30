# AeroCommons Holochain validation fixture

Test-only integrity zome for **AEROCOMMONS-011**.

This crate is deliberately a separate Cargo workspace so the current Mycelix
0.6 production zomes are not forced onto Holochain 0.7.

Pinned compatibility target:

- Holochain: 0.7.0
- HDI: 0.8.0
- Validation surface: real `Op`, `FlatOp`, `OpRecord`, `OpLink`, and
  `must_get_valid_record`

The fixture maps the 14 authority-ceiling cases to concrete Holochain 0.7
validation variants.

It is **not** a production zome and does not establish physical correctness,
engineering truth, certification, airworthiness, manufacturing conformity, or
regulatory authority.

## Verification

Run inside a Holochain 0.7 / Rust toolchain environment:

`cargo fmt --check`
`cargo check`
`cargo test`

The repository environment used to author this change does not expose Cargo,
so no local compile/test result is claimed here. The next verification gate is
repository CI plus an actual Holochain/sweettest integration run.
