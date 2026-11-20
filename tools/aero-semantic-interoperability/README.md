# AeroCommons semantic-interoperability comparator

A deterministic Rust evaluator for comparing normalized semantic assertion sets emitted by independent AP242/QIF adapters.

## Run

From the repository root:

    cargo test --manifest-path tools/aero-semantic-interoperability/Cargo.toml

## What it does

- Strictly admits the versioned semantic-case JSON shape.
- Checks source-manifest rights disposition before external cases can be used.
- Compares persistent identity, requirement/feature joins, semantic type, value and unit.
- Classifies missing required assertions as semantic loss, explicitly unsupported assertions as unsupported, cross-subject joins as wrong-subject joins, and unexpected records as ambiguous.
- Applies the frozen mutation corpus and checks expected dispositions.
- Uses ordered maps/sets so input order does not change results.

## What it does not do

It is not an AP242 STEP or QIF parser, geometric engine, GD&T evaluator, standards-conformance implementation or physical inspection authority. Adapters must parse/validate their source formats with qualified external tooling, normalize their semantics under a pinned profile, and then submit assertion sets to this comparator.

The v1 comparator uses literal canonical values and units. It intentionally does not silently normalize unit changes; conversion profiles must be explicit, tested and preserve the original values/units as provenance.
