# Root-channel pathway inventory auditor

Standalone Rust auditor for **typed pathway inventory metadata**. It is not an image segmenter, 3D reconstruction system, root classifier or flow solver.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/root-channel-inventory/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/root-channel-inventory/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/root-channel-inventory/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/root-channel-inventory/Cargo.toml -- \
  research/terra-preta/tools/root-channel-inventory/fixtures/synthetic-root-channel-inventory.json \
  /tmp/root-channel-report.json
```

## Audited contracts

- unique node/pathway IDs and valid endpoint references;
- coordinate dimensionality must match declared 2D / 3D observation support;
- positive finite path length / diameter, non-negative depth and tortuosity >= 1;
- explicit pathway origin class and separate origin evidence status;
- 2D sources cannot claim observed 3D connectivity;
- ordered time snapshots with exactly one occupancy state per pathway;
- non-root pathways cannot claim root occupancy;
- output includes origin class counts, unresolved root-origin evidence, directly observed 3D connectivity and endpoint-graph component count.

## Interpret with care

`endpoint_graph_component_count` describes connectivity of supplied node/edge identifiers only. It does **not** certify true 3D connectedness, hydraulic continuity or transported mass.

A pathway label is not proof of origin. Morphology-only / unresolved evidence remains unresolved. Synthetic truth is counted only within the synthetic fixture and is never a field observation.

The auditor does not estimate water flux, solute transport, hydraulic conductivity, root uptake, microbial activity, rhizosphere chemistry or causal crop response. Those require separate observation operators and independently qualified physical / biogeochemical models.
