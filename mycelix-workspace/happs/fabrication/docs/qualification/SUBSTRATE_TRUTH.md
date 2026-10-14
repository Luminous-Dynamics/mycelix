# Fabrication substrate truth

The Fabrication hApp is currently implemented against the Holochain 0.6-era Rust API family while the project target is Holochain 0.7.

This is intentionally recorded as:

`status = known-mismatch`

That state is **not** a compatibility qualification and is not a failure of the hApp's existing source-level tests. It is an explicit ceiling: a successful Rust test/build run cannot by itself establish Holochain 0.7 compatibility while the checked-in dependency stack remains on 0.6-era APIs.

## Frozen facts

The machine-readable source of truth is:

`docs/qualification/substrate-matrix.toml`

It commits the current workspace and Sweettest dependency families, the advertised README version, the 14-zome bundle count, and the migration/runtime qualification blockers.

The audit workflow verifies that the source tree still agrees with this matrix. A green **truth-audit** result means only that the repository's declared substrate state is internally consistent.

It does not mean:

- Holochain 0.7 compatibility;
- conductor/runtime compatibility;
- successful hApp assembly;
- successful Sweettest execution;
- coordinator/integrity action-model compatibility under 0.7;
- deployment currentness.

## Upgrade path

The actual substrate migration remains separately tracked by #4246.

The excluded Sweettest and bundle/runtime qualification boundary remains #4248.

The authenticated-anchor/action-model port boundary is #4371.

Do not silently change the dependency versions inside a provenance/security PR. The migration should update the workspace and Sweettest manifests together, port the 0.6 action/validation APIs explicitly, rebuild all 14 WASM zomes, execute the integration suite against the 0.7 runtime, and record exact toolchain/runtime provenance.

## Claim ceiling

Until that migration and executable qualification are complete, the repository may claim only that its **current source substrate is 0.6-era and its intended target is 0.7**.

No source hash, branch state, or truth-audit result may be promoted to a Holochain 0.7 compatibility or runtime qualification claim.
