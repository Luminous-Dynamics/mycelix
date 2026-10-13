# Furniture Compiler (prototype)

A small Rust-first reference compiler for a bounded modular-desk parameter schema. It is intentionally isolated from manufacturing order release.

## Status and claim boundary

See [QUALIFICATION_PLAN.md](QUALIFICATION_PLAN.md) for the proposed physical-test/evidence gates and current standards references, and [CANONICAL_ENCODING.md](CANONICAL_ENCODING.md) for the exact v2 field order, enum discriminants, binary hash-input encoding, and verifier contract. Both are specifications, not certification or release approval.

This is a prototype reference implementation, not a released furniture design, CAD exporter, nesting engine, structural solver, price calculator, safety certification, or production release authority. The output's release gate is always blocked until the listed engineering, supplier, process, qualification, inspection, and fulfillment evidence is bound to the exact revision.

The generated cut parts are rectangular panel envelopes for early contract testing. They do not specify joinery, drill patterns, fasteners, edge-band placement, tolerances, load limits, or accepted manufacturing methods. Do not manufacture or sell a product from the packet alone.

## Run

From the manufacturing workspace:

```bash
cargo test -p furniture_compiler
cargo run -p furniture_compiler --bin furniture-compile < crates/furniture_compiler/examples/desk-input.json > packet.json
cargo run -p furniture_compiler --bin furniture-compile -- --verify < packet.json
```

The CLI accepts one JSON document on standard input (maximum 64 KiB) and emits a JSON packet to standard output. Passing `--verify` switches to packet-verification mode: it regenerates the expected packet from the bound configuration and rejects any mismatch, including altered digests, part lists, blockers, or release-gate state. Verification output only attests to conformance with this prototype's deterministic reference model; it does not establish physical safety or qualification. Errors go to standard error and exit status 2.

## Input example

```json
{
  "schema_version": "furniture-desk-input-v1",
  "width_mm": 1200,
  "depth_mm": 600,
  "height_mm": 750,
  "panel_thickness_mm": 18,
  "material_id": "birch_plywood",
  "finish_id": "clear_coat"
}
```

Allowed prototype envelope:
- width: 600–1600 mm;
- depth: 400–900 mm;
- height: 650–850 mm;
- panel thickness: exactly 18 mm;
- birch plywood with clear coat, or melamine MDF with white/black melamine finish.

These are schema bounds, not safety or ergonomic acceptance criteria. The compatibility matrix also does not establish that a material/finish combination is available from a real supplier.

## Integrity profile

The workspace pins the BLAKE3 implementation to `=1.8.7` for this prototype profile. The compiler computes domain-separated BLAKE3 digests over the explicitly specified `canonical-binary-v2` encoding:
- `design_digest`: family/revision, explicit ruleset and geometry-rule IDs, fixed dimension bounds, panel-envelope constants, allowed material/finish matrix, and configuration;
- `cut_list_digest`: generated cut-part envelopes;
- `packet_digest`: the complete packet core, including blockers and subordinate digests.

Encoding rules:
- strings and collection counts use unsigned 64-bit big-endian lengths;
- dimensions, quantities, and sequence numbers use unsigned 32-bit big-endian integers;
- booleans use one byte, `0` or `1`;
- enum values use explicit one-byte discriminants defined by the Rust encoder;
- aggregate fields use the fixed order in each `encode_*` function;
- each hash includes a BLAKE3 derive-key context, domain label, NUL separator, encoded byte length, and encoded bytes.

This is a versioned binary protocol, not RFC 8785 canonical JSON. Cross-language verifiers must independently implement these exact rules and pass shared test vectors before relying on these digests. The packet schema is `furniture-compiler-packet-v2`; the design digest uses the `design-spec-v2` domain. Geometry semantics that change beyond the explicitly bound constraints must increment the corresponding ruleset/geometry-rule identifier. Any field-order, enum-discriminant, units, or encoding change requires a new protocol version. Digests establish integrity/binding only, not truth, authorship, safety, or qualification.

## Tests

The crate contains tests for deterministic recompilation, canonical encoding golden vectors, digest sensitivity to input and rule identity, stable parts ordering, expected geometry arithmetic, both sides of each dimension bound, schema rejection, incompatible finishes, unsupported thickness, unknown JSON fields, oversized input, tamper detection, and the permanent release-blocked state. CI also invokes the CLI verifier on a generated packet and requires it to reject an altered release gate. These tests must still be executed in the target Rust/Nix toolchain before this change is considered verified.
