# Standards registry seed

This directory is a **seed inventory**, not a conformance catalogue. Every entry tracks source metadata separately from Pulse implementation support. An entry marked `unreviewed` must not be presented as supported.

- `registry.v1.json`: source pointers and explicit open work across 30 records, revised 2026-10-16, including individual records for 9 RFC 9171 errata and 4 reported RFC 9172 errata.
- `schema.v1.json`: JSON Schema draft 2020-12 structure.
- `validator/`: standalone Rust contract validator and adversarial unit tests. The validator checks the registry's declared v1 fields/enums, IDs, URL/date shapes, support-evidence gates, and relation targets. It is deliberately not represented as a generic JSON Schema engine.
- `../docs/TRANSPORT_BENCHMARKS.md`: acceptance gates and benchmark methodology.
- `../docs/MULTI_TRANSPORT_INTEROPERABILITY_RFC.md`: architecture and layer boundaries.

## Rules

1. Treat the published standards-body document as authoritative. News pages and catalogues are discovery pointers where the normative text is not directly linked.
2. Preserve relations such as updates, amendments and errata. A document can be current without being the newest version of a related protocol family.
3. Record source revisions, edition, retrieval date, source digest, reviewer and test evidence before making support claims. A URL by itself is not a version lock.
4. Keep "source published/active" distinct from "Pulse implements it". The latter requires independent implementation and conformance evidence.
5. Unknown information must be represented as unknown or null, never inferred from a family name.
6. Do not turn registry updates into automatic radio activation, installation, transmissions, dependency changes, or model-policy changes.
7. The initial seed does not exhaustively cover all standards or revisions. RFC 9171's errata index reports 9 records (3 verified, 1 reported, 4 held for document update, 1 rejected); those dispositions are separately represented in the registry. RFC 9172's errata index reports 4 reported records, which are kept distinct from verified corrections and from Pulse's own implementation support. New entries require a source review and should be validated against the schema.

Run from the Pulse workspace root with `just test-standards-registry` and `just validate-standards-registry`. The synthetic route-policy model is in `validator/src/simulation.rs`, its CLI is `validator/src/bin/route-sim.rs`, and its eleven starter scenarios are in `../benchmarks/traces.v1.json`. Run `just simulate-transport` for 10,000 logical messages per scenario and three paired strategies. Every report marks itself synthetic and explicitly denies hardware, cryptographic, or authenticated-delivery qualification.

The checker and simulation harness have not yet been compiled or executed in this environment because a Rust toolchain is unavailable here. The existing root `.github/workflows/ci.yml` now includes an exact-PR-head job that runs the validator, negative controls, and deterministic route simulation and archives subject/input hashes. No pass is claimed until that job produces an artifact. Do not report the Rust tests as passing until exact-head evidence exists. Before treating the checker as reproducible release infrastructure, generate and review its standalone Cargo.lock in the project Nix environment, then change commands to `--locked` and record that lockfile hash. The workflow records a generated lockfile and its hash when present, but a missing lockfile is explicitly reported rather than mistaken for a locked build.

No production Python or Node-based runtime is required.
