# Standards registry seed

This directory is a **seed inventory**, not a conformance catalogue. Every entry tracks source metadata separately from Pulse implementation support. An entry marked `unreviewed` must not be presented as supported.

- `registry.v1.json`: reviewed source pointers and explicit open work, current as of 2026-10-15.
- `schema.v1.json`: JSON Schema draft 2020-12 structure.
- `../docs/TRANSPORT_BENCHMARKS.md`: acceptance gates and benchmark methodology.
- `../docs/MULTI_TRANSPORT_INTEROPERABILITY_RFC.md`: architecture and layer boundaries.

## Rules

1. Treat the published standards-body document as authoritative. News pages and catalogues are discovery pointers where the normative text is not directly linked.
2. Preserve relations such as updates, amendments and errata. A document can be current without being the newest version of a related protocol family.
3. Record source revisions, edition, retrieval date, source digest, reviewer and test evidence before making support claims. A URL by itself is not a version lock.
4. Keep "source published/active" distinct from "Pulse implements it". The latter requires independent implementation and conformance evidence.
5. Unknown information must be represented as unknown or null, never inferred from a family name.
6. Do not turn registry updates into automatic radio activation, installation, transmissions, dependency changes, or model-policy changes.
7. The initial seed does not exhaustively cover all standards or revisions. New entries require a source review and should be validated against the schema.

The current schema captures the initial information model. A follow-on implementation should add local validation, duplicate-ID checks, reciprocal relation checks, source digest capture, date-based stale status, and generated documentation in the repository's Rust/Nix test path. No production Python or Node-based runtime is required.
