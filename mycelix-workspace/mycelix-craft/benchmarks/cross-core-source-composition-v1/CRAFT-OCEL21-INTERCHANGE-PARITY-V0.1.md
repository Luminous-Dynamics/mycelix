# OCEL 2.1 Interchange-Parity Profile V0.1

**Status:** static golden fixtures and expected semantic projection; no adapters or measured round-trip pass yet.

## Why this matters

OCEL 2.1 pre5 has multiple transport forms. Single-file CSV is a compact table that carries events, O2O relations, object declarations and timed object attribute assignments. Bundled CSV/Parquet uses typed event/object/object-change/E2O/O2O tables and an authoritative `ocel-meta.json` mapping. A benchmark that only checks that each file parses can silently lose qualifiers, relationships or object state history.

The common benchmark object is the **logical semantic projection**, not the original bytes. Each supported serializer should import to the same typed sets and sequences, preserving all E2O and O2O triples, event attributes, object types and object attribute histories. Export/import round-trip is then checked against the frozen expected projection.

## Normative semantic dimensions to test

- Event identity, type, event time and typed attributes.
- Object identity and exactly one type per object ID.
- Qualified E2O edges (event, qualifier, object).
- Qualified O2O edges (source object, qualifier, target object), including O2O relations not implied by a shared event.
- Object attribute value history, including initial value and each timed change.
- Format-specific mapping metadata and row-kind rules.
- Typed null/missing behavior; schema and type metadata must not be inferred from filenames alone.
- Distinction between event time and source ingestion/registration time, which is outside OCEL event time but required in the broader Craft source envelope.

## Formats and exact rules

1. **OCEL 2.0 JSON:** top-level eventTypes, objectTypes, events and objects; E2O qualifiers in event relationships; O2O qualifiers in object relationships; object attributes retain their own timestamps.
2. **OCEL 2.1 single-file CSV:** rows are discriminated by `id`, `activity`, and `timestamp`. `activity=o2o` indicates an atemporal O2O edge; event rows create E2O edges through object-reference cells; blank id/activity with blank timestamp creates object declarations; blank id/activity with timestamp creates timed object attribute assignments. Object reference IDs/qualifiers cannot contain the separators `/`, `#`, or `{`.
3. **OCEL 2.1 bundled CSV:** metadata declares type-to-file mapping; event tables, object tables, object-change tables, E2O and O2O are separate typed tables. Empty change tables are still represented if referenced by metadata.
4. **OCEL 2.1 bundled Parquet:** same typed-table logical layout as bundled CSV, with Parquet encoding instead. It is explicitly NotImplemented in this static fixture pack until a Parquet serializer/importer is added.

## Frozen fixture

`fixtures/ocel21-interchange/ocel21-mini.json` is the canonical OCEL JSON sample.
`fixtures/ocel21-interchange/ocel21-mini.ocel.csv` is its single-file CSV companion.
`fixtures/ocel21-interchange/bundled-csv/` contains a bundled CSV directory layout including metadata, typed event and object tables, object changes, E2O and O2O relations.
`fixtures/ocel21-interchange/ocel21-interchange-parity-v0.1.json` freezes the expected semantic projection and eight mutation tests.

The fixture is synthetic. Pressure values, assets, zones, calibration IDs and sample state have no real-world significance.

## Acceptance gates (proposed)

- 100% of frozen semantic KATs preserved for every supported format.
- Zero E2O or O2O endpoint/qualifier loss.
- Zero object attribute history loss.
- Zero object-type reassignment acceptance.
- Zero event-time / ingest-time conflation.
- Zero type coercions that change the declared primitive type.
- Canonical semantic projection digest stable across format-specific encodings.

These are proposed targets. No serializer or importer result is claimed by adding fixtures alone.

## Reference implementation status (2026-11-28)

A test-only independent Go reference module is now present at `oracles/ocel21-interchange-go-v0.1/`. It includes parsers for OCEL JSON, OCEL 2.1 single-file CSV, and OCEL 2.1 bundled CSV, plus a canonical semantic projection and adversarial known-answer tests.

Local Go 1.23.2 validation completed successfully: package tests, race detector, vet, and statement-coverage run all passed; coverage was 72.6%. This is local reference-module validation only. The hosted exact-head workflow has been added and must run against the current PR head before an exact-head pass can be claimed. The module is not the production Craft parser. Bundled Parquet remains NotImplemented.

The independent parser specifically tests the actual bundled metadata attribute declaration shape (arrays of `{name,type}` records), column-name lookup regardless of column position, safe bundle-relative paths, object-reference escaping, cross-type ID reuse, O2O timestamped target attributes, and single-file scope type unification.
