# Craft OCEL 2.1 independent semantic-parity oracle V0.1

**Status:** independent test-only Go reference; local tests passed; hosted exact-head qualification pending.  
**Production status:** not integrated as a production parser or serializer.

## Purpose

This module compares the logical semantic projection of the same synthetic OCEL across:
- OCEL JSON;
- OCEL 2.1 single-file CSV;
- OCEL 2.1 bundled CSV.

The comparison is semantic, not byte-for-byte. It checks event identity/type/time and attributes; object identity/type; timed object-attribute history; qualified event-to-object (E2O) edges; and qualified object-to-object (O2O) edges.

## Standards rules represented

- Bundled CSV metadata declares type names, attribute names/types and table paths. Attribute declarations use arrays of `{name,type}` records. Table columns are located by their header names, not assumed positional.
- Bundled table paths are constrained to bundle-relative paths; absolute paths, empty components and `..` traversal are rejected.
- Single-file CSV distinguishes event rows, O2O rows, object declarations and timed object-attribute rows.
- Object reference parsing supports reserved-character escapes, including escaped slash, hash and brace. JSON object-reference attributes are parsed without splitting on slashes inside JSON strings.
- Single-file attribute scopes apply canonical lexical typing and scope unification; non-canonical forms such as `007`, `+5` and `1.50` remain strings.
- Event timestamps and object-attribute assignment timestamps require ISO-8601 timezone information.
- Invalid cross-type object ID reuse, missing relation endpoints, malformed rows, invalid object-declaration qualifiers, and conflicting same-time object attribute assignments are rejected.

## Known-answer fixture

The golden fixture contains two events, five objects, six object-attribute assignments, five E2O relations and two O2O relations. It includes an initial Pump status and one timed status transition, and an O2O calibration relationship that does not arise from sharing an event.

## Tests

With Go 1.23.2:

```sh
gofmt -w oracle/*.go
CRAFT_OCEL_FIXTURE_ROOT=../../../fixtures/ocel21-interchange go test ./...
CRAFT_OCEL_FIXTURE_ROOT=../../../fixtures/ocel21-interchange go test -race ./...
CRAFT_OCEL_FIXTURE_ROOT=../../../fixtures/ocel21-interchange go vet ./...
CRAFT_OCEL_FIXTURE_ROOT=../../../fixtures/ocel21-interchange go test -cover ./...
```

Local reference-module results: all package tests passed, race detector passed, vet passed, statement coverage 72.6%. These are local results in the temporary development workspace, not hosted exact-head CI results.

## Boundaries and limitations

- The module is an independent reference for a finite fixture profile, not the production Craft OCEL implementation.
- OCEL bundled Parquet is not implemented.
- Full canonicalization conformance, all OCEL 2.1 corner cases, Unicode edge cases, large-log streaming, archive/ZIP safety, and interoperability with third-party exporters still need broader validation.
- CSV single-file type inference is scoped as defined by the profile but should be expanded with more cross-value and adversarial fixtures.
- No performance comparison, field data, operational authority, source-universe completeness, or physical outcome is claimed.
