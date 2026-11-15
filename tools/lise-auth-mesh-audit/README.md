# LISE Auth Mesh Corpus Auditor

Dependency-free Go integrity gate for the public authenticated-workflow and OWASP ACS development corpora under `docs/research/auth-mesh/`.

## Run

From the repository root:

```sh
cd tools/lise-auth-mesh-audit
go test -race ./...
go vet ./...
go run . -root ../.. -json
```

The command audits all three corpus families:

- `LISE-AUTH-MESH-48-v1` — 48 scenarios / 24 A-B pairs;
- `ACS-COMP-18-v1` — 18 scenarios / 9 A-B pairs;
- `ACS-COMP-EXT-8-v1` — 8 scenarios / 4 A-B pairs.

It checks strict JSON decoding, expected top-level fields, Draft 2020-12 schema declaration, `additionalProperties: false`, required properties, the schema keywords used by the current record schemas (`type`, `enum`, `pattern`, `minLength`, `minItems`, `uniqueItems`, `items`), sequential stable IDs, pair consistency, exact A/B cardinality, one-to-one scenario/oracle linkage, public calibration disclosures, and SHA-256 digests for every input artifact.

The implementation is a deliberately bounded validator for the schema subset used by these files. It is **not** a complete JSON Schema implementation; use a full Draft 2020-12 validator in addition when broad schema semantics are introduced.

## Exit semantics

- Exit 0: all three corpora pass the structural integrity contract.
- Nonzero: any missing file, malformed JSON, schema mismatch, unknown field/value, incorrect count, duplicate/orphan ID, pair defect, or incorrect disclosure fails closed.

The JSON report is deterministic for identical file bytes and includes the SHA-256 digest of each source artifact.

## Claim ceiling

This tool validates structure, linkage and declared calibration status. It does not establish that an oracle classification is substantively correct, that an independent reviewer agrees, that the benchmark is representative, that an ACS deployment conforms, or that any product/system is secure. All included corpora and answer keys are public development material, not blind holdouts.
