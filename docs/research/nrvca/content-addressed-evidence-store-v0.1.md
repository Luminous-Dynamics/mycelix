# NRVCA-018: Content-addressed evidence capture store v0.1

**Status:** implementation draft; Rust qualification pending; no real source captured in this workstream  
**Tracking issue:** [#5448](https://github.com/Luminous-Dynamics/mycelix/issues/5448)  
**Prior step:** [NRVCA-017 byte-verifying source intake](./byte-verifying-source-intake-v0.1.md)  
**Implementation:** `mycelix-workspace/tools/nrvca-validator/src/source_intake.rs`

## Paradigm shift: retain the bytes that the digest describes

NRVCA-017 computed a digest from bytes read at one moment but left the file in its original location. Later extraction could use a changed file while retaining the previous receipt. NRVCA-018 adds `capture-source`: it streams an input file into a private staging object while calculating SHA-256, checks for detectable source changes, compares any independently supplied expected digest, and only then promotes the staged object into a deterministic digest path.

Objects are stored as:

```text
<store>/
  objects/
    sha256/
      <first-two-hex-digits>/
        <full-64-hex-digit-sha256>
```

Promotion uses same-filesystem hard-link creation, which atomically refuses to overwrite an existing target. If the selected filesystem does not support hard links or the staged and destination paths are not on the same filesystem, capture fails closed rather than falling back to a potentially overwriting move. If a digest object already exists, its bytes and size are re-hashed and compared before reuse. The staged object is created with owner-only permissions on Unix, and persisted objects are owner-read-only by default (0400 on Unix); a pre-existing object is reusable only when it already has the safe read-only mode. The intake tool refuses to chmod an existing inode because it may be hard-linked elsewhere. Receipt files created by the tool are owner read/write only by default (0600 on Unix). These are safer defaults, not a guarantee against the owning administrator changing permissions later.

The resulting record uses a store-relative `object_path` and a `sha256:<digest>` URI. It deliberately does not put the original input's absolute path into the capture receipt. Store root/host configuration stays local.

## Usage

```sh
cargo run --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml -- capture-source \
  --file ./captured-source.pdf \
  --store-dir ./private-evidence-store \
  --source-id source-001 \
  --publisher "Publisher name" \
  --source-uri "https://example.org/report.pdf" \
  --publication-id REPORT-2026 \
  --vintage 2026 \
  --captured-at "2026-10-22T17:00:00+02:00" \
  --rights-status evaluation_only \
  --rights-evidence-uri "https://example.org/documented-permission" \
  --evaluation-allowed true \
  --redistribution-allowed false \
  --locator-id table-1-row-2 \
  --locator "PDF page 4, Table 1, row 2" \
  --extraction-method "PDF table extraction" \
  --tool-version "operator-recorded-extractor-version" \
  --expected-sha256 "PASTE-INDEPENDENTLY-RECORDED-64-HEX-DIGEST" \
  --output ./private-evidence-store/source-001.json
```

The fields above are placeholders. They do not assert a source was captured, permission was granted, or an expected digest exists. Omit `--expected-sha256` where no independent digest exists; the tool still derives the digest from captured bytes, but publisher/source authenticity must be established separately.

## Integrity sequence

1. Resolve and validate a regular input file before opening content.
2. Resolve the selected store; reject an object-root or digest-shard path that resolves outside the selected root.
3. Copy source bytes into a create-new staging object in bounded memory while hashing the exact stream written.
4. Flush and sync the staged object; compare byte count and file metadata before/after capture.
5. Reject an expected-digest mismatch before installing any digest-addressed object.
6. Mark the staged inode read-only, sync it, and use atomic no-overwrite hard-link promotion to the digest path.
7. If an object exists already, refuse symlinks/non-regular paths and verify its actual digest and byte count before reuse.
8. Sync the digest directory on Unix, re-read and verify the installed object, then emit a source record referencing the relative object path.
9. If the optional receipt output already exists, fail without replacing it. The valid object may remain stored; rerunning can reuse it after verification.

If the receipt file cannot be written after object promotion, the verified content object can remain as an unreferenced object. This is safe for integrity but requires later store inventory/garbage-collection tooling; no garbage collector is included in v0.1.

## Interoperability strategy

The content-addressed object store should be an implementation detail behind an interoperable evidence package—not a new standalone provenance standard. The next package-format step should map source objects and their metadata into **RO-Crate 1.3** (using a versioned profile), with derivation activities and responsible agents represented using **W3C PROV**. KPI definitions/units/vintages should reference **SDMX** concepts and unit metadata wherever those definitions can be mapped without semantic loss. Keep NRVCA's additional rights/adjudication fields as an explicit profile rather than claiming that the generic standards already encode this project's entire policy.

- [RO-Crate specification and versions](https://www.researchobject.org/ro-crate/specification.html) — packaging data for reuse/preservation and recording contextual and provenance metadata.
- [W3C PROV Primer](https://www.w3.org/TR/prov-primer/) — interoperable representation of entities, activities, and agents.
- [SDMX guidance](https://sdmx.org/guidelines/) — common statistical metadata, definitions, units, and vintage modelling.

This is a roadmap, not a conformance claim: the current JSON source record is not yet a valid RO-Crate package and does not assert SDMX conformance.

## Qualification and safety boundary

Unit tests exercise known-answer digest vectors, exact stored bytes, content-address reuse, mismatch cleanup, tampered pre-existing objects, store path/symlink containment, source-admission compatibility, read-only behavior, and duplicate-store argument rejection. The CLI integration suite checks capture output, expected digest mismatch, object reuse, and help. **None of these Rust tests has been executed in this session.**

The content store is content-addressed, not metaphysically immutable. The owner or administrator can change permissions; another process with write privileges can tamper with objects. Re-hashing objects when reused detects many accidental or deliberate changes, but does not provide a signed chain of custody. A later step should emit signed acquisition attestations or commit receipt digests to an append-only log, while carefully separating signing identity from content truth.

This tool also does not authenticate the publisher, verify legal rights, prove source truth, or guarantee the stream was a coherent point-in-time snapshot if another process mutates a same-size file without a detectable metadata change. For high-assurance intake, first copy the source to a controlled immutable incoming area, then capture that copy. It never enables external evaluation automatically.

Privacy matters: source record metadata and rights-evidence links may still be sensitive. The relative object path avoids publishing a host-local path, but use a private store for restricted sources and sanitize any metadata that should not be public. Hashing does not confer redistribution permission.
