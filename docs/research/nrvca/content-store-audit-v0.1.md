# NRVCA-019: Read-only content-store integrity audit v0.1

**Status:** implementation draft; source/tests have not been executed in this session; no real sources audited  
**Tracking issue:** [#5449](https://github.com/Luminous-Dynamics/mycelix/issues/5449)  
**Prior step:** [NRVCA-018 content-addressed evidence capture](./content-addressed-evidence-store-v0.1.md)

## Why this is a separate assurance layer

NRVCA-018 verifies an object after capture and verifies an existing object before reuse. Those checks are point-in-time. A stored file can be changed later, a file can be placed under a false digest name, or unexpected symlinks and writable files can enter the tree. `verify-store` is a repeatable read-only audit of the *current* store state.

The command walks `objects/sha256/<two-hex-shard>/<64-lowercase-hex-digest>`, streams every candidate object's bytes through SHA-256, compares the digest with the filename, checks the shard prefix, refuses symlinks/non-regular objects, and checks the read-only permissions set by capture. It also reports malformed shards, unexpected entries and unsafe path types. It does not repair, delete, chmod, or rewrite store objects.

## Usage

```sh
cargo run --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml -- verify-store \
  --store-dir ./private-evidence-store
```

The command prints a deterministic JSON report to stdout. A `failed` report is still emitted as JSON and the process exits nonzero. An empty store reports `empty`, not `pass`: no files verified is not evidence that a source corpus passed. Operational failures such as an unreadable directory return an error and are reported on stderr.

Example report shape:

```json
{
  "schema_version": "nrvca-content-store-audit/0.1",
  "status": "pass",
  "entries_scanned": 3,
  "objects_verified": 1,
  "objects_failed": 0,
  "bytes_verified": 3,
  "diagnostics_count": 0,
  "diagnostics": []
}
```

The counts in this example are illustrative, not an observed run. `entries_scanned` counts entries visited in the object tree; early rejection of an invalid root shape may stop traversal. `objects_failed` counts individual object files that fail object checks; malformed directory layout is visible in `diagnostics_count` even when no object file could be counted. `bytes_verified` counts only bytes from objects that passed both digest and permission checks. Diagnostic records are sorted by relative path then diagnostic code; there is intentionally no wall-clock timestamp in the report so repeated audits of unchanged state can be byte-for-byte reproducible.

## Static test inventory (not executed here)

The Rust unit test module now defines 25 tests in total, including the pre-existing intake tests and store-audit tests for:

- healthy-store verification and deterministic output;
- empty-store distinction;
- post-capture byte corruption;
- invalid shard names, extra entries and non-UTF-8 paths;
- symlinked object rejection on Unix;
- writable-object rejection on Unix;
- proving a valid audit leaves object bytes and permissions unchanged.

The CLI integration suite defines four audit tests for successful exact counts, empty status, corrupted-object nonzero exit with machine-readable output and no repair, and help output. These definitions have been statically re-fetched from the branch, but the tests have **not** been run. Exact-head CI evidence remains required.

## Industry comparison: complement standards instead of inventing a new package format

NRVCA's object tree is a narrow content-addressed storage and integrity-audit mechanism, not a replacement for archive/package standards.

- **GNU `sha256sum --check`** verifies file contents against a previously generated checksum list. That is a strong primitive, but the checksum list itself must be trusted and maintained; NRVCA additionally checks its own digest-addressed path convention, file types, Unix permission policy, and emits a structured JSON audit report. It should be performance-compared against `sha256sum --check` on the exact same object corpus, not asserted to be faster without measurement. [GNU Coreutils SHA-2 utilities](https://www.gnu.org/software/coreutils/manual/html_node/sha2-utilities.html)
- **BagIt (RFC 8493)** standardizes packaging arbitrary payload files plus checksum manifests and defines a valid bag in terms of complete manifests whose checksums verify. That is the better interoperability baseline for transferring/preserving a corpus; NRVCA does not currently implement BagIt manifest parsing or claim BagIt conformance. A next integration should export/import a BagIt-compatible package and cross-verify checksums. [RFC 8493](https://www.rfc-editor.org/rfc/rfc8493.html)
- **RO-Crate 1.3** provides a machine-readable JSON-LD package metadata model for datasets, files, context, collection/creation and reuse metadata. The NRVCA JSON receipt is not currently a conforming RO-Crate. Keep the content store as the byte layer and map source, activity, rights and evidence metadata into a versioned RO-Crate profile rather than inventing another competing metadata standard. [RO-Crate 1.3 specification](https://www.researchobject.org/ro-crate/specification.html)

The practical advantage NRVCA is targeting is *composed assurance*: byte-level identity, reproducible source intake, fail-closed path checks, rights/adjudication metadata, and deterministic machine-readable audit status. That advantage is a design hypothesis until benchmarked against established tools and real, lawfully acquired corpora.

## Benchmark protocol and targets

All results must record exact source commit, OS/kernel, filesystem, CPU, RAM, object count, total bytes, file-size distribution, cache condition, and command/tool versions. Compare the same immutable dataset against at least the host's `sha256sum --check` and a validated BagIt implementation. Do not report synthetic mutation percentages as real-world error rates.

| Measure | Acceptance target | Evidence needed |
|---|---|---|
| Valid-object acceptance | 100% of a pre-registered clean corpus accepted on supported filesystems | Manifest of object names/digests, independent checksum validation, repeat runs |
| Mutation detection | 100% of the preregistered mutations detected; zero false accepts in negative controls | At least bit flip, truncation, append, same-size substitution, wrong digest path, wrong shard, symlink, non-file, unexpected entry and unsafe permission cases |
| Determinism | Byte-identical JSON for unchanged store state over 100 consecutive runs | SHA-256 of the canonical stdout report per run |
| Non-destructive operation | Zero changes to file bytes, object paths or permissions caused by the audit | Before/after tree inventory, content hashes and permission snapshot |
| Throughput | Initial engineering target: at least 80% of `sha256sum --check` throughput on large sequential objects, with the same hash algorithm/hardware/filesystem | Repeated 1 GiB+ and mixed-size runs, both cold-cache and warm-cache, median and p95; this target is unmeasured and may be revised on evidence |
| Scale | Exercise 1 million small objects and large-object corpora without holding every directory entry in memory | Peak RSS, wall time, read errors, objects/second and GiB/second; the implementation now streams directory enumeration, while retaining diagnostics in memory when failures occur |
| Operational safety | Corrupt-store command exits nonzero while still emitting a parsable failure report; an empty store is never labelled pass | CLI integration tests and independently checked exit codes/stdout/stderr |

The 80% throughput value is a target, not a current result. Because path, permission and structure checks add work beyond a checksum-only run, its purpose is to reveal excessive overhead rather than imply exact parity is already achieved. Report both end-to-end throughput and raw bytes hashed. Test stores with many small files separately from large files; hashing throughput alone hides filesystem metadata overhead.

## Limits

A digest verifies identity relative to the computed hash, not publisher authenticity, licensing, semantic correctness, source truth, or when bytes were first obtained. Permissions are a policy signal, not WORM storage: a privileged owner can change bytes or mode after the audit. Re-run audits on an operational schedule and before downstream evaluation. For tamper-evident history, follow with signed audit receipts or an append-only transparency log; do not silently rewrite earlier receipts. The current tool does not yet verify signed attestations or a historical event chain.

Concurrent changes during audit are only detected where file size or modification-time checks expose them; there is no filesystem snapshot/lock guarantee. For high-assurance preservation, audit an immutable snapshot and preserve its signed manifest separately.
