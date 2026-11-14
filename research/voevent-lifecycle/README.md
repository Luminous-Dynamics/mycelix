# Parser-bound VOEvent lifecycle replay v0.2

A read-only Rust replay package that derives a VOEvent lifecycle graph from the **exact source XML bytes** and a separate ingress receipt timestamp.

This is the executable slice for [ASTRO-VOEVENT-LIFECYCLE-001](https://github.com/Luminous-Dynamics/mycelix/issues/6158). It depends directly on the bounded parser in [`research/voevent-header`](../voevent-header/README.md); it does not accept caller-supplied parsed metadata.

## Core design

The receipt manifest contains exactly two packet inputs:

- `raw_packet_xml`: the UTF-8 XML string whose encoded UTF-8 bytes are passed directly to the parser;
- `received_at_unix_ms`: a separately supplied ingress receipt value, never inferred from `Who/Date`, event time, file timestamps or publisher claims.

The replay library invokes `mycelix-voevent-header::parse_packet` on those exact bytes. It does not accept an independently supplied IVOID, role, publisher, digest or citation projection. Both packet and manifest structs reject unknown fields, so a parallel `packet` projection is a hard deserialization error.

The parser computes the exact source byte length and SHA-256. The replay records the parser package version with every visible packet, preserving reproducibility across parser changes. The adapter also preserves:
- all root attributes, including uninterpreted ones;
- source-asserted `Who/AuthorIVORN` and source-native `Who/Date`;
- every typed `Citations/EventIVORN` edge and all its attributes;
- optional `Citations/Description`;
- packet role and whether the observation default was implicit.

These are source-derived fields, not proof of publisher identity or scientific truth. The underlying original XML remains the authoritative source artifact; the replay output is a parser-versioned projection bound to its digest.

## Temporal contract

1. A packet is visible only when `received_at_unix_ms <= cutoff_unix_ms`.
2. Future packets are removed by receipt cutoff **before parser invocation**. Malformed future arrivals therefore cannot poison an earlier historical replay.
3. When the cutoff advances to include such a packet, it must pass the pinned parser or the replay fails closed.
4. Publication time remains a source-native lexical value; it cannot backdate fabric visibility.
5. Unresolved citations remain explicit and may resolve in later replays without rewriting the earlier view.
6. Supersessions and retractions add append-only effects; they never delete the cited packet or original assertion.

The ingress timestamp is still an input assertion. This package does not attest the timestamp or provide a trusted append-only ingress log. Production use must bind it to an independently auditable receipt.

## Run

From the repository root:

```bash
cargo fmt --manifest-path research/voevent-header/Cargo.toml --all -- --check
cargo test --manifest-path research/voevent-header/Cargo.toml --all-targets
cargo clippy --manifest-path research/voevent-header/Cargo.toml --all-targets -- -D warnings

cargo fmt --manifest-path research/voevent-lifecycle/Cargo.toml --all -- --check
cargo test --manifest-path research/voevent-lifecycle/Cargo.toml --all-targets
cargo clippy --manifest-path research/voevent-lifecycle/Cargo.toml --all-targets -- -D warnings
```

The lifecycle package uses a local path dependency on the parser package. Normal Cargo dependency resolution may require the configured crate cache or registry; neither parser nor replay performs network access.

Replay a fixture:

```bash
cargo run --manifest-path research/voevent-lifecycle/Cargo.toml --bin voevent-lifecycle-replay -- research/voevent-lifecycle/tests/fixtures/parser-projection-thread.json
```

Despite the fixture's historical filename, its v0.2 contents are raw-source-only; no parsed projection is present.

## Manifest shape

```json
{
  "cutoff_unix_ms": 200,
  "packets": [
    {
      "received_at_unix_ms": 100,
      "raw_packet_xml": "<VOEvent ivorn=\"ivo://survey.example/event?one\" version=\"2.1\"><What/></VOEvent>"
    }
  ]
}
```

The strict Draft 2020-12 interchange schema is `RECEIPT_MANIFEST_V0_2.schema.json`. The Rust `deny_unknown_fields` contract is enforced even when a caller bypasses JSON Schema validation. A 64 MiB CLI manifest bound and 2 MiB per-packet parser bound constrain untrusted input sizes.

## Parser and semantic claim boundary

The parser implements a bounded VOEvent 2.1 extraction profile. It validates the root identity/version/role and the supported `Who` and `Citations` structures. It is **not** a complete VOEvent 2.1 XSD validator. The broader `What`, `WhereWhen`, `How`, `Why`, `References`, complete STC coordinate/time/frame semantics and external content are not yet parsed by this profile. Accordingly, `Packet.event_time` remains absent and no UTC/TT/TDB conversion is attempted.

The pipeline does not authenticate IVOIDs or publishers, validate XML signatures, qualify rights, calibration or source independence, authorize instruments, or establish astrophysical truth. A parsed publisher identifier is an attributed source assertion, not authenticated identity. A correct SHA-256 proves byte identity relative to the supplied bytes, not who produced them or whether the observation is true.

## Tests and benchmark status

The parser crate owns the syntax/metadata extraction tests. The lifecycle crate tests parser-derived identity/role/Who/citation extraction, exact digest change on source-byte change, root/citation attribute preservation, strict rejection of caller-supplied projections, receipt-time cutoff isolation, unresolved references, lifecycle cycles, contradiction preservation and deterministic replay.

These are synthetic known-answer tests. They do not establish arbitrary XML conformance or real astronomical validity. Proposed benchmark acceptance targets remain targets, not achieved results. See [BENCHMARKS.md](BENCHMARKS.md).

## Industry positioning

VOEvent 2.1 already standardizes event packets and lifecycle citations, and Rubin's ecosystem already has mature alert brokers for filtering, cross-matching, contextual enrichment, classification and prioritization. This is therefore designed as a broker-interoperable evidence and investigation layer—not a new high-volume alert broker.

The differentiator to test is whether a reviewer can reconstruct exactly which source bytes and parser version produced each event assertion, what was known at a specified receipt cutoff, and why an investigation accepted, rejected, deferred or abstained without treating duplicated broker outputs as independent observations.
