# VOEvent lifecycle replay v0.1

A read-only Rust implementation for replaying lifecycle citations from exact VOEvent XML bytes as they were **received by a specified cutoff**.

This is the executable slice tracked by [ASTRO-VOEVENT-LIFECYCLE-001](https://github.com/Luminous-Dynamics/mycelix/issues/6158). It directly depends on the same bounded VOEvent parser in [#6157](https://github.com/Luminous-Dynamics/mycelix/pull/6157) and its typed citation extension [#6180](https://github.com/Luminous-Dynamics/mycelix/pull/6180), so replay metadata is derived from the exact bytes being hashed. The full evaluation protocol is in [BENCHMARKS.md](BENCHMARKS.md). Source-native STC follow-on work is tracked in [ASTRO-WHEREWHEN-001](https://github.com/Luminous-Dynamics/mycelix/issues/6209).

## Why this exists

VOEvent 2.1 uses directed citations between packet instances:

- **followup**: the new packet reports a follow-up to a cited packet;
- **supersedes**: the new packet asserts that the cited packet is superseded;
- **retraction**: the new packet asserts that the cited packet is retracted.

Those are source assertions, not independent proof that the new packet is correct. The replay therefore keeps the original packet and every assertion. It does not collapse an event thread to a single “latest” record, silently repair cycles, or treat a publisher identifier as authenticated identity.

## Temporal contract

The API is deliberately an **as-known-at** view:

1. A packet is visible only when received_at_unix_ms is less than or equal to the replay cutoff.
2. Publication time and phenomenon/event time are preserved as source strings; neither can make a packet visible early.
3. Validation is scoped to the cutoff. A malformed or duplicate packet received in the future cannot alter a past replay.
4. References to packets not yet visible remain explicit unresolved edges. They can resolve in a later replay without rewriting the earlier result.
5. Retractions and supersessions add effects; they never delete historical packets or citations.
6. The exact source XML is parsed by the canonical parser library in the same call that computes its byte length and SHA-256; the replay layer does not accept a caller-supplied metadata projection.

The receipt timestamp is an input from the ingest layer; this crate does not attest that timestamp or provide an append-only receipt log. A production fabric must bind it to a trusted, independently auditable ingress receipt.

This crate performs bounded VOEvent XML extraction using the adjacent parser, but it does not perform full XSD/STC validation, parse or convert WhereWhen time scales, interpret coordinate frames, authenticate IVOIDs or publishers, check signatures, evaluate rights, qualify calibration, infer physical independence, or assess astrophysical truth. An attributed publisher is not an authenticated publisher. The digest is computed from the exact string's UTF-8 bytes, but the ingress timestamp remains unauthenticated until backed by a trusted receipt.

## Run

From this directory:

    cargo fmt --all -- --check
    cargo test --all-targets
    cargo clippy --all-targets -- -D warnings

The isolated package uses a local path dependency on the adjacent VOEvent parser plus pinned JSON dependencies. Normal Cargo dependency resolution may require the configured crate cache or registry; parsing and replay make no network requests.

## Wall-clock replay tests in v0.1

The synthetic unit corpus covers:

- late-arriving supersession must not leak into an earlier replay;
- retraction by a different publisher with the original packet retained;
- follow-up does not imply supersession;
- unresolved references resolve only after target receipt;
- future duplicate/malformed packets do not poison historical cutoffs;
- supersession cycles are diagnosed without rewriting the graph;
- source-native time lexemes and scales are preserved;
- deterministic output independent of input packet order;
- simultaneous supersession and retraction remain separately visible;
- self-references are reported and retained.

These are synthetic known-answer tests. They establish behavior of this implementation, not conformance of arbitrary XML documents or correctness of any physical observation.

## Benchmark programme and proposed gates

This package is a semantic oracle for a larger scientific-observability benchmark. Before comparing model or agent systems, freeze the packet corpus, receipt-time cutoffs, expected graph snapshots, and hidden adversarial cases.

| Measure | v0.1 target |
|---|---:|
| Historical no-lookahead violations | 0 |
| Lost source citations or silently deleted packets | 0 |
| Determinism across input permutations | 100% |
| Known-answer lifecycle edge/effect fidelity | 100% |
| Contradictory relation coercions | 0 |
| Rights, source-authentication, or instrument-authority bypasses | 0 |
| Sequence-cutoff look-ahead or chain-prefix mismatch accepted | 0 |
| Receipt-order violations under clock rollback | 0 |

Do not report these as achieved benchmark scores until the complete frozen corpus and independent evaluator exist. The current unit tests exercise the cases listed above and use the same parser crate directly; they are not real astronomical observations.

For a full investigation benchmark, report per-domain results rather than one pooled score: proper prediction scores, event false alarms per exposure, decision regret at equal resource budgets, follow-up cost, time-to-resolution, reviewer burden, worst-stratum performance, and reproducibility. Include no-follow-up, fixed-schedule, expert/broker, anomaly-greedy, information-gain-greedy, decision-theoretic EVSI/ENBS, and model/agent baselines. A hidden-truth oracle is a synthetic upper bound only.


## Replay exact source packets

Each manifest entry contains only the exact original UTF-8 XML and a caller-supplied receipt-time assertion. The lifecycle crate calls the shared parser library directly; there is no caller-supplied metadata projection. The output explicitly sets `receipt_time_basis` to `unattested_manifest_assertion`.

    {
      "received_at_unix_ms": 1731580000000,
      "raw_packet_xml": "<VOEvent ivorn=\\"ivo://publisher.example/stream?event-1\\" version=\\"2.1\\">...</VOEvent>"
    }

The shared parser derives packet identity, role, publisher attribution, source creation time, citation relations and SHA-256 from the same bytes. The timestamp is never derived from Who/Date or phenomenon time. It remains an unauthenticated assertion in this research CLI and must not be represented as a verified production receipt.

Run the CLI:

    cargo run --manifest-path research/voevent-lifecycle/Cargo.toml --bin voevent-lifecycle-replay -- research/voevent-lifecycle/tests/fixtures/source-packet-thread.json

The JSON output contains visible packets, lifecycle edges, unresolved citations, effects, diagnostics and an explicit `receipt_time_basis: unattested_manifest_assertion` marker. Visible packets preserve root attributes and source description text; resolved and unresolved edges preserve their original EventIVORN attributes. Future receipts are excluded before XML parsing, so malformed future packets do not poison historical views; they are parsed and rejected when a later cutoff includes them.

The parser computes the exact packet digest. The lifecycle layer does not accept an externally supplied digest or metadata projection, removing the projection-versus-byte mismatch surface. It does not authenticate the receipt-time assertion, publisher, source rights or astrophysical truth. Production as-known-at queries must consume the canonical signed DeSci authority receipt and receipt-chain verifier owned by [KNOW-ASOF-001 #5860](https://github.com/Luminous-Dynamics/mycelix/issues/5860); this CLI does not implement that verifier.

## Receipt-sequenced replay v0.2

The wall-clock manifest API above remains a synthetic/retrospective compatibility path. It marks its cutoff as `unattested_manifest_assertion`. For an as-known-at query, use the **receipt sequence** API instead.

### Why sequence, not wall-clock time?

An ingest clock can drift, be reset, be misconfigured or be manipulated. A packet received later can therefore carry an earlier timestamp. Sorting lifecycle assertions by wall clock can also reorder a thread after clock rollback.

Receipt-sequenced replay uses a monotonically increasing journal sequence as the inclusion and causal-order boundary. Wall-clock receipt time remains preserved metadata, but it does not decide which packets were visible or how the journal is ordered.

### Receipt chain v1

Each `ReceiptJournalEntry` commits to:

- a 1-based sequence number;
- the source-reported receipt time in Unix milliseconds;
- the SHA-256 of the exact raw packet XML bytes;
- the previous receipt digest;
- the current receipt digest.

The receipt digest uses SHA-256 with the domain tag `MYCELIX:VOEVENT:INGRESS-RECEIPT:V1\0`, then fixed-width big-endian sequence/time fields and the two 32-byte digests. The chain begins at the fixed genesis commitment exposed by `receipt_genesis_sha256()`.

`seal_receipt_entry()` creates a journal entry from raw XML and the prior receipt digest. `replay_receipt_sequence()` verifies the packet bytes, digest, complete sequence prefix, previous-hash links and final checkpoint before invoking the same VOEvent parser used by the packet CLI. Future sequence entries are filtered before packet parsing; a malformed future packet cannot poison an earlier checkpoint.

### Separate checkpoint and CLI

The replay takes an independently supplied `ReceiptCheckpoint` with a sequence and expected terminal receipt digest. The checked-in example deliberately has a wall-clock rollback: sequence 2 has an earlier `received_at_unix_ms` than sequence 1, but it remains later in the replay.

    cargo run --manifest-path research/voevent-lifecycle/Cargo.toml --bin voevent-receipt-replay -- research/voevent-lifecycle/tests/fixtures/receipt-journal-clock-rollback.json research/voevent-lifecycle/tests/fixtures/trusted-receipt-checkpoint.json

The journal and checkpoint are separate files. The replay output reports `receipt_time_basis = hash_chained_sequence_checkpoint`, `cutoff_sequence`, `cutoff_receipt_sha256`, each packet's `receipt_sequence`, and each lifecycle edge's asserting receipt sequence. Wall-clock cutoff is `null` for this mode.

### Exact security boundary

The chain detects mutation of packet bytes, receipt timestamps, sequence values, previous-hash links, missing prefix entries and a checkpoint that does not match the supplied prefix. It also prevents later sequence entries from appearing in an earlier replay even when their wall-clock timestamp is backdated.

It does **not** prove that the timestamp was correct when first recorded. A hash chain does not authenticate the ingress host, the operator, or the checkpoint publisher. An operator able to rewrite the entire journal and replace the expected checkpoint can create a different internally consistent history. The checkpoint must therefore be pinned or signed/witnessed through a separate trust channel before production decisions rely on it. This v0.2 chain is **not** an RFC 9162 Certificate Transparency implementation: it has no Merkle inclusion/consistency proofs, no signed tree head, and no gossip/fork detection. Those are the next transparency-log increment, not capabilities claimed here.

The research implementation caps the accepted prefix at 1,000,000 entries and each packet at 2 MiB. The CLI separately bounds the journal file to 64 MiB and the checkpoint file to 4 KiB. Production ingress needs durable append/fsync semantics, key rotation/revocation, independent witnesses, checkpoint retention, backup/restore and operational recovery procedures; this library only defines and verifies the deterministic receipt-chain format.

## Complexity

Replay sorts the visible packets and traverses the visible citations. No throughput claim is made by v0.1; benchmark scaling at 10^3, 10^4, 10^5, and 10^6 packets before selecting an indexed or streaming implementation.
