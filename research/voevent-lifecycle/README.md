# VOEvent lifecycle replay v0.1

A dependency-free, read-only Rust reference implementation for replaying the lifecycle citations of VOEvent packets as they were **received by a specified cutoff**.

This is the executable slice tracked by [ASTRO-VOEVENT-LIFECYCLE-001](https://github.com/Luminous-Dynamics/mycelix/issues/6158). It complements, rather than replaces, the bounded XML root-header parser in [#6157](https://github.com/Luminous-Dynamics/mycelix/pull/6157). The full evaluation protocol is in [BENCHMARKS.md](BENCHMARKS.md).

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
6. The parser projection is bound to the supplied exact UTF-8 XML bytes by both byte length and a SHA-256 recomputed by this adapter before replay.

The receipt timestamp is an input from the ingest layer; this crate does not attest that timestamp or provide an append-only receipt log. A production fabric must bind it to a trusted, independently auditable ingress receipt.

This crate does not parse timestamps, convert TT/TDB/UTC, interpret coordinate frames, validate XML, authenticate IVOIDs or publishers, check signatures, evaluate rights, qualify calibration, infer physical independence, or assess astrophysical truth. An attributed publisher is not an authenticated publisher. A carried digest is not a verified digest unless the upstream byte-binding step has been independently checked.

## Run

From this directory:

    cargo fmt --all -- --check
    cargo test --all-targets
    cargo clippy --all-targets -- -D warnings

The isolated package uses pinned direct Rust dependencies for JSON serialization, SHA-256 and hexadecimal encoding. Normal Cargo dependency resolution may require the configured crate cache or registry; the replay itself makes no network requests.

## Tests in v0.1

The 12-case unit corpus covers:

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

Do not report these as achieved benchmark scores until the complete frozen corpus and independent evaluator exist. The current unit tests only exercise the cases listed above.

For a full investigation benchmark, report per-domain results rather than one pooled score: proper prediction scores, event false alarms per exposure, decision regret at equal resource budgets, follow-up cost, time-to-resolution, reviewer burden, worst-stratum performance, and reproducibility. Include no-follow-up, fixed-schedule, expert/broker, anomaly-greedy, information-gain-greedy, decision-theoretic EVSI/ENBS, and model/agent baselines. A hidden-truth oracle is a synthetic upper bound only.


## Replay the parser's JSON packet projections

The `voevent-lifecycle-replay` binary consumes a JSON manifest containing parser projections plus a separately supplied ingest receipt timestamp for each packet. The manifest cutoff is also explicit. A sample fixture is available at `tests/fixtures/parser-projection-thread.json`.

Run it from the repository root:

    cargo run --manifest-path research/voevent-lifecycle/Cargo.toml --bin voevent-lifecycle-replay -- research/voevent-lifecycle/tests/fixtures/parser-projection-thread.json

The output is a JSON `Replay` containing visible packets, lifecycle edges, unresolved citations, effects and diagnostics.

The expected packet shape comes from the `parse_packet` JSON projection emitted by `research/voevent-header`. Each manifest entry includes the exact original UTF-8 XML string, the parser projection and a separately supplied ingress receipt:

    {
      "received_at_unix_ms": 1731580000000,
      "raw_packet_xml": "<VOEvent ivorn=\\"ivo://publisher.example/stream?event-1\\" version=\\"2.1\\">...</VOEvent>",
      "packet": {
        "header": {
          "ivoid": "ivo://publisher.example/stream?event-1",
          "version": "2.1",
          "packet_byte_length": 123,
          "packet_sha256": "<64 hex characters>"
        },
        "who": {
          "author_ivoid": "ivo://publisher.example/stream",
          "source_created_at": "2026-11-14T08:00:00Z"
        },
        "citations_present": false,
        "citations": []
      }
    }

The adapter checks version 2.1, the 2 MiB packet limit, exact UTF-8 byte length and a freshly recomputed SHA-256 against the parser projection before accepting a visible packet. It also checks role consistency, citation presence/edge consistency and IVOID lexical shape. Unknown parser fields (such as root attributes and the citation description) are ignored by the replay DTO so the projection can evolve without breaking the minimum replay contract; evidence-bearing fields remain required. This verifies that the projection's digest refers to the provided XML bytes, but it does not re-run the XML parser or prove that every projected metadata field was faithfully derived from those bytes. It also does not authenticate AuthorIVORN.

The ingest layer must supply `received_at_unix_ms` from its own receipt record. Do not derive it from `Who/Date`, the XML file's modification time, event time, or a publisher's assertion. This manifest format carries the receipt value; it does not authenticate it or replace an append-only ingress receipt log.

This version maps `Who/AuthorIVORN` to asserted publisher attribution and `Who/Date` to source creation/publication time. `WhereWhen` event time and time-scale/frame metadata are not yet extracted, so `Packet.event_time` remains absent. No UTC/TDB/TT conversion is attempted.

Historical cutoff filtering occurs before projection validation: malformed packets whose supplied receipt time is later than the requested cutoff cannot poison that historical view. The same packet is validated when a later cutoff includes it. The CLI's 64 MiB manifest bound and each packet's 2 MiB bound limit untrusted input size.

## Complexity

Replay sorts the visible packets and traverses the visible citations. No throughput claim is made by v0.1; benchmark scaling at 10^3, 10^4, 10^5, and 10^6 packets before selecting an indexed or streaming implementation.
