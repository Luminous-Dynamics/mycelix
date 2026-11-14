# VOEvent lifecycle replay v0.1

A read-only Rust implementation for replaying lifecycle citations from exact VOEvent XML bytes as they were **received by a specified cutoff**.

This is the executable slice tracked by [ASTRO-VOEVENT-LIFECYCLE-001](https://github.com/Luminous-Dynamics/mycelix/issues/6158). It directly depends on the same bounded VOEvent parser in [#6157](https://github.com/Luminous-Dynamics/mycelix/pull/6157) and its typed citation extension [#6180](https://github.com/Luminous-Dynamics/mycelix/pull/6180), so replay metadata is derived from the exact bytes being hashed. The full evaluation protocol is in [BENCHMARKS.md](BENCHMARKS.md).

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

The isolated package uses a local path dependency on the adjacent VOEvent parser plus pinned JSON dependencies. Normal Cargo dependency resolution may require the configured crate cache or registry; parsing and replay make no network requests.

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


## Replay exact source packets

Each manifest entry contains only the exact original UTF-8 XML and a separately supplied ingress receipt. The lifecycle crate calls the shared parser library directly; there is no caller-supplied metadata projection.

    {
      "received_at_unix_ms": 1731580000000,
      "raw_packet_xml": "<VOEvent ivorn=\\"ivo://publisher.example/stream?event-1\\" version=\\"2.1\\">...</VOEvent>"
    }

The shared parser derives packet identity, role, publisher attribution, source creation time, citation relations and SHA-256 from the same bytes. The ingress receipt timestamp is never derived from Who/Date or phenomenon time. It remains a separate assertion and must ultimately be backed by an independently auditable receipt log.

Run the CLI:

    cargo run --manifest-path research/voevent-lifecycle/Cargo.toml --bin voevent-lifecycle-replay -- research/voevent-lifecycle/tests/fixtures/parser-projection-thread.json

The JSON output contains visible packets, lifecycle edges, unresolved citations, effects and diagnostics. Future receipts are excluded before XML parsing, so malformed future packets do not poison historical views; they are parsed and rejected when a later cutoff includes them.

The parser computes the exact packet digest. The lifecycle layer does not accept an externally supplied digest or metadata projection, removing the projection-versus-byte mismatch surface. This still does not authenticate the ingress receipt or publisher, verify source rights, or establish astrophysical truth.

## Complexity

Replay sorts the visible packets and traverses the visible citations. No throughput claim is made by v0.1; benchmark scaling at 10^3, 10^4, 10^5, and 10^6 packets before selecting an indexed or streaming implementation.
