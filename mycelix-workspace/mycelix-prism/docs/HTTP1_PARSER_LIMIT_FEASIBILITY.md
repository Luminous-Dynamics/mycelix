# HTTP/1 parser-limit feasibility matrix

**Status:** test-only Hyper feasibility probes; no production transport migration and no exact-head execution result implied.

## Gap

Prism's production egress transport remains Reqwest 0.13.5. It configures the HTTP/1 response header-count cap and HTTP/2 received header-list cap. The current Reqwest builder configuration does not impose a separate explicit aggregate HTTP/1 header-byte limit before the parsed response is returned. A post-parse byte check is not equivalent because parser allocations have already occurred.

Hyper 1.12.0 exposes an HTTP/1 client max_header_size limit for the response header block, including the status line. The same limit is applied to chunked trailers. Hyper separately exposes max_headers; that setting moves header storage to heap memory and its documentation describes an approximately 5% performance drop. These are separate resource dimensions and must be benchmarked independently. See the pinned Hyper API references in the PEAB protocol.

## Feasibility matrix

| ID | Fixture | Control | Required capped behavior | Scope |
|---|---|---|---|---|
| HTTP-002 | One response header with a value larger than 4 KiB | Identical response without max_header_size is accepted | A 1 KiB parser cap rejects before a response is delivered | One large field |
| HTTP-004 | Twelve individually small fields whose combined header block exceeds 1 KiB | Uncapped response succeeds; a 256-byte under-limit fixture succeeds with cap | Aggregate block rejected | Aggregate bytes |
| HTTP-005 | Chunked body with a trailer field larger than 2 KiB | Uncapped client drains successfully | Capped parser fails while body/trailers are consumed | Trailer bytes |

All probes use the same HTTP/1 field-count setting, loopback transport and bounded test timeouts. Negative controls are mandatory: without them, a failing connection/server fixture could masquerade as a parser-limit result.

## Qualification gate before a production Hyper prototype

A successful feasibility test only establishes that Hyper's parser API can reject these fixtures. It does not qualify Hyper as a drop-in replacement for Prism's Reqwest client.

Before migration, the prototype must preserve and independently test:

- resolve_to_addrs-equivalent pinning and complete A/AAAA answer policy;
- original URL hostname for HTTP Host, TLS SNI and certificate verification;
- explicit no-proxy behavior and disabled implicit retries;
- HTTP/1 and HTTP/2 parity, ALPN, connection reuse, cancellation, deadlines and redirects;
- fresh capability and DNS authorization for each redirect/retry;
- response body leases across Bytes ownership and Axum handoff;
- TLS trust-store behavior, compression/decompression bounds, and diagnostic body ceilings;
- peak RSS, allocation count, parser CPU, throughput, p50/p95/p99 latency, and cold/warm connection costs.

Do not swap transports solely because the parser-limit test passes. Compare complete egress semantics and operational costs against the current pinned Reqwest baseline.

## Evidence status

The probes are source additions. They remain not_run_exact_head until executed on the exact current PR SHA. They are partial source coverage, not a production property. No parser, compile, performance, or runtime security pass is claimed here.
