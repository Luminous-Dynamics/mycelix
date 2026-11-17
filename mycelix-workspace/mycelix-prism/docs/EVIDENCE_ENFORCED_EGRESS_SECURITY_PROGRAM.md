# Prism Evidence-Enforced Egress Security Programme

Status: design and verification plan; implementation is intentionally staged.
Scope: `mycelix-workspace/mycelix-prism` in the Luminous-Dynamics/mycelix monorepo.

## Executive decision

Prism must stop treating URL validation as the network security boundary. A URL is an untrusted request for authority. Every outbound operation must pass through one policy-enforced egress boundary that authorizes the destination actually connected to, not merely the hostname originally parsed.

The target architecture is:

`Untrusted intent -> URL normalization -> context policy -> DNS resolution -> address-set classification -> pinned connection -> redirect re-authorization -> bounded streaming -> receipt`

A request is denied when any required decision is unknown. No implicit fallback to the system resolver, environment proxy, automatic redirects, unbounded body buffering, or permissive defaults.

## Findings verified against the current main branch

The following are source-level findings, not claims that an exploit has been demonstrated:

1. `prism-common/src/ssrf.rs` rejects selected literal addresses and hostnames, but domain names pass through without resolution or connection pinning. The validator is useful input hygiene, not an SSRF boundary.
2. `prism-proxy` and `prism-serve` use automatic limited redirects. Their 10 MiB response check occurs after `Response::bytes()`, so the check does not bound memory consumed while downloading.
3. `prism-net` uses automatic redirects and checks the declared content length before calling `Response::text()`. A missing, misleading, or compressed content length does not provide a hard streaming bound. The fetch path also has no shared destination policy in the inspected implementation.
4. `prism-tauri` fetches the supplied URL directly with an automatically redirecting client and reads the full body as text. This is a distinct trust context: user-directed local/LAN browsing may be legitimate, so it must not silently inherit a server-proxy public-only policy.
5. The Prism ingest binaries use blocking HTTP clients and need an explicit classification as controlled offline/build-time ingestion, not an unexamined exemption.
6. The current proxy URL policy is a partial deny-list. It does not establish that the connected address is public, and it does not protect against DNS rebinding/TOCTOU.

## Security model: separate trust contexts

Do not force every fetch operation into one coarse policy.

- **Server proxy / hosted service:** public Internet destinations by default; deny loopback, private, link-local, shared, reserved, multicast, unspecified, documentation, benchmarking, and other non-global/special-purpose ranges. Exceptions require an explicit operator-owned allowlist with a narrow port/host scope.
- **Desktop browser navigation:** user-initiated LAN and loopback access may be valid. Keep this capability explicit and user-visible. A remote web page, preview, metadata fetch, extension, or background task must not inherit the user's local-network authority.
- **Build-time ingestion:** fixed source allowlists, bounded timeouts and body sizes, reproducible source manifests, and no arbitrary user-provided URL entry point.
- **Provider API calls:** fixed provider origins and credentials scoped to the provider. Do not route provider credentials through arbitrary redirect targets.

The policy must be explicit in types (for example `EgressContext::HostedProxy`, `DesktopNavigation`, and `PinnedProvider`), not inferred from which binary happens to call it.

## Required invariants

1. **One egress choke point:** all untrusted-URL network operations use the same policy and transport implementation. Direct clients are allowed only for documented fixed-origin/provider or offline-ingest cases.
2. **Resolve then pin:** resolve A and AAAA records once for each hop. Reject the hostname if the answer set is empty or contains any prohibited address. Connect only to the approved address set while retaining the original hostname for HTTP Host, TLS SNI, and certificate validation. Never perform a second unvalidated lookup between decision and connect.
3. **No ambient proxy bypass:** disable environment/system proxy discovery for the pinned transport, or model the proxy as a separately trusted egress hop with equivalent enforcement. A remote proxy can otherwise perform DNS resolution outside the policy boundary.
4. **Redirects are new requests:** disable automatic redirects. Parse and authorize each Location independently, re-resolve and pin it, cap hops, detect loops, reject scheme downgrades and unsupported schemes, and strip sensitive headers on origin changes.
5. **Bound while reading:** enforce a hard byte budget on streamed response chunks, including decompressed bytes. A Content-Length precheck is an optimization only. Stop reading and cancel as soon as the budget is exceeded.
6. **Bound all dimensions:** request body bytes, response bytes, header bytes/count, URL length, redirect count, connection time, total deadline, concurrent fetches, and per-origin concurrency.
7. **No unsafe success defaults:** missing, malformed, timed-out, or unknown policy/safety values deny the operation. In particular, an absent safety signal must never be interpreted as `safe = true`.
8. **Receipts are evidence, not authority:** emit a structured receipt containing policy version, context, normalized origin, resolved address set, selected peer address where observable, redirect chain, byte counts, decision, and reason. Do not log credentials, sensitive query values, or response bodies.
9. **Defense in depth:** production deployment also blocks internal/control-plane ranges at the network layer. Application checks are not a substitute for egress firewalling or isolation.

## Interim hardening already applied on the feature branch

The first runtime increment is deliberately fail-closed while the full transport is built:

- `prism-proxy` and `prism-serve` incrementally read response chunks and stop before appending a chunk that would exceed the 10 MiB budget. This bounds accumulated response memory; it is not a streaming-to-client implementation.
- All four upstream response paths in each binary use the bounded reader (generic proxy, DuckDuckGo, Brave, Perplexity).
- Automatic redirects are disabled in both binaries. Redirect responses are returned rather than followed; this temporarily sacrifices transparent redirect support to avoid following an unvalidated Location.
- Ambient system proxy discovery is disabled in both binaries with `no_proxy()`. A future explicitly configured egress proxy must be treated as a trusted network hop and included in the policy model.

These changes are not the completed SSRF boundary. DNS resolution/pinning, per-hop redirect support, deterministic tests, and the other network clients remain open. The workflow status reported for an implementation commit was `skipped`, not passed.

## Implementation sequence

### E0 — inventory and baseline

- Keep a maintained inventory of every outbound HTTP client, socket, subprocess, and browser fetch path under Prism.
- Classify each by trust context, caller-controlled inputs, redirect behavior, resolver behavior, body handling, credentials, and network privileges.
- Add a negative-control test that proves a newly introduced direct client is detected by the inventory check.

### E1 — shared egress crate

Create a native-only `prism-egress` crate (do not put reqwest/tokio into `prism-common`, which is shared with WASM). It should expose a small typed API:

- `EgressPolicy` / `EgressContext`
- `DestinationDecision` with explicit allow/deny reason
- `Resolver` abstraction for deterministic tests
- `PinnedTransport` that owns resolution, connection pinning, redirects, limits, and receipt generation
- `FetchLimits` for request/response/header/time/concurrency bounds

Keep address classification pure and deterministic. Prefer a maintained IANA special-purpose address registry or a carefully reviewed checked-in range table with provenance and update tests; do not rely only on `is_private()`.

### E2 — transport correctness

- Use reqwest's per-host address override (`resolve_to_addrs`) or a connector that demonstrably pins the actual socket destination. Verify the exact reqwest version and features in this workspace before implementation.
- Use a fresh per-hop transport/client or a rigorously scoped connection pool so a previous DNS decision cannot authorize a different origin/address.
- Set redirect policy to none and implement a bounded redirect state machine.
- Stream and count bytes after decompression; abort immediately on overflow.
- Ensure request and response timeouts cover the complete operation, not just connection establishment.

### E3 — integrate by trust context

1. `prism-serve` and `prism-proxy`: hosted public-only policy first.
2. `prism-net`: explicit browsing policy; prevent background fetches from inheriting desktop navigation privileges.
3. `prism-tauri`: retain intentional user-approved local browsing, but make local-network access explicit and ensure redirects/background work cannot silently broaden authority.
4. `prism-ingest`: fixed source manifests and bounded blocking reads; no arbitrary URL bypass.
5. Remove or quarantine duplicate direct clients only after their callers have been migrated and tests prove parity.

### E4 — protocol and application boundaries

- Bound Unix-socket line/frame reads and reject incomplete/oversized messages.
- Fail closed on missing or malformed Symthaea safety fields.
- Bound inbound proxy request sizes and concurrent work.
- Normalize and encode provider query parameters structurally; do not concatenate untrusted query strings.
- Avoid forwarding upstream Set-Cookie, hop-by-hop, authentication, or unsafe response headers to browser clients.

### E5 — independent assurance

- Unit/property tests for address classification, URL parsing, redirects, limits, and policy state transitions.
- Deterministic fake DNS and local test servers for mixed answers, rebinding, redirect-to-private, chunked/missing/misleading Content-Length, compression bombs, slow responses, and cancellation.
- Mutation/negative controls: deliberately remove a check and prove the test suite fails.
- Run targeted crate tests, fmt, clippy, and workspace integration checks locally. CI queued/skipped is pending, never a pass.
- Publish exact-head test receipts and a reproducible benchmark corpus.

## Benchmark programme

There is no universally accepted single SSRF-prevention benchmark that fairly ranks application egress controls. Prism should publish a transparent adversarial suite rather than claim superiority from a hand-picked score.

### Security gates (release blockers)

| Measure | Initial target | Stretch target |
|---|---:|---:|
| Prohibited destination corpus blocked (IPv4, IPv6, mapped, transition, special-use) | 100% | 100% |
| Mixed public + prohibited DNS answer sets denied | 100% | 100% |
| DNS rebinding / TOCTOU cases reaching prohibited peer | 0 | 0 |
| Redirect-to-private / metadata / loopback blocked | 100% | 100% |
| Response body over limit consumed beyond hard cap | 0 bytes (excluding at most one bounded in-flight chunk if implementation contract documents it) | 0 bytes |
| Unknown policy state resulting in allow | 0 | 0 |
| Direct-client inventory coverage for in-scope Prism fetch paths | 100% | 100% |
| Negative controls detected | 100% | 100% |

### Performance and reliability (measure, do not assume)

Compare the same corpus and hardware against the current implementation and a conventional reverse-proxy/egress-gateway baseline.

- Added p50/p95/p99 latency per request and per redirect hop.
- DNS lookup count, cache hit ratio, and stale-pin rejection rate.
- Throughput at 1/10/50/100 concurrent fetches.
- Peak resident memory under many simultaneous near-limit and oversized chunked responses.
- CPU per MiB and bytes read before cancellation.
- Legitimate public destination false-positive rate; target 0% on a versioned curated corpus.
- Desktop local-navigation compatibility, tested separately from hosted proxy policy.

Do not optimize away safety. A performance improvement is valid only if all security gates remain green.

## Competitive positioning

The defensible differentiator is not claiming a stronger deny-list than OWASP or a faster HTTP client. It is an auditable, typed, context-aware egress capability with:
- connection-level destination binding rather than URL-only validation;
- redirect-by-redirect authorization;
- streaming resource budgets;
- separation of user navigation from ambient server authority;
- machine-readable receipts and reproducible negative controls;
- a public adversarial benchmark corpus.

Compare against OWASP guidance, reverse proxies/egress gateways, and cloud network controls as complementary layers. Do not claim Prism replaces network segmentation, a firewall, or a dedicated egress gateway.

## Research questions worth pursuing

1. Can a capability token bind origin, address set, port, method, byte budget, deadline, and policy version into a one-use authorization?
2. Can the transport return a cryptographically chained receipt for every redirect hop and byte budget decision without retaining sensitive payloads?
3. Can the direct-client inventory become a compile-time or CI-enforced architectural rule, rather than a periodic grep?
4. Can a model-based state machine generate redirect/DNS interleavings and verify that no transition reaches an unauthorized peer?
5. Can the same policy model be consumed by Rust clients, desktop navigation, and deployment firewall generation while preserving distinct trust contexts?

## Source references

- OWASP SSRF Prevention Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Server_Side_Request_Forgery_Prevention_Cheat_Sheet.html
- reqwest ClientBuilder documentation (including `resolve_to_addrs`): https://docs.rs/reqwest/latest/reqwest/struct.ClientBuilder.html
- reqwest redirect behavior: https://docs.rs/reqwest/latest/reqwest/

These references inform the design; they do not constitute verification of Prism's implementation.


## Current external references

- OWASP SSRF Prevention Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Server_Side_Request_Forgery_Prevention_Cheat_Sheet.html — explicitly warns about unsafe redirects and DNS rebinding; recommends validating resolved addresses and ensuring the client connects to the validated address.
- IANA IPv4 Special-Purpose Address Registry: https://www.iana.org/assignments/iana-ipv4-special-registry — authoritative address-block attributes include destination validity and global reachability.
- IANA IPv6 Special-Purpose Address Registry: https://www.iana.org/assignments/iana-ipv6-special-registry — equivalent IPv6 registry.
- Reqwest ClientBuilder: https://docs.rs/reqwest/latest/reqwest/struct.ClientBuilder.html — documents `resolve_to_addrs`, redirect policy, and custom DNS resolver support.
- Reqwest overview: https://docs.rs/reqwest/latest/reqwest/ — documents automatic redirect defaults and system proxy environment behavior.

Registry data must be versioned or generated reproducibly. Do not fetch IANA data at runtime to make security decisions; updates should be reviewed, pinned, and covered by regression tests.
