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

## Current feature-branch implementation (PR #6299)

Status: the following are source-level controls on `security/prism-evidence-enforced-egress-v1`; they are not yet validated by a passing build or runtime security run.

- **Typed native egress:** `prism-egress` is a native-only crate with one-use, non-cloneable, short-lived capabilities bound to an exact URL, authority, method, headers, body, DNS snapshot, timeout, response budgets, and policy version.
- **DNS admission and destination policy:** the default resolver collects bounded unique socket answers; the complete set is rejected if empty, oversized, or contains any address denied by the IPv4/IPv6 classifier. DNS admission and blocking resolver work have separate bounded capacity.
- **Pinned connection:** a fresh Reqwest client per capability uses `resolve_to_addrs` for the authorized set, retains the URL hostname for HTTP/TLS identity, and disables ambient proxy discovery. After the response arrives, Prism requires `remote_addr()` to be present and a member of the authorized socket set. That peer check is post-connect evidence; the address override is the connection-time restriction.
- **Manual redirect state machine:** only `PublicWeb` follows redirects, with a five-hop ceiling, loop detection, HTTPS downgrade rejection, and a fresh DNS authorization for every next hop. Provider and fixed-source authorities do not follow redirects. Redirect hops are included in the receipt.
- **No hidden transport retries:** the pinned Reqwest builder explicitly uses `retry(reqwest::retry::never())`. Future retries must be an application-level transition that obtains a fresh capability and records the attempt; they must not be introduced through an implicit client default.
- **Resource budgets:** request body/header and URL ceilings, bounded DNS, execution concurrency, connection/request deadlines, response header count/bytes after parsing, and incremental response-body caps are enforced in source. Defaults are 10 MiB success / 64 KiB error; fixed ingestion may raise its success budget to 16 MiB. Reqwest may deliver one in-flight decoded chunk before Prism can reject it; benchmark received-chunk overshoot separately from bytes appended/returned.
- **Authority partitioning:** public web, Brave, Perplexity, Wikipedia, and Wikidata have distinct exact origin/path/method/header scopes and separate concurrency pools. Provider credentials cannot be forwarded to arbitrary destinations.
- **Ingestion provenance:** the optional native Wikipedia/Wikidata binaries use fixed authorities and a versioned source manifest, and emit query/source and output digests with redacted egress receipts.
- **Desktop navigation remains separate:** `prism-net` and `prism-tauri` retain the user-navigation trust context, with explicit HTTP(S)/URL checks and bounded streamed bodies. They still use their own Reqwest clients and browser-like redirects; DNS pinning and per-hop redirect reauthorization for this desktop context remain outstanding. Do not describe the complete Prism product as having one universal egress boundary yet.
- **Architectural guard:** CI checks the direct Reqwest-client inventory and required egress source invariants. The current grep-based inventory is a useful ratchet, not a complete compiler-level proof against all possible networking APIs.

Focused source tests cover address policy, mixed DNS answers, resolver snapshots, connection override/Host preservation, peer validation, redirects, response limits, provider scopes, and cancellation of blocking DNS work. They have not been run successfully on the current PR head. The latest workflow is `skipped`, not passed; Cargo test/fmt/clippy and end-to-end exploit evidence remain pending.

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
4. `prism-ingest`: fixed Wikipedia/Wikidata sources use typed `prism-egress` authorities, bounded async fetches, pinned DNS snapshots, peer validation, and per-source success/error budgets; no arbitrary URL bypass.
5. Retain only the two explicit desktop-navigation Reqwest clients; any new server-side HTTP caller must use a typed `prism-egress` authority.

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

## Direct-fetch surface audit

A source review of the default-branch Prism clients found that the proxy/serve hardening is not yet end-to-end:

- `prism-net/src/lib.rs`: uses a shared `reqwest::Client` with automatic redirects limited to 10. It checks `Content-Length` when present, then calls `Response::text()`; this is not a hard streaming byte cap when the length is absent or misleading. It does not currently call the shared `validate_proxy_url` policy.
- `prism-tauri/src/main.rs`: the desktop `fetch_url` command uses a separate reqwest client with up to 10 automatic redirects and reads the full body with `Response::text()`. The URL is parsed for Reflex analysis after the network fetch. This path must be treated as user-directed desktop navigation, not silently forced into the hosted public-only policy; local/LAN navigation needs explicit user authority and a separate redirect policy.
- `prism-ingest` exposes optional native fetch binaries behind the `fetch` feature. Both Wikipedia and Wikidata now use `prism-egress` with fixed source authorities, complete DNS-set validation, pinned connections, observed-peer checks, no redirects, response budgets, and redacted receipts. Native Tokio/egress dependencies remain optional and are not pulled into non-fetch/WASM consumers.

The Rust server-side HTTP inventory is now covered by `prism-egress`. The only direct Reqwest clients remaining are `prism-net` and `prism-tauri`, which are deliberately restricted to interactive desktop navigation. The browser service-worker fetch remains browser-origin asset/cache traffic, not server-side URL-fetch authority. CI asserts exactly two reviewed desktop-navigation Reqwest builders. Do not apply one global private-address deny-list to those user-directed LAN browsing surfaces.
## Benchmark programme

There is no universally accepted single SSRF-prevention benchmark that fairly ranks application egress controls. Prism should publish a transparent adversarial suite rather than claim superiority from a hand-picked score.

### Security gates (release blockers)

| Measure | Initial target | Stretch target |
|---|---:|---:|
| Prohibited destination corpus blocked (IPv4, IPv6, mapped, transition, special-use) | 100% | 100% |
| Mixed public + prohibited DNS answer sets denied | 100% | 100% |
| DNS rebinding / TOCTOU cases reaching prohibited peer | 0 | 0 |
| Redirect-to-private / metadata / loopback blocked | 100% | 100% |
| Bytes appended to/returned from the response buffer beyond the configured cap | 0 | 0 |
| Transport-delivered in-flight decoded chunk overshoot | Measured and reported separately; bounded by transport chunk behavior | 0 bytes |
| Unknown policy state resulting in allow | 0 | 0 |
| Direct-client inventory coverage for in-scope Prism fetch paths | 100% | 100% |
| Negative controls detected | 100% | 100% |
| Implicit HTTP-client retries outside Prism's authorization state machine | 0 | 0 |
| Redirect/retry attempts without a fresh authorization receipt | 0 | 0 |

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

## Branch implementation status (2026-11-17)

This section records source changes on `security/prism-evidence-enforced-egress-v1`. It is an implementation inventory, not a claim that the branch compiles or passes its tests.

### Native public egress primitive

A new native-only `prism-egress` crate now provides:

- A crate-private injectable `EgressResolver` seam and system resolver implementation. Deterministic unit tests can supply complete answer sets without exposing a public production hook that lets downstream callers replace the trusted system resolver.
- All-answer-set classification: the request is denied if any returned IPv4 or IPv6 address is outside the conservative public-unicast policy. Mixed public/private answers fail closed.
- A per-request Reqwest client using `resolve_to_addrs` for the validated address set, retaining the URL hostname for HTTP authority and TLS verification. The exact Reqwest 0.12 API is documented to override hostname resolution for the specified address set. After connecting, Prism reads `Response::remote_addr()`, requires it to match an exact socket address in the authorized set, and fails closed if peer evidence is unavailable or unexpected. The receipt records this observed peer. See the Reqwest 0.12 Response and ClientBuilder documentation.
- DNS work is independently budgeted: 16 concurrent hostname lookups, two-second queue wait, five-second caller deadline, and a maximum of 32 unique returned addresses. Numeric IP literals do not consume resolver slots. The OS resolver deduplicates while collecting and stops only after the 33rd unique address, which is enough to reject an oversized answer set without truncating a duplicate-heavy set before a later private address. The owned resolver permit moves into Tokio's blocking DNS closure and remains held after caller timeout until non-cancellable OS lookup work exits.
- Automatic redirects and ambient proxy discovery remain disabled at the Reqwest layer. `PublicWeb` uses a manual, five-hop maximum redirect state machine for 301/302/303/307/308 only: resolve relative locations, reject loops and HTTPS-to-HTTP downgrades, issue fresh authorization for every destination, validate the complete DNS set, pin the new client, and verify the actual connected peer. Redirect responses are dropped without reading their bodies. BraveSearch and Perplexity authorities never follow redirects.
- A non-cloneable, in-memory, one-use `EgressCapability` that binds the exact URL, typed authority, method, outbound headers, body, resolved address set, expiry, response budget, timeouts, and policy version. Execution consumes the capability. This is a Rust ownership boundary, not a cryptographically signed or remotely transferable token.
- Typed authority scopes are stored in the one-use capability and execution receipt: `PublicWeb` is an unauthenticated, bodyless GET; `BraveSearch` is fixed to the Brave API host/path with token + Accept headers; `Perplexity` is fixed to its completion host/path with POST, Bearer authorization, and JSON content type; `WikipediaSummary` is fixed to one Wikipedia summary title segment; `WikidataSparql` is fixed to its endpoint and the exact `query` + `format=json` parameter shape. Provider and fixed-source scopes reject extra headers and never follow redirects.
- HTTP admission partitions are 52 PublicWeb, 4 BraveSearch, 4 Perplexity, 2 WikipediaSummary, and 2 WikidataSparql slots (64 total). The separate authorities reserve capacity for provider calls and fixed-source ingestion during general-web fetch bursts.
- Constructor-enforced absolute ceilings: five-second authorization lifetime; 8 KiB URL; 16 KiB total request-header budget (including a 512-byte allowance for transport-generated fields); 2 MiB request body; 16 MiB successful response body; 64 KiB non-success diagnostic response body; 64 KiB / 128-entry accepted response headers; five-second connect timeout; 30-second default / 90-second absolute total operation timeout across all redirect hops. Callers may choose lower limits but cannot widen these hard maximums. Default success budget is 10 MiB; Wikipedia selects 2 MiB and Wikidata 16 MiB.
- HTTP execution has a global limit of 64 in-flight requests, partitioned into 52 PublicWeb, 4 BraveSearch, 4 Perplexity, 2 WikipediaSummary, and 2 WikidataSparql slots. Each partition has its own two-second admission wait, so general public traffic cannot consume fixed-provider or fixed-source capacity.
- Response parsing is constrained before application checks where Reqwest 0.12 exposes a supported control: HTTP/2 received header lists are capped at 64 KiB. The application then enforces a protocol-independent 64 KiB / 128-entry check. In Reqwest 0.12, HTTP/1 header-count control is not exposed by `ClientBuilder`; its byte/count checks occur after parsing, so HTTP/1 parser-side allocation remains a known limitation. Reqwest 0.13.5 adds `http1_max_headers`, but that upgrade is deferred until the wider workspace can be compatibility-tested.
- A known oversized Content-Length is rejected before reading the response body. Unknown/chunked/decompressed bodies are checked incrementally. Success and non-success responses have separate caps, and receipts record the selected response-body limit. Reqwest may have already delivered one in-flight chunk before the application refuses to append it; that bounded overshoot is measured separately in PEAB.
- Privacy-minimized receipts contain final origin (not path/query), authority, method, final authorized address set, observed final `remote_peer`, status, byte counts, duration, policy version, and an ordered redirect-hop record (from/to origin, 3xx status, previous-hop peer, and newly authorized target address set). Request/response bodies and query strings are not included.
- `prism-proxy` and `prism-serve` emit successful receipts and per-hop redirect evidence into server audit logs. The log projection contains only the receipt fields above and deliberately excludes request/response headers, URL path/query, provider keys, and payload bytes. Failed operations log only the route context and safe status class, not raw transport errors or request URLs.

The conservative address table is aligned to the IANA IPv4 and IPv6 Special-Purpose registries, whose current pages show a last-updated date of 2025-10-09. Entire parent aggregates such as IPv4 192.0.0.0/24 and IPv6 2001::/23 are denied even though IANA lists more-specific globally reachable assignments inside them. That choice favours a smaller attack surface over maximum compatibility and should be tracked as an explicit false-positive trade-off, not described as a perfect mirror of IANA's per-prefix reachability field.



The redirect state machine deliberately prefers a failed fetch over an unreviewed destination: non-HTTP(S) targets, credentials in redirect URLs, non-default ports, unsafe DNS answers, DNS timeouts, loops, downgrade attempts, and more than five hops all terminate the operation before connecting to the next peer. This remains a public-web context only; no provider secret is carried across a redirect because provider authorities do not follow.



The success/error budget split is now used by fixed-source ingestion: Wikipedia caps successful payloads at 2 MiB; Wikidata caps successful payloads at 16 MiB and non-success diagnostic payloads at 64 KiB. The 90-second absolute total-operation ceiling accommodates the previous Wikidata timeout while preserving a 30-second default for other authorities. Both fixed-source callers record the selected body limit as part of their redacted receipt output.

### Call-site migration and remaining contexts

- `prism-proxy` and `prism-serve`: generic URL proxy and DuckDuckGo use unauthenticated PublicWeb scope and can follow up to five manually reauthorized public redirects; Brave and Perplexity use fixed typed provider scopes and never follow redirects. Provider query parameters are encoded with URL query-pair APIs. Inbound body limits remain 2 MiB.
- `prism-net`: now explicitly documents its user-directed navigation role, limits URLs to 8 KiB and response bodies to the configured 10 MiB default while streaming, rejects non-HTTP(S) schemes and URL credentials, and preserves declared legacy HTTP charsets. It deliberately retains local/LAN destinations and a ten-redirect browser-like limit; it is not the hosted public-egress policy.
- `prism-tauri`: applies the same 8 KiB URL and 10 MiB streamed response limits and rejects non-HTTP(S) schemes and URL credentials. Explicit desktop LAN navigation remains possible. Redirects remain browser-like and bounded to ten; redirects to local resources are therefore part of the desktop navigation threat model and require separate UX/authority work.
- `prism-ingest`: Wikipedia summary fetches use a fixed `WikipediaSummary` authority bound to `en.wikipedia.org/api/rest_v1/page/summary/{single-title-segment}`, with a 2 MiB success-body limit. Wikidata uses a fixed `WikidataSparql` authority bound to `query.wikidata.org/sparql` and exactly two query parameters (`query` and `format=json`), with a 16 MiB success-body limit and 64 KiB diagnostic error-body limit. Both are GET-only, headerless, bodyless, never follow redirects, use the shared DNS/peer-pinned transport, and log redacted execution receipts. `prism-ingest` activates `prism-egress`, Tokio, and URL parsing only behind its optional `fetch` feature.
- `prism-shell` uses `prism-net` and therefore stays in the explicit desktop-navigation context. `prism-knowledge-bridge` constructs a Holochain client, not an HTTP Reqwest client. `prism-ui/static/sw.js` uses browser `fetch(event.request)` for browser-controlled asset/cache traffic; it is not a server-side fetch capability and must be benchmarked/classified separately from hosted egress.
- A repository source scan found no additional direct Hyper, ureq, isahc, surf, or raw Tokio TCP egress callers in Prism. Browser service-worker `fetch(event.request)` calls are classified as browser-origin asset/cache traffic, not server-side egress authority. Holochain client construction is classified separately from HTTP Reqwest. This is a source inventory, not proof against dynamically linked/generated code or new networking dependencies.

The Symthaea bridge also now fails closed when the daemon omits or malforms its `safe` field, and its Unix-socket response frame is bounded to 64 KiB and must be newline-terminated.

### CI architecture ratchet

The Prism CI job runs Cargo package commands from the actual outer `mycelix-workspace` Cargo workspace root (the `mycelix-prism/Cargo.toml` file is a placeholder, not a workspace root). The existing `mycelix-prism/*` membership glob includes the new crate. Formatting and package-test lists include `prism-egress`; fixed-source ingestion binaries are compile-checked and their manifest/query-provenance tests run with `--features fetch`; a source-level ratchet now permits exactly two direct Reqwest client constructors, both in the desktop-navigation clients (`prism-net` and `prism-tauri`).

The CI source ratchet also asserts the hosted egress invariants: no independent Reqwest client or automatic redirect policy in the proxy servers, one pinned egress client builder, `resolve_to_addrs`, observed peer validation, the five-hop PublicWeb redirect state machine, redirect-hop receipt creation, and provider authority routing. The hosted handlers must call the receipt logger for proxy, DuckDuckGo, Brave, and Perplexity results.

This ratchet is an initial guardrail, not a complete static proof: aliases, wrappers, other HTTP libraries, browser-side fetches, and generated code need separate inventory coverage.

### Ingestion source manifest and run provenance

The fixed-source ingestion paths now have a versioned contract at `prism-ingest/sources/manifest.json`:

- Manifest ID `luminous.prism.ingest.sources`, version `2026-11-17.1`, bound to egress policy `prism-public-egress-v1`.
- Wikipedia entry specifies the exact title-segment endpoint, GET-only/no-redirect authority, 2 MiB success and 64 KiB error budgets, and ten-second timeout.
- Wikidata entry specifies the exact SPARQL endpoint and `query` + `format=json` shape, 16 MiB success and 64 KiB error budgets, 90-second timeout, three-attempt 429 retry policy, and all 79 query IDs.
- The Wikidata generator hashes the exact UTF-8 SPARQL query bytes with SHA-256. Each generated claim carries its query ID and digest in its source metadata; the run sidecar records each query's digest, success/failure, result-row count, and generated-claim count.
- Both ingestion jobs emit provenance sidecars that bind the exact source-manifest SHA-256 to the generated data/source artifact SHA-256, source ID, authority, policy version, and resource budgets. The Wikipedia sidecar also records the successful article-title set.

The binary-level tests compare the live query registry and source limits against the manifest. CI runs those tests with `cargo test -p prism-ingest --features fetch --bins`. This is the start of a reproducible source-acquisition contract; the next step is to add external response digests and independently attested retrieval timestamps without storing sensitive data.

### Versioned benchmark artifacts added

The first PEAB data artifacts now exist in the crate and are consumed by integration tests:

- `prism-egress/tests/data/address_policy_cases.json`: 41 IPv4/IPv6 cases, with expected allow/deny outcome, case ID, policy snapshot, registry provenance, and an explicit note about conservative aggregate denial.
- `prism-egress/tests/data/dns_answer_sets.json`: 8 complete-answer-set cases covering public IPv4, public IPv6, dual-stack public, mixed loopback, mixed ULA, metadata, IPv4-mapped IPv6, and empty answer sets.
- `prism-egress/tests/address_policy_corpus.rs`: integration tests parse both JSON corpora and compare expected outcomes against the public classifier and all-or-nothing resolver-set validator.
- `prism-egress` unit tests also exercise the fixed Wikipedia/Wikidata origin/path/query allowlists and separate success/error response budgets.

These are regression corpora, not yet a full performance benchmark or independent penetration-test result. The next PEAB increment should add a hermetic local DNS server capable of scripted A/AAAA/CNAME responses, capture the actual peer socket, run redirect/retry/slow-stream/compression cases, and emit JSON receipts with exact commit and policy digests.

### Adversarial tests added (not yet executed)

The `prism-egress` source and integration corpus include tests for:
- representative public IPv4/IPv6 addresses and special-purpose, private, transition, multicast, documentation, and benchmarking ranges;
- mixed public/private DNS answer rejection and empty answer sets;
- deterministic injected-resolver snapshots and exactly one resolver call during authorization;
- actual Reqwest destination pinning to a local test listener while preserving the original Host authority, followed by observed-peer membership validation;
- missing/unexpected peer evidence rejection and successful receipts recording the observed peer;
- metadata-host redirect rejection through the high-level PublicWeb execution path, relative redirect target parsing, HTTPS downgrade rejection, loop detection, and provider no-follow policy;
- URL scheme, port, credentials, local hostname, body, header, capability expiry, HTTPS requirements, provider-origin/header allowlists, and duplicate credential headers.

No passing test result is claimed. The PR remains a draft; the workflow's `pull_request` gate skips draft PRs, and the latest branch head has not been built or tested in a runner or local checkout.

## Research and benchmark plan: evidence before SOTA claims

OWASP's current SSRF guidance says to validate resolved A and AAAA destinations, ensure the HTTP client connects only to those validated addresses, preserve the original hostname for Host/SNI/certificate checks, and disable unsafe automatic redirects. Those are design requirements, not proof that this implementation meets them. Cloud egress controls such as AWS security-group egress restrictions and DNS Firewall add network-layer controls; Prism should complement these, not claim to replace them.

The proposed **Prism Egress Assurance Benchmark (PEAB)** should be a public, reproducible corpus with a versioned policy snapshot, deterministic DNS server, local HTTP/TLS peers, adversarial redirect server, raw request/peer capture, mutation controls, and machine-readable run receipts.

#### Security release gates

| Measure | Release target |
|---|---:|
| Denied destination corpus blocked (IPv4, IPv6, mapped, transition, special-use, metadata) | 100% |
| Mixed public + prohibited A/AAAA/CNAME answer sets denied | 100% |
| Requests reaching a prohibited socket peer | 0 |
| DNS rebinding attempts that reach a peer outside the authorized snapshot | 0 |
| Redirect-to-private, redirect-to-metadata, scheme downgrade, and redirect loop bypasses | 0 |
| Retry/fallback connections outside the current capability | 0 |
| Bytes appended/returned beyond configured response cap | 0 |
| In-scope server fetches outside the approved transport | 0 |
| Negative controls that fail to detect a deliberately removed guard | 0 |

#### Compatibility, performance, and reliability measurements

Run matched workloads against (1) baseline Reqwest with the current Prism configuration, (2) Prism's public-egress client, and (3) a conventional Envoy/egress-gateway or cloud-firewall baseline where configuration permits. Publish hardware, resolver, TLS, connection reuse, cache, response-size, and concurrency conditions.

Measure p50/p95/p99 added latency; DNS lookups and answer-set sizes; TLS handshakes; throughput at 1/10/50/64/100 concurrent requests; queue wait and rejection rate; CPU per MiB; peak RSS; response bytes received in the in-flight chunk at cancellation versus bytes appended/returned; and public-destination false-positive rate. The current chunk loop refuses to append a chunk that would exceed the cap, but Reqwest may already have delivered that in-flight chunk to the application. The benchmark must quantify that bounded overshoot separately. Set no performance superiority claim until the comparisons are measured.

Initial performance goals are hypotheses to validate, not current results:
- p95 overhead below 10% against matched Reqwest for warm, connection-reusable same-origin traffic once safe pooling exists;
- p99 queue admission below 100 ms at 50 concurrent requests under the target deployment profile;
- peak memory bounded by active-request count times configured body budget plus measured fixed overhead;
- zero false positives on a versioned 1,000-host public compatibility corpus, with every exception reviewed and recorded.



Industry comparison: AWS's published egress patterns recommend composing DNS-layer domain filtering, network-layer inspection, and least-privilege security-group egress rather than relying on one mechanism. Prism's typed application authority and per-request destination pinning operate at a different layer. The target deployment should pair Prism with network egress controls (for example, deny-all-except security-group rules and DNS firewalling); neither layer substitutes for the other. AWS documentation also calls out that security groups do not filter Route 53 Resolver DNS traffic, which is why DNS-layer policy must be designed separately.



Industry comparison: Envoy's dynamic forward proxy offers a mature general-purpose HTTP forward-proxy mechanism, but Envoy documentation warns that deployments with untrusted clients or destinations can become confused-deputy vulnerabilities. Istio's egress-gateway model provides a centralized policy and telemetry point, but Istio explicitly notes that configuring a gateway alone does not securely force all traffic through it; deployment-level routing and node isolation are still required. Kubernetes NetworkPolicy provides additive L3/L4 egress controls, but a default-deny policy also blocks DNS unless DNS egress is separately allowed. These systems are valuable complementary layers. Prism's narrower differentiation is request-level typed authority plus DNS-answer validation and socket-destination pinning inside the application transport.

#### Paradigm shifts worth pursuing

1. **Authority-carrying requests:** make network access an explicit capability scoped to trust context, origin, method, headers, body digest, destination set, expiry, and resource budgets—not a URL string plus ambient process privileges.
2. **Proof-carrying execution:** extend receipts with intent ID, policy digest, DNS answer-set digest and TTL, actual peer, TLS identity, redirect-hop decisions, and response-budget outcome. Sign receipts with a workload identity; never store request/response bodies or raw secret-bearing URLs.
3. **Safe pooling without authority leakage:** pool only by a key that includes origin, trust context, authorized address set, policy version, and expiry. A connection established under one address set must never serve a capability authorized under another.
4. **Model-based adversarial verification:** generate DNS/redirect/retry/timeout/cancellation interleavings and check the invariant: every successful socket connection must target an address authorized for the exact unexpired capability.
5. **Policy-as-code across layers:** generate application policy, deployment firewall rules, allowlist manifests, and benchmark cases from one reviewed policy representation while preserving the distinction between public hosted fetch, pinned providers, fixed-source ingestion, and user-approved desktop navigation.
6. **Live policy drift detection:** alert when DNS, certificates, route egress, or the IANA range snapshot changes in a way that invalidates assumptions; policy updates should be reviewed, versioned, and regression-tested, never downloaded at runtime to silently change authorization.

The current implementation includes a manual PublicWeb redirect state machine, fixed-source ingestion DNS/peer pinning, and observed socket-peer validation, but remains deliberately narrower than the research targets. It does not yet implement signed receipts, validated connection pooling, a cryptographic transferable capability token, desktop-navigation redirect reauthorization, cryptographically verified TLS certificate/workload identity in receipts, or an independent hermetic DNS/redirect conformance harness.

## Competitive positioning

The defensible differentiator is not claiming a stronger deny-list than OWASP or a faster HTTP client. It is an auditable, typed, context-aware egress capability with:
- connection-level destination binding rather than URL-only validation;
- redirect-by-redirect authorization;
- streaming resource budgets;
- separation of user navigation from ambient server authority;
- machine-readable receipts and reproducible negative controls;
- typed fixed-source authorities for auditable ingestion in addition to user-directed navigation;
- a public adversarial benchmark corpus.

Compare against OWASP guidance, reverse proxies/egress gateways, and cloud network controls as complementary layers. Do not claim Prism replaces network segmentation, a firewall, or a dedicated egress gateway.

## Research questions worth pursuing

1. Can a capability token bind origin, address set, port, method, byte budget, deadline, and policy version into a one-use authorization?
2. Can the transport return a cryptographically chained receipt for every redirect hop and byte budget decision without retaining sensitive payloads?
3. Can the direct-client inventory become a compile-time or CI-enforced architectural rule, rather than a periodic grep?
4. Can a model-based state machine generate redirect/DNS interleavings and verify that no transition reaches an unauthorized peer?
5. Can the same policy model be consumed by Rust clients, desktop navigation, browser asset routing, and deployment firewall generation while preserving distinct trust contexts?
6. Can fixed-source ingestion be granted a verifiable data-source manifest with versioned URI templates, query schemas, and update windows—so data acquisition is auditable and reproducible, not just network-safe?

## Interim implementation ledger (branch: security/prism-evidence-enforced-egress-v1)

The current draft branch includes the following bounded, fail-closed controls in addition to the design programme:

- `prism-proxy` and `prism-serve` disable automatic redirects until a per-hop authorization state machine exists.
- Both clients disable ambient system proxy discovery; any future proxy must be an explicitly configured and reviewed trust boundary.
- All four upstream response paths in each binary consume response chunks incrementally and stop before exceeding the 10 MiB response budget.
- Both servers explicitly cap inbound request bodies at 2 MiB.
- The Symthaea Unix-socket response frame is capped at 64 KiB and must be newline-terminated.
- Missing, null, or malformed Symthaea `safe` metadata maps to `false`; focused unit tests cover missing/malformed and explicit boolean values.

These changes are source edits only until the exact branch head is compiled and tested. The current environment has not produced passing Cargo, formatting, clippy, or adversarial runtime evidence. Do not interpret the presence of unit tests as evidence that they have passed.
## Source references

- OWASP SSRF Prevention Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Server_Side_Request_Forgery_Prevention_Cheat_Sheet.html
- reqwest ClientBuilder documentation (including `resolve_to_addrs`): https://docs.rs/reqwest/latest/reqwest/struct.ClientBuilder.html
- reqwest Response documentation (including `remote_addr`): https://docs.rs/reqwest/0.12.24/reqwest/struct.Response.html
- Tokio `lookup_host` implementation: https://docs.rs/tokio/latest/src/tokio/net/addr.rs.html — the system resolver uses the blocking pool; started blocking tasks cannot be aborted, so the resolver permit is held by the blocking closure.
- reqwest redirect behavior: https://docs.rs/reqwest/latest/reqwest/

These references inform the design; they do not constitute verification of Prism's implementation.


## Current external references

- OWASP SSRF Prevention Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Server_Side_Request_Forgery_Prevention_Cheat_Sheet.html — explicitly warns about unsafe redirects and DNS rebinding; recommends validating resolved addresses and ensuring the client connects to the validated address.
- IANA IPv4 Special-Purpose Address Registry: https://www.iana.org/assignments/iana-ipv4-special-registry — authoritative address-block attributes include destination validity and global reachability.
- IANA IPv6 Special-Purpose Address Registry: https://www.iana.org/assignments/iana-ipv6-special-registry — equivalent IPv6 registry.
- Reqwest ClientBuilder: https://docs.rs/reqwest/latest/reqwest/struct.ClientBuilder.html — documents `resolve_to_addrs`, redirect policy, and custom DNS resolver support.
- Reqwest overview: https://docs.rs/reqwest/latest/reqwest/ — documents automatic redirect defaults and system proxy environment behavior.

Registry data must be versioned or generated reproducibly. Do not fetch IANA data at runtime to make security decisions; updates should be reviewed, pinned, and covered by regression tests.

Additional implementation and industry references:
- Tokio `lookup_host` and source implementation: https://docs.rs/tokio/latest/tokio/net/fn.lookup_host.html and https://docs.rs/tokio/latest/src/tokio/net/addr.rs.html — hostname resolution in the system resolver path delegates to Tokio's blocking pool; started blocking work is not abortable, hence the permit must travel into the blocking closure.
- Tokio `spawn_blocking` cancellation behaviour: https://docs.rs/tokio/latest/tokio/task/fn.spawn_blocking.html.
- Envoy dynamic forward proxy: https://www.envoyproxy.io/docs/envoy/latest/configuration/http/http_filters/dynamic_forward_proxy_filter.html — warns about confused-deputy risks when clients or destinations are untrusted and recommends network/firewall and workload isolation.
- Istio egress gateway security considerations: https://istio.io/latest/docs/tasks/traffic-management/egress/egress-gateway/ and https://istio.io/latest/docs/ops/best-practices/security/ — a configured gateway alone does not prevent sidecar bypass; network controls must enforce the path.
- Kubernetes NetworkPolicy: https://kubernetes.io/docs/concepts/services-networking/network-policies/ — default-deny egress also blocks DNS until DNS is separately allowed.
- AWS outbound network security guidance: https://docs.aws.amazon.com/prescriptive-guidance/latest/secure-outbound-network-traffic/ — combines security groups, DNS hostname filtering, and network firewalls.

