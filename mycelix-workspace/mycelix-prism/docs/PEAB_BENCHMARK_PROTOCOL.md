# Prism Egress Assurance Benchmark (PEAB) v0.1

Status: benchmark protocol proposal; no benchmark measurements have been run.
Scope: compare egress security semantics, operational costs and failure behavior—not marketing claims or isolated throughput.

## 1. Why this benchmark exists

A URL filter is not an egress security boundary. The measurable unit is a complete attempted network action:

`intent → authority decision → DNS snapshot → address policy → socket connect → TLS identity → HTTP exchange → redirect/retry decision → response retention → downstream consumption → evidence`

PEAB measures whether every transition preserves the same authority and resource constraints. It also measures the cost paid for those properties.

The primary research hypothesis is that a short-lived, one-use, request-scoped authority capability plus a pinned DNS snapshot can make the application decision and the actual socket destination refer to the same finite set, while a separate kernel/network observer checks what really connected. This is a hypothesis to test—not a claim that Prism is already superior to Envoy or network policy.

## 2. Comparison rules

### 2.1 Two separate comparison tracks

**Track A — semantic-equivalence application transports.** Compare:
1. Prism `prism-egress`.
2. A hardened Reqwest reference implementation using the same URL/authority policy, A+AAAA resolution, all-answer classification, pinned addresses, preserved original hostname for Host/SNI/certificate verification, disabled ambient proxies, disabled automatic redirects and retries, and equivalent body/header/deadline budgets.
3. An unguarded Reqwest transport only as a performance floor. It is not a security comparator and must never be reported as a competing secure design.

**Track B — deployment defense in depth.** Compare:
1. Envoy Dynamic Forward Proxy with `resolved_address_filter`, bounded DNS cache/circuit breakers and recorded TTL/host-cache settings.
2. Envoy plus an enforced L3/L4 egress policy (for example, Cilium NetworkPolicy or an equivalent host/cloud firewall).
3. A direct workload constrained by an equivalent network namespace/firewall policy, where the environment supports it.

Track B is not a substitute for Track A. A network firewall often knows the destination IP and port but not the originating URL authority, redirect chain, request purpose or application-level body budget. Conversely, an application client cannot by itself prevent unrelated processes or native libraries in the same workload from opening sockets. Report composition, not a single blended score.

### 2.2 Pin and publish every baseline

For every run, record:
- source commit and dirty-tree status;
- exact compiler, target, dependency lockfile and build profile;
- Reqwest, Hyper, Envoy, Cilium and kernel versions actually used;
- Envoy bootstrap/configuration and resolved-address matcher;
- Cilium policy, DNS proxy mode, trusted resolver, and policy revision;
- CPU model/topology, RAM, kernel, OS, container/runtime, cgroup limits and power profile;
- resolver address, DNS fixture revision, cache state, RTT, TTLs, and network topology;
- TLS version/cipher, certificate chain, ALPN, HTTP protocol, connection reuse and proxy environment;
- raw samples, run seeds, harness version, and all exclusions.

Do not compare a warm pooled Envoy connection with a cold one-shot Prism request and call the result a transport comparison. Publish cold and warm tracks separately. If a system cannot implement a scenario, record `not_supported`; do not silently omit it or score it as a pass.

## 3. Threat and correctness corpus

Every scenario must have a stable ID, deterministic fixture, expected decision, expected socket events and a negative control. The negative control deliberately removes or weakens the protection and must make the corresponding test fail; otherwise the test may not be exercising the intended boundary.

| Family | Required cases | Primary oracle |
|---|---|---|
| DNS answer policy | A only, AAAA only, dual stack, CNAME chain, duplicate answers, empty answer, NXDOMAIN, SERVFAIL, timeout, malformed response, UDP truncation/TCP fallback | Resolver transcript and complete accepted/rejected address set |
| Mixed-answer SSRF | Public A + private AAAA; public + loopback; link-local; metadata; ULA; mapped IPv6; special-use and transition ranges | No socket attempt to any denied answer |
| Rebinding | Answer changes after authorization; change between A and AAAA; change at next redirect; change after TTL expiry | Actual connect peer must remain inside the specific capability's authorized set |
| Redirect authority | Public→private, metadata, localhost, alternate port, scheme change, HTTPS downgrade, duplicate/missing Location, loops, too many hops, malformed URL | Every hop independently re-authorized; no credential bleed |
| Retry/fallback | Connect reset, timeout, protocol NACK, retryable status, alternate-address fallback, cancellation at each phase | No implicit second attempt; each permitted attempt has a fresh authority decision and evidence |
| TLS identity | Pinned IP with original host SNI/Host; valid SAN; wrong SAN; untrusted issuer; expired cert | SNI/Host remain the URL authority; certificate validation fails closed |
| Parser/resource limits | Excess HTTP/1 headers, oversized aggregate header bytes, HTTP/2 header list, huge single value, chunked body, compressed expansion, false Content-Length | Rejection point, bytes allocated/read, and limit overshoot |
| Retained memory | Concurrent maximum-size responses, shallow Bytes clones, proxy/serve handoff, stalled reader, cancellation, disconnect | Lease and measured memory remain bounded until final downstream owner releases |
| Ambient path escape | HTTP(S)_PROXY/ALL_PROXY, environment changes, proxy credentials, alternate resolver, IPv4/IPv6, direct socket from same cgroup | No unapproved destination or proxy connection |
| DNS cache semantics | Envoy cached good answer then all-new answers filtered; expiry, DNS failure, stale cache and refresh races | Exact answer/cache used and any stale address retained; classify behavior, do not presume |
| Deployment escape | A second process/native client in the workload attempts a denied connect; DNS bypass attempt; alternate egress route | Network policy blocks the escape even when application policy is bypassed |

The DNS lab should extend the existing loopback UDP/TCP fixture and preserve its A/AAAA query-count assertions. Include deterministic event schedules rather than sleeps wherever possible. For races that require timing, capture monotonic timestamps and repeat with seeded interleavings.

## 4. Workload profiles

### P0 — Security correctness (release-blocking)
- Execute every deterministic positive and negative scenario.
- Record each DNS query, answer, policy decision, socket connect, peer, TLS result, HTTP status, redirect, retry, response-byte event and downstream release.
- Include the same attack suite with the guard disabled as a negative control.
- Run at least 10,000 seeded adversarial sequences per family for the initial qualification, then increase to 1,000,000 sequences for the long-running campaign.
- Required outcome: zero unauthorized connect attempts, zero accepted peers outside the capability address set, zero un-authorized redirect/retry hops, and zero response-body lease overcommit.

Zero observed failures is evidence for the tested corpus, not proof of zero risk. Always publish the denominator and confidence interval.

### P1 — Latency and throughput
Run cold DNS/cold connection, warm DNS/cold connection, cold DNS/warm connection where the baseline supports it, and warm pooled connection as distinct test cases. Include HTTP/1.1 and HTTP/2 separately.

Measure:
- end-to-end p50/p95/p99 latency and confidence intervals;
- requests/s and completed bytes/s;
- DNS latency and DNS queries per logical request;
- capability/authorization time, connect/TLS time, first-byte time and total body time;
- connection count, reuse ratio, file descriptors and open sockets;
- rejected-request latency and admission-queue wait.

Use fixed payload classes (1 KiB, 64 KiB, 1 MiB, 10 MiB) and redirect chains (0, 1, 3, maximum). Do not mix rejected and successful requests into one latency distribution.

### P2 — Resource saturation and slow consumers
Sweep concurrency at 1, 8, 32, 64, 128 and 256 (stop before host instability). Exercise full-size reservations, partially consumed bodies, cloned body handles, slow readers, dropped clients, cancellations and multiple simultaneous ingestion/proxy responses.

Measure process RSS, cgroup memory.current/peak, allocation/retained-byte counters, lease availability/reservations, task count, FD count, CPU, p95 queue wait and recovery time after consumers disconnect. The current 128 MiB logical response-body budget is not an RSS ceiling; transport/parser/TLS/runtime memory must be measured separately.

### P3 — Operational resilience
Inject DNS stalls/failures, TCP fallback, connect resets, upstream slow headers, slow body chunks, downstream disconnects, clock jumps where relevant, resolver restart and policy-version change. Record fail-closed behavior and recovery. Never treat a timeout as a successful denial if the workload could still have opened a forbidden socket.

## 5. Measurement and statistical protocol

- Use an isolated network namespace and deterministic local DNS/HTTP/TLS fixtures for security tests. Public internet endpoints are unsuitable as the primary oracle.
- Run at least 5 independent repetitions per configuration after one documented warm-up. For latency/throughput, target at least 30,000 completed requests per repetition and report achieved sample counts.
- Randomize baseline order within each repetition to reduce thermal and temporal bias.
- Use the same payloads, DNS schedules, certificate material, delay injection and hardware for every applicable baseline.
- Report raw samples or lossless histogram artifacts. Report median and p50/p95/p99, plus 95% bootstrap confidence intervals. Include rejected workloads in their own distributions.
- Report both the median of per-run metrics and the pooled distribution; do not hide run-to-run variance.
- Retain failed runs and explain exclusions. No cherry-picked best run.
- Publish a reproduction command, fixture/harness revision, machine manifest, config files and machine-readable result. No aggregate score may hide a security failure.

## 6. Metrics and proposed gates

### Security gates — hard requirements
1. Unauthorized socket attempts: **0**.
2. Successful connection peer outside the exact authorized set: **0**.
3. Redirect/retry/fallback without a fresh authority decision: **0**.
4. Ambient proxy or alternate-resolver escape in the egress workload: **0**.
5. Retained response body leases above the configured global budget: **0**.
6. Invalid TLS identity accepted: **0**.
7. Required adversarial test omitted, skipped or missing a negative control: **not qualified**.

A single failure blocks qualification regardless of latency.

### Correctness and availability gates — proposed targets
- At least 99.9% successful completion on the deterministic allowlisted corpus under fault-free fixtures.
- 100% deterministic rejection of known-disallowed address/authority/redirect cases.
- No unexplained behavior changes across 5 repeated runs.
- Report benign false-positive rate by address vector and authority; do not waive conservative false positives silently.

### Performance objectives — initial hypotheses, not achieved results
- Warm-path p95 overhead versus the hardened Reqwest reference: target ≤10%.
- Cold-path p95 overhead: report without an initial pass threshold until the DNS/TLS setup costs are characterized.
- At saturation, bounded admission and predictable rejection are more important than maximum throughput.
- Slow-consumer retained response buffers must remain within the 128 MiB logical lease budget; publish total RSS separately.
- Report all resource/latency Pareto points. A design that is faster by dropping checks is not a valid winner.

These targets are deliberately provisional. After the first complete baseline campaign, freeze a versioned acceptance profile before tuning Prism. Do not move thresholds after seeing a result without publishing a new protocol version and rationale.

## 7. Evidence model and independent observation

Each logical request receives a random non-secret run ID and a one-use intent/capability ID. Record hashes—not credentials or full sensitive URLs—of:
- canonical authority and policy version;
- address-set snapshot and resolver transcript;
- each hop's method/origin and redirect decision;
- observed peer, TLS server identity result, protocol and response outcome;
- byte limits, bytes accepted, elapsed phases and lease lifecycle;
- binary/source/configuration and fixture revisions.

Hash-chain the ordered event records for tamper-evident run integrity. A hash chain does not authenticate the producer; signing keys, key rotation, verifier identity and attestation policy must be specified before calling receipts signed or attested.

On Linux, add an optional independent observer attached to the workload cgroup using `BPF_CGROUP_INET4_CONNECT` and `BPF_CGROUP_INET6_CONNECT` (or an equivalent kernel/network telemetry mechanism). Correlate connect events with the application run ID via a bounded correlation channel. Test event loss, buffer overflow, process restart and detach behavior. The cgroup hook is an observation/enforcement point, not automatically a cryptographic attestation. Keep the observer separate from the application that emits EgressReceipt.

For deployment-defense tests, enforce a deny-by-default network policy outside the process. Demonstrate that a second process in the same workload cannot reach loopback, link-local, metadata or private destinations that the workload is not meant to access. Preserve explicit exceptions only when a deployment's requirements justify them and test those exceptions independently.

## 8. Current comparison notes (not benchmark results)

- **OWASP SSRF guidance:** recommends validating resolved A/AAAA addresses, connecting to a validated IP while retaining the original hostname for Host/SNI/certificate checks, and disabling unsafe automatic redirects. Prism's current design follows this application-level pattern; PEAB must test actual socket peers, retries and every redirect hop, not just URL parsing.
- **Envoy Dynamic Forward Proxy:** provides DNS caching, DNS refresh/TTL behavior, resolver circuit breakers, host-cache limits and `resolved_address_filter`. Its documented all-addresses-filtered behavior treats resolution as a failure and retains a previously resolved host entry. PEAB must measure that precise stale-cache transition with controlled DNS changes. This is a semantic difference to characterize, not proof of a vulnerability.
- **Cilium FQDN policy:** combines learned DNS answers with L3/L4 policy and is valuable defense in depth. Its documentation describes DNS-proxy collection of responses, TTL-aware address insertion, and a trust assumption around intercepted cluster DNS. PEAB should test DNS proxy availability, resolver trust, multi-pod address learning and bypass attempts.
- **Reqwest/Hyper:** Reqwest exposes `resolve_to_addrs` and an HTTP/1 response header-count limit. Hyper's lower-level HTTP/1 client builder also exposes `max_header_size` for response headers; evaluate whether a Hyper-only transport can close Prism's currently documented pre-parse aggregate-header-byte limitation without weakening DNS pinning, SNI, ALPN, cancellation or memory accounting. Hyper documents that changing header-count handling may affect allocations/performance; benchmark this rather than assuming it is free.

## 9. Deliverables and release discipline

A completed PEAB run produces:
1. `manifest.json`: protocol version, machine, software, config, corpus and run identifiers.
2. `events.jsonl`: ordered, redacted resolver/authority/socket/TLS/HTTP/redirect/retry/resource events.
3. `samples.csv` or lossless histogram files: raw latency and resource samples.
4. `security-results.json`: per-case decision, observed socket set, negative-control result and pass/fail.
5. `summary.md`: comparison tables, confidence intervals, exclusions, limitations and reproduction instructions.
6. Content digests for every artifact, with signatures only after the signing model is implemented.

A report is **incomplete** if it lacks raw artifacts, exact versions, negative controls, security results, or exact source/configuration revisions. A benchmark pass does not itself certify production readiness. Production qualification also requires platform coverage, threat-model review, dependency review, incident/failure drills and independent assessment.

## 10. Initial implementation sequence

1. Convert the existing DNS, redirect, TLS and memory lifetime tests into named PEAB cases with stable IDs and structured outcomes.
2. Add the hardened Reqwest reference transport in a separate benchmark-only target; share test vectors, not Prism internals.
3. Add Envoy DFP configuration with pinned version and a matching address matcher; add the stale-answer/filtered-refresh case.
4. Add a Linux namespace/cgroup test profile and an independent connect observer; prove event-loss detection.
5. Add slow-consumer RSS and resolver saturation workloads.
6. Only then publish comparative performance numbers and set frozen performance acceptance thresholds.

## References

- OWASP, Server Side Request Forgery Prevention Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Server_Side_Request_Forgery_Prevention_Cheat_Sheet.html
- Envoy 1.40.0, Dynamic Forward Proxy DNS cache configuration: https://www.envoyproxy.io/docs/envoy/v1.40.0/api-v3/extensions/common/dynamic_forward_proxy/v3/dns_cache.proto
- Cilium, Locking Down External Access with DNS-Based Policies: https://docs.cilium.io/en/stable/security/dns/
- Cilium, Layer 3 Policies / DNS-based policies: https://docs.cilium.io/en/stable/security/policy/layer3/
- Reqwest ClientBuilder: https://docs.rs/reqwest/latest/reqwest/struct.ClientBuilder.html
- Hyper HTTP/1 client connection Builder: https://docs.rs/hyper/latest/hyper/client/conn/http1/struct.Builder.html
- Hyper 1.12.0 parser feasibility probe: test-only negative-control and bounded cases exercise max_header_size against a large single response header. This is not production transport migration. Hyper also documents a heap-allocation/performance trade-off when setting max_headers; include it in P1 measurements.
- eBPF cgroup socket address programs: https://docs.ebpf.io/linux/program-type/BPF_PROG_TYPE_CGROUP_SOCK_ADDR/


## 11. Industry behavior details frozen for the initial comparator review (2026-11-18)

The benchmark must compare behavior, not merely configuration names.

### Envoy DFP

The Envoy v1.40.0 DNS cache uses record TTLs subject to a configurable minimum refresh interval (documented default 5 seconds), resolver query timeouts, host-cache TTL/limits and pending-request circuit breakers. The HTTP DFP architecture also documents original-host SNI and SAN verification behavior.

When `resolved_address_filter` removes every new address, Envoy treats the refresh as failed and retains a previous resolved host address. This is a defined stale-cache behavior, not proof of a security defect. PEAB-ENVOY-001 must test a cached-good answer followed by an all-filtered refresh; record cache state, whether old connections remain alive, whether *new* connections can use the stale address, and when host TTL/circuit-breaker behavior evicts it.

### Cilium FQDN policy

Cilium learns IPs from intercepted DNS proxy responses, caches them with DNS TTL semantics and compiles matches into L3 address policy that can be combined with L4 port policy. Its threat model explicitly assumes those intercepted responses come from trusted cluster DNS. Address observations for a domain can be collected from multiple pods managed by a Cilium instance, so the permission scope is not inherently one request/one DNS snapshot.

Cilium also has connection-tracking lifecycle behavior: expired DNS mappings may remain deferred while conntrack determines whether an existing flow uses them. Existing connections can remain allowed, while new connections after the (possibly configured minimum) TTL require fresh DNS state. PEAB-CILIUM-001 must measure active-flow and new-flow behavior around expiry, resolver changes and DNS-proxy/agent restarts.

### Required matched scenarios

- **Cache poisoning/staleness:** cached public answer → new private-only answer → all answers filtered; measure stale cache and active/new connection outcomes.
- **Expiry race:** DNS TTL 1s, 5s, 30s and 300s; client starts before expiry, during expiry, and after expiry; no silent TTL extension.
- **Cross-workload learning:** two pods/workloads query the same hostname but receive different answer sets; test whether each workload can use the other's observed address.
- **Resolver trust:** untrusted DNS server, resolver substitution, DNS proxy restart, truncated UDP/TCP retry and NXDOMAIN/SERVFAIL.
- **HTTP authority:** same DNS answer set but different Host/SNI, port, redirect target, parent capability and HTTP method; an L3/L4 match must not be mistaken for HTTP authorization.
- **Existing connection vs new connection:** lease expiry/revocation blocks new socket admission; existing established sessions are measured separately. Kernel connect hooks do not themselves terminate established flows.

### Proposed composition-specific differentiation

The target Prism+Nixward property is a request/hop-scoped resolution lease bound to one parent capability, workload identity and policy generation. Nixward resolves once and supplies the exact complete A/AAAA set; Prism pins its HTTP transport to those same addresses while retaining the hostname for Host/SNI/TLS. Redirects/retries need fresh leases. This is narrower in lifetime/scope than a general workload DNS cache, but its operational cost and security benefit must be demonstrated experimentally.

References: Envoy v1.40.0 DNS cache https://www.envoyproxy.io/docs/envoy/v1.40.0/api-v3/extensions/common/dynamic_forward_proxy/v3/dns_cache.proto ; Envoy HTTP DFP https://www.envoyproxy.io/docs/envoy/v1.40.0/intro/arch_overview/http/http_proxy ; Cilium DNS policy https://docs.cilium.io/en/stable/security/dns/ ; Cilium L7 DNS proxy https://docs.cilium.io/en/stable/security/policy/layer7/ ; Cilium FQDN cache/conntrack lifecycle https://docs.cilium.io/en/latest/contributing/development/debugging/ ; OWASP SSRF https://cheatsheetseries.owasp.org/cheatsheets/Server_Side_Request_Forgery_Prevention_Cheat_Sheet.html
