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
| Parent authority scope | Substitute resolution-lease scope digest; mix public/private A/AAAA answers; attempt an out-of-scope private CIDR | Lease matches the exact owner-authorized parent capability and every resolved address fits the parent scope |
| Browser workload networking | Chromium redirects, subresources, workers, WebSockets, WebRTC/UDP, QUIC and alternate DNS | Kernel workload policy blocks unapproved sockets independently of CDP action receipts |


The resolved-address scope is owner authority, not a property inferred from DNS. Every hostname parent capability must bind an explicit address scope (public-unicast-only or exact approved CIDRs); the short-lived resolution lease must carry the matching scope digest and all returned addresses must fit it. For integrated HTTP, the same exact address set must be consumed by Prism's pinned transport and Nixward's workload enforcement. Chromium is a separate client with its own resolver and subresource network stack; Prism's Reqwest transport does not mediate it.

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


## 12. Observer completeness, socket identity and connection lifecycle

The independent observer must be treated as a fallible measurement instrument. A missing event is not evidence that no socket was opened. For every observer interval, record the cgroup identity and generation, attach/detach lifecycle, monotonic clock domain, event sequence, buffer capacity and kernel-reported lost-event counters. If a ring-buffer or perf-buffer loss counter is nonzero, if the observer restarts without a reconciled sequence boundary, or if a socket event cannot be joined unambiguously to its workload generation, mark the affected interval and run **incomplete**. Never convert an incomplete observation into a zero-violation result.

Correlate with kernel socket identity (for example, socket cookie where available), cgroup identity/generation, process identity and a bounded short-lived intent token. Destination tuple and timestamp alone are not a sufficient join key under concurrency, NAT, retries or rapid socket reuse. Keep the application receipt and kernel observer independently produced; the correlation layer must not let the application assert its own kernel observation.

A cgroup connect hook observes or controls a new connect operation. It does not, by itself, prove TLS identity, HTTP authority, application intent, packet delivery, or termination of an already-established flow. Test established-flow continuation separately from new-connect admission. Also test inherited and SCM_RIGHTS-passed connected descriptors: a second process may use an existing socket without issuing a new connect event. Workload-level packet enforcement or protocol-aware mediation is required if policy intends to constrain that use.

Connection pooling is a separate research track, not an automatic optimization. A future pool may reuse a connection only when the complete compatibility key is equal: scheme/authority/port, workload and trust context, authorized address-set digest, verified TLS identity, policy generation and expiry/lifecycle state. A changed DNS snapshot or policy generation must not silently inherit an old socket's authorization. Measure the cost of this rule against both no pooling and a conventional authority-keyed pool; report cold and warm results separately.

New planned PEAB cases: OBS-001 (buffer overflow invalidates observer completeness), OBS-002 (unambiguous concurrent event correlation), KERNEL-002 (inherited/passed sockets), KERNEL-003 (established versus new connection lifecycle), and POOL-001 (exact-scope connection reuse). These are planned cases, not implemented tests or passing results. The manifest intentionally keeps their source coverage as `planned` and execution evidence null.

## 13. Evidence reconciliation and the network-action transaction

The composed Prism/Nixward path should be treated as a **distributed, fail-closed network-action transaction**, not as one boolean authorization check. It crosses independent authorities and failure domains: owner policy, resolver, broker/replay ledger, kernel map generation, HTTP transport, TLS verifier, observer and durable evidence store.

### 13.1 Evidence states

Do not collapse all outcomes into success/failure. Each hop has one of these explicit evidence states:

| State | Meaning | Permitted conclusion |
|---|---|---|
| PENDING | Intent or lease exists; execution is not yet reconciled | No execution claim |
| MATCHED | Prism and Nixward records correlate; observer interval is complete; address, workload, policy generation and peer agree | Corroborated execution for the measured hop |
| DENIED | Authority or kernel policy denied the action and the relevant observation window is complete | Denial for that attempted action |
| UNMATCHED | Kernel event exists without a durable application receipt, or vice versa | Investigation/reconciliation required |
| INCOMPLETE | Observer loss, detach gap, missing clock/sequence boundary or ambiguous correlation | No zero-violation or complete-execution claim |
| CONFLICT | Records disagree on capability, lease, workload, generation, address set or peer | Quarantine the hop; never select the more favorable record |

MATCHED requires both planes; a Prism receipt alone is an application assertion, while a cgroup connect event alone is not proof of a completed TCP handshake, TLS identity or HTTP response. DENIED must be scoped to an explicit attempted action and complete observation interval. An absent event is not proof of no event if the observer was not demonstrably complete.

### 13.2 Transaction stages

1. **Prepare authority:** verify the owner-authorized parent capability, exact workload identity, protocol, port and address-scope digest.
2. **Resolve:** the trusted resolver obtains the complete A/AAAA/CNAME result and validity interval; classify the complete set under the parent scope.
3. **Issue lease:** canonicalize and digest the lease; bind resolver identity, request/hop identity, parent and workload/policy generations; sign only with the dedicated resolver action/audience.
4. **Consume nonce:** atomically consume the stable one-shot lease nonce in the durable replay ledger. A challenge timestamp must not create a new replay identity for the same lease.
5. **Stage enforcement:** write the exact IP+port set into an inactive kernel-map generation and read back/verify its contents and cgroup binding.
6. **Commit enforcement:** atomically activate the generation. If the result is unknown, query the active generation and transaction record before retrying. Unknown outcome means no new grant.
7. **Execute HTTP:** Prism consumes the request capability, pins the client to the exact leased addresses, preserves the original hostname for Host/SNI/certificate validation, and checks the actual peer. Redirects and retries require fresh hop leases.
8. **Observe independently:** Nixward records socket-admission and packet-level enforcement observations separately from the Prism receipt.
9. **Reconcile:** join by opaque request/hop, lease/capability digest, workload/cgroup generation, kernel socket identity where supported, and a bounded correlation record. Do not rely on destination tuple plus wall-clock timestamp alone under concurrency.
10. **Close lifecycle:** expire/revoke the lease, release response-memory reservations only after final ownership ends, and persist a terminal evidence state. Existing flows and new socket admission are separate revocation questions.

### 13.3 Crash consistency and unknown outcomes

Inject a crash after every mutation boundary: nonce consumption, inactive-map write, map readback, active-generation switch, socket connect, response receipt creation, and durable receipt publication. The test oracle must distinguish:

- **Known not committed:** safe to retry only with the same transaction state and no broadened authority.
- **Known committed:** continue or report the committed generation/result.
- **Unknown:** reconcile authoritative state before any retry; do not issue a fresh nonce or a broader capability simply because the response was lost.

Keep the previous known-good static deny-by-default network policy active until the new dynamic generation is proven active. A partially staged generation must remain inert. A successful userspace write is not proof that the kernel is enforcing the intended contents.

### 13.4 Observer completeness and packet enforcement

Cgroup connect hooks are useful for connection admission attempts. They are not a complete record of every packet emitted by a workload. The test matrix must separately cover:

- TCP IPv4 and IPv6 connect attempts;
- UDP IPv4 and IPv6 sendmsg paths;
- packet-level cgroup egress enforcement;
- inherited and SCM_RIGHTS-passed connected descriptors;
- process/cgroup moves, socket lifetime and existing flows;
- observer detach, ring/perf buffer loss, restart and sequence discontinuity.

For a TCP-only HTTP(S) lease, UDP/QUIC remains denied unless a distinct protocol capability and enforcement profile are approved. A packet-level egress hook is a separate enforcement/measurement mechanism; it does not replace TLS or HTTP authority validation. Validate the actual kernel attach type and socket attribution on every supported kernel family.

A run is observer-complete only when attach/detach generations, covered time interval, event sequence and kernel loss counters are recorded and reconciled. If any are missing or report loss, mark the affected interval INCOMPLETE, not zero unauthorized connections.

### 13.5 Comparative findings that must become PEAB tests

**Envoy Dynamic Forward Proxy:** v1.40.0 resolved_address_filter removes matching IPs from a DNS response. When all addresses are filtered, resolution is treated as failure and the host retains a previously resolved address. PEAB must distinguish existing pooled connections from new connections to that retained address, record host-cache state and test expiry/recovery. This is documented behavior to characterize, not a vulnerability assertion.

**Cilium FQDN policy:** current 1.20.x documentation describes DNS-proxy learning and TTL-aware address policy, but also documents that IPs from DNS responses to all pods managed by a Cilium instance can be allowed for a matching FQDN. Its tofqdns-min-ttl setting can deliberately extend the effective lifetime beyond a shorter authoritative TTL. PEAB must test endpoint-specific answer divergence, agent-wide learning and effective minimum TTL rather than treating an FQDN rule as a request-scoped lease. These semantics are useful deployment controls, but they have a different permission scope from a one-request/one-hop capability.

### 13.6 New PEAB cases

The manifest adds planned cases for cross-endpoint Cilium DNS scope, Cilium minimum-TTL extension, UDP sendmsg bypass, passed connected descriptors, unmatched application/kernel evidence, observer detach gaps, and the lack of authoritative TTL/CNAME provenance in a generic OS resolver result. All remain not_run_exact_head with null execution evidence until implemented and executed.

References:
- Envoy v1.40.0 DNS cache resolved_address_filter: https://www.envoyproxy.io/docs/envoy/v1.40.0/api-v3/extensions/common/dynamic_forward_proxy/v3/dns_cache.proto
- Cilium FQDN/DNS proxy trust and address learning: https://docs.cilium.io/en/stable/security/dns/ and https://docs.cilium.io/en/stable/security/policy/layer7/
- Cilium TTL and endpoint cache configuration: https://docs.cilium.io/en/stable/helm-reference/ and https://docs.cilium.io/en/latest/contributing/development/debugging/
- Linux cgroup socket-address connect/sendmsg attach types: https://docs.ebpf.io/linux/program-type/BPF_PROG_TYPE_CGROUP_SOCK_ADDR/
- Linux cgroup packet egress enforcement: https://docs.ebpf.io/linux/program-type/BPF_PROG_TYPE_CGROUP_SKB/
- Hyper HTTP/1 client parser header-size limit: https://docs.rs/hyper/latest/src/hyper/client/conn/http1.rs.html
- OWASP SSRF Prevention Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Server_Side_Request_Forgery_Prevention_Cheat_Sheet.html


### Canonical address-set correlation digest

Prism now computes a versioned, domain-separated BLAKE3 digest for each hop's complete authorized IP:port set and records it alongside the explicit address list in the application receipt. For redirect hops the receipt carries both the source-hop set/digest and the newly authorized target-hop set/digest. The byte encoder is exposed as `canonical_address_set_encoding` and is covered by a shared golden vector. The digest uses BLAKE3 derive-key context `org.luminousdynamics.prism.address-set.v1`, followed by the unique-record count as u64 big-endian. Records sort by family (IPv4 before IPv6), raw address bytes, numeric port, then IPv6 scope ID. An IPv4 record is tag `0x04`, four address bytes, and u16 big-endian port. An IPv6 record is tag `0x06`, sixteen address bytes, u16 big-endian port, and u32 big-endian scope ID. IPv6 flowinfo is excluded. Deduplication uses exactly those encoded fields, not the full platform SocketAddr structure. The empty set has a deterministic digest but must never be accepted as an authorization.

The digest is only a stable correlation value. It is **not** a signed receipt, a DNS transcript digest, proof of resolver authenticity, proof of kernel enforcement, or independent observation. The Nixward lease digest and resolver snapshot digest remain distinct authority/evidence values. Any cross-repository consumer must adopt the exact same domain separator and byte encoding and test golden vectors before treating digests as equal. The new unit test checks order independence, duplicate elimination, port binding and address-family separation; it is source-only until exact-head execution.


### Rust-enforced PEAB manifest contract

The PEAB scenario manifest is now included at compile time by `prism-egress` tests. The Rust test enforces stable unique IDs, required negative-control declarations, coverage vocabulary, consistency between source-coverage classifications and source-test declarations, and exact-head/run/job binding for any execution evidence. An unrun case must retain null execution evidence. The existing CI source-reference scan remains complementary: it verifies that named source tests really exist in Rust files. The Rust test does not execute the scenarios; it validates the integrity of the benchmark specification. PEAB-001 records this distinction. The new Rust test is still unexecuted until the exact PR-head workflow runs.

### Current Prism ↔ Nixward encoding alignment

The current draft Nixward PR #162 now implements the same address-set byte encoder and BLAKE3 derive-key context as Prism, with the shared 38-byte IPv4+IPv6 golden vector in both codebases. Nixward uses scope ID zero for its lease-v1 bare `IpAddr` values. Nixward also exposes the digest as a correlation helper on `NetworkResolutionLeaseV1`. This is source-level cross-repository alignment—not a live resolver broker, signed IPC transaction, kernel map writer, or runtime enforcement integration. Both code changes and their golden tests remain unexecuted pending their own exact-head validation.

Related bridge work: https://github.com/Luminous-Dynamics/nixward/pull/162


### Per-hop TLS and HTTP protocol evidence

Each successful hop now records the HTTP protocol version reported by Reqwest and an optional TLS identity result. For HTTPS, the receipt records the URL reference identity, the hostname used for SNI when the URL host is a DNS name, and the fact that the configured Reqwest certificate/hostname verifier allowed a response. IP-literal HTTPS has no DNS SNI name but still records its reference identity. Plain HTTP has no TLS identity object. Redirect-hop receipts retain the same fields independently for the hop being left; the final receipt describes the final response.

This is application-level transport evidence, not a certificate-chain fingerprint, pinning proof, remote attestation, or kernel observation. It is only emitted after Reqwest returns a response; invalid TLS identity produces an error and no successful receipt. TLS-003 describes the source-level assertions; it remains unexecuted until exact-head test evidence exists.


### Nixward-compatible logical request correlation

Prism now creates a random 16-byte UUID correlation ID for standalone requests and exposes `authorize_with_authority_and_correlation` for a trusted adapter to supply an existing 32-character lowercase-hex logical request ID and zero-based redirect-hop index. The fields are copied into the final application receipt and each redirect-hop receipt. Redirects preserve the logical ID and increment the hop index; out-of-range metadata is rejected before DNS resolution. The ID is explicitly metadata for evidence joining, not a capability, secret, signer identity or source of authority. It matches the shape used by Nixward `NetworkResolutionLeaseV1.logical_request_id` and `redirect_hop`, but no live lease consumption or adapter exists yet. CORR-001 describes these source assertions; runtime evidence remains pending.


Correlation IDs remain local evidence metadata: Prism does not add them to outbound HTTP headers, URL paths, query parameters or request bodies. The redirect integration test asserts that the exact ID in the final receipt is absent from both upstream request byte streams. PRIV-001 records this privacy boundary; this is a source assertion until the test executes.


## 14. Request-scoped resolution snapshot interface

The resolver boundary now returns an \`EgressResolutionSnapshot\`, not a bare \`Vec<SocketAddr>\`. Every snapshot binds the canonical hostname, destination port, logical request ID, zero-based redirect-hop index and complete socket-address set. Optional provenance includes a monotonic validity deadline, the canonical address-set correlation digest, the signed Nixward resolution-lease payload digest and the resolver snapshot/transcript digest.

The built-in operating-system resolver creates a system snapshot with no authoritative TTL, CNAME, DNSSEC or resolver-transcript claim because \`ToSocketAddrs\` does not expose those fields. Prism applies its local capability TTL. The custom resolver interface is public for a trusted adapter; it is part of Prism's trusted computing base.

For a Nixward-backed snapshot, the adapter must first authenticate the resolver response and verify the signed lease, owner/parent capability, workload/cgroup identity, explicit address scope, policy generation, nonce/replay state, resolver audience and resolver provenance. Only then should it construct \`EgressResolutionSnapshot::from_verified_nixward_lease\`. That constructor checks field shape, 32-character request ID, hop ceiling, nonempty/bounded addresses, uniform requested socket port, future validity and the canonical address-set digest. **It does not verify signatures, workload identity or replay consumption.** Its name documents the caller precondition; the method is not a cryptographic verifier.

Prism independently checks snapshot hostname, port, logical request ID and hop against the request that triggered resolution. It sorts/deduplicates addresses, rejects empty or oversized sets, checks the set digest when present, applies the complete address policy, and sets capability expiry to:

\`min(local_capability_expiry, resolution_lease_valid_until)\`

A snapshot that does not match the request or has expired is rejected before a capability is issued. The application receipt now carries the lease payload digest and resolver snapshot digest separately from the address-set digest. Redirect-hop receipts carry source and target values separately so each hop can be reconciled independently. These are identifiers and application-reported provenance values, not signatures or independent kernel evidence.

The API is an integration seam, not a live Prism↔Nixward broker. No signed IPC transport, resolver process, replay-ledger call, kernel map staging, active-generation readback or packet enforcement was added by this change. LEASE-001 and LEASE-002 cover the implemented snapshot API and its fail-closed shape checks; both remain unexecuted at exact PR head.



### Lease provenance through the application receipt

The new local pinned-HTTP integration test supplies a structurally valid snapshot produced by the trusted-adapter constructor, authorizes an exact request/hop, executes against the snapshot's loopback socket in a test-only destination profile, and checks that the final application receipt preserves the logical request ID, address-set digest, resolution-lease payload digest, resolver snapshot digest and observed peer. It also checks that the correlation ID is absent from outbound HTTP request bytes. This test does not verify a Nixward signature or exercise a live broker: the fixture represents a snapshot after the external adapter's cryptographic and parent/workload checks. LEASE-003 records the source-level assertion and remains unexecuted at exact head.
