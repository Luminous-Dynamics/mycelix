# Luminous Dynamics Egress, Network Authority and Evidence Integration Map

Status: repository architecture map and integration plan. It does not claim cross-repository runtime integration or production qualification.

## 1. Finding

Prism is not an isolated security project. Multiple Luminous repositories already implement complementary parts of the same evidence-bound execution problem:

- Prism: request-level HTTP authority, DNS pinning, peer/TLS checks, response resource bounds and per-hop authorization.
- Nixward: workload identity, owner-authorized network capabilities, cgroup-BPF enforcement contracts, runtime lab, policy activation and metadata-only network telemetry.
- Symthaea Browser: browser action capabilities and URL preflight policy, with an explicit need for a lower-level DNS/socket boundary.
- Symthaea Iroh patch: a precedent for pinning a Reqwest probe to a resolved address set.
- Symthaea flight/fabrication evidence: canonical evidence records, external signing-provider boundaries, one-use execution authority and recovery semantics.
- Mycelix Health: signed post-build artifact evidence, offline signing, signer trust/rotation and fail-closed artifact identity verification.
- Xenia: human operator consent, role-scoped authority, signed consent records, revocation and replay-protected encrypted sessions.
- SCAP: concrete slow-consumer/backpressure lessons that reinforce Prism's retained-response memory budget.

The new integration step is Nixward PR #162: a proposed, versioned resolution lease that composes Prism's exact HTTP capability with Nixward's workload/cgroup network boundary.

## 2. Repository map

| Repository / component | Existing asset | Relationship to Prism | Important limit |
|---|---|---|---|
| [Mycelix Prism](https://github.com/Luminous-Dynamics/mycelix/pull/6299) | `prism-egress`: typed request capability, DNS snapshot pinning, original Host/SNI, peer membership, redirect reauthorization, response memory lease and PEAB | Application-level HTTP authority and peer/TLS observation | Current receipt is not a signed attestation; no independent kernel enforcement is wired in yet |
| [Nixward network covenant](https://github.com/Luminous-Dynamics/nixward/blob/main/src/network_covenant.rs) | `WorkloadIdentity`, `NetworkCapability`, `NetworkPolicyPlan`, expiring `NetworkCapabilityLease` | Owner-authorized workload-level policy and scope | Existing capability lease does not carry a resolved A/AAAA snapshot |
| [Nixward network enforcement](https://github.com/Luminous-Dynamics/nixward/blob/main/src/network_enforcement.rs) | Compiles policy into cgroup-BPF rules; exact kernel feature and cgroup-binding proof | Outer workload containment so a process cannot bypass Prism via another socket API | Hostname rules remain unresolved pending a resolution covenant; do not claim production enforcement from the plan type |
| [Nixward runtime lab](https://github.com/Luminous-Dynamics/nixward/tree/main/tests/network-runtime-lab) | Disposable NixOS VM, test-only cgroup BPF program and 14 required allow/deny/lease/generation/failure cases | Foundation for runtime bypass and lease-expiry tests | Current BPF lab is port/mode/generation oriented, not a production exact-IP-set policy |
| [Nixward telemetry](https://github.com/Luminous-Dynamics/nixward/blob/main/src/network_telemetry.rs) | Metadata-only, bounded, BLAKE3 hash-chained ledger; no payload capture and no enforcement authority | Privacy-preserving network event correlation | Hash chain does not authenticate collector identity |
| [Nixward network architecture PR #67](https://github.com/Luminous-Dynamics/nixward/pull/67), [#71](https://github.com/Luminous-Dynamics/nixward/pull/71), [#72](https://github.com/Luminous-Dynamics/nixward/pull/72) | Network architecture/task corpus, deterministic fault harness, offline rtnetlink evidence verifier | Existing work to extend, not duplicate | Read-only/offline verifiers are not live egress enforcement |
| [Symthaea BrowserSafetyPolicy](https://github.com/Luminous-Dynamics/symthaea/blob/main/crates/domains/symthaea-browser/src/safety.rs), [BrowserExecutor](https://github.com/Luminous-Dynamics/symthaea/tree/main/crates/domains/symthaea-browser/src) and [boundary PR #12789](https://github.com/Luminous-Dynamics/symthaea/pull/12789) | Explicit Observe/Navigate/Interact/EnterText capabilities, Phi thresholds, URL/origin/IP-literal preflight, local-network deny by default, plus a 12-case browser network test plan | Browser intent and navigation safety | URL preflight does not control Chromium's actual DNS results or sockets; subresources, service workers, WebSockets and UDP need an isolated workload boundary and trusted resolver path |
| [Symthaea Iroh report generator](https://github.com/Luminous-Dynamics/symthaea/blob/main/patches/iroh/iroh/src/net_report/reportgen.rs) | Reqwest `resolve_to_addrs` use for relay/captive-portal probes | A smaller example of DNS-pinned transport in the codebase | It is not the complete Prism policy model; PEAB should test complete dual-stack semantics and observed-peer evidence |
| [Symthaea helicopter signed flight evidence](https://github.com/Luminous-Dynamics/symthaea/blob/main/crates/domains/symthaea-helicopter/src/evidence_signature.rs) | Canonical recorder bytes, domain-separated digest, explicit external `EvidenceCryptoProvider`, fail-closed unavailable provider | Useful precedent for signing Prism/Nixward evidence without inventing ad-hoc crypto | Signature strength depends on the actual provider, key custody and trust policy; the trait is not itself a signing implementation |
| [Symthaea fabrication secure release](https://github.com/Luminous-Dynamics/symthaea/tree/main/crates/domains/symthaea-fabrication-kernel/docs) | Exact artifact manifest, verified attestation, single-use authorized job, receipt and recovery binding | Pattern for exact policy/binary/BPF object provenance | Its verifier/provider boundaries must be reused explicitly; do not conflate deterministic checksums with authenticity |
| [Mycelix Health deployment evidence](https://github.com/Luminous-Dynamics/mycelix-health/blob/main/docs/release/DEPLOYMENT_EVIDENCE_PROTOCOL.md) | Post-build Ed25519 artifact manifest, canonical signed bytes, offline signing ceremony, trust-store rotation/revocation | Strong precedent for independently verifiable signed release/evidence bundles | Explicitly does not claim transparency-log inclusion, global rollback prevention or hardware-backed keys |
| [Xenia wire](https://github.com/Luminous-Dynamics/xenia-wire) and [Xenia peer](https://github.com/Luminous-Dynamics/xenia-peer) | AEAD framing, replay window, key epochs, PQC handshake options, operator consent ledger and revocation | Possible human/operator approval channel and protected remote session | Pre-alpha; not an HTTP egress proxy, DNS resolver or IP-layer firewall |
| [SCAP fork](https://github.com/Luminous-Dynamics/scap) | PipeWire frame backpressure work; measured unbounded-channel memory growth and bounded-channel correction | Relevant to Prism's slow-reader, cancellation and retained-memory qualification | Different data plane; use as a backpressure design reference, not a direct HTTP transport dependency |
| [Luminous Platform](https://github.com/Luminous-Dynamics/luminous-platform) / [Luminous Edge](https://github.com/Luminous-Dynamics/luminous-edge) | Composition of Nixward, system platform and Xenia network/session layer | Future deployment packaging and platform boundary | Composition diagrams are not proof that the Prism/Nixward bridge is deployed |

## 3. Proposed architecture: one request, two enforcement planes

```text
        owner / delegated network policy
                    |
                    v
       Nixward NetworkCapability (workload)
                    |
       authenticated resolution authority
       complete A + AAAA + CNAME + TTL
                    |
        short-lived ResolutionLease
                    |
          +---------+---------+
          |                   |
          v                   v
   Nixward cgroup BPF     Prism prism-egress
   exact IP:port set      exact URL/method/headers
   bound workload         pinned to same IP set
   monotonic expiry        original Host/SNI/TLS
          |                   |
          |                   v
          |             observed remote_addr
          |             body/memory/redirect
          |             EgressReceipt
          |                   |
          +--------+----------+
                   |
          correlation by opaque
          request/hop/lease IDs
                   |
          privacy-minimized evidence
```

The intended contract is **dual-plane but not duplicate authority**:

- Nixward decides whether this workload may reach this hostname/address scope and exact port at the kernel boundary.
- Prism decides whether this specific HTTP request is allowed, whether its TLS identity is valid, how redirects are authorized, and how much data/time/memory it may consume.
- Both use the same resolver snapshot. Prism must not independently resolve again after Nixward issues the lease.
- Nixward's cgroup hook provides an independent admission signal, not proof of a completed TCP connection or authenticated TLS peer. Prism's `remote_addr()` and certificate verification remain necessary.
- A second process in Prism's cgroup must be denied from reaching any destination outside the active address/port lease set.

## 4. The role of the browser agent

Symthaea Browser is a distinct trust context, not another public-web fetch client.

Its `BrowserCapability::Navigate` and `BrowserSafetyPolicy` should remain the intent/preflight layer. Chromium itself performs DNS, TCP/QUIC, TLS, redirects, subresource fetches and service-worker activity. Routing only top-level navigation URLs through `prism-egress` would not constrain all browser-originated sockets.

The correct integration is:
1. Treat a browser session as a separately identified Nixward workload or guest.
2. Require explicit user/navigation authority for each navigation intent.
3. Apply a distinct network policy to the browser workload; preserve deliberate LAN access only under an explicit local-navigation profile.
4. Prevent browser subresources, redirects, service workers and native components from bypassing workload-level enforcement.
5. Keep Prism public-web authority stricter than browser LAN authority. Never add global private-network allowances to Prism to make local browsing work.

## 5. Evidence and signing: reuse, do not reinvent

The repositories already contain useful precedents, but each provides a different guarantee:

- Prism EgressReceipt: application-produced event record; currently unsigned.
- Nixward NetworkTelemetryLedger: metadata-only hash chain; collector authenticity is not provided by the chain itself.
- Symthaea SignedFlightSegmentSeal: fail-closed external crypto-provider boundary around canonical evidence bytes.
- Mycelix Health: actual Ed25519 signed artifact statement plus offline signing and trust-store lifecycle.
- Xenia consent ledger: a human/operator consent event and revocation record, not an automatic network capability.

The shared future contract should separate:
1. **intent/authorization evidence** (who permitted this operation and under which parent policy);
2. **execution evidence** (what the application and kernel observed);
3. **artifact provenance** (which source/build/BPF object produced the executors);
4. **independent verification** (who checked the signatures, state transition and raw evidence).

A digest identifies bytes. A hash chain links records. Neither establishes who observed the event or whether the observation is complete. Signed and independently verified receipts require a concrete key/trust/verifier design.

## 6. Industry comparison and where the ecosystem can differentiate

Envoy Dynamic Forward Proxy already provides a mature DNS cache, refresh controls and resolved-address filtering. Cilium FQDN policy adds a separate L3/L4 workload boundary using DNS-proxy observations and has explicit DNS trust assumptions. Prism alone cannot replace those deployment controls; Nixward alone cannot interpret HTTP authority, TLS SNI or redirect semantics.

The target differentiated property is a **shared, exact, short-lived resolution snapshot with independent application and kernel enforcement**, measured end-to-end. PEAB should compare:
- hardened Reqwest reference;
- Prism application controls only;
- Nixward workload/cgroup enforcement only;
- Prism + Nixward composition;
- Envoy DFP with resolved-address filtering;
- Envoy/Cilium (or equivalent enforced network-policy) composition.

Hard security gates: zero successful connections outside the authorized address set, zero redirects/retries without new authorization, zero cgroup bypass, zero stale/replayed lease use, zero map partial-update grants, and zero invalid TLS identity acceptance. Then compare latency, throughput, resolver queries, map updates, memory, CPU, admission queueing, fault recovery and benign false-positive rate. Do not claim SOTA superiority until the matched campaign is run.

## 7. Current work and next steps

### Committed / current source

- Prism egress implementation and PEAB protocol: [Mycelix PR #6299](https://github.com/Luminous-Dynamics/mycelix/pull/6299).
- Nixward resolution lease integration contract/schema/fixtures: [Nixward PR #162](https://github.com/Luminous-Dynamics/nixward/pull/162).
- Existing Nixward network architecture and evidence work: [#67](https://github.com/Luminous-Dynamics/nixward/pull/67), [#71](https://github.com/Luminous-Dynamics/nixward/pull/71), [#72](https://github.com/Luminous-Dynamics/nixward/pull/72).

### Execution sequence

1. Review and freeze the parent address-scope semantics and resolver trust profile.
2. Implement a pure Nixward resolution-lease model and strict adversarial/property tests; no privileged writes.
3. Implement the trusted resolver adapter and authenticated broker IPC.
4. Extend the isolated BPF lab to exact IPv4/IPv6 address+port sets, atomic multi-entry generations and independent process bypass attempts.
5. Add Prism adapter to consume Nixward's exact snapshot and correlate application/kernel evidence.
6. Run PEAB matched baselines and full slow-consumer/RSS/fault campaigns.
7. Consider a one-workload owner-authorized NixOS VM pilot only after exact target-host activation evidence.

No production integration, signed Prism receipt, successful current-head CI result or PEAB measurement is claimed by this map.


## 8. Browser-specific test corpus

Symthaea Browser now has a companion machine-readable plan in [PR #12789](https://github.com/Luminous-Dynamics/symthaea/pull/12789): 12 stable scenarios for IP literals, DNS rebinding, redirects, private subresources, service workers, WebSocket/WebTransport, UDP/WebRTC/QUIC, alternate DNS, explicit LAN access, cgroup moves and failure recovery. The cases are explicitly labelled unrun.

This is a separate test surface from PEAB's HTTP client scenarios. A Reqwest success/failure test cannot demonstrate Chromium network isolation, and a browser URL preflight unit test cannot prove that all Chromium-originated sockets are constrained.

