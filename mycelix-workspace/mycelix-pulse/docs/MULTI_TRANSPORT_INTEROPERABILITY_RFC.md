# Pulse Multi-Transport Communications and Standards Registry RFC

- **Status:** Proposed architecture; not implementation or conformance evidence
- **Scope:** Public Mycelix repository, Pulse transport architecture, shared standards knowledge for Pulse/Mycelix/Symthaea
- **Related work:** Pulse Holochain 0.7 migration PR #4891; Pulse V2 crypto specification; existing Pulse fail-closed delivery/evidence contracts
- **Benchmark gates:** [Transport Benchmarks and Acceptance Gates](TRANSPORT_BENCHMARKS.md)
- **Machine-readable catalogue:** [Standards registry](../standards-registry/README.md), [registry seed](../standards-registry/registry.v1.json), and [JSON Schema](../standards-registry/schema.v1.json)
- **Design principle:** One authenticated, end-to-end encrypted message model; many independently qualified delivery paths

## 1. Executive summary

Pulse should not be coupled to a single network substrate. It should be able to use multiple available paths—ordinary IP, local wireless, intermittently connected links, low-bandwidth radio gateways, cellular/satellite services, and future adapters—without changing message identity, end-to-end encryption, authorization, or evidence semantics.

“Support all EM/RF communications” is not a realistic claim for one application binary. Radio interoperability crosses different physical layers, modulation/coding, MACs, network topologies, regional spectrum rules, hardware drivers, service-provider interfaces, and sometimes closed or licensed ecosystems. Software cannot turn a phone without the appropriate radio, firmware, permissions, subscription, antenna, gateway, or legal authorization into a universal transceiver.

The achievable goal is a **transport-neutral application protocol, a stable adapter contract, a maintained standards registry, and a staged conformance program**. This makes the architecture extensible and makes unsupported combinations explicit rather than silently pretending they work.

The first implementation should be conservative: keep Pulse's current encrypted message contract and Holochain delivery path intact; create a separately testable transport abstraction; add standards inventory and deterministic link simulation before transmitting over new hardware.

## 2. Responsibilities and trust boundaries

### Pulse

Pulse owns the canonical message lifecycle: recipient identity, message identifiers, end-to-end encryption, signatures, authorization, deduplication, expiry, user-visible delivery state, and verifiable receipts. Transport choice must not change what a message means.

The currently documented V2 profile uses hybrid X25519/ML-KEM-768 key establishment, a domain-separated HKDF-SHA256 combiner, AES-256-GCM content encryption, and both Holochain-agent and ML-DSA-65 application signatures. This RFC does not replace or broaden that profile. Its security properties and exclusions remain governed by the V2 specification and release evidence.

### Mycelix

Mycelix supplies cross-application identity/credential contracts, consent and revocation semantics, durable evidence, and coordination across peers or communities. Trust metadata can help evaluate a route or relay, but a peer's reputation must never substitute for a cryptographic message check or a real authorization decision.

### Symthaea

Symthaea may estimate delivery probability, link stability, energy/cost trade-offs, congestion, and likely disconnection; recommend route order; and explain uncertainty. Initially, its decisions are **advisory and reproducible**. A deterministic policy engine enforces hard constraints. Symthaea must never override user consent, cryptographic checks, capability revocation, regulatory controls, device safety controls, or downgrade protection.

Model context is not a source of standards truth. Symthaea should retrieve versioned entries from the standards registry, cite their source and revision, report missing evidence as unknown, and never invent support claims.

### Transport adapters and radio hardware

Adapters bridge Pulse's encrypted frames to operating-system network APIs, approved device interfaces, or authenticated gateways. Adapters do not receive plaintext message content or private message keys. Drivers that require special privileges run outside the core application and expose a narrow, typed IPC boundary.

The application must distinguish a direct radio link from an IP service provided over a radio network, and from a gateway translating one protocol into another. These are not equivalent forms of native radio interoperability.

## 3. Proposed path

1. Compose and authenticate a canonical Pulse message.
2. Encrypt and sign it using the negotiated, explicitly supported cryptographic profile. No adapter may downgrade that profile.
3. Produce a transport-neutral encrypted frame with stable logical message ID, envelope version, bounded payload, expiry, and the minimum routing metadata required.
4. Ask a deterministic policy layer to filter adapters by permissions, hardware availability, user choice, region, link capability, security and cost constraints.
5. Let a baseline route scorer order the eligible adapters; Symthaea may later provide a bounded recommendation alongside the baseline score.
6. Send through one or more selected adapters. A durable outbox records attempted, accepted, deferred, failed, and acknowledged states separately.
7. Reassemble and deduplicate at the receiving side before passing the encrypted envelope to Pulse's existing verification/decryption pipeline.
8. Produce receipts that identify what was actually observed. A local adapter acceptance, gateway acknowledgement, remote cryptographic receipt, and recipient-visible delivery are different facts and must remain distinct.

Holochain remains a peer/application substrate rather than an all-purpose radio driver. The current migration work documents a Holochain 0.7 transport change to Iroh over QUIC and requires a fresh conductor due to incompatible DNA/runtime changes. That does not by itself add Bluetooth, LoRaWAN, cellular-radio control, amateur radio, or satellite service access. Keep that migration scope independent.

## 4. Adapter contract

Implement the core contract in Rust after reviewing the current Pulse workspace conventions. The design should express these concepts without making hardware-specific types leak into message semantics:

- Adapter identity, version, declared standards profile, and implementation maturity.
- Capabilities: send/receive, unicast/group support, payload budget, effective throughput, expected latency, power/airtime cost, delivery/receipt support, available directionality, and whether a path is online, opportunistic, or store-and-forward.
- Availability and health observations with timestamp, source, confidence, and expiry. Stale link information is unknown, not healthy.
- Explicit user and OS permission state, device presence, radio/service availability, and relevant regional constraints.
- Send result categories that distinguish local queue acceptance, link-layer acknowledgement, remote gateway acknowledgement, permanent rejection, temporary unavailability, and uncertain outcome.
- Bounded fragmentation/reassembly with authenticated envelope identity, expiry, resource quotas, and replay resistance.
- Cancellation, backpressure, retry budget, and idempotent duplicate handling.

The adapter contract must not accept a caller-supplied “verified=true” boolean as evidence. Proofs and verification results should be typed and produced by the relevant verifier at the proper authority boundary.

### Payload and bandwidth truth

Not every path is suitable for normal messages. Narrowband/LPWAN paths may have small regional payload limits, airtime restrictions, low downlink capacity, duty-cycle constraints, or network-server dependencies. On such paths, Pulse may first support compact encrypted alerts, delivery hints, or a user-authorized reference to content available over another path. Do not advertise full chat delivery over a path until representative ciphertext sizes, fragmentation, latency, costs, and loss/retry behaviour have been measured.

Do not automatically place sensitive message content into SMS, push-notification previews, untrusted gateways, public DHT records, radio telemetry, or routing metadata. Keep envelope encryption independent of the carrier.

## 5. Standards registry: initial inventory to maintain

This table is an **inventory seed**, not a claim that Pulse currently implements or conforms to these specifications. Exact editions, amendments, errata, required profiles, regional parameters, licence terms, and certification requirements must be checked against the authoritative source during implementation and recorded in versioned registry data.

| Domain | Families and standards to track | Integration boundary / priority |
|---|---|---|
| Core Internet | IPv4/IPv6, UDP, TCP, TLS 1.3, QUIC, DNS/DNSSEC and relevant IETF specifications | First: normal IP paths; observe OS network changes. QUIC is an Internet transport, not a radio standard. |
| Wi-Fi / WLAN | IEEE 802.11 family, including appropriate generations; Wi-Fi Direct and Wi-Fi Aware where the operating system exposes them; 802.11ah / Wi-Fi HaLow | High: IP connectivity and local/offline discovery where permitted. Do not equate local Wi-Fi with mesh support. |
| Bluetooth | Bluetooth Core, GATT and relevant profiles; Bluetooth Mesh where applicable | High: nearby device exchange / provisioning where OS APIs permit; measure payload and background limitations. |
| Low-rate device networks | IEEE 802.15.4; Thread; Zigbee/Connectivity Standards Alliance ecosystems; Matter | Medium: keep link, network, and application layers distinct. Matter is an application interoperability layer and can use different underlying networks; it is not itself a radio PHY. |
| LPWAN | LoRa physical-layer ecosystems, LoRaWAN link/network/backend interfaces and regional parameters; Wi-Fi HaLow; other relevant LPWAN families such as mioty where supported by hardware/ecosystem | Medium/experimental: begin with a lab adapter or gateway; prioritize small, explicit message classes. LoRa and LoRaWAN are not interchangeable names for one complete network. |
| Cellular | 3GPP GSM/UMTS legacy considerations, LTE, 5G NR, NB-IoT, LTE-M, carrier APIs, SIM/eSIM interfaces where relevant | High through the operating system/IP first; native modem control requires device/vendor access, permissions, and carrier/regional constraints. |
| Non-terrestrial / satellite | 3GPP NTN evolutions; provider-specific satellite messaging/IP offerings; relevant CCSDS space-link and space-internetworking standards where applicable | Medium: prefer standard OS/IP or documented provider interfaces; satellite PHY access is not implied by NTN awareness. |
| Delay-tolerant networking | IETF Bundle Protocol v7 (RFC 9171) and current updates/errata; Bundle Protocol Security (RFC 9172); relevant convergence-layer profiles | High architecture priority for offline/outage resilience; define expiry, custody/receipt semantics, store quotas, anti-replay, and security profiles. |
| Ad hoc / mesh routing | Babel (RFC 8966), OLSRv2 (RFC 7181), AODV (RFC 3561), Bluetooth Mesh, Thread, and device/community mesh ecosystems | Research and test individually. A route protocol may be implemented below Pulse or in a gateway; avoid duplicating a mesh stack unnecessarily. |
| Amateur / community radio | AX.25, APRS, and documented modem/gateway protocols; region-specific licensing and band plans | Experimental, opt-in, gateway-first. Do not assume one radio interface or that payload privacy is provided by the radio protocol. |
| Public-safety / professional mobile radio | ETSI TETRA; relevant DMR/P25 profiles and jurisdictional/operator integration documents | Reference inventory only until authorized hardware, interoperable data interfaces, operator approval, security restrictions, and legal boundaries are established. Never bypass restricted systems. |
| Space data systems | CCSDS Blue Books and relevant Space Internetworking Services / IP-over-CCSDS documents | Specialized gateway/integration research, not a consumer-phone default. Use the CCSDS active-publication catalogue. |
| Industrial / embedded wireless | WirelessHART, ISA100.11a, Wireless M-Bus and relevant industry profiles | Only for explicit use cases and tested gateways; these protocols are not general consumer chat transports. |
| Identity / data envelope / security | W3C DID and Verifiable Credentials as applicable; Pulse V2 crypto contract; IETF application security, cryptographic agility, and content-independent transport framing | Shared profile; do not conflate transport security with end-to-end message security or a federation assertion with a canonical event. |

### Registry record requirements

Every tracked standard/profile entry should include:

- Stable registry ID and human-readable name.
- Standards body / project and canonical source URL.
- Edition/version, amendment list, errata, publication date and retrieval date.
- Current status from the authority (active, draft, deprecated, withdrawn, superseded, or unknown).
- Applicable layer, compatible profiles, required options, regional variants and known interoperability constraints.
- Licensing/patent/certification considerations, where applicable.
- Support state: **unreviewed**, **reviewed**, **prototype**, **tested against profile**, **externally certified** or **explicitly excluded**.
- Links to implementation, deterministic fixtures, conformance evidence, test hardware/profile, and exact tested commit.
- Security/privacy considerations, known limitations, provenance, and next review date.

Do not conflate “reviewed” with “implemented,” “test passed” with “certified,” or “one device works” with “interoperable across the standard.” Never label every edition as supported just because a family is present in the registry.

## 6. Keeping Symthaea and Mycelix standards-aware

Create a machine-readable registry (initially versioned TOML/JSON or YAML with a schema and deterministic validation; select the format to fit the Rust workspace and Nix tooling). Human-readable notes may supplement, but not substitute for, the machine-readable record.

The update workflow should:

1. Pull metadata from canonical standards-body catalogues or known official landing pages; where no stable API exists, require a reviewed update rather than fragile scraping.
2. Preserve the retrieved source URL, retrieval timestamp, content hash, authority status, edition and reviewer/provenance record.
3. Show source changes and likely supersession as a reviewable diff.
4. Require human review for changed normative versions, new security profiles, certification/licensing implications, regional rules, and adapter support claims.
5. Generate a standards-by-layer and standards-by-adapter matrix for docs and retrieval.
6. Supply Symthaea a retrieval interface returning bounded, sourced records and uncertainty; external documents are untrusted input, not executable instructions or policy.
7. Retain last-known metadata with age and stale status when a source is unreachable; never silently represent stale metadata as current.
8. Keep registry ingestion independent from transmission or deployment. Updating standards knowledge must not automatically enable a radio, install firmware, or transmit.

Suggested query contract: request topic, applicable layer/use case, region, and device capabilities; receive matching registry entries, authoritative source, version/status date, implementation evidence, limitations, and explicit unknowns. Recommendations must name the evidence and may recommend “unsupported / needs hardware / needs standards review.”

## 7. Routing: optimize for delivery without surrendering authority

For each eligible path, gather bounded observations: predicted reachability, measured success rate, latency distribution, payload limit, power/airtime use, monetary cost, congestion, privacy exposure, and expiry risk. The baseline selector should produce a deterministic ranking from explicit weights and hard constraints. Later, evaluate Symthaea’s recommendations against that baseline in replay/simulation before allowing even bounded advisory use.

Potential modes:

- **Normal:** prefer fast, low-cost, already-authorized paths.
- **Resilient:** queue durable ciphertext and try alternate eligible paths.
- **Low-bandwidth:** send only message classes explicitly allowed by the user/policy.
- **Offline/community relay:** use local or opportunistic links with quotas and expiry.
- **Sensitive:** prohibit paths whose metadata, gateways, retention, or end-to-end properties do not meet the user's chosen policy.
- **Emergency:** do not presume that software has emergency-service access. Use only configured, lawful, explicitly consented integrations and state their limits.

Avoid unlimited duplication across every adapter. Use policy-bounded retries, deduplicate by stable authenticated identity, cap storage, and record uncertainty. Route optimization never weakens message signatures, key negotiation, recipient policy, consent, revocation, or replay checks.

## 8. Security, privacy, and regulatory invariants

- End-to-end encryption and signature validation remain above all transports.
- No automatic downgrade to a weaker crypto suite or unsigned message.
- Adapters may see only the ciphertext and the minimum metadata they need.
- Treat relays and gateways as untrusted unless they are explicitly within a separately defined trust boundary.
- Do not expose secrets/tokens in diagnostics, logs, receipts, or RF metadata.
- Restrict device permissions and privileged radios; explicit per-user opt-in before first transmission over a new medium.
- Respect transmit power, channel/frequency, airtime/duty-cycle, licensing, certification, and local regulatory requirements. Default test mode is loopback/simulation with no RF transmission.
- Never build jamming, interference, covert interception, unlawful access, or safety-control bypass features into this project.
- Bound parser inputs, fragment counts, reassembly lifetime, queued bytes, retries, peer discovery, and relay resources; reject malformed or ambiguous frames.
- Preserve provenance and make partial delivery, stale observations, replay, congestion and unsupported hardware visible to users.
- Standardized discovery identifiers can themselves create tracking/correlation risk; default to minimum disclosure and rotating identifiers when the relevant standard and use case allow it.

## 9. Qualification plan and acceptance gates

### Phase 0 — RFC and inventory

- Review this RFC against the existing Pulse V2 contract, V2/realtime/evidence boundaries, Holochain migration, and actual workspace layout.
- Add a standards registry schema, seed official source entries, validate required fields, and generate a matrix.
- Each row begins unreviewed until its revision and scope have been checked. This phase does not claim any new transport support.

### Phase 1 — Transport-neutral contract

- Implement and test the Rust adapter/interface types without activating new radio paths.
- Add deterministic fixtures for capability negotiation, link loss, partial/unknown send outcome, duplicate/replay, expiry, malformed frames, resource exhaustion, and stale link data.
- Prove that adapter failure cannot create a plaintext fallback, fabricated delivery receipt, or crypto downgrade.
- Preserve the existing Pulse message contract and current alpha exclusions.

### Phase 2 — Simulation and IP path

- Run deterministic simulated multi-path tests with latency/loss/bandwidth/availability schedules.
- Compare baseline routing with Symthaea recommendations on identical traces; record seed, trace hash, exact subject commit, metrics, and output.
- Validate reconnect/offline queue/restart recovery and one logical message delivered over duplicated paths.
- Keep hosted-run status distinct: queued/cancelled jobs are not passes.

### Phase 3 — Nearby and outage-tolerant links

- Implement one constrained adapter at a time, selected from device access and use case—likely Bluetooth LE for nearby transfer and a BPv7-compatible store-and-forward prototype for intermittent links.
- Use virtual interfaces, emulators, and loopback first; hardware testing is an explicit separate qualification.
- For every prototype, record tested devices/versions, exact capabilities, region/profile, tested payload sizes, loss, latency, power/airtime, security results, and known gaps.

### Phase 4 — LPWAN, cellular/NTN, specialist radio

- Add adapters only when there is a documented API, supported hardware/gateway, licensing/regulatory review, and an achievable conformance method.
- Use carrier-provided IP or published APIs before attempting direct modem or SDR control.
- For narrowband links, demonstrate a measured compact-message profile before claiming general messaging capability.
- Maintain a clear feature matrix by standards profile and hardware, not a single “RF supported” badge.

### Required release evidence

Every adapter release must include exact source commit, registry revision/hash, dependency/toolchain identity, hardware/firmware and operating-system details, configuration/profile, deterministic test traces, exit codes, raw logs/artifact hashes, failed/unsupported cases, and a claim ceiling. Preserve negative controls for disabled hardware, invalid profile, region mismatch, expired/replayed frames, unauthorized adapter access, stale standards metadata, and simulated-versus-real hardware confusion.

## 10. Initial official references

These are starting points for a maintained registry. Before claiming conformance, consult the exact specification, applicable amendments/errata, and implementation test requirements—not just these summaries.

- IEEE 802.15.4 low-rate wireless networks: https://standards.ieee.org/ieee/802.15.4/11041/
- Bluetooth Core Specification 6.3 (2026-05-05): https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core_v6.3/out/en/index-en.html
- IEEE 802.11 working group: https://www.ieee802.org/11/
- Bluetooth SIG specification catalogue: https://www.bluetooth.com/specifications/specs/
- Thread Group overview: https://www.threadgroup.org/What-is-Thread/Overview
- Connectivity Standards Alliance / Matter: https://csa-iot.org/all-solutions/matter/
- LoRa Alliance developer/standard catalogue: https://lora-alliance.org/lorawan-for-developers/
- LoRa Alliance TS014, TS018 and TR016 announcement (2026-08-04; normative documents still need direct review): https://resources.lora-alliance.org/lora-alliance-news/new-lora-alliance-standards-make-lorawan-deployments-faster-simpler-and-more-scalable
- 3GPP non-terrestrial networks overview: https://www.3gpp.org/technologies/ntn-overview
- IETF QUIC RFC 9000: https://www.rfc-editor.org/rfc/rfc9000
- IETF Bundle Protocol v7 RFC 9171 (check updates and errata): https://www.rfc-editor.org/rfc/rfc9171
- IETF RFC 9713, Bundle Protocol Version 7 Administrative Record Types Registry (updates RFC 9171): https://www.rfc-editor.org/info/rfc9713
- IETF RFC 9758, Updates to the 'ipn' URI Scheme (updates RFC 9171 endpoint-identifier handling): https://www.rfc-editor.org/info/rfc9758
- IETF Bundle Protocol Security RFC 9172: https://www.rfc-editor.org/rfc/rfc9172
- IETF Babel RFC 8966: https://www.rfc-editor.org/rfc/rfc8966
- IETF OLSRv2 RFC 7181: https://www.rfc-editor.org/rfc/rfc7181
- ETSI wireless/mobile technologies (including TETRA): https://www.etsi.org/technologies/wireless-mobile-technologies/
- CCSDS active publications and Blue Books: https://ccsds.org/publications/allpubs/
- Holochain 0.7 migration and transport changes: https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/
- Holochain 0.7 compatibility table: https://developer.holochain.org/resources/compatibility/holochain-0.7/

## 11. Explicit non-claims

This RFC does not establish that Pulse presently supports any new radio, that a given phone can transmit on arbitrary bands, that all standards or revisions have been exhaustively inventoried, that standards knowledge alone confers interoperability, or that a simulated result proves real-world delivery. It proposes an incremental route to measurable interoperability while preserving Pulse's cryptographic and evidence boundaries.
