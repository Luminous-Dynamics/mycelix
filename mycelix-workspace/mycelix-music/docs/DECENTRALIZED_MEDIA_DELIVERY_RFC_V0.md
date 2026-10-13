# Mycelix Media Delivery Fabric — RFC v0.1

**Status:** Proposed architecture; not a deployed service  
**Reviewed:** 2026-10-13  
**Scope:** Active public repository, `Luminous-Dynamics/mycelix`  
**Primary implementation direction:** Rust-first control plane; interoperable HTTP delivery; optional IPFS and Iroh peer paths  
**Decision requested:** Accept the architectural boundaries and phase gates before implementing a delivery service or paying providers.

## 1. Executive decision

Build a **hybrid, progressively decentralized media delivery fabric**, not a replacement for every CDN component on day one.

Use each system for the job it is structurally suited to:

- **Mycelix / Holochain:** authenticated publication and rights records, signed provider-capability/availability claims, moderation and dispute state, and auditable references to delivery evidence. Store metadata and bounded evidence records—not video/audio bytes—in the DHT.
- **Media knowledge graph:** a separate semantic index of relationships between works, versions, creators, licenses, sources, citations, captions, transcripts, and derivatives. It improves discovery and provenance; it is not itself a byte-transfer protocol or availability guarantee.
- **Mycelix DKG (distributed key generation):** a distinct cryptographic primitive. The repository currently refers to DKG in the context of governance threshold signing. It may support jointly controlled signatures or authorization attestations for gated media if a dedicated protocol is designed and qualified; do not assume it provides threshold decryption, provider discovery, content verification, or byte delivery.
- **IPFS/IPLD:** interoperable content addressing, block exchange, content discovery, archival replication, and HTTP gateway compatibility.
- **Iroh:** an optional Rust-native peer transport for direct, resumable and cryptographically verified blob/range delivery, particularly for peers and managed cache nodes that can run an Iroh endpoint.
- **HTTP caches and conventional CDN edges:** reliable, cacheable delivery to ordinary browsers and devices, including the fallback path. This is a compatibility feature and an operating tool, not a philosophical failure.
- **HLS/CMAF:** initial on-demand playback baseline with adaptive renditions and broad player compatibility.
- **Media over QUIC (MoQ):** a separately qualified experiment for live or low-latency media. Do not make it the only playback path until the chosen client/server profile and browser support pass the compatibility matrix.

The target is not “no servers.” The target is **no single required delivery operator**, transparent and replaceable providers, verifiable immutable content, and several independently operated ways to retrieve the same authorized asset.

A content hash establishes byte identity under its hash/profile; it does not establish that content is licensed, safe, authentic, currently available, or watched by a human.

## 2. Repository baseline and current gap

This RFC is grounded in the active public repository's current source paths:

- `mycelix-workspace/mycelix-music/dnas/mycelix-music/zomes/catalog/integrity/src/lib.rs` models songs using an IPFS CID.
- `mycelix-workspace/mycelix-music/apps/leptos/src/types.rs` currently turns that CID into a single hard-coded `https://ipfs.io/ipfs/{cid}` URL in `Song::audio_url()`.
- `mycelix-workspace/mycelix-music/apps/leptos/src/components/player.rs` drives browser audio playback from that URL.
- `mycelix-civic/zomes/media-publication/integrity/src/lib.rs` already models media types, content hashes, authorship claims, licenses, publication versions, and blocks.
- `mycelix-civic/apps/leptos/src/main.rs` still mounts the component showcase rather than a real media application.
- `mycelix-workspace/mycelix-music/IMPLEMENTATION_STATUS.md` describes the delivery/deployment path as not yet production-verified and explicitly calls for end-to-end verification and a canonical runtime.

Relevant code links:

- [Music catalog types and URL construction](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/mycelix-music/apps/leptos/src/types.rs)
- [Music player](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/mycelix-music/apps/leptos/src/components/player.rs)
- [Music catalog integrity](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/mycelix-music/dnas/mycelix-music/zomes/catalog/integrity/src/lib.rs)
- [Civic publication integrity](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-civic/zomes/media-publication/integrity/src/lib.rs)
- [Music implementation status](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/mycelix-music/IMPLEMENTATION_STATUS.md)

The first concrete code-level seam is to replace single-gateway URL construction with a versioned asset manifest and a delivery resolver that can return multiple *typed* retrieval candidates plus their integrity and access guarantees. Do not scatter fallback URL concatenation across the UI.

This RFC does not claim that an Iroh provider, an IPFS pinning fleet, a distributed edge network, or production CDN exists in the current source tree.

## 3. Why a hybrid is the right architecture

### IPFS: portable content addressing and an interoperable network

IPFS provides CIDs, IPLD/UnixFS block graphs, provider discovery and block transfer. Its HTTP gateways make content-addressed data usable by clients that cannot speak IPFS natively. Gateways can be recursive or local-only; pinning improves persistence but must be operated and monitored.

Use IPFS for:
- stable content identifiers and portable asset references;
- optional public distribution and independently run replicas;
- gateway retrieval and migration between providers;
- verifiable raw-block/CAR retrieval where the client validates the response;
- cold-cache and archival replication paths.

Do not assume that a CID makes the bytes available. A CID does not pin its content, keep a node online, guarantee bandwidth, or certify rights. Public-gateway availability and latency vary.

**Gateway selection and privacy:** the IPFS Standards document [User-Preferred Gateway Detection (2026-08-07)](https://specs.ipfs.tech/http-gateways/gateway-detection/) says applications SHOULD NOT silently default to hard-coded public gateways. Public gateways can observe requester network metadata and the requested CID/path. The web client cannot reliably read a user's local IPFS environment variable or gateway file, so the deployment must provide an explicit ordered pool or the user-facing application must obtain a gateway choice. The current implementation uses build-time `MYCELIX_MEDIA_GATEWAY_ORIGINS` with a maximum of eight HTTPS origins; an unset pool produces no candidate, and public gateways are included only if deliberately listed. Before a public release, add a visible gateway preference/consent flow and explain the metadata disclosed to each selected operator. Do not conflate choosing a gateway with verifying the bytes it returns.

Sources: [How IPFS works](https://docs.ipfs.tech/concepts/how-ipfs-works/), [IPFS gateways](https://docs.ipfs.tech/concepts/ipfs-gateway/), [IPFS lifecycle and pinning](https://docs.ipfs.tech/concepts/lifecycle/), [gateway best practices](https://docs.ipfs.tech/how-to/gateway-best-practices/), [trustless gateway specification](https://specs.ipfs.tech/http-gateways/trustless-gateway/).

### Iroh: verified peer transport, not a magic global CDN

Iroh is attractive because it is Rust-oriented and supports BLAKE3-addressed blobs, resumable transfer, range retrieval, and verified streaming over its peer transport. It fits opt-in user peers, creator-origin nodes, and operated cache peers.

However, global content discovery is a distinct problem from transferring a blob once a provider is known. Iroh's September 30, 2026 global-discovery write-up describes a working experimental design using BitTorrent Mainline plus an address-index service, and explicitly describes the workflow as an experimental repository. Treat that discovery path as an experiment to evaluate, not a settled production dependency.

Use Iroh first for:
- native peer-to-peer retrieval from a known/advertised provider;
- prefetch, resumable segment retrieval, and verified range downloads;
- controlled federation or creator/organization-owned nodes;
- optional cache-to-cache transfer.

Do not initially make successful playback depend on global Iroh discovery. Keep a standard HTTPS route available.

**Discovery privacy gate:** the experimental Mainline-based discovery design is not an anonymity layer. Provider announcements can expose public endpoint addresses; unencrypted DHT traffic may expose lookup metadata to network observers. Do not enable this discovery path by default for listeners or providers, and do not use it as the sole path for private/gated assets. M2 must document which endpoint metadata is public, require explicit operator opt-in, measure what client and provider network metadata is exposed, and compare a known-peer/private-tracker path plus suitable relay or privacy-preserving alternatives before any public rollout. Do not claim that transport encryption alone hides discovery lookups or provider IP addresses.

**Security gate:** Iroh published an `iroh-blobs` security fix on 2026-10-08. Its release note says versions 0.94.0 through 0.103.0 were affected by a request-mask bug that could permit unauthorized blob pushes and, in certain configurations, let remote peers fill a provider's disk. The note identifies 0.103.1 as the fixed 0.103-series release and corresponding fixed versions for several earlier series. Before deploying any Iroh provider, select a vendor-confirmed patched version, lock it, explicitly disable `push` unless an authenticated use case needs it, and separately enforce authorization for `get`, `get_many`, `observe`, and `push`. Add disk quotas and adversarial regression tests. Never assume that setting one read gate automatically secures every request type.

Sources: [Iroh global content discovery, 2026-09-30](https://www.iroh.computer/blog/iroh-global-content-discovery), [iroh-blobs security fix, 2026-10-08](https://www.iroh.computer/blog/iroh-blobs-push-fix), [Iroh blob-store design](https://www.iroh.computer/blog/blob-store-design-challenges).

### Holochain: the control plane, not the media data plane

The Holochain DHT is appropriate for validated application records shared by a DNA's peer network. It should hold compact, validated records and references, not the payload of multi-gigabyte videos or frequently requested segments.

Use Mycelix records for:
- the canonical publication-to-manifest binding;
- rights/license/authorization references and dispute state;
- expiring provider advertisements and capability claims;
- replication leases and bounded delivery evidence;
- removal, suspension, and policy state;
- creator-consented analytics and settlement references.

The DHT is not a globally complete public index of all IPFS/Iroh providers. Its discovery scope is the relevant Holochain network, and gossip/validation are not equivalent to a globally provisioned CDN control plane.

Source: [Holochain DHT concepts](https://developer.holochain.org/concepts/4_dht/). Holochain's data guidance also recommends storing large objects outside the DHT where appropriate: [Working with data](https://developer.holochain.org/build/working-with-data/), [CRUD actions and storage considerations](https://developer.holochain.org/concepts/6_crud_actions/).

## 4. Proposed architecture

Separate the fabric into a control plane and data plane. The layers can be operated by different organizations.

```text
                      Mycelix Media clients
               Leptos web / mobile / desktop players
                              |
                   Asset manifest + policy resolver
                              |
              +---------------+----------------+
              |                                |
       Control plane                      Data plane
  Mycelix / Holochain / media graph  Immutable media bytes
  publication, rights, provenance     HLS/CMAF segments + images
  provider leases, moderation         captions, transcripts, covers
  optional DKG authorization
              |                                |
       Candidate providers        +------------+-----------------+
       and access rules           |            |                 |
              |                HTTP edge     IPFS nodes       Iroh peers
              |                caches/CDN    gateways/pins     QUIC blobs
              +--------------------+------------+-----------------+
                                   |
                      durable origin / creator copy
```

### 4.1 Control plane records

Keep long-lived content identity separate from short-lived provider locations.

**`MediaAssetManifestV1`** should bind:
- schema/profile version and canonical asset ID;
- Mycelix publication reference and content type;
- one or more rendition/track descriptions (codec, dimensions/audio properties, duration, language, media type);
- immutable root references and the exact integrity profile used for each storage format;
- HLS/CMAF playlist/segment references where applicable;
- captions, transcripts, cover art, and derivative references;
- rights/license and access-policy references;
- parent/derived-from lineage for edits, translations, remixes, and alternate encodings;
- signer/author identity, version, and the exact signed bytes.

Do not embed changing provider IP addresses in the immutable manifest. That harms cacheability and creates stale manifests.

**`MediaProviderLeaseV1`** should be separately signed and expire. It should identify:
- the exact asset/renditions/ranges the provider claims to serve;
- a stable provider identity/endpoint commitment;
- supported protocols and integrity profiles;
- a lease expiry and renewal sequence;
- coarse region or network class only when useful, accurate, and privacy-safe;
- capacity/replication-policy reference and measured evidence references, if any.

A lease is a claim, not proof of current availability. Availability is established only by fresh probes and successful retrievals. Provider reputation must never replace cryptographic content verification.

### 4.2 Hash formats and manifest identity

IPFS CIDs and Iroh BLAKE3 hashes are not interchangeable strings. One may identify a block/DAG while another may identify a complete blob byte sequence. Do not convert or compare them as if they necessarily represent the same byte domain.

The manifest must explicitly bind each storage representation to:
- the exact bytes or block/DAG it identifies;
- its algorithm and encoding;
- expected byte length;
- canonical relationship to the logical asset and rendition;
- verification method and verification result.

When an IPFS representation and an Iroh blob contain the same bytes, compute and record both identifiers under their own formats and verify that both resolve to the declared byte sequence. Do not introduce a custom hash bridge without golden vectors and independent tests.

### 4.3 Retrieval and fallback algorithm

The client/resolver should:

1. Load the canonical publication and manifest reference from Mycelix and validate the signature, schema, version, rights/access state, and supported rendition.
2. Check the local verified cache.
3. Collect fresh candidate providers from applicable sources: local/nearby peers, unexpired Mycelix provider leases, IPFS provider routing, known Iroh endpoints, operated edge caches, and HTTPS origins/gateways.
4. Filter candidates for access policy, expected protocol/integrity support, lease freshness, size/range support, quotas, and client compatibility.
5. Order candidates using measured success rate, recent latency/throughput, region preference, privacy requirements, and cost limits. Do not use a single global reputation number to decide truth or integrity.
6. Retrieve from one provider or race a bounded small number for cold-start latency. Bound concurrent downloads, retries, connection attempts, and bytes.
7. Verify content at the strongest integrity layer supported by the route. Reject mismatched content, record the failure safely, and try another provider.
8. Populate only caches allowed by the asset's rights/access policy.
9. On total failure, expose a useful recoverable playback error instead of silently reporting a play as successful.

A lower-integrity fallback must not be described as equally verified. A browser's native `<audio>`/`<video>` element reading a normal HTTPS URL does not independently prove that the response matches the CID or BLAKE3 digest in Mycelix. Use an integrity-capable client/segment loader where independent end-to-end verification is required, or explicitly label the route as HTTPS/TLS-delivered rather than client-verified.

Do not append `?format=car` to the current native `<audio>` URL and treat the result as verified streaming: a CAR response is a structured block container, not a directly playable media file. An integrity-capable path must parse the CAR, validate each addressed block and the manifest's declared byte domain, reconstruct the requested file or segment, and only then pass verified bytes to a player-compatible source. Keep memory and buffering bounded; do not buffer an unbounded whole movie to prove one segment. If the browser path cannot perform this verification, it must continue to report `HttpsTransportOnly`.

### 4.4 Playback protocols

**Phase 1 — VOD baseline:** HLS with CMAF-compatible segments and multiple renditions. Give immutable segment objects immutable URLs so HTTP caches can retain them for a long time. Version or content-address playlists; give mutable rights/availability state a separate, shorter-lived path. Add seek, retry, resume, captions, and rendition fallback tests.

**Phase 2 — optional P2P acceleration:** let native clients and managed peers prefetch or retrieve verified media segments through Iroh; use IPFS providers/gateways as another content-addressed path. The browser stays functional if no peer path is reachable. Peer uploads must be opt-in and rights-aware.

**Phase 3 — live/low latency:** evaluate MoQ over QUIC/Iroh against HLS/LL-HLS using actual clients, relay topologies, packet loss, congestion, NATs, and expected concurrency. Pin a specific protocol/profile and versions. MoQ is an evolving standards and implementation ecosystem; do not treat a demo or vendor scaling claim as independent verification of Mycelix's own operating requirements.

Sources: [Iroh video streaming overview](https://www.iroh.computer/solutions/video), [Iroh's 2026 secure-video prototype write-up](https://www.iroh.computer/blog/secure-video-everywhere), [MoQ protocol overview](https://doc.moq.dev/), [MoQ transport concepts](https://doc.moq.dev/concept/transport), [MoQ standard/profile status](https://doc.moq.dev/concept/standard).

### 4.5 Caching and replication policy

- **Public immutable segments and images:** content-addressed URLs; long-lived cache headers; verify on ingestion and on any peer retrieval.
- **Playlists and manifests:** immutable/versioned where possible; no cache key that can mix authorization contexts.
- **Mutable publication, moderation, rights, and availability state:** fresh policy check or a deliberately bounded cache TTL, with explicit stale-state behavior.
- **Private/gated media:** encrypt payloads before distribution; keep decryption keys out of public DHT entries and public provider announcements; authorize key release separately. A user who already received plaintext or a decryption key cannot be made to “unsee” it; revocation can block future key release and first-party distribution, not erase every existing copy.
- **Durability:** creators should keep a durable origin copy. Public works can enroll in an explicit replication policy across independently operated failure domains. Do not claim “permanent,” “always available,” or “decentralized” based only on a CID, a successful pin request, or an untested peer count.
- **Backpressure:** provider nodes need storage quotas, eviction policy, bandwidth ceilings, per-principal rate limits, and overload rejection.
- **Legal/creator policy:** support takedown and dispute workflows. Content-addressing does not supersede copyright, consent, safety, or a user's control over first-party publication. Nodes must be able to stop advertising/serving a work they are not permitted to distribute.

## 5. Where the media knowledge graph, Mycelix DKG, and Symthaea help

Treat the **media knowledge graph** as a graph of evidence-backed relationships, not as a magical availability oracle. Candidate relationships include:
- publication → asset manifest → rendition → segment;
- work → source/citation → claim → fact-check/dispute;
- original → edit/translation/caption/remix;
- creator → collaborator → rights claim → license terms;
- asset → provider lease → fresh successful retrieval observation.

Graph traversal and semantic search can power “find this moment,” “show the source,” “find alternate-language versions,” “show authorized derivatives,” and “discover creators discussing this topic.”

**Mycelix DKG is distributed key generation, not the media knowledge graph.** The current repository describes DKG in the governance/threshold-signing context ([workspace overview](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/README.md)). It may support threshold signatures or jointly controlled authorization evidence for gated media, but that is not implemented by this delivery RFC. A future encrypted-media key-release design would need its own protocol, authorization and revocation semantics, client integration, threat model, and independent tests. Do not publish secret shares or decryption keys in public DHT records, and do not assume threshold signing automatically provides threshold decryption.

Symthaea can propose semantic matches, segment summaries, transcript indexes, and user-tunable recommendations. The UI must distinguish model-generated analysis from signed evidence and canonical facts.

Do not use a Symthaea embedding or recommendation result to:
- certify file integrity or provider availability;
- establish authorship or legal rights;
- override privacy/entitlement policy;
- create a payable play or settle money;
- turn a fact-check candidate into an authoritative verdict.

Keep raw viewing histories private by default. Shared delivery metrics should be aggregated or pseudonymized; provider routing should not require creating a globally public graph of every user's watching behavior.

## 6. Abuse and threat model

Required boundaries:

- **Malicious provider returns wrong bytes:** content hash/manifest verification rejects it; no provider reputation override.
- **Provider lies about availability:** leases expire; probes and real retrievals update a bounded freshness score; failover tries another candidate.
- **Iroh storage-fill attack:** patched/locked dependencies, push disabled by default, independent authorization per request type, quotas, rate limits, and a regression test that unsolicited push fails.
- **DHT/provider poisoning or Sybil providers:** treat discovery as untrusted hints; verify the endpoint/content; cap races; never trust a listing as proof.
- **DDoS or origin-cost attack:** authenticated upload; bounded cache misses; origin shielding; per-principal quotas; egress budgets; request de-duplication; safe backpressure.
- **Private metadata leakage:** minimize public metadata; protect view histories, exact locations, and entitlement details; use short leases and opaque references.
- **Rights/retention conflict:** explicit legal/creator state, suspension and removal from first-party routing; disclose that already-replicated public bytes cannot be remotely erased reliably.
- **Provider compromise or stale authorization:** separate immutable content identity from mutable authorization; recheck policy at bounded intervals and before private-key release.
- **Fake playback/royalty farming:** playback telemetry is not itself proof of a human listen; settlement requires a separately qualified, replay-resistant eligibility pipeline and canonical rights/allocation policies.

## 7. Implementation plan and release gates

### M0 — Replace the hard-coded gateway seam (first implementation)

Add a small Rust domain module for asset identities, manifests, typed retrieval candidates, integrity profiles, and resolver outcomes. Preserve backward compatibility with existing catalog entries by treating the current IPFS CID as one content address, not the whole delivery contract. Resolve only through an explicit, ordered HTTPS origin pool (`MYCELIX_MEDIA_GATEWAY_ORIGINS`, at most eight entries); do not add implicit public-gateway defaults. A deployment may explicitly list public gateway origins after accepting their privacy trade-offs. An empty pool must return a visible no-provider result, not silently contact a third party.

Acceptance:
- no public gateway is contacted unless its origin is explicitly configured; no implicit `ipfs.io` or `dweb.link` fallback exists;
- pure tests cover empty/malformed identifiers, unsupported schemes, candidate order, empty/duplicate/oversized origin pools, access denial, timeout, and no-provider outcomes;
- fixture/mock transport remains explicit and cannot masquerade as production connectivity;
- the same asset can have an IPFS path and an HTTPS mirror without changing logical identity;
- standard playback keeps working if Iroh or peer discovery is unavailable.

### M1 — Real multi-origin VOD

Implement an ingestion/asset-manifest flow and one HTTP cache/origin plus one IPFS provider path. Measure:
- time to first frame and startup failures;
- cold vs. warm cache p50/p95;
- seek and rendition-switch latency;
- buffer/stall ratio and rebuffer count;
- cache-hit ratio, origin bytes avoided, egress cost per GiB;
- provider failure and recovery time;
- integrity mismatches, which must be rejected.

For repeatability, freeze a synthetic corpus and exact client/network profiles. Include low-bandwidth, latency, packet-loss, peer-offline, slow-origin, corrupted-segment, expired-lease, and cache-miss cases. Publish measured baseline and exact test environment before asserting improvement.

### M2 — Iroh peer path

Only after reviewing the current Rust dependency tree and the security fix, add a feature-gated Iroh provider/client. Begin with controlled providers and known endpoint identities; do not require the experimental global discovery service for successful playback.

Acceptance:
- dependency lock records a vendor-confirmed fixed `iroh-blobs` release;
- unauthorized push is rejected and consumes no unbounded disk;
- `get`, `get_many`, and `observe` each have independent authorization tests;
- interrupted transfer resumes correctly and verified data matches the manifest;
- peer unavailability always falls back; bounded retry does not stall the player indefinitely.

### M3 — Independent operator mesh

Add opt-in cache nodes, signed short-lived provider leases, probes, placement/replication policy, operator quotas, and minimal verifiable delivery receipts. Start with unpaid/explicitly opted-in nodes or contracted operators; do not mint a token or promise “earnings” until the accounting semantics are qualified.

Acceptance:
- loss of any one provider does not make a replicated public test asset unavailable when another healthy copy exists;
- stale leases cannot remain the only path selected;
- replication claims have probe evidence and clear expiry;
- no network receipt is treated as proof of human attention;
- privacy, takedown, incident response, and operator exit are documented.

### M4 — MoQ experiment for live media

Maintain HLS fallback. Benchmark a frozen MoQ/Iroh profile against HLS/LL-HLS in the target browsers and native clients. Include NAT traversal, relay loss, congestion, reconnect, fan-out topology, authorization, and abuse tests. Ship only the client/profile combinations actually qualified.

## 8. Required conformance corpus

Before this is considered an operational media fabric, freeze and test these controls:

1. Identical immutable bytes resolve through IPFS, HTTPS cache, and Iroh; each route reports its actual verification level.
2. A single byte mutation fails verification; no corrupted media is played or cached as verified.
3. A CID/manifest mismatch, wrong asset length, invalid codec, or missing rendition fails closed with a recoverable user-facing error.
4. An expired lease is rejected; one live alternate provider succeeds.
5. No providers, DNS failure, gateway timeout, Iroh unreachable, and origin unavailable all produce bounded retry and a clear final result.
6. Repeated resolver calls do not mutate Holochain merely to compute an address or query metadata.
7. Unauthorized access cannot retrieve a private manifest, decryption key, or private segment via a different provider protocol.
8. Public-cache entries cannot cross authorization contexts.
9. Revocation prevents new first-party advertisements and future gated-key access; the UI does not falsely claim previously downloaded plaintext was erased.
10. A malicious provider's reputation cannot make bad bytes valid.
11. A provider cannot fill storage with unauthorized Iroh push requests; `get_many` and `observe` do not bypass the access policy.
12. Cache eviction, process restart, and interrupted transfers preserve manifest identity and resumability.
13. Provider meters distinguish bytes sent, bytes verified, bytes requested, and successful playback signals; none alone establishes human viewing.
14. Settlement retries are idempotent and cannot create a second payable obligation for one eligible play.
15. Public or private status changes are reflected according to a stated cache freshness bound.
16. Logs, routing records, and benchmark exports do not expose raw viewing history or precise user location.
17. Results are bound to exact source commit, manifest corpus, dependency lock, configuration, and test environment. Queued/skipped tests are not PASS.

## 9. Decision summary

Proceed with a hybrid architecture:
1. remove the one-gateway assumption through a Rust asset-manifest/resolver boundary;
2. keep Holochain as the validated metadata/policy layer and keep large bytes out of its DHT;
3. support HTTP/CDN delivery first for dependable browser playback;
4. add IPFS as an interoperable content-addressed provider/replication path;
5. add Iroh as an optional verified peer path after dependency/security review;
6. use the media knowledge graph for provenance, rights, and discovery; treat Mycelix DKG (distributed key generation/threshold signing) as a separate cryptographic primitive, not as the graph or delivery protocol;
7. evaluate MoQ separately for live/low-latency media;
8. do not advertise CDN reliability, creator earnings, or decentralized availability until the relevant exact-head tests and operational evidence exist.

## External references

- IPFS: [How IPFS works](https://docs.ipfs.tech/concepts/how-ipfs-works/), [Gateway concepts](https://docs.ipfs.tech/concepts/ipfs-gateway/), [Lifecycle/pinning](https://docs.ipfs.tech/concepts/lifecycle/), [Gateway best practices](https://docs.ipfs.tech/how-to/gateway-best-practices/), [Trustless Gateway specification](https://specs.ipfs.tech/http-gateways/trustless-gateway/).
- Holochain: [DHT](https://developer.holochain.org/concepts/4_dht/), [Working with data](https://developer.holochain.org/build/working-with-data/), [CRUD and storage guidance](https://developer.holochain.org/concepts/6_crud_actions/).
- Iroh: [Global content discovery (2026-09-30)](https://www.iroh.computer/blog/iroh-global-content-discovery), [iroh-blobs security fix (2026-10-08)](https://www.iroh.computer/blog/iroh-blobs-push-fix), [blob-store design](https://www.iroh.computer/blog/blob-store-design-challenges), [video solution overview](https://www.iroh.computer/solutions/video).
- Media over QUIC: [project docs](https://doc.moq.dev/), [transport concepts](https://doc.moq.dev/concept/transport), [protocol/profile status](https://doc.moq.dev/concept/standard).
