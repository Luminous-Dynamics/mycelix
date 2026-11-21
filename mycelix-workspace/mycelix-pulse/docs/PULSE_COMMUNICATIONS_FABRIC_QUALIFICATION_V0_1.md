# Pulse Communications Fabric — Qualification Profile v0.1

- **Workstream:** PULSE-FABRIC-QUAL-001 ([issue #6376](https://github.com/Luminous-Dynamics/mycelix/issues/6376))
- **Status:** Research and qualification contract; not a production implementation or security certification
- **Profile date:** 2026-11-20
- **Public source baseline:** main at `b42b5af299d575864ab3649ceaad8c8e963020ec`
- **Owner:** Mycelix Pulse
- **Scope:** Cross-layer message lifecycle, cryptographic/session profiles, authority, delivery, privacy, recovery, observability and independent evidence

## 1. Executive thesis

Pulse should optimize for a **logical communication that reaches the intended recipient, can be authenticated and recovered, and has an honest user-visible outcome**—not for socket writes, transport acknowledgements, raw throughput or feature count.

The product opportunity is to combine:

1. Mature, independently reviewed cryptographic protocols rather than inventing a new ratchet.
2. Local-first data custody and independently operated communication infrastructure.
3. Host-enforced, precisely scoped authority and revocation.
4. Durable, idempotent message operations across retries, paths, crashes and partitions.
5. Privacy minimization across content, routing, identity, timing and social metadata.
6. Evidence that allows a separate verifier to reproduce why the UI says sent, delivered, read, revoked, recovered or unknown.

The distinctive proposition is **evidence-carrying sovereign communications**. Pulse should not claim to outperform Signal, Matrix or SimpleX in general. It should identify specific properties, build matched baselines, and show improvements with reproducible evidence.

## 2. Current implementation boundary

The public mainline is still a working alpha:

- The canonical product path is the Leptos browser application and Holochain messages DNA.
- The Pulse Holochain workspace currently pins HDK 0.6.1 / HDI 0.7.1; current build recipes still pin the 0.6.1 CLI.
- The documented V2 envelope uses X25519 + ML-KEM-768, HKDF-SHA256, AES-256-GCM, Holochain agent signatures and application-level ML-DSA-65 signatures.
- V2 is a one-message hybrid envelope. It does **not** provide a PQ ratchet, forward secrecy against later compromise of the static recipient key, or post-compromise recovery.
- Key recovery is currently limited to the same browser profile. There is no portable key recovery in the supported alpha claim.
- The public V2 DHT data model exposes message-routing and relationship metadata; encrypted content must not be confused with metadata anonymity.
- The current capability coordinator has a source-reviewed host-grant retirement gap: the application record is updated to revoked, while the source comment says the actual Holochain grant is not deleted because its ActionHash is not tracked. This is a source finding, not a demonstrated runtime exploit; exact 0.7.1 host enforcement must be tested.
- Holochain 0.7.1 is the selected migration target, but Pulse runtime qualification is not complete. The 0.7.0 migration draft is not authority to claim a 0.7.1 migration.

These boundaries are intentional inputs to the profile. Do not silently promote experimental or documented future features into current capabilities.

Primary local sources:
- [Pulse README](../README.md)
- [V2 crypto specification](PULSE_V2_CRYPTO_SPEC.md)
- [ADR-002 working-alpha contracts](adr/ADR-002-working-alpha-contracts.md)
- [Surface status](SURFACE_STATUS.md)
- [Migration qualification issue #5719](https://github.com/Luminous-Dynamics/mycelix/issues/5719)
- [Capability lifecycle issue #5772](https://github.com/Luminous-Dynamics/mycelix/issues/5772)

## 3. Governing theorem: UI state is a derived claim

A user-visible state is not authority and is not proof merely because one component emitted it.

~~~text
intent + explicit consent
  -> authenticated user/device/session state
  -> encrypted logical message + stable message identity
  -> exact authority admission
  -> durable local outbox intent
  -> zero or more transport attempts
  -> recipient-side admission and durable inbox commit
  -> recipient decryption and cryptographic verification
  -> receipt creation (where the profile supports it)
  -> independent receipt verification
  -> caller reconciliation
  -> UI projection
~~~

Each arrow is a distinct event. A transport ACK, Holochain remote signal, DHT link, application record, trust score, or cached UI state cannot independently establish that a recipient has retrieved, decrypted or accepted a message.

The evidence graph is a **partial order**. Causal predecessors, ActionHash lineage and per-source-chain frontiers establish order. Do not compare wall clocks across conductors to invent causality; do not collapse independent Holochain chains into one scalar sequence. Missing, stale, conflicting or unverifiable evidence remains UNKNOWN or INCOMPLETE.

The v0.1 trace schema represents each causal predecessor as an event reference plus a typed relation (for example, local program order, source-chain successor, request/response, transport-for, commits, receipt-for, verifies, revokes, checkpoint-descendant-of or recovered-from). The independent verifier must resolve both endpoints, check the relation against the evidence roles and exact identities/lineage, and reject cycles or unsupported edges. JSON Schema can check shape; it cannot prove that a claimed causal edge is true.

The logical operation ID is stable across retries and routes. The attempt ID is null only for operation-scoped events that occur before an attempt exists (for example, recording durable intent); execution, transport, recipient and receipt events require a concrete attempt ID. An attempt is not a new logical message.

## 4. Architecture direction

### 4.1 Cryptographic protocol adapters, not bespoke cryptography

Keep the current V2 envelope as an explicitly versioned historical/current-alpha profile. Evaluate successor profiles behind a strict protocol registry and no-downgrade rules.

**One-to-one sessions:** compare the current V2 one-shot envelope with mature asynchronous session protocols. Signal's PQXDH is a useful initial-key-establishment reference; Signal's ML-KEM Braid / SPQR is a sparse continuous key-agreement design intended to feed higher-level ratchets with successive post-quantum secrets. The Triple Ratchet direction is a stronger comparator for ongoing key evolution. Reuse mature implementation work only after license, API, platform, side-channel, state persistence and independent review checks. Do not independently invent a ratchet.

**Groups:** evaluate MLS (RFC 9420) as the group-session architecture. The ML-KEM and hybrid MLS cipher-suite draft -08, published 18 November 2026 and sent to IESG Last Call on 19 November, is a timely research input—not a final standard. Track its finalization, interop vectors and independent implementations before treating it as a deployable baseline.

**Crypto agility:** a profile identifier must bind KEMs, signatures, AEAD, KDF, transcript version, ratchet/session semantics, identity binding, device model, recovery model and downgrade policy. An algorithm name alone is not a security claim. Keep test vectors and canonical transcripts versioned. An unknown or unqualified profile fails closed.

### 4.2 Authority is an operation lifecycle

Separate:
- Holochain host decision for the exact cell, grantee, capability and zome/function.
- Application/domain authorization for the exact resource transition.
- Authored source-chain action.
- Independent integrity validation.
- DHT observer projection and freshness.
- Caller acknowledgement and observed outcome.

A grant record is not a host grant. A stale DHT projection is not current permission. A checkpoint is a read-freshness constraint, never a bearer token. Track exact grant identity and make revocation target the exact host grant. Distinguish a new call after revocation from a call admitted before revocation but committed afterward; the latter must preserve the actual admission/commit ordering instead of asserting an invented cancellation guarantee.

### 4.3 Delivery is a durable logical operation

Use stable logical message and recipient identities across routes, attempts and process restarts. Keep transport attempt IDs separate from logical operation IDs. Persist outbox intent before sending; persist recipient inbox deduplication with application acceptance; reconcile ambiguous results before retrying. A retry must not create a second logical message or receipt. A gateway or transport ACK is not a delivery receipt.

The operation state machine must preserve at least:
- queued / durable locally;
- attempted;
- definitely not sent;
- possibly sent / outcome unknown;
- recipient admitted;
- recipient committed;
- receipt observed;
- receipt independently verified;
- expired / revoked;
- terminal failure;
- incomplete / unknown.

No Boolean field named delivered may collapse these states.

### 4.4 Privacy is a data-model property

Inventory visibility and linkability separately for:
- plaintext and attachments;
- ciphertext size and timing;
- sender and recipient identifiers;
- public action authors and ActionHashes;
- mailbox links and tags;
- key-bundle references and rotation;
- thread/reply relationships;
- transport metadata, relay logs and IP information;
- device membership, recovery and diagnostics.

Use data minimization before adding encryption around metadata that should not have been collected. A public Holochain action still exposes its author; hiding a sender field inside an entry does not remove the public action-author relationship. Evaluate opaque, high-entropy mailbox references, relationship-scoped routing, relay/queue designs and traffic-analysis countermeasures as hypotheses. Do not claim anonymity based on pseudonymous identifiers alone.

### 4.5 Recovery is verified reconstruction, not database copying

Keep these data classes distinct:
- public Holochain source-chain history;
- encrypted message bodies and attachments;
- private application/session state;
- device identity and keys;
- host CapGrants and DirectSignal capabilities;
- recovery credentials and user-controlled backup material.

Holochain 0.7.1 restore-from-DHT restores public chain data with the original agent key; it does not restore private entries, local browser keys, CapSecrets or private app data. App-specific encrypted backup and session re-establishment are separate protocols. Restored public state must never recreate host authority. Reissue grants under the qualified runtime and require explicit device/session re-enrollment.

## 5. Industry comparison: what to learn and where to differentiate

| Reference | Strong established direction | Pulse implication |
|---|---|---|
| Signal | Mature 1:1 encrypted messaging; PQXDH initial exchange; ML-KEM Braid / SPQR and Triple Ratchet direction for ongoing PQ key evolution | Current Pulse V2 is behind on ratcheting and post-compromise recovery. Evaluate a mature implementation; do not claim parity yet. |
| MLS / RFC 9420 | Standardized asynchronous group key establishment with forward secrecy and post-compromise security | Candidate group-session architecture; track the November 2026 ML-KEM/hybrid cipher-suite draft separately from published standards. |
| Matrix | Open federation with Olm/Megolm E2EE and ongoing MLS integration work | Learn interoperable federation and server choice; qualify federation metadata and group-state semantics. |
| SimpleX | No global user accounts or persistent user identifiers; per-connection queues and data-minimized routing | Strong privacy reference; examine residual transport logs, queue abuse, availability and recovery trade-offs. |
| Holochain | Local-first source chains and DHT validation; capabilities are cell-scoped host authority | Keep user sovereignty and distributed validation, but qualify runtime version, public metadata exposure, restore and authority boundaries. |

These systems solve different problems. Signal's centralized service model, Matrix's server federation, SimpleX's queue routing and Holochain's app-specific DHT should not be ranked by a single throughput or "decentralization" score.

### Specific opportunity for Pulse

Compete on a composition that existing products do not expose as one user-verifiable contract:

1. Protocol-grade E2EE, using established and reviewed session/group protocols.
2. Precise, host-enforced resource authority with auditable revocation.
3. Durable multi-route delivery whose retries converge to one logical effect.
4. Privacy budgets for content, identity, relationship, routing and operational metadata.
5. Recovery that proves exactly which data and authority were reconstructed.
6. Independent evidence replay that explains every terminal UI claim.
7. Local-first operation with optional user/organization-operated relays and interoperability bridges.

This is a research proposition, not an achieved competitive advantage.

## 6. Qualification and benchmark design

### 6.1 Non-compensable safety gates

A single critical violation blocks the tested profile. Do not average safety failures into a quality score.

| Property | Qualification target |
|---|---|
| Delivery truth | Zero false Delivered claims across ACK-only, timeout, wrong recipient, altered commitment, replay, missing receipt and verifier failure |
| Logical idempotency | Zero duplicate user-visible inbox effects for one logical message/recipient across duplicate paths, concurrent attempts, restart and replay |
| Durability | Zero silent loss of an acknowledged outbox record within the declared storage durability model |
| Authorization | Zero cross-cell, cross-function, wrong-grantee, wrong-protocol-generation or revoked-grant host admissions in declared tests |
| Revocation | Zero new host admissions after the observed revoke boundary in non-overlapping schedules; in-flight calls preserve actual admission/commit order |
| Crypto | Zero silent downgrade, unqualified fallback, nonce reuse, transcript substitution or acceptance of invalid signatures |
| Recovery | Every case either reconstructs its declared data/key set or returns explicit partial/unrecoverable status; no false restoration claim |
| Telemetry | Zero plaintext, private keys, raw session secrets, CapSecrets or recovery material in benchmark traces |
| Verifier | Every named critical mutant is detected by its intended invariant; parser/configuration errors are not accepted as mutant detections |

### 6.2 Reproducible workload matrix

Use deterministic scenario IDs and frozen inputs. Initial dimensions:

- **Message payloads:** 1 KiB, 16 KiB, 256 KiB and 1 MiB; separately record attachment chunks.
- **Group sizes:** 2, 10, 100 and 1,000 participants where the candidate protocol supports them.
- **Network profiles:** RTT 20/100/500 ms; packet loss 0/1/5/20%; bandwidth constrained; interrupted and asymmetric links.
- **Availability:** online, receiver offline, sender restart, receiver restart, relay loss, peer churn, partition, delayed peer discovery and recovery after outage.
- **Common causes:** shared carrier, device, power, gateway, DNS/bootstrap, relay operator, geography and software dependency. Different adapter names do not prove independent failure domains.
- **Security faults:** stale/revoked capability, wrong cell/function, replay, concurrent grant issuance, key substitution, trust-root rollback, malformed receipt, DHT fork/split view and malicious coordinator.
- **Recovery faults:** no peers at start, partial public chain, missing private backup, corrupt ciphertext, stale device, revoked key and interrupted restore.

Record the actual fault injection and observed topology. Do not silently treat missing peers as independent failures or a temporarily empty view as a complete empty inbox.

### 6.3 Performance and resilience metrics

Report p50/p95/p99 and raw observations for:
- initial key establishment and per-message cryptographic work;
- ratchet/MLS update overhead and state growth;
- encrypt/decrypt and signature verification;
- bytes on wire and ciphertext expansion;
- time to recipient inbox commit;
- time to independently verified receipt;
- probability of verified recipient receipt before declared deadline;
- retry amplification and duplicate suppression;
- recovery time and recovered-state completeness;
- CPU, peak RSS, storage, disk writes, battery/energy and network bytes;
- per-relay/per-peer cost and fairness, not just aggregate system cost.

Measure each actor's monotonic durations in its own clock domain. Cross-actor ordering comes from authenticated causal edges, not wall-clock subtraction unless clock-offset uncertainty is measured and reported.

### 6.4 Performance improvement targets

Do not publish numeric SLOs before a reproducible Pulse baseline exists. After that baseline, preregister a holdout evaluation with these initial research targets:

- Healthy connected profile: no more than 2 percentage points loss in deadline-success rate versus the simplest eligible baseline.
- Correlated-outage profile: seek at least 20% relative improvement in verified-receipt-by-deadline versus a predeclared single-path baseline, with a paired confidence interval excluding zero.
- Energy/bytes: report the incremental cost of the resilience gain; a provisional stretch constraint is no more than 10% extra energy in the healthy profile.
- Correctness: zero release-blocking invariant violations regardless of latency or availability gains.

Targets are hypotheses. Freeze hardware, network profile, sample population, seed, cache state, protocol settings and comparison policy before each run. Report absolute percentage-point changes as well as relative changes.

### 6.5 Privacy evaluation

Generate synthetic ground truth for sender/recipient, groups, timing and threads. Give adversaries distinct observation vantage points: one DHT participant, one relay, two colluding relays, network observer and compromised endpoint. Measure:
- contact/linkability inference precision and recall;
- top-k recipient inference and anonymity-set distribution;
- thread and group-membership inference;
- timing/size correlation success;
- metadata retained per observer and retention duration;
- utility/delivery cost introduced by mitigation.

A lower inference score applies only to the tested adversary, dataset and observation vantage. Never convert "not inferred in this corpus" into an anonymity claim.

### 6.6 Usability and agency

Use preregistered, accessible user tasks to measure:
- first-message success without hidden demo fallback;
- contact/device verification comprehension;
- correct understanding of sent vs recipient-committed vs receipt-verified vs read;
- successful backup/recovery completion;
- comprehension of key-change and stale-device warnings;
- block/report and consent revocation completion;
- offline queue clarity and recovery after network return.

A user study is required before claiming that the evidence model is understandable or that safety behavior is usable.

## 7. Exact evidence and independent replay

The trace schema and starter scenarios shipped alongside this document are normative for this profile's v0.1 synthetic fixtures.

Every run binds:
- profile and schema versions;
- scenario ID, seed and frozen corpus digest;
- source commit and canonical-source/exporter lineage;
- Cargo, Nix and JavaScript lock digests;
- Rust, HDK, HDI, Holochain, client, Lair and Kitsune2 versions;
- WASM, DNA and hApp hashes;
- conductor, cell and peer references (redacted/pseudonymous as appropriate);
- protocol profile and crypto-library versions;
- topology, network/fault profile and host hardware;
- raw trace and artifact digests;
- verifier implementation/version and verdict;
- claim ceiling and explicit non-tested dimensions.

The verifier must be independent of the candidate's decision code. It checks schema, identity binding, digest binding, causal references, role separation, state transitions and terminal disposition. It must refuse an unsupported inference. Do not upload plaintext or publicize stable identity/relationship metadata merely to make a benchmark easy to inspect.

## 8. Group fan-out boundary

The group model is a parent operation with one stable child logical operation per
required recipient. A group receipt is not one event: a recipient-specific
receipt can support only that recipient's child. The parent must bind to one
frozen membership epoch and one exact required-recipient set; retries on
alternate paths reuse the child operation ID and allocate new attempt IDs.

A first independent implementation slice now exists in the verifier:

- A strict Draft 2020-12 parent-manifest schema commits to the group identity,
  membership epoch, membership snapshot digest, content commitment, explicit
  ALL_REQUIRED policy, and sorted recipient child set.
- Each child manifest item pins one recipient commitment, logical operation
  ID and SHA-256 digest of the exact child trace bytes.
- The trace schema binds each child to the same parent fanout, membership
  snapshot, epoch, recipient-set commitment, content commitment and its own
  opaque recipient commitment.
- The Rust verifier requires an externally supplied SHA-256 pin for the
  exact manifest bytes, verifies every supplied child trace against its
  manifest digest and recipient identity, rejects missing/extra/duplicate
  children, and runs the existing recipient-level causal verifier on each
  child.
- The CLI exposes this as verify-fanout. Unit and end-to-end CLI tests are
  authored for all-required completion, missing recipient children and
  wrong manifest pin.

The current structural verifier deliberately reports
ALL_REQUIRED_RECEIPT_CLAIMS_CONSISTENT when every manifest child has a
SUCCESS_VERIFIED trace claim with the required receipt-creation and
separately-attributed receipt-verification events. It does **not** report
COMPLETE_VERIFIED: it does not authenticate the manifest author, validate
receipt signatures, establish membership-snapshot truth, prove actor
independence, or verify trusted deadline compliance. The external manifest
digest binds exact bytes, but the mechanism that authorizes that digest is
outside this verifier. Those are mandatory future trust-layer requirements
before product UI or compliance systems may promote the aggregate to a
cryptographically verified group-completion claim.

The public manifest and trace must not contain a raw member roster. Use opaque,
scoped pseudonyms or keyed recipient commitments; do not publish bare hashes
of low-entropy identifiers. A commitment must be collision-resistant and
privacy-preserving under the actual observer threat model. Membership changes,
late join, member removal and per-device synchronization remain separate
qualification dimensions. An account-level child cannot imply all devices
received the message.

Tracked in [PULSE-GROUP-FANOUT-QUAL-001 / #6401](https://github.com/Luminous-Dynamics/mycelix/issues/6401), composed with
[PULSE-FABRIC-QUAL-001 / #6376](https://github.com/Luminous-Dynamics/mycelix/issues/6376).

## 9. Phased execution

| Phase | Work | Exit criterion |
|---|---|---|
| P0 | Freeze contract, trace schema and synthetic scenarios | Schema and corpus independently validate; all cases name expected disposition and invariant |
| P1 | Implement dependency-light independent replay verifier | Positive cases replay; every named mutation is caught for the intended reason |
| P2 | Add adapters for existing Pulse crypto, authority, outbox, inbox and receipt outputs | Adapters preserve source semantics and expose missing evidence as UNKNOWN |
| P3 | Qualify exact Holochain 0.7.1 migration and runtime | Canonical source parity, fresh conductor, DNA/hApp, signed host calls, multi-agent and independent client evidence |
| P4 | Comparative and adversarial performance campaign | Matched workloads, raw artefacts, confidence intervals and explicit comparator claim ceilings |
| P5 | Usability and privacy research | Preregistered tasks, independent adjudication and explicit limits |

CI queue state is not evidence. A skipped, queued, cancelled, stale-head or superseded run is not a pass. A synthetic fixture pass is not a real-conductor result.

## 10. Existing work to compose

This workstream coordinates rather than duplicates:
- #5110 / #5111 — industry/security/recovery benchmark.
- #5121 / #5112 — metadata privacy and privacy budget.
- #5122 — session ratchet, forward secrecy and post-compromise recovery.
- #5062 / #5135 / #5148 — durable delivery, idempotent receipt and truthful Delivered state.
- #5718 / #5719 — Holochain 0.7.1 migration profile and qualification.
- #5771 / #5772 / #5773 / #5917 / #5924 — authorization models, runtime histories, legacy capability-secret retirement, cell scope and revocation ordering.
- #5863 / #5942 — signed evidence and transparency anti-equivocation.
- #5905 — public source-chain restoration.
- #6337 — Wind Tunnel compatibility with the exact 0.7.1 stack.
- #6401 — group fan-out recipient ledger and aggregate group outcome qualification.

## 11. Claim ceiling

A passing profile validator establishes only manifest/schema correctness. A passing synthetic replay establishes only the encoded fixtures and independent verifier. A finite model check establishes only its bounded model. A real conductor run establishes only the exact source, package tuple, topology, fault schedule and scenarios. A performance result applies only to the named hardware/workload. None alone establishes universal Byzantine safety, anonymity, cryptographic correctness without independent review, production readiness or general superiority to an industry comparator.

## 12. Research references checked 20 November 2026

- [Signal PQXDH specification](https://signal.org/docs/specifications/pqxdh/)
- [Signal ML-KEM Braid specification](https://signal.org/docs/specifications/mlkembraid/)
- [Signal Protocol and Post-Quantum Ratchets](https://signal.org/blog/spqr/)
- [IETF RFC 9420 — Messaging Layer Security](https://www.rfc-editor.org/rfc/rfc9420.html)
- [IETF draft-ietf-mls-pq-ciphersuites-08](https://datatracker.ietf.org/doc/draft-ietf-mls-pq-ciphersuites/): published 18 November 2026; IESG Last Call announced 19 November 2026, due 3 December 2026; not yet an RFC.
- [Matrix E2EE implementation guide](https://matrix.org/docs/matrix-concepts/end-to-end-encryption/)
- [Matrix discussion of MLS integration](https://matrix.org/blog/2025/06/dispelling-myths/)
- [SimpleX privacy design](https://github.com/simplex-chat/simplex-chat/blob/stable/PRIVACY.md)
- [Holochain 0.6 to 0.7 upgrade guide](https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/)
- [Holochain Dev Pulse 157: 0.7.1, DirectSignal and chain restore](https://blog.holochain.org/dev-pulse-157-holochain-0-7-1-brings-chain-restore-direct-signal-security/)
- [NIST IR 8547 — Transition to Post-Quantum Cryptography Standards](https://csrc.nist.gov/pubs/ir/8547/final)
