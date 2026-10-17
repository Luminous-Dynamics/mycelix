# Pulse competitive security, recovery and usability benchmark

**Review date:** 2026-10-17  
**Status:** Research baseline and proposed qualification plan; not a certification or feature-completeness claim.  
**Scope:** Public `Luminous-Dynamics/mycelix` repository only. Related tracker: [#5110](https://github.com/Luminous-Dynamics/mycelix/issues/5110).

## Executive decision

Pulse should not compete by copying Signal's or Telegram's feature list. It should aim for a trustworthy, sovereign communications system combining:

1. Signal-grade expectations for default end-to-end privacy and key authenticity;
2. Telegram-grade convenience for multi-device continuity, communities, discovery, bots/integrations and media;
3. Pulse-specific strengths in transparent delivery evidence, user-controlled identity/data, loss-aware interoperability, and explicit consent/authority boundaries.

Those first two are targets to qualify, not claims about Pulse today. Signal and Telegram have mature deployed products and very different system trade-offs. We should benchmark specific capabilities and failure behavior rather than publish an unsupported overall security ranking.

## Current Pulse reality — do not blur roadmap and implementation

The public repository currently documents:

- Pulse Mail as the supported alpha surface.
- The Chat route as deliberately unavailable pending social semantic and durable-state qualification ([#3728](https://github.com/Luminous-Dynamics/mycelix/issues/3728), [#3746](https://github.com/Luminous-Dynamics/mycelix/issues/3746)).
- An experimental transport-core delivery lifecycle that is in-memory and not wired into the supported alpha send/receive path; durable outbox/inbox replay remains open ([#5062](https://github.com/Luminous-Dynamics/mycelix/issues/5062)).
- A draft bounded delivery model-checking starter and expanded verification plan ([#5093](https://github.com/Luminous-Dynamics/mycelix/pull/5093), [#5091](https://github.com/Luminous-Dynamics/mycelix/issues/5091)).
- An existing V2 message substrate and an intended deterministic projection into Chat, which must be qualified rather than presumed correct ([#3746](https://github.com/Luminous-Dynamics/mycelix/issues/3746)).
- ActivityPub/interoperability work that must preserve provider identity, authorship, message identity, moderation observations and native Pulse authority as distinct things ([#2873](https://github.com/Luminous-Dynamics/mycelix/issues/2873)).

A strong existing V2 encrypted envelope does not on its own establish a Signal-equivalent end-to-end security protocol, robust device lifecycle, metadata privacy, or production recovery. Those are separate qualification claims.

### A concrete Pulse metadata boundary to address

A source review at main commit `f81361124d073a1db309db977abcf7bdce8ad4cb` found that the canonical `EncryptedEmailV2` entry includes sender/recipient AgentPubKeys, message ID, sender/recipient key IDs and bundle-hash references, thread/reply references, and a creation timestamp next to the encrypted ciphertext. The entry type has no explicit private-visibility override, while `send_email_v2` creates a DHT inbox link based on the recipient AgentPubKey. The integrity validator also requires the message action author to equal the sender field. See [V2 envelope fields](https://github.com/Luminous-Dynamics/mycelix/blob/f81361124d073a1db309db977abcf7bdce8ad4cb/mycelix-workspace/mycelix-pulse/holochain/zomes/messages/integrity/src/lib.rs#L162-L188), [entry type](https://github.com/Luminous-Dynamics/mycelix/blob/f81361124d073a1db309db977abcf7bdce8ad4cb/mycelix-workspace/mycelix-pulse/holochain/zomes/messages/integrity/src/lib.rs#L413-L418), [send/link path](https://github.com/Luminous-Dynamics/mycelix/blob/f81361124d073a1db309db977abcf7bdce8ad4cb/mycelix-workspace/mycelix-pulse/holochain/zomes/messages/coordinator/src/lib.rs#L219-L275), and [link validator](https://github.com/Luminous-Dynamics/mycelix/blob/f81361124d073a1db309db977abcf7bdce8ad4cb/mycelix-workspace/mycelix-pulse/holochain/zomes/messages/integrity/src/lib.rs#L1145-L1183). Holochain documents that public entries are stored on the DHT, all actions are public, and links are metadata: [Entries](https://developer.holochain.org/build/entries/) · [Working with data](https://developer.holochain.org/build/working-with-data/) · [DHT model](https://developer.holochain.org/concepts/4_dht/).

This is a **source-level metadata-linkability finding**, not a message-plaintext leak or cryptographic break. Exact access/correlation in a frozen multi-agent topology still needs runtime measurement against Pulse's pinned HDK 0.6.1 / HDI 0.7.1 stack. A direct switch to private entries would also break current DHT-based recipient discovery and would not hide the public action portion or action author. Pulse has opened [#5121](https://github.com/Luminous-Dynamics/mycelix/issues/5121) to explore successor designs that reduce this metadata while preserving authentication, spam control, delivery, recovery and revocation semantics.

## Source-linked comparison

The following matrix distinguishes published design/feature claims from Pulse's current status. Features and policies can change, so rerun the source review before using this in public marketing.

| Dimension | Signal — documented current strength | Telegram — documented current strength/trade-off | Pulse current evidence and opportunity |
|---|---|---|---|
| Default message privacy | Signal says messages and calls are always end-to-end encrypted. It publishes technical protocols and specifications. | Ordinary Cloud Chats use client/server encryption, not E2EE; E2EE is used for device-specific Secret Chats and supported calls. | Not yet qualified as a complete Signal-equivalent E2EE messaging system. Make E2EE-by-default an acceptance condition for private communications, not a badge inferred from encrypted storage. |
| Key authenticity/transparency | Published safety-number mechanism plus automatic key verification/key transparency announced 2026-08-11. Signal's support documentation says automatic verification is currently unavailable in some username-started cases or when the phone number is not available; safety-number comparison remains a fallback. It does not verify real-world identity. | Secret Chats offer a visual key comparison; the reviewed public FAQ does not document a Signal-style automatic key-transparency system. | No claim of a qualified transparency system yet. Opportunity: verifiable key-directory consistency, coverage transparency, clear device-key change UX, and a visible Unknown/Conflict state. |
| Ratcheting/post-quantum direction | Publishes PQXDH, Double Ratchet, and ML-KEM Braid specifications. | Publishes MTProto documentation; Secret Chats use its own E2EE protocol. Do not compare cryptographic strength by algorithm name alone. | Reuse mature, reviewed implementations where feasible. No bespoke cryptography. Compare the actual implemented protocol and version under independent review. |
| Metadata minimization | Signal has published designs such as sealed sender, private groups and private contact discovery to reduce service-visible metadata. Do not interpret this as zero metadata or universal anonymity. | Privacy policy says service security/abuse prevention may process IP address, devices/apps and username-change history, with collected metadata kept for up to 12 months maximum under that policy. | No complete, independently reviewed Pulse metadata inventory is established by the delivery/social issues cited here. Define every collected field, purpose, reader, retention period, deletion rule and residual leakage. |
| Identity/discoverability | Usernames and phone-number privacy controls let people connect without making a phone number generally visible. Signal notes usernames/keys do not independently prove a human identity. | Usernames allow messaging without exposing the phone number in many interactions; phone-number visibility is configurable. | Potential for user-controlled identifiers and scoped identity links. Do not conflate a public key, provider account, Holochain AgentPubKey, legal/human identity or cross-platform account link. |
| Multi-device | Up to five linked devices according to current support documentation; linked-device operation and sync have defined primary-device constraints. | Cloud chats sync across an arbitrary number of concurrently logged-in devices, per FAQ. | Not qualified. Treat device enrollment, history transfer, key isolation, revocation and stale-device rejection as one explicit lifecycle. |
| Backup/recovery | Signal announced cross-platform encrypted backups and refreshed on-device backup support on 2026-09-28; backup scope and media limits vary by plan/flow. | Cloud history offers convenience and recovery because messages are stored in Telegram's cloud; that convenience does not provide E2EE for ordinary Cloud Chats. | Durable outbox/inbox recovery is still open. Aim for E2EE-safe user-controlled backups and tested recovery without making the service capable of decrypting message history. |
| Groups and communities | Strong private-messaging focus, with group messaging and privacy-protective design. | Major strength: large groups, channels, public discovery, admin tools, bots and Mini Apps. FAQ currently states up to 200,000 members in a group and unlimited channel audiences. | Social types and Chat projection are planned, not feature-complete. Build communities on existing durable semantics; qualify roles, audience, moderation, edits/deletes and provider projections separately. |
| Integrations/interoperability | Public clients, server and libraries; no general open federation model is promised. | Public Bot API and broad application ecosystem; cloud-centric semantics enable convenience. | ActivityPub and other provider bridges are existing work directions. Differentiate through loss-aware interoperability without converting provider observations into native authorship, authority or delivery receipts. |
| Delivery state semantics | Mature deployed service, but this research did not independently benchmark its end-to-end delivery correctness. | Mature deployed service; cloud sync and transport behavior are not a substitute for E2EE recipient proof. | A concrete opportunity: separate queued, sent/possibly-sent, server-accepted, recipient-committed, receipt-verified, read/opened and Unknown. Prove transitions; do not infer from adapter acknowledgments. |
| Source transparency | Published protocol specs, client source and server source repositories. Source availability alone is not a proof that a deployed binary matches source or that all claims are correct. | Client source and API are available; Telegram's FAQ itself notes server code publication cannot provide the same guarantee about code actually running server-side. | Public Rust-first development can make the process inspectable. Add reproducible qualification receipts and independent review; public code alone is not a security certification. |
| Abusive content, spam and user safety | Message requests, blocking and other privacy-oriented controls; measure the end-user workflows rather than assuming all abuse is solved. | Large-scale community, moderation and bot capabilities; the platform documents processing some metadata/content for abuse prevention. | Safety-by-design can differentiate: scoped block/mute/report, consent boundaries, rate limiting, community-defined policy, appeal paths and minimal disclosure during reports. Keep enforcement power explicit and auditable. |
| Product maturity | Mature cross-platform product at global scale. | Mature global platform with a broad ecosystem. | Early alpha. Do not claim parity on uptime, reach, calls, groups, bots, ease of onboarding or operational maturity without implementation and measured results. |

## Source register

### Signal

- [Signal technical documentation](https://signal.org/docs/) — published PQXDH, Double Ratchet, Sesame and ML-KEM Braid specifications.
- [Automatic key verification](https://signal.org/blog/automatic-key-verification/) — announced 2026-08-11; describes key transparency, consistency checking and external auditors.
- [Automatic Key Verification support details](https://support.signal.org/hc/en-us/articles/10223569377562-Automatic-Key-Verification) — states availability constraints and clarifies that it does not verify the real-world identity of the account holder.
- [Back to Backups: A Year in Review](https://signal.org/blog/backup-improvements/) — published 2026-09-28; describes cross-platform backups, media policies, local backups and device transfer.
- [Linked Devices support](https://support.signal.org/hc/en-us/articles/360007320551-Linked-Devices) — current published device limit and linking constraints.
- [Sealed sender preview](https://signal.org/blog/sealed-sender/) — design background for reducing service visibility into who messages whom.
- [How do I know my communication is private?](https://support.signal.org/hc/en-us/articles/360007318911-How-do-I-know-my-communication-is-private/) — Signal's own explanation of its E2EE guarantee.

### Telegram

- [Telegram FAQ](https://telegram.org/faq) — cloud/Secret Chat distinction, multi-device cloud sync, groups/channels, usernames and bots. The FAQ notes that it can contain outdated information, so revalidate before any release or marketing use.
- [Telegram Privacy Policy](https://telegram.org/privacy) — current policy description for Cloud Chats, Secret Chats and security/abuse metadata.
- [End-to-End Encryption / Secret Chats](https://core.telegram.org/api/end-to-end) — protocol documentation for device-specific Secret Chats.
- [Bots for developers](https://core.telegram.org/bots) — broad integration ecosystem.

### Cross-cutting standards and Pulse work

- [IETF RFC 9420 — Messaging Layer Security](https://www.rfc-editor.org/rfc/rfc9420.html) — standards-track asynchronous group key establishment with forward secrecy and post-compromise security for groups from two to thousands. Evaluate only if its group use cases, threat model, implementation maturity and integration boundaries fit Pulse.
- [IETF RFC 9458 — Oblivious HTTP](https://www.rfc-editor.org/rfc/rfc9458.html) — optional relay/gateway composition that can separate client network identity from request contents under a specific trust split. It does not hide all request-level identifiers and expressly leaves traffic analysis outside scope; use only for carefully scoped requests with independent relay/gateway operators and tested configuration.
- [IETF RFC 9576 — Privacy Pass Architecture](https://www.rfc-editor.org/rfc/rfc9576/) — informational architecture for privacy-preserving authorization tokens. It is a candidate input for bounded contact-discovery/abuse-control queries, not an automatic anonymity guarantee or a drop-in fix for enumeration, issuer/origin collusion, or transport metadata.
- [Signal private contact discovery design note](https://signal.org/blog/private-contact-discovery/) — useful historical explanation of why plain hashes of low-entropy phone/email identifiers can be enumerated; do not mistake this 2017 technology preview for a claim about Signal's entire current deployment. Mycelix's broader composition program is tracked in [#2297](https://github.com/Luminous-Dynamics/mycelix/issues/2297).
- [OWASP MASVS](https://mas.owasp.org/MASVS/) — mobile security control families including storage, cryptography, authentication, networking, platform, code, resilience and privacy.
- Pulse delivery persistence [#5062](https://github.com/Luminous-Dynamics/mycelix/issues/5062), formal modeling [#5091](https://github.com/Luminous-Dynamics/mycelix/issues/5091), Chat semantics [#3728](https://github.com/Luminous-Dynamics/mycelix/issues/3728) and Chat projection [#3746](https://github.com/Luminous-Dynamics/mycelix/issues/3746).

## Paradigm shifts Pulse should pursue

### 1. Evidence-carrying communication

A message status is a claim and each claim needs a precise evidence source. Keep the following states separate:

```text
Created
  -> LocallyQueued
  -> SendAuthorized(policy + device epoch)
  -> Attempted / PossiblySent
  -> TransportAccepted (non-authoritative)
  -> RecipientInboxCommitted (recipient evidence)
  -> ReceiptCryptographicallyVerified
  -> Delivered (only if receipt semantics authorize this claim)
```

Keep `Unknown`, `ConflictingEvidence`, `Expired` and `Revoked` as honest outcomes rather than forcing a success/failure boolean. Define whether Delivered means recipient application committed, device received, or a stronger user-facing event; never silently use read/opened semantics.

### 2. Privacy quantified as a metadata budget

Maintain a field-level data-flow table: field, source, reason, who can access it, retention, deletion, export behavior and whether it can be minimized, encrypted, locally derived or hidden. Explicitly threat-model identifiers, IPs, delivery times, social graph, group membership, route choice, device inventory, backup access, abuse reports and federated bridge data. Encryption of content does not hide all traffic metadata.

### 3. Verifiable identity changes, not just encryption

Encryption protects against outside readers only if participants have authentic keys. Define device addition/removal, key rotation, recovery, stale-device rejection, key-directory consistency, conflicting observations and warnings. Evaluate key transparency and/or auditable append-only commitments with independent review. Do not imply that a ledger by itself proves human identity or prevents every account takeover.

### 4. Recovery that does not require trusting the service with plaintext

Design backup and recovery as an explicit threat model: encryption key custody, recovery-key loss, old-device theft, rollback, malicious backup provider, device migration and revoked devices. Users should understand the trade-off between recovery convenience and who can access history. Consider locally encrypted portable backup plus optional user-selected storage, and only later consider more complex guardian/threshold recovery after threat analysis. No default service-held decryption key.

### 5. Federation with semantic integrity

Treat ActivityPub/other providers as adapters and projections, not as the source of universal truth. Provider ID != Pulse message ID; provider delivery receipt != Pulse receipt; provider group role != Pulse authorization; message content equality != event identity. Expose transformation loss and unsupported fields instead of silently flattening semantics.

### 6. Security UX for normal humans

Verification and recovery are only useful when understood. Keep key changes, new-device enrollment, recipient safety, unknown delivery outcomes and backup loss understandable, accessible and actionable. Safety warnings need tested comprehension, not merely a cryptographic explanation. Measure false reassurance as a failure.

### 7. Small, reviewed cryptographic surface

Do not invent a new messaging cipher, ratchet or group protocol. Compare mature reviewed libraries and standards, prove how they are used, fuzz hostile inputs, keep dependency provenance and update plans, and commission independent protocol/implementation review before claiming security. Signal's protocol documentation and MLS/RFC 9420 are research inputs, not automatic choices.

## Proposed qualification benchmarks

Numbers below are qualification targets, not claims of present capability.

### P0 — hard correctness gates

| ID | Measure | Goal |
|---|---|---:|
| SEC-01 | Private message plaintext exposed to relay/service telemetry or logs | 0 |
| DEL-01 | False `Delivered` claims in adversarial fixtures | 0 |
| DEL-02 | Duplicate application-visible commits for one logical message key under retries, concurrent handlers, restart and cross-route replay | 0 |
| DEL-03 | Silently lost accepted durable outbox entries under the declared durability model | 0 |
| ID-01 | Critical simulated key-directory equivocation/replacement not detected by the chosen verification design | 0 |
| DEV-01 | Stale/revoked device accepted as authorized after revocation takes effect | 0 |
| POL-01 | New send authorization after applicable expiry/revocation linearization point | 0 |
| REC-01 | Silent partial backup restore or falsely reported successful recovery | 0 |
| FED-01 | Provider identity/role/receipt silently promoted into native identity/authority/receipt | 0 |
| MUT-01 | Critical invariant mutants that survive the checker/test harness | 0 |

Do not use a passing property test as a substitute for a complete independent cryptographic review or production threat-model review.

### P1 — reliability and operational quality

Measure, by platform and network profile:

- send-to-recipient-inbox-commit latency: p50/p95/p99;
- time from recipient commit to receipt verification;
- reconnect, restart and history-replay recovery time;
- outbox loss/duplication/conflict under injected crashes at every durable boundary;
- offline queue duration, bounded-storage behavior and expiry handling;
- cross-device key-change/revocation propagation and stale-device rejection;
- payload/storage bytes, CPU, memory, battery and bandwidth overhead;
- privacy leakage under logging, diagnostic export, crash dumps and support workflows.

Start with an exact Pulse baseline. Only set absolute latency/uptime claims after reproducible measurements exist. Synthetic route independence must never be presented as measured real-world independence.

### P2 — usable privacy and product quality

Conduct preregistered task tests with representative users and accessible interfaces. Measure:

- first-message success without exposing a phone number unnecessarily;
- percentage correctly identifying key-change / identity-conflict warnings;
- backup creation and restore completion, and proportion of users who understand recovery-key responsibilities;
- block, mute, report and group-invite safety task completion;
- group audience/retention comprehension;
- error recovery completion, keyboard/screen-reader accessibility and low-bandwidth usability;
- message send success, recovery and onboarding under intermittent connectivity.

Report denominators, confidence intervals, task scripts, device builds, known exclusions and failure causes. Do not treat a small convenience sample as general proof.

## Priority roadmap

### P0 — build a truthful secure core first

1. Finish durable outbox/inbox and replay (#5062), including atomic receiver deduplication and recovery after crashes.
2. Expand/validate the formal delivery model (#5091/#5093): explicit durable write boundaries, duplicate concurrent receiver, idempotency/content commitment map, policy-generation races, and mutant sensitivity.
3. Define a field-level metadata inventory and a device/key lifecycle threat model; no unreviewed cryptographic replacement.
4. Qualify the existing V2-to-Chat projection (#3746) without opening the Chat route prematurely.

### P1 — close the highest-value security/product gaps

1. Select and document the mature E2EE protocol/library boundary after threat-model review.
2. Prototype key-change transparency and user-visible device verification with negative controls and external review.
3. Define encrypted portable backup/recovery contracts and stale-device revocation semantics before broad multi-device claims.
4. Deliver a cohesive first-message, contact-discovery, blocking/reporting and backup UX; test comprehension.
5. Add robust offline/retry/recovery behavior with visible Unknown states and minimal metadata logging.

### P2 — build the sovereignty advantage

1. Qualified Chat, threads and groups based on the existing durable substrate, not a duplicate store.
2. Loss-aware ActivityPub and other interoperability projections with explicit consent, provenance and bridge-loop handling.
3. User-controlled data export/deletion and clearly documented retention.
4. Auditable community roles/moderation policies, appeals and abuse resistance.
5. Only after the secure core: bots/plugins/agent assistance with scoped capabilities, user approval, no automatic disclosure of private content and explicit local-vs-remote processing indicators.

## Go/no-go rules

- Do not claim Pulse is more secure/private than Signal without independent evidence tied to an exact implementation and threat model.
- Do not claim a feature because a type, interface, mock UI or roadmap exists.
- Do not enable Chat or high-risk features while the authoritative storage, identity and privacy contracts remain unqualified.
- Do not choose cryptography by popularity, benchmark speed or brand affiliation alone.
- Do not weaken negative controls to make CI green.
- Every qualification artifact must identify exact commit SHA, toolchain, configuration, corpus digest, assumptions, state/results and claim ceiling.
- A queued, skipped, stale-head or unavailable check is not a pass.

## Intended product promise — once qualified

**Private by default. Recoverable by choice. Clear about what was delivered. Interoperable without surrendering identity or control.**

This is a product direction, not a present-tense security claim.
