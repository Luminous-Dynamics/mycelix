# Pulse metadata-minimizing mailbox architecture study

**Status:** Design study only; no alternative selected or qualified.  
**Review subject:** Mycelix main commit `f81361124d073a1db309db977abcf7bdce8ad4cb`.  
**Tracker:** [#5121](https://github.com/Luminous-Dynamics/mycelix/issues/5121).  
**Inputs:** [Privacy inventory](privacy-inventory.json), [negative controls](privacy-negative-controls.json), [privacy audit plan](privacy-review-plan.md), [V2 cryptographic envelope specification](../PULSE_V2_CRYPTO_SPEC.md).

## 1. Concrete baseline

The existing V2 envelope protects message payload content, but its canonical Holochain data model also includes metadata:

- `EncryptedEmailV2` carries `message_id`, `sender`, `recipient`, sender/recipient key IDs, bundle ActionHash references, `in_reply_to`, `thread_id` and `created_at_micros` as fields alongside `ciphertext`.
- The `EncryptedEmailV2` entry type at `holochain/zomes/messages/integrity/src/lib.rs` is declared with required validation count but no private-visibility override. The relevant Holochain entry documentation says public is the default.
- `send_email_v2` creates that entry and an `AgentToInboxV2` link whose base is the recipient AgentPubKey and whose target is the envelope ActionHash.
- The integrity validator binds the link base to `envelope.recipient` and binds `action.author` to `envelope.sender`.
- The workspace pins `hdk = 0.6.1` and `hdi = 0.7.1`; the exact pinned-version semantics need verification before an operational claim.

Source anchors at the reviewed commit:

- [V2 envelope fields, lines 162–188](https://github.com/Luminous-Dynamics/mycelix/blob/f81361124d073a1db309db977abcf7bdce8ad4cb/mycelix-workspace/mycelix-pulse/holochain/zomes/messages/integrity/src/lib.rs#L162-L188)
- [Entry type definition, lines 413–418](https://github.com/Luminous-Dynamics/mycelix/blob/f81361124d073a1db309db977abcf7bdce8ad4cb/mycelix-workspace/mycelix-pulse/holochain/zomes/messages/integrity/src/lib.rs#L413-L418)
- [V2 commit and link creation, lines 219–275](https://github.com/Luminous-Dynamics/mycelix/blob/f81361124d073a1db309db977abcf7bdce8ad4cb/mycelix-workspace/mycelix-pulse/holochain/zomes/messages/coordinator/src/lib.rs#L219-L275)
- [Inbox-link validation, lines 1145–1183](https://github.com/Luminous-Dynamics/mycelix/blob/f81361124d073a1db309db977abcf7bdce8ad4cb/mycelix-workspace/mycelix-pulse/holochain/zomes/messages/integrity/src/lib.rs#L1145-L1183)
- Holochain references: [Entries and visibility](https://developer.holochain.org/build/entries/) and [Working with data and links](https://developer.holochain.org/build/working-with-data/).

This is a source-level metadata disclosure finding, **not** a claim that message plaintext is exposed or that the cryptographic construction has been broken. Runtime access, replication and correlation must still be measured against the pinned conductor configuration.

## 2. Threat model

The primary research question is not “is the body encrypted?” It is:

> Given the public records and actions of a DNA participant, which sender, recipient, contact graph, conversation, key lifecycle and timing facts can be inferred?

Separate these observers:

| Observer | Likely evidence surface | Primary question |
|---|---|---|
| Sender and recipient | Full envelope and local key state | Can each identify/authenticate the conversation correctly? |
| Ordinary DNA participant | Public entries, actions, links, hashes and tags | Can sender, recipient, thread or timing be correlated? |
| Validator | Entry fields, action author, validation rules and public references | Which anti-abuse/authorship checks require identities to be visible? |
| Relay/ingress operator (if added) | Client network metadata, ingress timing and destination token | Can sender and recipient be joined by logs or collusion? |
| Passive network observer | Connection endpoints, size, timing, retry patterns | Can sessions be correlated despite encrypted payloads? |
| Malicious contact | Legitimate keys, received messages, receipts, relationship capabilities | Can it replay, enumerate, impersonate, or leak a capability? |
| Colluding observers | Combined logs, public DHT observations and timing | Does the privacy split collapse under collusion? |

A privacy claim must name which observers are in scope and which collusion assumptions apply. “DHT participant” is not the same as “any internet user” or “global passive observer.”

## 3. Candidate designs — hypotheses only

### A. Keep V2 semantics and publish a transparent metadata contract

Retain current routing semantics, but formally state which fields are public to DNA participants. Add UI/help text, threat-model documentation, and tests preventing inaccurate “metadata-private” claims.

- Advantages: lowest migration risk; supports existing inbox discovery and validation.
- Limitations: does not hide recipient relationships, sender authorship or clear thread/time metadata.
- Best use: minimum immediate safety baseline, not a privacy differentiator.

### B. Opaque, relationship-scoped mailbox anchors

Use high-entropy mailbox locators that do not encode an AgentPubKey directly. Evaluate rotation and pairwise/epoch scope so unrelated conversations do not share a stable public anchor. Any proposal needs to explain exactly how the sender obtains the locator, how the recipient finds messages, and how capabilities are revoked.

Possible variants to compare:

1. one opaque anchor per recipient (low overhead, but repeated linkability remains);
2. one anchor per sender-recipient relationship (less cross-contact linkability, but still links events within that relationship);
3. rotating single-use/per-epoch anchors (less direct correlation but increases discovery, state, recovery and stale-token complexity).

A random address does not automatically make a record private: links are public metadata, repeated locators remain correlatable, and the original action author may still identify the sender. A capability must be distributed without publishing a stable mapping from that capability to the recipient's AgentPubKey. First contact/discovery is a distinct unresolved problem; it cannot be hand-waved away by hashing a phone number.

### C. Pseudonymous message-authoring identity

Decouple a message's public Holochain action author from the sender's long-term identity, then authenticate the true sender to the recipient inside the encrypted/authenticated envelope.

This needs a complete protocol for authorization, validation and abuse resistance. A dedicated pseudonymous identity may still be linkable across all messages that reuse it. Per-conversation/epoch identities need lifecycle and recovery rules. A relay identity can become a universal correlator. Anonymous credentials or scoped admission tokens could be candidates, but cannot simply replace sender verification.

The present invariant `action.author == envelope.sender` is part of the existing integrity contract. Relaxing it without a replacement would weaken the security model. Any change must specify what validators can still prove, how recipients authenticate the sender, and how rate limiting/spam control works without exposing persistent sender identity.

### D. Independent ingress relay plus opaque mailbox addressing

Consider an optional relay/gateway split where a relay observes a client network address but not request contents, while the destination sees a request without directly seeing the original client IP, under an exact trust/non-collusion profile. Oblivious HTTP (RFC 9458) is a candidate for compatible stateless HTTP discovery/control requests, not for magically hiding all Holochain message metadata.

- Reference: [IETF RFC 9458 — Oblivious HTTP](https://www.rfc-editor.org/rfc/rfc9458.html).
- Limits: it adds latency and bytes, requires an explicitly supported relay/gateway, does not eliminate traffic analysis, and cannot protect a stable identifier included in request contents. It is not a generic drop-in transport for the Holochain DHT.
- Require independently operated relay/gateway roles, no client-identifying fields in request payload, controlled padding/timing experiments and a clear failure/availability story.

### E. Recipient-local/private storage path

Evaluate whether some metadata can remain in recipient-owned private state instead of being published to a shared DHT. This may reduce public graph exposure but risks losing asynchronous offline delivery or requires another storage/relay contract, and sender cannot simply write private entries onto recipient's source chain. Do not claim this solves asynchronous messaging until authority, availability, recovery, and delivery evidence are specified.

## 4. Recommended research order

1. **Instrument and report the baseline first.** Use a local multi-agent conductor; capture entries, signed actions, public link records and recipient inbox retrieval under the exact pinned HDK/HDI.
2. **Build a metadata-observer matrix.** For each field and observer, classify visible, hidden, derivable, linkable, conditional, unknown; keep source facts distinct from runtime measurements.
3. **Prototype only the weakest necessary alternative.** Start with one relationship-scoped opaque mailbox locator while explicitly measuring the remaining author/link correlation; do not yet change the default production path.
4. **Evaluate sender pseudonymization separately.** Compare dedicated pseudonymous writer identities, a relay-authored DHT publication, and the current long-term author model. Write down each one's abuse-control and trust costs.
5. **Test recovery and abuse before promotion.** Include rotation, lost capability, revocation, relay outage, replay, duplicate routes, conflicting identity, Sybil/spam attempts and malicious relay behavior.
6. **Independent review.** Have a reviewer challenge both the cryptographic assumptions and the Holochain data-flow assumptions before choosing a new envelope version.

Do not invent a new cipher, signature scheme, anonymous credential or group protocol to make this work. Reuse mature, reviewed primitives and obtain expert review for any composition-level cryptographic change.

## 5. Required experiments

### Baseline capture

Use synthetic identities and messages on an isolated local DNA. For each send, capture the exact public entry/action/link view from the sender, recipient, and an independent DNA participant. Verify whether the participant can recover:

- sender and recipient AgentPubKeys;
- message ActionHash, message ID and key-bundle references;
- thread and reply lineage;
- action author and action timestamps;
- send order and destination graph;
- public receipt and deletion/tombstone relationships.

The experiment should explicitly check the pinned `hdk = 0.6.1` / `hdi = 0.7.1` behavior. Do not infer visibility merely from a current online manual.

### Linkability experiments

Use disjoint train/test synthetic sender-recipient sets. Compare an adversary using only public DHT observations against random guessing for sender, recipient, cross-message pairing and conversation membership.

For any future claim of non-linkability under a named observer model, preregister an AUC target of **at most 0.55 with the upper 95% confidence bound at or below 0.55**, plus adversarial tests for unique IDs, rare timing patterns and repeated anchors. This is a target for that specific synthetic threat model, not a universal anonymity theorem.

Report the corpus, number of independent identities and sessions, split strategy, confidence interval, false-positive/negative rates, observer privileges, and residual signals. A small sample or non-significant result is not evidence of no leakage.

### Correctness and abuse tests

- Zero unauthorized sender/recipient identity claims.
- Zero duplicate visible message commits across retries, restarts and route duplication.
- Zero acceptance of revoked/expired mailbox capabilities after the exact revocation linearization point.
- Zero silent message loss under the stated durable storage/network assumptions.
- Zero unbounded message, capability or replay history.
- 100% detection of critical negative-control mutants.
- All delivery outcomes remain distinguished: accepted, possibly sent, recipient committed, receipt verified, expired/revoked, conflict, unknown.
- Measure p50/p95/p99 time-to-recipient-commit, extra round trips, bandwidth and bytes/message versus V2 baseline before setting absolute performance SLOs.

## 6. Decision gate

Choose an alternative only if it demonstrates a real reduction in the selected metadata threat without silently breaking:

- recipient discovery while offline;
- sender authenticity to recipient;
- spam/abuse controls;
- key rotation and device revocation;
- retry and at-most-once inbox commitment;
- recovery after loss of local state;
- delivery-receipt semantics;
- integrity validation and migration from V2.

Keep the old V2 profile explicit and backward-compatible. Do not silently reinterpret existing entries. A passing synthetic test qualifies only the named protocol, topology, observer model and finite corpus.

## Claim boundary

This study establishes neither a current cryptographic break nor metadata anonymity. It is a structured comparison of candidate data-flow architectures. The target is narrower and falsifiable: reduce specific DHT-level sender/recipient linkability while retaining independently qualified message authenticity, delivery, recovery, and abuse boundaries.
