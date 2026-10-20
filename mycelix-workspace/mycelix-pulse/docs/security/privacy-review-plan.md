# Pulse metadata privacy review plan

**Status:** Seed audit plan; runtime inventory is not yet complete.  
**Date:** 2026-10-17  
**Machine-readable records:** [privacy-inventory.json](privacy-inventory.json)  
**Record schema:** [privacy-inventory.schema.json](privacy-inventory.schema.json)  
**Machine-readable negative controls:** [privacy-negative-controls.json](privacy-negative-controls.json)  
**Negative-control schema:** [privacy-negative-controls.schema.json](privacy-negative-controls.schema.json)  
**Opaque mailbox architecture study:** [opaque-mailbox-architecture-study.md](opaque-mailbox-architecture-study.md)  
**Metadata-privacy architecture tracker:** [#5121](https://github.com/Luminous-Dynamics/mycelix/issues/5121)  
**Ratchet/forward-secrecy research:** [#5122](https://github.com/Luminous-Dynamics/mycelix/issues/5122)  
**Tracker:** [#5112](https://github.com/Luminous-Dynamics/mycelix/issues/5112)

## Why this is a paradigm shift

Do not collapse privacy into a single badge, a single risk score, or the statement "messages are encrypted". Privacy properties are different and can conflict:

- content confidentiality;
- sender/recipient and social-graph confidentiality;
- identifier unlinkability across services, devices and time;
- timing, size and route-pattern resistance;
- device/key lifecycle integrity;
- storage, deletion and backup behavior;
- abuse prevention and rate-limit authority;
- user understanding and control.

A score can help prioritize remediation, but it must not allow strong content encryption to mathematically "cancel out" a severe social-graph leak. Record each leakage dimension and claim independently.

The word **budget** in this document means a visible, reviewable decision about each data field and its residual leakage. It does **not** mean differential-privacy epsilon, nor does the inventory claim a formal privacy budget theorem.

## Claim-status vocabulary

| Status | Meaning | May it support a privacy claim? |
|---|---|---|
| design_documented | A public design issue describes a field or desired semantic. | No runtime claim. |
| candidate_surface | This surface is plausible and must be inspected; current presence is not asserted. | No. |
| source_verified_in_repo | Exact source/config at a named commit was inspected. | Only the narrow source fact, not runtime behavior by itself. |
| not_audited | Runtime behavior/exposure/persistence/retention/deletion has not been verified. | No. |
| verified_present / verified_absent | Confirmed for an exact build and named inspection method, with evidence attached. | Only for that build/method/threat model and observation limits. |

Never convert an unknown answer into "not collected". Absence-of-evidence is not evidence of absence, particularly for remote services, crash-report vendors or hidden server-side logs.

## Source-backed baseline finding: canonical V2 DHT metadata

A focused review of exact Mycelix main source commit `f81361124d073a1db309db977abcf7bdce8ad4cb` identified a concrete metadata surface that must be included in the audit. This is a source-review finding, not a runtime packet-capture result or a cryptographic break.

- The canonical `EncryptedEmailV2` struct at `holochain/zomes/messages/integrity/src/lib.rs#L162-L188` contains clear fields for `message_id`, sender/recipient `AgentPubKey`, sender/recipient key IDs, key-bundle ActionHash references, `in_reply_to`, `thread_id`, and `created_at_micros`, alongside the encrypted `ciphertext`.
- `EntryTypes::EncryptedEmailV2` at lines 413–418 has a `required_validations` setting but no `visibility = "private"` override. The Holochain entry documentation says public is the default; the workspace manifest pins `hdk = 0.6.1` and `hdi = 0.7.1`, so the exact pinned-version behavior must remain part of the qualification record.
- `send_email_v2` at `holochain/zomes/messages/coordinator/src/lib.rs#L219-L275` commits the V2 entry and creates an `AgentToInboxV2` link with the recipient AgentPubKey as base and the message ActionHash as target.
- The integrity validator at `holochain/zomes/messages/integrity/src/lib.rs#L1145-L1183` checks that the inbox link base equals the envelope recipient and that `action.author` equals the envelope sender.
- Holochain documents that all actions are public, public entries are stored on the DHT, and links are metadata: [Entries](https://developer.holochain.org/build/entries/) and [Working With Data](https://developer.holochain.org/build/working-with-data/).

This implies that DNA participants may correlate sender/recipient identity, message IDs, thread/reply lineage, timing and key-bundle references from public entry/action/link metadata even though subject/body plaintext is encrypted. The exact observer view and runtime distribution still require tests against a frozen conductor topology. Do not generalize this to "visible to anyone on the internet" without an additional threat model.

**Do not flip V2 entry visibility to private as a quick fix.** The current inbox depends on DHT-visible entries and links; hiding the entry would break remote recipient discovery unless a replacement retrieval/capability protocol is supplied. Hiding the `sender` JSON field alone also does not hide Holochain action authorship while `action.author == envelope.sender` remains enforced.

Two related source-reviewed surfaces also matter:

- DeliveryReceipt carries the original email ActionHash, recipient AgentPubKey and delivery timestamp. Its EntryTypes variant has no private-visibility override, and it is linked from the original email. The coordinator's acknowledge_delivery function explicitly documents that repeated calls create new receipts because the timestamp differs. This is both an activity-metadata surface and an idempotency issue; it is tracked in delivery work #5062.
- The supported Leptos V2 compose path logs the ActionHash response from send_email_v2 to the browser console. This is a local diagnostic/correlation identifier, not evidence of remote telemetry leakage; inspect any diagnostic-export path separately.

Private visibility for an entry would keep its entry content on the author's source chain, but Holochain documents that the action portion remains public. That still exposes action authorship and some timing/entry-hash metadata, so it is not sufficient to hide the sender under the current action-author invariant. The recipient link/discovery contract would also need a new design.

This finding is tracked in [#5121](https://github.com/Luminous-Dynamics/mycelix/issues/5121). The source review also found an explicit current-profile limitation: V2 does not claim a PQ ratchet, forward secrecy against later static endpoint compromise, post-compromise recovery, or portable key recovery. Those questions have a separate research gate in [#5122](https://github.com/Luminous-Dynamics/mycelix/issues/5122). The inventory's source_observations identify exact commit, file blob, lines, symbol and fact; runtime_presence = not_audited is intentionally unchanged until runtime evidence exists.

## Candidate trust-boundary map

The following is a **review map**, not an assertion that every path is implemented today. Mark actual connections only after exact source and runtime inspection.

~~~text
                    [Identity / key directory]
                              |
                              v
[Client local store] -> [V2/Holochain message surface] -> [Recipient client]
       |                          |                            |
       |                          v                            v
       |                 [DHT-visible records/links]   [Local notifications,
       |                                                 indexes, caches]
       |
       +---- candidate ----> [Diagnostics / support export]
       |
       +---- candidate ----> [Encrypted backup / restore]

[Client] -> [Contact-discovery query] -> [PSI / directory boundary]
    |                                      |
    +------------ network observer --------+

[Client] -> [Transport adapter / relay] -> [Transport] -> [Recipient]
                  |              |
                  v              v
             [Attempt logs]   [Receipt evidence]

[Provider adapter] <-> [Federation bridge] <-> [External provider]
                                |
                                v
                      [Native Pulse projection]
                      (must not mint authority)

[User report] -> [Abuse/moderation review] -> [Decision / appeal]
~~~

Important: the word "candidate" does not mean a path exists. The audit must locate actual edges, dependencies and egress points in a frozen build.

## Inventory record contract

For every individual field or tightly related field group, record:

1. stable ID and versioned schema;
2. exact data class and origin;
3. surface and direction: local read/write, remote send/receive, persisted, exported, derived;
4. implementation evidence class and exact commit/build reference;
5. stated purpose and whether the behavior is optional or required;
6. readers/recipients, including client, Holochain/DHT, provider, relay, service operator, SDK vendor, support and other participants;
7. persistence locations, retention rule, deletion semantics and backup/restore behavior;
8. sensitivity, identifiability, linkability and plausible misuse;
9. mitigations/minimized alternatives and residual leakage;
10. required test, negative control, owner/reviewer and open questions;
11. exact evidence artifact digest, environment and claim ceiling.

A cryptographic commitment is not automatically harmless metadata. A keyed/hash identifier may still be linkable; low-entropy identifiers can be enumerated; a valid proof can still leak public inputs, timing or repeated-query structure.

## Threat model to evaluate

| Adversary | Question to test |
|---|---|
| Honest-but-curious service/operator | Which identities, relationships, timings and activity events can the operator correlate despite ciphertext? |
| DHT/record reader | Which fields, links, revisions and tombstones are publicly or broadly observable? |
| Passive network observer | Can size, timing, connection reuse, route changes or retry patterns correlate endpoints? |
| Active relay/provider | Can it delay, replay, suppress, reorder or selectively expose metadata to infer relationship or activity? |
| Stolen/unlocked device | What plaintext, keys, previews, indexes, clipboard content and backups remain accessible? |
| Malicious contact/member | Can invitation, read state, group membership, profile or report behavior expose another user? |
| Key-directory equivocation attacker | Can different devices be shown inconsistent account-to-key mappings or can stale keys be replayed? |
| Bridge/provider adversary | Can external actor IDs, room membership, content or delivery state be linked to native Pulse identities or audiences? |
| Abuse/spam actor | Can low-entropy lookup, invitation, receipt or rate-limit endpoints be enumerated or exhausted? |
| Insider/diagnostics vendor | Does a bug report, analytics SDK, crash dump or abuse report transmit more than the user intended? |

Document which adversaries are in scope, which protections are provided, and what the system intentionally does not protect against. Do not use "anonymous", "zero knowledge", "private", or "metadata-free" without a specific theorem and scope.

## Audit execution: no real user telemetry required

### Step 0 — freeze subject

- Record exact source commit, tree/dirty status, build profile, lockfiles, feature flags, OS/device, configuration and artifact digest.
- Generate a list of all network egress, persistence stores, logging sinks and external SDKs by static review.
- Distinguish the supported alpha path from experimental crates, design docs and future features.

### Step 1 — static data-flow review

Search source/configuration for serialization, logs, tracing spans, HTTP/WebSocket/QUIC calls, Holochain entry/link creation, filesystem writes, notification previews, crash reporting, clipboard, search indexes, backup, export, provider adapters and deletion routines.

For each occurrence, record exact symbol/path/line or stable source link and evaluate whether sensitive fields flow into it. Do not rely on keyword absence alone; wrappers and generated code may hide flows.

### Step 2 — synthetic runtime egress and storage inspection

Use only synthetic identities, fake message bodies, canary tokens and deterministic fixtures. Instrument the local client and controlled test endpoints. Inspect:

- cleartext packet payloads where visible to the test harness;
- request headers/URLs/query parameters and connection metadata;
- serialized records, links, logs, traces and crash dumps;
- diagnostic export archive contents before any upload;
- local DB, search indexes, notification previews and temporary files;
- relay/adapter event metadata and receipt bytes.

The canary should appear only where deliberately injected; an unexpected appearance in telemetry/storage/egress is a fail. Do not use genuine addresses, contacts, recipient identifiers, tokens or message content to prove this point.

### Step 3 — lifecycle and deletion

For every storage surface, test creation, process restart, device revoke, logical delete/tombstone, account export/delete, backup restore, key loss, and rollback/replay where applicable. A UI delete button is not evidence that copies, caches, backups or external provider records are erased.

### Step 4 — side-channel and correlation review

Run paired synthetic sessions where exactly one variable changes: sender, recipient, group, message size, timing, route, retry, device or provider. Compare observers' views and document distinguishing signals. Report what the experiment could observe and could not observe; one negative run is not a proof of no leakage.

### Step 5 — high-risk composition

Reuse the existing PSI work (#2297) for contact discovery. Require separate evidence for input privacy, query enumeration control, relay/transport metadata, registry authenticity/currentness, epoch/key scoping, retention and issuer/server collusion. Do not create a second PSI protocol in Pulse to avoid the harder audit.

Evaluate IETF Oblivious HTTP (RFC 9458) only as an optional relay/gateway composition for selected discovery or directory requests: the relay/gateway must be independently operated under the chosen threat model; request contents must not reintroduce stable identifiers; and traffic analysis, padding, timing, and service availability remain separate limitations. OHTTP does not make ordinary messaging anonymous by itself. Evaluate Privacy Pass (RFC 9576) as an authorization architecture for bounded, privacy-preserving query budgets; its token semantics, issuer/origin collusion, replay, service scoping and enumeration resistance must be separately qualified. Neither standard is a drop-in proof that a complete contact-discovery system is private.

For federation, preserve source identity, audience, provenance and loss records; require explicit authority for every outbound publication. For key transparency, distinguish consistent account-to-key mapping from verification of a real-world person.

### Step 6 — independent review and release decision

Have a reviewer distinct from the author inspect the exact build and its evidence. The reviewer records open findings, scope exclusions and claim ceilings. High-risk unresolved findings block only the relevant release/claim but must be visible; do not inflate an aggregate privacy score to hide them.

## Negative controls

The machine-readable catalog in privacy-negative-controls.json defines thirteen synthetic control cases, each with inventory references, observer vantage points, expected results, a sensitivity gate and explicit claim limits. It includes a dedicated V2 DHT metadata observer test, delivery-receipt idempotency/replay checks, and local ActionHash console-log minimization. It explicitly distinguishes the current public metadata baseline from any future metadata-privacy target. Validate the catalog against privacy-negative-controls.schema.json, then bind each executed run to exact build/config/tool versions, fixture digest and observer method. All catalog entries currently have status not_run; this is a plan, not execution evidence.

Every audit run must include appropriate controls proving that the test is sensitive:

1. Inject a unique canary plaintext into a synthetic message; intentionally add it to a fake log; confirm the scanner catches it.
2. Inject a fake recipient identifier into an outbound request; confirm the egress inspector locates it.
3. Create a fake contact list; show that a naive hash of low-entropy identifiers is recoverable from a synthetic dictionary.
4. Return a changed/stale key mapping; confirm the client does not display an unqualified verified identity.
5. Reuse a provider event ID with a conflicting semantic commitment; require a conflict, not silent overwrite.
6. Give a diagnostic exporter a secret/canary field; require redaction or explicit refusal before export.
7. Put a canary in a backup and delete the source record; verify the backup path's documented retention/deletion semantics instead of assuming it vanished.
8. Add an unsupported schema/unknown metadata field; ensure it cannot cross a trusted positive-constructor boundary.

A surviving negative control means the audit harness is insufficient or the associated property failed. It is not a passing report.

## Acceptance and reporting

- Every record has a stable ID, status, source, owner/reviewer, audit procedure, and risk-based decision.
- Zero factual "not collected/not retained" assertions without exact-build observation evidence.
- Zero plaintext message bodies in service-side delivery telemetry and qualification artifacts.
- Every identified stored/transmitted field has a purpose, reader/access boundary, persistence/retention and deletion disposition.
- Every high-risk flow has at least one negative control and a stated claim ceiling.
- Unknown, unavailable, conflicting, not inspected and verified-absent are separate values.
- Reports bind exact source/build/config/tool versions, source and fixture digests, observer vantage point, results, exclusions and raw minimized failures.
- The final inventory gets an independent privacy review; a schema-valid JSON file is not that review.

## Standards mapping

Use OWASP MASVS-PRIVACY as a baseline for app-side data minimization, identification prevention, collection transparency and user control. Use the other MASVS families where relevant for storage, cryptography, authentication, network and platform interaction.

MASVS privacy checks do not establish what a remote server, Holochain network participant, transport operator, bridge or diagnostic vendor retains. Those require separate evidence and sometimes external review. This document is not a substitute for legal review or a full privacy impact assessment.
