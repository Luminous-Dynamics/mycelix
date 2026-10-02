# Pulse V2 Delivery Frontier

## Status

**Design-only contract.** This does not enable `/chat` or introduce a second inbox store.

## Problem

Host enumeration of V2 inbox links can prove that the observed candidates are internally consistent, but cannot prove:

> No other recipient-entitled V2 message exists outside the observed retrieval frontier.

A dynamic link collection is therefore not a protocol-level completeness witness.

## Required proof shape

A future witness must be scoped to:
1. one recipient;
2. an authoritative finite set of entitled senders;
3. one authenticated frontier for every entitled sender;
4. deterministic validation that the candidate set covers every qualifying V2 message through each frontier.

This is not a global DHT snapshot.

## Sender-relative frontier

For sender S and recipient R:

- F(S,R) = authenticated point on S's public source chain.
- P(S,R) = source-chain prefix ending at F(S,R).
- M(S,R) = every valid EncryptedEmailV2 create action in P(S,R) whose recipient is R.

Qualification requires:

`Candidates(S,R) = M(S,R)`

If the frontier and bounded prefix are valid and completely available, the resulting statement is:

> This sender has no qualifying V2 messages for this recipient before or at this frontier that are absent from the candidate set.

It does **not** prove that the sender will never send another message after the frontier.

Holochain source-chain actions already have monotonic sequence indices and previous-action links, and public agent activity can be queried by bounded sequence ranges. These properties make a sender-relative boundary materially stronger than an inbox-link enumeration.

## Repository audit: no existing authoritative sender set

The current Pulse model does not yet supply the missing entitlement primitive:

- `EncryptedEmailV2` contains exactly one sender and one recipient, plus optional thread/reply metadata; it does not carry a conversation participant set.
- The existing `EmailThread.participants` field is not an authoritative multi-party membership record. The current thread constructor initializes it with the local agent only, so it cannot certify the complete entitled-sender set.
- The Contacts zome's agent-to-contact relation is a user/address-book observation, not a delivery entitlement. It therefore cannot define who is entitled to place V2 messages in a recipient's Chat frontier.
- The Mailbox capability model grants access to a mailbox/resource; it is not currently defined as the authoritative set of agents whose V2 messages are entitled for a recipient.

Accordingly, reusing any of these existing structures as `E(R)` would manufacture authority that the current protocol does not actually establish. The next semantic primitive should be explicit about who defines membership, how membership changes, and which authenticated boundary makes that membership set finite and auditable.

## The unresolved dimension: entitled senders

For an inbox-wide claim the verifier needs an authoritative finite set:

`E(R) = {S1, S2, ... Sn}`

Observing S1 and S2 cannot prove that undiscovered S3 is not entitled.

Therefore the safest near-term qualification scope is **conversation-scoped**: the participant set of a conversation can be explicit. Inbox-wide Chat qualification remains closed until an authoritative membership primitive exists.

## Receipts and digests

The legacy DeliveryReceipt proves observation of one particular legacy EncryptedEmail. It does not prove a sender frontier, coverage of earlier messages, completeness across senders, or absence of undiscovered messages. V2 has no equivalent completeness receipt.

A sender-signed digest of an observed set is also insufficient unless its domain is authoritative. “Messages I chose to include” proves inclusion integrity, not completeness.

A valid frontier therefore needs:
1. an authenticated source-chain boundary; and
2. a deterministic rule defining which V2 records inside that boundary are entitled to the recipient.

## Candidate primitive

The smallest protocol primitive worth prototyping is conceptually:

`DeliveryFrontierV1 { recipient, sender, frontier_action_hash, frontier_action_seq, frontier_previous_action_hash, protocol_version }`

Exact encoding is intentionally deferred.

A verifier would authenticate the frontier, establish that it belongs to an un-forked source chain, retrieve the bounded prefix, identify and validate every V2 create action in that prefix, select those addressed to R, and compare that deterministic set with the qualified candidates.

If this is too expensive, optimize with a sender-authored authenticated accumulator over the **same authoritative domain**. Do not add a second recipient inbox database.

## Multi-sender composition

For finite E(R):

`Complete(R) iff forall S in E(R): Complete(S,R)`

A missing frontier for any entitled sender keeps Chat promotion closed.

A missing frontier is not evidence that a sender has no messages; it is a missing completeness witness.

## Required adversarial tests

Before implementation can open the gate, test:
- frontier before/after V2 messages;
- V2 messages interleaved with unrelated actions;
- missing V2 record;
- missing source-chain segment;
- source-chain fork;
- wrong frontier author;
- recipient mismatch;
- duplicate candidates;
- post-frontier V2 message;
- multiple senders with one missing frontier;
- sender-set membership changes;
- replay/stale frontiers;
- malicious omission from an application-level accumulator.

## Decision

**Do not open `/chat`.**

Do not treat inbox enumeration, delivery receipts, ACKs, observed-record digests, or sender manifests as completeness proofs.

Next implementation target: define the authoritative entitled-sender set, then prototype the sender-relative frontier against existing V2 source-chain records. Only after those semantics are executable and adversarially tested should `HostObservedComplete` acquire a path toward Chat promotion.
