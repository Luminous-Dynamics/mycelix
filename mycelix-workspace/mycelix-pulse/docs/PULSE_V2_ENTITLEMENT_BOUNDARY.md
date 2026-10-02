# Pulse V2 Entitlement Boundary

Status: design evidence only. This document does not enable `/chat`, change V2 delivery semantics, or add a second inbox store.

## Finding

The current Pulse V2 protocol does not contain an authoritative finite recipient-to-sender entitlement relation.

V2 delivery is currently created by `send_email_v2`, which:

1. creates one immutable `EncryptedEmailV2` record;
2. creates a sender `AgentToSentV2` link; and
3. creates a recipient `AgentToInboxV2` link.

The recipient link proves that this sender-created message was indexed for the recipient. It does not define the complete set of senders whose V2 messages the recipient is entitled to receive.

## Existing structures that do not solve this

### V1 EmailThread participants

The existing `EmailThread.participants` field is not an entitlement authority.

The current thread creation helper initializes participants with the local agent only. It is V1-oriented, and V2 sending does not create or update `EmailThread` records.

Therefore it cannot safely be interpreted as:

`E(R, conversation) = {all senders entitled to deliver to R}`.

### Holochain capability grants

Holochain capability grants authorize callers to invoke zome functions. They are source-chain records describing function access and optional assignees; claims let a recipient retain a capability secret. They are not, by themselves, a Pulse message-delivery membership ledger.

A future design may deliberately bind a message-entitlement primitive to capability state, but that mapping must define:

- which grant/function establishes message entitlement;
- whether entitlement is sender-to-recipient or conversation-scoped;
- revocation semantics;
- effective time / source-chain boundary;
- how all entitled senders are enumerated deterministically;
- how stale or revoked grants affect a delivery frontier.

No such Pulse-specific contract exists today.

## Required primitive

Before V2 Chat promotion can become provable, Pulse needs an authoritative finite entitlement set:

`E(R, scope) = {S1, ..., Sn}`

where every member is an authenticated sender whose V2 messages are within the recipient's entitled delivery domain.

The set must itself have a protocol-level completeness boundary. Merely discovering some contacts, thread participants, capability claims, or previously observed senders is insufficient.

## Consequence for the delivery frontier

Once `E(R, scope)` exists, the previously defined sender-relative frontier can be composed across all members:

`Complete(R, scope) iff forall S in E(R, scope): Complete(S, R, scope)`.

A missing frontier for one entitled sender remains a missing completeness witness; it must never be interpreted as proof that that sender has no messages.

## Near-term implementation rule

Do not retrofit `EmailThread.participants` or generic Holochain capability grants into the completeness witness by convention.

First define the Pulse entitlement semantics as an explicit protocol object or an explicitly specified existing relation. Then implement:

1. authoritative entitlement-set validation;
2. sender-relative frontier validation;
3. adversarial tests for membership changes, revocation, stale state, and missing senders;
4. only then a path from `HostObservedComplete` toward Chat projection.

Until those steps exist, `/chat` remains fail-closed.
