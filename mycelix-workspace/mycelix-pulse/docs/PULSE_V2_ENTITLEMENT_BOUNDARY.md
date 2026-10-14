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


## Existing capability model: useful substrate, not yet an entitlement witness

The Pulse capabilities zome already models a useful relationship vocabulary: a `MailboxCapability` binds a grantor to a grantee, carries explicit permissions, supports expiration/revocation fields, and includes `ThreadAccess { thread_id }` as an access scope. This makes it a plausible substrate for future entitlement semantics, but it is not currently wired into V2 message creation or V2 inbox qualification.

In particular, `send_email_v2` currently constructs the durable envelope from the caller and supplied recipient/thread metadata, creates the sender's `AgentToSentV2` link, and creates the recipient's `AgentToInboxV2` link. It does not require, reference, or validate a `MailboxCapability` or `ThreadAccess` capability. Therefore treating existing capability records as the authoritative set of V2 delivery-entitled senders would silently change the current protocol semantics.

### Safe reuse path

If Pulse chooses to make capabilities authoritative for a scoped delivery domain, the contract should bind the capability to that domain explicitly rather than infer the binding:

1. **Scope identity** — define whether entitlement is mailbox-, thread-, conversation-, or direct-recipient-scoped.
2. **Grant semantics** — define exactly which capability permission establishes delivery entitlement, rather than reusing `can_read` or `can_send` by implication.
3. **Authority** — define who may grant and revoke entitlement and how that authority is authenticated.
4. **Effective interval** — bind entitlement to a deterministic source-chain boundary or other signed interval; do not rely on mutable wall-clock interpretation alone.
5. **Membership completeness** — define how the finite set of currently entitled senders is itself proven complete.
6. **Message binding** — define which V2 fields must match the entitlement scope, including recipient and, where applicable, thread/conversation identity.
7. **Revocation semantics** — specify whether messages authored before revocation remain entitled, and where the entitlement frontier ends.
8. **Stale/fork handling** — specify how qualification treats stale capabilities, source-chain forks, and unavailable capability records.

The key distinction is that **authorization to perform an operation** and **membership in the set of agents whose messages must be included in a completeness proof** are different protocol properties. Existing Holochain capability mechanisms are designed around zome-call authorization, while Pulse's missing primitive is an authoritative finite delivery-entitlement relation. Holochain validation also requires dependencies to be addressable and deterministic; mutable link collections are not suitable as a validation-time completeness dependency. citeturn0search0turn0search2

Accordingly, the current safest design remains:

`capability grant -> (if explicitly adopted by Pulse) entitlement state -> authoritative finite sender set -> sender-relative frontiers -> multi-sender completeness witness -> Chat projection`

and **not**:

`capability grant -> assumed delivery completeness`.

This keeps the existing capability subsystem reusable without accidentally granting it a semantic role it does not currently possess.


## Adjacent security finding: application revocation is not yet Holochain-grant revocation

The existing capability coordinator currently marks a `MailboxCapability` as `revoked = true` by updating the application entry, but the same function contains a TODO noting that the corresponding Holochain capability-grant action hash is not retained for `delete_cap_grant`. Therefore the application-level `verify_capability` path can report a capability revoked while the conductor-level zome-call grant may remain present.

This is **not** being used as V2 entitlement evidence, and this PR does not change capability authorization semantics. It is nevertheless important to keep separate from the entitlement design: a future capability-backed entitlement protocol must define revocation at both the application semantic layer and the actual Holochain call-authorization layer, with explicit effective boundaries.

Holochain's capability model makes the distinction concrete: access is controlled by capability-grant entries on the grantor's source chain, and revocation is performed by deleting the grant entry; an application flag alone is not equivalent to deleting that system capability. citeturn0search1turn0search4

Recommended follow-up is a separate capability-security change: retain the system capability-grant action hash returned at grant time and exercise the actual deletion path, with regression tests for both application verification and conductor authorization. Do not couple that fix to V2 Chat qualification.
