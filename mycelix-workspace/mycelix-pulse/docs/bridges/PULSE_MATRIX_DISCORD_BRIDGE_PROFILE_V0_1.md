# PULSE-BRIDGE-001T — Matrix + Discord external bridge profile v0.1 r3

Status: **R3 SOURCE CANDIDATE / NOT QUALIFIED / NOT PASS**

Reviewed: 2026-09-22

Tracks Mycelix #2873, #2881, #2892 and resolved ownership census #2899.

## Purpose

Freeze the first external-provider semantics and deterministic fixture corpus for Matrix Application Service and Discord Bot/OAuth/Gateway/Webhook observations before implementing any bridge projector.

This profile owns **external observation semantics only**. It does not define a generic common-message ontology and cannot mint native Pulse message, receipt, persistence or publication authority.

## Governing separations

- provider event identity != provider request/delivery identity
- provider event identity != native Pulse `MessageId`
- provider actor identity != Mycelix principal identity
- provider room/guild/channel identity != Pulse community authority
- provider edit/delete observation != canonical history rewrite
- provider delivery != Pulse `RecipientReceipt`
- provider visibility/role/power != Mycelix authority
- parser/projection success != outbound publication authority

Native Pulse semantics remain owned by the existing Pulse protocol and messages-zome surfaces identified in #2899.

## Matrix profile

Profile ID: `matrix-appservice-v1.19-reviewed-2026-09-22`

Reviewed source: `https://spec.matrix.org/v1.19/application-service-api/`

The profile freezes only the semantics required by this corpus:

1. Homeserver-to-application-service delivery uses authenticated `PUT /_matrix/app/v1/transactions/{txnId}`.
2. `txnId` identifies the transaction/request for idempotency. It is not a room-event identity.
3. A retry under the same `txnId` is expected to represent the same transaction contents. The r3 comparison corpus therefore supplies both current and prior transaction IDs/commitments when it asks the pure kernel to classify a retry.
4. Different `txnId` values are different delivery/request coordinates. Different batch commitments across different transaction IDs are not a conflict merely because an event appears in both batches.
5. `event_id` is the provider event coordinate where supplied.
6. `room_id` and Matrix user IDs remain Matrix-scoped identifiers.
7. State-vs-message classification uses presence of `state_key` where applicable; the bridge must not infer state solely from an event-type name.
8. Ephemeral application-service data remains distinct from persistent room events and from native Pulse receipts.
9. Application-service observation or injection capability does not transfer room/community authority into Mycelix.

### Matrix identity law

`(provider profile, room scope, event_id)` is the provider event coordinate for this profile.

`txnId` remains a delivery/idempotency coordinate around one event batch.

For comparative classification:

- same `txnId` + same transaction commitment -> `IdempotentRedelivery`;
- same `txnId` + different transaction commitment -> `Conflict`;
- different `txnId` -> distinct delivery observation; batch commitments may differ without creating the same-transaction conflict.

A later adapter may bind authenticated homeserver/registration identity. This fixture does not establish that live authentication theorem.

## Discord profile

Profile ID: `discord-bot-oauth-gateway-reviewed-2026-09-22`

Reviewed sources include Discord's 2026 server-data access update, OAuth2 documentation, and self-bot policy.

This profile intentionally binds **data-access state** independently from installation/authentication.

Relevant coordinates remain independently scoped:

- application/bot identity;
- OAuth-linked user identity;
- guild;
- channel;
- thread;
- message;
- reaction;
- webhook;
- Gateway/session delivery metadata where separately profiled.

### Content availability

`content_access` is explicit:

- `available`
- `withheld_by_provider_profile`
- `unknown`

When Discord withholds content under the applicable access profile, missing/empty content fields do **not** establish that the source message itself was empty.

The expected loss state is `ContentUnavailableUnderProviderAccessProfile`.

### Authorship

Bot, webhook and ordinary-user authorship remain distinct observations.

`webhook-authored != end-user-authored`

Display name/avatar similarity cannot change this.

### Automation boundary

Ordinary-user self-bot automation is outside this profile. Live adapters must use supported Bot/OAuth/Application surfaces.

## Provider field firewall

A provider profile constrains not only interpretation but the fields that may be admitted.

Matrix-only delivery/replay coordinates such as `delivery_id`, `prior_delivery_id`, and transaction commitments cannot be smuggled into a Discord observation.

Discord-only coordinates such as `guild_id` or `webhook_id` cannot be smuggled into a Matrix observation.

Profile-inapplicable authority-bearing/identity-bearing fields must fail typed rather than be silently ignored.

This prevents:

`provider mismatch + ignored foreign fields -> accidental cross-provider semantics`

## External bridge observation boundary

The implementation following this corpus should expose a constructor-controlled positive type conceptually equivalent to `AdmittedExternalBridgeObservationV1`.

It may retain:

- exact provider/profile identity;
- provider event coordinate;
- delivery/request coordinate where present;
- provider actor and room/channel coordinates;
- external chronology;
- revision/reply/thread/reaction targets;
- attachment references;
- audience/visibility observations;
- bridge-origin lineage;
- explicit loss markers;
- opaque commitment for unsupported future extensions.

It must not expose constructors/conversions that mint Pulse `MessageId`, `EncryptedEnvelopeV2HybridPqc`, Holochain `EncryptedEmailV2`, native `RecipientReceipt`, native `DerivedDeliveryState`, DHT commit authority, or provider publication authority.

## Event-kind-specific evidence

Do not impose a single message-shaped required-field schema on every provider event.

Examples frozen by r3:

- Matrix ephemeral receipt can be meaningful without inventing a top-level message author;
- Discord message delete/update observations need not fabricate the original message author;
- Discord message create requires an observed author coordinate;
- Discord reaction events require an explicit reaction target relation;
- unknown/future provider event kinds require a bounded opaque extension commitment rather than guessed semantics.

## Loss vocabulary v0.1

- `LosslessWithinProjectionProfile`
- `UnsupportedProviderRelation`
- `AudienceSemanticsNotRepresentable`
- `AttachmentNotFetched`
- `ProviderModerationObservationOnly`
- `ProviderStateSemanticsNotNative`
- `ProviderEphemeralSemanticsNotNative`
- `ContentUnavailableUnderProviderAccessProfile`
- `UnknownProviderExtension`

These remain distinct. In particular:

`ProviderStateSemanticsNotNative != ProviderModerationObservationOnly != ProviderEphemeralSemanticsNotNative != UnsupportedProviderRelation`

`LosslessWithinProjectionProfile` means only that every field required by this *external observation projection* was retained. It does not claim semantic equivalence to a native Pulse message.

## Loop lineage

Bridge-loop handling must use exact ancestry/source commitments, not content equality.

`same text != same provider event`

`bridge ancestry observed != permission to suppress/delete external state`

## Corpus r3

The sibling fixture `fixtures/PULSE_BRIDGE_001_V0_1.json` freezes 23 synthetic cases covering:

- same-Matrix-transaction retry/idempotency and conflicting retry;
- same event observed through a distinct transaction whose batch commitment may differ;
- explicit Matrix state and ephemeral treatment;
- Discord bot/webhook authorship;
- Discord content withholding;
- revisions and tombstones without history rewrite;
- replies;
- opaque future features;
- linked-account non-collapse;
- bridge ancestry loops;
- attachment fetch/disclosure separation;
- audience mismatch;
- provider role/power non-authority;
- profile substitution;
- same-content/different-event non-deduplication;
- Matrix-field-in-Discord and Discord-field-in-Matrix smuggling rejection;
- Discord reaction target requirement.

The fixture is synthetic and contains no credentials or live community data.

## Follow-on architecture

`PULSE-BRIDGE-002A` should implement only the pure external-observation projector.

Any native import is later and explicit:

`external observation + mapping/policy + native construction authority -> NativePulseImportCandidate`

Even that candidate is not a DHT commit.

Outbound Matrix/Discord publication is a still-later effect theorem with independent identity, audience, authorization, durability/idempotency and reconciliation requirements.

## Claim ceiling

A future qualification PASS for this exact r3 profile/corpus may establish only that the supplied provider semantics and expected classifications are internally consistent under the frozen rules.

It does not establish live provider authenticity, delivery completeness, account ownership, same-human identity, end-to-end confidentiality, native Pulse admission, native delivery/read state, Holochain persistence, moderation correctness, API-policy compliance or any outbound publication authority.
