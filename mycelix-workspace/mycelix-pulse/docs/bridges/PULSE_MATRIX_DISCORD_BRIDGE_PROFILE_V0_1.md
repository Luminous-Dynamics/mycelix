# PULSE-BRIDGE-001T — Matrix + Discord external bridge profile v0.1

Status: **SOURCE CANDIDATE / NOT QUALIFIED / NOT PASS**

Reviewed: 2026-09-22

Tracks Mycelix #2873, #2881, #2892 and the resolved ownership census #2899.

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

Profile ID:

`matrix-appservice-v1.19-reviewed-2026-09-22`

Reviewed source:

`https://spec.matrix.org/v1.19/application-service-api/`

The profile freezes only the semantics required by this corpus:

1. Homeserver-to-application-service delivery uses authenticated
   `PUT /_matrix/app/v1/transactions/{txnId}`.
2. `txnId` identifies the transaction/request for idempotency. It is not a room-event identity.
3. A retry under the same `txnId` is expected to represent the same transaction contents. A different transaction commitment under the same ID is therefore a conflict in this Mycelix profile.
4. `event_id` is the provider event coordinate where supplied.
5. `room_id` and Matrix user IDs remain Matrix-scoped identifiers.
6. State-vs-message classification is based on the presence of `state_key` where applicable; the bridge must not infer state solely from an event-type name.
7. Ephemeral application-service data remains distinct from persistent room events.
8. Application-service observation or injection capability does not transfer room/community authority into Mycelix.

### Matrix identity law

`(provider profile, room scope, event_id)` is the provider event coordinate for this profile.

`txnId` remains a delivery/idempotency coordinate around one event batch.

A later adapter may bind additional authenticated homeserver/registration identity. This fixture does not establish that live authentication theorem.

## Discord profile

Profile ID:

`discord-bot-oauth-gateway-reviewed-2026-09-22`

Reviewed sources include:

- Discord's 2026 server-data access update;
- Discord OAuth2 documentation;
- Discord's self-bot policy.

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
- Gateway/session delivery metadata where supplied.

### Content availability

`message_content_access` is explicit:

- `available`
- `withheld_by_profile`
- `unknown`

When Discord withholds content/embeds/attachments/components under the applicable access profile, empty fields do **not** establish that the source message itself was empty.

The expected loss state is:

`ContentUnavailableUnderProviderAccessProfile`

### Authorship

Bot, webhook and ordinary user authorship remain distinct observations.

`webhook-authored != end-user-authored`

Display name/avatar similarity cannot change this.

### Automation boundary

Ordinary-user self-bot automation is outside this profile. Live adapters must use supported Bot/OAuth/Application surfaces.

## External bridge observation boundary

The implementation following this corpus should expose a constructor-controlled positive type conceptually equivalent to:

`AdmittedExternalBridgeObservationV1`

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

It must not expose constructors/conversions that mint:

- Pulse `MessageId`;
- `EncryptedEnvelopeV2HybridPqc`;
- Holochain `EncryptedEmailV2`;
- native `RecipientReceipt`;
- native `DerivedDeliveryState`;
- DHT commit authority;
- provider publication authority.

## Loss vocabulary v0.1

- `LosslessWithinProjectionProfile`
- `UnsupportedProviderRelation`
- `AudienceSemanticsNotRepresentable`
- `AttachmentNotFetched`
- `ProviderModerationObservationOnly`
- `ContentUnavailableUnderProviderAccessProfile`
- `UnknownProviderExtension`

`LosslessWithinProjectionProfile` means only that every field required by this *external observation projection* was retained. It does not claim semantic equivalence to a native Pulse message.

## Loop lineage

Bridge loop handling must use exact ancestry/source commitments, not content equality.

`same text != same provider event`

`bridge ancestry observed != permission to suppress/delete external state`

## Corpus

The sibling fixture `fixtures/PULSE_BRIDGE_001_V0_1.json` freezes 20 cases covering:

- Matrix transaction retry/idempotency and conflict;
- event-vs-delivery identity;
- explicit Matrix state and ephemeral treatment;
- Discord bot/webhook authorship;
- Discord content withholding;
- revisions and tombstones;
- replies;
- opaque future features;
- linked-account non-collapse;
- bridge ancestry loops;
- attachment fetch/disclosure separation;
- audience mismatch;
- provider role/power non-authority;
- profile substitution;
- same-content/different-event non-deduplication.

The fixture is synthetic and contains no credentials or live community data.

## Follow-on architecture

`PULSE-BRIDGE-002A` should implement only the pure external-observation projector.

Any native import is later and explicit:

`external observation + mapping/policy + native construction authority -> NativePulseImportCandidate`

Even that candidate is not a DHT commit.

Outbound Matrix/Discord publication is a still-later effect theorem with independent identity, audience, authorization, durability/idempotency and reconciliation requirements.

## Claim ceiling

A future qualification PASS for this exact profile/corpus may establish only that the supplied provider semantics and expected classifications are internally consistent under the frozen v0.1 rules.

It does not establish live provider authenticity, delivery completeness, account ownership, same-human identity, end-to-end confidentiality, native Pulse admission, native delivery/read state, Holochain persistence, moderation correctness, API-policy compliance or any outbound publication authority.
