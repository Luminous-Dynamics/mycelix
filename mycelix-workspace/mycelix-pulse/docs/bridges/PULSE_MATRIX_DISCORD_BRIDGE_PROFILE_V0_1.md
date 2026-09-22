# PULSE-BRIDGE-001T — Matrix + Discord external bridge profile v0.1

Status: source candidate profile only. No provider delivery, native Pulse admission, or outbound publication capability is qualified by this document.

Reviewed: 2026-09-22.

## Purpose

Freeze a loss-aware external-observation profile for Matrix Application Service and Discord Bot/OAuth/Gateway/Webhook inputs before implementing PULSE-BRIDGE-002A.

## Native Pulse firewall

- Matrix event ID != Discord message ID != native Pulse MessageId.
- Matrix transaction ID != provider event identity.
- provider delivery observation != native Pulse RecipientReceipt.
- provider room/guild/channel/thread identity != Mycelix community authority.
- provider actor/account identity != Mycelix principal identity.
- provider edit/delete != canonical history rewrite.
- bridge projection != outbound publication authority.

Native Pulse message/envelope/receipt/DHT semantics remain owned by existing Pulse types and zomes.

## Matrix Application Service profile

Profile id: `mycelix:pulse:matrix-appservice:review-2026-09-22:v0.1`.

Freeze these reviewed semantics:

- homeserver pushes batches to `PUT /_matrix/app/v1/transactions/{txnId}` under an authenticated application-service profile;
- `txnId` is a request/idempotency coordinate for the transaction batch, not an event identity;
- identical retry of the same transaction coordinate and same transaction commitment is idempotent delivery evidence;
- same transaction coordinate with a conflicting transaction commitment is Conflict;
- event/room/user IDs remain Matrix-scoped identities;
- state events are identified through `state_key` presence rather than event-type guessing;
- ephemeral events remain distinct from persistent room events;
- application-service observation does not grant Pulse authorship, room authority, or publication authority.

## Discord profile

Profile id: `mycelix:pulse:discord-bot-oauth-gateway-webhook:review-2026-09-22:v0.1`.

Freeze independently:

- application/bot identity and installation scope;
- OAuth user/account-link evidence;
- guild/channel/thread/message/reaction identities;
- webhook identity/authorship distinct from end-user authorship;
- enabled Gateway intents;
- privileged-intent/data-access review state;
- message-content/member/presence availability state;
- provider session/delivery coordinates separately from message identity.

Ordinary-user self-bot automation is forbidden/out of scope.

Provider fields withheld under an access profile are classified as unavailable under that profile, not as proven absent at source.

## Loss vocabulary

V0.1 fixture classifications may use:

- LosslessWithinProfile
- DroppedUnsupportedRelation
- AudienceSemanticsNotRepresentable
- AttachmentNotFetched
- ContentUnavailableUnderProviderAccessProfile
- ProviderModerationStateOnly
- UnknownProviderExtension
- Conflict

LosslessWithinProfile is always scoped to the exact mapping profile; it is not universal semantic equivalence.

## Bridge origin / loop control

A bridged event carries source-provider identity, source event identity and a bridge-origin/source-semantic commitment. Text equality is never sufficient for deduplication. A Matrix -> Pulse -> Discord -> Pulse loop must remain recognizable from lineage rather than content heuristics.

## Identity and moderation

Explicit consented account links may correlate provider accounts with a Mycelix principal while retaining distinct provider actor coordinates. Names, avatars, shared rooms, roles, Discord permissions, or Matrix power levels cannot establish same-human identity or Mycelix authority.

## Attachment and audience boundary

Attachment/media references can be represented without fetching them. Retrieval/publication requires a separate disclosure/access profile. Visibility/audience mismatch must prevent a lossless cross-post claim.

## Claim ceiling

A future qualification of this exact profile/corpus may establish only the frozen external-observation and loss classifications. It does not establish live provider authenticity, complete delivery, account ownership, native Pulse message admission, Holochain persistence, end-to-end confidentiality, moderation correctness, provider policy compliance, or outbound publication authority.