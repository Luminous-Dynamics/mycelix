# GAME-STEAM-001T — Steamworks capability/profile v0.1

Status: source candidate profile only. No provider capability is qualified by this document.

## Scope

This profile freezes the first Mycelix-facing Steam capability vocabulary for Symtropy integration. Provider-native identifiers and runtime observations remain scoped to the exact Steam/AppID/runtime profile and do not become canonical Mycelix identity, ownership, save, content, session, credential, or economic authority.

## Core separations

- Steam runtime initialized != Steam account authenticated.
- locally observed SteamID != backend-authenticated account link.
- authenticated account != app/DLC ownership.
- ownership observation != universal property right.
- Steam Input availability != account identity.
- lobby membership != game-session authority.
- Steam Networking/SDR transport != Xenia/Mycelix session authorization.
- Steam Cloud locator != canonical save identity.
- Workshop PublishedFileId != portable content identity.
- achievement/stat != credential or proof of skill.
- inventory observation != marketplace/economic authority.

## Exact profile coordinates

Profile id: `mycelix:game-steam:capability-profile:v0.1`

The implementation/runtime profile must separately bind exact AppID, Rust wrapper versions, native Steamworks artifact identities, platform/architecture, and source/qualification refs. Wrapper versions do not define provider semantics.

## Capability dimensions

Each dimension is independent:

- RuntimeInitialized
- AuthenticatedSteamId
- AppOwnershipObservation
- SteamInputAvailable
- StatsAchievementsAvailable
- CloudAvailable
- WorkshopUGCAvailable
- LobbyMatchmakingAvailable
- SteamNetworkingAvailable
- FriendsPresenceAvailable
- InventoryAvailable

No enum ordering or one positive capability may imply another.

## Evidence roles

Distinguish at least:

- RuntimeObservation
- LocalSteamIdObservation
- BackendTicketVerification
- OwnershipObservation
- CapabilityAvailabilityObservation
- ProviderLocatorObservation
- Unknown
- Conflict

Only an exact backend ticket-verification profile may support `AuthenticatedSteamId`; a local client SteamID alone cannot.

## Secret boundary

Publisher/Web API keys, encrypted-ticket private keys, reusable backend credentials and other provider secrets are forbidden in portable fixtures/evidence. Later client session tickets are short-lived provider artifacts and must not become generic logs or durable semantic identity.

## Runtime split

Symtropy owns Steamworks/Bevy runtime integration. Mycelix owns portable provider-neutral identity/provenance/capability projections. Symthaea remains advisory and receives no ambient Steam effect authority.

## Corpus rules

The accompanying fixture is synthetic/sanitized. It tests independence, profile scoping, identity non-collapse, secret exclusion and explicit unknown/conflict states. It does not claim Valve availability or live Steam behavior.

## Claim ceiling

A future qualification of this exact profile/corpus may establish only that the supplied examples and classifications implement the frozen v0.1 semantic distinctions. It does not establish live Steamworks behavior, account ownership, entitlement truth, SDK correctness, networking safety, anti-cheat correctness, provider policy compliance, or any provider effect/economic authority.