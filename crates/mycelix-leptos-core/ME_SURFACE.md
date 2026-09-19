# Mycelix Me Surface v1

Status: **experience architecture contract**

Scope: the shared task-first `Me` surface for identity, devices, recovery, preferences, permissions, local data, and user-centric trust inspection.

## Governing rule

> **Me helps a person understand and control their relationship with Mycelix; it must not compress identity, security, trust, recovery, and authority into one reassuring score.**

The surface is an inspection/control composition over existing authoritative providers. It does not become a new identity provider, capability authority, trust oracle, recovery authority, or synchronization engine.

## Recommended top-level sections

1. **Identity** — identifiers, key associations, and what is actually established about them.
2. **Security & recovery** — local protection, backup/recovery options, and their current demonstrated status.
3. **Devices** — current/authorized devices and pairing/revocation when those capabilities genuinely exist.
4. **Permissions** — inspect current capabilities/authority relevant to the user.
5. **Preferences & accessibility** — theme, motion, density, language, and other user-controlled experience settings.
6. **Data & sync** — local data, pending synchronization, export/backup, and storage controls.
7. **Trust & reputation** — contextual supplied trust/reputation signals, explicitly separate from identity and authority.

A section MAY be absent when there is no meaningful provider. The shell MUST NOT fake a completed capability merely to make the control center look comprehensive.

## ME-001 — Identity facts remain separate

The Me surface MUST preserve distinctions such as:

`local DID exists`

`!= browser key associated with a DID document`

`!= browser key is an active authentication method`

`!= identity proofing completed`

`!= current capability/authority`

`!= trust/reputation tier`.

The UI MUST NOT label a user `Verified` merely because a DID string exists, a browser key exists, a DID document exists, or a trust tier is known.

## ME-002 — Local identity and conductor identity are not silently collapsed

The current frontend can hold a browser-generated Ed25519 identity while the Holochain DID document is anchored to the conductor agent key.

Me SHOULD explain this in progressive disclosure using ordinary language first, for example:

- `Identity on this device`
- `Network identity`
- `This device key is associated with your network identity`

Expert detail MAY expose exact DIDs/public-key identifiers.

The presentation MUST NOT imply that the browser key is the DID document's controller when that is not established.

## ME-003 — Anchored is not live-verified

A local client flag indicating that browser-key anchoring previously succeeded is a local historical fact.

It MUST NOT be presented as a live assertion that the verification method is currently present, active, valid, or accepted by every remote participant.

Useful wording may distinguish:

- `Association previously recorded from this device`;
- `Current remote status not checked`;
- a future independently verified live state when that capability exists.

## ME-004 — Protection, lock state, and recovery are independent

At minimum Me SHOULD distinguish:

- whether local secret-key storage is passphrase-protected;
- whether that protected identity is unlocked for the current session;
- whether a recovery phrase can be exported/imported;
- whether a user has merely viewed/exported recovery material versus whether a recoverable path has actually been tested;
- whether guardian/trustee recovery is available;
- whether multi-device pairing is available.

`Protected at rest != unlocked now != recoverable != recovery tested`.

The shell MUST NOT call guardian recovery or multi-device pairing `Enabled` while those flows are not implemented.

## ME-005 — Never require the seed phrase for routine authentication

Recovery material is for recovery/backup, not a normal login secret.

The product SHOULD avoid repeatedly asking users to transcribe or memorize seed phrases, recovery codes, or passphrases when a safer assistive mechanism or alternate authentication method can accomplish the task.

Authentication forms SHOULD support password managers and paste where technically compatible with the security model.

A recovery phrase SHOULD be treated as highly sensitive material: hidden by default, revealed through an intentional action, and never copied into analytics/logs.

## ME-006 — Recovery UX should make failure survivable

Me SHOULD make recovery preparedness understandable before a device is lost.

Useful states include concrete facts such as:

- `This device identity has no additional recovery method configured`;
- `Recovery phrase can restore this local signing key`;
- `Guardian recovery is not available yet`;
- `Device pairing is not available yet`;
- a future `Recovery path tested on <date>` only when a real test has established that fact.

Avoid vague scoring such as `Recovery strength: 82%` unless a separately justified, inspectable model exists.

## ME-007 — Devices are authorizations, not copies of the raw key

Future multi-device pairing SHOULD provision an authorization/device key relationship rather than transfer the raw long-lived private key between devices.

Me SHOULD eventually let users inspect:

- current device;
- recognized/authorized devices;
- last demonstrated activity/freshness where authoritative;
- capability scope where applicable;
- revocation status;
- revocation controls with explicit consequence preview.

A device being listed does not by itself prove it is currently online, uncompromised, or authorized for every action.

## ME-008 — Permissions are inspectable authority, not trust

The Permissions section SHOULD answer human questions such as:

- `What am I currently allowed to do?`
- `Why am I allowed to do it?`
- `When does this permission expire?`
- `Who delegated it?`
- `Has it been revoked?`

Permission presentation MUST consume authoritative capability/authorization state. It MUST NOT derive permissions from:

- trust tier;
- DID presence;
- profile completeness;
- route visibility;
- previous successful action;
- Finder/Home/Inbox visibility.

If current authority cannot be established, show Unknown/Unavailable rather than preserving a stale enabled action.

## ME-009 — Trust/reputation stays contextual

Trust/reputation is useful user context but MUST remain separate from:

- identity proofing;
- authentication state;
- capability/authorization;
- evidence validity;
- safety/security posture.

The surface SHOULD say what a trust signal represents and which provider/context supplied it.

A cross-domain `Overall trust score` SHOULD NOT be introduced merely to make the UI simpler.

## ME-010 — Preferences are user choices, not identity claims

Theme, reduced motion, density, text sizing, notification preferences, language, and similar settings belong in Me when useful.

Preferences SHOULD be usable locally and SHOULD clearly indicate whether they are:

- local to this browser/device;
- synchronized across devices;
- organization/domain policy controlled;
- unavailable/unknown.

`Preference saved locally != preference synchronized`.

Accessibility preferences MUST NOT be hidden behind an expert/developer surface.

## ME-011 — Data & sync makes locality understandable

Me SHOULD provide a human-readable view of the person's local data relationship with Mycelix, including where technically available:

- locally stored identity/key material category (never raw secret bytes in ordinary UI);
- locally durable drafts/content;
- work waiting to synchronize;
- conflicts requiring review;
- export/backup options;
- local data deletion controls and their exact scope.

The UI MUST distinguish local deletion from remote/provider deletion.

The UI MUST distinguish local durability from remote synchronization, federation, confirmation, and settlement.

## ME-012 — Destructive controls require consequence clarity

Actions such as:

- removing passphrase protection;
- deleting local identity/key material;
- revoking a device;
- deleting local data;
- revoking a capability;
- replacing recovery configuration;

MUST show what will actually change, what will remain, whether the action is local or remote, and whether recovery is possible.

The shared Me shell MAY host the preview but MUST NOT bypass domain/provider authorization checks.

## ME-013 — Unknown and unavailable remain visible

Me is particularly vulnerable to false reassurance because users interpret account/security screens as authoritative.

If a remote identity, device, permission, recovery, or sync state cannot be established, the surface MUST say Unknown or Unavailable rather than showing the last known positive state as current.

Cached historical facts MAY be shown if clearly labeled with their provenance/freshness.

## ME-014 — No universal security score in v1

The v1 shared Me model SHOULD NOT calculate one security/recovery/account-health percentage.

Such a score would collapse incomparable facts and can hide the exact remediation a user needs.

Prefer concrete statements and actions:

- `Private key is stored unencrypted on this device`;
- `Protect with a passphrase`;
- `Recovery phrase available`;
- `Guardian recovery not yet available`;
- `2 unsynchronized local drafts`;
- `1 capability expires tomorrow`.

## ME-015 — The surface should be progressively disclosed

Default presentation SHOULD use ordinary language and actionable facts.

Expert disclosure MAY expose:

- DIDs;
- public keys/fingerprints;
- verification method identifiers;
- provider IDs;
- capability IDs;
- evidence/provenance;
- exact timestamps/freshness;
- cryptographic algorithm identifiers.

Progressive disclosure MUST NOT delete contradictory or weaker evidence.

## Proposed shared provider model

A future Me runtime contract SHOULD avoid one giant mutable `UserProfile` object. Providers should contribute facts/controls by section:

```text
MeBatch
  provider_id
  state: Ready | Unavailable | Unknown
  items: [MeItem]

MeItem
  id
  provider_id
  section: Identity | SecurityRecovery | Devices | Permissions | Preferences | DataSync | Trust
  label
  value / state label
  detail?
  freshness?
  evidence/provenance?
  target?: Navigate | PreviewControl
```

Consequential controls should remain provider-owned typed workflows rather than generic shell callbacks.

## Current implementation-aware baseline

The existing shared frontend currently establishes several useful facts that Me can eventually expose truthfully:

- a browser Ed25519 local identity exists and derives its local DID from the public key;
- passphrase-wrapped local key storage is optional rather than default;
- a BIP-39 recovery phrase can export/import the local Ed25519 seed;
- the browser key can be recorded as an additional verification method on the network DID document;
- the client keeps a local historical flag after anchoring succeeds.

It does **not** currently establish:

- guardian/trustee recovery UI completion;
- multi-device pairing;
- browser key as an active DID authentication method;
- a live verification that the locally remembered browser-key association is still present remotely;
- a universal identity-proofing result.

Me must expose the former without implying the latter.

## Accessibility/authentication baseline

Authentication and recovery flows SHOULD be designed so users are not forced to memorize or transcribe secrets when an accessible mechanism/alternative is available.

For passphrase entry and recovery-related inputs, future qualification SHOULD verify, where compatible with the security design:

- paste is not unnecessarily blocked;
- password-manager/autofill mechanisms are not unnecessarily blocked;
- labels and error associations are programmatic;
- keyboard operation is complete;
- recovery material is not exposed in accessible names, analytics, or logs unless intentionally requested;
- instructions explain consequences without relying on memory of previous screens.

## Qualification scenarios

Future Me qualification SHOULD cover at least:

1. identify whether local key storage is protected without implying current unlock state;
2. distinguish local DID, network DID/controller, and browser verification-method association;
3. distinguish historical anchor record from live remote verification;
4. export recovery material without making it part of routine authentication;
5. recover the same supported identity through each implemented recovery path;
6. correctly show unavailable guardian/device recovery before implementation;
7. inspect an expiring/revoked capability without deriving authority from trust;
8. change an accessibility preference and correctly describe local-vs-synced scope;
9. inspect local unsynchronized work without implying data loss;
10. preview a destructive local-data or identity operation with exact scope;
11. complete authentication/recovery workflows with keyboard and assistive technology;
12. verify Unknown/Unavailable remote state does not fall back to stale positive claims.

## Research alignment

NIST SP 800-63-4 emphasizes customer-centered digital identity design, realistic usability evaluation, and systems that make the right action easy and recovery from mistakes practical.

WCAG 2.2 Accessible Authentication requires avoiding unsupported cognitive-function tests in authentication paths and recognizes mechanisms such as password-manager support and copy/paste as ways to reduce memory/transcription burden.

## Proposed PR sequence

1. `UX-ME-002` — typed provider/item inspection envelope.
2. `UX-ME-003` — truthful local identity/protection projection.
3. `UX-ME-004` — recovery-readiness projection and explicit unimplemented states.
4. `UX-ME-005` — permissions/capability inspection adapter.
5. `UX-ME-006` — preferences/accessibility scope model.
6. `UX-ME-007` — local data/sync inspection.
7. `UX-ME-008` — accessible Me presentation.
8. `UX-ME-009` — browser/recovery/keyboard qualification.

The first goal is not to make users feel secure. It is to let them accurately understand what security, recovery, authority, and data-control facts are actually established, and what action they can take next.