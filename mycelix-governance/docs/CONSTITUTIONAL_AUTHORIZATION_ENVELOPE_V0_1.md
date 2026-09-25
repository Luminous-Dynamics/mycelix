# Mycelix Constitutional Authorization Envelope v0.1

Status: **experimental / semantic-only**

Tracking: #1246

Stack: MYC-CONST-001 (#1189) -> MYC-CONST-001B (#1233) -> this tranche

This tranche defines the data that later runtime enforcement will authorize, hash/sign, replay-protect, and preserve as evidence. It does **not** activate live zome/DNA authorization.

## 1. Core theorem

A branch/guardian role name is not sufficient authorization.

A constitutional invocation must bind:

1. **constitutional holder class** — which constitutional domain owns the authority;
2. **holder identity** — which exact office/institution/constituent process holds it;
3. **actor identity** — which concrete runtime actor invokes it on the holder's behalf;
4. **action** — the exact sovereign power or shared entitlement;
5. **resource** — the exact target and payload digest;
6. **context** — jurisdiction, source/provenance, matter, purpose, time, nonce/use rules, concurrence, delegation commitment, and review route.

This is analogous to mature authorization systems that model principal + action + resource + context, but Mycelix adds constitutional provenance and separation-of-power semantics.

## 2. Holder is not actor

`holder_id` and `actor_id` intentionally differ.

Example:

- holder class: `Branch::Integrity`
- holder ID: `integrity:region-a`
- actor ID: `auditor-key:7`

The institution holds constitutional authority. The human/key/service performs a concrete invocation.

This allows Mycelix to detect both:

- **institutional conflicts** — the same constitutional office attempting incompatible functions; and
- **actor conflicts** — the same person/key/operator attempting incompatible functions through different offices.

A software service may appear as an `actor_id` for bounded automation without becoming a constitutional sovereign. `AuthorityPrincipal::AutomatedAgent` remains prohibited as the constitutional holder class.

Runtime must authenticate the relationship between actor and holder; a string field is not proof of representation.

## 3. MatterId

`MatterId` is the dynamic-conflict/provenance domain.

It contains:

- a constrained namespace; and
- a stable matter identifier.

Examples:

- `election / municipal-2030-seat-4`
- `procurement / bridge-contract-42`
- `case / constitutional-case-17`
- `emergency / flood-2027-region-a`
- `protocol-upgrade / authority-kernel-v3`

The pure crate validates shape only. Runtime must derive or verify `MatterId` from authenticated provenance. An interested actor must never be trusted merely because it labels two related acts as different matters.

Dynamic separation of duty uses a verified common `MatterId` to prohibit same-matter self-review while allowing compact organizations to reuse personnel safely across unrelated matters.

## 4. Resource binding

Every envelope includes a resource kind, exact resource ID, and payload digest reference.

An authorization for:

`appropriation:42 / sha256:A`

must not authorize:

`appropriation:43 / sha256:A`

or:

`appropriation:42 / sha256:B`.

Runtime digest policy is external to this crate so cryptographic algorithms can evolve without changing constitutional semantics. The runtime must reject algorithms outside the approved profile.

## 5. Canonical bytes

Authorization signatures must not depend on incidental JSON ordering or serializer behavior.

`AuthorizationEnvelope::canonical_bytes()` uses:

- fixed domain separator;
- fixed schema version;
- fixed field order;
- explicit stable enum tags;
- big-endian fixed-width integers;
- explicit length prefixes for strings;
- explicit presence tags for optional fields; and
- canonical sorting for fields whose semantics are sets (for example required concurrence domains and exclusion lists).

Changing any security-relevant field changes the canonical bytes.

The crate intentionally does not hash or sign. Runtime may hash/sign canonical bytes using the current approved Xenia/Mycelix cryptographic profile.

## 6. Source and delegation commitment

The envelope binds a `CapabilitySource` plus an optional source digest.

A delegated source additionally requires `delegation_chain_digest`, committing the invocation to the authenticated delegation chain/root. The digest does not replace parent/chain validation from MYC-CONST-001B.

Intrinsically non-delegable constitutional powers remain non-delegable even if a caller supplies a plausible chain commitment.

Runtime must verify:

- source exists;
- source is authentic;
- source version/hash is current enough under policy;
- source actually authorizes the claimed holder/action/jurisdiction;
- every delegation edge is valid and non-amplifying; and
- no ancestor is expired/revoked.

## 7. Use policy and replay

The semantic envelope distinguishes:

- `OneShot`; and
- explicitly bounded multi-use authority.

The type alone cannot remember consumption. Runtime requires a replay/consumption store keyed by the signed envelope digest and nonce (or equivalent authenticated state).

Runtime must reject:

- a consumed one-shot authorization;
- use count beyond the bounded maximum;
- duplicate nonce where uniqueness is required;
- execution after expiry/revocation;
- payload/target/matter/source substitution; and
- reuse against a different branch/DNA boundary.

## 8. Concurrence diversity

Multiple signatures are not automatically independent checks.

`ConcurrenceRequirement` can constrain:

- minimum approval count;
- minimum number of distinct constitutional domains;
- required domains;
- unique holder identities;
- unique actor identities; and
- excluded conflicted holder/actor identities.

Every approval must bind the **same canonical envelope digest**.

Important: `ConcurrenceApproval` metadata is not proof by itself. Runtime must authenticate each approval signature/credential and derive/verify its holder class/domain from the corresponding authority evidence. A caller cannot satisfy an Integrity requirement by self-labeling an approval as `Integrity`.

Similarly, unique strings are not sufficient Sybil resistance. Runtime identity/office/control-plane validation must establish whether approvals are genuinely distinct under the applicable constitutional profile.

## 9. Separation of duty

The tranche models three forms:

### Static

The same holder or actor may not activate both authority classes regardless of matter.

### Same-matter / dynamic

The same holder or actor may perform both functions generally but not for the same verified `MatterId`.

Example:

`ExecuteAppropriation` + `AuditPublicExpenditure`

may be dynamically incompatible for the same matter.

### Cooling-off

A configured minimum temporal gap separates selected role transitions.

Runtime profiles determine which actions are incompatible; the envelope crate does not invent public policy.

## 10. Receipts

`AuthorizationReceipt` records the authorization-system event, including:

- envelope digest;
- decision state;
- verifier/runtime identity and version;
- evidence references;
- concurrence/approval references;
- timestamp;
- optional output/action reference; and
- optional review reference.

A receipt proves what the authorization machinery recorded. It does **not** prove that the underlying public decision was substantively wise, just, true, or politically legitimate beyond the constitutional checks represented by its evidence.

## 11. Security boundaries deliberately deferred

This tranche does not implement:

- cryptographic signatures;
- signature/key-to-office binding;
- live holder or actor authentication;
- jurisdiction graph resolution;
- MatterId provenance resolution;
- source freshness/revocation storage;
- replay/consumption storage;
- controller/beneficial-control graph for independence;
- Holochain entry validation;
- cross-zome/DNA enforcement;
- constituent voting semantics; or
- policy interpretation by AI.

These must fail closed when runtime enforcement is activated.

## 12. Research grounding

The security shape intentionally borrows mature ideas without importing their governance policy:

- Cedar's principal/action/resource/context request structure;
- NIST RBAC static and dynamic separation-of-duty concepts;
- Macaroons' attenuation/caveat model for delegated authorization; and
- in-toto's emphasis on signed, authorized, provenance-linked steps and artifacts.

Mycelix applies those security disciplines to constitutional public authority.

## 13. Core principle

> **Constitutional authority is not a role label. It is an exact, attributable, provenance-bearing, context-bound authorization over a specific act, with explicit separation and review semantics.**
