# did:mycelix Method — Implementation Profile v0.1

**Status:** Draft implementation profile, not a finalized W3C DID Method specification.

This document records the behavior currently implemented by the Mycelix Identity DNA and its browser-safe APIs. It intentionally distinguishes implemented guarantees from unresolved protocol questions.

## 1. Method identity

- DID method name: \`mycelix\`
- Current DID form: \`did:mycelix:<method-specific-id>\`
- Current method-specific identifier: the calling Holochain agent public key rendered in its canonical Holochain string form.
- Current construction: \`did:mycelix:{agent_pub_key}\`.
- The method-specific identifier is the canonical Holochain `AgentPubKey` textual form.
- The integrity layer rejects empty identifiers, URI delimiters, whitespace, non-ASCII characters, and other characters outside the canonical AgentPubKey textual alphabet before accepting a DID document.
- DID URLs containing query, path, or fragment components are not accepted by the current DID-only resolver; DID URL dereferencing remains a separate protocol feature.
- The current Identity DNA declares network seed \`mycelix-identity-v1\`.

### Deployment scope decision

A Holochain `AgentPubKey` is not inherently bound to one DNA. The current DID string therefore does **not** claim that every independent Identity DNA deployment is an authority for the same global namespace.

The implementation profile adopts this scope rule:

- `did:mycelix:<agent_pub_key>` belongs to the canonical Mycelix Identity deployment;
- staging, test, and foreign Identity DNA deployments are non-authoritative for the `did:mycelix` namespace;
- those deployments must not be presented as independent production DID authorities;
- deployment identity remains infrastructure metadata rather than being silently appended to the DID string.

This avoids changing the already-deployed identifier shape while making the namespace boundary explicit. The production deployment identity and bootstrap/relay configuration therefore become part of the operational trust root, not part of the DID subject identifier.
## 2. Internal canonical model versus external DID representation

The internal Holochain entry stores \`controller\` as an \`AgentPubKey\` because that is the canonical authorization primitive for this implementation.

The browser-safe DID projection converts that value to the DID string:

\`did:mycelix:<agent_pub_key>\`

The browser-safe projection also exposes \`created\` and \`updated\` as RFC3339 timestamp strings.

Raw Holochain \`Record\`, action-hash, and entry-hash envelopes are not required for ordinary browser rendering.

## 3. Create

The current \`did_registry.create_did\` operation:

1. obtains the caller's conductor-owned agent public key;
2. derives the DID identifier from that key;
3. rejects duplicate DID creation for the agent;
4. creates the initial DID document and verification method;
5. initializes MFA state;
6. initializes progressive self-recovery on a best-effort basis;
7. records links used for DID discovery;
8. emits the existing Identity bridge event.

The browser invokes \`create_did_view\`, which returns a typed display projection instead of a Holochain Record.

## 4. Authorization

Current write authority is rooted in Holochain source-chain action signatures and the calling agent.

The Identity integrity layer additionally validates canonical \`AgentToDid\` links:

- the link base must be the relevant \`AgentPubKey\`;
- the create-link action author must equal the base agent;
- the target must be an action containing a valid DID document;
- the target DID and controller must match the base agent.

Substrate/provider discovery uses a separate `SubstrateRoleToAgent` link type. It is not overloaded onto `DidToService`; this lets DID service links and global provider advertisements have distinct authorization semantics. Provider resolution also re-checks the provider's canonical DID and excludes deactivated identities, so stale advertisements do not resolve as active providers.

This prevents an unrelated agent from creating a namespace link that redirects resolution to another DID.

Deactivation links receive the corresponding author/base/DID binding.

### Controller versus verification-method rotation

The DID controller is the Holochain agent public key embedded in the method-specific identifier and is immutable for the lifetime of the current DID.

The implementation supports **verification-method rotation**, not controller rotation:

- the current controller must authorize the update through its Holochain action;
- the old verification method remains in the DID Document under its original DID URL;
- the old method is removed from the active `authentication` relationship;
- the new method is added with a distinct DID URL and becomes active;
- preserving the old DID URL allows historical signatures to continue to identify the method that existed when they were produced.

A completed social-recovery request currently does **not** transfer an existing DID to a new Holochain controller. The `claim_recovered_did` entry point fails closed until a dedicated, cryptographically verifiable controller-transfer protocol is specified. This avoids accepting an update that would contradict the immutable controller invariant.

## 5. Update

The current DID update path:

- is controller-authorized through Holochain;
- produces a new DID document version;
- preserves the DID identifier;
- preserves the controller binding;
- updates mutable verification-method, key-agreement, and service state;
- retains an append-only DID history index for every committed version;
- replaces the canonical agent-to-DID discovery link so the latest state becomes resolvable;
- exposes exact historical versions through `resolve_did_version`.

The deterministic DSID laboratory verifies that an ordinary update results in version 2 and that the updated service state is returned by canonical resolution. Every accepted DID update increments the version by exactly one.

## 6. Read and resolution

Current raw resolver:

\`resolve_did(did) -> Option<Record>\`

Current typed resolver:

\`resolve_did_view(did) -> Option<DidDocumentView>\`

Current metadata resolver:

\`resolve_did_resolution(did) -> DidResolutionView\`

The metadata projection separates:

- the resolved W3C DID Document wire representation;
- resolution metadata including the returned document media type and errors;
- DID document metadata including \`created\`, \`updated\`, \`deactivated\`, and \`versionId\`.

Successful DID Document resolution currently advertises \`contentType = application/did\`, matching the current W3C DID Resolution binding.

A missing DID is represented as a method-level \`notFound\` resolution error in the typed metadata API.

The current resolver obtains the DID's controller agent key from the method-specific identifier and resolves the canonical DHT state through the agent-to-DID index. The coordinator additionally parses the identifier as a Holochain `AgentPubKey`, so syntactically plausible but non-key identifiers cannot reach DHT resolution.

### Authenticity boundary

Holochain validates authored actions and DHT records according to the DNA's integrity rules. The typed browser projection itself does not carry an independent detached proof object. A generic external DID resolver profile and portable authenticity verification format remain future work.

## 7. Deactivation

The current \`deactivate_did\` operation creates a deactivation record and a corresponding owner-bound deactivation link.

Canonical typed resolution continues to return the DID document, but its document metadata sets:

\`deactivated = true\`

and the document projection sets:

\`active = false\`

This preserves historical identity state while making deactivation machine-readable.

## 8. Security properties currently enforced

The current implementation includes:

- fail-closed live Identity connection behavior;
- explicit signer-readiness checks before browser writes;
- no mock identity fixtures in live mode;
- canonical DID derivation from the conductor-owned agent key;
- duplicate-DID prevention;
- owner-bound canonical DID links;
- owner-bound deactivation links;
- MFA initialization as a fail-closed creation dependency;
- initial MFA primary-key factor identifiers derived from SHA-256 of the canonical AgentPubKey representation;
- MFA refuses initialization when canonical DID existence cannot be verified;
- type-specific, coordinator-authoritative MFA strength decay;
- browser redaction of raw MFA factor identifiers and metadata;
- progressive self-recovery state exposed through a typed projection;
- credential revocation checks fail-closed when revocation state cannot be verified.

## 9. Privacy considerations

The current identifier embeds the Holochain agent public key. The key is public cryptographic material, but making it the stable method-specific DID creates straightforward correlation opportunities across records and services.

The final method specification must document:

- correlation across Mycelix hApps;
- public DHT metadata exposure;
- service-endpoint correlation;
- recovery metadata exposure;
- whether alternate DIDs or unlinkable personas are supported;
- data-retention and deletion expectations;
- resolver logging and operational privacy.

## 10. DID Resolution interoperability

For eventual W3C interoperability, the method must define a complete resolver contract covering:

- DID input parsing and normalization;
- method-specific resolution;
- DID Document representation;
- DID Document metadata;
- resolution metadata;
- deactivation;
- version selection;
- error codes;
- DID URL dereferencing;
- authenticity verification of resolver output.

The current typed APIs are an implementation bridge toward that contract, not a claim of complete generic DID Resolution conformance. The current wire representation uses the W3C DID Resolution profile's `application/did` document representation and `https://www.w3.org/ns/did/v1.1` context. The initial Ed25519 verification method is encoded as `z` + base58btc(multicodec `0xed01` + the raw 32-byte agent key); the Holochain hash-type bytes are not included.

A checked-in vector corpus at `docs/identity/did-mycelix-conformance-vectors-v0.1.json` freezes the currently intended syntax, resolution, wire-property, verification-key, authority, and deployment-scope behavior.

## 11. Deterministic conformance coverage

The current qualification laboratory maps concrete protocol behavior to deterministic scenarios:

| Scenario | Covered behavior |
| --- | --- |
| DSID-001 | create + canonical read |
| DSID-002 | initial MFA + recovery state |
| DSID-003 | duplicate creation rejection |
| DSID-004 | multi-agent DHT resolution |
| DSID-005 | deactivation state transition |
| DSID-006 | self-recovery projection |
| DSID-007 | MFA browser redaction |
| DSID-008 | canonical update |
| DSID-009 | credential projection |
| DSID-010 | typed cross-agent resolution |
| DSID-011 | resolution metadata + deactivation + W3C wire names |
| DSID-012 | substrate discovery + deactivated-provider filtering |
| DSID-013 | malformed DID identifiers fail closed |
| DSID-014 | canonical initial Ed25519 multibase verification key |
| DSID-015 | verification-method rotation preserves historical DID URL references |
| DSID-016 | deterministic historical DID version resolution |
| DSID-017 | structured W3C not-found resolution error |
| DSID-018 | initial MFA factor hash binding |
| DSID-019 | structured unsupported-method resolution error |
| DSID-020 | recovery configuration owner binding |
| DSID-021 | self-recovery request actor binding |
| DSID-022 | cross-agent DHT-derived recovery quorum |

The qualification workflow records scenario IDs, agents, DNA hash, action hashes, entry hashes, expected and observed behavior, pass state, and commit SHA, then hashes the complete evidence capsule.

## 12. Known protocol gaps

The following are intentionally not implied to be complete:

1. Final method-specific syntax, normalization, and uniqueness specification.
2. Deployment/network scoping for globally unique \`did:mycelix\` identifiers.
3. Portable resolver authenticity proof/verification semantics.
4. Complete DID URL dereferencing semantics.
5. Generic DID Resolution input/output compatibility.
6. Formal version selection and historical-version resolution.
7. Complete controller-key rotation and recovery authority semantics.
8. DID Method registration and interoperability test vectors.
9. Production network-infrastructure guarantees and availability characteristics.
10. Migration of the Identity implementation from the current Holochain 0.6 generation to the current 0.7 generation.

## 13. Recovery authority model

Social recovery configuration is controller-owned and cannot be attached to another controller's DID. Self-recovery requests are authored by the designated replacement agent; this permits recovery when the original controller is unavailable without letting an unrelated agent create the request.

Trustee votes remain immutable records authored by individual trustees. Their quorum is therefore derived from DHT-visible vote records rather than requiring one trustee to mutate a request entry authored by another trustee. The `get_recovery_status` projection exposes the derived approval/rejection state across agents.

The remaining recovery transition is deliberately separate: a completed recovery currently does not mutate the original DID controller. The safe protocol shape is a successor DID plus explicit recovery linkage, or another method-level transfer construction that preserves the identifier/controller binding. That transition remains a protocol design gate rather than being inferred from the existing recovery votes.

## References

- W3C Decentralized Identifiers v1.1: https://www.w3.org/TR/did-1.1/
- W3C Decentralized Identifiers v1.0: https://www.w3.org/TR/did-core/
- Holochain Identifiers: https://developer.holochain.org/build/identifiers/
- Holochain Timestamp 0.6.3: https://docs.rs/holochain_timestamp/0.6.3/
