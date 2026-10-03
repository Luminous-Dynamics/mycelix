# did:mycelix Method — Implementation Profile v0.1

**Status:** Draft implementation profile, not a finalized W3C DID Method specification.

This document records the behavior currently implemented by the Mycelix Identity DNA and its browser-safe APIs. It intentionally distinguishes implemented guarantees from unresolved protocol questions.

## 1. Method identity

- DID method name: \`mycelix\`
- Current DID form: \`did:mycelix:<method-specific-id>\`
- Current method-specific identifier: the calling Holochain agent public key rendered in its canonical Holochain string form.
- Current construction: \`did:mycelix:{agent_pub_key}\`.
- The identifier is treated as an opaque, case-sensitive value by the current resolver. Percent-decoding, case folding, aliases, and alternate textual normalizations are not currently specified.
- The current Identity DNA declares network seed \`mycelix-identity-v1\`.

### Important scope question

A Holochain \`AgentPubKey\` is not inherently bound to one DNA. The current DID string does not contain the Identity DNA hash or another registry deployment identifier. Therefore the final DID Method specification must explicitly define the uniqueness scope of a \`did:mycelix\` identifier and how different Mycelix Identity deployments are distinguished.

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

### Key rotation

Verification-method updates exist in the current DID implementation, but a complete method-level authority transition from one controller key to another is not yet specified here. This is a required part of the final method contract.

## 5. Update

The current DID update path:

- is controller-authorized through Holochain;
- produces a new DID document version;
- preserves the DID identifier;
- preserves the controller binding;
- updates mutable verification-method, key-agreement, and service state;
- replaces the canonical agent-to-DID discovery link so the latest state becomes resolvable.

The deterministic DSID laboratory verifies that an ordinary update results in version 2 and that the updated service state is returned by canonical resolution.

## 6. Read and resolution

Current raw resolver:

\`resolve_did(did) -> Option<Record>\`

Current typed resolver:

\`resolve_did_view(did) -> Option<DidDocumentView>\`

Current metadata resolver:

\`resolve_did_resolution(did) -> DidResolutionView\`

The metadata projection separates:

- the resolved DID document;
- resolution failure metadata;
- document metadata including \`created\`, \`updated\`, \`deactivated\`, and \`version_id\`.

A missing DID is represented as a method-level \`notFound\` resolution error in the typed metadata API.

The current resolver obtains the DID's controller agent key from the method-specific identifier and resolves the canonical DHT state through the agent-to-DID index.

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

The current typed APIs are an implementation bridge toward that contract, not a claim of complete generic DID Resolution conformance. The initial Ed25519 verification method is encoded as `z` + base58btc(multicodec `0xed01` + the raw 32-byte agent key); the Holochain hash-type bytes are not included.

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
| DSID-011 | resolution metadata + deactivation |
| DSID-012 | method syntax / representation conformance |

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

## References

- W3C Decentralized Identifiers v1.1: https://www.w3.org/TR/did-1.1/
- W3C Decentralized Identifiers v1.0: https://www.w3.org/TR/did-core/
- Holochain Identifiers: https://developer.holochain.org/build/identifiers/
- Holochain Timestamp 0.6.3: https://docs.rs/holochain_timestamp/0.6.3/
