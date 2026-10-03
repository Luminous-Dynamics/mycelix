# D6U — Holochain 0.7 Runtime Authority Boundary

Status: **ReferenceModelOnly**

D6U is the live-substrate companion to D6S-CANON-2.

D6S-CANON-2 provides a deterministic 17-case reference matrix. D6U binds 15 of those cases to an actual Holochain 0.7 test conductor using the same protocol primitives exposed by Holochain's 0.7 test stack:

- Sweettest conductor;
- inline zomes;
- signed AppRequest::CallZome;
- ZomeCallParamsSigned;
- capability grant entries;
- exact nonce and expiry construction.

The two remaining D6S-CANON-2 cases stay outside this harness:

- isolated authenticated-but-not-yet-authorized state;
- blocked provenance.

They require lower-level or system-policy instrumentation not needed by the ordinary application-call path, so the harness records them as unsupported rather than fabricating an observation.

## Boundary

The harness distinguishes:

1. wire authentication;
2. D6S payload integrity;
3. invocation routing/binding;
4. capability authorization;
5. nonce and expiry enforcement;
6. zome entry;
7. semantic validation.

A pre-zome failure is never represented as a semantic zome rejection.

Holochain 0.7's application interface may return AppResponse::ZomeCalled even when authorization fails. The returned serialized ZomeCallResponse must therefore be inspected to distinguish a zome-level result from an authorization or other pre-zome failure.

## Capability lifecycle

The fixture creates an assigned capability grant on Alice's local source chain, uses the returned secret from Bob, and then deletes the grant before attempting reuse. Provenance mismatch is exercised separately by presenting the valid secret from a third agent.

The same-agent author-grant path is tested independently and does not rely on an explicit capability grant.

## Nonce and expiry

Replay is exercised by sending exactly the same signed call twice with one nonce. A higher nonce is accepted first, followed by a lower nonce to exercise stale/older rejection. An invocation with an already expired timestamp is rejected before the probe zome is reached.

These cases are observations of the pinned runtime behavior, not reimplementations of the authorization algorithm.

## Evidence

The runtime workflow records the exact source commit, workflow hash, Holochain/HDK/HDI versions, Rust identity, generated Cargo.lock hash, test result, supported/unsupported case sets, and claim ceiling.

The artifact status is runtime-reference-evidence, not qualified. The D6U manifest freezes the exact D6S-CANON-2 manifest and fixture identities used by the runtime harness.

## Claim ceiling

**ReferenceModelOnly.**

A passing D6U run establishes only observed behavior of the pinned Holochain 0.7 test substrate and the supplied authority-boundary fixture. It does not establish semantic truth, production security, legal authority, physical safety, or actuation authority.
