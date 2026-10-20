# D6U — Holochain 0.7 Runtime Authority Boundary

Status: **ReferenceModelOnly**

D6U is the live-substrate companion to D6S-CANON-2.

D6S-CANON-2 provides a deterministic 17-case reference matrix. D6U binds 14 of those cases to native Holochain 0.7 runtime behavior using the same protocol primitives exposed by Holochain's 0.7 test stack:

- Sweettest conductor;
- inline zomes;
- signed AppRequest::CallZome;
- ZomeCallParamsSigned;
- capability grant entries;
- exact nonce and expiry construction.

Three D6S-CANON-2 cases stay outside native Holochain 0.7 execution in this harness:

- isolated authenticated-but-not-yet-authorized state;
- distinct stale/older-nonce state;
- pre-zome D6S commitment-mismatch state.

The first two require lower-level or system-policy instrumentation not exposed as standalone application-call results. The third is a D6S integrity property rather than a native Holochain invocation gate: the harness retains a probe-local commitment test but does not present it as Holochain enforcement.

## Boundary

The harness distinguishes:

1. wire authentication;
2. invocation routing/binding;
3. capability authorization;
4. nonce and expiry enforcement;
5. zome entry;
6. semantic validation;

The D6S payload commitment test is recorded separately as application-level evidence and is not counted as native Holochain authority enforcement.
A pre-zome failure is never represented as a semantic zome rejection.

Holochain 0.7's application interface may return AppResponse::ZomeCalled even when authorization fails. A successful `AppResponse::ZomeCalled` contains the zome payload directly; authorization failures remain `AppResponse::Error` values. The harness inspects these separately.

## Capability lifecycle

The fixture creates an assigned capability grant on Alice's local source chain, uses the returned secret from Bob, and then deletes the grant before attempting reuse. Provenance mismatch is exercised separately by presenting the valid secret from a third agent.

The same-agent author-grant path is tested independently and does not rely on an explicit capability grant.

## Nonce and expiry

Replay is exercised by sending exactly the same signed call twice with one nonce. The D6S stale/older-nonce case is not independently executable on Holochain 0.7 because nonces are random 256-bit values and the witness state distinguishes fresh, duplicate, expired, and excessively-future expiry conditions. The harness instead exercises a supplemental future-expiry rejection, while the canonical `nonce-stale` case remains explicitly unsupported. An invocation with an already expired timestamp is rejected before the probe zome is reached.

These cases are observations of the pinned runtime behavior, not reimplementations of the authorization algorithm.

## Evidence

The runtime workflow records the exact source commit and GitHub workflow execution identity, workflow hash, Holochain/HDK/HDI versions, Rust identity, generated Cargo.lock hash, test result, native supported/unsupported case sets, supplemental substrate witnesses, the separate probe-local D6S application check, and claim ceiling.

The artifact status is runtime-reference-evidence, not qualified. The D6U manifest freezes the exact D6S-CANON-2 manifest and fixture identities used by the runtime harness.

## Claim ceiling

**ReferenceModelOnly.**

A passing D6U run establishes only observed behavior of the pinned Holochain 0.7 test substrate and the supplied authority-boundary fixture. It does not establish semantic truth, production security, legal authority, physical safety, or actuation authority.
