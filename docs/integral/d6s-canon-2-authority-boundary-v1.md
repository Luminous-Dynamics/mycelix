# D6S-CANON-2 — Authority Boundary Reference

Status: **ReferenceModelOnly**

## Purpose

D6S-CANON-1 establishes the exact bytes and integrity commitments of a D6S payload. D6S-CANON-2 defines the next boundary as an explicit authority matrix so that payload integrity, transport authentication, invocation binding, capability authorization, and zome-level semantic validation are not collapsed into one assertion.

The matrix is a **reference mapping**, not a Holochain runtime test.

## Boundary decomposition

`wire bytes + signature`
→ **authentication**
→ `D6S-CANON-1 commitment`
→ **payload integrity**
→ `CellId + zome + function + provenance + capability + nonce + expiry`
→ **Holochain invocation/authorization**
→ **zome execution**
→ **semantic validation**

A passing D6S commitment does not authorize a call. Conversely, a successfully authorized call does not make its payload semantically valid.

## Holochain 0.7 mapping

The Holochain 0.7 `ZomeCallInvocation` contains:

- `cell_id`
- `zome`
- `cap_secret`
- `fn_name`
- `payload`
- `provenance`
- `nonce`
- `expires_at`

Its documented authorization surface includes `verify_nonce`, `verify_grant`, `verify_blocked_provenance`, and `is_authorized`.

The documented authorization order is significant: nonce checking, grant checking, then blocked-provenance checking. Holochain explicitly notes that nonce witnessing is a write operation, so signature verification must happen before nonce witnessing.

Capability authorization is also not identical to authentication. Holochain capability grants bind access to zome functions, and an author grant is a special same-key case that does not require an explicit capability grant.

The `expires_at` field is part of the invocation contract. This reference fixture records expiration as a substrate-owned rejection boundary; its exact internal enforcement point is deliberately not guessed here.

## D6S-CANON-2 cases

The frozen 17-case matrix covers:

- valid and invalid wire signatures;
- exact canonical payload and commitment mutation;
- cell, zome, and function binding;
- valid, wrong, and revoked capabilities;
- the same-agent author-grant path;
- provenance mismatch and blocked provenance;
- nonce replay and stale nonce;
- expired invocation;
- authorized-but-semantically-invalid payload.

Pre-zome failures must never be reported as semantic zome rejection. A case that reaches the zome must first be represented as authorized.

## Runtime boundary

The manufacturing conformance workspace currently declares:

- `hdk = "0.4"`
- `hdi = "0.5"`

This makes the present D6S-CANON-2 layer intentionally independent of a Holochain 0.7 runtime dependency. The manifest therefore records `runtime_binding_status = ReferenceMappingOnly`.

The next runtime tranche should be a dedicated Holochain integration harness pinned to the exact 0.7.x substrate version actually adopted by Mycelix. It should exercise the real conductor/zome-call path rather than reproducing authorization logic inside the reference crate.

Until that exists, this fixture must not be promoted from `ReferenceModelOnly`.

## Claim ceiling

**ReferenceModelOnly.**

This specification establishes only deterministic fixture structure and an explicit mapping to the documented Holochain 0.7 authority boundary. It does not establish cryptographic authenticity in a live conductor, capability revocation in a live source chain, replay resistance in a live conductor, semantic truth, physical safety, or actuation authority.

