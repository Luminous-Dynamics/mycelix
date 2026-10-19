# Mycelix Content Fabric — Nix Edge Cache v0.1 Contract

Status: CF-00 design contract for CF-07.

## Adoption principle

Do not modify Nix. Expose an ordinary HTTP binary cache backed by Content Fabric. A normal Nix client should not need to know that Mycelix exists.

## Required facade

The adapter must serve the standard binary-cache surface needed by Nix clients, including:

- `nix-cache-info`;
- `<store-path-hash>.narinfo`;
- compressed NAR objects referenced by the narinfo metadata;
- HTTP HEAD/GET behavior compatible with normal substituter use.

The precise compatibility profile and compression choices are implementation details for CF-07 and must be tested against stock Nix.

## Retrieval flow

1. Nix requests a store path through the ordinary substituter interface.
2. The local/LAN edge checks its verified CAS.
3. On miss, the adapter resolves an immutable Fabric object/provider set.
4. Content is fetched over an available transport and cryptographically verified.
5. Only verified bytes are promoted to the local CAS.
6. The adapter presents normal narinfo/NAR responses to Nix.
7. Nix independently performs its existing trust/admission verification.

## Trust separation

`Who delivered these bytes?` and `Am I authorized to install these bytes?` are distinct questions.

A community/MSP/ISP edge may deliver bytes without becoming a software trust root. Existing Nix signature/admission policy remains the authority for installability.

## Failure behavior

- Remote provider failure must not corrupt an existing local cache entry.
- Partial downloads must never become addressable.
- If one transport is unavailable, another configured transport may be used.
- If Holochain coordination is unavailable, already cached immutable content remains usable.
- An untrusted edge modifying a NAR must be detected by content and/or Nix verification.

## Edge Seed demo target

The milestone demonstration should prove normal Nix interoperability across multiple geographically separated Fabric nodes, including first-request remote fill, second-machine local cache hit, continued operation after the original source disconnects, and later replication to a newly available node based on an advisory placement proposal.

## Non-goals

- custom Nix client or fork;
- replacing Nix trust/signature policy;
- requiring Holochain for every local cache hit;
- making provider reputation an integrity primitive;
- payment-market dependency for basic technical delivery.
