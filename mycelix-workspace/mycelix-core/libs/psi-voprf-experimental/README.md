# PSI-002A — synthetic VOPRF-assisted contact discovery

This crate is the first concrete PSI experiment above the PEC structural profile layer. It is deliberately **synthetic-only** and remains `Experimental`.

## Exact construction

The construction uses RFC 9497 verifiable OPRF evaluation to derive pseudorandom equality tags under an exact server key and exact domain-separated input profile.

The client/server roles are separated in the API:

```text
CLIENT                                      SERVER
------                                      ------
raw synthetic identifiers
  -> canonical set
  -> domain-separated VOPRF inputs
  -> blind locally
  -> BlindedRequest ----------------------> evaluate_blinded()
                                               sees blinded elements only
                         <------------------ BlindResponse + VOPRF proofs
  -> verify proofs/finalize locally
  -> compare final tags against
     PreparedRegistry tag set
  -> client-only intersection result
```

`SyntheticVoprfServer::evaluate_blinded` does not accept `SyntheticIdentifier`, raw protocol input, or client VOPRF state.

The server may use raw **server-owned synthetic registry identifiers** when preparing its registry. `PreparedRegistry` retains only VOPRF-derived tags, tag-set commitment, domain commitment, server public-key commitment and count metadata.

## Low-entropy evidence rule

The experiment does **not** publish an unsalted SHA-256 commitment over raw identifier sets. Such a commitment would itself be dictionary-testable for low-entropy contacts.

Receipts instead bind:

- randomized blinded-request commitment;
- VOPRF-derived server tag-set commitment;
- domain commitment;
- canonicalization-profile commitment;
- exact server public-key commitment.

This does not establish enumeration resistance. The committed adversarial helper intentionally demonstrates that a caller with online VOPRF access can test guesses unless a separately qualified abuse-control layer constrains the oracle.

## Exact backend profile

```text
crate       voprf 0.5.0
repository  https://github.com/facebook/voprf
upstream tag v0.5.0
upstream commit f0531f0812387cd6be01923b21e2157399a9b295
ciphersuite ristretto255-SHA512
features    default-features=false, ristretto255-ciphersuite
```

The upstream v0.5.0 source identifies itself as RFC-9497 synchronized. This repository still treats that as a building-block fact, not a Mycelix PSI security qualification.

## Synthetic identifier profile

Only values in the explicit namespace below are accepted:

```text
synthetic-contact-<lowercase-ascii/digit/_/->
```

Real phone numbers, email addresses, address books, account identifiers and Holochain/Mycelix directories are outside PSI-002A.

## Explicit nonclaims

Even if the Rust tests pass, PSI-002A does not establish:

- generic PSI security;
- malicious-client or malicious-server PSI security;
- offline or online enumeration resistance;
- client anonymity;
- transport/traffic-analysis privacy;
- registry authenticity or freshness;
- operational VOPRF key lifecycle;
- a qualified network wire format;
- authorization/consent for real contacts;
- production admission;
- application authority.

Follow-on composition gates are tracked separately in #2282, #2283, #2285, #2286, #2287 and #2288.
