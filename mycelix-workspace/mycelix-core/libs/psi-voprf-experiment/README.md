# PSI-002A — Synthetic VOPRF Contact-Discovery Experiment

Status: **source constructed / experimental / not compile-qualified / not a PSI security claim**

This crate is the first concrete protocol experiment under the Mycelix PEC privacy-computation line. It is an exact child of canonical PEC-002A subject:

```text
41a26efa89435fbc328bb5ac68b9e971f4b162cd
```

It pins:

```text
voprf = 0.5.0
mode = RFC 9497 VOPRF
ciphersuite = ristretto255-SHA512
construction = voprf-tagged-set-v1
operation = Intersection
collection = Set
output = ClientOnly
participants = TwoParty
qualification = Experimental
```

## Synthetic-only boundary

Only identifiers in the synthetic namespace are accepted:

```text
syn-contact-v1:<synthetic-id>
```

No real address book, phone number, email corpus, Holochain agent directory, or other personal dataset belongs in this experiment.

The canonicalizer is intentionally narrow:

```text
ASCII
trim outer whitespace
lowercase
strict synthetic prefix
restricted local alphabet [A-Za-z0-9._-] before lowercasing
maximum 128 bytes
```

Canonical duplicates fail closed.

## Construction

For each synthetic identifier, the input to VOPRF is an unambiguous length-prefixed transcript binding:

```text
protocol
construction
backend profile
RFC ciphersuite
equality domain
service domain
session domain
canonical identifier
```

The server directly evaluates the same input for its synthetic registry and obtains a pseudorandom tag. The client obtains its tag through RFC 9497 blind → blind-evaluate/proof → finalize. Exact tag equality produces the synthetic intersection.

## Deliberate abuse demonstration

The crate exposes `oracle_tag_for_guess` specifically to demonstrate that an actor with online oracle access can test guesses from a low-entropy identifier dictionary.

```text
VOPRF
!= enumeration resistance
```

Domain separation reduces accidental cross-service/session tag reuse but does not establish anonymous transport, rate limiting, authorization, abuse resistance, or a general unlinkability theorem.

## Receipt

`ExperimentReceipt` binds the exact protocol/backend/profile identities, canonicalization profile, canonical PEC subject, service/session domains, server public-key digest, client/server snapshot digests, result, and basic wire-size/count metrics.

Even a successfully produced receipt reports:

```text
psi_security_established = false
enumeration_resistance_established = false
client_anonymity_established = false
production_admission_granted = false
application_authority_granted = false
```

## Source corpus

The committed tests are intended to cover:

- canonical PEC profile construction;
- deterministic canonicalization and duplicate alias rejection;
- service/session domain separation;
- client VOPRF output equality with direct server evaluation;
- exact synthetic intersection;
- same-key cross-service domain separation;
- explicit online dictionary-enumeration demonstration;
- server snapshot binding;
- malformed blinded-element rejection;
- mismatched proof rejection;
- pre-protocol maximum-set-size rejection.

## Qualification boundary

This source was authored against the public `voprf 0.5.0` API and RFC 9497, but the current authoring environment did not provide Rust/Cargo execution. Therefore:

```text
source exists
!= source compiles
!= tests pass
!= RFC implementation qualified
!= PSI qualified
```

A separate exact-source qualifier must compile/test/fmt/Clippy the exact frozen subject before any executable PASS is claimed. Backend cryptographic qualification and contact-discovery privacy remain separate later gates.
