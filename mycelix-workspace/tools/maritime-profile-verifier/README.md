# Maritime Profile Verifier

A small, effects-disabled Rust library for checking the consistency of supplied,
version-pinned Maritime Commons interoperability evidence.

**Status:** exploratory implementation slice for Mycelix #5953. This crate is
not a normative protocol implementation, a certification, or a production
qualification.

## Why this crate is separate

The Mycelix workspace is large and has independent crate/workspace boundaries.
This library declares its own Cargo workspace so that its compilation and test
surface can be evaluated independently. It does not join the root workspace,
add Holochain zomes, or create a second identity, registry, provenance, finance,
inventory, or event-store subsystem.

## Initial checks

- Exact standard-artifact pins, including version, digest, and maturity. A
  production release and a pre-release are intentionally different identities.
- Scheme-qualified external identifiers, with explicit subject, issuer,
  jurisdiction, purpose, and validity scope.
- Currency amount interpretation against a caller-supplied exact metadata
  snapshot and digest. Minor units are canonical decimal strings, avoiding
  IEEE-754 coercion and fixed-width integer overflow. No currency exponent
  table is embedded.
- Temporal evidence consistency across the original local-time lexeme,
  source offset, zone identifier, tzdb version, resolution disposition, and
  resolved UTC instant.
- Typed quantity dimensionality against a caller-supplied pinned unit registry.
  Decimal values remain strings; no binary floating-point conversion occurs.
- Jurisdiction-overlay maturity and the difference between a technical test
  context and an authority-dependent claim.
- Five dispositions: Pass, Reject, Conflict, Unknown, and NotEvaluable.

## Explicit trust boundary

This crate validates the evidence objects and comparisons supplied to it. It
does not independently:

- fetch or authenticate standards artifacts, registries, or metadata;
- verify Verifiable Credential proofs, signatures, key status, or revocation;
- decide whether an issuer is legally competent or locally recognized;
- run the IANA time-zone database or prove that a resolver used it correctly;
- decide law, tax treatment, customs, immigration, port entry, navigation,
  safety, or environmental compliance;
- establish physical availability, resource ownership, provider commitment,
  payment settlement, refund finality, or service fulfilment;
- call provider APIs, write to a network, issue credentials, make bookings, or
  move money.

The host must provide exact pinned artifacts and independently evaluated
recognition/resolver evidence. A non-empty evidence reference is a linkage, not
cryptographic proof. A Pass means only that the declared local consistency
checks passed for the supplied inputs.

## Dispositions

- Pass: the specific local consistency predicate passed.
- Reject: a supplied value contradicts the declared scope or invariant.
- Conflict: supplied profile/artifact identities disagree.
- Unknown: required evidence is absent or cannot be established.
- NotEvaluable: the claim is outside the verifier's declared ability or the
  supplied context is explicitly not authorized for that claim.

The report retains each check; an aggregate disposition never erases the
underlying reasons. Unknown and NotEvaluable are never promoted to Pass.

## Run

From this directory:

    cargo test

The repository environment used to prepare this slice did not expose Cargo or
rustc, so no local Rust build result is claimed. CI or another toolchain-enabled
reviewer must compile and run the exact commit before any implementation claim
is upgraded.

## Next steps

1. Compile and run these tests on the exact PR head.
2. Reconcile the time-boundary, aggregate-state, and provider-currentness
   decisions with the normative Stay Protocol owner before integrating this
   library into the broader semantic verifier.
3. Add independent property/differential tests and exact-pinned external
   artifacts only after their source and redistribution terms are reviewed.
4. Keep provider adapters and all effectful actions outside this crate.
