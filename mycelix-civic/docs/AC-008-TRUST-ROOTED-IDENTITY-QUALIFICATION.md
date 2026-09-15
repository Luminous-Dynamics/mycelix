# AC-008 — Trust-Rooted Identity Qualification Boundary

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-008 closes the distinction between **structurally valid identity evidence** and **deployment-authoritative identity qualification**.

AC-006 defines reversible identity-link evidence and review state.
AC-007 defines an internal reversible equivalence-view engine.
Neither layer possesses cryptographic trust roots by itself.

AC-008 is therefore the public boundary that requires an injected verifier before identity links can reach the internal equivalence engine.

## Governing rule

`structural evidence + time-bounded qualification receipt + trusted verifier -> analytical equivalence view`

not:

`strings that look authoritative -> trusted identity equivalence`

## Public versus internal APIs

The low-level AC-006 matcher and AC-007 equivalence constructor are crate-private.

Public callers use `IdentityQualificationContract` and must supply:

- `QualifiedIdentityLinkInput` containing the exact AC-006 link;
- `IdentityQualificationReceipt` bound to that link;
- the required qualification-policy reference;
- evaluation time;
- an implementation of `IdentityQualificationVerifier`.

This ensures the civic-types crate cannot accidentally become its own trust root.

## Qualification receipt

A receipt contains:

- unique receipt reference;
- exact AC-006 `link_ref`;
- subject commitment for the exact link representation;
- authority reference;
- qualification-policy reference;
- verification-method reference;
- digest-bearing evidence provenance;
- issue time;
- mandatory expiry time.

The core validates the receipt's structural envelope but deliberately does not claim to understand or verify the cryptographic subject commitment.

That is the verifier's responsibility.

## Injected verifier

`IdentityQualificationVerifier` is a deployment-supplied trust-root interface.

A production implementation can verify, for example:

- Xenia signatures;
- Mycelix authority grants;
- canonical link commitments;
- registry signatures/attestations;
- policy authorization;
- reviewer authority;
- revocation state;
- delegation scope;
- receipt freshness.

The trait returns either success or a stable non-secret failure code.

The anti-capture core therefore says:

> I know what must be verified, but I do not pretend to own the keys or institutional authority needed to verify it.

## Time boundedness

Every qualification receipt has mandatory `issued_at` and `expires_at` values.

A receipt is usable only when:

`issued_at <= evaluated_at < expires_at`

This prevents an old identity decision from becoming permanent invisible infrastructure after the underlying registry, organization or evidence changes.

## Replay resistance at the typed boundary

The core requires:

- `receipt.link_ref == link.id`;
- `receipt.qualification_policy_ref == required_policy_ref`;
- non-empty subject commitment;
- non-empty authority and method references;
- digest-bearing evidence;
- unique receipt references within one evaluation.

The injected verifier must additionally verify that the subject commitment actually commits to the supplied link.

A receipt for one link therefore cannot be silently reused for another through the official API.

## Projection identity

The public metric path requires a non-empty caller-supplied projection ID.

AC-007 internally derives baseline/projected observation IDs from that identity, but AC-008 refuses to synthesize authority from an empty caller identifier.

## Failure semantics

AC-008 fails closed for:

- missing required qualification-policy reference;
- duplicate receipt references;
- malformed receipt envelope;
- receipt/link mismatch;
- qualification-policy mismatch;
- not-yet-valid or expired receipt;
- verifier rejection;
- empty metric projection identity;
- any AC-007 structural or metric projection failure.

Verifier failures expose only a stable failure code. Implementations should not leak private identifiers, credentials or signature material through that code.

## Relationship to Xenia

AC-008 intentionally creates a narrow integration seam for Xenia.

A future Xenia-backed verifier should bind:

`canonical EntityIdentityLink bytes -> commitment -> signed qualification receipt -> authorized issuer/policy -> validity/revocation`

without pushing cryptographic implementation into `civic-types`.

This separation keeps:

- Xenia responsible for trust/authentication;
- Mycelix Civic responsible for institutional semantics and provenance;
- Symthaea responsible for analysis;
- humans/institutions responsible for legitimate review and policy.

## Analytical outputs

Once every link passes AC-008, the internal AC-007 engine can return:

- qualified equivalence components;
- baseline AC-004 metrics;
- identity-projected AC-004 metrics;
- exact identity-link lineage responsible for regrouping.

AC-008 does not change those analytical semantics. It only controls whether the identity assumptions are allowed to enter the engine through the public path.

## Non-goals

AC-008 does not:

- implement a signature algorithm;
- hold Xenia private keys;
- decide which organization should be trusted as a qualification authority;
- make AC-006 fuzzy matching authoritative;
- create identity links itself;
- mutate AC-003 source identity;
- infer corruption, guilt or sanctions;
- turn identity equivalence into civic standing.

## Qualification gate

Before AC-008 is qualified:

1. exact-subject Civic workspace tests pass;
2. rustfmt and warnings-denied Clippy pass;
3. mock-verifier tests prove valid receipts unlock the internal view only through the public boundary;
4. expired/not-yet-valid receipt tests fail closed;
5. replay tests prove a receipt cannot be used for a different link reference;
6. policy-mismatch tests fail closed;
7. verifier-rejection tests prove the internal equivalence engine is never reached;
8. duplicate-receipt tests fail closed;
9. empty-projection-ID tests fail closed;
10. an Xenia integration test verifies a real canonical link commitment/signature and revocation path;
11. API review confirms AC-006 low-level proposal and AC-007 low-level view constructors remain crate-private;
12. threat-model review covers malicious verifier implementations and clearly identifies verifier selection as an external trust decision.

## Next tranche

After AC-008 is integrated with a real trust verifier, AC-009 should add identity-resolution **sensitivity diagnostics**: quantify how much each qualified equivalence component changes concentration and how fragile a conclusion is to withdrawing one or more identity assumptions. Those diagnostics must remain uncertainty analysis, never wrongdoing labels.
