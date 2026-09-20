# PSI-002B — Contact-Discovery Composition Semantics

Status: **structural source only / not compile-qualified / no privacy theorem**

This crate sits above PSI-002A and freezes the system properties that the VOPRF experiment intentionally does **not** establish.

It implements no Privacy Pass, ARC, OHTTP, registry signature, Holochain, Xenia, VOPRF, or PSI cryptography.

## Governing boundary

```text
VOPRF query hiding
+ bounded query policy
+ oblivious transport declaration
+ registry snapshot declaration
+ key epoch declaration
+ retention declaration
    != abuse resistance
    != transport unlinkability
    != registry authenticity/currentness
    != private contact discovery established
```

## Stable semantic dimensions

The v1 evaluator binds:

- exact PSI-002A source subject;
- authorization-profile identity;
- service domain + budget epoch;
- maximum identifiers per request;
- maximum redemptions per epoch;
- replay/nullifier policy;
- redemption-linkability declaration;
- issuer/verifier-collusion declaration;
- direct vs oblivious-relay transport;
- relay/gateway profile identity;
- gateway client-network-identity declaration;
- explicit traffic-analysis nonclaim;
- VOPRF service/key epoch and exact key digest;
- registry snapshot digest, sequence and registry epoch;
- registry/key-epoch binding;
- separate registry-authenticity and currentness declarations;
- retention classes for raw IDs, blinded elements, derived tags, registry tags, results, authorization evidence and network logs.

## Standards adapter boundary

Privacy Pass RFC 9576/9577 and RFC 9458 OHTTP are useful adapter candidates, but their names are **not** baked into the semantic enums.

For example an adapter may declare:

```text
ExternalProfileIdentity {
    protocol: "privacy-pass",
    version: "rfc9577",
    profile: "example-token-profile-v1"
}
```

or:

```text
ExternalProfileIdentity {
    protocol: "oblivious-http",
    version: "rfc9458",
    profile: "example-relay-profile-v1"
}
```

That declaration alone does not qualify the adapter.

Draft anonymous rate-limited credential work must remain a separately versioned Experimental adapter if evaluated later.

## Fail-closed rules

The default requirement profile rejects:

- zero query or redemption budgets;
- absent replay protection;
- empty scoped-nullifier domains;
- stable/pseudonymous presentation when unlinkable redemption is required;
- redemption linkability not declared hidden;
- direct transport when gateway network identity must be hidden;
- malformed oblivious-relay profiles;
- claims that the generic oblivious-relay profile hides traffic analysis;
- VOPRF key service-domain mismatch;
- malformed key or registry SHA-256 identities;
- registry/key epoch mismatch;
- missing registry authenticity/currentness declarations;
- persistent client-derived tags when persistence is forbidden;
- zero-second retention values silently masquerading as erasure.

## Authority ceiling

Even `StructurallyCompatible` always reports false for:

```text
abuse_resistance_established
transport_unlinkability_established
registry_authenticity_established
registry_currentness_established
composition_qualified
production_admission_granted
application_authority_granted
```

## Why traffic analysis is explicit

RFC 9458 separates the client address from plaintext request content under its relay/gateway trust model, but traffic analysis is outside its main theorem. Therefore the generic oblivious-relay semantic profile deliberately rejects `traffic_analysis = DeclaredHidden` unless a future, separately frozen transport profile defines and qualifies that stronger property.

## Retention

Derived pseudorandom tags are not automatically harmless. Persisting them can create a new correlation surface. The default v1 requirement therefore demands `ForbiddenPersistence` for client-derived tags.

That still does not prove deletion from external logs, backups, disclosed outputs, or compromised endpoints.

## Qualification boundary

The current tranche is source construction only:

```text
source exists
!= source compiles
!= tests pass
!= Privacy Pass adapter qualified
!= OHTTP adapter qualified
!= registry evidence authenticated
!= composition qualified
```

A separate exact-source qualifier is required before any compile/test PASS claim.
