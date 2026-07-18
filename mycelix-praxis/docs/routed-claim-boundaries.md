# Routed claim boundaries

Several Praxis routes are useful product concepts but do not yet have matching
coordinator contracts. A routed page is not permission to present a prototype
as live network state.

## Verification

Both `/verify` and `/verify/:id` fail closed. They may preserve and display a
credential reference, but they do not call a verifier or produce a positive
result. String length, identifier shape, proof-field presence, and a Holochain
action hash are not verification.

A future positive response requires a shared, versioned contract that:

1. resolves the referenced credential without trusting caller-supplied fields;
2. reconstructs the canonical signed payload;
3. verifies the issuer key and signature;
4. checks subject ownership, expiry, and proof purpose; and
5. consults an authenticated revocation or suspension source.

The existing coordinator `verify_credential` endpoint returns `is_valid: false`
until those checks are implemented.

## Employer portal

No employer-search coordinator contract is present. Local and Live modes show
an unavailable state and claim no candidates, reputation values, skills, ZK
proofs, or mesh results. Demo mode retains fictional candidate cards as a
visual concept with example identifiers and disabled controls.

## Governance

The supplied DAO does not expose the merit-steering, hardware capability-grant,
or prediction-market surface assumed by the original page. Local and Live
modes therefore show an unavailable state. Demo mode labels every proposal,
percentage, multiplier, market, and outcome as illustrative and disables every
vote, stake, and submission control.

## Careers

`/careers` is a local planning surface, not an issuer. Its percentages are
calculated from prerequisite states stored in the current browser profile.
Curriculum mappings, salary and demand values, and local mastery records are
not certification, eligibility, standards compliance, or verification. The
clipboard export carries the same limitations and contains no synthetic DID,
mastery hash, compliance label, community attestation, or verification URL.

## Dashboard economics

Browser-local topic progress is labeled as local progress and is not presented
as mesh-certified. The stewardship ledger and learning-economy card render only
inside the dashboard's Demo branch, below a conspicuous fiction disclosure.
Ledger controls and the learning-economy claim control are disabled. Loading a
local activity counter does not create a TEND balance or entitlement, and the
Demo card performs no conductor verification or claim call.

## Structural gate

Run the dependency-free claim gate with:

```sh
python3 scripts/validate_routed_claims.py
```

This source-level gate prevents the removed hash-length verification,
fabricated talent results, inert governance success paths, credential-like
career exports, and ungated economic simulations from returning. It does not
replace a browser build or real-conductor test.
