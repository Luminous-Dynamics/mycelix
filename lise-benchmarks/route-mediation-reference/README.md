# LISE route mediation reference evaluator

**Status:** synthetic finite-graph research reference. It is not a real provider route scanner, PEP, authorizer or production non-bypassability certificate.

This crate answers a narrow but important question:

> Does every effect-capable path from the declared principal entry points to a protected resource cross a currently valid enforcement point that is bound to this exact principal, action, purpose, resource, operation and policy?

It enumerates every simple effect-capable route in a bounded directed graph. A path is counted as mediated only when a PEP node on that path has a matching owner-side mediation receipt whose validity and observation times cover the action and knowledge cutoffs.

## Run

    cargo fmt --manifest-path lise-benchmarks/route-mediation-reference/Cargo.toml --check
    cargo check --all-targets --manifest-path lise-benchmarks/route-mediation-reference/Cargo.toml
    cargo test --all-targets --manifest-path lise-benchmarks/route-mediation-reference/Cargo.toml
    cargo run --manifest-path lise-benchmarks/route-mediation-reference/Cargo.toml -- \
      lise-benchmarks/route-mediation-reference/fixtures/route-inventory.json \
      /tmp/route-mediation-report.json

## Result semantics

- \`ALL_PATHS_MEDIATED\`: every enumerated path in the supplied, receipt-bound finite graph crosses a matching PEP.
- \`BYPASS_DETECTED\`: at least one enumerated path reaches the protected effect without such a PEP.
- \`INDETERMINATE\`: the independent inventory receipt is incomplete, stale, mismatched or untrusted; or path enumeration exceeds its bound.
- \`NO_REACHABLE_EFFECT_PATH\`: no path was found in this graph. This is not a claim that no real-world path exists.
- \`INVALID_INPUT\`: duplicate/dangling graph identities or malformed fields.

The bundled synthetic inventory deliberately contains one mediated route and two bypasses (a direct API route and a background/webhook route). The all-mediated positive control is separately tested.

## Scope and trust boundary

The evaluator validates that the inventory receipt binds the graph digest, principal, action, purpose, resource, operation digest and policy digest. It requires an observer, exact scope, observation cutoff and coverage interval. However, the fixture receipt is an untrusted synthetic assertion. The evaluator cannot establish that the route inventory was actually complete, that the PEP truly enforced the policy, or that a provider effect matched its trace.

A production qualification requires an independently sourced owner inventory, connector/IAM/cloud control-plane evidence, PEP logs, direct-provider path discovery, active probing where authorized, and independent protected-effect observations. Unknown routes must force INDETERMINATE rather than a clean pass.

The path enumerator uses simple paths, bounded to 256 nodes, 1,024 edges, 2,048 routes and 128 nodes per route. Exceeding a bound is INDETERMINATE; the tool does not silently truncate and pass.

## Claim ceiling

No real route discovery, provider integration, authentication, external side effect, security certification, or production non-bypassability claim is made.
