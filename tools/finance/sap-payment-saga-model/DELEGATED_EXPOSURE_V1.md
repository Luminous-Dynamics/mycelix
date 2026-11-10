# Delegated exposure escrow reference model v1

**Status:** effects-disabled Rust/Go reference protocol. This is not a production distributed reservation primitive, owner-authenticated grant, liquidity facility or credit line.

## Why delegation is needed

Independent per-destination books do not enforce a common parent cap. If a pool has 1,000 minor units of exposure headroom and separately grants two destinations 800 each, each destination can pass a local 700-unit reservation while the parent pool is exposed to 1,400.

A single atomic transaction spanning multiple independent Holochain source chains should not be assumed. The conservative alternative is parent escrow with delegated sublimits.

## Protocol

A parent owner authorizes child budgets from one parent pool/currency scope. The parent reserves the *entire child budget* against its own cap while that grant is Active or Revoking.

Each grant binds:
- parent pool and currency;
- grant ID and monotonic grant sequence;
- exact child owner, pool, destination and currency;
- delegated maximum amount;
- grant digest.

The child receives an independent ExposureBook whose aggregate and per-effect caps are both bounded by the delegated amount.

### Grant lifecycle

| Status | New child reservation | Existing effect resolution | Parent budget encumbered |
|---|---|---|---:|
| Active | Allowed within child cap | Allowed | Yes |
| Revoking | Rejected (exact existing retry remains idempotent) | Allowed | Yes |
| Closed | Rejected | No further transitions | No |

Revocation does not release the budget. It only blocks new risk-increasing reservations. Existing dispatched, indeterminate and committed-unsettled effects remain represented in the child book and must be settled or authoritatively absent before the parent may close the grant.

A closed grant remains in canonical history and cannot be reactivated. A new grant requires a new grant ID and a higher owner-local grant sequence.

## Parent accounting

\`allocated_parent_budget = sum(budget of Active and Revoking grants)\`

\`available_parent_budget = max(0, parent_limit - allocated_parent_budget)\`

This is intentionally conservative. Unused child sublimits remain encumbered until the grant is explicitly closed. That sacrifices dynamic utilization but prevents multiple destinations from independently spending the same parent headroom while partitioned.

The parent can reallocate a closed budget through a new grant. This reference does not implement cross-owner atomic rebalancing or mid-flight budget shrink.

## Retry, finality and reconciliation

- An exact retry for an existing effect resolves idempotently, including while the grant is Revoking.
- A changed intent under the same effect ID conflicts.
- New effects cannot reserve under Revoking or Closed grants.
- Final settlement and authoritative absence are distinct typed resolution kinds.
- A provider acceptance acknowledgement is not final settlement evidence.
- Reconciliation continues during revocation.
- Parent escrow cannot be reclaimed while any child exposure remains.

The child close check in this in-memory model inspects the child book directly. Production must replace it with a child-owner-authenticated close report bound to the grant, child snapshot digest, owner-local frontier and policy version; the parent must validate that report before releasing escrow.

## Canonical snapshot

Domain: \`MXSAP-DELEGATED-EXPOSURE-V1\` followed by a zero byte.

The snapshot binds parent owner/pool/currency, policy sequence and limits, policy digest, and all grants sorted by grant ID. Each grant binds exact child scope, grant digest, delegated amount, sequence, status and the child's canonical exposure snapshot digest.

The shared fixture \`fixtures/sap-payment-delegated-exposure-v1.kat\` pins the parent snapshot. Rust and Go independently verify the same vector. The expected SHA-256 was calculated separately from both implementations.

## Adversarial coverage

- Two provider destinations cannot jointly exceed the parent cap by independently claiming sublimits.
- Child outstanding effects cannot exceed its delegated budget.
- Revocation blocks new risk while allowing exact retries and resolution.
- An unresolved child effect prevents grant closure and parent budget reclamation.
- Final settlement and authoritative absence use separate resolution profiles.
- Changed grant terms, wrong pool/currency, stale sequences and noncanonical inputs reject.
- Parent snapshot digest is insertion-order independent and cross-language KAT-bound.

## Qualification boundary

The Rust and Go parent books are serial in-memory reference models. They do not prove:
- owner-authorized Holochain grant creation or native source-chain sequence uniqueness;
- atomic parent budget allocation under real concurrent source-chain writes;
- child-owner authentication of close reports;
- distributed revocation propagation or partial-DHT convergence;
- provider receipt authenticity, authoritative absence or actual settlement finality;
- bank liquidity, collateral sufficiency, solvency or legal netting.

Production requires an owner-native parent grant action, child-local reservation actions, authenticated close reports, restart/recovery tests and real multi-agent contention qualification. There is no global Luminous authority over independent owners.
