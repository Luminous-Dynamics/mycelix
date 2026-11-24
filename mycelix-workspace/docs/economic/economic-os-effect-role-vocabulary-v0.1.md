# Economic OS effect-role vocabulary and claim templates v0.1

**Status:** typed role vocabulary and reusable claim-profile constructors. These are not policy approvals, authorization grants, or production domain adapters.

## Design rule

An economic effect role names one domain-owned mutation or assertion. The operation ID identifies the obligation; the effect ID identifies one `(operation_id, role, owner)` slot using the existing FIN-EFFECT-001 algorithm. The claim-profile digest binds the exact required role/owner set. No new operation/effect identity algorithm is introduced.

The vocabulary is extensible by adding new role variants with stable namespaced strings. Existing Finance v1 strings are frozen and are not renamed or reinterpreted:

- `payer_debit`
- `beneficiary_output`
- `beneficiary_claim`
- `treasury_fee`
- `pool_contribution`
- `pool_disbursement`
- `external_settlement_observation`

New namespaced role families:

| Namespace | Examples | Meaning boundary |
|---|---|---|
| `hearth.*` | member note spend, pool contribution receipt, allocation reservation, pool disbursement | Member-owned and pool-owned effects remain distinct |
| `tend.*` | provider credit, recipient debit, mutual-credit exposure, repayment, clearing adjustment | Mutual-credit positions are not SAP or fiat settlement |
| `grant.*` | appropriation, budget reservation, award, tranche release, recipient claim, recovery | Approval/reservation/release/recipient claim remain separate |
| `procurement.*` | order commitment, goods/service receipt, supplier liability, payable journal, payment instruction, bank settlement receipt | Invoice, delivery, payable, instruction and settlement are separate |
| `payroll.*` | earning accrual, withholding liability, net payable, release, bank payment, worker receipt | Payroll calculation, release, rail payment and receipt remain separate |
| `public_transfer.*` | eligibility, entitlement, budget reservation, disbursement, recipient claim, appeal/recovery | Eligibility is not payment or continuing eligibility |
| `resource.*` | reservation, provider commitment, delivery event, beneficiary acceptance, quality exception | Capacity/reservation/delivery/acceptance are distinct |
| `reputation.*` | attestation, projection, correction, revocation | Reputation is non-transferable and never becomes money or execution authority |
| `external.*` | instruction, provider acceptance, ledger observation, reversal | External message/status is not automatically final settlement |
| `valueflows.*` | intent, commitment, claim, EconomicEvent, EconomicResource | Semantic mapping preserves native meaning; it does not grant authority |

## Claim template constructors

The Rust crate exposes nine versioned EconomicClaimTemplateV1 presets. Each binds a required role set to caller-supplied owner identities and creates a digest-verified CompositeClaimProfileV1.

| Template | Required role set | Claim scope |
|---|---|---|
| SAP bilateral transfer reconciled | payer debit + beneficiary claim | Both owner effects observed |
| Hearth contribution accepted | member note spend + pool contribution receipt | Member input and pool receipt observed |
| Hearth withdrawal claimed | pool disbursement + beneficiary claim | Pool disbursement and recipient claim observed |
| TEND bilateral clearing | provider credit + recipient debit + mutual-credit exposure | Both credit legs and exposure observed |
| Grant recipient claimed | award + budget reservation + tranche release + recipient claim | Approval, reservation, release and recipient claim observed |
| Procurement received and settled | order + goods receipt + supplier liability + payable + payment instruction + bank settlement receipt | Each procurement and settlement effect observed |
| Payroll worker receipt observed | net payable + payroll release + bank payment + worker receipt | Payable, release, rail payment and receipt observed |
| Commons service accepted | resource reservation + provider commitment + delivery + beneficiary acceptance | Reservation through beneficiary acceptance observed |
| Reputation attestation recorded | attestation + reputation projection | Issuer attestation and derived projection recorded |

The caller must supply each required role exactly once with its actual owner identity. The constructor rejects a missing required role, a role not included by the selected template, duplicate bindings for the same role, and invalid owner components.

Bindings are sorted deterministically before the canonical FIN-EFFECT-004 profile digest is derived. Template IDs and claim names are versioned and included in that digest.

## What templates do not do

A template is a reusable shape, not a pre-approved claim profile. It does not authenticate an owner identity or infer it from a DID string; establish membership, allocation, procurement, employment, grant or benefit authority; verify a policy/jurisdiction revision; prove the template is semantically complete for a particular law, contract or purpose; commit a domain-owned state mutation; authenticate source evidence or prove completeness of a DHT/rail view; or establish bank settlement, legal discharge, resource title, delivery quality or causal service outcome.

Domain owners and qualified authorities must approve a real profile and provide authenticated evidence. A scoped support result means only that supplied evidence covers the required effect slots of that exact profile.

## Benchmark and tests

The machine-readable catalogue and closed Draft 2020-12 schema define the 9 templates, 59 stable role strings and owner-binding slots. The exact-head workflow checks JSON syntax and selected catalogue invariants; template and role ID uniqueness; required-role and owner-slot parity; standalone/workspace mirror bytes; and the Rust kernel tests, formatting, Cargo lock immutability and strict Clippy.

Targets, not measured outcomes: 100% template determinism for identical role-owner bindings; zero missing, unexpected or duplicate bindings silently accepted; zero changes to existing FIN-EFFECT-001 known-answer vectors; zero false SupportedWithinDeclaredScope results on at least 20 cross-domain adversarial profiles; report p50/p95/p99 profile construction and projection latency by role and receipt count.

A full Draft 2020-12 validator and hidden, independently adjudicated evaluation set remain follow-up work. The public catalogue and corpus are development fixtures, not performance evidence.

## Claim ceiling

The role catalogue and templates broaden the vocabulary Mycelix can represent. They do not establish a production Economic OS adapter, policy approval, authorization, legal ownership, accounting or tax compliance, global uniqueness, external settlement, physical delivery or production readiness.
