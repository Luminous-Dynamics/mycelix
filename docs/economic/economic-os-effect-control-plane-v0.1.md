# Economic OS effect control plane v0.1

**Status:** compositional architecture and conformance contract, not a production implementation or benchmark result.

## Paradigm shift: federated effects, not one universal transaction

The Economic OS should not become one giant mutable balance table or one universal transaction state machine. It should be a federated network of effect owners coordinated by shared identity, authority, policy and evidence contracts.

A SAP transfer, TEND time-credit, Hearth withdrawal, grant award, procurement liability, payroll payment, water delivery and external bank settlement do not share one owner or one definition of completion. They can share a rigorous description of the obligation and each owner-specific effect.

Keep six questions separate: (1) obligation identity; (2) immutable intent integrity; (3) effect role and owner identity; (4) admission authority and epoch; (5) local owner commit; (6) evidence and finality.

A hash is not authority. A stable operation ID is not a global lock. A signed receipt is not necessarily complete evidence. A payment message is not settlement finality. Model recommendations never grant execution permission.

## Core abstraction: an effect vector

Maintain independently evidenced components keyed by operation ID, effect role and effect owner. Each component carries identity, authority, admission, owner commit, evidence and finality dimensions. This is a partial-order/evidence-vector model, not one linear global status.

Examples: payer debit may be committed while beneficiary claim is pending; a grant award may be approved while funds remain unreserved; an invoice may exist while goods receipt is absent; a resource may be reserved while physical fulfilment remains outstanding; a TEND clearing pair is not automatically fiat settlement.

A composite projection may summarize the vector, but must retain each effect status and hold reason. 'Completed' or 'settled' is valid only when the relevant owner and finality profiles establish it.

## Portable envelope, not a parallel ledger

Proposed fields: schema/protocol profile; stable operation and obligation reference; immutable intent fingerprint; effect role/owner/effect ID; asset/resource and exact unit profile; exact quantity; issuer/network/contract/account/resource identity; authority profile/epoch/signers/transition certificate where required; policy/jurisdiction revision and valid-time/knowledge-time frontiers; causal parents and owner commit; assertion-specific evidence, completeness frontier and contradiction state; settlement/delivery/finality profile and claim ceiling; append-only corrections, reversals, reorgs and restatements.

Do not put private keys, bearer secrets, raw identity credentials or unnecessary personal data in the portable envelope. Use access-controlled evidence references.

## Domain ownership crosswalk

| Domain | Distinct owner effects | Non-collapse rule |
|---|---|---|
| SAP | member spend, beneficiary claim, fees, demurrage, pool allocation | SAP unit profile remains explicit; demurrage/pool effects are not hidden transfer arithmetic |
| TEND | provider credit, recipient debit, exposure, repayment, clearing | mutual credit is not SAP or fiat; matched entries are not necessarily cash settlement |
| MYCEL | attestation, reputation projection, correction/revocation | non-transferable reputation never becomes money, entitlement or execution authority |
| Hearth | member note spend, pool receipt, reservation, disbursement, recipient claim | membership is not entitlement; pool debit is not recipient claim |
| Grants/budgets | appropriation, reservation, award, tranche, recipient claim | approval is not reservation, disbursement or receipt |
| Procurement | order, receipt, supplier liability, payable, instruction, bank settlement | invoice is not goods receipt or final settlement |
| Payroll | earnings, withholding, payable, release, bank payment, worker receipt | calculation, approval, journal, settlement and receipt differ |
| Public transfers | eligibility, entitlement, reservation, disbursement, recipient claim | eligibility is not an award or proof of payment |
| Commons services | reservation, provider commitment, delivery, acceptance, quality claim | capacity, availability, reservation, delivery and acceptance differ |
| External assets | instruction, provider acceptance, ledger observation, owner effect, settlement, correction | ticker is not asset identity; token balance is not legal ownership |
| Valueflows | Intent, Commitment, Claim, EconomicEvent, EconomicResource mapping | semantic mapping does not itself prove authority, ownership or delivery |

See the JSON crosswalk for 11 domain profiles and 18 qualification metrics.

## Compose rather than duplicate

- FIN-EFFECT-001 / #6588: stable obligation identity, immutable intent fingerprint, per-role/per-owner effect IDs.
- FIN-EFFECT-002 / #6593 and PCOE #6595: admission authority, quorum assumptions, separation of certificate, owner commit and spendability.
- FIN-EFFECT-003 / #6596 and PCOE #6601: bounded candidate epoch handover; not a production witness registry or revocation service.
- FIN-SAFE-020 / #6575: Hearth member-owned value-note spend, pool receipt/disbursement and beneficiary claim.
- AC-055 / #3942: jurisdiction packs over one semantic kernel, not country-specific kernel forks.
- FIN-POLICY-EXEC-001 / #5792: bitemporal deterministic source-compiled policies.
- FIN-ACC-EVIDENCE-001 / #5798 and FIN-ACC-EVAL-001 / #5799: assertion-specific evidence and oracle-isolated evaluation.
- MYC-ECO-002 / #3343 and FIN-GLOBAL-001 / #5811: external instruments, Valueflows, accounting and jurisdiction profiles.

FIN-EFFECT operation identity is not a universal account, asset, policy, invoice or journal ID. It links an obligation to effects while preserving each domain's canonical IDs and owner boundaries.

## Composition invariants

1. Identity survives adapters; exact retries resolve original operation, semantic changes conflict.
2. One effect role/owner has one effect identity; coordinators cannot impersonate owners.
3. Authority precedes admission; admission precedes owner commit; finality is independently evidenced.
4. Local absence is not global non-existence; incomplete DHT/registry views remain unknown or indeterminate.
5. Asset, issuer, network, contract, unit scale, origin and provenance survive translation; lossy mappings are explicit.
6. Conservation is per instrument/unit; fees, demurrage, reserves, rounding and mutual-credit legs are explicit.
7. Historical policy replay binds valid time and knowledge time; later rules do not silently rewrite history.
8. Evidence is assertion-specific; completeness, authority, contradiction and freshness stay distinct.
9. Corrections append lineage; they do not rewrite economic history.
10. Symthaea, forecasts and simulations may propose or explain, but cannot mint authority or journal effects.
11. Adapters may narrow claim ceilings, never widen them without new evidence and an approved profile.
12. Safety and liveness are separate; quorum loss may hold operations, never lower thresholds.

## Benchmarks and SOTA strategy

Targets are not measured results: 100% exact retry resolution; 100% detection of declared immutable intent-field mutations; zero duplicate owner effects under in-profile replay/race/restart; zero conflicting valid certificates under stated assumptions; zero unilateral epoch changes; zero stale/premature epoch effects; zero unexplained conservation residual; zero silent asset/unit loss; zero historical replay mismatch on frozen inputs; zero false spendability/finality promotion.

Report throughput, p50/p95/p99 latency, recovery time, partition liveness, time in Indeterminate, storage/network cost and operator burden. Do not set a universal latency goal before matched baselines by domain/profile exist. The metric catalogue distinguishes hard safety gates from measurements.

Compare layers fairly: Stripe/Adyen/PayPal for idempotency scope and retention; Temporal for durable orchestration/retries (external Activity effects still need idempotency); transactional outbox for local atomicity with at-least-once delivery; Raft joint consensus and CometBFT validator-set activation for membership transitions; ISO 20022, Peppol, XBRL, SNA, BPM7, GFSM, SEEA, SDMX and Valueflows for interchange/reporting; BIS Project Agora as an advanced institutional comparison, not apples-to-apples.

Do not claim Mycelix beats institutional wholesale settlement maturity. The defensible differentiation is an open assurance fabric spanning community currencies, mutual credit, commons resources and institutional adapters, with exact identity, owner boundaries, evidence completeness, bitemporal policy and explicit finality.

## Execution order

1. Complete exact-head epoch-transition TLC and named mutants.
2. Crosswalk against Hearth, SAP and TEND before adding new IDs.
3. Add property tests for conservation and semantic preservation.
4. Add multi-agent Holochain owner-chain race, fork/incomplete-DHT and restart tests.
5. Implement durable journals, signer alias resolution, rotation/revocation and rollback-resistant recovery before quorum-backed spendability.
6. Build oracle-isolated candidate/evaluator sets with matched expert and system baselines.
7. Add domain adapters incrementally; unsupported semantics fail closed.

## Claim ceiling

This is a design and conformance contract. It does not establish legal ownership, solvency, tax/accounting compliance, bank settlement, physical delivery, fairness, global uniqueness, production BFT or production readiness. Every domain retains its owner, authority, legal scope, instrument profile and acceptance criteria.
