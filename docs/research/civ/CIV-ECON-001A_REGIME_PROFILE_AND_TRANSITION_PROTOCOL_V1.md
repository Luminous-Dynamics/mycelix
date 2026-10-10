# CIV-ECON-001A — Economic Regime Profile and Transition Protocol v1

Status: proposed research contract; not an implementation or deployment claim.

Parent architecture: #4804 — CIV-ECON-001, Civilization OS protocol contract.  
Stacked review target: #4809, branch research/civ-econ-001-protocol-contract.  
This companion is intentionally narrower: it makes the profile-to-profile transition boundary executable in principle.

Related work to compose, not duplicate:
- #3936 — AC-053, Economic OS policy profiles and interoperability ABI.
- #3343 and #3347–#3351 — Economic Fabric, external instrument interoperability and conformance.
- #4578 — open-economy / foreign-exchange settlement accounting.
- #4771 / #4772 — Creditism protocol family.
- #4808 — stock-flow, balance-sheet and intergenerational accounting.
- #4806 — power, rights, agency and non-domination.
- #4813 — empirical calibration, identification and uncertainty.

## 1. Purpose and design decision

Mycelix should support heterogeneous economic regimes and mixed arrangements without forcing them into one currency, one policy ideology, or one authoritative ledger. A regime is a versioned policy profile over shared identity, rights, evidence, resource, authority, coordination, instrument, production and dispute infrastructure.

This document does **not** create a competing EconomicPolicyProfile, instrument registry, AssetId, amount type, Finance ledger, or settlement protocol. AC-053 remains the candidate owner of the policy-profile ABI; the existing Economic Fabric and Finance work remain owners of their instrument, event, execution, settlement, and reconciliation semantics. This protocol binds exact references to those owners and defines the transition lifecycle that composes them.

Normative words **MUST**, **MUST NOT**, **SHOULD**, and **MAY** are requirements for a future implementation contract, not evidence that a current implementation already satisfies them.

## 2. The invariant: economic transition is not token conversion

A transition may change property rules, entitlements, labor recognition, pricing, allocation, capital formation, risk allocation, public provision, governance, and external trade. It therefore cannot be modeled as one exchange-rate operation.

Preserve these distinctions:

- regime profile != regime instance != current policy decision;
- instrument != entitlement != ownership claim != authority != resource capacity;
- valuation != unit conversion != exchange execution != settlement != legal discharge;
- authorization != reservation != execution != verified outcome;
- simulation result != independent qualification != adoption authority;
- source-state snapshot != current state unless currentness is proved;
- migration approved != migration executed != migration reconciled != migration complete.

A downstream mapping may narrow a source claim's meaning or use. It MUST NOT silently widen that claim, erase its provenance, or imply additional authority.

## 3. Canonical identities and objects

The protocol composes the following immutable or version-bound objects. Names are semantic roles; implementations should reuse an existing canonical type when one already owns the meaning.

| Object | Required binding |
|---|---|
| Regime profile reference | Canonical profile ID, immutable version, exact content digest, authority/scope, validity, predecessor or supersession reference |
| Regime instance | Instance ID, exact profile digest, participating jurisdiction/community scope, effective interval, local configuration digest |
| Source snapshot | Exact event/frontier identity, state digest, capture time/evidence, known gaps, unresolved conflicts |
| Transition manifest | Source and target profile references, exact source snapshot, target initial-state plan, policy delta, mapping set, approval conditions and claim ceiling |
| Transition attempt | Manifest digest, unique attempt identity, current stage, fencing/version identity, durable progress receipts |
| Economic mapping | Source record/claim/resource identity, exact unit/instrument reference, quantity, disposition, target reference and evidence |
| Effect receipt | Deterministic effect identity, authorized attempt, input/output identity, acknowledgement or typed failure |
| Reconciliation report | Exact source and target snapshots, mapping coverage, stock-flow results, external settlement status, unresolved exceptions and independent verifier identity |

Human-readable labels, ticker symbols, mutable URLs, branch names, and “latest” aliases are not authoritative profile identities. A transition MUST pin immutable versions and content identities before evaluation or approval.

The protocol references the canonical profile and instrument owners instead of copying their entire schemas. A profile reference that cannot be resolved to an exact version and digest remains unresolved and cannot pass qualification.

## 4. Transition lifecycle

The normal lifecycle is monotonic:

**Draft → Frozen → Simulated → Qualified → Authorized → PilotActive (optional) → CutoverReady → CutoverCommitted → Reconciled → Completed**

Exceptional dispositions are **Rejected**, **Aborted**, **Quarantined**, and **Indeterminate**. These are not success synonyms.

| Stage | Entry requirements | Exit evidence |
|---|---|---|
| Draft | Candidate source/target profiles are named | Complete candidate manifest |
| Frozen | Exact profile digests, source snapshot, transition scope and policy delta are pinned | Frozen manifest digest; inventory coverage statement; unknowns preserved |
| Simulated | Frozen inputs and declared simulation profile are available | Reproducible run identity, seeds, environment, output vector and failures |
| Qualified | Independent checks consume the frozen subject, not the model's self-reported result | Verifier receipt bound to exact manifest, scenario corpus, oracle and outputs |
| Authorized | Required affected-party, governance and institutional approvals are valid for this exact plan | Authority decisions bound to manifest digest, scope and validity window |
| PilotActive | Pilot scope, real-vs-simulated effects, exit conditions and dual-accounting rules are explicit | Period-bounded observations and reconciliation of pilot activity |
| CutoverReady | Every in-scope claim, liability, right, reservation and external obligation has a disposition; critical prerequisites are current | Independent readiness report; no hidden pending effect |
| CutoverCommitted | Cutover has crossed the declared effect boundary using deterministic idempotency identities | Durable per-effect receipts; partial completion represented explicitly |
| Reconciled | Source and target projections are independently compared for the declared cutover frontier | Reconciliation report with all discrepancies classified |
| Completed | All required effects are acknowledged or explicitly disposed under an authorized exception policy; appeals and follow-up obligations have owners | Terminal receipt bound to the exact transition and reconciliation report |

The implementation MUST reject impossible or unauthorized stage jumps. Each state change is append-only and binds the previous state digest, next state, actor/authority, evidence, and exact transition attempt. Replaying an already acknowledged effect MUST return the existing result or a defined conflict; it MUST NOT execute the effect twice.

### Abort and indeterminate semantics

Before any irreversible or externally visible cutover effect, a transition MAY be aborted if its policy authorizes that path and the source remains authoritative.

After cutover effects may have occurred, uncertainty MUST NOT be labeled as a clean abort or rollback. The state is **Indeterminate** until exact effect evidence resolves the frontier. Recovery uses idempotent replay, compensation entries where supported, and forward reconciliation. It never deletes history or pretends that an external payment, legal novation, or physical action did not happen.

A UI action, queued workflow, generated plan, present-but-empty receipt, or model recommendation is not stage evidence.

## 5. Required transition plan

### 5.1 Inventory and semantic delta

The frozen plan MUST state what changes and what remains invariant across:
- rights and essential-service guarantees;
- identity and participation eligibility;
- ownership, access and stewardship;
- debt, equity, mutual credit and other liabilities;
- monetary and non-monetary entitlements;
- price formation, labor/contribution recognition and capital formation;
- public budgets, commons funds and risk pools;
- physical resource reservations, inventory and maintenance obligations;
- privacy, data collection and observability;
- governance, dispute resolution, appeals, exit and institutional amendment;
- external trade, payment rails, taxation/reporting and jurisdictional dependencies where applicable.

A regime name such as “market”, “cooperative”, “Creditism”, “commons”, “planned”, or “mixed” is insufficient. The target profile digest and explicit semantic delta define the actual target.

### 5.2 Obligation-by-obligation disposition

Every in-scope source position, claim, obligation, entitlement, reservation and dispute MUST receive exactly one primary disposition:

| Disposition | Meaning |
|---|---|
| Preserve | Continue under the same proven semantics and exact identity |
| Convert | Apply a named, versioned mapping/rate/unit policy with explicit evidence |
| Novate | Change the obligor, beneficiary or governing contract only with the required authority and counterparty acceptance |
| Discharge | Mark satisfied only with adequate discharge/finality evidence |
| Freeze | Preserve the position while disabling specified operations under lawful/authorized controls |
| Dispute | Preserve the record and route it to the declared dispute process |
| Quarantine | Isolate an unresolved or untrusted claim without treating it as zero or valid |
| RetainSourceOnly | Keep the source record historically authoritative without importing it as a target claim |

The mapping MUST preserve claimant/counterparty identity, source provenance, quantity and unit, maturity, priority/seniority, collateral or security where relevant, dispute status, and correction lineage. These fields may be explicitly unknown; they may not be silently guessed.

No in-scope obligation may disappear because it is inconvenient, too old, denominated in a foreign unit, or incompatible with the target model. Write-downs, haircuts, forgiveness, expiry, or altered rights require a named policy, valid authority, affected-party treatment, and an auditable record. The protocol itself does not authorize any of those outcomes.

### 5.3 Conversion and external settlement

A conversion MUST identify:
- exact source and target instrument/unit profiles;
- conversion or valuation rule and version;
- rate source, observation time, validity interval and uncertainty, where applicable;
- fees, spread, rounding and residual treatment;
- execution identity and counterparty;
- settlement rail and its status/finality model;
- reversal, chargeback, reorganization or correction pathway.

A reference basket or FX rate is not a settlement asset. A quote is not an executed trade. An accepted payment instruction is not final settlement. Internal deletion of a community credit does not extinguish an external liability unless the required external discharge is evidenced.

Where source and target units are not semantically equivalent, the transition MUST NOT infer 1:1 parity from matching names, numeric quantities, a shared ticker, or user-interface display.

### 5.4 Physical feasibility and rights floors

Before CutoverReady, the plan MUST reconcile exclusive resource reservations against current source and target inventories and capacities. Authorization is not capacity, a credit issuance is not a produced good, and an allocation record is not proof of delivery.

Rights and essential guarantees remain explicit non-tradable constraints. A target economic balance, reputation, contribution score, employment status, or governance rank MUST NOT automatically remove a person's declared rights floor. If profiles encode different rights or governance rules, the delta must be visible and go through the independently authorized constitutional process, not be hidden inside a currency conversion.

### 5.5 Pilot and dual operation

A pilot MUST declare whether each action is:
- simulated only;
- recorded as a proposal;
- authorized but not executed;
- executed against a source system;
- executed against a target system;
- externally settled;
- reconciled.

Dual operation MUST have one unambiguous authority for each claim and effect. Copying a balance into both systems MUST NOT make both spendable. Reservations and idempotency identities must prevent duplicate fulfillment. The pilot declares its duration, population/scope, safety floor, stop conditions, exit path, data-retention rules and who may stop it.

## 6. Authority separation

The proposer, simulator/model, independent evaluator, adoption authority, executor and reconciler are distinct roles. Where an organization cannot make every identity different, the declared separation policy must state the accepted combination and its risk; a single role may not self-certify every stage by default.

At minimum:
- model output is a candidate, not approval;
- signature validity is not policy legitimacy;
- governance approval is not execution evidence;
- execution evidence is not settlement finality;
- settlement finality is not proof of legal or political legitimacy;
- a current authority decision does not prove a past physical outcome;
- a verifier cannot widen the claim ceiling of the evidence it checks.

Every authorization binds the exact frozen transition manifest, affected scope, expiry/effective period, authority profile and revocation/supersession rules.

## 7. Reconciliation and recovery

The independent reconciliation pass compares source and target at a declared cutover frontier. It checks:
1. all in-scope items have a disposition;
2. preserved positions retain canonical identity and semantics;
3. conversions match their pinned rules and exact arithmetic;
4. claim/debtor/counterparty relationships remain consistent where applicable;
5. external instructions, bookings, settlement and finality are distinguished;
6. no effect is duplicated across retries or the dual-run period;
7. resource allocations and reservations do not exceed the declared capacity envelope;
8. stock-flow accounts close to the declared fidelity or identify every residual;
9. rights, access, privacy and dispute handling remain within the approved target profile;
10. corrections append lineage and do not overwrite original evidence.

All differences are classified as reconciled, explained residual, approved exception, disputed, quarantined, or unresolved. An unresolved difference prevents Completed where it affects authority, rights, liability, settlement, resource exclusivity or the declared acceptance criteria.

Recovery is typed: retry-safe, compensate, reconcile-forward, quarantine, dispute, or require new authorization. “Rollback” is permitted only where the underlying effect can actually be reversed and the reversal itself is evidenced. Historical records remain available under the adopted retention/privacy policy.

## 8. Simulation and qualification programme

Comparisons MUST use the same declared physical world, demand, starting stocks, information regime, shocks and measurement definitions where the question calls for a controlled comparison. Do not give one institution perfect information while another receives delayed observations without declaring that difference.

Initial comparison families SHOULD include:
- conventional market and capital structures;
- market plus social dividend / UBI;
- cooperative and federated ownership;
- time bank and mutual credit;
- commons / polycentric allocation;
- Creditism-style PC/CC flows as a separate adapter;
- public-budget or capacity-constrained planning;
- mixed regimes with and without external settlement.

Required experiment families:
- gradual transition, rapid cutover and dual operation;
- debt/equity/ownership disposition;
- external trade and FX pressure;
- supply, energy, ecological and maintenance shocks;
- insolvency, provider outage, network partition and partial external settlement;
- minority-rights and essential-service challenges;
- governance capture, verifier capture, bridge centralization and cross-regime extraction;
- adverse outcomes, null outcomes and exit by affected participants.

Report a vector, not a mandatory winner score. At minimum retain essential access, unmet demand, resource utilization, ecological overshoot, housing access, purchasing-power and productive-asset concentration, authority and verifier concentration, debt burden, maintenance continuity, innovation/entry/exit, resilience, external dependence, migration cost, privacy burden and dispute burden.

A simulation result is only as strong as the scenario and oracle that produced it. The evaluator must not accept the simulator's own unverified claim of correctness as its oracle.

## 9. Required acceptance gates

A future implementation may claim only the gates it actually executed. At minimum, the test harness should verify:

1. profile references pin exact versions/content identities;
2. stale source frontier or changed manifest invalidates prior qualification and approval;
3. every scoped liability/entitlement/reservation gets exactly one explicit disposition;
4. conversion without an authorized mapping profile fails closed;
5. parity, ticker or matching display label cannot establish equivalence;
6. duplicate request/retry does not duplicate effects;
7. partial external settlement remains pending/indeterminate, not final;
8. target balances do not authorize spending absent target authority and current capacity;
9. a copied dual-run claim cannot be spent twice;
10. unresolved mandatory items block CutoverReady or Completed;
11. rights floors cannot be bypassed by economic rank or reputation;
12. correction/reversal appends lineage instead of rewriting the original;
13. cross-regime recognition does not implicitly mint a different instrument;
14. the independent evaluator consumes the exact frozen subject and does not merely trust candidate-produced outcomes;
15. failure, unknown and indeterminate states remain first-class outputs.

Machine-readable companion artifacts:
- CIV-ECON-001A_TRANSITION_MANIFEST_SCHEMA_V1.json — structural schema for a transition manifest;
- CIV-ECON-001A_TRANSITION_ADVERSARIAL_CORPUS_V1.json — fail-closed adversarial cases and expected dispositions.

JSON Schema validates structure only. It does not prove signatures, distinct authority identities, causal completeness, external settlement, resource truth, legal discharge or real-world effectiveness. Those require independent validators and evidence at the appropriate layer.

## 10. Existing standards and empirical boundary

The Valueflows ontology provides shared concepts for Intent, Commitment, Claim, Economic Event, Economic Resource and quantified flows; it is useful as an interoperability vocabulary, not as a substitute for Mycelix authority or finality rules: https://www.valueflo.ws/specification/all_vf.html

The BIS Project Agorá 2026 prototype demonstrated atomic wholesale cross-border settlement using tokenized central-bank reserves and commercial-bank deposits across currencies and jurisdictions; the BIS explicitly keeps the underlying legal character of those liabilities and obligations intact. This is a useful precedent for carefully scoped settlement interoperability, not evidence that retail payments, Mycelix instruments, or all economic regimes are already interoperable: https://www.bis.org/publications/project-agora-shared-programmable-platform-wholesale-cross-border-payments

The protocol therefore treats external payment and economic standards as adapters with pinned profiles and claim ceilings. It does not treat adoption of a data standard as proof of regulatory compliance, settlement finality, legal ownership, macroeconomic stability or ethical legitimacy.

## 11. Claim ceiling

This document defines a transition contract and proposed qualification criteria. It does not establish that the current Mycelix code implements this state machine, that any transition has passed the gates, that any economic regime is optimal, or that the protocol is legally enforceable or production-ready.

Promotion path: authored specification → schema/corpus validation → executable reference validator → independent adversarial qualification → cross-domain integration → synthetic transition scenario → bounded pilot design → real-world operation only with its own lawful authority and safety evidence.
