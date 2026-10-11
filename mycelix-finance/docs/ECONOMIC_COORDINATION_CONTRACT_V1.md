# Economic Coordination Contract v1

**Status:** proposed cross-component contract; not yet implemented or qualified.
**Purpose:** bind a human need, a real resource/capacity claim, legitimate authority, economic obligations, rights constraints, ecological budgets, fulfilment, and evidence to one versioned transaction identity.
**Design rule:** this contract coordinates claims about reality; it does not make those claims true merely because they are serialized.

## 1. The boundary it is meant to close

A currency ledger can be internally consistent while the promised goods do not exist. A governance vote can be valid while the allocation violates rights or ecological limits. A resource reservation can be correct while the associated debt has no feasible settlement path. A price can be numeric but stale, misdenominated, or fabricated. An external effect can have happened even though its caller never observed the receipt.

The contract therefore models four distinct classes of fact:

1. **Financial facts:** balances, instruments, obligations, due dates, reserves, conversion profiles, and settlement records.
2. **Physical/social capacity facts:** resource identity, amount/unit, place, time window, custody, availability, service capability, and fulfilment.
3. **Authority/rights facts:** who was permitted to decide, the policy version applied, consent, need floors, due process, dispute and appeal routes, and safe exit.
4. **Evidence/uncertainty facts:** the source, observation time, trust profile, digest, freshness, independent verification, and whether a fact is Unknown, Stale, or Contested.

They may be linked by one transaction ID, but must not be collapsed into one status or one scalar score.

## 2. Proposed top-level envelope

The transport-neutral envelope should carry these fields. Concrete Rust/Serde names may differ during implementation, but their semantics must not.

| Field | Required meaning |
|---|---|
| schema_version | Explicit contract version; unknown versions fail closed rather than guessed down-conversion. |
| transaction_id | Stable, opaque identity for this need/offer/fulfilment lifecycle. Retries and revisions reference the same identity or a declared successor. |
| community_id | The community that owns the decision policy and records which rules govern the transaction. |
| subject_refs | Minimal identities for requester, provider, affected party, and responsible organization, using capability-limited references where possible. |
| created_at / expires_at | Time window for authorization and reservations; expiry must be checked by the authority responsible for accepting the transition. |
| need | Need category, quantity, unit, acceptable delivery window, location/scope, priority class, and the privacy classification for its details. |
| resource_claims | One or more typed resources and capacity intervals, not merely a monetary price. Every quantity carries its unit and source. |
| authority | Decision ID, policy digest, authorized actor/capability, time, jurisdiction/community, and the applicable challenge/appeal route. |
| rights_constraints | Named hard floors, consent/participation condition, applicable due process, and blockers that cannot be offset by aggregate benefit. |
| ecological_constraints | Resource/impact budget IDs, quantity and unit, geography, period, measurement/estimate class, and remaining capacity under the relevant budget. |
| instrument_terms | Instrument ID, amount, precision/unit, account roles, conversion profile, fee schedule, and any reserve or credit-limit policy used. |
| obligations | Debtor, creditor, principal, instrument/denomination, due dates, settlement source/plan, seniority or dependency, default path, and who bears residual loss. |
| valuation_evidence | Source/asset identity, denomination, observation time, freshness limit, source digest, verification profile, and status: Verified, Unknown, Stale, or Contested. |
| external_effect | Effect identity, exact request digest, provider profile, idempotency-binding reference, and adapter receipt/evidence state. Secret keys should not be copied into public transaction records. |
| lifecycle | Current state, prior-state digest, transition decision, reservation/fulfilment/settlement receipts, dispute state, and unresolved mandatory items. |
| evidence_manifest | Digests of inputs, policy, source evidence, transition outputs, independent evaluator, and environment where reproducible evaluation applies. |

A field's presence is not proof of its truth. Each evidence field needs an explicit trust source and verification rule. An unsigned caller-supplied value must not become Verified simply because it matches another caller-supplied value.

## 3. Typed quantities and uncertainty

Every amount or capacity must include a unit and an economic meaning. For example, “500” without the denomination, precision, unit, place, and interval is not an admissible resource claim. Currency amount, stored collateral value, forecast production, current inventory, hours of care, and tonnes of material are not interchangeable quantities.

Valuation status must never be inferred from a number alone:

- **Verified:** the required authority/source, unit, freshness, and evidence checks succeeded.
- **Unknown:** required evidence is absent or an adapter/measurement failed.
- **Stale:** evidence exists but is outside the declared freshness window.
- **Contested:** valid parties/evidence conflict or a dispute is open.

Unknown, Stale, and Contested are not aliases for zero, healthy, solvent, insolvent, delivered, or permitted. A transition that depends on one of these facts must wait, choose a declared safe alternative, or fail with a reason. It must not manufacture a numeric default.

Forecasts must be labelled Forecast, with a model/version, confidence or uncertainty representation, and horizon. Observations must be labelled Observed and retain provenance. A forecast does not become proof of delivery.

## 4. Lifecycle and transition authority

The minimum logical lifecycle is:

1. **Proposed:** a need/offer/request exists; no resource is promised and no external effect is authorized.
2. **Authorized:** the responsible community policy approves the decision, and the record binds the authority, rights checks, and policy digest.
3. **Reserved:** the system has an explicit capacity reservation for the given unit and time interval. Reservations must be concurrency-safe, expire, and be released through a recorded transition.
4. **Committed:** counterparties accept the obligation and cancellation/default/dispute terms; an external effect may now be possible only through the designated adapter and capability.
5. **Delivered:** source evidence claims that the real good/service/resource transfer occurred.
6. **Verified:** a verifier independent of the candidate action has checked the delivery and its quantity/quality criteria.
7. **Settled:** the instrument ledger records the authorized settlement, fees, reserve/credit effects, and any remaining obligation.
8. **Disputed:** a party challenges authority, quality, delivery, valuation, or accounting. The contested part must be visible, and appeal/recourse remains accessible.
9. **Indeterminate:** an external effect may have happened but the available evidence is insufficient to decide. It is not safe to equate this state with either “did not happen” or “success”.
10. **Cancelled / Failed:** only permitted when the evidence proves which commitments can be released and no unaccounted external effect remains. If an effect may have started, retain Indeterminate until reconciled.

Every transition must identify the rule that permitted it, its inputs, and the decision authority. Candidate policy code may propose a transition; it cannot serve as its own independent authority or verifier.

## 5. Non-negotiable economic invariants

### 5.1 Stock-flow and obligations

- Every instrument's issuance, transfer, fee, reserve change, cancellation, default, and write-off has a defined accounting rule.
- A transfer is not automatically value creation. Count real delivery separately from financial movement.
- New credit is an obligation and a corresponding claim under the instrument's accounting model; it must not be reported as newly fulfilled real need merely because an account balance rose.
- Refinancing or maturity extension creates a new recorded transition in the same obligation chain. It is not repayment, profit, or risk elimination.
- Debt sustainability scenarios must state the plausible settlement source and horizon; stress refinancing freezes, correlated defaults, reduced income, and lower collateral values.
- A rising asset price alone cannot establish solvency. Keep liquidity, solvency, valuation confidence, and physical service capacity as separate dimensions.
- No obligation may lose its principal, owner, maturity, seniority/dependency, or loss bearer through a migration or policy update without an explicit governed transition and retained predecessor link.

These rules do not prohibit credit or investment. They prohibit silently counting a growing chain of financial claims as if it were already delivered value.

### 5.2 Physical capacity and ecological budgets

- A reservation cannot exceed verified feasible capacity for a resource, unit, location, and interval.
- Capacity claims require custody/availability authority and a source; the contract must guard against double-booking, duplicate issuance, stale inventory, and conflicting replicas.
- Relevant absolute material, energy, land, water, emission, waste, or ecosystem-impact limits must be represented by budget identity, geography, period, and uncertainty. Intensity metrics alone cannot establish a reduction in total impact.
- Reuse/recycling assumptions must account for measured recovery rate, quality loss, energy demand, residual waste, and time; circularity cannot be assumed to be lossless.
- Resource budgets are not minted by creating a currency, reputation point, offset, or credit. A permitted offset must satisfy the separately specified evidence and accounting rules; it cannot waive a hard ecological threshold by label alone.
- When feasible supply is below requested need, the system must expose unmet need and the allocation rule. It must not invent capacity or hide scarcity as a price-only result.

### 5.3 Rights, violence, and externalized harm

- Rights floors and due-process requirements are hard constraints, not soft terms in a welfare score. Gains in output, exchange volume, or asset value cannot compensate for forced labour, violence, unlawful dispossession, retaliation, or denial of essential needs as leverage.
- Consent must be informed, revocable where applicable, and free from threats or arbitrary exclusion. Exit must include a safe state-export/dispute path; exit cannot be conditioned on surrendering unrelated rights.
- Debt enforcement, eviction, job loss, service withdrawal, and dispute remedies must identify the authority, evidence threshold, notice, proportionality, appeal, and who monitors misuse.
- Track material burdens on people who do not appear as parties to the transaction: unpaid care/time, workplace danger, pollution exposure, displacement, downstream remediation, public subsidy, and future liabilities where data can support it.
- An impact ledger makes burdens visible; it does not turn a rights violation into an acceptable purchasable cost. Uncertain or missing harm data must be disclosed rather than treated as zero.
- Record who benefits, who bears risk/cost, whether they had effective voice, and the available remedy. An optimizer must not gain a higher “efficiency” score by moving coercion or harm outside its accounting boundary.

### 5.4 Interoperability and sovereignty

- Every community retains control over its authority/policy version, local resource budgets, disclosure rules, dispute process, and exit terms, subject to the obligations it has validly accepted.
- Cross-community transitions identify both sides' policy versions, units, settlement terms, exchange/conversion evidence, timeout/reconciliation rules, and disagreement path.
- No always-online global authority should be assumed for a local essential service. Partitions must surface bounded uncertainty and prevent duplicate reservations/settlement; they cannot silently create capacity or erasure.
- Minimize shared personal data; expose attestations/capabilities when they are sufficient instead of copying whole private dossiers into a global log.
- Migration and exit preserve open obligations and valid disputes without forcing a community to surrender unrelated private data or local policy.

## 6. Evidence and verification contract

Every material transition should generate a receipt binding:

- transaction ID and schema version;
- predecessor-state digest;
- requested transition and outcome;
- policy/authority digest and relevant capability reference;
- resource, obligation, valuation, rights, and ecological evidence digests used;
- explicit unresolved facts and why they did not block or did block the transition;
- verifier identity/version and decision digest;
- commit/build identity and reproducible environment for qualification runs.

Receipts prove what was evaluated, not that a dishonest upstream source reported truth. External-source authentication and physical delivery verification remain separate responsibilities. The candidate must not generate the expected output and the sole acceptance verdict.

## 7. Minimum conformance scenarios

1. **Rollover freeze:** revenue falls, obligations mature, lenders stop refinancing, and collateral prices drop. Verify that rollover is not counted as repayment, losses remain attributed, and minimum needs are handled by explicit policy.
2. **No-capacity booking:** two callers try to reserve the final unit of water/energy/housing/care capacity. Exactly one valid reservation succeeds; the other sees scarcity and recourse.
3. **Unknown valuation:** oracle outage, stale quote, invalid denomination, or conflicting reports. No automatic healthy, liquidation, insolvency, or new credit decision may rely on missing evidence.
4. **Component fraud:** inconsistent component values, asset types, weights, aggregate, or diversification bonus are rejected; valid internal arithmetic still does not imply authenticated valuation.
5. **External-effect crash:** the system crashes after dispatch may have occurred and before receipt persistence. It reconciles from the original attempt identity and provider evidence, not from an invented new idempotency key.
6. **Externalized harm:** a lower-priced option depends on unsafe labour, coercive eviction, displacement, or unrecorded pollution. A rights violation is a hard failure; other impacts and uncertainty are disclosed.
7. **Ecological rebound:** per-unit impact falls while absolute output and total impact rise. The evaluator reports intensity and absolute totals and does not declare ecological improvement from intensity alone.
8. **Unpaid-care substitution:** public cost falls by shifting more unpaid care/time burden to already constrained households. The burden must be counted and evaluated against distributional constraints.
9. **Governance capture:** a policy with an apparently valid vote violates a protected floor or denies recourse to a minority. Aggregate output cannot override the constitutional constraint.
10. **Federation partition and exit:** nodes operate offline, later reconcile obligations, and one community exports/exits. No double capacity, duplicate spendable value, erased claim, forced disclosure, or lost appeal path is permitted.

Each scenario must specify a deterministic schedule or seed, predeclared pass/fail criteria, baselines, data licensing/provenance, evaluator digest, and a run receipt. Null, adverse, and failed results remain in the corpus.

## 8. Implementation sequence

1. **Schema and conformance tests:** implement versioned wire types, strict unit/value parsers, explicit uncertainty states, transition rules, and independent contract tests.
2. **One vertical slice:** choose a housing repair or care service; exercise need -> authority -> capacity reservation -> instrument obligation -> delivery evidence -> verification -> settlement/dispute. Keep external execution simulated until the evidence/authentication gates exist.
3. **Fault injection:** test race, crash, stale values, partition, duplicate attempts, malformed receipts, and policy-version changes at every transition boundary.
4. **Interoperate narrowly:** map one existing mutual-credit/commons protocol to the contract and reject unsupported semantics explicitly; do not require a global proprietary conversion layer.
5. **Independent evaluation:** freeze baselines and metrics before runs, use an independent evaluator, publish all Pareto dimensions and counterexamples, and do not infer real-world efficacy from unit tests or simulations.
6. **Field pilot and replication:** proceed only with consenting communities and agreed safeguards; measure support burden, harm, exit success, and outcomes. Require an independent operator to reproduce the workflow and its evidence.

This proposal is a contract design, not a claim that Mycelix already enforces these rules. The first implementation should target the smallest end-to-end scenario that can prove the rules, not attempt to retrofit every domain simultaneously.
