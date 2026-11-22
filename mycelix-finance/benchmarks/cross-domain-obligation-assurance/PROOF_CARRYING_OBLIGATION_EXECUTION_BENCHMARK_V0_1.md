# Proof-Carrying Obligation Execution (PCOE) Benchmark v0.1

**Status:** proposed research benchmark; not a standard, certification, legal opinion, or production qualification.  
**Scope:** interoperable financial, physical-world, agent-authority, and evidence systems.  
**Version:** 0.1 — 22 November 2026.

## 1. Why this benchmark exists

A payment rail can settle correctly while the underlying delivery was never made. A meter can emit a signed value that is physically false. A verifier can sign a statement outside its competence. A contract can be accepted by an actor whose authority was revoked. A workflow can mark a trade settled after a timeout even though the external outcome is unknown.

These are not isolated component defects. They are **cross-boundary failures**: a fact, authority, or outcome is silently promoted while moving between systems.

PCOE evaluates the whole obligation lifecycle:

1. a contract or obligation is defined and version-pinned;
2. an accountable principal grants narrowly scoped authority;
3. resources or capacity are reserved without double allocation;
4. physical observations are collected with provenance, calibration, boundary, units, and uncertainty;
5. a verifier independent of the producer evaluates the exact evidence set;
6. an authorized decision-maker accepts an exact contract milestone;
7. a payment instruction is issued once to an external rail;
8. an authoritative receipt establishes the rail's stated finality;
9. physical, contractual, and financial records reconcile;
10. an independent party can reproduce the disposition from the evidence package.

The benchmark does not treat a long event chain as a proof by itself. Signatures establish who made an assertion and whether it was altered; they do not establish that the assertion is true, that the signer is competent, that the evidence is independent, or that a payment is final.

## 2. Paradigm shift: commit by proof, not by optimistic status

The core proposal is to model economic work as a **proof-carrying obligation graph**. The graph is append-only; every decision cites the exact parent records and versions it consumed. Corrections are new events that point to the records they supersede. Historical assertions are not silently rewritten.

Do not represent the entire workflow with one boolean such as settled, verified, or complete. Maintain separate state dimensions:

| Dimension | Example states | What it proves — and does not prove |
|---|---|---|
| Authority | valid, expired, revoked, indeterminate | The actor may perform this action within a precise scope; not that the action is correct |
| Physical evidence | claimed, observed, invalid, incomplete | What a specific device or source reported; not necessarily independent truth |
| Verification | verified-for-scope, rejected, contested, indeterminate | What a qualified verifier established within a declared method and scope |
| Contract acceptance | accepted, partially accepted, rejected, disputed | What an authorized counterparty accepted under a pinned contract version |
| Execution | not-started, reserved, submitted, reconciled | Whether the intended side effect was attempted and reconciled |
| Settlement | paid-final, rejected, indeterminate, manual-attested | What the settlement rail or separately governed manual process establishes |

A state in one dimension must never promote another dimension implicitly. A model prediction is not a meter observation. A signed meter observation is not independent verification. Verification is not buyer acceptance. Buyer acceptance is not payment. A timeout is not success. A manual attestation is not rail finality.

### 2.1 Canonical obligation envelope

A conforming adapter should preserve, at minimum:

- immutable obligation ID and exact contract/version digest;
- principal, actor, delegation chain, action class, resource, purpose, amount/quantity limit, expiry, and revocation epoch;
- milestone and the exact evidence digests required by its acceptance rule;
- subject, issuer, verifier, method, declared conflicts, competence/accreditation reference, and verification scope;
- measurement boundary, unit, device identity, sequence, calibration interval, clock quality, uncertainty, and raw-payload digest where applicable;
- event time and record/observation time as separate fields;
- request digest, one-time idempotency identity, external transaction reference, beneficiary, amount, currency, and rail finality policy;
- exceptions, dispute/appeal route, supersession links, and the reason for each state transition.

Canonicalization, cryptographic signatures, credential validation, persistence, and external API authentication belong in trusted adapters. The pure policy kernels must not be described as performing those tasks.

### 2.2 Authority and delegation

Use subject and actor identities explicitly. Delegation must attenuate: a delegate cannot expand the original scope, amount, resources, duration, or action class. Authority must be checked at the authoritative mutation boundary, not only when a user interface or agent plans an action. The check must include revocation freshness and prevent time-of-check/time-of-use races.

High-impact actions (the existing BR3 class) require a recorded human disposition. An agent cannot register itself, change its own authority, or use a derived credential to escape its parent constraints.

Existing interoperable building blocks include OAuth 2.0 Token Exchange (RFC 8693), which distinguishes delegation from impersonation and carries subject/actor semantics, and Rich Authorization Requests (RFC 9396), which provides fine-grained authorization details. They are useful building blocks, not complete governance or revocation solutions.

### 2.3 Evidence independence

Two different signer IDs are not necessarily two independent witnesses. The benchmark therefore records a failure-domain matrix covering, where relevant:

- legal entity and beneficial control;
- operator and staff;
- device, calibration laboratory, and firmware supply chain;
- data origin, ingestion path, and cloud/control plane;
- signing keys, custody, and administrative control;
- commercial incentives and declared conflicts.

Critical verification must state which dimensions are independent and which remain common-cause risks. Count independent failure domains, not signatures. Do not claim independence merely because two keys or two DIDs differ.

### 2.4 External side effects and unknown outcomes

Use a durable intent/receipt pattern. Before an external effect, persist a uniquely identified intent bound to the exact request digest. The adapter then submits the effect and appends the authoritative receipt or an indeterminate outcome. An exact retry retrieves the prior result; a changed request under the same idempotency identity is rejected.

When a network timeout or malformed response leaves the external outcome unknown, the state is **indeterminate**. Reconcile with the rail before attempting another side effect. A manual path, where legally allowed, must be a separate state with named attestors, evidence, conflict declarations, and dispute rights. It must not be coerced into paid-final.

This pattern should be implemented at the authoritative boundary. A coordinator-only check or UI-level disabled button is not an adequate substitute for integrity validation, durable consumption, and concurrency control.

## 3. Benchmark levels

| Level | Test surface | Minimum evidence |
|---|---|---|
| L0 — arithmetic/property | Pure Rust kernels | deterministic properties, overflow boundaries, conservation, idempotency |
| L1 — component contract | Each adapter/kernel boundary | positive fixtures, negative controls, schema validation, exact request/receipt binding |
| L2 — cross-system lifecycle | Symthaea, Mycelix, Symtropy, Sol-Atlas, verifier, rail simulator | one end-to-end obligation with reproducible linked records |
| L3 — adversarial distributed execution | concurrency, partitions, retries, restarts, stale caches, correlated witnesses | fault-injection trace, invariant checks, recovery/reconciliation evidence |
| L4 — field pilot | one real installation and sandboxed/regulated payment integration | metrology records, independent inspection, contract, legal/operational controls, external audit |

A passing L0 or L1 result does not qualify L2–L4. A synthetic fixture cannot establish metrology accuracy, legal enforceability, custody, or bank-rail finality.

## 4. Adversarial corpus

The benchmark now has four separate machine-readable suites comprising **71 scenarios**. Each scenario includes a threat, preconditions, the injected mutation, expected disposition, and the control that must be demonstrated:

- `pcoe-scenarios.v0.1.json`: 39 core obligation scenarios.
- `pcoe-agent-authority-scenarios.v0.1.json`: 10 agent identity/delegation/approval scenarios.
- `pcoe-transparency-metrology-scenarios.v0.1.json`: 10 transparency, legal-metrology and regulatory-adapter scenarios.
- `pcoe-energy-attribute-scenarios.v0.1.json`: 12 renewable energy-attribute issuance/ownership/retirement scenarios.

The Rust reference kernel maps all 71 IDs into explicit coverage classes: 30 implemented in reference-kernel tests, 15 partial/adapter-dependent, and 26 not yet implemented. These labels are not full PCOE conformance. Adapter-level suites are not replaced by in-memory policy tests.

The corpus is intentionally cross-layer. It includes:
- expired, revoked, replayed, or escalated delegated authority;
- simulation promoted to production, invalid calibration, replayed meter sequences, implausible capacity, unit/boundary substitution, and clock drift;
- self-verification, common-cause witnesses, conflicts, evidence-digest mismatch, and tampered records;
- contract-version substitution, absent acceptance authority, partial delivery presented as full delivery, concurrent oversell, and duplicate collateral;
- timeout ambiguity, malformed responses, duplicate side effects, idempotency conflicts, beneficiary/amount mismatch, manual-attestation promotion, stale receipts, and post-finality reversals;
- pending transparency anchors, fork/split-brain histories, privacy over-disclosure, model outage, prediction/measurement confusion, and unreconciled physical/financial quantities;
- unregistered or suspended generation devices, overlapping intervals, EAC over-issuance, duplicate certificate IDs, double transfer/retirement, cross-registry conflicts, storage-provenance laundering and certificate correction/claim-reconciliation failures.

The EAC suite preserves a strict distinction between physical generation, contractual delivery/consumption, certificate ownership/retirement and a consumer's renewable-energy claim. PCOE should interoperate with an accredited EAC registry; it is not a shadow certificate registry.

A scenario is not passed because an error was logged. The harness must assert the resulting state, the absence of unauthorized side effects, and relevant balance/quantity deltas. Every negative control must fail when its corresponding safeguard is removed.

## 5. Metrics and proposed targets

These are proposed programme targets, not measured results or regulatory thresholds.

| Metric | Proposed target | Qualification method |
|---|---:|---|
| Unauthorized high-impact actions accepted | 0 | adversarial authority and revocation suite |
| Simulation or prediction accepted as metered production | 0 | source-class and provenance negative controls |
| False paid-final transitions | 0 | rail timeout, malformed receipt, mismatch, replay, and partial-payment attacks |
| Duplicate economic side effects | 0 | retry, concurrency, crash, and split-brain tests |
| Reserve coverage for redeemable liabilities | at least 100% eligible liquid coverage | independently authenticated positions and liability reconciliation |
| Quantity/value conservation | exact at integer minor-unit / Wh boundary | property tests and end-to-end ledger reconciliation |
| Unknown outcomes silently promoted to success | 0 | inject timeout after every external side-effect boundary |
| Revocation honored at commit boundary | 100% of tested stale/revoked cases | race and stale-cache tests |
| Exact replay behavior | same prior disposition, no second side effect | repeated identical digest |
| Mutated replay under same key | 100% rejected | same key, different canonical request digest |
| Independent reproduction | 100% of published qualification bundles reproducible | second implementation / clean-room replay |
| Healthy-path liveness | measured and reported, not traded for safety | p50/p95/p99, throughput, queue depth, resource use |
| Reconciliation burden | decrease against matched baseline | staff time, exception rate, and time-to-resolution |

For every metric publish denominator, workload, failure injections, confidence interval where statistical, hardware/software versions, and exclusions. A zero in a small fixture means only zero in that fixture. Expand through deterministic property testing, fuzzing, mutation testing, and held-out adversarial cases.

### 5.1 Credit and lending extension

The current credit assurance helper explicitly computes a simplified PD × LGD × EAD arithmetic result; it does not estimate PD/LGD, stage exposures, discount cash shortfalls, or implement a complete IFRS 9 ECL process. Keep that boundary visible.

The next risk-evaluation increment should support scenario-weighted expected cash shortfalls with discounting and a documented horizon/staging policy. Model evaluation must use time-based out-of-sample cohorts and report at least Brier score, log loss, calibration intercept/slope, calibration error with confidence intervals, and performance by adequately powered cohorts. Compare against a transparent logistic baseline and any incumbent model under the same data split. Report fairness, affordability, explainability, missing-data behavior, and drift separately; aggregate accuracy is not enough.

A 2% expected-calibration-error goal and calibration slope in the 0.9–1.1 range may be used as initial research targets only when sample size and binning are preregistered and confidence intervals are published. They are not underwriting rules or proof of regulatory compliance. Missing or unqualified data must not be silently converted into a high-confidence denial. Hardship, dispute, and appeal paths must remain explicit.

### 5.2 Energy and physical-world extension

Measure net delivered energy at an explicitly named metering boundary, with units and interval semantics. Distinguish PV DC output, inverter AC output, site net generation, grid export/import, storage charge/discharge, and site load. State whether a number is cumulative or interval energy. The benchmark must not add unlike boundaries or infer delivery from modelled production.

Meter accuracy must be qualified against the actual device class, installation, calibration and applicable legal metrology/grid rules. A software test can verify arithmetic and declared calibration coverage; it cannot prove the physical meter is honest or accurately installed.

### 5.3 Monetary and custody extension

A currency claim must identify its issuer, redemption right, seniority, legal perimeter, custody model, eligible reserve definition, valuation time, encumbrances, and independent evidence. Keep productive assets and loan receivables separate from liquid redemption reserves unless the explicit liability structure and applicable law permit otherwise. Benchmark redemption under normal and stressed conditions, direct-claim enforceability, custody/intermediary failure, reserve concentration, asset-liability mismatch, and recovery/resolution plans. The FSB global stablecoin recommendations call for robust legal claims and timely redemption; for single-fiat-referenced arrangements they call for par redemption and reserves at least equal to outstanding stablecoins unless equivalent prudential safeguards apply. Redemption and issuance must be idempotent and reconciled against external custody and payment facts.

## 6. Industry comparison: compete on the missing boundary

Do not try to beat every specialist at its primary job. Integrate with them and compete on total end-to-end assurance.

| Industry reference | What it demonstrates / standardizes | PCOE response |
|---|---|---|
| BIS Project Agorá | Controlled real-value wholesale multi-currency settlement exercise; July 2026 involved 28 institutions/central banks, around CHF 800,000 across 17 scenarios (30 transactions), with reported average initiation-to-settlement around 80 seconds | Do not claim to replace payment rails. Test compatible settlement adapters and add physical delivery, mandate, verifier independence, contract acceptance, and evidence-to-payment reconciliation |
| FSB global stablecoin recommendations | Require accountable governance, disclosure, robust legal claims, timely redemption, and reserve/stabilisation or equivalent prudential safeguards; the FSB's 2025 review found implementation gaps and inconsistencies, especially for stablecoin arrangements | Measure reserve quality, encumbrance, legal redemption rights, stress-time redemption, recovery planning, transparent liabilities, and jurisdictional readiness—not just token supply arithmetic |
| W3C Verifiable Credentials | A Recommendation for interoperable issuer/holder/verifier credential data and integrity mechanisms; VC Data Model 2.1 is a Working Draft as of 22 October 2026 | Reuse versioned credential profiles; separately qualify issuer authority, revocation, verifier competence, privacy and factual scope |
| IETF OAuth delegation | RFC 8693 defines token exchange and delegation/impersonation semantics; RFC 9396 carries fine-grained authorization details | Use interoperable tokens where appropriate, but enforce attenuation, one-time effects, revocation freshness, and human approval in the execution boundary |
| IFRS 9 | Forward-looking, probability-weighted, discounted expected credit-loss measurement using reasonable and supportable information | Extend beyond the one-period scalar helper; qualify staging, discounting, scenario weights, calibration and outcomes |
| BCBS 239 risk-data principles | Emphasize accurate, complete and timely risk data and reporting, including end-to-end lineage and traceability | Test provenance from source observation through model feature, decision, ledger entry and regulatory report; reconcile lineage under stress and during ad-hoc reporting |
| South African electricity market | Electricity Regulation Amendment Act 38 of 2024 commenced 1 January 2025 and provides for an open market platform and NTCSA-related functions | Treat market participation, licensing, wheeling, metering, and settlement as jurisdiction-specific legal/operational adapters; obtain NERSA and legal review before production claims |
| Web4 Internet-Draft | An individual, informational Internet-Draft that proposes a verifiable, agent-native Web architecture; it is not an IETF-endorsed standard | Implement testable readiness profiles without claiming IETF conformance; use independent anchors, machine-readable interfaces, authority records, and disclosed curation |

Project Agorá is an important comparator, not a matched benchmark result. Its controlled wholesale-payment exercise is not directly comparable to a community energy project with physical metrology and local contract enforcement. Any speed or cost comparison must use a matched workload, the same start/end definitions, the same failure assumptions, and the same finality semantics.

## 7. Cross-system demonstration

The first field-oriented demonstration should finance, install, measure, accept, and pay for one community renewable-energy installation.

| Stage | Responsible system / actor | Evidence output |
|---|---|---|
| Site and project assessment | Symthaea (advisory only) + Sol-Atlas context | cited site facts, model assumptions, uncertainty, rights/provenance |
| Identity and authority | Mycelix institutional fabric | identities, mandate, attenuated delegation, revocation state |
| Project and contract | Mycelix | pinned contract digest, milestone conditions, payment authority |
| Engineering and commissioning | Symtropy + qualified installer/inspector | design/commissioning records; simulation labelled as simulation |
| Metering | independent metering provider | calibrated meter observations with interval/boundary/raw digest |
| Verification | independent qualified verifier | scope-bound attestation over exact observations and claim digest |
| Acceptance | authorized buyer/community institution | signed acceptance or partial/disputed disposition against exact contract |
| Payment | sandbox rail first, regulated rail only after approval | idempotent payment intent, authoritative receipt, finality policy |
| Reconciliation | independent participant | reproducible contract/energy/payment graph, exceptions and deltas |

Start with a sandbox payment rail and one installation. No SAP issuance against projected energy. No claim of verified production until measurement, calibration, independent inspection, and relevant legal requirements are satisfied. The first successful demo should publish the complete evidence bundle and its limitations.

## 8. Work order and exit gates

1. **Preserve the current P0 blockers.** Resolve SAP issuance/custody and energy trade settlement integrity before any real-money or production-energy claim. Oracle outage and correlated reporters must fail closed.
2. **Qualify the credit crate.** Obtain exact-head format/build/test evidence; add boundary, overflow, replay, hardship, dispute, and payment-receipt tests. Then implement the scenario-weighted discounted ECL extension behind explicit versioned inputs.
3. **Turn PCOE scenarios into executable tests.** Start with pure Rust contracts and deterministic rail/meter simulators. Require assertions on final state and no-side-effect deltas, not log messages.
4. **Build authoritative adapters.** Bind authors, signatures, verifier credentials, revocation, and contract digests in integrity/transaction boundaries. Keep unknown external outcomes indeterminate.
5. **Publish reproducibility receipts.** Pin repository commit, source digests, compiler/toolchain, test corpus version, deterministic seeds, exact commands, run URLs, artifact digests, and all skipped/failed cases.
6. **Run an independent reproduction.** A second implementation must verify the evidence package without trusting the producer's UI or database.
7. **Only then enter a field pilot.** Legal, financial, metrology, safety, grid, privacy and operational approvals remain separate gates.

## 9. Current evidence ledger (22 November 2026)

This section is a snapshot, not a claim that all systems are qualified.

| Work item | Evidence observed | Explicitly not established |
|---|---|---|
| SAP assurance core PR #6506 | PR description reports exact-head dedicated CI: 18 tests passed, including a deterministic one-million-case property loop; the tested source/workflow did not change in later documentation-only commit | live issuance safety, independent custody, STARK verification, bank integration, reserve audit, legal redeemability |
| Energy assurance core PR #6517 | PR description reports exact-head dedicated CI: 25 tests passed; branch-push and PR workflow runs supplied by project | live Holochain zome safety, real meter accuracy, verifier accreditation, bank/rail finality, grid compliance |
| Credit assurance core | Rust source and focused workflow are present on research branch | no passing build/test evidence yet; no PD/LGD model, full IFRS 9 ECL, credit-bureau data or underwriting qualification |
| Web4 PR #3858 | local structural/semantic fixtures and draft mappings are reported | no usable CI run, full JSON Schema qualification, W4AR cryptographic verification, live endpoints, independent anchor or full conformance |

Blocking issues to keep visible:
- Mycelix #6505 — SAP issuance, physical evidence, custody and STARK verification.
- Mycelix #6516 — energy-grid settlement fallback, replay, author binding and offer bounds.
- Mycelix #6514 — oracle consensus and fail-closed integration test.
- Luminous Web4 #3859/#3860 — implementation and validation tracks (the referenced repository currently needs visibility/permission confirmation before further writes).

## 10. References

- BIS, Project Agorá: https://www.bis.org/project/agora
- FSB, High-level Recommendations for Global Stablecoin Arrangements: https://www.fsb.org/2023/07/high-level-recommendations-for-the-regulation-supervision-and-oversight-of-global-stablecoin-arrangements-final-report/
- FSB, 2025 thematic review of crypto-asset/stablecoin implementation gaps: https://www.fsb.org/2025/10/fsb-finds-significant-gaps-and-inconsistencies-in-implementation-of-crypto-and-stablecoin-recommendations/
- Basel Committee, BCBS 239 implementation monitoring: https://www.bis.org/publications/implementation-principles-effective-risk-data-aggregation-and-risk-reporting-bcbs-239-principles
- IETF Datatracker, draft-reilly-web4-00 (individual Internet-Draft; not IETF-endorsed): https://datatracker.ietf.org/doc/draft-reilly-web4/
- W3C, Verifiable Credentials Data Model v2.0 Recommendation: https://www.w3.org/TR/vc-data-model/
- W3C, Verifiable Credentials Data Model v2.1 Working Draft history: https://www.w3.org/standards/history/vc-data-model-2.1/
- IETF RFC 8693, OAuth 2.0 Token Exchange: https://www.rfc-editor.org/rfc/rfc8693
- IETF RFC 9396, OAuth 2.0 Rich Authorization Requests: https://www.rfc-editor.org/rfc/rfc9396
- IFRS Foundation, IFRS 9 project summary — impairment / measuring expected credit losses: https://www.ifrs.org/content/dam/ifrs/project/fi-impairment/ifrs-standard/published-documents/project-summary-july-2014.pdf
- South African Government, Electricity Regulation Amendment Act 38 of 2024: https://www.gov.za/documents/acts/electricity-regulation-amendment-act-38-2024-english-isizulu-20-aug-2024
- I-TRACK Foundation, Product Code for Electricity (I-REC(E)): https://www.trackingstandard.org/product-code/electricity/
- U.S. EPA, How to Avoid Double Counting Renewable Energy Certificates: https://www.epa.gov/green-power-markets/double-counting
- IEC 62052-11:2020, electricity meter general requirements and type tests: https://webstore.iec.ch/en/publication/28212
- Efficiency Valuation Organization, IPMVP Core Concepts 2022: https://evo-world.org/en/products-services-mainmenu-en/protocols/ipmvp

---

**Interpretation rule:** PCOE success means the tested obligation was correctly authorized, evidenced, verified, accepted, executed, and reconciled under the declared assumptions. It does not mean that a cryptographic proof makes a physical fact true, that a software test creates a legal right, or that a local kernel qualifies a production institution.
