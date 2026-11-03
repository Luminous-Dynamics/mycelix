# South African Payments Interoperability Profile v1

**Status:** research profile; synthetic qualification only  
**Created / evidence cut:** 2026-11-03  
**Jurisdiction:** ZA  
**Parent:** [SA-PAY-INTEROP-001 / #5725](https://github.com/Luminous-Dynamics/mycelix/issues/5725)  
**Composes with:** [FIN-ISO20022-001 / #2432](https://github.com/Luminous-Dynamics/mycelix/issues/2432), [payment adapter qualification / #4957](https://github.com/Luminous-Dynamics/mycelix/pull/4957), [FIN-SYNC-003 / #2443](https://github.com/Luminous-Dynamics/mycelix/issues/2443), [bank migration / #5715](https://github.com/Luminous-Dynamics/mycelix/issues/5715), [bank resilience / #5302](https://github.com/Luminous-Dynamics/mycelix/issues/5302).

## 1. Purpose and claim boundary

This profile records a time-bounded, source-linked view of South Africa's payments-system landscape so Finance adapters, reconciliation, migration and resilience work can be qualified against the right institutional boundaries.

It does **not** implement a connection to SAMOS, SADC-RTGS, PayShap, TCIB, PayInc, an NPU, PEMKey, a bank, or any payment service provider. It is not a SARB-approved profile, a legal interpretation, a provider onboarding pack, or evidence of production access.

The machine-readable companion is [south-african-payment-interop-v1.json](./south-african-payment-interop-v1.json). Its cases are synthetic and effects-disabled.

## 2. Current-source snapshot

The South African Reserve Bank (SARB) Payments Ecosystem Modernisation (PEM) programme page describes a landscape with distinct existing systems, transition programmes and future capabilities. These categories must remain distinct in software and operator decisions.

| System / programme | Evidence class as of 2026-11-03 | What the cited source supports | What it does not establish |
|---|---|---|---|
| SAMOS | Current high-value RTGS; modernisation underway | SARB identifies SAMOS as South Africa's RTGS system and says it is being overhauled | A new RTGS feature, a specific participant's connectivity, a particular message version, or a Mycelix integration |
| SADC-RTGS | Current regional high-value RTGS; modernisation underway | SARB identifies the regional SADC-RTGS system and describes its role in high-value payments | A low-value instant-payment capability, universal currency conversion, or finality semantics for every participant |
| PayShap | Existing named low-value payment rail in the SARB PEM landscape | SARB names PayShap as part of the low-value digital payments landscape | A universal API, all-bank availability, a specific participant's current service limits, or a Mycelix finality profile |
| TCIB | Existing named regional low-value/faster-payment service in the SARB PEM landscape | SARB describes TCIB as the SADC equivalent in its low-value payments discussion | An assumption that cross-border amount, FX, participant, liquidity or settlement semantics equal PayShap |
| PayInc / NPU modernisation | Transitioning programme; specified PSMB handover completed | SARB describes the wider NPU direction. The PSMB handover of functions to PayInc took effect on 2 September 2026; its PCH allocation includes EFT credit/debit, Authenticated Collections, PayShap, Registered Mandate and real-time clearing | Completion of the PSMB handover does not mean all NPU objectives, new participant access or every planned service are complete |
| Future fast payment system | Planned / in development | SARB describes a future fast payment system and a pre-funded model intended to enable real-time settlement | That the pre-funding model is already a property of PayShap, TCIB, or a Mycelix payment channel |
| Future RTGS | Planned / in development | SARB says the new RTGS is intended to support multiple currencies, richer payment data and easier connections | That the existing SAMOS or SADC-RTGS interface already exposes those future capabilities |
| PEMKey / Payment Credential Ecosystem | Programme enabler | SARB describes reusable trusted credentials; the Governor's 20 October 2026 speech describes user-controlled wallets, consent and safeguards | That a credential alone proves current authority, account ownership, funds, transaction consent or settlement |

Primary source: SARB, [Payments Ecosystem Modernisation Programme](https://www.southafricanreservebank.co.za/en/home/what-we-do/payments-and-settlements/pem), accessed 2026-11-03.

**Temporal rule:** every capability assertion in an adapter must bind to a source observation date, a profile version, an effective interval where known, and a named participant/product. A programme announcement is not an executable capability. A later programme milestone does not retroactively upgrade a historic transaction or adapter profile.



### 2.1 Governance allocation and delivery timeline

The PSMB handover is no longer simply a future transition. SARB states that rule/standard responsibilities and licensing, authorisation and registration of payment institutions moved to SARB effective **11 August 2026**, while functions transitioning to PayInc took effect **2 September 2026**. SARB's published allocation places EFT credit/debit, Authenticated Collections, PayShap, Registered Mandate and real-time clearing with PayInc; card, immediate/cash settlement and named securities/market PCH functions are allocated to SARB. This function allocation is not an adapter API contract and does not grant any institution participant access. [SARB PSMB transition](https://www.resbank.co.za/en/home/what-we-do/payments-and-settlements/psmb).

The Governor's Payments Conference address is dated **20 October 2026** (not 26 October). It set out these intentions:

| Initiative | Target stated in the address | Qualification rule |
|---|---|---|
| Exemption Notice / Authorisation Framework | By end November 2026 | Do not assume publication, legal effect or a named institution's authorisation before the actual official instrument is inspected |
| QR+ | Pilot in Q1 2027 | A QR interoperability standard is not itself a payment rail, an acceptance guarantee or a settlement proof |
| PayShap adoption work | Begin addressing constraints by end March 2027 | Track parity, pricing, promotion and merchant acceptance separately |
| Qualifying regional retail payments to TCIB | Late April 2027; speech says both "under R100,000" and "up to R100,000" | Do not turn this wording into a route-selection threshold until the official rulebook clarifies inclusive/exclusive boundary, currencies, corridors and participants |
| TCIB hub-and-spoke pilot | Eight countries by October 2027 | Do not infer live corridors or participant onboarding before official operational evidence |

All milestones are announced targets until later authoritative evidence is admitted. If a target date passes without a new source, the profile becomes stale and requires review; date passage is not proof of completion. Source: SARB Governor Lesetja Kganyago, [Rewiring Payments: Open. Connected. Inclusive.](https://www.resbank.co.za/en/home/publications/publication-detail-pages/speeches/speeches-by-governors/2026/kganyago-payments-conference), 20 October 2026.

### 2.2 PayShap adoption and the open-network constraint

The Governor stated that PayShap had passed one billion transactions and R1 trillion in cumulative value since its 2023 inception by 20 October 2026. He also compared the prior year's PayShap transaction count with roughly 6% of card transaction count and less than 40% of EFT credit volumes. Those are dated system-level comparisons, not current forecasts or participant-level service guarantees. He identified four adoption constraints: unequal functionality/limits/user experience across banks, pricing, low public awareness and a narrow merchant acceptance perimeter. These are national-level observations, not participant-specific service-level guarantees.

The same address warns that scaled, independent closed-loop payment systems can restrict customer choice, entrench acceptance control and fragment liquidity. Mycelix therefore must not treat a closed-loop internal currency or wallet as a long-term substitute for open connectivity. The design objective should be **interoperable participation with explicit jurisdictional safeguards**, not a walled garden. Any expansion in scale and functionality must be assessed against the applicable connection and regulatory obligations.

This suggests additional benchmark dimensions beyond raw payment latency: participant parity, effective all-in cost for low-value payments, merchant acceptance coverage, user comprehension, time-to-onboard, successful dispute/return handling and interoperability across providers. Source: the Governor's address above.

### Standards and resilience context

- The CPMI's updated report on harmonised ISO 20022 data requirements was published on 26 February 2026. It updates the cross-border harmonisation requirements and technical data model, with flexibility for market adoption through the end of 2027. This is a reason to pin exact message definitions and market-practice profiles, not to assume that all ISO 20022 implementations are interchangeable. [BIS / CPMI report](https://www.bis.org/publications/harmonised-iso-20022-data-requirements-enhancing-cross-border-payments-updated-report).
- Basel operational-resilience guidance calls for mapping critical-operation dependencies—including people, processes, information, technology, facilities and third parties—and for continuity, third-party and incident-management planning. A payment adapter therefore needs an operational dependency profile, not just a happy-path API wrapper. [Basel consolidated operational-resilience guidance](https://www.bis.org/committees/bcbs/basel-consolidated-guidelines/module/orr/20).
- NIST IR 8547, finalised on 27 October 2026, describes the transition from quantum-vulnerable public-key cryptography to post-quantum standards. Mycelix should preserve algorithm/profile identity and historical verification evidence and plan cryptographic agility; this is not a claim that current Mycelix signatures are post-quantum secure. [NIST IR 8547](https://csrc.nist.gov/pubs/ir/8547/final).

## 3. Paradigm shift: a jurisdictional interoperability graph

Do not model "South African payments" as one API or one universal rail enum. Model a graph of independently governed systems, participants, products, message profiles, instruments, authorities, evidence rules and dependencies.

An executable adapter admission must bind all of the following:

1. Jurisdiction and legal/institutional scope.
2. Named system and network.
3. Exact participant, provider and product/service.
4. API, message-definition and market-practice profile versions.
5. Exact instrument, currency, scale, FX and fee semantics.
6. Supported operation and lifecycle.
7. Idempotency scope, retention horizon and conflict behavior.
8. Prepare, commit, cancel, query and reversal capabilities.
9. Exact settlement-evidence/finality profile.
10. Current institutional authority, credential and policy profile.
11. Operational dependency and recovery profile.
12. Evidence expiry, requalification triggers and source provenance.

Missing or unknown dimensions are not filled using a neighbouring rail's behavior. They yield Unsupported, Quarantined, or Indeterminate according to the exact failure mode.

### The core separation

The following are different events and must remain different in code, receipts and operator displays:

- message parsed;
- instruction formed;
- sender authenticated;
- sender authorized;
- instruction submitted;
- provider accepted;
- booking observed;
- clearing completed;
- settlement observed;
- settlement qualified under a rail-specific profile;
- finality established under that profile;
- legal discharge determined by the responsible institution and applicable law.

A positive status code or a transport acknowledgement cannot promote an instruction to settlement. A reconciliation pass cannot create cutover authority. A credential cannot create payment authority. A Mycelix receipt cannot make Mycelix the authoritative bank ledger.

## 4. Normalized evidence envelope

An inbound or outbound adapter should preserve, at minimum, the fields actually available and explicitly mark unsupported or lossy projections:

- exact raw message bytes length and digest/commitment;
- source system, participant, provider and product profile;
- exact message definition and market-practice profile version;
- transport/header identity separated from business-message identity;
- message ID, business correlation ID and provider operation reference;
- source and destination parties/agents with qualified identity references;
- legal-entity and account-scope references, privacy-protected where required;
- exact amount, currency/instrument identity, scale, fees and FX quote/leg references;
- declared event time, source-recorded time, observation time and effective/settlement time as distinct fields;
- source lifecycle/status code and the versioned mapping rule;
- original event, predecessor, return, reversal, correction and dispute lineage;
- parser version, mapping version, adapter build and policy/profile fingerprints;
- loss report: Preserved, OpaquePreserved, Approximated, Omitted, Unsupported or Unknown;
- required evidence references and an explicit claim ceiling.

Use integer minor units or a declared exact-decimal representation. Never use binary floating point for monetary quantities. A currency code is not, by itself, proof of an external instrument identity. A cross-border route does not imply a zero-FX or same-currency transfer.

The adapter is an airlock around institutional semantics. It must not silently flatten all external events into a Mycelix-native payment state.

## 5. Identity, credentials and payment authority

PEMKey's Payment Credential Ecosystem is an important programme-level interoperability direction. It should be composed as credential evidence, not treated as a universal authorization token.

Keep at least four decisions separate:

1. **Cryptographic authenticity:** does the signature verify over the exact domain-separated canonical transcript?
2. **Credential status:** is the issuer trusted under the selected trust profile, and is the credential current, correctly scoped, unexpired and unrevoked?
3. **Subject and audience binding:** does the credential refer to the intended person/organisation, relying party, account context and operation?
4. **Transaction authority:** does the institution's current authorization policy permit this exact payer, payee, amount, instrument, purpose and operation now?

A credential can pass the first three checks and still not authorize a payment. Conversely, a signature-shaped string or a valid credential reference is not cryptographic verification.

Compose with receipt-authentication work (#5716), exact authority work, the sender-scoped idempotency work (#4491), and the Finance integrity boundary. Do not invent cryptographic algorithms. Bind algorithm identifiers and key-profile versions; preserve historical verification and key-rotation evidence as post-quantum migration proceeds.



### 5.1 User-controlled financial data and purpose limitation

The 20 October 2026 Governor's address explicitly links reusable payment credentials and transaction histories to lower onboarding friction and evidence of economic activity, including for informal businesses. It also sets limits: payment history must not automatically become a credit score; data should be accurate, relevant and open to correction; and access to essential payment services must not depend on agreement to unrelated data uses.

Mycelix should implement this as a **data-purpose capability boundary**:
- user-controlled sharing and revocation references;
- exact purpose, relying party, recipient and retention scope;
- field-level minimisation and a data lineage trail;
- correction as an append-only successor, not destructive rewriting;
- no automatic conversion of payment records into credit decisions;
- essential payment access independent of unrelated secondary-use consent; and
- separate institutional credit authority and adverse-decision process.

Transaction history may be a user-authorised evidence source. It is not itself collateral, repayment capacity, creditworthiness or consent to underwriting.

## 6. Payment-system capability profile

Use FIN-SYNC-003 / #2443 as the owner of rail capability semantics rather than defining a second copy here. For each named adapter, qualify:

| Capability dimension | Required evidence |
|---|---|
| Prepare / hold / reserve | Exact provider or rail semantics, owner, expiry, release and failure behavior |
| Commit | Exact operation identity and the evidence that classifies the external effect |
| Cancel / abort | Whether cancellation is guaranteed, best-effort, pre-submission only, or unsupported |
| Query / recovery | Read-after-timeout behavior, stale response behavior, correlation scope and query limits |
| Idempotency | Key scope, retention horizon, payload-conflict behavior and retry semantics |
| Finality | Exact FIN-ECO finality profile and admissible evidence; no universal status mapping |
| Return / reversal | Successor-event semantics, authority, timing and linkage to the original effect |
| Resilience | Critical dependencies, outage behavior, recovery objective, fallback limits and exit strategy |
| Privacy | Data minimisation, retention, disclosure, redaction and secret separation |

If a rail only supports independent submit operations, do not describe a multi-rail transfer as atomic PvP or DvP. Use a qualified weaker saga or reject the requested coordination profile before dispatch. A compensating transfer is not rollback.

## 7. Synthetic benchmark suite

The companion JSON freezes 27 synthetic scenarios and six governance/programme milestones. It deliberately includes negative and positive controls, but it does not call real endpoints or allow real external effects.

The suite tests whether the system:
- keeps current, transitioning and future programme capabilities separate;
- refuses unsupported participant, product, message-version and finality assumptions;
- prevents status-to-settlement promotion;
- preserves exact currency/instrument and FX semantics;
- prevents duplicate value movement on retry and refuses blind retry after unknown outcomes;
- appends returns and reversals without erasing prior events;
- rejects wrong-issuer, wrong-audience, wrong-subject, expired and revoked credentials;
- prevents credential or QR/alias data from substituting for transaction authorization;
- keeps announced milestones from becoming live capabilities when their target dates pass;
- separates completed PSMB role allocation from the wider NPU programme; and
- prevents payment-history reuse from bypassing purpose limitation, consent and correction rights;
- degrades safely when dependencies are unavailable; and
- minimises personal and confidential payment data.

### Benchmark targets

Safety invariants are hard gates and must not be averaged against speed or cost:

| Measure | Qualification target |
|---|---|
| Source-backed profile claims | 100% carry a source, observation date and assurance/lifecycle class |
| Planned-to-current capability promotion | 0 false-safe promotions |
| Status-to-finality promotion without required rail evidence | 0 |
| Instrument, currency, amount-scale or FX substitution | 0 |
| Duplicate external effects under exact retry | 0 |
| Blind retry after ambiguous external outcome | 0 |
| Credential-to-transaction-authority promotion | 0 |
| Unknown or untested scenario silently reported as PASS | 0 |
| Required record disposition in a migration/reconciliation corpus | 100% |
| Deterministic replay of identical frozen inputs | 100% byte-identical canonical output |
| Canonical Finance / workspace projection drift | 0 byte mismatches |

Do not publish a latency or cost victory before measuring one. On identical frozen inputs and hardware, measure p50/p95/p99 normalization latency, throughput, peak memory, storage per event, source coverage, semantic-loss rate, false-safe rate, operator investigation time, recovery time under dependency failure, implementation effort and total maintenance cost.

### Industry comparison protocol

Run three implementations against the same corpus:

1. **Incumbent/manual baseline:** source-system exports, existing bank/processor reconciliation, documented operator procedures and the institution's current exception workflow.
2. **Conventional adapter baseline:** one provider/rail integration plus its production runbook, retry policy, callbacks and reconciliation process.
3. **Mycelix evidence-qualified baseline:** versioned adapter profile, exact source evidence, independent semantic/finality evaluation, idempotent effect identity, explicit uncertainty and replayable reconciliation.

Report correctness, resilience, operator effort, performance and cost as separate dimensions. An incumbent may outperform on throughput or integration maturity; a conventional orchestration provider may outperform on time-to-connect. Mycelix only demonstrates an advantage if it reduces semantic loss, false-safe outcomes, unresolved reconciliation work or recovery cost without weakening the safety gates. This profile makes no comparative result claim.

## 8. Staged implementation plan

1. **Profile admission:** maintain the time-bounded source register and synthetic corpus; reject unknown or future capabilities.
2. **Independent validation:** validate the JSON Schema with a standards-compliant validator; add a Rust reference evaluator that does not reuse production adapter decisions.
3. **Parser/projection:** implement one exact ISO 20022 message-definition and market-practice subset at a time, with raw-byte commitments and explicit loss reports. Keep XML/parser dependencies outside the smallest authority kernels.
4. **Effects-disabled simulation:** add synthetic participant/provider adapters, retries, stale callbacks, partial effects, crash injection and dependency outages.
5. **Open acceptance and data-use qualification:** test QR+ parsing separately from provider acceptance and measure participant parity, pricing, merchant coverage, purpose-limited transaction-history sharing and correction handling.
6. **Read-only shadow:** only with a willing institution and approved data path, compare source and Mycelix observations without creating or changing payments.
7. **Controlled canary:** only after named-provider qualification, legal/compliance approvals, independent review, operational controls and institution-owned cutover authority.
8. **Expansion:** add other domestic and regional products only as their exact profiles and test evidence become available.

## 9. Nonclaims and stop conditions

This profile does not establish provider access, participant eligibility, SARB or PayInc endorsement, production integration, ISO certification, regulatory compliance, KYC/AML completion, account ownership, funds availability, transaction authority, settlement finality, legal discharge, custody, or production readiness.

Stop before dispatch if the named participant/product, instrument, authority, capability, idempotency, evidence or finality profile is missing, stale, contradictory or out of scope. After an ambiguous external outcome, preserve Indeterminate and query/reconcile the original effect; do not create a new payment to make the workflow appear complete.

## 10. Sources

1. SARB, [Payments Ecosystem Modernisation Programme](https://www.southafricanreservebank.co.za/en/home/what-we-do/payments-and-settlements/pem), accessed 2026-11-03.
2. CPMI/BIS, [Harmonised ISO 20022 data requirements for enhancing cross-border payments — updated report](https://www.bis.org/publications/harmonised-iso-20022-data-requirements-enhancing-cross-border-payments-updated-report), published 2026-02-26.
3. Basel Committee/BIS, [Operational resilience — consolidated guidelines](https://www.bis.org/committees/bcbs/basel-consolidated-guidelines/module/orr/20).
4. NIST, [IR 8547: Transition to Post-Quantum Cryptography Standards](https://csrc.nist.gov/pubs/ir/8547/final), final 2026-10-27.
