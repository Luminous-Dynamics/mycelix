# SAP Reserve Standard v0
## Monetary constitution and qualification design

Status: Draft research specification  
Scope: Mycelix Finance / SAP  
Related issue: AC-091 (#4109)  
Companion hardening: AC-092 (#4110), AC-093 (#4111)  
Design priority: neutrality + auditability + liquidity + resilience

---

## 1. Purpose

SAP should be designed to become a **reserve and settlement standard before it is treated as a reserve currency**.

A reserve currency is normally an instrument that monetary authorities and financial institutions choose to hold as a store of external liquidity. A reserve standard is broader: it provides a common unit of account, clearing convention, settlement mechanism, and evidence model through which heterogeneous monetary and economic instruments can interoperate.

The distinction matters because Mycelix should not make a premature claim that Holochain creates stable purchasing power, legal money, sovereign monetary authority, or a globally accepted reserve asset.

The target architecture is:

    heterogeneous economic instruments
               |
               v
        SAP clearing standard
               |
        +------+------+
        |             |
        v             v
   SAP settlement   reserve
       unit        instruments
        |             |
        +------+------+
               |
               v
        liquidity / exit
               |
               v
      external currencies

SAP itself therefore remains a transferable settlement/accounting unit. Reserve instruments are separately typed claims or assets that may be held, collateralised, or redeemed under explicit legal and operational arrangements.

---

## 2. External research basis

### 2.1 Current monetary-system lesson

The BIS's 2025 work on the next-generation monetary and financial system describes central-bank reserves as the trusted final settlement asset in today's two-tier system. It identifies three practical properties for the monetary backbone: **singleness, elasticity, and integrity**. The same work argues that tokenisation can combine money and assets on a unified ledger, while government securities remain important as benchmark safe assets and collateral.

This is an important constraint on SAP's design: a digital ledger is not itself a monetary anchor. The anchor comes from credible settlement, liquidity, and institutional arrangements.

### 2.2 SDR lesson

The IMF explicitly describes the Special Drawing Right as an **interest-bearing international reserve asset**, not a currency and not a claim on the IMF. Its value is based on a basket of currencies, and its practical liquidity comes from arrangements that allow holders to exchange SDRs for freely usable currencies.

The useful design pattern is not to copy the SDR mechanically. It is to separate:

- unit/valuation methodology;
- reserve-asset status;
- liquidity and exchange mechanisms;
- governance of the system.

### 2.3 Stablecoin lesson

The FSB and related prudential frameworks emphasize that reserve-backed payment instruments create liquidity, credit, concentration, redemption, and fire-sale risks. A claim that can be redeemed quickly requires sufficiently liquid assets and explicit governance.

SAP should therefore avoid silently becoming an algorithmic or fractional-reserve stablecoin merely because its ledger is deterministic.

### 2.4 South African settlement lesson

The South African Reserve Bank states that SAMOS settlement is final and irrevocable and is based on a pre-funded principle. That reinforces the importance of finality and liquidity discipline for high-value settlement.

---

## 3. Constitutional ontology

The following categories are **not interchangeable**:

    SAP balance
    SAP settlement obligation
    SAP reserve instrument
    collateral
    productive-capacity evidence
    liquidity facility
    oracle observation
    legal claim

The protocol must never infer one category from another without an explicit typed relationship.

Examples:

- A price observation does not prove ownership.
- A signed reserve report does not prove solvency.
- Collateral registration does not prove market liquidity.
- Productive capacity does not equal immediately redeemable value.
- SAP balance does not equal reserve capital.
- MYCEL reputation does not constitute monetary backing.
- TEND credit does not constitute SAP supply.
- Governance approval does not itself establish external asset existence.

This ontology is constitutional and should be reflected in Rust types and validation rules rather than only in documentation.

---

## 4. SAP unit

### 4.1 Base unit

The base settlement unit is:

    1 SAP = 1,000,000 μSAP

Monetary state must use exact integer arithmetic.

Floating-point values may be used only for non-monetary analytical statistics where their uncertainty is explicit and they cannot directly create, destroy, transfer, or settle SAP.

### 4.2 No forced purchasing-power peg

v0 should **not** promise:

    1 SAP = $1
    1 SAP = fixed kWh
    1 SAP = fixed basket of goods
    1 SAP = fixed quantity of gold

Instead, SAP receives a market value through use, liquidity, and exchange.

A reference basket may be used for an **analytical purchasing-power index**, but this index must not silently become a redemption guarantee or minting oracle.

This avoids the classic failure mode in which a supposedly stable token inherits a hidden promise that its issuer cannot actually honor.

### 4.3 Demurrage

The existing SAP demurrage mechanism is economically meaningful because it distinguishes circulating balances from protected reserves. It should be treated as a monetary-policy parameter, not as proof of reserve quality.

Any future change to demurrage must be governed as a constitutional monetary-policy change and must not modify historical transactions.

---

## 5. Monetary supply constitution

Every supply increase must belong to exactly one issuance class.

Recommended v0 classes:

### Class A — Governance issuance

A constitutional governance process explicitly authorizes issuance.

Required fields:

- proposal identifier;
- authorized issuer;
- recipient;
- amount;
- issuance policy version;
- validity period;
- unique issuance identifier;
- evidence references;
- authorization threshold;
- action timestamp.

The issuance must consume one unique authorization.

### Class B — Qualified external collateral issuance

SAP may be issued against explicitly qualified external collateral.

Required:

- collateral identifier;
- owner/controller evidence;
- custody state;
- valuation snapshot;
- haircut policy;
- eligibility policy;
- liquidation policy;
- issuance ratio;
- bridge/domain identity;
- freshness deadline.

Price alone is never sufficient.

### Class C — Qualified productive-capacity issuance

A future economic design may permit issuance against reproducible productive claims, such as verified energy or other outputs.

This class must be much stricter than a sensor-signature model.

A qualifying record needs:

    physical observation
        + source identity
        + measurement method
        + time window
        + anti-replay identity
        + ownership/beneficiary relationship
        + independent verification
        + valuation method
        + issuance policy

The physical claim and the monetary issuance remain distinct objects.

### Class D — Emergency liquidity issuance

Emergency issuance exists only to preserve settlement continuity under a defined crisis policy.

It requires:

- explicit crisis state;
- maximum duration;
- maximum aggregate amount;
- authorized issuer set;
- collateral/haircut requirements;
- automatic expiry;
- post-event reconciliation;
- public audit record.

Emergency authority must never be an unbounded escape hatch.

---

## 6. The non-negotiable supply invariant

For all issuance paths:

    total_new_supply
        <=
    sum(authorised issuance amounts)

and:

    every authorised issuance
        -> exactly one issuance identity
        -> at most one successful consumption

For conservation transfers:

    sender_delta + receiver_delta = 0

For demurrage:

    holder_delta + commons_credit + explicit_sink = 0

No subsystem may create net SAP merely by mutating a balance.

This implies the long-term closure of the known raw-credit surface is not optional. AC-092 is therefore foundational to any reserve ambition.

---

## 7. Reserve-asset taxonomy

The reserve layer should classify instruments by what they actually provide.

### R0 — Observation

Examples:

- market quote;
- sensor reading;
- exchange-rate publication.

Provides information only.

### R1 — Evidence-backed claim

A signed or otherwise authenticated claim with provenance and methodology.

Provides auditable evidence, but not necessarily enforceable redemption.

### R2 — Collateral

An asset legally or operationally pledged to secure a liability.

Provides a contingent recovery path.

### R3 — Liquid reserve asset

An asset that can be converted to settlement liquidity rapidly at bounded expected loss.

Requires:

- verified custody/control;
- observable liquidity;
- concentration limits;
- haircuts;
- stressed liquidation assumptions.

### R4 — Final settlement reserve

An instrument whose transfer itself is accepted as final settlement within the relevant legal/institutional domain.

This is the strongest reserve classification and should be difficult to obtain.

### R5 — Liquidity facility

A committed source of contingent liquidity, such as a contractual market-maker, credit facility, or central/cooperative liquidity pool.

A facility is not collateral and should never be counted twice.

---

## 8. Reserve accounting

Every reserve position should expose at least:

- gross quantity;
- unit;
- valuation source;
- valuation timestamp;
- valuation methodology;
- haircut;
- net eligible value;
- liquidity horizon;
- jurisdiction;
- custodian/controller;
- encumbrance state;
- concentration bucket;
- evidence quality state;
- expiry/freshness;
- unique asset identity.

A reserve report should distinguish:

    gross value
    eligible value
    immediately liquid value
    stressed liquidation value
    legally enforceable value

These are not the same number.

A reserve dashboard that reports only one number is unsafe for systemically important use.

---

## 9. Reserve coverage metrics

v0 should define at least four ratios.

### Immediate liquidity coverage

    ILC =
      immediately-liquid net assets
      /
      short-horizon settlement obligations

### Stressed reserve coverage

    SRC =
      stressed liquidation value
      /
      eligible reserve liabilities

### Collateral coverage

    CC =
      net eligible collateral value
      /
      collateralised SAP obligations

### Emergency liquidity coverage

    ELC =
      committed emergency liquidity
      /
      stressed net liquidity deficit

The protocol should never call an arrangement “fully reserved” merely because a gross asset value exceeds liabilities.

---

## 10. Haircuts

Haircuts should be deterministic and versioned.

A haircut function should depend on attributes such as:

    asset class
    liquidity horizon
    price volatility
    concentration
    legal enforceability
    custody risk
    correlation
    oracle quality
    encumbrance

An asset can therefore be:

    100 SAP market value
    70 SAP eligible reserve value
    45 SAP stressed liquidity value

without contradiction.

This is healthier than pretending that a volatile or illiquid asset is equivalent to cash.

---

## 11. Redemption and exit

SAP v0 should distinguish three concepts:

### Redemption right

A legally/contractually enforceable right to exchange an instrument for another asset.

### Market exit

The ability to sell the instrument through a market.

### Protocol transfer

The ability to move SAP from one agent to another.

These should never be treated as synonyms.

A reserve instrument may provide contractual redemption while SAP itself remains market-valued.

Where par redemption is offered, the issuer/reserve manager must publish:

- eligible redemption asset;
- redemption queue policy;
- maximum processing time;
- fees;
- capacity constraints;
- exceptional suspension conditions;
- reserve composition;
- legal claim structure.

“Redeemable” without these details is not a sufficient reserve property.

---

## 12. Liquidity architecture

A reserve-standard system needs liquidity in normal and crisis states.

Recommended layers:

    L0 ordinary settlement liquidity
    L1 market-making liquidity
    L2 cooperative liquidity pool
    L3 emergency liquidity facility
    L4 external bilateral/official liquidity

Each layer should have an explicit mandate.

The mistake to avoid is assuming that abundant collateral automatically implies immediate liquidity.

A system can be asset-rich and liquid-poor.

Circuit breakers should therefore trigger on **liquidity state**, not merely price volatility.

---

## 13. Settlement architecture

Settlement should optimize for finality, atomicity and recoverability.

For cross-domain transactions:

    instruction
       -> lock / reserve
       -> validate counterparties and evidence
       -> execute legs
       -> deterministic receipt
       -> finality
       -> reconciliation

Where true atomic settlement is impossible, the protocol should explicitly represent:

- pending;
- partially committed;
- disputed;
- failed;
- compensated.

A “success” record may not be emitted merely because one leg succeeded.

This is particularly important for cross-currency and bridge operations.

---

## 14. Oracle architecture

Reserve-grade valuation should be a layered evidence process:

    observation
       ↓
    authenticated source
       ↓
    source qualification
       ↓
    canonical aggregation
       ↓
    valuation snapshot
       ↓
    reserve eligibility
       ↓
    issuance / collateral decision

The existing community price oracle can remain useful for local price discovery, but it should not directly authorize reserve issuance.

The reserve oracle profile must use:

- typed sources;
- freshness windows;
- methodology identifiers;
- exact units;
- source-domain binding;
- provenance digests;
- explicit uncertainty;
- deterministic source ordering;
- missing-data semantics;
- conflict handling;
- historical snapshots.

AC-093 captures this hardening boundary.

---

## 15. External evidence truth boundary

A cryptographically signed claim proves that a key signed bytes.

It does not automatically prove:

- the physical asset exists;
- the signer owns it;
- the measurement is correct;
- the asset has not been pledged elsewhere;
- the market price is fair;
- the asset is liquid;
- the asset is legally enforceable.

Reserve-grade evidence therefore needs independent qualification layers.

Recommended evidence states:

    Unobserved
    Reported
    Authenticated
    IndependentlyVerified
    Qualified
    Stale
    Disputed
    Revoked

Only an explicitly qualified state may participate in reserve calculations.

---

## 16. Anti-double-counting model

Every reserve asset must have a canonical identity.

The protocol must be able to detect:

    same asset -> two reserve managers
    same claim -> two collateral positions
    same production -> multiple mint events
    same custody position -> multiple liabilities

A reserve object therefore needs:

- canonical asset identifier;
- beneficial-owner/controller;
- encumbrance set;
- custody state;
- active claim set;
- evidence lineage.

If the system cannot establish uniqueness, the reserve value is zero for strict qualification purposes.

This is a much stronger rule than “the owner signed the reserve report.”

---

## 17. Governance separation

Reserve-standard monetary governance should keep the following roles distinct:

    issuer
    reserve custodian
    verifier
    valuation/oracle operator
    settlement operator
    liquidity provider
    emergency authority
    governance electorate
    auditor

A single agent may hold multiple roles operationally in small deployments, but the protocol should retain explicit role separation so concentration becomes measurable rather than invisible.

No MYCEL score should directly create monetary authority.

No TEND balance should directly create SAP.

No governance vote should directly mint without satisfying the issuance constitution.

---

## 18. MYCEL / TEND firewall

The following conversions are prohibited by default:

    MYCEL -> SAP
    SAP -> MYCEL
    TEND -> SAP
    SAP -> TEND

Economic activity may generate evidence that is later considered by another system, but the semantic meaning of one currency must not silently mutate into another.

This protects three distinct concepts:

- SAP = settlement/accounting value;
- TEND = bounded mutual credit and reciprocity;
- MYCEL = contextual non-transferable standing/evidence.

Interoperability is achieved through typed claims and exchange mechanisms, not semantic collapse.

---

## 19. Reserve instrument design

A mature SAP economy should probably contain reserve instruments distinct from ordinary SAP balances.

Conceptually:

    SAP
      = settlement unit

    SRN / reserve note
      = qualified reserve claim

    SAP collateral certificate
      = pledged asset claim

    liquidity facility claim
      = contingent settlement liquidity

The exact names can change, but the separation should remain.

This gives the system a powerful property:

**transaction money can remain liquid while reserve capital can remain conservative.**

That is closer to the architecture of real financial systems than requiring every unit of spending money to be directly matched by a warehouse asset.

---

## 20. Monetary policy

The constitution should define which parameters are:

### Immutable

- μSAP denomination;
- integer monetary representation;
- conservation semantics;
- provenance requirement;
- no MYCEL/TEND semantic conversion;
- no double-counted reserves.

### Governance-changeable under delay

- demurrage rate;
- reserve eligibility;
- haircut schedules;
- emergency limits;
- liquidity facility parameters;
- issuance ceilings.

### Emergency-changeable under strict limits

- temporary liquidity ceilings;
- temporary collateral haircuts;
- temporary settlement throttles.

Every policy change needs:

    policy version
    effective timestamp
    authorisation
    rationale
    scope
    expiry where relevant

Historical transactions must never change meaning because a later policy version exists.

---

## 21. Crisis state machine

The reserve system should define explicit states:

    NORMAL
      |
      v
    STRESSED
      |
      v
    LIQUIDITY_EVENT
      |
      +------> RECOVERY
      |
      v
    EMERGENCY

State transitions must be triggered by measurable conditions, not discretionary narrative.

Examples:

- reserve coverage below threshold;
- liquidity horizon breached;
- major custodian unavailable;
- oracle disagreement exceeds bound;
- redemption demand exceeds facility capacity;
- reserve concentration threshold exceeded.

During EMERGENCY:

- new issuance narrows;
- weak reserve classes may become ineligible;
- settlement limits may tighten;
- liquidity facilities activate;
- every exceptional action is auditable.

Emergency mode must automatically return to normal only through explicit recovery criteria.

---

## 22. Adversarial qualification corpus

The initial corpus should include:

### Monetary integrity

1. unauthorized mint;
2. forged issuer;
3. duplicated mint authorization;
4. amount substitution;
5. recipient substitution;
6. policy-version substitution;
7. emergency authority abuse;
8. mint-cap race.

### Reserve integrity

9. same asset backing two liabilities;
10. stale reserve evidence;
11. future-dated evidence;
12. custodian substitution;
13. beneficial-owner substitution;
14. encumbrance omission;
15. frozen asset represented as liquid;
16. methodology substitution;
17. oracle source substitution;
18. correlated-source concentration.

### Liquidity

19. redemption run;
20. liquidity facility exhaustion;
21. market-maker default;
22. sudden price gap;
23. venue shutdown;
24. jurisdictional seizure;
25. custodian insolvency.

### Settlement

26. replayed settlement;
27. duplicate settlement;
28. cross-leg failure;
29. partial bridge execution;
30. stale authorization replay;
31. conflicting concurrent state;
32. recovery after process crash.

### Semantic attacks

33. MYCEL inflation presented as SAP backing;
34. TEND credit presented as SAP reserve;
35. oracle observation presented as ownership;
36. governance vote presented as asset existence;
37. collateral registration presented as liquidity.

### History attacks

38. rewrite historical reserve evidence;
39. replace valuation methodology retroactively;
40. remove a prior encumbrance;
41. fork reserve lineage;
42. construct an alternate history with higher eligible reserves.

A green result means the exact adversarial case was rejected or handled according to the frozen profile. It does not mean the real-world economic event is impossible.

---

## 23. Stress-test program

Before any external reserve claim, simulations should model at minimum:

### Scenario A — redemption wave

Assume 10%, 25%, 50%, and 75% of redeemable claims request exit inside one liquidity horizon.

Measure:

- settlement latency;
- reserve drawdown;
- realized haircut;
- remaining coverage;
- failed redemptions.

### Scenario B — oracle shock

Inject:

- 5%;
- 20%;
- 50%;
- total oracle disagreement.

Measure whether reserve eligibility changes deterministically and whether issuance reacts too quickly.

### Scenario C — reserve seizure

Remove the largest custodian or jurisdiction.

Measure concentration and recovery.

### Scenario D — market closure

Disable the main market venue for each reserve class.

Measure surviving liquidity.

### Scenario E — bridge failure

Allow one settlement leg to finalize while the other becomes unavailable.

Require compensating state rather than silent success.

### Scenario F — governance capture

Assume a supermajority of a governance body is malicious.

Measure maximum extractable SAP before constitutional limits stop further issuance.

### Scenario G — correlated reserve collapse

Simultaneously stress assets that appeared diversified but share:

- one custodian;
- one jurisdiction;
- one oracle;
- one market;
- one collateral manager.

Diversification is real only when common failure domains are also diversified.

---

## 24. Adoption ladder

### Stage 0 — internal settlement

SAP works inside Mycelix with deterministic issuance, transfer, accounting, and reconciliation.

Qualification:

- exact balance conservation;
- provenance complete;
- replay resistance;
- deterministic reads;
- concurrency tests.

### Stage 1 — cooperative clearing

Independent cooperatives settle obligations using SAP as their common unit.

Required:

- explicit legal counterparties;
- bilateral liquidity;
- dispute handling;
- measurable settlement latency.

### Stage 2 — reserve accounting standard

Treasuries begin recording parts of their reserve portfolios in SAP terms.

SAP does not need to be their primary asset yet.

Required:

- stable valuation methodology;
- reserve taxonomy;
- independent auditing;
- reproducible snapshots.

### Stage 3 — reserve instrument ecosystem

Qualified SAP reserve instruments exist with explicit claims and liquidity facilities.

Required:

- legal claim structures;
- custody;
- redemption;
- stress-tested coverage.

### Stage 4 — cross-border clearing standard

Multiple economic regions use SAP as a common clearing denomination.

Required:

- FX/asset conversion markets;
- interoperable identity and compliance;
- cross-domain settlement;
- sufficient liquidity.

### Stage 5 — official reserve asset candidate

Public monetary authorities or analogous institutions voluntarily hold SAP-denominated reserve instruments.

This stage is not protocol-controlled. It emerges from external adoption, institutional trust, liquidity, legal enforceability and macroeconomic utility.

---

## 25. What success looks like

A successful SAP reserve-standard system should eventually permit an outside participant to answer, from machine-verifiable records:

1. What exactly is one SAP?
2. How much SAP exists?
3. Why was each unit issued?
4. Who was authorized to issue it?
5. What reserve claims exist?
6. Which reserves are liquid now?
7. Which reserves are encumbered?
8. What methodology values them?
9. How stale can the valuation be?
10. What happens under a 50% redemption shock?
11. Who provides emergency liquidity?
12. What happens if the largest custodian disappears?
13. Which governance actions can change monetary policy?
14. What is immutable?
15. Which claims are merely observations rather than verified assets?

The system should make these answers **computable**, not dependent on trust in a dashboard operator.

---

## 26. Core design theorem

The intended constitutional theorem is:

    No SAP becomes reserve-grade
    merely because it is recorded on Holochain.

Instead:

    SAP reserve-grade eligibility
      =
    monetary provenance
    + asset uniqueness
    + qualified evidence
    + conservative valuation
    + liquidity qualification
    + settlement finality
    + governance separation
    + stress survivability

This is deliberately stricter than a normal cryptocurrency.

It is also the reason the reserve-standard vision is credible: every layer that currently depends on social trust becomes a candidate for explicit provenance, bounded authority, reproducible computation, or legally enforceable external structure.

---

## 27. Immediate implementation sequence

Priority order:

1. **AC-092 — close raw credit/mint surface**
2. **AC-093 — reserve-grade oracle provenance**
3. canonical reserve-asset identity and anti-double-counting
4. typed reserve valuation snapshots
5. haircut and eligible-value engine
6. liquidity coverage state machine
7. redemption/exit semantics
8. emergency issuance constitution
9. reserve stress simulator
10. cross-domain settlement qualification
11. external legal/institutional profiles
12. long-running economic validation

The first two are prerequisites because reserve accounting is meaningless while the ledger can still receive unexplained increases or consume weakly authenticated valuations.

---

## 28. Qualification ceiling

A qualified SAP implementation can establish:

- exact monetary arithmetic;
- conservation rules;
- provenance relationships;
- deterministic reserve calculations;
- bounded issuance;
- explicit liquidity state transitions;
- reproducible stress-model outputs.

It cannot establish by cryptography alone:

- legal ownership;
- physical existence;
- economic productivity;
- market efficiency;
- legal enforceability;
- universal convertibility;
- absence of political capture;
- global reserve-currency status.

The system should therefore remain aggressively fail-closed about external truth.

---

## 29. Conclusion

SAP should not attempt to win by being “the next Bitcoin” or by promising an artificial one-to-one peg.

The more ambitious and defensible objective is:

**SAP becomes the neutral monetary coordination layer through which different economic systems can account, clear, settle, collateralise and eventually hold reserves with unusually strong provenance and auditability.**

Reserve status then becomes an empirical consequence of:

    utility
    + liquidity
    + resilience
    + institutional trust
    + legal enforceability
    + network adoption

rather than a property asserted by the protocol itself.


---

## 30. Primary sources

The external monetary and settlement claims in this document should be checked against primary institutional publications:

- Bank for International Settlements, *The next-generation monetary and financial system* (Annual Economic Report 2025): https://www.bis.org/publications/aer-2025/next-generation-monetary-financial-system
- Bank for International Settlements, *Next-generation monetary and financial system takes shape, based on a tokenised unified ledger* (24 June 2025): https://www.bis.org/media-releases/20250624-next-generation-monetary-and-financial-system-takes-shape-based-tokenised-unified-ledger-bis
- International Monetary Fund, *Special Drawing Rights*: https://www.imf.org/en/topics/special-drawing-right
- International Monetary Fund, *What is the SDR?*: https://www.imf.org/en/about/factsheets/sheets/2023/special-drawing-rights-sdr
- Financial Stability Board, *Regulation, Supervision and Oversight of “Global Stablecoin” Arrangements*: https://www.fsb.org/2023/07/high-level-recommendations-for-the-regulation-supervision-and-oversight-of-global-stablecoin-arrangements-final-report/
- South African Reserve Bank, *SAMOS*: https://www.resbank.co.za/en/home/what-we-do/payments-and-settlements/samos

These sources are reference material, not endorsements of SAP. They establish the external concepts against which SAP should be evaluated.
