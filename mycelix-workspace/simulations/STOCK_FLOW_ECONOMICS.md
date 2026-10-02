# Stock-Flow Economic Dynamics — Research Integration

Status: research implementation on branch `research/keen-sfc-economic-dynamics`.

## Why this exists

Mycelix Finance already contains demurrage, mutual credit, collateral, lending, treasury,
oracle, and metabolic-policy mechanisms. The missing abstraction is an explicit accounting
layer that can connect those mechanisms over time without silently creating or destroying
financial claims.

This work is informed by:

- Steve Keen / Hyman Minsky: debt dynamics, endogenous credit, financial instability and feedback.
- Wynne Godley / Marc Lavoie: stock-flow consistency and explicit monetary accounting.
- Ecological macroeconomics: coupling monetary stocks and flows to physical constraints.
- Elinor Ostrom: institutional rules for commons governance.

The implementation does **not** encode a preferred economic policy. It supplies accounting
primitives and measurable diagnostics so competing models can be simulated against the same
state representation.

## Current implementation

`mycelix-workspace/sdk/src/economics/stock_flow.rs` provides:

- `ActorBalanceSheet`
- `MonetaryStock`
- `RealStock`
- explicit `inventory_carrying_value` monetary stock
- explicit `MonetaryFlow`
- explicit `CreditCreation`
- explicit `DebtRepayment`
- explicit cash vs deposit instruments
- bank loan/deposit double-entry credit creation
- deterministic `EconomicTransition` timestep execution
- `EconomicStepReceipt` state/transition hashes
- `EconomicState`
- aggregate asset/liability accounting
- gross leverage observable
- net credit impulse observable
- accounting-invariant tests

Credit creation now follows the minimal private-money balance-sheet structure: the lender records a loan claim and deposit liability while the borrower records the matching deposit asset and debt liability. Debt repayment reverses the loan/deposit entries. Physical cash remains a distinct instrument. This closes an important modeling gap: in a bank-credit model, a deposit is itself a bank liability rather than an unexplained pool of money. Godley/Lavoie-style SFC accounting explicitly represents deposits and loans this way.

## Why this is the correct first step

A dynamic model should not begin by choosing a policy rule such as "raise/lower demurrage."
It should first make the conservation/accounting constraints executable.

```
Accounting substrate
        |
        +--> Keen/Minsky debt dynamics
        |
        +--> Godley/Lavoie SFC models
        |
        +--> ecological / energy constraints
        |
        +--> commons / Ostrom institutional rules
        |
        +--> Mycelix governance and epistemic evidence
```

This keeps the metabolic oracle from becoming an unexplained policy oracle. Future
adjustments can instead be evaluated as interventions over an explicit state-transition model.

## Current second increment

`sector_flow.rs` now adds a sector transaction matrix with explicit sector, category, direction, and amount. It can verify that a period's sectoral monetary flows clear before behavioral equations are applied.

The individual balance sheet model was tightened again: **cash**, **deposits**, **loan claims**, **debt liabilities**, and **issued deposit liabilities** are distinct. This makes credit creation and repayment explicit rather than treating bank-created money as an unbacked cash injection. The new deterministic timestep layer hashes the complete pre-state, exact ordered transition list, and post-state, and rejects a step if modeled claims and liabilities do not reconcile.

## Next research increments

### 1. Sector balance-sheet matrix

The actor-to-sector reconciliation layer is now present. `SectorBalanceSheet::from_state` deterministically consolidates actors into sectors, while `financial_rows_clear()` checks modeled claim/liability rows. Cash is deliberately treated as issuer-backed residual until a central-bank/public-money instrument is explicitly modeled. The existing sector transaction matrix remains a flow layer; the balance sheet is now the stock layer that it must reconcile against.

Add explicit sectors:

- households
- firms
- banks
- commons pools
- public/governance institutions
- external sector

Every flow should identify its source sector, destination sector, and accounting category.

### 2. Debt-dynamics observables

Add time-series measurements for:

- debt stock
- new credit
- repayment
- interest/service burden
- leverage
- asset-price exposure
- liquidity buffer
- credit impulse

These should be observations, not scores.

### 3. Minsky-style regime observables

Represent financing structures explicitly:

- hedge: cash flow covers principal + interest
- speculative: cash flow covers interest but not principal
- Ponzi: cash flow does not cover interest

The simulator should report regime transitions and cascade behavior rather than declaring
one regime preferable.

### 4. Physical coupling

Connect financial investment to real stocks:

- energy
- food
- housing
- water
- productive capital
- ecological capacity

Recent ecological macroeconomic work demonstrates that stock-flow consistency can be
combined with physical and ecological constraints rather than treating the monetary system
in isolation.

### 5. Evidence-bound simulation

Every scenario should carry:

- model version
- parameter set
- seed
- initial state hash
- transition-rule hash
- observation provenance
- uncertainty assumptions
- output hash

The timestep layer now implements the core hash binding for one period. The next step is to chain receipts across periods and bind the transition-rule/model version and parameter manifest into the evidence record. That makes economic simulations compatible with Mycelix's existing evidence/attestation work.

### 6. Counterfactual laboratory

Only after the accounting layer is stable should we compare alternative rule sets:

- different credit constraints
- different reserve requirements
- different demurrage schedules
- different mutual-credit limits
- different commons allocation rules
- different energy/resource constraints

The simulator should expose trajectories and tradeoffs rather than emit a single "best" policy.

## Research basis

Godley/Lavoie SFC work explicitly links financial and real sides of an economy through coherent stocks and flows; the SFC literature uses balance-sheet and transactions-flow matrices and emphasizes that financial assets must have counterpart liabilities. Keen's work connects Minskyan debt dynamics with SFC accounting and treats changes in debt as a distinct driver in monetary dynamics. Recent ecological SFC work can then be layered on top of this monetary substrate rather than replacing it.

Relevant sources include:

- Godley/Lavoie, *Monetary Economics* / selected SFC writings.
- Nikiforos & Zezza, survey of stock-flow-consistent macroeconomic models.
- Dafermos, Nikolaidi & Galanis, stock-flow-fund ecological macroeconomics.
- Serra & Gallo, 2026 ecological SFC business-cycle model.
- Ostrom Workshop materials on robust commons institutions.

## Non-goals

This module is not:

- a claim that Keen's complete economic theory is correct;
- a policy recommendation;
- a replacement for empirical calibration;
- a macroeconomic forecasting engine;
- an autonomous monetary-policy controller.

The goal is a reusable, auditable substrate on which those hypotheses can be tested.

## Acceptance criteria for the next phase

Before adding autonomous policy adaptation:

1. All financial transitions preserve explicit accounting identities.
2. Credit creation and debt retirement are separately observable.
3. Sectoral balance sheets reconcile every timestep.
4. Real-resource flows cannot appear without an explicit source.
5. Scenario outputs are deterministic under fixed seed/model version.
6. Every policy intervention is represented as an explicit input.
7. Simulation evidence is reproducible from a pinned configuration.


## Research update: why the matrix matters

The research cross-check reinforces the architecture. SFC models conventionally use two linked structures: a balance-sheet matrix for stocks and a transactions-flow matrix for flows. Financial rows/columns clear because a financial asset held by one sector is a liability or counterpart position elsewhere. Recent SFC energy-transition work continues to use this structure, including explicit deposits and bank loans.

Ecological SFC research goes one step further by explicitly formalising monetary and physical stocks and flows together, including resource constraints and thermodynamic accounting. That supports treating the Mycelix financial substrate as one layer rather than the whole economic ontology.


## Stock-flow delta reconciliation (implemented)

The accounting substrate now includes an explicit reconciliation boundary between the deterministic transition log and the consolidated sector balance sheet.

For the currently modeled financial transitions, the invariant is:

`BalanceSheet(t+1) - BalanceSheet(t) == postings(transitions)`

The reconciliation layer:

- replays the exact ordered transition list against a private working state;
- derives settlement-aware postings for deposit transfers, endogenous credit creation, and debt repayment;
- consolidates those postings by sector and balance-sheet instrument;
- compares them with the observed pre/post sector balance-sheet delta;
- emits structured sector/instrument mismatches instead of silently accepting unexplained stock changes;
- hashes the pre-state, post-state, transition sequence, and canonicalized posting set for evidence binding.

This is intentionally narrower than a full SFC model. Wages, consumption, interest, investment, taxes, transfers, production, inventory changes, capital formation, revaluation, and physical-resource flows are not admitted until each has an explicit posting rule. This prevents the economic simulation from gaining apparent realism by introducing unaccounted stock changes.

The design follows the SFC convention that the balance-sheet matrix and transaction-flow matrix form a linked accounting skeleton, with financial assets matched by counterpart liabilities and sector flows satisfying budget constraints. Recent SFC work continues to use explicit bank deposits/loans and linked balance-sheet/transaction-flow structures. Keen's monetary Minsky work likewise treats credit/debt dynamics as explicit monetary state variables rather than an exogenous residual. The next implementation step is therefore to extend the posting vocabulary rather than bypass it with aggregate formulas.

### Sector-flow projection (implemented)

The sector transaction matrix is now derived directly from the ordered transition log. Each supported transition is projected into a sector-to-sector flow with an explicit category:

- IncomeTransfer -> Wage / Interest / Tax / Transfer / Consumption / Investment
- CapitalInvestment -> Investment
- CreditCreation -> LoanCreation
- DebtRepayment -> DebtRepayment
- MonetaryTransfer -> Other

SectorTransactionMatrix::validate_against now checks that the actor-to-sector assignment covers the state, that the matrix clears, and that its exact ordered flow list matches the transition-derived projection. A hand-edited or stale sector matrix therefore cannot silently diverge from the authoritative transition evidence.

This closes the accounting chain one step further:

transition log -> sector transaction matrix -> sector balance-sheet delta -> evidence hashes

The remaining conceptual gap is not another aggregate formula; it is the real production layer. Inventories, output, intermediate inputs, depreciation, and physical-resource flows should be introduced as typed stock transformations with their own conservation rules rather than being folded into CapitalInvestment.

### Production and inventory bridge (implemented)

Production is an explicit physical-stock transition rather than an implicit side effect of investment. `ProductionEvent` consumes a producer's `resources` and creates `inventories`; it carries no hidden monetary transfer.

The sector layer now keeps physical quantities out of the monetary balance sheet. `SectorBalanceSheet` contains monetary claims, liabilities, productive capital values, and inventory carrying value; `SectorPhysicalStock` separately consolidates inventory and resource quantities. This removes the previous dimensional ambiguity where a unit count could enter a currency-denominated equity identity.

The reconciliation engine mirrors that separation: monetary postings reconcile against the sector balance sheet, while physical postings reconcile against the sector physical-stock projection. Both posting sets are hashed, so a reproduced period must match both accounting domains.

This is closer to ecological SFC practice, which explicitly combines monetary and physical stocks/flows rather than treating physical quantities as monetary values.

### Next accounting frontier

1. Add explicit production-cost accumulation so wages, intermediate inputs, and other production costs can feed inventory carrying value without hidden financing.
2. Add explicit depreciation and capital-consumption postings for productive capital.
3. Add typed physical units and material-balance/conservation rules for the ecological SFC layer.
4. Add institutional/financial-regime observables (leverage, debt service, liquidity, refinancing need) on top of reconciled stocks.
5. Only then add Minsky/Keen behavioral equations, so financial-instability dynamics operate on auditable accounting state rather than hidden balances.


## Income/equity double entry (implemented)

The next stock-flow boundary is now explicit: an income transfer is not merely a deposit movement. It also changes the accounting equity residual of both counterparties.

For a deposit-settled income transfer from payer to recipient:

- payer deposits: -amount
- payer equity: -amount
- recipient deposits: +amount
- recipient equity: +amount

The sector balance-sheet representation exposes equity as a signed liability-side residual, so the stock delta is fully explainable without allowing net worth to appear from an unposted behavioral flow.

This is the foundation needed for wages, interest, taxes, transfers, and consumption to become genuine SFC transactions rather than ad hoc balance updates. The next refinement should distinguish the economic category of each income transfer while preserving one accounting posting mechanism.

## Semantic flow categories and period ledger (implemented)

The income/equity boundary has been tightened so **equity is a derived balance-sheet residual**, not a second mutable asset/liability store. For an actor:

`net worth = financial assets - financial liabilities + monetary-valued real assets`

The sector matrix exposes equity as the signed balancing row `-net worth`. This follows the standard SFC convention in which net worth is the balancing item of the balance-sheet matrix, while real assets are not someone else's financial liability. The model therefore does not double-count equity by adding a mutable equity stock on top of net financial position.

`IncomeTransfer` now carries an explicit semantic category:

- `Wage`
- `Interest`
- `Tax`
- `Transfer`
- `Consumption`

The accounting engine remains singular: every category uses the same deposit/equity double-entry posting mechanism. The category is part of the serialized transition, so changing a wage into an interest payment changes the transition evidence hash even when payer, recipient, and amount are identical.

`EconomicPeriodLedger` is a deterministic projection of the ordered transition log. It records:

- category totals;
- ordinary monetary-transfer volume;
- newly-created credit;
- debt repayment;
- transition count;
- the exact transition-list hash;
- a derived ledger hash.

This is deliberately **derived state rather than mutable state**. The transition log remains authoritative, preventing semantic summaries from becoming a second accounting system. Physical quantities are kept in a separate physical-stock projection and are never added to currency-denominated net worth.

The sector balance sheet now validates the monetary per-sector identity:

`signed financial rows + signed equity residual + monetary-valued real assets = 0`

Physical inventory and resource quantities are validated in a separate physical-stock dimension.

while keeping cash outside the clearing requirement until an explicit issuer/public-money sector is modeled. This preserves the distinction between internal financial claims and real wealth. The SFC literature explicitly links the balance-sheet matrix to the transactions-flow matrix and uses net worth as the balancing item.

### Why this matters for the next layer

This gives the substrate a clean path from accounting to behavior:

`transition log -> period ledger -> sector flow matrix -> balance-sheet delta -> observables`

Only after those identities are executable should production, investment, inventories, interest accrual, capital gains, and Minsky-style financing regimes be added. Ecological SFC work similarly integrates monetary and physical stocks/flows only after the accounting structure is explicit.

## Capital formation bridge (implemented)

The accounting substrate now includes `CapitalInvestment`.

A capital-formation transition explicitly performs:

- buyer deposits: `-I`
- buyer productive capital: `+I`
- producer deposits: `+I`
- producer equity residual: `-I` in the signed sector matrix

This is intentionally a **capital-formation primitive**, not yet a complete production model. It represents the creation of productive capital financed by an explicit deposit payment. The resulting sector identities remain executable and reconciled against the ordered transition log.

This follows the SFC structure in which investment is represented in the transactions-flow matrix while the corresponding capital stock appears on the balance sheet. The literature also treats credit, money, equities, and real capital as linked stocks and flows across periods.

The next refinement should therefore be a distinct production/inventory layer rather than silently expanding `CapitalInvestment` to cover everything. That layer can introduce output, inventories, intermediate inputs, resource depletion, wages, and operating surplus while retaining the same deterministic posting/reconciliation machinery.


## Physical inventory circuit and carrying value (implemented)

The physical side has four explicit quantity transitions:

- `ProductionEvent`: resources -> finished-goods inventory;
- `InventoryTransfer`: inventory moves between actors;
- `InventoryConsumption`: inventory is explicitly drawn down by final use, spoilage, destruction, or another modeled sink;
- `GoodsSale`: seller inventory decreases and buyer inventory increases by the stated quantity.

Physical quantities never enter a monetary equity identity. They are reconciled through `SectorPhysicalStock` and `PhysicalStockPosting`.

Inventory carrying value is now a separate monetary stock. Two explicit accounting transitions expose the valuation boundary:

- `InventoryCostAddition`: add a cost amount to physically-held inventory;
- `InventoryCostRelief`: remove a cost amount from inventory, serving as the COGS posting in a sale sequence.

A sale therefore has an explicit three-part accounting pattern when valuation is available:

`InventoryCostRelief -> GoodsSale -> InventoryCostAddition`

The first side recognizes seller COGS by relieving carrying value; `GoodsSale` records the physical quantity and deposit consideration; the buyer-side addition records acquired inventory at its explicit carrying amount. `InventoryCostAddition` must not be treated as free value creation: it records an externally determined cost allocation/reclassification and any underlying financing, wage, or input-cost transition remains explicit. No FIFO, weighted-average, specific-identification, unit-price, or cost allocation rule is inferred by the transition engine. Those policy/calculation results must be supplied explicitly.

The period ledger now derives `sales_consideration`, `cost_of_goods_sold`, and `gross_operating_surplus = sales_consideration - cost_of_goods_sold`. This is deliberately a **gross trading surplus** measure until wages, intermediate inputs, depreciation, interest, and taxes have their own explicit expense/accrual boundaries.


## Explicit goods-sale bridge

The substrate now has a `GoodsSale` transition that couples two explicit domains without collapsing their units:

- physical side: seller inventory decreases and buyer inventory increases by `quantity`;
- monetary side: buyer deposits decrease and seller deposits increase by `consideration`;
- carrying value: unchanged by the sale itself; COGS and buyer inventory cost are separate explicit transitions.

The transition therefore never multiplies quantity by an implicit price and never assumes consideration equals carrying value. The sector transaction matrix still classifies sale consideration as `Other`, because a sale can represent final consumption, intermediate demand, investment goods, or external demand.

This gives a deterministic accounting chain:

`transition log -> physical stock delta + monetary balance-sheet delta -> period ledger`

and, when inventory costing is present:

`inventory cost relief -> goods sale -> inventory cost addition -> gross operating surplus`

The resulting structure aligns with conventional inventory accounting's distinction between physical inventory, carrying cost, and cost recognized as an expense, while remaining compatible with the SFC requirement that stocks and flows reconcile.
