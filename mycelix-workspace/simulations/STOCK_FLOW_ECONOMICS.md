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
- explicit `Depreciation`
- explicit cash vs deposit instruments
- bank loan/deposit double-entry credit creation
- deterministic `EconomicTransition` timestep execution
- `EconomicStepReceipt` state/transition hashes
- `EconomicChainReceipt` multi-period evidence chaining
- `EconomicEvidenceManifest` configuration identity
- `EconomicEvidenceCapsule` terminal evidence sealing
- `EconomicSimulationStep` / `EconomicSimulationTrace` multi-period execution
- `EconomicState`
- aggregate asset/liability accounting
- gross leverage observable
- net credit impulse observable
- EconomicObservables derived financial/operating measurements
- descriptive Minsky financing-regime classifier
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

The derived EconomicObservables layer now reports:

- aggregate debt and loan stocks;
- deposits, cash, deposit liabilities, and liquidity;
- exact rational leverage and liquidity/debt-service ratios;
- credit created, debt repaid, and net credit impulse;
- interest paid and gross debt service;
- sales consideration, COGS, gross operating surplus, and depreciation.

Ratios are represented as integer numerator/denominator pairs rather than floating-point scores. A zero denominator is represented as an undefined ratio.

These are observations, not scores.

### 3. Minsky-style regime observables

classify_financing_regime now provides a descriptive Hedge / Speculative / Ponzi classification when the caller supplies an explicit cash-flow-available-for-debt-service amount and contractual interest/principal due. The function does not infer cash flow from revenue, accounting surplus, or liquidity.

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

1. Add explicit production-cost accumulation so wages, intermediate inputs, and other production costs can feed inventory carrying value without hidden financing. The current layer can already represent the accounting pattern as separate wage/payment and inventory-cost transitions.
2. Add explicit depreciation and capital-consumption postings for productive capital. (The monetary depreciation boundary is now implemented.)
3. Add typed physical units and material-balance/conservation rules for the ecological SFC layer.
4. Add institutional/financial-regime observables (leverage, debt service, liquidity, refinancing need) on top of reconciled stocks. The current layer provides aggregate measurements and an externally-fed financing-regime classifier.
5. Add Minsky/Keen behavioral equations only after the accounting state is observable and reconciled, with clear separation between operating surplus and financing flows.


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


## Deterministic multi-period trace (implemented)

EconomicSimulationTrace runs an ordered list of EconomicSimulationStep values against a private working state.

For every period it:

- applies the complete transition list atomically;
- creates a normal EconomicStepReceipt;
- links that receipt to the previous chain node;
- carries the original genesis state hash through the chain.

The final trace also binds the canonical initial state hash, final state hash, and ordered receipt list into a trace hash. State hashing is canonical over actor ordering and rejects duplicate actor IDs. A failed later period returns an error without exposing a partially committed trace.

This gives scenario runners a direct execution primitive:

`initial state -> period transitions -> chained receipts -> final state`

and lets the evidence capsule seal the resulting trace against model and parameter identity.

## Reproducible evidence capsule (implemented)

The evidence layer binds five independent configuration identities:

- model version;
- parameter-set hash;
- random seed;
- initial-state hash;
- terminal receipt-chain identity.

EconomicEvidenceManifest is metadata, not economic state. EconomicEvidenceCapsule seals that manifest against the final multi-period chain node and terminal observations. The capsule can bind both aggregate EconomicObservables and the deterministic actor-level ActorEconomicObservables map; the latter is stored as a separate observation hash.

The complete audit path is therefore:

`model version + parameter hash + seed + initial state -> chained transitions -> terminal observations`

Changing any configuration identity, state trajectory, transition history, aggregate observations, or supplied actor observations produces a different evidence hash. This gives scenario runners a compact reproducibility anchor that can later be stored alongside Mycelix provenance/attestation records.

## Multi-period evidence chain (implemented)

A single deterministic timestep is now linkable into an append-only evidence history through EconomicChainReceipt.

Each chain node binds:

- the genesis state hash;
- the predecessor receipt hash;
- the complete step receipt;
- the resulting chain hash.

The API links a new node from the previous chain node rather than accepting a bare predecessor string, so the genesis identity propagates automatically. Adjacent receipts must satisfy `next.pre_state_hash == previous.post_state_hash`; disconnected histories are rejected. Therefore changing transition order, pre/post state, period, genesis, or any predecessor changes the downstream chain. This provides a lightweight audit primitive for long economic simulations without putting evidence bookkeeping into the economic state itself.

The intended evidence path is:

`model/parameters -> initial state hash -> step receipt -> chained receipt -> observations`

Later scenario runners can persist the chain alongside parameter manifests and observation provenance so an output can be traced to the exact sequence of state transitions that produced it.

## Actor-level liquidity and financing observations (implemented)

ActorEconomicObservables derives deterministic per-actor measurements from a reconciled terminal state and the ordered transition log.

The projection records:

- current liquidity, debt, loan claims, inventory carrying value, productive capital, net financial position, and net worth;
- credit received/originated and debt actually repaid;
- wages, interest, taxes, transfers, consumption, investment, sales, purchases, COGS, and depreciation by actor;
- net liquidity change across the transition sequence;
- explicitly classified financing and investment liquidity changes;
- a residual non-financing liquidity change.

The residual is intentionally not called operating cash flow. It can still contain monetary transfers, operating receipts/payments, taxes, interest, or other flows that have not been assigned a formal cash-flow statement treatment.

Financing-regime classification requires explicit contractual interest due and principal due. Actual principal repaid remains a separate observation and is never substituted for principal due.

## Derived financial and financing observables (implemented)

EconomicObservables is a pure projection of reconciled state plus the deterministic period ledger. It adds measurements without modifying the accounting state.

The layer deliberately keeps two concepts separate:

- accounting surplus is derived from recognized sales/COGS/depreciation;
- debt-service financing regime requires an explicit cash-flow input.

This prevents an accounting profit measure from being silently treated as cash available for repayment. It also makes the later Keen/Minsky behavioral layer able to consume the same auditable observations without embedding a policy rule inside the accounting substrate.

## Explicit depreciation boundary (implemented)

`Depreciation` is now an explicit monetary-valued productive-capital transition:

- productive capital: `-D`;
- equity residual: `+D` in the signed balance-sheet representation.

It changes no deposits and creates no hidden cash flow. The period ledger records depreciation separately and derives `operating_surplus_after_depreciation = gross_operating_surplus - depreciation`.

This distinction matters because depreciation can also enter production overhead/cost of conversion and subsequently flow into inventory carrying value and COGS. The substrate therefore does not automatically subtract the same depreciation twice. A caller must represent capitalization versus period expense explicitly.

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

The period ledger now derives `sales_consideration`, `cost_of_goods_sold`, `gross_operating_surplus = sales_consideration - cost_of_goods_sold`, and explicit `depreciation`. It also exposes `operating_surplus_after_depreciation` as a transparent derived quantity. This is deliberately not a complete profit measure: wages, intermediate inputs, interest, taxes, financing flows, and any depreciation already capitalized into inventory remain separate boundaries.


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


### Trade credit and working-capital boundary (implemented)

The accounting substrate now models deferred commercial settlement explicitly instead of
treating every goods sale as an immediate deposit transfer.

TradeCreditSale:

- moves physical inventory from seller to buyer;
- records seller trade receivables;
- records buyer trade payables;
- does not create deposits and does not infer inventory carrying value.

TradeCreditSettlement:

- requires matching outstanding receivable/payable;
- transfers deposits from buyer to seller;
- retires both the receivable and payable;
- increases the period's monetary settlement volume.

This is important for SFC dynamics because the timing of revenue recognition, liquidity,
and debt service need not coincide. A firm can therefore have positive sales with rising
receivables and no corresponding immediate deposit inflow. The actor-level observables
expose both trade-credit extension/settlement and net_working_capital_change.

Net operating working capital is defined here as:

inventory carrying value + trade receivables - trade payables

Cash, deposits, physical inventory units, and productive capital are excluded from this
specific working-capital measure. Physical inventory remains in the separate physical-stock
dimension.

The sector balance sheet now contains TradeReceivables and TradePayables, and the
stock-flow reconciliation layer verifies their deltas exactly. Trade-credit settlement is
a monetary transaction; the original deferred sale is a non-cash financial/physical
transaction and is therefore kept out of the cash-sector transaction projection rather
than falsely reported as deposit flow.

This gives the simulation a clean next boundary for cash-flow analysis:

accrual sales/COGS -> working-capital change -> realized liquidity -> debt-service coverage


The accounting closure now also performs an aggregate projection check: aggregate stocks and measured period flows must equal the checked sums of actor observations and the corresponding sector observations. This makes actor, sector, and aggregate reporting three mutually reconciling views over the same authoritative transition sequence rather than independently hash-bound payloads.

Actor-level cash-flow decomposition is now explicit: operating liquidity change is the
period's actor liquidity change after separately classified investing, financing, and
other monetary-transfer changes. The decomposition is checked against the observed
liquidity change and therefore cannot silently lose or double-count a cash movement.

This is a model-defined liquidity-flow bridge, not a claim of IFRS presentation compliance.
The IAS 7 distinction between operating, investing, and financing cash flows is useful
as a reference boundary, while entity-specific classification rules remain outside this
simulation substrate.

The working-capital layer is intentionally accounting-only: no behavioral credit-growth, collection-delay, default, or inventory-demand equation is implied by the new stocks and transitions.


## Sector liquidity projection and evidence binding (implemented)

The actor-level liquidity decomposition now has a deterministic sector projection:
`SectorEconomicObservables::from_actor_observations` consolidates exactly-assigned actors into
canonical sector aggregates without replaying transitions a second time.

Each sector observation carries:

- opening and closing liquidity;
- net liquidity change;
- operating, investing, financing, and other liquidity components;
- net working-capital change;
- credit received/originated and debt repaid;
- trade-credit received/extended/settled/collected;
- sales, goods purchases, COGS, and depreciation.

Two independent stock-flow identities are checked for every resulting sector:

`opening_liquidity + net_liquidity_change = closing_liquidity`

and

`operating + investing + financing + other = net_liquidity_change`.

The sector layer is therefore a deterministic reporting projection over the actor observation
layer, not a second source of economic truth. Sector assignments must cover every actor exactly
once, and duplicate or unknown assignments are rejected.

Sector-aware evidence sealing is also available through
`EconomicEvidenceCapsule::seal_with_actor_and_sector_observations`. When supplied, the
canonical sector observation map receives its own hash and is included in the evidence binding.
Actor-only sealing retains its prior evidence-hash binding, so adding the new optional projection
does not silently rewrite previously defined actor-only evidence semantics.

This is deliberately a **model-defined liquidity-flow bridge**, not a claim of IAS 7 presentation
compliance. IAS 7 classifies cash flows as operating, investing, and financing and defines cash
and cash equivalents separately; the Mycelix research substrate currently reports cash plus
deposits as a broader liquidity measure and keeps its classification policy explicit rather than
pretending to reproduce financial-reporting rules. 


## Financial-claim sector matrix (implemented)

Deferred trade settlement is now represented in a separate
`SectorFinancialFlowMatrix`. The existing `SectorTransactionMatrix` remains the
cash/monetary projection; the new matrix represents changes in financial claims and
obligations.

Supported financial-claim transitions are:

- `CreditCreation` -> `LoanCreation`
- `DebtRepayment` -> `DebtRepayment`
- `TradeCreditSale` -> `TradeCreditExtension`
- `TradeCreditSettlement` -> `TradeCreditSettlement`

Trade-credit extension is oriented buyer -> seller: the buyer acquires a payable and the
seller acquires the matching receivable. This is deliberately **not** emitted as a cash
receipt/payment. Settlement is separately represented as extinction of that financial
relationship while the actor-level liquidity observations capture the accompanying deposit
movement.

This gives the sector accounting stack two explicit projections with different dimensional
meaning:

`transition log -> cash/liquidity transaction matrix`

and

`transition log -> financial-claim matrix`.

Neither projection is authoritative over the transition log. Both are deterministic views over
the same ordered transitions, so deferred settlement can be analyzed without contaminating cash
flows while still remaining visible in sector-level SFC accounting.


## Provenance and numerical hardening (implemented)

Actor observations now preserve the true period-opening liquidity stock explicitly rather than
reconstructing it from terminal liquidity minus the observed change. The actor-level identity is:

`opening_liquidity + net_liquidity_change = closing liquidity`.

Sector consolidation consumes that explicit opening stock and verifies the actor identity before
aggregating, providing a clearer provenance chain from the initial balance sheet into sector
observations.

The sector cash transaction matrix and the separate financial-claim matrix now expose checked
aggregation helpers. Their clearing predicates fail closed if extreme `i128` arithmetic would
overflow. Sector balance-sheet and physical-stock total helpers likewise expose checked variants,
and financial-row / balance-sheet identity validation no longer relies on wrapping arithmetic.

The evidence capsule schema marks newly added sector hashes optional for deserialization, so older
capsules without those fields remain readable. The new full-seal API additionally binds the
sector financial-claim projection into the evidence hash while leaving the historical aggregate,
actor-only, and actor+sector binding paths unchanged.

This matters for reproducibility: evidence should distinguish a genuinely different projection
from an arithmetic artifact, and schema evolution should not silently turn an older receipt into
an unreadable artifact.

The accounting boundary remains intentionally layered:

`initial actor stocks`
-> `authoritative transitions`
-> `actor observations`
-> `sector observations`
-> `cash transaction projection + financial-claim projection`
-> `sector stock/reconciliation`
-> `evidence hashes`.

No model behavior is inferred from the reporting layers. In particular, trade-credit extension and
settlement do not imply collection delays, default probabilities, maturity behavior, credit
demand, or policy responses until those are introduced as separate explicit transition rules.

## Research boundary: cash-flow presentation vs SFC accounting

IAS 7 defines a financial-reporting cash-flow statement around cash and cash equivalents and
classifies cash flows as operating, investing, and financing. Its indirect operating method also
reconciles profit/loss with non-cash items and changes in operating inventories, receivables, and
payables. The Mycelix substrate intentionally does not claim to reproduce those presentation rules:
its actor/sector liquidity bridge is a simulation accounting observable over the model's explicit
cash and deposit instruments.

The SFC purpose is different and complementary. The balance-sheet matrix, transactions-flow matrix,
and explicit financial commitments provide the accounting skeleton on which later behavioral
equations can operate. The separate financial-claim projection keeps deferred settlement visible
as a stock/claim event instead of forcing every economic transaction into a cash-flow category.


## Sector stock snapshot closure (implemented)

`SectorEconomicObservables` now carries the sector-level closing stock mirror needed
to keep behavioral equations from consuming a flow-only projection:

- loan claims;
- debt;
- trade receivables and trade payables;
- inventory carrying value;
- productive capital;
- net financial position;
- net worth;
- closing monetary net working capital.

`SectorEconomicObservables::from_state_and_transitions` now derives actor observations,
consolidates them, constructs the sector balance sheet, and validates every sector stock snapshot
against that balance-sheet projection before returning it.

The resulting closure is:

`state + transitions -> actor observations -> sector observations <-> sector balance sheet`

For working capital the closing stock is explicitly:

`inventory carrying value + trade receivables - trade payables`

while `net_working_capital_change` remains the period delta. Keeping both the stock and
the change available avoids forcing later behavioral equations to reconstruct one from unrelated
fields.

This follows the SFC principle that opening stocks interact with period transactions to generate
closing stocks, while sector financial positions remain subject to exact accounting constraints.
The transaction-flow and balance-sheet structures therefore stay coupled, but neither becomes a
behavioral assumption by itself. 


## Trade-credit settlement cross-matrix closure (implemented)

Trade-credit timing is now represented orthogonally across the two sector-flow projections:

- `TradeCreditSale` is omitted from the monetary transaction matrix because it creates a
  receivable/payable without moving cash or deposits, but it appears in the financial-claim
  matrix as `TradeCreditExtension`.
- `TradeCreditSettlement` appears in the monetary transaction matrix as an explicit
  `TradeCreditSettlement` flow from buyer sector to seller sector because deposits actually
  move.
- The same `TradeCreditSettlement` transition appears in the financial-claim matrix because
  the receivable/payable pair is simultaneously extinguished.

Therefore one deferred sale followed by settlement is not forced into a single overloaded
transaction category. The economic event has a non-cash claim creation phase and a later
monetary settlement phase, both tied to the same authoritative transition sequence.

This is especially useful for later working-capital and debt-service dynamics: a model can
distinguish recognized sales, outstanding operating claims, and actual liquidity settlement
without inventing a collection-delay function at the accounting layer.


## Closed-state and aggregate arithmetic hardening (implemented)

The root `EconomicState` now exposes checked aggregate asset, liability, claim, and net-financial-
position sums. The legacy convenience methods remain available, but the checked variants are used
by the checked aggregate-observable constructor.

A closed-model invariant is also available through `closed_financial_rows_clear`. It verifies,
with fail-closed arithmetic, that modeled deposits equal issued deposit liabilities, loan claims
equal debt, and trade receivables equal trade payables. This is intentionally a closed-system
assertion: a model that includes external counterparties should represent those counterparties
explicitly rather than silently accepting an unpaired claim.

The distinction matters for SFC calibration. A model can deliberately choose an open boundary, but
the boundary should be explicit; otherwise a debt or deposit shock can appear to create or destroy
net financial assets merely because its counterpart was omitted.

The aggregate observable layer now has a checked derivation path as well. This makes the numerical
integrity chain:

`actor fields -> checked state aggregates -> aggregate observations`

instead of allowing an extreme-value fixture to wrap before it reaches evidence/reporting layers.

## Cross-layer accounting closure receipt (implemented)

`EconomicAccountingClosure::validate_and_seal` is the single-step closure proof that binds the
current accounting projections back to one authoritative transition sequence and its terminal
state. It verifies all of the following together:

- actor observations reproduce the post-state actor stocks exactly;
- the sector transaction matrix is an exact transition projection and clears;
- the sector financial-claim matrix is an exact transition projection and reconciles loan,
  debt, trade-receivable, and trade-payable deltas;
- the monetary and physical stock-flow postings reconcile exactly against the post-state;
- sector observations reproduce the terminal sector balance-sheet projection.

The receipt then binds the pre-state hash, post-state hash, transition hash/count, actor-observation
hash, posting hashes/counts, sector transaction hash, sector financial-flow hash, and sector
observation hash into one deterministic `closure_hash`.

This is deliberately an accounting/provenance boundary rather than a behavioral model. It makes the
next layer safer: working-capital equations, leverage dynamics, debt-service rules, Minsky regime
classification, credit feedback, and ecological constraints can consume one closure-verified step
instead of independently trusting several projections.

`EconomicEvidenceCapsule::seal_with_accounting_closure` additionally self-verifies the closure and
requires its pre-state, post-state, transition hash, and transition count to match the final
evidence-chain step before binding the closure hash into the capsule.
## Source-revision provenance (implemented)

`EconomicEvidenceManifest` now supports an optional exact source revision, such as the Git commit
SHA used to generate the evidence. The field is omitted from serialization when unset, preserving
legacy manifest bytes and therefore legacy hashes; when supplied, it becomes part of the manifest
hash and is consequently bound into the final evidence capsule.

This separates three reproducibility identities explicitly:

`source revision + parameter hash + seed`

with the initial-state and transition/evidence hashes providing the run-specific execution identity.
## Explicit financial-commitment diagnostic (implemented)

`FinancialCommitmentObservation` provides a descriptive Minsky-style coverage object over three
explicit inputs:

`cash-flow available, interest due, principal due`

It derives total contractual service, service shortfall, whether total service is covered, whether
interest is covered, and the existing hedge/speculative/Ponzi classification.

The diagnostic does **not** infer cash flow from revenue, profit, liquidity, or actual repayment. It
also does not create debt or automatically trigger refinancing/default. This keeps the accounting
substrate authoritative while making financial fragility a reproducible derived observation.

This matches the core Minsky formulation in which financing posture is defined by the relationship
between prospective cash flows and payment commitments on liabilities.

## Working-capital component deltas (implemented)

Actor and sector observables now expose the monetary components of period working-capital change
explicitly, rather than requiring later equations to reverse-engineer them from the aggregate:

- inventory carrying-value change;
- trade-receivables change;
- trade-payables change.

The exact identity is:

`ΔNWC = Δinventory_carrying_value + Δtrade_receivables - Δtrade_payables`.

These deltas are derived from the same opening state and authoritative transition replay that
produce the closing stock snapshot, then checked with overflow-safe arithmetic. Sector values are
deterministic sums of the actor-level components.

This is still an accounting representation, not a behavioral assumption: it does not choose
inventory targets, payment terms, collection lags, supplier finance, default rates, or credit
demand. It simply exposes the state changes that a later working-capital behavioral layer can
consume without reconstructing hidden intermediate quantities.


## Aggregate-observation provenance closure (implemented)

The cross-layer accounting closure now also binds the aggregate `EconomicObservables` projection derived from the exact post-state and deterministic period ledger.

This prevents a subtle mix-and-match evidence failure: a valid actor/sector/stock-flow closure could previously coexist with an independently supplied aggregate observation payload. The closure now stores `aggregate_observations_hash`, and `seal_with_accounting_closure` requires the supplied aggregate observation hash to match it before the evidence capsule can be sealed.

The resulting provenance chain is:

`pre-state + ordered transitions -> post-state + ledger -> aggregate observations`

and that aggregate observation hash is bound alongside the actor, sector, financial-claim, physical-posting, and sector-transaction projections.

This is consistent with the SFC emphasis on integrating opening stocks, period flows, and closing stocks rather than treating reporting aggregates as an independent source of truth. It also keeps the behavioral boundary intact: no credit-demand, collection, refinancing, or policy rule is introduced by this change.


## Input-domain and mutation hardening (implemented)

The accounting boundary now treats deserialization as an explicit trust boundary rather than assuming every public transition was constructed through its convenience constructor.

EconomicTransition::validate is the canonical domain check for transition amounts and is consumed by timestep execution, the period ledger, and both sector-flow projections. The underlying EconomicState mutators retain their own checks as defense in depth.

EconomicState::validate now enforces:

- unique, non-empty actor identifiers;
- non-negative monetary and physical stock quantities;
- non-negative period counters;
- aggregate modeled financial claims equal aggregate modeled liabilities.

apply_step validates both the incoming state and resulting state, while core mutators preflight checked arithmetic before changing either side of a transaction. This prevents malformed starting ledgers, malformed deserialized transitions, and overflow paths from being converted into partially mutated economic states.

The reconciliation layer also validates both terminal states before certifying a stock-flow receipt. A reconciliation therefore cannot become an evidence artifact merely because a malformed state happens to be unchanged across an empty transition set.

The resulting trust boundary is:

deserialized input -> domain validation -> atomic transition -> state validation -> stock/flow reconciliation -> evidence binding

This is aligned with the wider accounting objective of making stock changes, cash flows, and non-cash changes explicitly traceable rather than relying on implicit balancing adjustments.

## Projection-boundary and trace-sequence hardening (implemented)

Sector consolidation now treats actor-to-sector assignments as a closed input domain. Every
assignment record must refer to a known actor, and every state actor must have exactly one
assignment. Unknown records are rejected rather than silently ignored. The same validation is
shared by monetary balance-sheet and physical-stock projections.

This closes an evidence seam in which a caller could provide a valid assignment for every modeled
actor plus an extra unknown assignment and receive a projection that quietly discarded the extra
record. The projection boundary is now explicit:

`state actors <-> sector assignments -> deterministic sector projection`

The simulation-trace runner also validates its initial economic state before producing even an
empty trace. Period identifiers in a multi-period trace are required to be strictly increasing,
so a receipt chain cannot describe a temporally contradictory sequence while remaining
hash-consistent.

These are structural invariants, not economic behavior. They strengthen reproducibility by ensuring
that the same validated domain is consumed whether a trace contains many periods, one period, or
none. This complements the SFC principle that accounting structures should remain internally
consistent across opening stocks, transactions, and closing stocks, and the broader accounting
practice of keeping changes in assets and liabilities traceable rather than silently absorbed into
unexplained reconciliation differences.

## Replay-verified evidence sealing (implemented)

A stronger evidence path, EconomicEvidenceCapsule::seal_verified_step, now exists for callers that
have the authoritative initial state, terminal state, ordered transitions, terminal observations,
sector assignments, and final chain receipt.

Before sealing, it:

- verifies the supplied receipt's internal chain hash;
- validates the initial state;
- replays the exact transition sequence through apply_step;
- requires the replayed terminal state and step receipt to match the supplied artifacts;
- checks the supplied terminal state hash against the receipt;
- rebuilds the cross-layer accounting closure; and
- requires the supplied aggregate observations to match that closure.

This deliberately sits beside the legacy evidence APIs rather than silently changing their historical
semantics. The older sealers remain useful for compatibility and hash binding; the replay-verified
path is the stronger provenance boundary when the authoritative execution inputs are available.

The resulting distinction is explicit:

hash-consistent receipt
-> replay-verified state transition
-> accounting-closure-verified evidence

This is particularly useful for reproducibility because an independently constructed observation
payload can no longer be mistaken for an observation actually produced by the supplied transition
program when the stronger sealing path is used.

The design also follows the current accounting direction of making asset/liability changes and
non-cash changes traceable to the underlying statement information rather than hidden inside an
unexplained aggregate reconciliation.

## Full-trace replay provenance (implemented)

The stronger evidence boundary now also supports whole-trace sealing through
`EconomicEvidenceCapsule::seal_verified_trace`.

For a multi-period run, this path validates the initial state and manifest genesis hash, verifies
the supplied trace, then replays every period from the initial state. The replayed trace must be
byte-equivalent in its structured receipt representation to the supplied trace, and the replayed
terminal state must equal the supplied terminal state.

The final accounting closure is then evaluated against the **actual opening state of the final
period**, obtained by replaying the prefix of the trace. This is important: using the simulation
genesis state as the accounting pre-state for the final period would incorrectly collapse
multi-period provenance into a single-step proof.

This closes two distinct tampering classes:

1. a receipt can be internally rehashed while describing a transition that was never produced by
   the authoritative transition program;
2. a final-period accounting closure can be valid while an earlier receipt in the chain has been
   altered.

The resulting strongest evidence path is:

initial state
-> replayed ordered transition program
-> complete verified receipt chain
-> final-period replayed pre-state
-> cross-layer accounting closure
-> terminal evidence capsule

This is consistent with the SFC accounting discipline in which sectoral transactions and stocks are
kept jointly coherent, and with current IASB work seeking clearer traceability between statement
of financial position items, cash-flow reconciliations, and specified non-cash changes.

## Chain-constructor hardening (implemented)

Receipt construction now verifies an existing predecessor before extending the evidence chain.
A successor therefore cannot be built on top of a receipt whose own chain hash or required identity
fields are already inconsistent.

Newly constructed chain nodes also reject empty pre-state, transition, or post-state hashes.
Trace verification separately requires non-empty trace identity hashes before checking the receipt
sequence and trace hash.

This establishes a useful invariant for transported evidence:

`deserialize -> structural identity checks -> predecessor verification -> chain linkage -> trace verification`

A recomputed hash is therefore not sufficient to make malformed evidence acceptable at the chain
boundary; the structure must also satisfy the same identity rules used by normal construction.

## Static compile seam closure (implemented)

The hardening pass also surfaced and corrected a syntactic defect in the sector transaction
projection: the GoodsSale match arm lacked its separating comma before the next transition arm.

This was found during boundary inspection rather than assumed away because the remote CI queue had
not yet produced a completed compiler result. Keeping the correction explicit preserves the evidence
discipline: static inspection findings are distinguished from compiler-verified findings until the
workflow actually completes.

## Observable input-domain hardening (implemented)

Actor-level observation derivation now validates the supplied opening economic state before deriving
any measurements and validates every deserialized transition through the canonical
`EconomicTransition::validate` boundary before replay.

This closes the final direct-reporting side door: an empty transition list can no longer turn a
malformed opening ledger into a seemingly valid observation payload, and a malformed transition
cannot bypass the shared domain validator merely by entering through the reporting layer.

Because sector observables are built from actor observables, the validation propagates upward into
the sector reporting path as well.

The resulting input discipline is now consistent across execution and reporting:

`state validation -> transition validation -> deterministic replay -> observation projection`

This follows the core SFC requirement that stocks and flows be treated as one accounting system
rather than allowing independently constructed reporting data to float free of the underlying
accounts.
