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

Credit creation now follows the minimal private-money balance-sheet structure: the lender records a loan claim and deposit liability while the borrower records the matching deposit asset and debt liability. Debt repayment reverses the loan/deposit entries. Physical cash remains a distinct instrument. This closes an important modeling gap: in a bank-credit model, a deposit is itself a bank liability rather than an unexplained pool of money. Godley/Lavoie-style SFC accounting explicitly represents deposits and loans this way. citeturn3search15turn3search16

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

The sector transaction matrix is intentionally not yet treated as a balance-sheet matrix. Its current `clears()` check only establishes aggregate sector conservation; it cannot by itself prove that sector stocks changed consistently. The next increment is therefore a real sector balance-sheet matrix, with instrument rows (deposits, loans, cash/reserves, equity, etc.) and sector columns, followed by explicit reconciliation against actor-level state.

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

Godley/Lavoie SFC work explicitly links financial and real sides of an economy through coherent stocks and flows; the SFC literature uses balance-sheet and transactions-flow matrices and emphasizes that financial assets must have counterpart liabilities. citeturn2search12turn3search15 Keen's work connects Minskyan debt dynamics with SFC accounting and treats changes in debt as a distinct driver in monetary dynamics. citeturn0search1turn0search2 Recent ecological SFC work can then be layered on top of this monetary substrate rather than replacing it.

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
