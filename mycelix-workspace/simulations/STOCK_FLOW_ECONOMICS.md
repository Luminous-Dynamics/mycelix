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
- `EconomicState`
- aggregate asset/liability accounting
- gross leverage observable
- net credit impulse observable
- accounting-invariant tests

Credit creation is represented as a matching increase in the lender's financial claim,
the borrower's spendable monetary asset, and the borrower's liability. Debt repayment retires
the corresponding asset and liability.

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

## Next research increments

### 1. Sector balance-sheet matrix

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

That makes economic simulations compatible with Mycelix's existing evidence/attestation work.

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

Godley/Lavoie SFC work explicitly links financial and real sides of an economy through
coherent stocks and flows. Recent ecological SFC research couples aggregate demand,
distribution, banking, green-energy investment, and physical resource constraints.

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
